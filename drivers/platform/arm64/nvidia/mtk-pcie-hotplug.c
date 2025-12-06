// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2014-2025 MediaTek Inc.
/*
 * Generic PCIe hotplug driver with GPIO interrupt support
 * Originally designed for MediaTek platforms with PCIe hotplug capabilities
 *
 * This driver manages PCIe device hotplug using GPIO interrupts and platform
 * resources. It supports cable insertion/removal detection and device power
 * management.
 *
 * MMIO Resource Requirements (via platform_device resources):
 *   Resource 0: TOP region    - PCIe top-level control registers
 *   Resource 1: PROTECT region - Bus protection control registers  
 *   Resource 2: CKM region     - Clock management registers
 *   Resource 3+: MAC regions   - Per-port MAC control registers
 *
 * Example ACPI resource definition:
 *   Name (_CRS, ResourceTemplate() {
 *       Memory32Fixed(ReadWrite, 0x1d600000, 0x1000)  // TOP
 *       Memory32Fixed(ReadWrite, 0x1d640000, 0x1000)  // PROTECT
 *       Memory32Fixed(ReadWrite, 0x16bd0000, 0x1000)  // CKM
 *       Memory32Fixed(ReadWrite, 0x1d790000, 0x1000)  // MAC port 0
 *       Memory32Fixed(ReadWrite, 0x1d690000, 0x1000)  // MAC port 1
 *   })
 */

#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/property.h>

#define HP_PORT_MAX		8
#define HP_POLL_CNT_MAX		200
#define PCIE_REG_SIZE		0x1000
#define MAX_VENDOR_DATA_LEN	16

/* Hardware timing requirements (in microseconds unless noted) */
#define PCIE_HP_DELAY_SHORT_US		10	/* Short delay for register writes */
#define PCIE_HP_DELAY_STANDARD_US	10000	/* Standard delay (10ms) */
#define PCIE_HP_DELAY_BUS_PROTECT_US	5000	/* Bus protection setup delay */
#define PCIE_HP_DELAY_PHY_RESET_US	3000	/* PHY reset delay */
#define PCIE_HP_DELAY_LINK_STABLE_MS	100	/* Link stabilization delay (ms) */
#define PCIE_HP_POLL_SLEEP_US		10000	/* Polling loop sleep interval */

#define PLUG_IN_EVT "HOTPLUG_STATE=plugin"
#define REMOVAL_EVT "HOTPLUG_STATE=removal"

/*
 * Use bus protect to prevent pcie core_reset glitch issue.
 * stage 0: init bus protection during probe
 * stage 1: disable pcie ltssm
 * stage 2: set bus protection and enable pcie ltssm
 */
#define BUS_PROTECT_INIT		0
#define BUS_PROTECT_CABLE_REMOVAL	1
#define BUS_PROTECT_CABLE_PLUGIN	2
 
enum pcie_hp_state {
    STATE_READY = 0,
    STATE_PLUG_OUT,		/* Cable plug-out */
    STATE_DEV_POWER_OFF,	/* Device is powered off */
    STATE_PLUG_IN,		/* Cable plug-in detected */
    STATE_DEV_POWER_ON,		/* Device is powered on */
    STATE_DEV_FW_START,		/* Device firmware is running */
    STATE_RESCAN,		/* Device ready, can perform bus rescan */
    STATE_UNKNOWN
};
 
 enum pcie_pin_index {
     PCIE_PIN_BOOT = 0,	/* Device boot status pin */
     PCIE_PIN_PRSNT,		/* Presence detection pin */
     PCIE_PIN_PERST,		/* PCIe reset pin */
     PCIE_PIN_EN,		/* Power enable pin */
     PCIE_PIN_CLQ0,		/* Clock request pin 0 */
     PCIE_PIN_CLQ1,		/* Clock request pin 1 */
     PCIE_PIN_MAX
 };
 
/* PCIe port information */
struct pcie_port_info {
    int domain;
    int bus;
    int devfn;
};

struct rp_bus_mmio_top {
    u32 ctrl;
    u32 port_bits[HP_PORT_MAX];
    u32 update_bit;
};

struct rp_bus_mmio_protect {
    u32 mode;
    u32 enable;
    u32 port_bits[HP_PORT_MAX];
};

struct rp_bus_mmio_mac {
    u32 init_ctrl;
    u32 ltssm_bit;
    u32 phy_rst_bit;
};

struct rp_bus_mmio_ckm {
    u32 ctrl;
    u32 disable_bit;
};

struct rp_bus_mmio_info {
    struct rp_bus_mmio_top top;
    struct rp_bus_mmio_protect protect;
    struct rp_bus_mmio_mac mac;
    struct rp_bus_mmio_ckm ckm;
};
 
struct gpio_acpi_context {
    struct device *dev;
    unsigned int debounce_timeout_us; /* in microseconds */
    int pin;
    int wake_capable;
    int triggering;
    int polarity;
    unsigned long irq_flags;
    int valid;
    unsigned int connection_type;
    char vendor_data[MAX_VENDOR_DATA_LEN + 1];
};

/* Forward declaration */
struct pcie_hp_dev;

struct pcie_hp_plat_data {
    int port_nums;
    struct pcie_port_info ports[HP_PORT_MAX];
    u32 vendor_id;
    u32 device_id;
    int num_devices;
    /* Platform-specific MMIO configuration */
    struct rp_bus_mmio_info rp_bus_mmio;
    void (*rp_bus_protect)(struct pcie_hp_dev *dev, int port_idx, int stage);
    u32 ltssm_reg;
    u32 ltssm_l0_state;
    /* Pinctrl configuration */
    int pin_nums;
    struct pinctrl_map pinmap[];
};

struct pcie_hp_gpio_ctx {
    struct gpio_desc *desc;
    struct gpio_acpi_context *ctx;
    struct pcie_hp_dev *hp_dev;
};

struct acpi_gpio_parse_context {
    struct gpio_acpi_context *ctx;
    struct pcie_hp_dev *hp_dev;
};

enum pcie_hp_debug_val {
    PCIE_HP_DEBUG_PLUG_OUT = 0,
    PCIE_HP_DEBUG_PLUG_IN,
    PCIE_HP_DEBUG_MAX_VAL
};

/**
 * struct pcie_hp_mmio_runtime - Runtime MMIO base addresses
 * @top_base: Mapped TOP region base
 * @protect_base: Mapped PROTECT region base
 * @ckm_base: Mapped CKM region base
 * @mac_port_base: Mapped MAC port base addresses (per port)
 *
 * These are the dynamically mapped MMIO base addresses, separate from the
 * const platform data which only contains offsets and bit masks.
 */
struct pcie_hp_mmio_runtime {
	void __iomem *top_base;
	void __iomem *protect_base;
	void __iomem *ckm_base;
	void __iomem *mac_port_base[HP_PORT_MAX];
};

struct pcie_hp_dev {
    struct pcie_hp_gpio_ctx *pins;
    struct pcie_hp_plat_data *pd;
    struct platform_device *pdev;
    enum pcie_hp_state state;
    int gpio_count;
    int boot_pin;
    int prsnt_pin;
    enum pcie_hp_debug_val debug_state;
    bool suppress_gpio_irq; /* Suppress GPIO IRQs during sysfs operations */
    spinlock_t lock; /* Protect state changes (IRQ-safe) */
    struct pci_dev *cached_root_ports[HP_PORT_MAX]; /* Cached root port pointers */
    struct pcie_hp_mmio_runtime mmio; /* Runtime mapped MMIO base addresses */
};
 
static int pcie_hp_pinctrl_init(struct pcie_hp_dev *hp_dev)
{
    int ret;

    if (!hp_dev->pd->pin_nums)
        return 0;

    ret = pinctrl_register_mappings(hp_dev->pd->pinmap, hp_dev->pd->pin_nums);
    if (ret) {
        dev_err(&hp_dev->pdev->dev, "Failed to register pinctrl mappings\n");
        return ret;
    }

    return 0;
}

static void pcie_hp_pinctrl_remove(struct pcie_hp_dev *hp_dev)
{
    if (hp_dev->pd->pin_nums)
        pinctrl_unregister_mappings(hp_dev->pd->pinmap);
}

static int pcie_hp_change_state(struct pcie_hp_dev *hp_dev, const char *new_state)
{
    struct pinctrl *pinctrl;
    struct pinctrl_state *state;
    int ret;

    pinctrl = devm_pinctrl_get(&hp_dev->pdev->dev);
    if (IS_ERR(pinctrl)) {
        dev_err(&hp_dev->pdev->dev, "Failed to get pinctrl\n");
        return PTR_ERR(pinctrl);
    }

    state = pinctrl_lookup_state(pinctrl, new_state);
    if (IS_ERR(state)) {
        dev_err(&hp_dev->pdev->dev, "Failed to lookup state:%s\n", new_state);
        return PTR_ERR(state);
    }

    ret = pinctrl_select_state(pinctrl, state);
    if (ret) {
        dev_err(&hp_dev->pdev->dev, "Failed to select pinctrl state:%s\n", new_state);
        return ret;
    }

    return 0;
}

static void pcie_hp_send_uevent(struct pcie_hp_dev *hp_dev, const char *msg)
{
    char *uevent = NULL;
    char *envp[2];

    uevent = kasprintf(GFP_KERNEL, msg);
    if (!uevent) {
        dev_err(&hp_dev->pdev->dev, "Failed to allocate uevent string\n");
        return;
    }

    envp[0] = uevent;
    envp[1] = NULL;

    if (kobject_uevent_env(&hp_dev->pdev->dev.kobj, KOBJ_CHANGE, envp))
        dev_err(&hp_dev->pdev->dev, "Failed to send uevent\n");

    kfree(uevent);
}
 
/**
 * pcie_hp_reg_update_bits - Update specific bits in a register
 * @base: MMIO base address
 * @offset: Register offset
 * @mask: Bits to modify
 * @set: true to set bits, false to clear bits
 */
static inline void pcie_hp_reg_update_bits(void __iomem *base, u32 offset,
                                            u32 mask, bool set)
{
    u32 val = readl(base + offset);
    
    if (set)
        val |= mask;
    else
        val &= ~mask;
    
    writel(val, base + offset);
}

/**
 * pcie_hp_toggle_update_bit - Toggle control register update bit
 * @base: MMIO base address
 * @ctrl_offset: Control register offset
 * @bits: Bits to set/clear before toggling update
 * @update_bit: Update bit mask
 * @set: true to set bits, false to clear bits
 *
 * Performs the sequence: modify bits, clear update bit, set update bit
 */
static void pcie_hp_toggle_update_bit(void __iomem *base, u32 ctrl_offset,
                                       u32 bits, u32 update_bit, bool set)
{
    pcie_hp_reg_update_bits(base, ctrl_offset, bits, set);
    pcie_hp_reg_update_bits(base, ctrl_offset, update_bit, false);
    pcie_hp_reg_update_bits(base, ctrl_offset, update_bit, true);
}

/**
 * pcie_hp_bus_protect_enable - Enable bus protection for a port
 * @dev: hotplug device
 * @port_idx: Port index
 */
static void pcie_hp_bus_protect_enable(struct pcie_hp_dev *dev, int port_idx)
{
    struct rp_bus_mmio_info *mmio_info = &dev->pd->rp_bus_mmio;
    u32 port_bit = mmio_info->protect.port_bits[port_idx];
    
    pcie_hp_reg_update_bits(dev->mmio.protect_base,
                             mmio_info->protect.mode, port_bit, true);
    pcie_hp_reg_update_bits(dev->mmio.protect_base,
                             mmio_info->protect.enable, port_bit, true);
}

/**
 * pcie_hp_bus_protect_disable - Disable bus protection for a port
 * @dev: hotplug device
 * @port_idx: Port index
 */
static void pcie_hp_bus_protect_disable(struct pcie_hp_dev *dev, int port_idx)
{
    struct rp_bus_mmio_info *mmio_info = &dev->pd->rp_bus_mmio;
    u32 port_bit = mmio_info->protect.port_bits[port_idx];
    
    pcie_hp_reg_update_bits(dev->mmio.protect_base,
                             mmio_info->protect.enable, port_bit, false);
    pcie_hp_reg_update_bits(dev->mmio.protect_base,
                             mmio_info->protect.mode, port_bit, false);
}

static void pcie_hp_ckm_control(struct pcie_hp_dev *dev, bool disable)
{
    struct rp_bus_mmio_info *mmio_info = &dev->pd->rp_bus_mmio;
    
    if (!dev->mmio.ckm_base)
        return;
    
    pcie_hp_reg_update_bits(dev->mmio.ckm_base, mmio_info->ckm.ctrl,
                             mmio_info->ckm.disable_bit, disable);
}

/**
 * struct pcie_hp_acpi_mmio - Container for parsed ACPI MMIO resources
 * @mmio_regions: array of Memory32Fixed resources from ACPI _CRS
 * @count: number of MMIO regions found
 * @dev: device pointer for logging
 *
 * This structure is used to collect MMIO resources during ACPI _CRS parsing.
 * The callback function populates this as it walks through the ACPI resources.
 */
struct pcie_hp_acpi_mmio {
	struct acpi_resource_fixed_memory32 mmio_regions[5];
	int count;
	struct device *dev;
};

/**
 * pcie_hp_parse_acpi_resources - ACPI resource callback for parsing _CRS
 * @ares: ACPI resource being processed
 * @data: pointer to pcie_hp_acpi_mmio structure
 *
 * This callback is invoked by acpi_walk_resources() for each resource in
 * the device's _CRS. It extracts Memory32Fixed resources and stores them
 * for later mapping.
 *
 * Returns: AE_OK to continue iteration, AE_ERROR on error
 */
static acpi_status pcie_hp_parse_acpi_resources(struct acpi_resource *ares, void *data)
{
	struct pcie_hp_acpi_mmio *parsed = data;

	switch (ares->type) {
	case ACPI_RESOURCE_TYPE_FIXED_MEMORY32:
		if (parsed->count >= 5) {
			dev_warn(parsed->dev, "DEBUG:SCK: More than 5 MMIO regions in _CRS, ignoring extras\n");
			dev_warn(parsed->dev, "More than 5 MMIO regions found, ignoring extras\n");
			break;
		}
		/* Store the Memory32Fixed resource */
		parsed->mmio_regions[parsed->count] = ares->data.fixed_memory32;
		dev_info(parsed->dev, "DEBUG:SCK: ACPI _CRS: Found Memory32Fixed[%d]: addr=0x%08x, len=0x%08x\n",
			parsed->count,
			parsed->mmio_regions[parsed->count].address,
			parsed->mmio_regions[parsed->count].address_length);
		dev_dbg(parsed->dev, "Found MMIO[%d]: addr=0x%08x len=0x%08x\n",
			parsed->count,
			parsed->mmio_regions[parsed->count].address,
			parsed->mmio_regions[parsed->count].address_length);
		parsed->count++;
		break;
	default:
		/* Ignore other resource types (GPIO, IRQ, etc.) */
		break;
	}

	return AE_OK;
}

/**
 * pcie_hp_map_resources - Map all MMIO regions from ACPI _CRS
 * @dev: hotplug device
 *
 * Uses acpi_walk_resources() to parse Memory32Fixed entries in ACPI _CRS,
 * then maps each region using devm_ioremap(). This is the upstream-friendly
 * method for ARM64 ACPI platforms where platform_get_resource() may not work.
 *
 * Expected MMIO order in _CRS:
 *   0: TOP region (PCIe control)
 *   1: PROTECT region (bus protection)
 *   2: CKM region (clock management)
 *   3: MAC Port 0 (per-port MAC)
 *   4: MAC Port 1 (per-port MAC)
 *
 * Returns: 0 on success, negative error code on failure
 */
static int pcie_hp_map_resources(struct pcie_hp_dev *dev)
{
	struct rp_bus_mmio_info *mmio = &dev->pd->rp_bus_mmio;
	struct platform_device *pdev = dev->pdev;
	struct acpi_device *adev;
	struct pcie_hp_acpi_mmio parsed = { .count = 0, .dev = &pdev->dev };
	acpi_status status;
	int i;

	dev_info(&pdev->dev, "DEBUG:SCK: ========================================\n");
	dev_info(&pdev->dev, "DEBUG:SCK: Starting MMIO Resource Mapping\n");
	dev_info(&pdev->dev, "DEBUG:SCK: ========================================\n");
	dev_info(&pdev->dev, "Parsing MMIO regions from ACPI _CRS\n");

	/* Get ACPI companion device */
	dev_info(&pdev->dev, "DEBUG:SCK: Getting ACPI companion device...\n");
	adev = ACPI_COMPANION(&pdev->dev);
	if (!adev) {
		dev_err(&pdev->dev, "DEBUG:SCK: ACPI companion device NOT FOUND\n");
		dev_err(&pdev->dev, "No ACPI companion device found\n");
		return -ENODEV;
	}
	dev_info(&pdev->dev, "DEBUG:SCK: ACPI companion OK - %s\n", acpi_device_hid(adev));

	/* Walk through ACPI _CRS resources to find Memory32Fixed entries */
	dev_info(&pdev->dev, "DEBUG:SCK: Walking ACPI _CRS resources...\n");
	status = acpi_walk_resources(adev->handle, METHOD_NAME__CRS,
				      pcie_hp_parse_acpi_resources, &parsed);
	if (ACPI_FAILURE(status)) {
		dev_err(&pdev->dev, "DEBUG:SCK: acpi_walk_resources() FAILED: %s\n",
			acpi_format_exception(status));
		dev_err(&pdev->dev, "Failed to walk ACPI resources: %s\n",
			acpi_format_exception(status));
		return -ENODEV;
	}
	dev_info(&pdev->dev, "DEBUG:SCK: ACPI walk completed - found %d MMIO regions\n", parsed.count);

	/* Verify we found all required MMIO regions */
	if (parsed.count < 5) {
		dev_err(&pdev->dev, "DEBUG:SCK: INSUFFICIENT MMIO regions - expected 5, found %d\n", parsed.count);
		dev_err(&pdev->dev, "Expected 5 MMIO regions, found %d\n", parsed.count);
		return -ENODEV;
	}

	dev_info(&pdev->dev, "Found %d MMIO regions in _CRS, mapping...\n", parsed.count);

	/* Map each MMIO region using the addresses from ACPI */
	dev_info(&pdev->dev, "DEBUG:SCK: Mapping %d MMIO regions...\n", parsed.count);
	for (i = 0; i < parsed.count; i++) {
		void __iomem *base;
		u32 addr = parsed.mmio_regions[i].address;
		u32 size = parsed.mmio_regions[i].address_length;
		const char *name;

		/* Determine region name based on index */
		switch (i) {
		case 0: name = "TOP"; break;
		case 1: name = "PROTECT"; break;
		case 2: name = "CKM"; break;
		case 3: name = "MAC Port 0"; break;
		case 4: name = "MAC Port 1"; break;
		default: name = "Unknown"; break;
		}

		dev_info(&pdev->dev, "DEBUG:SCK: [MMIO %d/%d] Mapping %s: addr=0x%08x, size=0x%x\n",
			 i+1, parsed.count, name, addr, size);

		/* Map the region (devm handles cleanup automatically) */
		base = devm_ioremap(&pdev->dev, addr, size);
		if (!base) {
			dev_err(&pdev->dev, "DEBUG:SCK: [MMIO %d/%d] %s mapping FAILED\n", i+1, parsed.count, name);
			dev_err(&pdev->dev, "Failed to map %s region (0x%08x)\n", name, addr);
			return -ENOMEM;
		}

		dev_info(&pdev->dev, "Mapped %s: 0x%08x (size 0x%x) -> %p\n",
			 name, addr, size, base);
		dev_info(&pdev->dev, "DEBUG:SCK: [MMIO %d/%d] %s SUCCESS -> virtual addr %p\n",
			 i+1, parsed.count, name, base);

		/* Store the mapped address in the runtime structure */
		switch (i) {
		case 0:
			dev->mmio.top_base = base;
			break;
		case 1:
			dev->mmio.protect_base = base;
			break;
		case 2:
			dev->mmio.ckm_base = base;
			break;
		case 3:
			if (dev->pd->port_nums > 0)
				dev->mmio.mac_port_base[0] = base;
			break;
		case 4:
			if (dev->pd->port_nums > 1)
				dev->mmio.mac_port_base[1] = base;
			break;
		}
	}

	dev_info(&pdev->dev, "DEBUG:SCK: ========================================\n");
	dev_info(&pdev->dev, "DEBUG:SCK: MMIO Mapping Summary:\n");
	dev_info(&pdev->dev, "DEBUG:SCK:   TOP:        %p (0x%08x)\n", dev->mmio.top_base, 
	         parsed.mmio_regions[0].address);
	dev_info(&pdev->dev, "DEBUG:SCK:   PROTECT:    %p (0x%08x)\n", dev->mmio.protect_base,
	         parsed.mmio_regions[1].address);
	dev_info(&pdev->dev, "DEBUG:SCK:   CKM:        %p (0x%08x)\n", dev->mmio.ckm_base,
	         parsed.mmio_regions[2].address);
	dev_info(&pdev->dev, "DEBUG:SCK:   MAC Port 0: %p (0x%08x)\n", dev->mmio.mac_port_base[0],
	         parsed.mmio_regions[3].address);
	dev_info(&pdev->dev, "DEBUG:SCK:   MAC Port 1: %p (0x%08x)\n", dev->mmio.mac_port_base[1],
	         parsed.mmio_regions[4].address);
	dev_info(&pdev->dev, "DEBUG:SCK: All %d MMIO regions mapped successfully!\n", parsed.count);
	dev_info(&pdev->dev, "DEBUG:SCK: ========================================\n");
	dev_info(&pdev->dev, "Successfully mapped all MMIO regions from ACPI _CRS\n");
	return 0;
}
 
static void mt8901_rp_bus_protect(struct pcie_hp_dev *dev, int port_idx, int stage)
{
    struct rp_bus_mmio_info *mmio_info = &dev->pd->rp_bus_mmio;
    void __iomem *mac_base;

    if (port_idx >= dev->pd->port_nums)
        return;

    mac_base = dev->mmio.mac_port_base[port_idx];
    if (!mac_base)
        return;

    if (stage == BUS_PROTECT_INIT) {
        /* Initialize bus protection during probe - nothing to do */
        return;
    }

    if (stage == BUS_PROTECT_CABLE_REMOVAL) {
        /* Deassert LTSSM enable and PHY reset */
        pcie_hp_reg_update_bits(mac_base, mmio_info->mac.init_ctrl,
                                 mmio_info->mac.ltssm_bit, false);
        pcie_hp_reg_update_bits(mac_base, mmio_info->mac.init_ctrl,
                                 mmio_info->mac.phy_rst_bit, false);
    }

    if (stage == BUS_PROTECT_CABLE_PLUGIN) {
        if (!dev->mmio.top_base || !dev->mmio.protect_base)
            return;

        /* Deassert way_en */
        pcie_hp_toggle_update_bit(dev->mmio.top_base, mmio_info->top.ctrl,
                                   mmio_info->top.port_bits[port_idx],
                                   mmio_info->top.update_bit, false);
        udelay(PCIE_HP_DELAY_SHORT_US);

        /* Enable bus protection */
        pcie_hp_bus_protect_enable(dev, port_idx);
        usleep_range(PCIE_HP_DELAY_BUS_PROTECT_US, PCIE_HP_DELAY_BUS_PROTECT_US + 1000);

        /* Assert LTSSM enable and PHY reset */
        pcie_hp_reg_update_bits(mac_base, mmio_info->mac.init_ctrl,
                                 mmio_info->mac.phy_rst_bit, true);
        pcie_hp_reg_update_bits(mac_base, mmio_info->mac.init_ctrl,
                                 mmio_info->mac.ltssm_bit, true);
        usleep_range(PCIE_HP_DELAY_PHY_RESET_US, PCIE_HP_DELAY_PHY_RESET_US + 1000);

        /* Disable bus protection */
        pcie_hp_bus_protect_disable(dev, port_idx);

        /* Assert way_en */
        pcie_hp_toggle_update_bit(dev->mmio.top_base, mmio_info->top.ctrl,
                                   mmio_info->top.port_bits[port_idx],
                                   mmio_info->top.update_bit, true);
    }
}
 
static void retrain_pcie_link(struct pci_dev *dev)
{
    u16 link_control, lnksta;
    int pos, i = 0;

    pos = pci_find_capability(dev, PCI_CAP_ID_EXP);
    if (!pos) {
        dev_err(&dev->dev, "PCIe capability not found\n");
        return;
    }

    pci_read_config_word(dev, pos + PCI_EXP_LNKCTL, &link_control);
    link_control |= PCI_EXP_LNKCTL_RL;

    pci_write_config_word(dev, pos + PCI_EXP_LNKCTL, link_control);

    while (i < HP_POLL_CNT_MAX) {
        i++;
        pcie_capability_read_word(dev, PCI_EXP_LNKSTA, &lnksta);
        if (lnksta & PCI_EXP_LNKSTA_DLLLA)
            break;
        usleep_range(PCIE_HP_POLL_SLEEP_US, PCIE_HP_POLL_SLEEP_US + 1000);
    }

    pcie_capability_write_word(dev, PCI_EXP_LNKSTA, PCI_EXP_LNKSTA_LBMS);
}

static struct pci_dev *
get_port_root_port(struct pcie_hp_dev *hp_dev, int port_idx)
{
    struct pcie_port_info *port;

    if (port_idx >= hp_dev->pd->port_nums)
        return NULL;

    port = &hp_dev->pd->ports[port_idx];

    /* Try to find the root port if not cached */
    if (!hp_dev->cached_root_ports[port_idx]) {
        hp_dev->cached_root_ports[port_idx] = 
            pci_get_domain_bus_and_slot(port->domain,
                                                        port->bus,
                                                        port->devfn);
        if (!hp_dev->cached_root_ports[port_idx]) {
            dev_warn(&hp_dev->pdev->dev,
                     "Root port not found for domain %d bus %d\n",
                     port->domain, port->bus);
            return NULL;
        }
    }

    return hp_dev->cached_root_ports[port_idx];
}
 
static void remove_device(struct pcie_hp_dev *dev)
{
    int i;

    dev_info(&dev->pdev->dev, "DEBUG:SCK: remove_device() START\n");

    /* Apply bus protection before removal */
    if (dev->pd->rp_bus_protect) {
        dev_info(&dev->pdev->dev, "DEBUG:SCK: Applying bus protection for removal...\n");
        for (i = 0; i < dev->pd->port_nums; i++) {
            dev_info(&dev->pdev->dev, "DEBUG:SCK: Bus protect port %d\n", i);
            dev->pd->rp_bus_protect(dev, i, BUS_PROTECT_CABLE_REMOVAL);
        }
    }

    /* Deassert PCIe reset */
    dev_info(&dev->pdev->dev, "DEBUG:SCK: Deasserting PERST (set to 0)\n");
    gpiod_set_value(dev->pins[PCIE_PIN_PERST].desc, 0);

    /* Change pinctrl state */
    dev_info(&dev->pdev->dev, "DEBUG:SCK: Changing pinctrl state to 'default'\n");
    pcie_hp_change_state(dev, "default");

    /* Disable clock */
    dev_info(&dev->pdev->dev, "DEBUG:SCK: Disabling clock (CKM control)\n");
    pcie_hp_ckm_control(dev, true);

    /* Power off device */
    dev_info(&dev->pdev->dev, "DEBUG:SCK: Powering off device (EN GPIO = 0)\n");
    gpiod_set_value(dev->pins[PCIE_PIN_EN].desc, 0);
    dev_info(&dev->pdev->dev, "DEBUG:SCK: remove_device() COMPLETE\n");
}

static int polling_link_to_l0(struct pcie_hp_dev *dev)
{
    struct pci_dev *pci_dev;
    u32 ltssm_reg = dev->pd->ltssm_reg;
    u32 l0_state = dev->pd->ltssm_l0_state;
    u32 *ltssm_vals;
    int count = 0;
    int i, all_ready;

    if (!ltssm_reg || !l0_state)
        return 0; /* Skip if not configured */

    ltssm_vals = kcalloc(dev->pd->port_nums, sizeof(u32), GFP_KERNEL);
    if (!ltssm_vals)
        return -ENOMEM;

    /* Poll until all ports reach L0 state */
    while (count < HP_POLL_CNT_MAX) {
        all_ready = 1;
        
        for (i = 0; i < dev->pd->port_nums; i++) {
            pci_dev = get_port_root_port(dev, i);
            if (!pci_dev)
                continue;

            pci_read_config_dword(pci_dev, ltssm_reg, &ltssm_vals[i]);
            if ((ltssm_vals[i] & l0_state) != l0_state)
                all_ready = 0;
        }

        if (all_ready)
            break;

        usleep_range(PCIE_HP_POLL_SLEEP_US, PCIE_HP_POLL_SLEEP_US + 1000);
        count++;
    }

    kfree(ltssm_vals);

    if (count >= HP_POLL_CNT_MAX) {
        dev_err(&dev->pdev->dev, "Timeout waiting for link to reach L0\n");
        return -ETIMEDOUT;
    }

    return 0;
}

static int rescan_device(struct pcie_hp_dev *dev)
{
    struct pci_dev *pci_dev;
    int i, err;

    dev_info(&dev->pdev->dev, "DEBUG:SCK: rescan_device() START\n");

    /* Change pinctrl state */
    dev_info(&dev->pdev->dev, "DEBUG:SCK: Changing pinctrl state to 'clkreqn'\n");
    err = pcie_hp_change_state(dev, "clkreqn");
    if (err) {
        dev_err(&dev->pdev->dev, "DEBUG:SCK: pcie_hp_change_state() FAILED: %d\n", err);
        return err;
    }

    /* Enable clock */
    dev_info(&dev->pdev->dev, "DEBUG:SCK: Enabling clock (CKM control)\n");
    pcie_hp_ckm_control(dev, false);
    usleep_range(PCIE_HP_DELAY_STANDARD_US, PCIE_HP_DELAY_STANDARD_US + 1000);

    /* Resume root ports */
    dev_info(&dev->pdev->dev, "DEBUG:SCK: Resuming root ports...\n");
    for (i = 0; i < dev->pd->port_nums; i++) {
        pci_dev = get_port_root_port(dev, i);
        if (!pci_dev) {
            dev_info(&dev->pdev->dev, "DEBUG:SCK: Port %d: No root port found\n", i);
            continue;
        }

        dev_info(&dev->pdev->dev, "DEBUG:SCK: Port %d: Resuming %s\n", i, pci_name(pci_dev));
        err = pm_runtime_resume_and_get(&pci_dev->dev);
        if (err < 0) {
            dev_err(&dev->pdev->dev,
                    "DEBUG:SCK: Port %d: Runtime resume FAILED: %d\n", i, err);
            dev_err(&dev->pdev->dev,
                    "Runtime resume failed for %s: %d\n",
                    pci_name(pci_dev), err);
        } else {
            dev_info(&dev->pdev->dev, "DEBUG:SCK: Port %d: Runtime resume OK\n", i);
        }
    }

    /* Assert PCIe reset */
    dev_info(&dev->pdev->dev, "DEBUG:SCK: Asserting PERST (set to 1)\n");
    gpiod_set_value(dev->pins[PCIE_PIN_PERST].desc, 1);

    /* Apply bus protection */
    if (dev->pd->rp_bus_protect) {
        dev_info(&dev->pdev->dev, "DEBUG:SCK: Applying bus protection for plug-in...\n");
        for (i = 0; i < dev->pd->port_nums; i++) {
            dev_info(&dev->pdev->dev, "DEBUG:SCK: Bus protect port %d\n", i);
            dev->pd->rp_bus_protect(dev, i, BUS_PROTECT_CABLE_PLUGIN);
        }
    }

    /* Wait for links to reach L0 */
    dev_info(&dev->pdev->dev, "DEBUG:SCK: Polling for links to reach L0 state...\n");
    err = polling_link_to_l0(dev);
    if (err) {
        dev_err(&dev->pdev->dev, "DEBUG:SCK: polling_link_to_l0() FAILED: %d\n", err);
        return err;
    }
    dev_info(&dev->pdev->dev, "DEBUG:SCK: All links reached L0 state\n");

    /* Retrain PCIe links */
    dev_info(&dev->pdev->dev, "DEBUG:SCK: Retraining PCIe links...\n");
    for (i = 0; i < dev->pd->port_nums; i++) {
        pci_dev = get_port_root_port(dev, i);
        if (pci_dev) {
            dev_info(&dev->pdev->dev, "DEBUG:SCK: Port %d: Retraining %s\n", i, pci_name(pci_dev));
            retrain_pcie_link(pci_dev);
        }
    }

    /* Wait for link stability */
    dev_info(&dev->pdev->dev, "DEBUG:SCK: Waiting %d ms for link stability...\n", PCIE_HP_DELAY_LINK_STABLE_MS);
    msleep(PCIE_HP_DELAY_LINK_STABLE_MS);

    dev_info(&dev->pdev->dev, "DEBUG:SCK: rescan_device() COMPLETE (success)\n");
    return 0;
}
 
static irqreturn_t pcie_hp_work(int irq, void *dev_id)
{
    struct pcie_hp_gpio_ctx *app_ctx = dev_id;
    struct pcie_hp_dev *hp_dev = app_ctx->hp_dev;
    enum pcie_hp_state state;
    unsigned long flags;
    int ret;

    spin_lock_irqsave(&hp_dev->lock, flags);
    state = hp_dev->state;

    dev_info(app_ctx->ctx->dev, "DEBUG:SCK: pcie_hp_work() IRQ handler - current state=%d\n", state);

    switch (state) {
    case STATE_PLUG_OUT:
        dev_info(app_ctx->ctx->dev, "DEBUG:SCK: pcie_hp_work() - STATE_PLUG_OUT\n");
        dev_dbg(app_ctx->ctx->dev, "Cable plug out\n");
        remove_device(hp_dev);
        break;
    case STATE_PLUG_IN:
        dev_info(app_ctx->ctx->dev, "DEBUG:SCK: pcie_hp_work() - STATE_PLUG_IN\n");
        dev_dbg(app_ctx->ctx->dev, "Enable device power\n");
        dev_info(app_ctx->ctx->dev, "DEBUG:SCK: Setting EN GPIO = 1\n");
        gpiod_set_value(hp_dev->pins[PCIE_PIN_EN].desc, 1);
        dev_info(app_ctx->ctx->dev, "DEBUG:SCK: EN GPIO set, device powering on\n");
        break;
    case STATE_DEV_POWER_OFF:
        dev_info(app_ctx->ctx->dev, "DEBUG:SCK: pcie_hp_work() - STATE_DEV_POWER_OFF\n");
        dev_dbg(app_ctx->ctx->dev, "Waiting for device to be ready\n");
        break;
    case STATE_DEV_POWER_ON:
        dev_info(app_ctx->ctx->dev, "DEBUG:SCK: pcie_hp_work() - STATE_DEV_POWER_ON\n");
        dev_dbg(app_ctx->ctx->dev, "Waiting for device to be ready\n");
        break;
    case STATE_DEV_FW_START:
        dev_info(app_ctx->ctx->dev, "DEBUG:SCK: pcie_hp_work() - STATE_DEV_FW_START\n");
        dev_dbg(app_ctx->ctx->dev, "Waiting for device to be ready\n");
        break;
    case STATE_RESCAN:
        dev_info(app_ctx->ctx->dev, "DEBUG:SCK: pcie_hp_work() - STATE_RESCAN\n");
        dev_dbg(app_ctx->ctx->dev, "Cable plug in, rescanning\n");
        dev_info(app_ctx->ctx->dev, "DEBUG:SCK: Calling rescan_device()...\n");
        ret = rescan_device(hp_dev);
        if (ret) {
            dev_err(app_ctx->ctx->dev, "DEBUG:SCK: rescan_device() FAILED: %d\n", ret);
            dev_err(app_ctx->ctx->dev, "Rescan failed: %d\n", ret);
        } else {
            dev_info(app_ctx->ctx->dev, "DEBUG:SCK: rescan_device() SUCCESS\n");
            hp_dev->state = STATE_READY;
            dev_info(app_ctx->ctx->dev, "DEBUG:SCK: State changed to STATE_READY\n");
        }
        break;
    default:
        dev_err(app_ctx->ctx->dev, "DEBUG:SCK: pcie_hp_work() - UNKNOWN STATE: %d\n", state);
        dev_err(app_ctx->ctx->dev, "Unknown state: %d\n", state);
        break;
    }

    dev_info(app_ctx->ctx->dev, "DEBUG:SCK: pcie_hp_work() COMPLETE\n");
    spin_unlock_irqrestore(&hp_dev->lock, flags);
    return IRQ_HANDLED;
}

static irqreturn_t hotplug_irq_handler(int irq, void *dev_id)
{
    struct pcie_hp_gpio_ctx *app_ctx = dev_id;
    struct pcie_hp_dev *hp_dev = app_ctx->hp_dev;
    struct gpio_acpi_context *gpio_ctx = app_ctx->ctx;
    unsigned long flags;
    int value;
    enum pcie_hp_state state;

    value = gpiod_get_value(app_ctx->desc);
    dev_info(gpio_ctx->dev, "DEBUG:SCK: hotplug_irq_handler() - IRQ %d fired, pin=%d, value=%d\n",
             irq, gpio_ctx->pin, value);

    spin_lock_irqsave(&hp_dev->lock, flags);

    /* Ignore GPIO events during sysfs operations to prevent conflicts */
    if (hp_dev->suppress_gpio_irq) {
        dev_info(gpio_ctx->dev, "DEBUG:SCK: IRQ suppressed (sysfs operation active)\n");
        dev_dbg(gpio_ctx->dev, "IRQ ignored: GPIO suppression active (pin=%d, irq=%d)\n",
                gpio_ctx->pin, irq);
        spin_unlock_irqrestore(&hp_dev->lock, flags);
        return IRQ_HANDLED;
    }
    state = hp_dev->state;
    dev_info(gpio_ctx->dev, "DEBUG:SCK: Current state=%d\n", state);

    /* Handle presence pin events */
    if (gpio_ctx->pin == hp_dev->prsnt_pin) {
        dev_info(gpio_ctx->dev, "DEBUG:SCK: PRSNT pin IRQ - value=%d\n", value);
        if (value) {
            dev_info(gpio_ctx->dev, "DEBUG:SCK: Cable REMOVAL detected\n");
            dev_dbg(gpio_ctx->dev, "Presence pin: cable removal detected\n");
            pcie_hp_send_uevent(hp_dev, REMOVAL_EVT);
        } else {
            dev_info(gpio_ctx->dev, "DEBUG:SCK: Cable PLUG-IN detected\n");
            dev_dbg(gpio_ctx->dev, "Presence pin: cable plug-in detected\n");
            pcie_hp_send_uevent(hp_dev, PLUG_IN_EVT);
        }
        spin_unlock_irqrestore(&hp_dev->lock, flags);
        /* For physical hotplug, let userspace handle device management
         * to avoid potential deadlocks when hardware is in transition */
        return IRQ_HANDLED;
    }

    /* Handle boot status pin events */
    if (gpio_ctx->pin == hp_dev->boot_pin) {
        dev_info(gpio_ctx->dev, "DEBUG:SCK: BOOT pin IRQ - value=%d, state=%d\n", value, state);
        if (value && state == STATE_PLUG_IN) {
            dev_info(gpio_ctx->dev, "DEBUG:SCK: BOOT high + STATE_PLUG_IN → STATE_DEV_POWER_ON\n");
            dev_dbg(gpio_ctx->dev, "Boot pin high: device powered on\n");
            hp_dev->state = STATE_DEV_POWER_ON;
        } else if (value && state == STATE_DEV_FW_START) {
            dev_info(gpio_ctx->dev, "DEBUG:SCK: BOOT high + STATE_DEV_FW_START → STATE_RESCAN (device ready!)\n");
            dev_dbg(gpio_ctx->dev, "Boot pin high: device ready\n");
            hp_dev->state = STATE_RESCAN;
        } else if (!value && state == STATE_DEV_POWER_ON) {
            dev_info(gpio_ctx->dev, "DEBUG:SCK: BOOT low + STATE_DEV_POWER_ON → STATE_DEV_FW_START\n");
            dev_dbg(gpio_ctx->dev, "Boot pin low: firmware starting\n");
            hp_dev->state = STATE_DEV_FW_START;
        } else if (!value && state == STATE_PLUG_OUT) {
            dev_info(gpio_ctx->dev, "DEBUG:SCK: BOOT low + STATE_PLUG_OUT → STATE_DEV_POWER_OFF\n");
            dev_dbg(gpio_ctx->dev, "Boot pin low: device powered off\n");
            hp_dev->state = STATE_DEV_POWER_OFF;
        } else {
            dev_info(gpio_ctx->dev, "DEBUG:SCK: BOOT pin event ignored - value=%d state=%d (no transition)\n",
                     value, state);
            dev_dbg(gpio_ctx->dev, "Boot pin changed: pin=%d irq=%d value=%d state=%d\n",
                    gpio_ctx->pin, irq, value, state);
            spin_unlock_irqrestore(&hp_dev->lock, flags);
            return IRQ_HANDLED;
        }
        dev_info(gpio_ctx->dev, "DEBUG:SCK: State transition complete, waking threaded handler\n");
        spin_unlock_irqrestore(&hp_dev->lock, flags);
        return IRQ_WAKE_THREAD;
    }

    /* Unknown GPIO pin */
    dev_warn(gpio_ctx->dev, "DEBUG:SCK: UNKNOWN GPIO pin IRQ - pin=%d, irq=%d, value=%d\n",
             gpio_ctx->pin, irq, value);
    dev_warn(gpio_ctx->dev, "Unknown GPIO pin event: pin=%d irq=%d value=%d\n",
             gpio_ctx->pin, irq, value);
    spin_unlock_irqrestore(&hp_dev->lock, flags);
    return IRQ_HANDLED;
}
 
static acpi_status acpi_gpio_resource_handler(struct acpi_resource *ares, void *context)
{
    struct acpi_gpio_parse_context *parse_ctx = context;
    struct gpio_acpi_context *ctx = parse_ctx->ctx;
    struct pcie_hp_dev *hp_dev = parse_ctx->hp_dev;
    struct acpi_resource_gpio *agpio;
    int length;

    if (ares->type != ACPI_RESOURCE_TYPE_GPIO)
        return AE_OK;

    agpio = &ares->data.gpio;

    if (ctx->pin != agpio->pin_table[0])
        return AE_OK;

    ctx->valid = 1;
    ctx->debounce_timeout_us = agpio->debounce_timeout * 10;
    ctx->wake_capable = agpio->wake_capable;
    ctx->triggering = agpio->triggering;
    ctx->polarity = agpio->polarity;
    ctx->connection_type = agpio->connection_type;

    if (agpio->vendor_length) {
        length = min_t(int, agpio->vendor_length, MAX_VENDOR_DATA_LEN);

        memcpy(&ctx->vendor_data[0], agpio->vendor_data, length);
        ctx->vendor_data[MAX_VENDOR_DATA_LEN] = 0;

        /* Identify special GPIO pins by vendor data */
        if (!strncmp("BOOT", ctx->vendor_data, strlen("BOOT")))
            hp_dev->boot_pin = ctx->pin;
        else if (!strncmp("PRSNT", ctx->vendor_data, strlen("PRSNT")))
            hp_dev->prsnt_pin = ctx->pin;
    }

    /* Set IRQ flags based on triggering and polarity */
    if (agpio->triggering == ACPI_EDGE_SENSITIVE) {
        if (agpio->polarity == ACPI_ACTIVE_LOW)
            ctx->irq_flags = IRQF_TRIGGER_FALLING;
        else if (agpio->polarity == ACPI_ACTIVE_HIGH)
            ctx->irq_flags = IRQF_TRIGGER_RISING;
        else
            ctx->irq_flags = (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);
    } else {
        if (agpio->polarity == ACPI_ACTIVE_LOW)
            ctx->irq_flags = IRQF_TRIGGER_LOW;
        else
            ctx->irq_flags = IRQF_TRIGGER_HIGH;
    }

    return AE_OK;
}
 
/**
 * pci_devices_present_on_domain() - Check if PCI devices exist on a domain
 * @domain: PCI domain number to check
 *
 * Returns true if any PCI devices are present on the specified domain,
 * false otherwise. This is used as a safety check before hardware shutdown.
 */
static bool pci_devices_present_on_domain(int domain)
{
	struct pci_bus *bus;
	struct pci_dev *dev;
	bool has_endpoint_devices = false;

	/* Find the secondary bus (bus 01) for this domain
	 * This is where endpoint devices live, not the root ports.
	 * Only check for endpoint devices, not root ports (which must remain). */
	bus = pci_find_bus(domain, 1);
	if (!bus)
		return false;  /* No secondary bus means no endpoints */

	/* Check if the secondary bus has any devices (CX7 endpoints) */
	list_for_each_entry(dev, &bus->devices, bus_list) {
		has_endpoint_devices = true;
		break;
	}

	return has_endpoint_devices;
}
 
static ssize_t debug_state_show(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
    struct pcie_hp_dev *hp_dev = dev_get_drvdata(dev);
    
    if (!hp_dev)
        return -EINVAL;

    return scnprintf(buf, PAGE_SIZE, "%d\n", hp_dev->debug_state);
}

static ssize_t debug_state_store(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf, size_t count)
{
    struct pcie_hp_dev *hp_dev = dev_get_drvdata(dev);
    unsigned long val, flags;
    int err, i;

    if (!hp_dev)
        return -EINVAL;

    err = kstrtoul(buf, 10, &val);
    if (err)
        return err;

    spin_lock_irqsave(&hp_dev->lock, flags);

    /* Suppress GPIO interrupts during sysfs-initiated operations */
    hp_dev->suppress_gpio_irq = true;

    switch (val) {
    case PCIE_HP_DEBUG_PLUG_OUT:
        dev_info(dev, "DEBUG:SCK: ========================================\n");
        dev_info(dev, "DEBUG:SCK: debug_state_store() - PLUG_OUT Request\n");
        dev_info(dev, "DEBUG:SCK: ========================================\n");
        dev_info(dev, "Debug: simulating cable removal\n");
        
        /* Safety check: Verify no devices on the bus before hardware shutdown.
         * This prevents silicon bug where CPU access during link down causes
         * system hang. Userspace must remove PCI devices first. */
        dev_info(dev, "DEBUG:SCK: Checking for PCI devices on managed domains...\n");
        for (i = 0; i < hp_dev->pd->port_nums; i++) {
            dev_info(dev, "DEBUG:SCK: Checking domain 0x%04x...\n", hp_dev->pd->ports[i].domain);
            if (pci_devices_present_on_domain(hp_dev->pd->ports[i].domain)) {
                dev_err(dev, "DEBUG:SCK: SAFETY CHECK FAILED - Devices present on domain 0x%04x\n", 
                        hp_dev->pd->ports[i].domain);
                dev_err(dev, "PCI devices still present, remove them first\n");
                hp_dev->suppress_gpio_irq = false;
                spin_unlock_irqrestore(&hp_dev->lock, flags);
                return -EBUSY;
            }
        }
        dev_info(dev, "DEBUG:SCK: Safety check PASSED - No devices on bus\n");
        
        /* Safe to proceed - no devices on bus */
        dev_info(dev, "DEBUG:SCK: Setting state=STATE_PLUG_OUT\n");
        hp_dev->state = STATE_PLUG_OUT;
        dev_info(dev, "DEBUG:SCK: Calling remove_device()...\n");
        remove_device(hp_dev);
        dev_info(dev, "DEBUG:SCK: remove_device() completed\n");
        hp_dev->suppress_gpio_irq = false;  /* Re-enable GPIO interrupt handling */
        dev_info(dev, "DEBUG:SCK: PLUG_OUT Complete\n");
        break;

    case PCIE_HP_DEBUG_PLUG_IN:
        dev_info(dev, "DEBUG:SCK: ========================================\n");
        dev_info(dev, "DEBUG:SCK: debug_state_store() - PLUG_IN Request\n");
        dev_info(dev, "DEBUG:SCK: ========================================\n");
        dev_info(dev, "Debug: simulating cable plug-in\n");
        
        dev_info(dev, "DEBUG:SCK: Checking for PCI devices on managed domains...\n");
        for (i = 0; i < hp_dev->pd->port_nums; i++) {
            dev_info(dev, "DEBUG:SCK: Checking domain 0x%04x...\n", hp_dev->pd->ports[i].domain);
            if (pci_devices_present_on_domain(hp_dev->pd->ports[i].domain)) {
                dev_err(dev, "DEBUG:SCK: SAFETY CHECK FAILED - Devices already present on domain 0x%04x\n",
                        hp_dev->pd->ports[i].domain);
                dev_err(dev, "PCI devices already present, cannot reinitialize hardware\n");
                hp_dev->suppress_gpio_irq = false;
                spin_unlock_irqrestore(&hp_dev->lock, flags);
                return -EBUSY;
            }
        }
        dev_info(dev, "DEBUG:SCK: Safety check PASSED - No devices on bus\n");
        
        /* Safe to proceed - no devices on bus */
        dev_info(dev, "DEBUG:SCK: Setting state=STATE_PLUG_IN\n");
        hp_dev->state = STATE_PLUG_IN;
        
        /* Enable device power - GPIO IRQ state machine will handle rest */
        dev_info(dev, "DEBUG:SCK: Checking EN GPIO availability...\n");
        if (hp_dev->pins[PCIE_PIN_EN].desc) {
            dev_info(dev, "DEBUG:SCK: EN GPIO available - desc=%p\n", hp_dev->pins[PCIE_PIN_EN].desc);
            dev_info(dev, "DEBUG:SCK: Setting EN GPIO = 1 (power on)\n");
            gpiod_set_value(hp_dev->pins[PCIE_PIN_EN].desc, 1);
            dev_info(dev, "DEBUG:SCK: EN GPIO set, waiting for GPIO IRQ state machine\n");
        } else {
            dev_warn(dev, "DEBUG:SCK: EN GPIO is NULL - this shouldn't happen with _DSD!\n");
        }
        
        hp_dev->suppress_gpio_irq = false;  /* Re-enable GPIO interrupt handling */
        dev_info(dev, "DEBUG:SCK: Re-enabled GPIO IRQs\n");
        dev_info(dev, "DEBUG:SCK: PLUG_IN initiated - waiting for BOOT GPIO transitions\n");
        break;

    default:
        hp_dev->suppress_gpio_irq = false;
        spin_unlock_irqrestore(&hp_dev->lock, flags);
        return -EINVAL;
    }

    hp_dev->debug_state = val;
    spin_unlock_irqrestore(&hp_dev->lock, flags);

    return count;
}

DEVICE_ATTR_RW(debug_state);

static struct attribute *pcie_hp_attrs[] = {
    &dev_attr_debug_state.attr,
    NULL
};

static const struct attribute_group pcie_hp_attr_group = {
    .name = "pcie_hotplug",
    .attrs = pcie_hp_attrs
};
 
static struct gpio_acpi_context *gpio_acpi_setup(struct platform_device *pdev,
                                                   struct gpio_desc *desc,
                                                   struct pcie_hp_dev *hp_dev)
{
    struct acpi_gpio_parse_context parse_ctx;
    struct gpio_acpi_context *ctx;
    struct acpi_device *adev;

    adev = ACPI_COMPANION(&pdev->dev);
    if (!adev) {
        dev_err(&pdev->dev, "Failed to get ACPI companion device\n");
        return NULL;
    }

    ctx = devm_kzalloc(&pdev->dev, sizeof(*ctx), GFP_KERNEL);
    if (!ctx)
        return NULL;

    ctx->pin = desc_to_gpio(desc) - gpio_device_get_base(gpiod_to_gpio_device(desc));
    ctx->dev = &pdev->dev;

    parse_ctx.ctx = ctx;
    parse_ctx.hp_dev = hp_dev;

    acpi_walk_resources(adev->handle, METHOD_NAME__CRS,
                        acpi_gpio_resource_handler, &parse_ctx);

    if (ctx->valid)
        return ctx;

    devm_kfree(&pdev->dev, ctx);
    return NULL;
}

static int pcie_hp_setup_irq(struct pcie_hp_gpio_ctx *app_ctx)
{
    struct gpio_acpi_context *ctx = app_ctx->ctx;
    int irq, ret;

    irq = gpiod_to_irq(app_ctx->desc);
    if (irq < 0) {
        dev_err(ctx->dev, "Failed to get IRQ for GPIO\n");
        return irq;
    }

    if (ctx->wake_capable)
        enable_irq_wake(irq);

    ret = devm_request_threaded_irq(ctx->dev, irq,
                                     hotplug_irq_handler, pcie_hp_work,
                                     ctx->irq_flags | IRQF_ONESHOT,
                                     "pcie_hotplug", app_ctx);
    if (ret)
        dev_err(ctx->dev, "Failed to request IRQ %d: %d\n", irq, ret);

    return ret;
}

/*
 * pcie_hp_probe_gpios - Enumerate GPIOs using ACPI _DSD named connections
 *
 * This function uses named GPIO connections defined in ACPI _DSD to access
 * all 6 GPIOs regardless of kernel ACPI GPIO grouping behavior.
 *
 * ACPI _DSD provides:
 *   - boot-gpios: BOOT status pin
 *   - prsnt-gpios: Presence detection pin
 *   - perst-gpios: PCIe reset pin
 *   - enable-gpios: Power enable pin
 *   - clkreq-gpios: Clock request pins (CLQ0, CLQ1 as array)
 *
 * Returns: 0 on success, negative error code on failure
 */
static int pcie_hp_probe_gpios(struct platform_device *pdev,
                                struct pcie_hp_dev *hp_dev)
{
    struct device *dev = &pdev->dev;
    struct pcie_hp_gpio_ctx *pin_ctx;
    int ret;

    dev_info(dev, "DEBUG:SCK: ========================================\n");
    dev_info(dev, "DEBUG:SCK: Starting ACPI _DSD GPIO Enumeration\n");
    dev_info(dev, "DEBUG:SCK: ========================================\n");

    /* Allocate GPIO context array for all pins */
    hp_dev->pins = devm_kcalloc(dev, PCIE_PIN_MAX,
                                 sizeof(struct pcie_hp_gpio_ctx),
                                 GFP_KERNEL);
    if (!hp_dev->pins) {
        dev_err(dev, "DEBUG:SCK: Failed to allocate GPIO context array\n");
        return -ENOMEM;
    }
    dev_info(dev, "DEBUG:SCK: Allocated pins array for %d GPIOs\n", PCIE_PIN_MAX);

    /* GPIO 0: BOOT - Device boot status (GpioInt) */
    dev_info(dev, "DEBUG:SCK: [GPIO 0/6] Requesting BOOT via devm_gpiod_get(dev, \"boot\", GPIOD_IN)...\n");
    pin_ctx = &hp_dev->pins[PCIE_PIN_BOOT];
    pin_ctx->desc = devm_gpiod_get(dev, "boot", GPIOD_IN);
    if (IS_ERR(pin_ctx->desc)) {
        ret = PTR_ERR(pin_ctx->desc);
        dev_err(dev, "DEBUG:SCK: [GPIO 0/6] BOOT FAILED: error=%d (%s)\n", ret,
                ret == -ENOENT ? "ENOENT-not-found" :
                ret == -EPROBE_DEFER ? "EPROBE_DEFER" :
                ret == -EINVAL ? "EINVAL-invalid" : "unknown");
        dev_err(dev, "Failed to get BOOT GPIO: %d\n", ret);
        return ret;
    }
    dev_info(dev, "DEBUG:SCK: [GPIO 0/6] BOOT SUCCESS - desc=%p\n", pin_ctx->desc);
    pin_ctx->hp_dev = hp_dev;

    /* GPIO 1: PRSNT - Presence detection (GpioInt) */
    dev_info(dev, "DEBUG:SCK: [GPIO 1/6] Requesting PRSNT via devm_gpiod_get(dev, \"prsnt\", GPIOD_IN)...\n");
    pin_ctx = &hp_dev->pins[PCIE_PIN_PRSNT];
    pin_ctx->desc = devm_gpiod_get(dev, "prsnt", GPIOD_IN);
    if (IS_ERR(pin_ctx->desc)) {
        ret = PTR_ERR(pin_ctx->desc);
        dev_err(dev, "DEBUG:SCK: [GPIO 1/6] PRSNT FAILED: error=%d (%s)\n", ret,
                ret == -ENOENT ? "ENOENT-not-found" :
                ret == -EPROBE_DEFER ? "EPROBE_DEFER" :
                ret == -EINVAL ? "EINVAL-invalid" : "unknown");
        dev_err(dev, "Failed to get PRSNT GPIO: %d\n", ret);
        return ret;
    }
    dev_info(dev, "DEBUG:SCK: [GPIO 1/6] PRSNT SUCCESS - desc=%p\n", pin_ctx->desc);
    pin_ctx->hp_dev = hp_dev;

    /* GPIO 2: PERST - PCIe reset (GpioIo, output) */
    dev_info(dev, "DEBUG:SCK: [GPIO 2/6] Requesting PERST via devm_gpiod_get(dev, \"perst\", GPIOD_OUT_HIGH)...\n");
    pin_ctx = &hp_dev->pins[PCIE_PIN_PERST];
    pin_ctx->desc = devm_gpiod_get(dev, "perst", GPIOD_OUT_HIGH);
    if (IS_ERR(pin_ctx->desc)) {
        ret = PTR_ERR(pin_ctx->desc);
        dev_err(dev, "DEBUG:SCK: [GPIO 2/6] PERST FAILED: error=%d (%s)\n", ret,
                ret == -ENOENT ? "ENOENT-not-found" :
                ret == -EPROBE_DEFER ? "EPROBE_DEFER" :
                ret == -EINVAL ? "EINVAL-invalid" : "unknown");
        dev_err(dev, "Failed to get PERST GPIO: %d\n", ret);
        return ret;
    }
    dev_info(dev, "DEBUG:SCK: [GPIO 2/6] PERST SUCCESS - desc=%p\n", pin_ctx->desc);
    pin_ctx->hp_dev = hp_dev;

    /* GPIO 3: EN - Power enable (GpioIo, output) */
    dev_info(dev, "DEBUG:SCK: [GPIO 3/6] Requesting EN via devm_gpiod_get(dev, \"enable\", GPIOD_OUT_LOW)...\n");
    pin_ctx = &hp_dev->pins[PCIE_PIN_EN];
    pin_ctx->desc = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
    if (IS_ERR(pin_ctx->desc)) {
        ret = PTR_ERR(pin_ctx->desc);
        dev_err(dev, "DEBUG:SCK: [GPIO 3/6] EN FAILED: error=%d (%s)\n", ret,
                ret == -ENOENT ? "ENOENT-not-found" :
                ret == -EPROBE_DEFER ? "EPROBE_DEFER" :
                ret == -EINVAL ? "EINVAL-invalid" : "unknown");
        dev_err(dev, "Failed to get EN GPIO: %d\n", ret);
        return ret;
    }
    dev_info(dev, "DEBUG:SCK: [GPIO 3/6] EN SUCCESS - desc=%p\n", pin_ctx->desc);
    pin_ctx->hp_dev = hp_dev;

    /* GPIO 4: CLQ0 - Clock request 0 (GpioIo, input, part of clkreq group) */
    dev_info(dev, "DEBUG:SCK: [GPIO 4/6] Requesting CLQ0 via devm_gpiod_get_index(dev, \"clkreq\", 0, GPIOD_IN)...\n");
    pin_ctx = &hp_dev->pins[PCIE_PIN_CLQ0];
    pin_ctx->desc = devm_gpiod_get_index(dev, "clkreq", 0, GPIOD_IN);
    if (IS_ERR(pin_ctx->desc)) {
        ret = PTR_ERR(pin_ctx->desc);
        dev_err(dev, "DEBUG:SCK: [GPIO 4/6] CLQ0 FAILED: error=%d (%s)\n", ret,
                ret == -ENOENT ? "ENOENT-not-found" :
                ret == -EPROBE_DEFER ? "EPROBE_DEFER" :
                ret == -EINVAL ? "EINVAL-invalid" : "unknown");
        dev_err(dev, "Failed to get CLQ0 GPIO: %d\n", ret);
        return ret;
    }
    dev_info(dev, "DEBUG:SCK: [GPIO 4/6] CLQ0 SUCCESS - desc=%p\n", pin_ctx->desc);
    pin_ctx->hp_dev = hp_dev;

    /* GPIO 5: CLQ1 - Clock request 1 (GpioIo, input, part of clkreq group) */
    dev_info(dev, "DEBUG:SCK: [GPIO 5/6] Requesting CLQ1 via devm_gpiod_get_index(dev, \"clkreq\", 1, GPIOD_IN)...\n");
    pin_ctx = &hp_dev->pins[PCIE_PIN_CLQ1];
    pin_ctx->desc = devm_gpiod_get_index(dev, "clkreq", 1, GPIOD_IN);
    if (IS_ERR(pin_ctx->desc)) {
        ret = PTR_ERR(pin_ctx->desc);
        dev_err(dev, "DEBUG:SCK: [GPIO 5/6] CLQ1 FAILED: error=%d (%s)\n", ret,
                ret == -ENOENT ? "ENOENT-not-found" :
                ret == -EPROBE_DEFER ? "EPROBE_DEFER" :
                ret == -EINVAL ? "EINVAL-invalid" : "unknown");
        dev_err(dev, "Failed to get CLQ1 GPIO: %d\n", ret);
        return ret;
    }
    dev_info(dev, "DEBUG:SCK: [GPIO 5/6] CLQ1 SUCCESS - desc=%p\n", pin_ctx->desc);
    pin_ctx->hp_dev = hp_dev;

    dev_info(dev, "DEBUG:SCK: ========================================\n");
    dev_info(dev, "DEBUG:SCK: GPIO Enumeration COMPLETE\n");
    dev_info(dev, "DEBUG:SCK: All %d GPIOs successfully obtained via ACPI _DSD\n", PCIE_PIN_MAX);
    dev_info(dev, "DEBUG:SCK: ========================================\n");
    dev_info(dev, "Successfully enumerated all %d GPIOs via ACPI _DSD\n", PCIE_PIN_MAX);
    return 0;
}
 
static int pcie_hp_discover_devices(struct pcie_hp_dev *hp_dev)
{
    struct pci_dev *pci_dev = NULL;
    int device_count = 0;
    int i;

    /* If no vendor/device ID specified, skip device discovery */
    if (!hp_dev->pd->vendor_id || !hp_dev->pd->device_id)
        return 0;

    /* Find all matching PCI devices */
    while ((pci_dev = pci_get_device(hp_dev->pd->vendor_id,
                                      hp_dev->pd->device_id,
                                      pci_dev)) != NULL) {
        /* Wait for device to be initialized */
        if (!pci_dev->state_saved) {
            pci_dev_put(pci_dev);
            return -EPROBE_DEFER;
        }

        /* Verify device is on one of our managed ports */
        for (i = 0; i < hp_dev->pd->port_nums; i++) {
            if (pci_domain_nr(pci_dev->bus) == hp_dev->pd->ports[i].domain)
                break;
        }

        if (i == hp_dev->pd->port_nums) {
            dev_warn(&hp_dev->pdev->dev,
                     "Device %s found on unexpected domain %d\n",
                     pci_name(pci_dev), pci_domain_nr(pci_dev->bus));
        }

        device_count++;
    }

    if (hp_dev->pd->num_devices && device_count != hp_dev->pd->num_devices) {
        dev_info(&hp_dev->pdev->dev,
                 "Expected %d devices, found %d (may be hot-unplugged)\n",
                 hp_dev->pd->num_devices, device_count);
    }

    return 0;
}

static int pcie_hp_probe(struct platform_device *pdev)
{
    struct pcie_hp_plat_data *pd;
    struct pcie_hp_gpio_ctx *app_ctx;
    struct pcie_hp_dev *hp_dev;
    int ret, i;

    pd = (struct pcie_hp_plat_data *)device_get_match_data(&pdev->dev);
    if (!pd) {
        dev_err(&pdev->dev, "No platform data available\n");
        return -EINVAL;
    }

    if (pd->port_nums >= HP_PORT_MAX) {
        dev_err(&pdev->dev, "Invalid port count: %d (max %d)\n",
                pd->port_nums, HP_PORT_MAX);
        return -EINVAL;
    }

    hp_dev = devm_kzalloc(&pdev->dev, sizeof(*hp_dev), GFP_KERNEL);
    if (!hp_dev)
        return -ENOMEM;

    hp_dev->pdev = pdev;
    hp_dev->pd = pd;
    hp_dev->boot_pin = -1;
    hp_dev->prsnt_pin = -1;
    hp_dev->state = STATE_READY;
    hp_dev->suppress_gpio_irq = false;  /* GPIO interrupt handling enabled by default */
    spin_lock_init(&hp_dev->lock);
    
    /* Initialize cached root port pointers */
    for (i = 0; i < HP_PORT_MAX; i++)
        hp_dev->cached_root_ports[i] = NULL;

    /* Initialize bus protection (like 6.14's rp_bus_prepare during probe) */
    if (pd->rp_bus_protect) {
        for (i = 0; i < pd->port_nums; i++)
            pd->rp_bus_protect(hp_dev, i, BUS_PROTECT_INIT);
    }

    /* Enumerate all GPIOs using ACPI _DSD named connections */
    dev_info(&pdev->dev, "DEBUG:SCK: ========================================\n");
    dev_info(&pdev->dev, "DEBUG:SCK: Starting GPIO Enumeration (ACPI _DSD)\n");
    dev_info(&pdev->dev, "DEBUG:SCK: ========================================\n");
    ret = pcie_hp_probe_gpios(pdev, hp_dev);
    if (ret) {
        dev_err(&pdev->dev, "DEBUG:SCK: pcie_hp_probe_gpios() FAILED with error %d\n", ret);
        dev_err(&pdev->dev, "Failed to enumerate GPIOs: %d\n", ret);
        return ret;
    }
    dev_info(&pdev->dev, "DEBUG:SCK: pcie_hp_probe_gpios() returned SUCCESS\n");
    
    hp_dev->gpio_count = PCIE_PIN_MAX;
    dev_info(&pdev->dev, "DEBUG:SCK: Set gpio_count = %d\n", hp_dev->gpio_count);

    /* Setup ACPI context and IRQs for interrupt-capable GPIOs */
    dev_info(&pdev->dev, "DEBUG:SCK: ========================================\n");
    dev_info(&pdev->dev, "DEBUG:SCK: Setting up ACPI context and IRQs\n");
    dev_info(&pdev->dev, "DEBUG:SCK: ========================================\n");
    for (i = 0; i < PCIE_PIN_MAX; i++) {
        app_ctx = &hp_dev->pins[i];
        
        if (!app_ctx->desc) {
            dev_warn(&pdev->dev, "DEBUG:SCK: GPIO %d: desc is NULL, skipping\n", i);
            continue;
        }

        dev_info(&pdev->dev, "DEBUG:SCK: GPIO %d: Setting up ACPI context...\n", i);
        /* Setup ACPI GPIO context for metadata */
        app_ctx->ctx = gpio_acpi_setup(pdev, app_ctx->desc, hp_dev);
        if (!app_ctx->ctx) {
            dev_err(&pdev->dev, "DEBUG:SCK: GPIO %d: gpio_acpi_setup() FAILED\n", i);
            dev_err(&pdev->dev, "Failed to setup ACPI context for GPIO %d\n", i);
            ret = -ENODEV;
            goto gpio_release;
        }
        dev_info(&pdev->dev, "DEBUG:SCK: GPIO %d: ACPI context OK - pin=%d, type=%s\n", 
                 i, app_ctx->ctx->pin,
                 app_ctx->ctx->connection_type == ACPI_RESOURCE_GPIO_TYPE_INT ? "GpioInt" : "GpioIo");

        /* Configure debouncing */
        gpiod_set_debounce(app_ctx->desc, app_ctx->ctx->debounce_timeout_us);

        /* Setup IRQ only for GpioInt resources (BOOT and PRSNT) */
        if (app_ctx->ctx->connection_type == ACPI_RESOURCE_GPIO_TYPE_INT) {
            dev_info(&pdev->dev, "DEBUG:SCK: GPIO %d: Is GpioInt, setting up IRQ...\n", i);
            dev_info(&pdev->dev, "Setting up IRQ for %s GPIO (pin %d)\n",
                     i == PCIE_PIN_BOOT ? "BOOT" :
                     i == PCIE_PIN_PRSNT ? "PRSNT" : "UNKNOWN",
                     app_ctx->ctx->pin);
            
            ret = pcie_hp_setup_irq(app_ctx);
            if (ret) {
                dev_err(&pdev->dev, "DEBUG:SCK: GPIO %d: pcie_hp_setup_irq() FAILED: %d\n", i, ret);
                dev_err(&pdev->dev, "Failed to setup IRQ for GPIO %d: %d\n", i, ret);
                goto gpio_release;
            }
            dev_info(&pdev->dev, "DEBUG:SCK: GPIO %d: IRQ setup SUCCESS - IRQ %d\n", i, gpiod_to_irq(app_ctx->desc));
            dev_info(&pdev->dev, "IRQ %d registered for GPIO %d\n", 
                     gpiod_to_irq(app_ctx->desc), i);
        } else {
            dev_info(&pdev->dev, "DEBUG:SCK: GPIO %d: Is GpioIo, no IRQ needed\n", i);
        }
    }
    dev_info(&pdev->dev, "DEBUG:SCK: ========================================\n");
    dev_info(&pdev->dev, "DEBUG:SCK: ACPI context and IRQ setup COMPLETE\n");
    dev_info(&pdev->dev, "DEBUG:SCK: ========================================\n");

    /* Check that we found the required pins */
    if (hp_dev->boot_pin < 0 || hp_dev->prsnt_pin < 0) {
        dev_warn(&pdev->dev,
                 "Warning: boot_pin=%d prsnt_pin=%d (may not work correctly)\n",
                 hp_dev->boot_pin, hp_dev->prsnt_pin);
    }

    platform_set_drvdata(pdev, hp_dev);

    /* Initialize pinctrl */
    ret = pcie_hp_pinctrl_init(hp_dev);
    if (ret) {
        dev_err(&pdev->dev, "Pinctrl init failed: %d\n", ret);
        goto gpio_release;
    }

    /* Create sysfs interface */
    ret = sysfs_create_group(&pdev->dev.kobj, &pcie_hp_attr_group);
    if (ret) {
        dev_err(&pdev->dev, "Sysfs creation failed: %d\n", ret);
        goto pinctrl_remove;
    }

    /* Map MMIO regions from platform resources */
    ret = pcie_hp_map_resources(hp_dev);
    if (ret) {
        dev_err(&pdev->dev, "Failed to map MMIO resources: %d\n", ret);
        goto sysfs_remove;
    }

    /* Discover existing PCI devices */
    ret = pcie_hp_discover_devices(hp_dev);
    if (ret)
        goto sysfs_remove;

    /* Send initial state uevent */
    dev_info(&pdev->dev, "DEBUG:SCK: Reading PRSNT pin for initial state...\n");
    if (gpiod_get_value(hp_dev->pins[PCIE_PIN_PRSNT].desc)) {
        dev_info(&pdev->dev, "DEBUG:SCK: PRSNT=1 (cable removed), sending REMOVAL_EVT\n");
        hp_dev->debug_state = PCIE_HP_DEBUG_PLUG_OUT;
        pcie_hp_send_uevent(hp_dev, REMOVAL_EVT);
    } else {
        dev_info(&pdev->dev, "DEBUG:SCK: PRSNT=0 (cable present), sending PLUG_IN_EVT\n");
        hp_dev->debug_state = PCIE_HP_DEBUG_PLUG_IN;
        pcie_hp_send_uevent(hp_dev, PLUG_IN_EVT);
    }

    dev_info(&pdev->dev, "DEBUG:SCK: ========================================\n");
    dev_info(&pdev->dev, "DEBUG:SCK: Driver Initialization Summary:\n");
    dev_info(&pdev->dev, "DEBUG:SCK:   GPIOs enumerated: %d (via ACPI _DSD)\n", hp_dev->gpio_count);
    dev_info(&pdev->dev, "DEBUG:SCK:   BOOT pin: %d (IRQ enabled)\n", hp_dev->boot_pin);
    dev_info(&pdev->dev, "DEBUG:SCK:   PRSNT pin: %d (IRQ enabled)\n", hp_dev->prsnt_pin);
    dev_info(&pdev->dev, "DEBUG:SCK:   MMIO regions: 5 (TOP, PROTECT, CKM, MAC0, MAC1)\n");
    dev_info(&pdev->dev, "DEBUG:SCK:   Initial state: %s\n", 
             hp_dev->debug_state == PCIE_HP_DEBUG_PLUG_OUT ? "PLUG_OUT" : "PLUG_IN");
    dev_info(&pdev->dev, "DEBUG:SCK: ========================================\n");
    dev_info(&pdev->dev, "PCIe hotplug driver initialized successfully\n");
    return 0;

sysfs_remove:
    sysfs_remove_group(&pdev->dev.kobj, &pcie_hp_attr_group);
pinctrl_remove:
    pcie_hp_pinctrl_remove(hp_dev);
gpio_release:
    for (i = 0; i < hp_dev->gpio_count; i++) {
        app_ctx = &hp_dev->pins[i];
        if (app_ctx->desc)
            gpiod_put(app_ctx->desc);
    }

    return ret;
}
 
static void pcie_hp_remove(struct platform_device *pdev)
{
    struct pcie_hp_dev *hp_dev = platform_get_drvdata(pdev);
    struct pcie_hp_gpio_ctx *app_ctx;
    int i;

    if (!hp_dev)
        return;

    /* Remove sysfs interface */
    sysfs_remove_group(&pdev->dev.kobj, &pcie_hp_attr_group);

    /* Remove pinctrl */
    pcie_hp_pinctrl_remove(hp_dev);

    /* Release GPIO pins */
    for (i = 0; i < hp_dev->gpio_count; i++) {
        app_ctx = &hp_dev->pins[i];
        if (app_ctx->desc)
            gpiod_put(app_ctx->desc);
    }

    /* Release cached PCI device references */
    for (i = 0; i < hp_dev->pd->port_nums; i++) {
        if (hp_dev->cached_root_ports[i])
            pci_dev_put(hp_dev->cached_root_ports[i]);
    }

    platform_set_drvdata(pdev, NULL);

    /* MMIO regions are automatically unmapped via devm_* */
}
 
/*
 * Platform data for MT8901 PCIe hotplug support
 * 
 * All MMIO regions must be provided via platform device resources.
 * MMIO resources should be defined in Device Tree or ACPI tables:
 *   Resource 0: TOP region    - PCIe top-level control registers
 *   Resource 1: PROTECT region - Bus protection control registers
 *   Resource 2: CKM region     - Clock management registers
 *   Resource 3+: MAC regions   - Per-port MAC control registers (one per port)
 *
 * This configuration defines:
 *   - PCIe port topology (domains/buses to manage)
 *   - Expected devices (vendor ID, device ID, count)
 *   - MMIO register offsets and bit positions
 *   - Platform-specific callbacks for bus protection
 *   - Pinctrl state mappings
 */
static const struct pcie_hp_plat_data mt8901_plat_data = {
    .port_nums = 2,
    .ports = {
        { .domain = 0, .bus = 0, .devfn = 0 },
        { .domain = 2, .bus = 0, .devfn = 0 },
    },
    .vendor_id = PCI_VENDOR_ID_MELLANOX,
    .device_id = 0x1021, /* CX7 device ID */
    .num_devices = 4,
    .rp_bus_mmio = {
        .top = {
            .ctrl = 0x400,
            .update_bit = BIT(24),
            .port_bits = {BIT(6), BIT(2)},
        },
        .protect = {
            .mode = 0x38,
            .enable = 0x40,
            .port_bits = {BIT(20), BIT(16)},
        },
        .mac = {
            .init_ctrl = 0x008,
            .ltssm_bit = BIT(0),
            .phy_rst_bit = BIT(8),
        },
        .ckm = {
            .ctrl = 0xa8,
            .disable_bit = BIT(5) | BIT(7),
        },
    },
    .rp_bus_protect = mt8901_rp_bus_protect,
    .ltssm_reg = 0x728,
    .ltssm_l0_state = 0x11,
    .pin_nums = 4,
    .pinmap = {
        PIN_MAP_MUX_GROUP("MTKP0001:00", "default", "NVDA9221:00", "GPIO177", "func0"),
        PIN_MAP_MUX_GROUP("MTKP0001:00", "default", "NVDA9221:00", "GPIO178", "func0"),
        PIN_MAP_MUX_GROUP("MTKP0001:00", "clkreqn", "NVDA9221:00", "GPIO177", "func1"),
        PIN_MAP_MUX_GROUP("MTKP0001:00", "clkreqn", "NVDA9221:00", "GPIO178", "func1"),
    }
};

static const struct acpi_device_id pcie_hp_acpi_match[] = {
    {"MTKP0001", (kernel_ulong_t)&mt8901_plat_data},
    {}
};

MODULE_DEVICE_TABLE(acpi, pcie_hp_acpi_match);

static struct platform_driver pcie_hp_driver = {
    .probe = pcie_hp_probe,
    .remove = pcie_hp_remove,
    .driver = {
        .name = "mtk-pcie-hotplug",
        .acpi_match_table = ACPI_PTR(pcie_hp_acpi_match),
    },
};

module_platform_driver(pcie_hp_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek PCIe Hotplug Driver for NVIDIA DGX Systems");
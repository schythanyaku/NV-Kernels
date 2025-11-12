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
 * stage 1: disable pcie ltssm
 * stage 2: set bus protection and enable pcie ltssm
 */
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
    struct pci_dev *root_port;
    void __iomem *mac_base;
};

/* MMIO resource indices - for platform_get_resource() */
enum pcie_hp_resource_idx {
    PCIE_HP_RES_TOP = 0,
    PCIE_HP_RES_PROTECT,
    PCIE_HP_RES_CKM,
    PCIE_HP_RES_MAC_BASE,  /* First MAC resource, others follow */
};

/* Helper structure for MMIO region mapping */
struct mmio_region_map {
    const char *name;
    unsigned int res_idx;
    void __iomem **base_ptr;
};

struct rp_bus_mmio_top {
    void __iomem *base;
    u32 ctrl;
    u32 port_bits[HP_PORT_MAX];
    u32 update_bit;
};

struct rp_bus_mmio_protect {
    void __iomem *base;
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
    void __iomem *base;
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

enum pcie_hp_debug_val {
    PCIE_HP_DEBUG_PLUG_OUT = 0,
    PCIE_HP_DEBUG_PLUG_IN,
    PCIE_HP_DEBUG_MAX_VAL
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
    struct mutex lock; /* Protect state changes */
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
 * @mmio_info: MMIO configuration
 * @port_idx: Port index
 */
static void pcie_hp_bus_protect_enable(struct rp_bus_mmio_info *mmio_info,
                                         int port_idx)
{
    u32 port_bit = mmio_info->protect.port_bits[port_idx];
    
    pcie_hp_reg_update_bits(mmio_info->protect.base,
                             mmio_info->protect.mode, port_bit, true);
    pcie_hp_reg_update_bits(mmio_info->protect.base,
                             mmio_info->protect.enable, port_bit, true);
}

/**
 * pcie_hp_bus_protect_disable - Disable bus protection for a port
 * @mmio_info: MMIO configuration
 * @port_idx: Port index
 */
static void pcie_hp_bus_protect_disable(struct rp_bus_mmio_info *mmio_info,
                                          int port_idx)
{
    u32 port_bit = mmio_info->protect.port_bits[port_idx];
    
    pcie_hp_reg_update_bits(mmio_info->protect.base,
                             mmio_info->protect.enable, port_bit, false);
    pcie_hp_reg_update_bits(mmio_info->protect.base,
                             mmio_info->protect.mode, port_bit, false);
}

static void pcie_hp_ckm_control(struct pcie_hp_dev *dev, bool disable)
{
    struct rp_bus_mmio_info *mmio_info = &dev->pd->rp_bus_mmio;
    
    if (!mmio_info->ckm.base)
        return;
    
    pcie_hp_reg_update_bits(mmio_info->ckm.base, mmio_info->ckm.ctrl,
                             mmio_info->ckm.disable_bit, disable);
}

/**
 * pcie_hp_map_single_region - Map a single MMIO region
 * @pdev: platform device
 * @map: region mapping descriptor
 *
 * Returns: 0 on success, negative error code on failure
 */
static int pcie_hp_map_single_region(struct platform_device *pdev,
                                       const struct mmio_region_map *map)
{
    struct resource *res;
    void __iomem *base;

    res = platform_get_resource(pdev, IORESOURCE_MEM, map->res_idx);
    if (!res)
        return 0; /* Optional resource */

    base = devm_ioremap_resource(&pdev->dev, res);
    if (IS_ERR(base)) {
        dev_err(&pdev->dev, "Failed to map %s region\n", map->name);
        return PTR_ERR(base);
    }

    *map->base_ptr = base;
    dev_dbg(&pdev->dev, "Mapped %s region at %pR\n", map->name, res);
    return 0;
}

/**
 * pcie_hp_map_resources - Map all MMIO regions from platform resources
 * @dev: hotplug device
 *
 * All MMIO addresses must be provided via platform device resources.
 * Resources are automatically unmapped via devm_* management.
 *
 * Returns: 0 on success, negative error code on failure
 */
static int pcie_hp_map_resources(struct pcie_hp_dev *dev)
{
    struct rp_bus_mmio_info *mmio = &dev->pd->rp_bus_mmio;
    struct platform_device *pdev = dev->pdev;
    const struct mmio_region_map regions[] = {
        { "TOP", PCIE_HP_RES_TOP, &mmio->top.base },
        { "PROTECT", PCIE_HP_RES_PROTECT, &mmio->protect.base },
        { "CKM", PCIE_HP_RES_CKM, &mmio->ckm.base },
    };
    int i, ret;

    /* Map control regions */
    for (i = 0; i < ARRAY_SIZE(regions); i++) {
        ret = pcie_hp_map_single_region(pdev, &regions[i]);
        if (ret)
            return ret;
    }

    /* Map MAC regions (one per port) */
    for (i = 0; i < dev->pd->port_nums; i++) {
        struct mmio_region_map mac_map = {
            .name = "MAC",
            .res_idx = PCIE_HP_RES_MAC_BASE + i,
            .base_ptr = &dev->pd->ports[i].mac_base,
        };
        ret = pcie_hp_map_single_region(pdev, &mac_map);
        if (ret)
            return ret;
    }

    return 0;
}
 
static void mt8901_rp_bus_protect(struct pcie_hp_dev *dev, int port_idx, int stage)
{
    struct rp_bus_mmio_info *mmio_info = &dev->pd->rp_bus_mmio;
    void __iomem *mac_base;

    if (port_idx >= dev->pd->port_nums)
        return;

    mac_base = dev->pd->ports[port_idx].mac_base;
    if (!mac_base)
        return;

    if (stage == BUS_PROTECT_CABLE_REMOVAL) {
        /* Deassert LTSSM enable and PHY reset */
        pcie_hp_reg_update_bits(mac_base, mmio_info->mac.init_ctrl,
                                 mmio_info->mac.ltssm_bit, false);
        pcie_hp_reg_update_bits(mac_base, mmio_info->mac.init_ctrl,
                                 mmio_info->mac.phy_rst_bit, false);
    }

    if (stage == BUS_PROTECT_CABLE_PLUGIN) {
        if (!mmio_info->top.base || !mmio_info->protect.base)
            return;

        /* Deassert way_en */
        pcie_hp_toggle_update_bit(mmio_info->top.base, mmio_info->top.ctrl,
                                   mmio_info->top.port_bits[port_idx],
                                   mmio_info->top.update_bit, false);
        udelay(PCIE_HP_DELAY_SHORT_US);

        /* Enable bus protection */
        pcie_hp_bus_protect_enable(mmio_info, port_idx);
        usleep_range(PCIE_HP_DELAY_BUS_PROTECT_US, PCIE_HP_DELAY_BUS_PROTECT_US + 1000);

        /* Assert LTSSM enable and PHY reset */
        pcie_hp_reg_update_bits(mac_base, mmio_info->mac.init_ctrl,
                                 mmio_info->mac.phy_rst_bit, true);
        pcie_hp_reg_update_bits(mac_base, mmio_info->mac.init_ctrl,
                                 mmio_info->mac.ltssm_bit, true);
        usleep_range(PCIE_HP_DELAY_PHY_RESET_US, PCIE_HP_DELAY_PHY_RESET_US + 1000);

        /* Disable bus protection */
        pcie_hp_bus_protect_disable(mmio_info, port_idx);

        /* Assert way_en */
        pcie_hp_toggle_update_bit(mmio_info->top.base, mmio_info->top.ctrl,
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
    if (!port->root_port) {
        port->root_port = pci_get_domain_bus_and_slot(port->domain,
                                                        port->bus,
                                                        port->devfn);
        if (!port->root_port) {
            dev_warn(&hp_dev->pdev->dev,
                     "Root port not found for domain %d bus %d\n",
                     port->domain, port->bus);
            return NULL;
        }
    }

    return port->root_port;
}

static void remove_device(struct pcie_hp_dev *dev)
{
    int i;

    /* Apply bus protection before removal */
    if (dev->pd->rp_bus_protect) {
        for (i = 0; i < dev->pd->port_nums; i++)
            dev->pd->rp_bus_protect(dev, i, BUS_PROTECT_CABLE_REMOVAL);
    }

    /* Deassert PCIe reset */
    gpiod_set_value(dev->pins[PCIE_PIN_PERST].desc, 0);

    /* Change pinctrl state */
    pcie_hp_change_state(dev, "default");

    /* Disable clock */
    pcie_hp_ckm_control(dev, true);

    /* Power off device */
    gpiod_set_value(dev->pins[PCIE_PIN_EN].desc, 0);
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
    struct pci_bus *bus;
    int i, err;

    /* Change pinctrl state */
    err = pcie_hp_change_state(dev, "clkreqn");
    if (err)
        return err;

    /* Enable clock */
    pcie_hp_ckm_control(dev, false);
    usleep_range(PCIE_HP_DELAY_STANDARD_US, PCIE_HP_DELAY_STANDARD_US + 1000);

    /* Resume root ports */
    for (i = 0; i < dev->pd->port_nums; i++) {
        pci_dev = get_port_root_port(dev, i);
        if (!pci_dev)
            continue;

        err = pm_runtime_resume_and_get(&pci_dev->dev);
        if (err < 0) {
            dev_err(&dev->pdev->dev,
                    "Runtime resume failed for %s: %d\n",
                    pci_name(pci_dev), err);
        }
    }

    /* Assert PCIe reset */
    gpiod_set_value(dev->pins[PCIE_PIN_PERST].desc, 1);

    /* Apply bus protection */
    if (dev->pd->rp_bus_protect) {
        for (i = 0; i < dev->pd->port_nums; i++)
            dev->pd->rp_bus_protect(dev, i, BUS_PROTECT_CABLE_PLUGIN);
    }

    /* Wait for links to reach L0 */
    err = polling_link_to_l0(dev);
    if (err)
        return err;

    /* Retrain PCIe links */
    for (i = 0; i < dev->pd->port_nums; i++) {
        pci_dev = get_port_root_port(dev, i);
        if (pci_dev)
            retrain_pcie_link(pci_dev);
    }

    /* Wait for link stability */
    msleep(PCIE_HP_DELAY_LINK_STABLE_MS);

    /* Rescan buses */
    for (i = 0; i < dev->pd->port_nums; i++) {
        bus = pci_find_bus(dev->pd->ports[i].domain, dev->pd->ports[i].bus);
        if (bus) {
            pci_lock_rescan_remove();
            pci_rescan_bus(bus);
            pci_unlock_rescan_remove();
        }
    }

    return 0;
}
 
static irqreturn_t pcie_hp_work(int irq, void *dev_id)
{
    struct pcie_hp_gpio_ctx *app_ctx = dev_id;
    struct pcie_hp_dev *hp_dev = app_ctx->hp_dev;
    enum pcie_hp_state state;
    int ret;

    mutex_lock(&hp_dev->lock);
    state = hp_dev->state;

    switch (state) {
    case STATE_PLUG_OUT:
        dev_dbg(app_ctx->ctx->dev, "Cable plug out\n");
        remove_device(hp_dev);
        break;
    case STATE_PLUG_IN:
        dev_dbg(app_ctx->ctx->dev, "Enable device power\n");
        gpiod_set_value(hp_dev->pins[PCIE_PIN_EN].desc, 1);
        break;
    case STATE_DEV_POWER_OFF:
    case STATE_DEV_POWER_ON:
    case STATE_DEV_FW_START:
        dev_dbg(app_ctx->ctx->dev, "Waiting for device to be ready\n");
        break;
    case STATE_RESCAN:
        dev_dbg(app_ctx->ctx->dev, "Cable plug in, rescanning\n");
        ret = rescan_device(hp_dev);
        if (ret)
            dev_err(app_ctx->ctx->dev, "Rescan failed: %d\n", ret);
        else
            hp_dev->state = STATE_READY;
        break;
    default:
        dev_err(app_ctx->ctx->dev, "Unknown state: %d\n", state);
        break;
    }

    mutex_unlock(&hp_dev->lock);
    return IRQ_HANDLED;
}

static irqreturn_t hotplug_irq_handler(int irq, void *dev_id)
{
    struct pcie_hp_gpio_ctx *app_ctx = dev_id;
    struct pcie_hp_dev *hp_dev = app_ctx->hp_dev;
    struct gpio_acpi_context *gpio_ctx = app_ctx->ctx;
    int value;
    enum pcie_hp_state state;

    value = gpiod_get_value(app_ctx->desc);

    mutex_lock(&hp_dev->lock);
    state = hp_dev->state;

    /* Handle presence pin events */
    if (gpio_ctx->pin == hp_dev->prsnt_pin) {
        if (value) {
            dev_dbg(gpio_ctx->dev, "Presence pin: cable removal detected\n");
            pcie_hp_send_uevent(hp_dev, REMOVAL_EVT);
        } else {
            dev_dbg(gpio_ctx->dev, "Presence pin: cable plug-in detected\n");
            pcie_hp_send_uevent(hp_dev, PLUG_IN_EVT);
        }
        mutex_unlock(&hp_dev->lock);
        /* For physical hotplug, let userspace handle device management
         * to avoid potential deadlocks when hardware is in transition */
        return IRQ_HANDLED;
    }

    /* Handle boot status pin events */
    if (gpio_ctx->pin == hp_dev->boot_pin) {
        if (value && state == STATE_PLUG_IN) {
            dev_dbg(gpio_ctx->dev, "Boot pin high: device powered on\n");
            hp_dev->state = STATE_DEV_POWER_ON;
        } else if (value && state == STATE_DEV_FW_START) {
            dev_dbg(gpio_ctx->dev, "Boot pin high: device ready\n");
            hp_dev->state = STATE_RESCAN;
        } else if (!value && state == STATE_DEV_POWER_ON) {
            dev_dbg(gpio_ctx->dev, "Boot pin low: firmware starting\n");
            hp_dev->state = STATE_DEV_FW_START;
        } else if (!value && state == STATE_PLUG_OUT) {
            dev_dbg(gpio_ctx->dev, "Boot pin low: device powered off\n");
            hp_dev->state = STATE_DEV_POWER_OFF;
        } else {
            dev_dbg(gpio_ctx->dev, "Boot pin changed: pin=%d irq=%d value=%d state=%d\n",
                    gpio_ctx->pin, irq, value, state);
            mutex_unlock(&hp_dev->lock);
            return IRQ_HANDLED;
        }
        mutex_unlock(&hp_dev->lock);
        return IRQ_WAKE_THREAD;
    }

    /* Unknown GPIO pin */
    dev_warn(gpio_ctx->dev, "Unknown GPIO pin event: pin=%d irq=%d value=%d\n",
             gpio_ctx->pin, irq, value);
    mutex_unlock(&hp_dev->lock);
    return IRQ_HANDLED;
}
 
struct acpi_gpio_parse_context {
    struct gpio_acpi_context *ctx;
    struct pcie_hp_dev *hp_dev;
};

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
    struct pci_bus *bus;
    unsigned long val;
    int err, i;

    if (!hp_dev)
        return -EINVAL;

    err = kstrtoul(buf, 10, &val);
    if (err)
        return err;

    mutex_lock(&hp_dev->lock);

    switch (val) {
    case PCIE_HP_DEBUG_PLUG_OUT:
        dev_info(dev, "Debug: simulating cable removal\n");
        /* NOTE: Userspace must remove PCI devices BEFORE writing to debug_state.
         * This avoids kernel locking issues and follows the proven approach
         * from the legacy implementation. The kernel driver only manages
         * hardware (GPIO, power, PCIe link). */
        hp_dev->state = STATE_PLUG_OUT;
        remove_device(hp_dev);
        break;

    case PCIE_HP_DEBUG_PLUG_IN:
        dev_info(dev, "Debug: simulating cable plug-in\n");
        hp_dev->state = STATE_PLUG_IN;
        gpiod_set_value(hp_dev->pins[PCIE_PIN_EN].desc, 1);
        break;

    default:
        mutex_unlock(&hp_dev->lock);
        return -EINVAL;
    }

    hp_dev->debug_state = val;
    mutex_unlock(&hp_dev->lock);

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

static int pcie_hp_probe_io_info(struct platform_device *pdev)
{
    struct gpio_desc *desc;
    int count = 0;

    for (;;) {
        desc = gpiod_get_index(&pdev->dev, NULL, count, GPIOD_ASIS);
        if (IS_ERR(desc))
            break;
        count++;
        gpiod_put(desc);
    }

    return count;
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
    mutex_init(&hp_dev->lock);

    /* Discover GPIO pins */
    hp_dev->gpio_count = pcie_hp_probe_io_info(pdev);
    if (!hp_dev->gpio_count) {
        dev_err(&pdev->dev, "No GPIO descriptors found\n");
        return -ENODEV;
    }

    hp_dev->pins = devm_kcalloc(&pdev->dev, hp_dev->gpio_count,
                                 sizeof(struct pcie_hp_gpio_ctx),
                                 GFP_KERNEL);
    if (!hp_dev->pins)
        return -ENOMEM;

    /* Setup GPIO pins and IRQs */
    for (i = 0; i < hp_dev->gpio_count; i++) {
        app_ctx = &hp_dev->pins[i];
        app_ctx->desc = gpiod_get_index(&pdev->dev, NULL, i, GPIOD_ASIS);
        if (IS_ERR(app_ctx->desc)) {
            dev_err(&pdev->dev, "Failed to get GPIO %d: %ld\n",
                    i, PTR_ERR(app_ctx->desc));
            ret = PTR_ERR(app_ctx->desc);
            app_ctx->desc = NULL;
            goto gpio_release;
        }

        app_ctx->ctx = gpio_acpi_setup(pdev, app_ctx->desc, hp_dev);
        if (!app_ctx->ctx) {
            dev_err(&pdev->dev, "Failed to setup GPIO %d\n", i);
            ret = -ENODEV;
            goto gpio_release;
        }

        gpiod_set_debounce(app_ctx->desc, app_ctx->ctx->debounce_timeout_us);

        /* Setup IRQ for interrupt-type GPIOs */
        if (app_ctx->ctx->connection_type == ACPI_RESOURCE_GPIO_TYPE_INT) {
            app_ctx->hp_dev = hp_dev;
            ret = pcie_hp_setup_irq(app_ctx);
            if (ret) {
                dev_err(&pdev->dev, "Failed to setup IRQ for GPIO %d\n", i);
                goto gpio_release;
            }
        }
    }

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
    if (gpiod_get_value(hp_dev->pins[PCIE_PIN_PRSNT].desc)) {
        hp_dev->debug_state = PCIE_HP_DEBUG_PLUG_OUT;
        pcie_hp_send_uevent(hp_dev, REMOVAL_EVT);
    } else {
        hp_dev->debug_state = PCIE_HP_DEBUG_PLUG_IN;
        pcie_hp_send_uevent(hp_dev, PLUG_IN_EVT);
    }

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
        if (hp_dev->pd->ports[i].root_port)
            pci_dev_put(hp_dev->pd->ports[i].root_port);
    }

    mutex_destroy(&hp_dev->lock);
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
MODULE_DESCRIPTION("MediaTek PCIe Hotplug Driver with GPIO Support");
MODULE_AUTHOR("MediaTek Inc.");
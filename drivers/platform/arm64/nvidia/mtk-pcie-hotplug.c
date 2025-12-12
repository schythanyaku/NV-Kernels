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
#include <linux/pinctrl/pinconf.h>
#include <linux/platform_device.h>
 #include <linux/pm_runtime.h>
 #include <linux/of.h>
 #include <linux/property.h>
 
 #define HP_PORT_MAX		3
 #define HP_POLL_CNT_MAX		200
 #define PCIE_REG_SIZE		0x1000
 #define MAX_VENDOR_DATA_LEN	16
 #define PCIE_HP_MMIO_REGION_COUNT	5	/* TOP, PROTECT, CKM, MAC Port 0, MAC Port 1 */
 #define PCIE_HP_MIN_GPIO_COUNT	4	/* Minimum required: BOOT, PRSNT, PERST, EN */
 
/* Hardware timing requirements (in microseconds unless noted) */
#define PCIE_HP_DELAY_SHORT_US		10	/* Short delay for register writes */
#define PCIE_HP_DELAY_STANDARD_US	10000	/* Standard delay (10ms) */
#define PCIE_HP_DELAY_BUS_PROTECT_US	5000	/* Bus protection setup delay */
#define PCIE_HP_DELAY_PHY_RESET_US	3000	/* PHY reset delay */
#define PCIE_HP_DELAY_LINK_STABLE_MS	100	/* Link stabilization delay (ms) */
#define PCIE_HP_POLL_SLEEP_US		10000	/* Polling loop sleep interval */

/* BOOT_COMP polling timeouts (in milliseconds)
 * Note: These are initial values based on recommended approach examples.
 * Actual firmware boot timing may vary - adjust based on hardware testing.
 * 
 * low_wait: Device should start driving BOOT_COMP signal quickly after power-on.
 *           If pull-up masking is present, we want fast failure detection.
 * high_wait: Firmware boot time varies. This timeout should cover typical boot
 *            duration while preventing indefinite waits.
 * debounce: Short stability check to prevent false positives from glitches.
 */
#define BOOT_LOW_WAIT_TIMEOUT_MS	2000	/* Timeout to see BOOT go low after power enable */
#define BOOT_HIGH_WAIT_TIMEOUT_MS	10000	/* Timeout to see BOOT go high after seeing low */
#define BOOT_DEBOUNCE_HIGH_MS		10	/* Debounce window for stable high (5-10ms) */
 
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
 #define BUS_PROTECT_CLEANUP		3	/* Cleanup stage (for remove) */
 
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
     u32 addr;		/* MMIO base address */
     u32 ctrl;
     u32 port_bits[HP_PORT_MAX];
     u32 update_bit;
 };
 
 struct rp_bus_mmio_protect {
     u32 addr;		/* MMIO base address */
     u32 mode;
     u32 enable;
     u32 port_bits[HP_PORT_MAX];
 };
 
 struct rp_bus_mmio_mac {
     u32 addr[HP_PORT_MAX];	/* MMIO base addresses per port */
     u32 init_ctrl;
     u32 ltssm_bit;
     u32 phy_rst_bit;
 };
 
 struct rp_bus_mmio_ckm {
     u32 addr;		/* MMIO base address */
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
    /* GPIO pin number for EN (fallback for kernel 6.17 GPIO grouping workaround) */
    unsigned int gpio_pin_en;
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

/* Structure to collect all GPIO resources from ACPI walk */
struct acpi_gpio_walk_context {
    struct device *dev;
    struct gpio_info {
        unsigned int pin;              /* Hardware pin number (chip-relative) */
        unsigned int connection_type;  /* ACPI_RESOURCE_GPIO_TYPE_INT or ACPI_RESOURCE_GPIO_TYPE_IO */
        unsigned int triggering;        /* ACPI_EDGE_SENSITIVE or ACPI_LEVEL_SENSITIVE */
        unsigned int polarity;         /* ACPI_ACTIVE_HIGH or ACPI_ACTIVE_LOW */
        unsigned int debounce_timeout; /* Debounce timeout in 10ms units */
        unsigned int wake_capable;     /* Wake capability */
        char vendor_data[MAX_VENDOR_DATA_LEN + 1]; /* Vendor data string */
        char resource_source[16];      /* GPIO controller name (e.g., "\\_SB.GIO0") */
        unsigned int resource_source_index; /* Resource source index */
    } gpios[PCIE_PIN_MAX];
    int count;                          /* Number of GPIOs found */
    unsigned int gpio_chip_base;        /* GPIO chip base (to be determined) */
    bool found_chip_base;               /* Whether chip base was found */
};
 
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
     struct acpi_resource_fixed_memory32 mmio_regions[PCIE_HP_MMIO_REGION_COUNT];
     int count;
     struct device *dev;
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
     spinlock_t lock; /* Protect state changes (IRQ-safe) */
     struct pci_dev *cached_root_ports[HP_PORT_MAX]; /* Cached root port pointers */
     struct pcie_hp_mmio_runtime mmio; /* Runtime mapped MMIO base addresses */
     struct gpio_device *gdev; /* Cached GPIO device for ACPI walk path */
 };
 
 /**
  * pcie_hp_pinctrl_init - Register pinctrl mappings for the device
  * @hp_dev: hotplug device
  *
  * Registers platform-specific pinctrl mappings for GPIO pin multiplexing
  * states (default, clkreqn) for the MediaTek PCIe hotplug controller.
  * These mappings are SoC-specific and define pin names and mux functions.
  *
  * Returns: 0 on success, negative error code on failure
  */
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
 
     /* devm_pinctrl_get() prefers ACPI/DT-provided pinctrl if available,
      * otherwise falls back to hardcoded mappings registered in pcie_hp_pinctrl_init() */
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
         if (parsed->count >= PCIE_HP_MMIO_REGION_COUNT) {
             dev_warn(parsed->dev, "More than %d MMIO regions found, ignoring extras\n",
                  PCIE_HP_MMIO_REGION_COUNT);
             break;
         }
         /* Store the Memory32Fixed resource */
         parsed->mmio_regions[parsed->count] = ares->data.fixed_memory32;
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
  * pcie_hp_map_platform_mmio - Map MMIO regions using platform data addresses
  * @dev: hotplug device
  *
  * Fallback function to map MMIO regions using SoC-specific addresses from
  * platform data when ACPI _CRS resources are not available. These addresses
  * are defined in the platform data structure and are fixed for MediaTek MT8901 SoC.
  *
  * Returns: 0 on success, negative error code on failure
  */
 static int pcie_hp_map_platform_mmio(struct pcie_hp_dev *dev)
 {
     struct platform_device *pdev = dev->pdev;
     struct rp_bus_mmio_info *mmio_info = &dev->pd->rp_bus_mmio;
     void __iomem *base;
     int i;
 
     dev_warn(&pdev->dev, "ACPI _CRS not available, using SoC-specific MMIO addresses from platform data\n");
     dev_info(&pdev->dev, "Mapping MMIO regions using platform data addresses...\n");
 
     /* Map TOP region */
     if (mmio_info->top.addr) {
         base = devm_ioremap(&pdev->dev, mmio_info->top.addr, PCIE_REG_SIZE);
         if (!base) {
             dev_err(&pdev->dev, "Failed to map TOP region (0x%08x)\n",
                 mmio_info->top.addr);
             return -ENOMEM;
         }
         dev->mmio.top_base = base;
         dev_info(&pdev->dev, "Mapped TOP: 0x%08x -> %p\n", mmio_info->top.addr, base);
     }
 
     /* Map PROTECT region */
     if (mmio_info->protect.addr) {
         base = devm_ioremap(&pdev->dev, mmio_info->protect.addr, PCIE_REG_SIZE);
         if (!base) {
             dev_err(&pdev->dev, "Failed to map PROTECT region (0x%08x)\n",
                 mmio_info->protect.addr);
             return -ENOMEM;
         }
         dev->mmio.protect_base = base;
         dev_info(&pdev->dev, "Mapped PROTECT: 0x%08x -> %p\n", mmio_info->protect.addr, base);
     }
 
     /* Map CKM region */
     if (mmio_info->ckm.addr) {
         base = devm_ioremap(&pdev->dev, mmio_info->ckm.addr, PCIE_REG_SIZE);
         if (!base) {
             dev_err(&pdev->dev, "Failed to map CKM region (0x%08x)\n",
                 mmio_info->ckm.addr);
             return -ENOMEM;
         }
         dev->mmio.ckm_base = base;
         dev_info(&pdev->dev, "Mapped CKM: 0x%08x -> %p\n", mmio_info->ckm.addr, base);
     }
 
     /* Map MAC port regions */
     for (i = 0; i < dev->pd->port_nums && i < HP_PORT_MAX; i++) {
         if (mmio_info->mac.addr[i]) {
             base = devm_ioremap(&pdev->dev, mmio_info->mac.addr[i], PCIE_REG_SIZE);
             if (!base) {
                 dev_err(&pdev->dev, "Failed to map MAC Port %d region (0x%08x)\n",
                     i, mmio_info->mac.addr[i]);
                 return -ENOMEM;
             }
             dev->mmio.mac_port_base[i] = base;
             dev_info(&pdev->dev, "Mapped MAC Port %d: 0x%08x -> %p\n",
                  i, mmio_info->mac.addr[i], base);
         }
     }
 
     dev_info(&pdev->dev, "Successfully mapped all MMIO regions using platform data addresses\n");
     return 0;
 }
 
 /**
  * pcie_hp_map_resources - Map all MMIO regions from ACPI _CRS
  * @dev: hotplug device
  *
  * Uses acpi_walk_resources() to parse Memory32Fixed entries in ACPI _CRS,
  * then maps each region using devm_ioremap(). This is the upstream-friendly
  * method for ARM64 ACPI platforms where platform_get_resource() may not work.
  *
  * Expected MMIO regions in _CRS:
  *   - TOP region (PCIe control) - typically 0x1d600000
  *   - PROTECT region (bus protection) - typically 0x1d640000
  *   - CKM region (clock management) - typically 0x16bd0000
  *   - MAC Port 0 (per-port MAC) - typically 0x1d790000
  *   - MAC Port 1 (per-port MAC) - typically 0x1d690000
  *
  * Regions are matched by address (comparing with platform data addresses)
  * rather than index, as ACPI doesn't guarantee resource order.
  *
  * Falls back to hardcoded addresses if ACPI is unavailable or parsing fails.
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
 
     dev_info(&pdev->dev, "Parsing MMIO regions from ACPI _CRS\n");
 
     /* Prefer ACPI-provided MMIO addresses if available, otherwise fall back
      * to hardcoded SoC-specific addresses.*/
     adev = ACPI_COMPANION(&pdev->dev);
     if (!adev) {
         dev_warn(&pdev->dev, "No ACPI companion device found, falling back to hardcoded addresses\n");
         return pcie_hp_map_platform_mmio(dev);
     }
 
     /* Walk through ACPI _CRS resources to find Memory32Fixed entries */
     status = acpi_walk_resources(adev->handle, METHOD_NAME__CRS,
                       pcie_hp_parse_acpi_resources, &parsed);
     if (ACPI_FAILURE(status)) {
         dev_warn(&pdev->dev, "Failed to walk ACPI resources: %s, falling back to hardcoded addresses\n",
              acpi_format_exception(status));
         return pcie_hp_map_platform_mmio(dev);
     }
 
     /* Verify we found all required MMIO regions */
     if (parsed.count < PCIE_HP_MMIO_REGION_COUNT) {
         dev_warn(&pdev->dev, "Expected %d MMIO regions, found %d, falling back to hardcoded addresses\n",
              PCIE_HP_MMIO_REGION_COUNT, parsed.count);
         return pcie_hp_map_platform_mmio(dev);
     }
 
     dev_info(&pdev->dev, "Found %d MMIO regions in _CRS, mapping...\n", parsed.count);
 
     /* Map each MMIO region using the addresses from ACPI.
      * Match regions by address (comparing with platform data addresses) rather than
      * index. While firmware may follow conventions, the ACPI spec doesn't guarantee
      * resource order, so address matching is more robust. This approach is acceptable
      * for SoC-specific drivers where MMIO addresses are fixed hardware addresses.
      * If ACPI provides addresses that don't match platform data, we fall back
      * to platform data addresses immediately (which are the authoritative SoC-specific values). */
     int mapped_count = 0;
     for (i = 0; i < parsed.count; i++) {
         void __iomem *base;
         u32 addr = parsed.mmio_regions[i].address;
         u32 size = parsed.mmio_regions[i].address_length;
         const char *name = "Unknown";
         bool mapped = false;
 
         /* Match region by address (compare with platform data addresses) */
         if (mmio->top.addr && addr == mmio->top.addr) {
             name = "TOP";
             base = devm_ioremap(&pdev->dev, addr, size);
             if (!base) {
                 dev_err(&pdev->dev, "Failed to map %s region (0x%08x)\n", name, addr);
                 return -ENOMEM;
             }
             dev->mmio.top_base = base;
             mapped = true;
         } else if (mmio->protect.addr && addr == mmio->protect.addr) {
             name = "PROTECT";
             base = devm_ioremap(&pdev->dev, addr, size);
             if (!base) {
                 dev_err(&pdev->dev, "Failed to map %s region (0x%08x)\n", name, addr);
                 return -ENOMEM;
             }
             dev->mmio.protect_base = base;
             mapped = true;
         } else if (mmio->ckm.addr && addr == mmio->ckm.addr) {
             name = "CKM";
             base = devm_ioremap(&pdev->dev, addr, size);
             if (!base) {
                 dev_err(&pdev->dev, "Failed to map %s region (0x%08x)\n", name, addr);
                 return -ENOMEM;
             }
             dev->mmio.ckm_base = base;
             mapped = true;
         } else if (dev->pd->port_nums >= 1 && mmio->mac.addr[0] && addr == mmio->mac.addr[0]) {
             name = "MAC Port 0";
             base = devm_ioremap(&pdev->dev, addr, size);
             if (!base) {
                 dev_err(&pdev->dev, "Failed to map %s region (0x%08x)\n", name, addr);
                 return -ENOMEM;
             }
             dev->mmio.mac_port_base[0] = base;
             mapped = true;
         } else if (dev->pd->port_nums >= 2 && mmio->mac.addr[1] && addr == mmio->mac.addr[1]) {
             name = "MAC Port 1";
             base = devm_ioremap(&pdev->dev, addr, size);
             if (!base) {
                 dev_err(&pdev->dev, "Failed to map %s region (0x%08x)\n", name, addr);
                 return -ENOMEM;
             }
             dev->mmio.mac_port_base[1] = base;
             mapped = true;
         }
 
         if (mapped) {
             mapped_count++;
             dev_info(&pdev->dev, "Mapped %s: 0x%08x (size 0x%x) -> %p\n",
                  name, addr, size, base);
         } else {
             /* ACPI provided an address that doesn't match platform data.
              * This indicates ACPI may be incomplete or incorrect, so fall back
              * to platform data addresses immediately. */
             dev_warn(&pdev->dev, "Unknown MMIO region at 0x%08x (size 0x%x) in ACPI, falling back to platform data addresses\n",
                  addr, size);
             /* Clear any partially mapped regions (devm_ioremap will clean up automatically) */
             dev->mmio.top_base = NULL;
             dev->mmio.protect_base = NULL;
             dev->mmio.ckm_base = NULL;
             for (i = 0; i < HP_PORT_MAX; i++)
                 dev->mmio.mac_port_base[i] = NULL;
             return pcie_hp_map_platform_mmio(dev);
         }
     }
 
     /* Verify all required regions were successfully mapped.
      * We explicitly check critical regions (TOP, PROTECT, CKM) are mapped,
      * as these are essential for driver operation. MAC ports are optional
      * based on port_nums, but if ACPI provides them, they should be mapped. */
     if (!dev->mmio.top_base || !dev->mmio.protect_base || !dev->mmio.ckm_base) {
         dev_warn(&pdev->dev, "Not all required MMIO regions mapped from ACPI (mapped %d), falling back to platform data addresses\n",
              mapped_count);
         /* Clear any partially mapped regions (devm_ioremap will clean up automatically) */
         dev->mmio.top_base = NULL;
         dev->mmio.protect_base = NULL;
         dev->mmio.ckm_base = NULL;
         for (i = 0; i < HP_PORT_MAX; i++)
             dev->mmio.mac_port_base[i] = NULL;
         return pcie_hp_map_platform_mmio(dev);
     }
 
     dev_info(&pdev->dev, "Successfully mapped all MMIO regions from ACPI _CRS\n");
     return 0;
 }
 
 static void mt8901_rp_bus_protect(struct pcie_hp_dev *dev, int port_idx, int stage)
 {
     switch (stage) {
     case BUS_PROTECT_INIT:
         /* Initialize bus protection during probe - map MMIO regions (like 6.14's rp_bus_prepare) */
         {
             int ret;
 
             /* Map MMIO regions if not already mapped (check critical regions) */
             if (!dev->mmio.top_base || !dev->mmio.protect_base || !dev->mmio.ckm_base) {
                 ret = pcie_hp_map_resources(dev);
                 if (ret) {
                     dev_err(&dev->pdev->dev, "Failed to map MMIO resources during bus init: %d\n", ret);
                     return;
                 }
             }
         }
         return;
 
     case BUS_PROTECT_CLEANUP:
         /* Cleanup stage (called during remove) - matches 6.14's rp_bus_prepare(pdev, false).
          * Note: Both ACPI and platform data paths use devm_ioremap(), which automatically
          * cleans up, so no manual iounmap() is needed (and calling it would cause a
          * double-free bug). This cleanup clears pointers regardless of whether addresses
          * came from ACPI _CRS or platform data. */
         {
             int i;
 
             /* Clear MMIO base pointers (mappings are automatically cleaned up by devm_ioremap) */
             for (i = 0; i < HP_PORT_MAX; i++) {
                 if (dev->mmio.mac_port_base[i])
                     dev->mmio.mac_port_base[i] = NULL;
             }
             if (dev->mmio.top_base)
                 dev->mmio.top_base = NULL;
             if (dev->mmio.protect_base)
                 dev->mmio.protect_base = NULL;
             if (dev->mmio.ckm_base)
                 dev->mmio.ckm_base = NULL;
         }
         return;
 
     case BUS_PROTECT_CABLE_REMOVAL:
     case BUS_PROTECT_CABLE_PLUGIN:
         {
             struct rp_bus_mmio_info *mmio_info = &dev->pd->rp_bus_mmio;
             void __iomem *mac_base;
 
             if (port_idx >= dev->pd->port_nums)
                 return;
 
             mac_base = dev->mmio.mac_port_base[port_idx];
             if (!mac_base)
                 return;
 
             if (stage == BUS_PROTECT_CABLE_REMOVAL) {
                 /* Deassert LTSSM enable and PHY reset */
                 pcie_hp_reg_update_bits(mac_base, mmio_info->mac.init_ctrl,
                                          mmio_info->mac.ltssm_bit, false);
                 pcie_hp_reg_update_bits(mac_base, mmio_info->mac.init_ctrl,
                                          mmio_info->mac.phy_rst_bit, false);
                 return;
             }
 
             /* BUS_PROTECT_CABLE_PLUGIN */
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
         break;
 
     default:
         dev_warn(&dev->pdev->dev, "Unknown bus protect stage: %d\n", stage);
         break;
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
     u32 ltssm_vals[HP_PORT_MAX] = {0}; /* Stack array like 6.14's individual variables */
     int count = 0;
     int i;
     bool all_l0;
 
     if (!ltssm_reg || !l0_state)
         return 0; /* Skip if not configured */
 
     /* Poll until all ports reach L0 state (matching 6.14's while condition logic)
      * Initialize to ensure we enter the loop at least once */
     all_l0 = false;
     while (!all_l0) {
         all_l0 = true;
 
         for (i = 0; i < dev->pd->port_nums && i < HP_PORT_MAX; i++) {
             pci_dev = get_port_root_port(dev, i);
             if (!pci_dev) {
                 all_l0 = false;
                 continue;
             }
 
             pci_read_config_dword(pci_dev, ltssm_reg, &ltssm_vals[i]);
             if ((ltssm_vals[i] & l0_state) != l0_state)
                 all_l0 = false;
         }
 
         if (all_l0)
             break;
 
         usleep_range(PCIE_HP_POLL_SLEEP_US, PCIE_HP_POLL_SLEEP_US + 1000);
         count++;
 
         if (count > HP_POLL_CNT_MAX) {
             dev_err(&dev->pdev->dev, "Timeout waiting for link to reach L0 (reached max count)\n");
             break;
         }
     }
 
     if (count > HP_POLL_CNT_MAX) {
         return -ETIMEDOUT;
     }
 
     return 0;
 }
 
/**
 * verify_boot_transition_stable - Verify BOOT_COMP transition is stable with debounce
 * @dev: PCIe hotplug device
 * @expected_value: Expected value after transition (0 or 1)
 * @transition_name: Human-readable name for logging (e.g., "0→1", "1→0")
 *
 * Verifies that BOOT_COMP has transitioned to expected_value and stays stable
 * for a debounce period. This prevents false positives from glitches or pull-up/down.
 *
 * Returns: 0 if stable transition verified, -EIO if not stable
 */
static int verify_boot_transition_stable(struct pcie_hp_dev *dev, int expected_value,
                                         const char *transition_name)
{
    int i;
    int boot_value;
    const int DEBOUNCE_READS = 5; /* Require 5 consecutive reads */
    const int DEBOUNCE_DELAY_US = 2000; /* 2ms between reads = 10ms total */

    if (!dev->pins[PCIE_PIN_BOOT].desc) {
        dev_warn(&dev->pdev->dev, "BOOT pin not available, skipping stability check\n");
        return 0; /* Skip if boot pin not configured */
    }

    /* Verify BOOT_COMP stays at expected_value for debounce period */
    for (i = 0; i < DEBOUNCE_READS; i++) {
        boot_value = gpiod_get_value(dev->pins[PCIE_PIN_BOOT].desc);
        if (boot_value != expected_value) {
            dev_dbg(&dev->pdev->dev, "BOOT_COMP %s transition not stable "
                    "(read %d, expected %d at check %d)\n",
                    transition_name, boot_value, expected_value, i + 1);
            return -EIO;
        }
        if (i < DEBOUNCE_READS - 1) {
            usleep_range(DEBOUNCE_DELAY_US, DEBOUNCE_DELAY_US + 500);
        }
    }

    dev_info(&dev->pdev->dev, "BOOT_COMP GP77 %s transition stable "
            "(verified for %dms)\n",
            transition_name, DEBOUNCE_READS * DEBOUNCE_DELAY_US / 1000);
    return 0;
}

/**
 * configure_boot_gpio_bias - Configure BOOT_COMP GPIO bias to disable pull-up
 * @dev: PCIe hotplug device
 *
 * Attempts to reconfigure BOOT_COMP GPIO bias to disable pull-up/pull-down
 * at runtime. This is a fallback when DSDT has PullDefault=PullUp which masks
 * the true signal level.
 *
 * Returns: 0 on success (or if not supported), negative on error
 */
static int configure_boot_gpio_bias(struct pcie_hp_dev *dev)
{
    struct pinctrl *pinctrl;
    struct pinctrl_state *state;
    int ret;

    if (!dev->pins[PCIE_PIN_BOOT].desc) {
        return 0; /* Skip if boot pin not configured */
    }

    /* Try to get pinctrl for this GPIO */
    pinctrl = devm_pinctrl_get(&dev->pdev->dev);
    if (IS_ERR(pinctrl)) {
        dev_dbg(&dev->pdev->dev, "No pinctrl available for GPIO bias configuration\n");
        return 0; /* Not an error - pinctrl might not be available */
    }

    /* Try to configure bias-disable state */
    state = pinctrl_lookup_state(pinctrl, "boot-bias-disable");
    if (IS_ERR(state)) {
        /* Fallback: GPIO bias configuration via pinctrl might not be available
         * This is a best-effort attempt - if it fails, we'll proceed anyway
         * and rely on the timeout/edge detection logic */
        dev_dbg(&dev->pdev->dev, "GPIO bias configuration via pinctrl not available\n");
        ret = 0; /* Not an error - continue without bias configuration */
    } else {
        ret = pinctrl_select_state(pinctrl, state);
        if (ret) {
            /* Log loudly when bias reconfiguration fails - this is important for debugging */
            WARN_ONCE(ret, "mtk-pcie-hotplug: Failed to disable BOOT_COMP GPIO pull-up (ret=%d). "
                      "Phase-1 timeout may occur if GPIO is pulled up by hardware.\n", ret);
            dev_warn(&dev->pdev->dev, "Failed to select bias-disable state: %d\n", ret);
        } else {
            dev_info(&dev->pdev->dev, "BOOT_COMP GPIO bias disabled via pinctrl state\n");
        }
    }

    devm_pinctrl_put(pinctrl);
    return ret;
}

/**
 * polling_boot_complete - Poll for BOOT_COMP 0→1 transition (firmware ready)
 * @dev: PCIe hotplug device
 *
 * According to CX7 driver partition design, we MUST wait for BOOT_COMP GP77
 * to transition from 0 to 1 before proceeding with PCIe negotiation. This
 * indicates CX7 firmware has finished booting and is ready.
 *
 * Implements the recommended approach with separate timeouts for each phase:
 * 1. Phase 1 (low_wait): Wait up to BOOT_LOW_WAIT_TIMEOUT_MS to see BOOT go low
 *    - Fast failure (1-3s) if pull-up masking prevents seeing low
 * 2. Phase 2 (high_wait): After seeing low, wait up to BOOT_HIGH_WAIT_TIMEOUT_MS for 0→1
 *    - Longer timeout (5-15s) for firmware boot completion
 * 3. Phase 3 (debounce): Require stable high for BOOT_DEBOUNCE_HIGH_MS
 *    - Prevents false positives from glitches
 * 
 * If any timeout expires, abort bring-up and don't enumerate PCIe.
 *
 * If BOOT_COMP reads as 1 immediately after power-enable, attempts to disable
 * GPIO pull-up at runtime (fallback if DSDT can't be modified).
 *
 * Returns: 0 if 0→1 transition detected and stable, -ETIMEDOUT if timeout
 */
static int polling_boot_complete(struct pcie_hp_dev *dev)
{
    int boot_value;
    int stable_high_count = 0;
    bool seen_low_after_power = false;
    unsigned long low_wait_start_jiffies;
    unsigned long high_wait_start_jiffies = 0; /* Initialize to avoid uninitialized use */
    int debounce_reads;
    bool bias_configured = false;

    /* Calculate debounce reads - ensure no division by zero */
    if (PCIE_HP_POLL_SLEEP_US == 0) {
        dev_err(&dev->pdev->dev, "PCIE_HP_POLL_SLEEP_US is zero - invalid configuration\n");
        return -EINVAL;
    }
    debounce_reads = (BOOT_DEBOUNCE_HIGH_MS * 1000) / PCIE_HP_POLL_SLEEP_US;
    if (debounce_reads < 1) {
        debounce_reads = 1; /* Minimum 1 read for debounce */
    }

    dev_info(&dev->pdev->dev, "=== Starting BOOT_COMP polling (GP77: 0→1 transition) ===\n");
    dev_info(&dev->pdev->dev, "Timeout settings: low_wait=%dms, high_wait=%dms, debounce=%dms (%d reads)\n",
             BOOT_LOW_WAIT_TIMEOUT_MS, BOOT_HIGH_WAIT_TIMEOUT_MS, BOOT_DEBOUNCE_HIGH_MS, debounce_reads);

    /* Make GPIO bias explicit: If PullDefault=PullUp, attempt to disable pull-up */
    boot_value = gpiod_get_value(dev->pins[PCIE_PIN_BOOT].desc);
    dev_info(&dev->pdev->dev, "Initial BOOT_COMP read: %d (after power-on)\n", boot_value);
    
    if (boot_value == 1) {
        dev_info(&dev->pdev->dev, "BOOT_COMP reads HIGH initially - attempting to disable GPIO pull-up\n");
        configure_boot_gpio_bias(dev);
        bias_configured = true;
        /* Small delay to allow bias change to take effect */
        msleep(10);
        boot_value = gpiod_get_value(dev->pins[PCIE_PIN_BOOT].desc);
        dev_info(&dev->pdev->dev, "BOOT_COMP after bias config attempt: %d\n", boot_value);
        if (boot_value == 1) {
            dev_warn(&dev->pdev->dev, "BOOT_COMP still HIGH after bias configuration - pull-up may be hardware-level\n");
        }
    } else {
        dev_info(&dev->pdev->dev, "BOOT_COMP reads LOW initially - device is driving signal\n");
    }

    low_wait_start_jiffies = jiffies;
    dev_info(&dev->pdev->dev, "=== Phase 1: Waiting for BOOT_COMP to go LOW (timeout: %dms) ===\n",
             BOOT_LOW_WAIT_TIMEOUT_MS);

    /* Phase 1: low_wait_timeout (1-3s) - Force observation of BOOT=0 after device power-on */
    while (time_before(jiffies, low_wait_start_jiffies + msecs_to_jiffies(BOOT_LOW_WAIT_TIMEOUT_MS))) {
        boot_value = gpiod_get_value(dev->pins[PCIE_PIN_BOOT].desc);
        
        dev_dbg(&dev->pdev->dev, "Phase 1: BOOT_COMP=%d (elapsed: %lums)\n",
                boot_value, jiffies_to_msecs(jiffies - low_wait_start_jiffies));

        if (boot_value == 0) {
            seen_low_after_power = true;
            stable_high_count = 0;
            high_wait_start_jiffies = jiffies;
            dev_info(&dev->pdev->dev, "✓ Phase 1 SUCCESS: BOOT_COMP went LOW (elapsed: %lums)\n",
                     jiffies_to_msecs(jiffies - low_wait_start_jiffies));
            dev_info(&dev->pdev->dev, "=== Phase 2: Waiting for BOOT_COMP 0→1 transition (timeout: %dms) ===\n",
                     BOOT_HIGH_WAIT_TIMEOUT_MS);
            break; /* Exit Phase 1, proceed to Phase 2 */
        }

        usleep_range(PCIE_HP_POLL_SLEEP_US, PCIE_HP_POLL_SLEEP_US + 1000);
    }

    /* If any timeout expires, abort bring-up and don't enumerate PCIe */
    if (!seen_low_after_power) {
        unsigned long elapsed_ms = jiffies_to_msecs(jiffies - low_wait_start_jiffies);
        dev_err(&dev->pdev->dev, "✗✗✗ Phase 1 FAILED: Timeout waiting for BOOT_COMP to go LOW\n");
        dev_err(&dev->pdev->dev, "   Timeout: %dms, Elapsed: %lums\n", BOOT_LOW_WAIT_TIMEOUT_MS, elapsed_ms);
        dev_err(&dev->pdev->dev, "   Last BOOT_COMP value: %d\n", boot_value);
        dev_err(&dev->pdev->dev, "   Bias configuration attempted: %s\n",
                bias_configured ? "YES" : "NO");
        dev_err(&dev->pdev->dev, "\nCRITICAL: BOOT_COMP never went LOW after power-enable!\n");
        dev_err(&dev->pdev->dev, "Cannot verify 0→1 transition without seeing LOW state.\n\n");
        dev_err(&dev->pdev->dev, "This is a GPIO configuration / board issue, NOT a firmware timeout.\n");
        dev_err(&dev->pdev->dev, "\nPossible causes:\n");
        dev_err(&dev->pdev->dev, "  1. GPIO pull-up (PullDefault=PullUp) masking true level%s\n",
                bias_configured ? " (runtime bias disable attempted but failed)" : "");
        dev_err(&dev->pdev->dev, "  2. Device not driving BOOT_COMP signal\n");
        dev_err(&dev->pdev->dev, "  3. Incorrect GPIO mapping/polarity\n");
        dev_err(&dev->pdev->dev, "\nACTION: Check DSDT GPIO pull configuration or verify hardware.\n");
        dev_err(&dev->pdev->dev, "=== ABORTING PCIe initialization - GPIO/board configuration issue ===\n");
        return -ETIMEDOUT; /* Abort - don't enumerate PCIe */
    }

    /* Phase 2: high_wait_timeout (5-15s) - Wait for BOOT to transition to 1 */
    while (time_before(jiffies, high_wait_start_jiffies + msecs_to_jiffies(BOOT_HIGH_WAIT_TIMEOUT_MS))) {
        boot_value = gpiod_get_value(dev->pins[PCIE_PIN_BOOT].desc);
        
        dev_dbg(&dev->pdev->dev, "Phase 2: BOOT_COMP=%d, stable_high_count=%d/%d (elapsed: %lums)\n",
                boot_value, stable_high_count, debounce_reads,
                jiffies_to_msecs(jiffies - high_wait_start_jiffies));

        if (boot_value == 1) {
            /* Phase 3: Require BOOT to stay high for debounce window (5-10ms) */
            stable_high_count++;
            if (stable_high_count >= debounce_reads) {
                dev_info(&dev->pdev->dev, "✓✓✓ Phase 2 SUCCESS: CX7 firmware boot complete!\n");
                dev_info(&dev->pdev->dev, "   BOOT_COMP GP77: 0→1 transition detected\n");
                dev_info(&dev->pdev->dev, "   Stable HIGH for %d reads (~%dms)\n",
                        stable_high_count,
                        stable_high_count * (PCIE_HP_POLL_SLEEP_US / 1000));
                dev_info(&dev->pdev->dev, "   Total elapsed time: %lums\n",
                        jiffies_to_msecs(jiffies - low_wait_start_jiffies));
                dev_info(&dev->pdev->dev, "=== BOOT_COMP polling complete - proceeding with PCIe initialization ===\n");
                return 0; /* Success - valid rising edge detected */
            } else {
                dev_dbg(&dev->pdev->dev, "Phase 2: BOOT_COMP HIGH, debouncing (%d/%d)\n",
                        stable_high_count, debounce_reads);
            }
        } else {
            /* Signal went low again - reset debounce counter */
            if (stable_high_count > 0) {
                dev_dbg(&dev->pdev->dev, "Phase 2: BOOT_COMP went LOW again, resetting debounce counter\n");
            }
            stable_high_count = 0;
        }

        usleep_range(PCIE_HP_POLL_SLEEP_US, PCIE_HP_POLL_SLEEP_US + 1000);
    }

    /* Timeout in Phase 2 - saw low but never got stable high */
    unsigned long total_elapsed_ms = jiffies_to_msecs(jiffies - low_wait_start_jiffies);
    unsigned long phase2_elapsed_ms = jiffies_to_msecs(jiffies - high_wait_start_jiffies);
    dev_err(&dev->pdev->dev, "✗✗✗ Phase 2 FAILED: Timeout waiting for BOOT_COMP 0→1 transition\n");
    dev_err(&dev->pdev->dev, "   Phase 2 timeout: %dms, Elapsed: %lums\n",
            BOOT_HIGH_WAIT_TIMEOUT_MS, phase2_elapsed_ms);
    dev_err(&dev->pdev->dev, "   Total elapsed time: %lums\n", total_elapsed_ms);
    dev_err(&dev->pdev->dev, "   Last BOOT_COMP value: %d\n", boot_value);
    dev_err(&dev->pdev->dev, "   Stable high count: %d/%d (need %d for debounce)\n",
            stable_high_count, debounce_reads, debounce_reads);
    dev_err(&dev->pdev->dev, "\nCRITICAL: BOOT_COMP never asserted HIGH within %dms!\n", BOOT_HIGH_WAIT_TIMEOUT_MS);
    dev_err(&dev->pdev->dev, "FW did not signal ready - BOOT_COMP went LOW but never transitioned to HIGH.\n\n");
    dev_err(&dev->pdev->dev, "Possible causes:\n");
    dev_err(&dev->pdev->dev, "  1. CX7 firmware boot failure\n");
    dev_err(&dev->pdev->dev, "  2. Firmware boot takes longer than %dms (may need timeout adjustment)\n", BOOT_HIGH_WAIT_TIMEOUT_MS);
    dev_err(&dev->pdev->dev, "  3. BOOT_COMP signal issue (hardware problem)\n");
    dev_err(&dev->pdev->dev, "\nACTION: Check CX7 firmware status, increase timeout, or verify hardware.\n");
    dev_err(&dev->pdev->dev, "=== ABORTING PCIe initialization - firmware boot incomplete ===\n");
    return -ETIMEDOUT; /* Abort - don't enumerate PCIe */
}

static int rescan_device(struct pcie_hp_dev *dev)
{
    struct pci_dev *pci_dev;
    int i, err;

    /* IMPORTANT: This function is ONLY called after polling_boot_complete() succeeds.
     * BOOT_COMP GP77 0→1 transition has been verified, firmware is ready.
     * 
     * Correct sequence per CX7 spec:
     * 1. Power CX7 (GP251=1) ✓ (done before polling_boot_complete)
     * 2. Wait for BOOT_COMP 0→1 ✓ (polling_boot_complete)
     * 3. Assert PERST# (put device in reset)
     * 4. Enable REFCLK
     * 5. Wait for REFCLK stable
     * 6. Deassert PERST# (release reset, link can train)
     * 7. Wait for link-up
     * 8. Rescan devices
     */

    dev_info(&dev->pdev->dev, "=== Starting PCIe initialization (BOOT_COMP verified) ===\n");

    /* Step 1: Enable REFCLK - Change pinctrl state to clkreqn (enables clock request) */
    dev_info(&dev->pdev->dev, "Step 1: Enabling REFCLK (pinctrl: clkreqn)\n");
    err = pcie_hp_change_state(dev, "clkreqn");
    if (err)
        return err;

    /* Step 2: Enable clock control */
    dev_info(&dev->pdev->dev, "Step 2: Enabling clock control\n");
    pcie_hp_ckm_control(dev, false);
    usleep_range(PCIE_HP_DELAY_STANDARD_US, PCIE_HP_DELAY_STANDARD_US + 1000);

    /* Step 3: Resume root ports */
    dev_info(&dev->pdev->dev, "Step 3: Resuming root ports\n");
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

    /* Step 4: Assert PCIe reset */
    dev_info(&dev->pdev->dev, "Step 4: Asserting PCIe reset (PERST#)\n");
    gpiod_set_value(dev->pins[PCIE_PIN_PERST].desc, 1);

    /* Step 5: Apply bus protection */
    dev_info(&dev->pdev->dev, "Step 5: Applying bus protection\n");
    if (dev->pd->rp_bus_protect) {
        for (i = 0; i < dev->pd->port_nums; i++)
            dev->pd->rp_bus_protect(dev, i, BUS_PROTECT_CABLE_PLUGIN);
    }

    /* Step 6: Wait for links to reach L0 */
    dev_info(&dev->pdev->dev, "Step 6: Waiting for PCIe link to reach L0\n");
    err = polling_link_to_l0(dev);
    if (err) {
        dev_err(&dev->pdev->dev, "PCIe link failed to reach L0\n");
        return err;
    }
    dev_info(&dev->pdev->dev, "PCIe link reached L0 state\n");
 
    /* Step 7: Retrain PCIe links to Gen5 */
    dev_info(&dev->pdev->dev, "Step 7: Retraining PCIe links to Gen5\n");
    for (i = 0; i < dev->pd->port_nums; i++) {
        pci_dev = get_port_root_port(dev, i);
        if (pci_dev)
            retrain_pcie_link(pci_dev);
    }
 
    /* Step 8: Wait for link stability */
    dev_info(&dev->pdev->dev, "Step 8: Waiting for link stability (%dms)\n",
             PCIE_HP_DELAY_LINK_STABLE_MS);
    msleep(PCIE_HP_DELAY_LINK_STABLE_MS);
 
     dev_info(&dev->pdev->dev, "=== PCIe initialization complete - ready for device rescan ===\n");
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
    spin_unlock_irqrestore(&hp_dev->lock, flags);

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
        /* BOOT_COMP went low (1→0 transition) during power-down - verify it's stable.
         * This ensures clean power-down state and prevents stale state issues
         * if we try to power on again.
         */
        dev_dbg(app_ctx->ctx->dev, "BOOT_COMP low detected (power-down), verifying stability\n");
        ret = verify_boot_transition_stable(hp_dev, 0, "1→0 (power-down)");
        if (ret == 0) {
            dev_dbg(app_ctx->ctx->dev, "BOOT_COMP 1→0 transition verified, device powered off cleanly\n");
        } else {
            /* Not stable - might indicate issue, but device is already powered off */
            dev_warn(app_ctx->ctx->dev, "BOOT_COMP 1→0 transition not stable during power-down\n");
        }
        break;
    case STATE_DEV_POWER_ON:
        /* BOOT_COMP went low (1→0 transition) - verify firmware is starting boot.
         * Verify the 1→0 transition is stable before proceeding.
         */
        dev_dbg(app_ctx->ctx->dev, "BOOT_COMP low detected (firmware starting), verifying stability\n");
        ret = verify_boot_transition_stable(hp_dev, 0, "1→0");
        if (ret == 0) {
            /* Stable low - firmware is starting boot */
            spin_lock_irqsave(&hp_dev->lock, flags);
            hp_dev->state = STATE_DEV_FW_START;
            spin_unlock_irqrestore(&hp_dev->lock, flags);
            dev_dbg(app_ctx->ctx->dev, "BOOT_COMP 1→0 transition verified, firmware boot started\n");
        } else {
            /* Not stable - might be glitch, stay in STATE_DEV_POWER_ON */
            dev_dbg(app_ctx->ctx->dev, "BOOT_COMP 1→0 transition not stable, will retry\n");
        }
        break;
    case STATE_DEV_FW_START:
        /* BOOT_COMP interrupt fired - verify 0→1 transition (firmware ready).
         * The IRQ handler detected BOOT_COMP=1, but we need to verify:
         * 1. We actually saw the 0→1 transition (not just pull-up masking)
         * 2. It's stable before proceeding to PCIe initialization.
         */
        dev_info(app_ctx->ctx->dev, ">>> STATE_DEV_FW_START: BOOT_COMP interrupt detected, verifying 0→1 transition\n");
        ret = polling_boot_complete(hp_dev);
        if (ret == 0) {
            /* Valid 0→1 transition detected and stable - firmware is ready */
            dev_info(app_ctx->ctx->dev, ">>> BOOT_COMP verification SUCCESS - transitioning to STATE_RESCAN\n");
            spin_lock_irqsave(&hp_dev->lock, flags);
            hp_dev->state = STATE_RESCAN;
            spin_unlock_irqrestore(&hp_dev->lock, flags);
            /* Fall through to STATE_RESCAN case */
        } else {
            /* No valid 0→1 transition or not stable - stay in STATE_DEV_FW_START */
            dev_warn(app_ctx->ctx->dev, ">>> BOOT_COMP verification FAILED (ret=%d) - staying in STATE_DEV_FW_START, will retry\n", ret);
            break;
        }
        /* fall through */
    case STATE_RESCAN:
        dev_info(app_ctx->ctx->dev, ">>> STATE_RESCAN: Starting PCIe device rescan (BOOT_COMP verified)\n");
        ret = rescan_device(hp_dev);
        spin_lock_irqsave(&hp_dev->lock, flags);
        if (ret)
            dev_err(app_ctx->ctx->dev, "Rescan failed: %d\n", ret);
        else
            hp_dev->state = STATE_READY;
        spin_unlock_irqrestore(&hp_dev->lock, flags);
        break;
    default:
        dev_err(app_ctx->ctx->dev, "Unknown state: %d\n", state);
        break;
    }

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
 
    /* Handle presence pin events first - no lock needed as prsnt_pin is read-only */
    if (gpio_ctx->pin == hp_dev->prsnt_pin) {
        if (value) {
            dev_dbg(gpio_ctx->dev, "Presence pin: cable removal detected\n");
            pcie_hp_send_uevent(hp_dev, REMOVAL_EVT);
        } else {
            dev_dbg(gpio_ctx->dev, "Presence pin: cable plug-in detected\n");
            pcie_hp_send_uevent(hp_dev, PLUG_IN_EVT);
        }
        /* For physical hotplug, let userspace handle device management
         * to avoid potential deadlocks when hardware is in transition */
        return IRQ_HANDLED;
    }

    spin_lock_irqsave(&hp_dev->lock, flags);
    state = hp_dev->state;
 
     /* Handle boot status pin events (compare hardware pin numbers - robust against order changes) */
     if (gpio_ctx->pin == hp_dev->boot_pin) {
         if (value && state == STATE_PLUG_IN) {
             dev_dbg(gpio_ctx->dev, "Boot pin high: device powered on\n");
             hp_dev->state = STATE_DEV_POWER_ON;
         } else if (value && state == STATE_DEV_FW_START) {
             /* BOOT_COMP=1 detected - wake work function to verify stability.
              * Don't transition to STATE_RESCAN immediately - let work function
              * verify BOOT_COMP is stable high before proceeding.
              */
             dev_dbg(gpio_ctx->dev, "Boot pin high: device ready (will verify stability)\n");
             /* Stay in STATE_DEV_FW_START - work function will verify and transition */
         } else if (!value && state == STATE_DEV_POWER_ON) {
             dev_dbg(gpio_ctx->dev, "Boot pin low: firmware starting\n");
             hp_dev->state = STATE_DEV_FW_START;
         } else if (!value && state == STATE_PLUG_OUT) {
             dev_dbg(gpio_ctx->dev, "Boot pin low: device powered off\n");
             hp_dev->state = STATE_DEV_POWER_OFF;
         } else {
             dev_dbg(gpio_ctx->dev, "Boot pin changed: pin=%d irq=%d value=%d state=%d\n",
                     gpio_ctx->pin, irq, value, state);
             spin_unlock_irqrestore(&hp_dev->lock, flags);
             return IRQ_HANDLED;
         }
         spin_unlock_irqrestore(&hp_dev->lock, flags);
         return IRQ_WAKE_THREAD;
     }
 
     /* Unknown GPIO pin */
     dev_err(gpio_ctx->dev, "Unknown GPIO pin event: pin=%d irq=%d value=%d\n",
              gpio_ctx->pin, irq, value);
     spin_unlock_irqrestore(&hp_dev->lock, flags);
     return IRQ_HANDLED;
 }
 
/**
 * acpi_gpio_walk_handler - Handler for acpi_walk_resources to collect all GPIO resources
 * @ares: ACPI resource structure
 * @context: Pointer to acpi_gpio_walk_context
 *
 * This callback is invoked by acpi_walk_resources() for each GPIO resource in _CRS.
 * It collects all GPIO pins, their properties, and vendor data.
 */
static acpi_status acpi_gpio_walk_handler(struct acpi_resource *ares, void *context)
{
    struct acpi_gpio_walk_context *walk_ctx = context;
    struct acpi_resource_gpio *agpio;
    int length;

    if (ares->type != ACPI_RESOURCE_TYPE_GPIO)
        return AE_OK;

    if (walk_ctx->count >= PCIE_PIN_MAX) {
        dev_warn(walk_ctx->dev, "Too many GPIO resources, truncating at %d\n", PCIE_PIN_MAX);
        return AE_OK;
    }

    agpio = &ares->data.gpio;

    /* Get pin number from pin table (first pin) */
    if (!agpio->pin_table || agpio->pin_table_length == 0) {
        dev_warn(walk_ctx->dev, "GPIO resource has no pin table\n");
        return AE_OK;
    }

    /* Store GPIO information */
    walk_ctx->gpios[walk_ctx->count].pin = agpio->pin_table[0];
    walk_ctx->gpios[walk_ctx->count].connection_type = agpio->connection_type;
    walk_ctx->gpios[walk_ctx->count].triggering = agpio->triggering;
    walk_ctx->gpios[walk_ctx->count].polarity = agpio->polarity;
    walk_ctx->gpios[walk_ctx->count].debounce_timeout = agpio->debounce_timeout;
    walk_ctx->gpios[walk_ctx->count].wake_capable = agpio->wake_capable;

    /* Store vendor data if present */
    if (agpio->vendor_length && agpio->vendor_data) {
        length = min_t(int, agpio->vendor_length, MAX_VENDOR_DATA_LEN);
        memcpy(walk_ctx->gpios[walk_ctx->count].vendor_data, agpio->vendor_data, length);
        walk_ctx->gpios[walk_ctx->count].vendor_data[length] = '\0';
    } else {
        walk_ctx->gpios[walk_ctx->count].vendor_data[0] = '\0';
    }

    /* Store resource source (GPIO controller name) */
    if (agpio->resource_source.string_ptr) {
        length = min_t(int, agpio->resource_source.string_length, 15);
        memcpy(walk_ctx->gpios[walk_ctx->count].resource_source,
               agpio->resource_source.string_ptr, length);
        walk_ctx->gpios[walk_ctx->count].resource_source[length] = '\0';
    } else {
        walk_ctx->gpios[walk_ctx->count].resource_source[0] = '\0';
    }
    walk_ctx->gpios[walk_ctx->count].resource_source_index = agpio->resource_source.index;

    dev_dbg(walk_ctx->dev, "ACPI walk GPIO[%d]: pin=%u, type=%s, resource_source=%s, index=%u\n",
            walk_ctx->count,
            walk_ctx->gpios[walk_ctx->count].pin,
            agpio->connection_type == ACPI_RESOURCE_GPIO_TYPE_INT ? "GpioInt" : "GpioIo",
            walk_ctx->gpios[walk_ctx->count].resource_source[0] ? walk_ctx->gpios[walk_ctx->count].resource_source : "<none>",
            walk_ctx->gpios[walk_ctx->count].resource_source_index);

    walk_ctx->count++;
    return AE_OK;
}

/**
 * pcie_hp_walk_acpi_gpios - Walk ACPI _CRS to collect all GPIO resources
 * @pdev: Platform device
 * @walk_ctx: Context structure to fill with GPIO information
 *
 * Uses acpi_walk_resources() to parse all GPIO resources from ACPI _CRS.
 * This bypasses the kernel's GPIO grouping bug by directly accessing ACPI resources.
 *
 * Returns: 0 on success, negative error code on failure
 */
static int pcie_hp_walk_acpi_gpios(struct platform_device *pdev,
                                    struct acpi_gpio_walk_context *walk_ctx)
{
    struct acpi_device *adev;
    acpi_status status;

    adev = ACPI_COMPANION(&pdev->dev);
    if (!adev) {
        dev_err(&pdev->dev, "Failed to get ACPI companion device\n");
        return -ENODEV;
    }

    /* Initialize walk context */
    memset(walk_ctx, 0, sizeof(*walk_ctx));
    walk_ctx->dev = &pdev->dev;

    /* Walk all GPIO resources in _CRS */
    status = acpi_walk_resources(adev->handle, METHOD_NAME__CRS,
                                 acpi_gpio_walk_handler, walk_ctx);
    if (ACPI_FAILURE(status)) {
        dev_err(&pdev->dev, "Failed to walk ACPI GPIO resources: %s\n",
                acpi_format_exception(status));
        return -EIO;
    }

    dev_info(&pdev->dev, "Found %d GPIO resources via ACPI walk\n", walk_ctx->count);
    
    if (walk_ctx->count == 0) {
        dev_info(&pdev->dev, "ACPI walk found 0 GPIOs, will fallback to gpiod_get_index\n");
        return -ENOENT; /* Signal to caller to fallback to gpiod_get_index */
    }
    
    return 0;
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
 
     /* Store vendor data and identify BOOT/PRSNT pins (upstream-friendly) */
     if (agpio->vendor_length && agpio->vendor_data && hp_dev) {
         length = min_t(int, agpio->vendor_length, MAX_VENDOR_DATA_LEN);
         memcpy(&ctx->vendor_data[0], agpio->vendor_data, length);
         ctx->vendor_data[length] = '\0';
 
         /* Identify BOOT and PRSNT pins by vendor data and store hardware pin numbers */
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
 
    switch (val) {
    case PCIE_HP_DEBUG_PLUG_OUT:
        dev_info(dev, "Debug: simulating cable removal\n");

        /* Safety check: Verify no devices on the bus before hardware shutdown.
         * This prevents silicon bug where CPU access during link down causes
         * system hang. Userspace must remove PCI devices first.
         * Release lock before calling sleep-capable function (pci_find_bus can sleep) */
        for (i = 0; i < hp_dev->pd->port_nums; i++) {
            if (pci_devices_present_on_domain(hp_dev->pd->ports[i].domain)) {
                dev_err(dev, "PCI devices still present, remove them first\n");
                return -EBUSY;
            }
        }

        /* Safe to proceed - no devices on bus */
        spin_lock_irqsave(&hp_dev->lock, flags);
        hp_dev->state = STATE_PLUG_OUT;
        hp_dev->debug_state = val;
        spin_unlock_irqrestore(&hp_dev->lock, flags);
        /* Release lock before calling sleep-capable function */
        remove_device(hp_dev);
        return count;

    case PCIE_HP_DEBUG_PLUG_IN:
        dev_info(dev, "Debug: simulating cable plug-in\n");

        /* Release lock before calling sleep-capable function (pci_find_bus can sleep) */
        for (i = 0; i < hp_dev->pd->port_nums; i++) {
            if (pci_devices_present_on_domain(hp_dev->pd->ports[i].domain)) {
                dev_err(dev, "PCI devices already present, cannot reinitialize hardware\n");
                return -EBUSY;
            }
        }

        /* Safe to proceed - no devices on bus */
        spin_lock_irqsave(&hp_dev->lock, flags);
        hp_dev->state = STATE_PLUG_IN;
        hp_dev->debug_state = val;
        spin_unlock_irqrestore(&hp_dev->lock, flags);
        /* Release lock before calling sleep-capable function */
        /* Enable device power - GPIO IRQ state machine will handle rest */
        gpiod_set_value(hp_dev->pins[PCIE_PIN_EN].desc, 1);
        return count;

    default:
        return -EINVAL;
    }
 
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
                                                    struct pcie_hp_dev *hp_dev,
                                                    int gpio_index)
 {
     struct acpi_gpio_parse_context parse_ctx;
     struct gpio_acpi_context *ctx;
     struct acpi_device *adev;
     acpi_status status;
 
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
 
     /* Parse ACPI _CRS to get GPIO properties and identify special pins */
     status = acpi_walk_resources(adev->handle, METHOD_NAME__CRS,
                                  acpi_gpio_resource_handler, &parse_ctx);
     if (ACPI_FAILURE(status)) {
         devm_kfree(&pdev->dev, ctx);
         return NULL;
     }
 
     if (ctx->valid) {
         /* Fallback: If vendor data missing, identify BOOT/PRSNT pins by array index
          * This ensures compatibility with DSDT that doesn't have vendor data  */
         if (gpio_index == PCIE_PIN_BOOT && hp_dev->boot_pin == -1) {
             hp_dev->boot_pin = ctx->pin;
             dev_info(&pdev->dev, "BOOT pin identified by index: hardware pin %d\n", hp_dev->boot_pin);
         } else if (gpio_index == PCIE_PIN_PRSNT && hp_dev->prsnt_pin == -1) {
             hp_dev->prsnt_pin = ctx->pin;
             dev_info(&pdev->dev, "PRSNT pin identified by index: hardware pin %d\n", hp_dev->prsnt_pin);
         }
         return ctx;
     }
 
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

/**
 * pcie_hp_put_gpio_device - Release GPIO device reference
 * @data: GPIO device pointer
 *
 * Cleanup function called by devm_add_action_or_reset() to release
 * the GPIO device reference when the hotplug device is removed.
 */
static void pcie_hp_put_gpio_device(void *data)
{
    struct gpio_device *gdev = data;

    gpio_device_put(gdev);
}
 
 /**
  * pcie_hp_discover_devices - Discover existing PCI devices on managed ports
  * @pdev: platform device
  * @pd: platform data
  *
  * Scans for PCI devices matching the vendor/device ID specified in platform data
  * and verifies they're on one of the managed PCIe domains.
  * This version can be called before hp_dev is allocated (like 6.14).
  *
  * Returns: 0 on success, -EPROBE_DEFER if devices are still initializing,
  *         negative error code on failure
  */
 static int pcie_hp_discover_devices(struct platform_device *pdev,
                                      struct pcie_hp_plat_data *pd)
 {
     struct pci_dev *pci_dev = NULL;
     int device_count = 0;
     int i;
 
     /* If no vendor/device ID specified, skip device discovery */
     if (!pd->vendor_id || !pd->device_id)
         return 0;
 
     /* Find all matching PCI devices */
     while ((pci_dev = pci_get_device(pd->vendor_id,
                                       pd->device_id,
                                       pci_dev)) != NULL) {
         /* Wait for device to be initialized */
         if (!pci_dev->state_saved) {
             pci_dev_put(pci_dev);
             return -EPROBE_DEFER;
         }
 
         /* Verify device is on one of our managed ports */
         for (i = 0; i < pd->port_nums; i++) {
             if (pci_domain_nr(pci_dev->bus) == pd->ports[i].domain)
                 break;
         }
 
         if (i == pd->port_nums) {
             dev_err(&pdev->dev,
                     "Device %s found on unexpected domain %d\n",
                     pci_name(pci_dev), pci_domain_nr(pci_dev->bus));
             pci_dev_put(pci_dev);
             return -ENODEV;
         }
 
         device_count++;
     }
 
     if (pd->num_devices && device_count != pd->num_devices) {
         dev_err(&pdev->dev,
                 "Required number of devices not found. Expected=%d Actual=%d\n",
                 pd->num_devices, device_count);
         return -ENODEV;
     }
 
    return 0;
}

/**
 * pcie_hp_enumerate_gpios - Enumerate GPIOs with three-tier fallback
 * @pdev: Platform device
 * @hp_dev: Hotplug device structure
 * @pd: Platform data
 *
 * Attempts GPIO enumeration in this order:
 * 1. ACPI walk resources (bypasses kernel 6.17 GPIO grouping bug)
 * 2. gpiod_get_index (standard kernel API) + platform data fallback for EN pin
 *
 * Returns: Number of GPIOs found, or negative error code
 */
static int pcie_hp_enumerate_gpios(struct platform_device *pdev,
                                     struct pcie_hp_dev *hp_dev,
                                     struct pcie_hp_plat_data *pd)
{
    struct acpi_gpio_walk_context walk_ctx;
    struct gpio_desc *first_desc = NULL;
    unsigned int gpio_chip_base = 0;
    int ret, i, get_index_gpio_count = 0;
    bool use_acpi_walk = false;
    struct fwnode_handle *gpio_fwnode = NULL;
    struct acpi_device *gpio_adev = NULL;
    acpi_handle gpio_handle;
    acpi_status status;

    /* Step 1: Try ACPI walk resources first */
    ret = pcie_hp_walk_acpi_gpios(pdev, &walk_ctx);
    if (!ret && walk_ctx.count >= PCIE_HP_MIN_GPIO_COUNT) {
        dev_info(&pdev->dev, "ACPI walk found %d GPIOs, attempting to find GPIO device\n", walk_ctx.count);
        
        /* Find GPIO device using resource_source from first GPIO */
        if (walk_ctx.count > 0 && walk_ctx.gpios[0].resource_source[0] != '\0') {
            dev_info(&pdev->dev, "Looking up GPIO controller: %s\n", walk_ctx.gpios[0].resource_source);
            status = acpi_get_handle(NULL, walk_ctx.gpios[0].resource_source, &gpio_handle);
            if (ACPI_SUCCESS(status)) {
                gpio_adev = acpi_fetch_acpi_dev(gpio_handle);
                if (gpio_adev) {
                    dev_info(&pdev->dev, "Found ACPI GPIO device: %s\n", acpi_device_name(gpio_adev));
                    gpio_fwnode = acpi_fwnode_handle(gpio_adev);
                    hp_dev->gdev = gpio_device_find_by_fwnode(gpio_fwnode);
                    if (hp_dev->gdev) {
                        /* Successfully found GPIO device - manage reference */
                        ret = devm_add_action_or_reset(&pdev->dev, pcie_hp_put_gpio_device,
                                                       hp_dev->gdev);
                        if (ret) {
                            gpio_device_put(hp_dev->gdev);
                            hp_dev->gdev = NULL;
                            dev_warn(&pdev->dev, "Failed to register GPIO device cleanup, falling back to gpiod_get_index\n");
                        } else {
                            gpio_chip_base = gpio_device_get_base(hp_dev->gdev);
                            dev_info(&pdev->dev, "Found GPIO device via fwnode, chip base: %u\n", gpio_chip_base);
                            use_acpi_walk = true;
                        }
                    } else {
                        dev_info(&pdev->dev, "GPIO device not found (controller may not be loaded), deferring probe\n");
                        return dev_err_probe(&pdev->dev, -EPROBE_DEFER,
                                             "GPIO controller not available\n");
                    }
                } else {
                    dev_warn(&pdev->dev, "Failed to get ACPI device for GPIO controller %s, falling back to gpiod_get_index\n",
                             walk_ctx.gpios[0].resource_source);
                }
            } else {
                dev_warn(&pdev->dev, "Failed to get ACPI handle for GPIO controller %s, falling back to gpiod_get_index\n",
                         walk_ctx.gpios[0].resource_source);
            }
        } else {
            dev_warn(&pdev->dev, "No resource_source in ACPI GPIO resources, falling back to gpiod_get_index\n");
        }
    } else {
        if (ret)
            dev_info(&pdev->dev, "ACPI walk failed (%d), falling back to gpiod_get_index\n", ret);
        else
            dev_info(&pdev->dev, "ACPI walk found only %d GPIOs (< %d required), falling back to gpiod_get_index\n",
                     walk_ctx.count, PCIE_HP_MIN_GPIO_COUNT);
    }

    /* Step 2: Fall back to gpiod_get_index if ACPI walk didn't work */
    if (!use_acpi_walk) {
        get_index_gpio_count = pcie_hp_probe_io_info(pdev);
        if (get_index_gpio_count == 0) {
            dev_info(&pdev->dev, "gpiod_get_index found 0 GPIOs, exiting gracefully\n");
            return -ENODEV;
        } else if (get_index_gpio_count >= PCIE_HP_MIN_GPIO_COUNT) {
            dev_info(&pdev->dev, "gpiod_get_index found %d GPIOs\n", get_index_gpio_count);
            /* GPIO chip base not needed here - only needed for platform data fallback (EN pin) */
        } else {
            dev_warn(&pdev->dev, "gpiod_get_index found only %d GPIOs (< %d required), will use platform data fallback for EN pin\n",
                     get_index_gpio_count, PCIE_HP_MIN_GPIO_COUNT);
        }
    }

    /* Step 3: Determine final GPIO count and allocation */
    if (use_acpi_walk) {
        hp_dev->gpio_count = walk_ctx.count;
    } else {
        /* Use gpiod_get_index count - platform data fallback only for EN pin if missing */
        hp_dev->gpio_count = get_index_gpio_count;
        /* Get GPIO chip base only if we're using gpiod_get_index and might need platform data fallback for EN pin */
        if (get_index_gpio_count > 0 && pd->gpio_pin_en > 0) {
            first_desc = gpiod_get_index(&pdev->dev, NULL, 0, GPIOD_ASIS);
            if (!IS_ERR(first_desc)) {
                gpio_chip_base = gpio_device_get_base(gpiod_to_gpio_device(first_desc));
                gpiod_put(first_desc);
                dev_info(&pdev->dev, "GPIO chip base: %u (from gpiod_get_index for platform data EN pin fallback)\n",
                         gpio_chip_base);
            }
        }
    }

    if (hp_dev->gpio_count < PCIE_HP_MIN_GPIO_COUNT) {
        dev_err(&pdev->dev, "Insufficient GPIOs: required at least %d, got %d\n",
                PCIE_HP_MIN_GPIO_COUNT, hp_dev->gpio_count);
        return -ENODEV;
    }

    /* Allocate GPIO context array */
    hp_dev->pins = devm_kzalloc(&pdev->dev,
                                 sizeof(struct pcie_hp_gpio_ctx) * hp_dev->gpio_count,
                                 GFP_KERNEL);
    if (!hp_dev->pins) {
        dev_err(&pdev->dev, "Failed to allocate memory for GPIOs\n");
        return -ENOMEM;
    }

    /* Setup GPIO pins based on enumeration method */
    for (i = 0; i < hp_dev->gpio_count; i++) {
        struct pcie_hp_gpio_ctx *app_ctx = &hp_dev->pins[i];
        unsigned int global_gpio;

        if (use_acpi_walk && i < walk_ctx.count) {
            /* Use ACPI walk data with cached GPIO device - NO buggy APIs */
            if (hp_dev->gdev) {
                /* Use gpio_device_get_desc() to get descriptor directly - bypasses kernel 6.17 grouping bug */
                app_ctx->desc = gpio_device_get_desc(hp_dev->gdev, walk_ctx.gpios[i].pin);
                if (IS_ERR(app_ctx->desc)) {
                    dev_err(&pdev->dev, "Failed to get GPIO descriptor for ACPI pin %u (index %d): %ld\n",
                            walk_ctx.gpios[i].pin, i, PTR_ERR(app_ctx->desc));
                    return PTR_ERR(app_ctx->desc);
                }
                dev_info(&pdev->dev, "Using ACPI walk GPIO[%d]: pin %u (chip-relative) -> global GPIO %u via gpio_device_get_desc\n",
                         i, walk_ctx.gpios[i].pin,
                         gpio_device_get_base(hp_dev->gdev) + walk_ctx.gpios[i].pin);
            } else {
                /* GPIO device not available - this should not happen if we got here */
                dev_err(&pdev->dev, "GPIO device not available for ACPI walk path (index %d)\n", i);
                return -ENODEV;
            }
        } else {
            /* Use gpiod_get_index result */
            app_ctx->desc = gpiod_get_index(&pdev->dev, NULL, i, GPIOD_ASIS);
            /* Fallback to platform data only for EN pin if gpiod_get_index fails */
            if (IS_ERR(app_ctx->desc) && i == PCIE_PIN_EN && gpio_chip_base > 0 && pd->gpio_pin_en > 0) {
                global_gpio = gpio_chip_base + pd->gpio_pin_en;
                app_ctx->desc = gpio_to_desc(global_gpio);
                if (!app_ctx->desc) {
                    dev_err(&pdev->dev, "Failed to get GPIO descriptor for platform data EN pin %u (global GPIO %u)\n",
                            pd->gpio_pin_en, global_gpio);
                    return -ENODEV;
                }
                dev_info(&pdev->dev, "Using platform data GPIO fallback: pin %u -> global GPIO %u for EN (index %d)\n",
                         pd->gpio_pin_en, global_gpio, i);
            }
        }

        if (IS_ERR(app_ctx->desc)) {
            dev_err(&pdev->dev, "Failed to get GPIO %d: %ld\n", i, PTR_ERR(app_ctx->desc));
            return PTR_ERR(app_ctx->desc);
        }

        app_ctx->hp_dev = hp_dev;
    }

    return hp_dev->gpio_count;
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
 
     /* Discover existing PCI devices (like 6.14's device discovery in probe) */
     ret = pcie_hp_discover_devices(pdev, pd);
     if (ret) {
         dev_err(&pdev->dev, "Device discovery failed: %d\n", ret);
         return ret;
     }
 
    hp_dev = devm_kzalloc(&pdev->dev, sizeof(*hp_dev), GFP_KERNEL);
    if (!hp_dev) {
        dev_err(&pdev->dev, "Failed to allocate memory for hotplug device\n");
        return -ENOMEM;
    }

    hp_dev->pdev = pdev;
    hp_dev->pd = pd;
    hp_dev->state = STATE_READY;
    hp_dev->boot_pin = -1;
    hp_dev->prsnt_pin = -1;
    spin_lock_init(&hp_dev->lock);

    /* Initialize cached root port pointers */
    for (i = 0; i < HP_PORT_MAX; i++)
        hp_dev->cached_root_ports[i] = NULL;

    /* Enumerate GPIO pins with three-tier fallback:
     * 1. ACPI walk resources
     * 2. gpiod_get_index
     * 3. Platform data fallback */
    ret = pcie_hp_enumerate_gpios(pdev, hp_dev, pd);
    if (ret < 0) {
        dev_err(&pdev->dev, "Failed to enumerate GPIOs: %d\n", ret);
        return ret;
    }

    /* Setup GPIO ACPI context and IRQs */
    for (i = 0; i < hp_dev->gpio_count; i++) {
        app_ctx = &hp_dev->pins[i];
        
        app_ctx->ctx = gpio_acpi_setup(pdev, app_ctx->desc, hp_dev, i);
        if (!app_ctx->ctx) {
            dev_err(&pdev->dev, "Failed to setup GPIO %d\n", i);
            ret = -ENODEV;
            goto gpio_release;
        }

        gpiod_set_debounce(app_ctx->desc, app_ctx->ctx->debounce_timeout_us);

        /* Setup IRQ for interrupt-type GPIOs */
        if (app_ctx->ctx->connection_type == ACPI_RESOURCE_GPIO_TYPE_INT) {
            dev_info(&pdev->dev, "Setting up IRQ for GPIO %d (pin %d)\n", i, app_ctx->ctx->pin);
            ret = pcie_hp_setup_irq(app_ctx);
            if (ret) {
                dev_err(&pdev->dev, "Failed to setup IRQ for GPIO %d\n", i);
                goto gpio_release;
            }
            dev_info(&pdev->dev, "IRQ %d registered for GPIO %d\n", gpiod_to_irq(app_ctx->desc), i);
        }
    }

    platform_set_drvdata(pdev, hp_dev);
 
     /* Initialize pinctrl */
     ret = pcie_hp_pinctrl_init(hp_dev);
     if (ret) {
         dev_err(&pdev->dev, "Pinmux init failed, ret: %d\n", ret);
         goto gpio_release;
     }
 
     /* Create sysfs interface */
     ret = sysfs_create_group(&pdev->dev.kobj, &pcie_hp_attr_group);
     if (ret) {
         dev_err(&pdev->dev, "Sysfs creation failed: %d\n", ret);
         goto pinctrl_remove;
     }
 
     /* Initialize bus protection (like 6.14's rp_bus_prepare during probe)
      * This handles MMIO mapping and device discovery - call once for first port only
      * since MMIO mapping and device discovery are per-device, not per-port */
     if (hp_dev->pd->rp_bus_protect) {
         hp_dev->pd->rp_bus_protect(hp_dev, 0, BUS_PROTECT_INIT);
     }
 
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
 
 pinctrl_remove:
     pcie_hp_pinctrl_remove(hp_dev);
gpio_release:
    if (hp_dev && hp_dev->pins) {
        for (i = 0; i < hp_dev->gpio_count; i++) {
            app_ctx = &hp_dev->pins[i];
            if (app_ctx->desc)
                gpiod_put(app_ctx->desc);
        }
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
 
     /* Cleanup bus protection (matches 6.14's rp_bus_prepare(pdev, false)) */
     if (hp_dev->pd->rp_bus_protect)
         hp_dev->pd->rp_bus_protect(hp_dev, 0, BUS_PROTECT_CLEANUP);
 
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
             .addr = 0x1d600000,
             .ctrl = 0x400,
             .update_bit = BIT(24),
             .port_bits = {BIT(6), BIT(2)},
         },
         .protect = {
             .addr = 0x1d640000,
             .mode = 0x38,
             .enable = 0x40,
             .port_bits = {BIT(20), BIT(16)},
         },
         .mac = {
             .addr = {0x1d790000, 0x1d690000},
             .init_ctrl = 0x008,
             .ltssm_bit = BIT(0),
             .phy_rst_bit = BIT(8),
         },
         .ckm = {
             .addr = 0x16bd0000,
             .ctrl = 0xa8,
             .disable_bit = BIT(5) | BIT(7),
         },
     },
    .rp_bus_protect = mt8901_rp_bus_protect,
    .ltssm_reg = 0x728,
    .ltssm_l0_state = 0x11,
    /* GPIO pin number for EN from DSDT (fallback for kernel 6.17 GPIO grouping workaround) */
    .gpio_pin_en = 146,
    /* Pinctrl mappings: Platform-specific GPIO pin multiplexing configuration
     * for PCIe clock request signals. These define SoC-specific pin names
     * and mux functions for the MediaTek MT8901 platform. */
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
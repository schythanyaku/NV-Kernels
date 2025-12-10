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
 
 static int rescan_device(struct pcie_hp_dev *dev)
 {
     struct pci_dev *pci_dev;
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
 
     spin_lock_irqsave(&hp_dev->lock, flags);
     state = hp_dev->state;
 
     /* Handle presence pin events (compare hardware pin numbers - robust against order changes) */
     if (gpio_ctx->pin == hp_dev->prsnt_pin) {
         if (value) {
             dev_dbg(gpio_ctx->dev, "Presence pin: cable removal detected\n");
             pcie_hp_send_uevent(hp_dev, REMOVAL_EVT);
         } else {
             dev_dbg(gpio_ctx->dev, "Presence pin: cable plug-in detected\n");
             pcie_hp_send_uevent(hp_dev, PLUG_IN_EVT);
         }
         spin_unlock_irqrestore(&hp_dev->lock, flags);
         /* For physical hotplug, let userspace handle device management
          * to avoid potential deadlocks when hardware is in transition */
         return IRQ_HANDLED;
     }
 
     /* Handle boot status pin events (compare hardware pin numbers - robust against order changes) */
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
 
     spin_lock_irqsave(&hp_dev->lock, flags);
 
     switch (val) {
     case PCIE_HP_DEBUG_PLUG_OUT:
         dev_info(dev, "Debug: simulating cable removal\n");
 
         /* Safety check: Verify no devices on the bus before hardware shutdown.
          * This prevents silicon bug where CPU access during link down causes
          * system hang. Userspace must remove PCI devices first. */
         for (i = 0; i < hp_dev->pd->port_nums; i++) {
             if (pci_devices_present_on_domain(hp_dev->pd->ports[i].domain)) {
                 dev_err(dev, "PCI devices still present, remove them first\n");
                 spin_unlock_irqrestore(&hp_dev->lock, flags);
                 return -EBUSY;
             }
         }
 
         /* Safe to proceed - no devices on bus */
         hp_dev->state = STATE_PLUG_OUT;
         remove_device(hp_dev);
         break;
 
     case PCIE_HP_DEBUG_PLUG_IN:
         dev_info(dev, "Debug: simulating cable plug-in\n");
 
         for (i = 0; i < hp_dev->pd->port_nums; i++) {
             if (pci_devices_present_on_domain(hp_dev->pd->ports[i].domain)) {
                 dev_err(dev, "PCI devices already present, cannot reinitialize hardware\n");
                 spin_unlock_irqrestore(&hp_dev->lock, flags);
                 return -EBUSY;
             }
         }
 
         /* Safe to proceed - no devices on bus */
         hp_dev->state = STATE_PLUG_IN;
         /* Enable device power - GPIO IRQ state machine will handle rest */
         gpiod_set_value(hp_dev->pins[PCIE_PIN_EN].desc, 1);
         break;
 
     default:
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
 
    /* Discover GPIO pins via ACPI */
    int acpi_gpio_count = pcie_hp_probe_io_info(pdev);
    if (!acpi_gpio_count) {
        dev_err(&pdev->dev, "Failed to get gpio descriptors\n");
        return -ENODEV;
    }
    
    /* Use platform data fallback only for EN (index 3) if ACPI enumeration returns fewer GPIOs
     * due to kernel 6.17 bug. CLQ0/CLQ1 are handled via pinctrl, not GPIO descriptors */
    if (acpi_gpio_count >= PCIE_HP_MIN_GPIO_COUNT) {
        /* ACPI provided all required GPIOs including EN */
        hp_dev->gpio_count = acpi_gpio_count;
    } else {
        /* ACPI enumeration insufficient (< 4 GPIOs) but provides at least 1 GPIO for base calculation
         * - use platform data fallback for minimum required GPIOs (EN) */
        dev_warn(&pdev->dev, "ACPI GPIO enumeration returned %d GPIOs (required %d), using platform data fallback for EN\n",
                 acpi_gpio_count, PCIE_HP_MIN_GPIO_COUNT);
        hp_dev->gpio_count = PCIE_HP_MIN_GPIO_COUNT;
    }
 
     hp_dev->pins = devm_kzalloc(&pdev->dev,
                                  sizeof(struct pcie_hp_gpio_ctx) * hp_dev->gpio_count,
                                  GFP_KERNEL);
     if (!hp_dev->pins) {
         dev_err(&pdev->dev, "Failed to allocate memory for GPIOs\n");
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
 
    /* Setup GPIO pins and IRQs */
    /* Calculate GPIO chip base from first GPIO (BOOT pin, index 0) - needed for platform data fallback */
    unsigned int gpio_chip_base = 0;
    bool gpio_chip_base_valid = false;
    if (acpi_gpio_count > 0) {
        struct gpio_desc *first_desc = gpiod_get_index(&pdev->dev, NULL, 0, GPIOD_ASIS);
        if (!IS_ERR(first_desc)) {
            gpio_chip_base = gpio_device_get_base(gpiod_to_gpio_device(first_desc));
            gpiod_put(first_desc);
            gpio_chip_base_valid = true;
            dev_info(&pdev->dev, "GPIO chip base: %u (calculated from ACPI GPIO 0, BOOT pin)\n", gpio_chip_base);
        } else {
            dev_warn(&pdev->dev, "Failed to get BOOT pin (GPIO 0) for chip base calculation: %ld\n",
                     PTR_ERR(first_desc));
        }
    }
    
    for (i = 0; i < hp_dev->gpio_count; i++) {
        app_ctx = &hp_dev->pins[i];
        
        /* Use platform data GPIO pins only for EN (index 3) if ACPI enumeration didn't provide it
         * (kernel 6.17 GPIO grouping bug workaround). CLQ0/CLQ1 are handled via pinctrl, not GPIO descriptors */
        if (i == PCIE_PIN_EN && i >= acpi_gpio_count) {
            if (!gpio_chip_base_valid) {
                dev_err(&pdev->dev, "Cannot use platform data fallback for EN: GPIO chip base not available\n");
                ret = -ENODEV;
                goto gpio_release;
            }
            /* Fallback: Convert ACPI pin number to global GPIO number and use gpio_to_desc()
             * ACPI pin numbers are chip-relative, so add GPIO chip base to get global GPIO number */
            unsigned int global_gpio = gpio_chip_base + pd->gpio_pin_en;
            app_ctx->desc = gpio_to_desc(global_gpio);
            if (!app_ctx->desc) {
                dev_err(&pdev->dev, "Failed to get GPIO descriptor for EN (power enable) pin: ACPI pin %u (global GPIO %u)\n",
                        pd->gpio_pin_en, global_gpio);
                ret = -ENODEV;
                app_ctx->desc = NULL;
                goto gpio_release;
            }
            dev_info(&pdev->dev, "Using platform data GPIO: ACPI pin %u -> global GPIO %u for EN (index %d)\n",
                     pd->gpio_pin_en, global_gpio, i);
        } else {
            /* Normal ACPI GPIO access for BOOT, PRSNT, PERST (indices 0-2) */
            app_ctx->desc = gpiod_get_index(&pdev->dev, NULL, i, GPIOD_ASIS);
            if (IS_ERR(app_ctx->desc)) {
                dev_err(&pdev->dev, "Failed to get GPIO %d: %ld\n",
                        i, PTR_ERR(app_ctx->desc));
                ret = PTR_ERR(app_ctx->desc);
                app_ctx->desc = NULL;
                goto gpio_release;
            }
        }
 
         app_ctx->hp_dev = hp_dev;
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
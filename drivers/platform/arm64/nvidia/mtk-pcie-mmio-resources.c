// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2025 NVIDIA Corporation
/*
 * Platform device resource provider for MTK PCIe Hotplug Driver
 *
 * This module provides MMIO resources for the mtk-pcie-hotplug driver.
 *
 * This is a temporary solution until firmware team adds proper MMIO
 * resources to the ACPI DSDT for the PEDE device.
 *
 * Why this exists:
 * - Current DSDT only provides GPIO resources to PEDE device
 * - MMIO resources exist in DSDT under RES0 (PNP resource device)
 * - Driver cannot access resources under RES0 from PEDE device
 * - Firmware blocks ACPI table override attempts (AE_ALREADY_EXISTS)
 * - Platform device is the upstream-recommended workaround
 *
 * Once firmware adds MMIO directly to PEDE._CRS in DSDT, this module
 * can be removed and the driver will automatically use ACPI resources.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/ioport.h>

/*
 * MMIO resource definitions for MediaTek PCIe controller
 * These addresses are SoC-specific and must match hardware layout
 */
#define MTK_PCIE_MMIO_TOP_BASE      0x1D600000
#define MTK_PCIE_MMIO_TOP_SIZE      0x1000
#define MTK_PCIE_MMIO_PROTECT_BASE  0x1D640000
#define MTK_PCIE_MMIO_PROTECT_SIZE  0x1000
#define MTK_PCIE_MMIO_CKM_BASE      0x16BD0000
#define MTK_PCIE_MMIO_CKM_SIZE      0x1000
#define MTK_PCIE_MMIO_MAC0_BASE     0x1D790000
#define MTK_PCIE_MMIO_MAC0_SIZE     0x1000
#define MTK_PCIE_MMIO_MAC1_BASE     0x1D690000
#define MTK_PCIE_MMIO_MAC1_SIZE     0x1000

/*
 * Platform device resources
 * Order must match driver expectations:
 *   0: TOP region
 *   1: PROTECT region
 *   2: CKM region
 *   3: MAC port 0
 *   4: MAC port 1
 */
static struct resource mtk_pcie_mmio_resources[] = {
	{
		.name  = "pcie-top",
		.start = MTK_PCIE_MMIO_TOP_BASE,
		.end   = MTK_PCIE_MMIO_TOP_BASE + MTK_PCIE_MMIO_TOP_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.name  = "pcie-protect",
		.start = MTK_PCIE_MMIO_PROTECT_BASE,
		.end   = MTK_PCIE_MMIO_PROTECT_BASE + MTK_PCIE_MMIO_PROTECT_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.name  = "pcie-ckm",
		.start = MTK_PCIE_MMIO_CKM_BASE,
		.end   = MTK_PCIE_MMIO_CKM_BASE + MTK_PCIE_MMIO_CKM_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.name  = "pcie-mac0",
		.start = MTK_PCIE_MMIO_MAC0_BASE,
		.end   = MTK_PCIE_MMIO_MAC0_BASE + MTK_PCIE_MMIO_MAC0_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.name  = "pcie-mac1",
		.start = MTK_PCIE_MMIO_MAC1_BASE,
		.end   = MTK_PCIE_MMIO_MAC1_BASE + MTK_PCIE_MMIO_MAC1_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

/*
 * Platform device definition
 * IMPORTANT: Device name must match driver's platform_driver.driver.name
 * or ACPI device name for proper binding
 */
static struct platform_device mtk_pcie_mmio_device = {
	.name          = "MTKP0001:00",  /* Match ACPI device name */
	.id            = -1,
	.num_resources = ARRAY_SIZE(mtk_pcie_mmio_resources),
	.resource      = mtk_pcie_mmio_resources,
};

static int __init mtk_pcie_mmio_init(void)
{
	int ret;

	pr_info("mtk-pcie-mmio: ========================================\n");
	pr_info("mtk-pcie-mmio: Platform Device MMIO Resource Provider\n");
	pr_info("mtk-pcie-mmio: ========================================\n");
	pr_info("mtk-pcie-mmio: This is a workaround for ACPI _CRS limitation\n");
	pr_info("mtk-pcie-mmio: Providing 5 MMIO regions for MTKP0001:00:\n");
	pr_info("mtk-pcie-mmio:   [0] TOP:     0x%08lx - 0x%08lx\n",
	        (unsigned long)MTK_PCIE_MMIO_TOP_BASE, 
	        (unsigned long)(MTK_PCIE_MMIO_TOP_BASE + MTK_PCIE_MMIO_TOP_SIZE - 1));
	pr_info("mtk-pcie-mmio:   [1] PROTECT: 0x%08lx - 0x%08lx\n",
	        (unsigned long)MTK_PCIE_MMIO_PROTECT_BASE,
	        (unsigned long)(MTK_PCIE_MMIO_PROTECT_BASE + MTK_PCIE_MMIO_PROTECT_SIZE - 1));
	pr_info("mtk-pcie-mmio:   [2] CKM:     0x%08lx - 0x%08lx\n",
	        (unsigned long)MTK_PCIE_MMIO_CKM_BASE,
	        (unsigned long)(MTK_PCIE_MMIO_CKM_BASE + MTK_PCIE_MMIO_CKM_SIZE - 1));
	pr_info("mtk-pcie-mmio:   [3] MAC0:    0x%08lx - 0x%08lx\n",
	        (unsigned long)MTK_PCIE_MMIO_MAC0_BASE,
	        (unsigned long)(MTK_PCIE_MMIO_MAC0_BASE + MTK_PCIE_MMIO_MAC0_SIZE - 1));
	pr_info("mtk-pcie-mmio:   [4] MAC1:    0x%08lx - 0x%08lx\n",
	        (unsigned long)MTK_PCIE_MMIO_MAC1_BASE,
	        (unsigned long)(MTK_PCIE_MMIO_MAC1_BASE + MTK_PCIE_MMIO_MAC1_SIZE - 1));

	ret = platform_device_register(&mtk_pcie_mmio_device);
	if (ret) {
		pr_err("mtk-pcie-mmio: Failed to register platform device: %d\n", ret);
		return ret;
	}

	pr_info("mtk-pcie-mmio: Platform device registered successfully\n");
	pr_info("mtk-pcie-mmio: Driver can now use platform_get_resource()\n");
	pr_info("mtk-pcie-mmio: ========================================\n");
	return 0;
}

static void __exit mtk_pcie_mmio_exit(void)
{
	pr_info("mtk-pcie-mmio: Unregistering MMIO platform device\n");
	platform_device_unregister(&mtk_pcie_mmio_device);
	pr_info("mtk-pcie-mmio: Cleanup complete\n");
}

module_init(mtk_pcie_mmio_init);
module_exit(mtk_pcie_mmio_exit);

MODULE_DESCRIPTION("MMIO resource provider for MTK PCIe Hotplug Driver");
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:mtk-pcie-mmio");


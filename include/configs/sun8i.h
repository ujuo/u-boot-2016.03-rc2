/*
 * (C) Copyright 2014 Chen-Yu Tsai <wens@csie.org>
 *
 * Configuration settings for the Allwinner A23 (sun8i) CPU
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

/*
 * A23 specific configuration
 */

#ifdef CONFIG_USB_EHCI
/*#define CONFIG_USB_EHCI_SUNXI*/
#define CONFIG_USB_MAX_CONTROLLER_COUNT	1
#endif

#if defined(CONFIG_CMD_NAND)
#define CONFIG_NAND_SUNXI	1
#define CONFIG_SYS_MAX_NAND_DEVICE	1
#define CONFIG_SYS_NAND_BASE	0x01C03000
#define CONFIG_SYS_NAND_U_BOOT_OFFS	0x8000
#define CCM_NAND_CTRL_ENABLE	0x80000000	/* Make clock enable, OSC24Mhz */
#define AHB_DIV_1				0x0			/* divN = 1, divM = 1 */
#define CONFIG_MTD_NAND_ECC_HW	1
#endif

#define CONFIG_SUNXI_USB_PHYS	2
/*#define CONFIG_USB_HOST_ETHER	1*/
/*#define CONFIG_USB_ETHER_RTL8152	1*/

#ifndef CONFIG_MACH_SUN8I_A83T
#define CONFIG_ARMV7_PSCI		1
#if defined(CONFIG_MACH_SUN8I_A23)
#define CONFIG_ARMV7_PSCI_NR_CPUS	2
#elif defined(CONFIG_MACH_SUN8I_A33)
#define CONFIG_ARMV7_PSCI_NR_CPUS	4
#elif defined(CONFIG_MACH_SUN8I_H3)
#define CONFIG_ARMV7_PSCI_NR_CPUS	4
#else
#error Unsupported sun8i variant
#endif
#endif

#define CONFIG_SYS_MAX_FLASH_BANKS	1

#define CONFIG_TIMER_CLK_FREQ		24000000

#define	CONFIG_SYS_MEMTEST_START 0x40000000
#define	CONFIG_SYS_MEMTEST_END   0x5D000000
/*
 * Include common sunxi configuration where most the settings are
 */
#include <configs/sunxi-common.h>

#endif /* __CONFIG_H */

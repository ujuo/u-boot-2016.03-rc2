/*
 * (C) Copyright 2015 Hans de Goede <hdegoede@redhat.com>
 *
 * Sunxi PMIC bus access helpers
 *
 * The axp152 & axp209 use an i2c bus, the axp221 uses the p2wi bus and the
 * axp223 uses the rsb bus, these functions abstract this.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/arch/p2wi.h>
#include <asm/arch/rsb.h>
#include <i2c.h>
#include <asm/arch/pmic_bus.h>

#define AXP152_I2C_ADDR			0x30

#define AXP209_I2C_ADDR			0x34

#define AXP221_CHIP_ADDR		0x68
#define AXP221_CTRL_ADDR		0x3e
#define AXP221_INIT_DATA		0x3e

/* AXP818 device and runtime addresses are same as AXP223 */
#define AXP223_DEVICE_ADDR		0x3a3
#define AXP223_RUNTIME_ADDR		0x2d

//#define I4VINE_SPECIAL	0

int pmic_bus_init(void)
{
	/* This cannot be 0 because it is used in SPL before BSS is ready */
	static int needs_init = 1;
	__maybe_unused int ret;
	unsigned int *ptr;

	if (!needs_init)
		return 0;

#if defined CONFIG_AXP221_POWER || defined CONFIG_AXP818_POWER
# ifdef CONFIG_MACH_SUN6I
	p2wi_init();
	ret = p2wi_change_to_p2wi_mode(AXP221_CHIP_ADDR, AXP221_CTRL_ADDR,
				       AXP221_INIT_DATA);
# else
#ifdef CONFIG_I4VINE_BOARD_REV0
	/* gpio PH2,3 should be pull up as I2C(no external pull up). */
	ptr = (unsigned int *)0x01C20918;
	*ptr &= ~0xF0;
	*ptr |= 0x50;
	/* I2C 0 & 1 & 2 is not enabled in reset so enable it. */
	ptr = (unsigned int *)0x01C202D8;
	*ptr = 0x7 | (0x1F << 16); /* All UART should be enabled by CCM */
	/* 0x134 is CSI MCLK control */
	ptr = (unsigned int *)0x01C20134;
	*ptr = 0x8500; /* CLK SOURCE to HOSC, 1/1 and enable */
	/* GPIO portb 0 should be input and PULL up */
	ptr = (unsigned int *)0x01C20824;
	*ptr &= 0xFFFFFFFC;
	*ptr |= 1; /* Input */
	/* Pull up */
	ptr = (unsigned int *)(0x01C20824 + 0x1C);
	*ptr &= 0xFFFFFFFC;
	*ptr |= 1; /* Pull up */
	/* GPIO porte 12,13 have no pull up */
	ptr = (unsigned int *)0x01C208AC;
	*ptr |= 0x5 << 24;
	/* GPIO porte 14,15 should be high for normal operation */
	ptr = (unsigned int *)0x01C20890;
	*ptr &= 0xFFFFFF0F; /* Bit 1 for MCLK */
	*ptr |= 0x20; /* Bit 1 mode 2 for MCLK */
	ptr = (unsigned int *)0x01C20894;
	*ptr &= 0x00FFFFFF; /* Bit 14, 15 for RESET, and PWDN */
	*ptr |= 0x11000000; /* Make output */
	ptr = (unsigned int *)(0x01C20890 + 0x10);
	*ptr = 0x00004000; /* Make high */

	i2c_init(60000, CONFIG_SYS_I2C_SLAVE);
	return 0;
#else
	ret = rsb_init();
	if (ret)
		return ret;

	ret = rsb_set_device_address(AXP223_DEVICE_ADDR, AXP223_RUNTIME_ADDR);
#endif
# endif
	if (ret)
		return ret;
#endif

	needs_init = 0;
	return 0;
}

int pmic_bus_read(u8 reg, u8 *data)
{
#ifdef CONFIG_AXP152_POWER
	return i2c_read(AXP152_I2C_ADDR, reg, 1, data, 1);
#elif defined CONFIG_AXP209_POWER
	return i2c_read(AXP209_I2C_ADDR, reg, 1, data, 1);
#elif defined CONFIG_AXP221_POWER || defined CONFIG_AXP818_POWER
# ifdef CONFIG_MACH_SUN6I
	return p2wi_read(reg, data);
# else
#ifdef CONFIG_I4VINE_BOARD_REV0
	return i2c_read(AXP209_I2C_ADDR, reg, 1, data, 1);
#else
	return rsb_read(AXP223_RUNTIME_ADDR, reg, data);
#endif
# endif
#endif
}

int pmic_bus_write(u8 reg, u8 data)
{
#ifdef CONFIG_AXP152_POWER
	return i2c_write(AXP152_I2C_ADDR, reg, 1, &data, 1);
#elif defined CONFIG_AXP209_POWER
	return i2c_write(AXP209_I2C_ADDR, reg, 1, &data, 1);
#elif defined CONFIG_AXP221_POWER || defined CONFIG_AXP818_POWER
# ifdef CONFIG_MACH_SUN6I
	return p2wi_write(reg, data);
# else
#ifdef CONFIG_I4VINE_BOARD_REV0
	return i2c_write(AXP209_I2C_ADDR, reg, 1, &data, 1);
#else
	return rsb_write(AXP223_RUNTIME_ADDR, reg, data);
#endif
# endif
#endif
}

int pmic_bus_setbits(u8 reg, u8 bits)
{
	int ret;
	u8 val;

	ret = pmic_bus_read(reg, &val);
	if (ret)
		return ret;

	val |= bits;
	return pmic_bus_write(reg, val);
}

int pmic_bus_clrbits(u8 reg, u8 bits)
{
	int ret;
	u8 val;

	ret = pmic_bus_read(reg, &val);
	if (ret)
		return ret;

	val &= ~bits;
	return pmic_bus_write(reg, val);
}

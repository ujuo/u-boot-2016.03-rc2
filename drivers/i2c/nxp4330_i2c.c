/*
 * (C) Copyright 2009
 * jung hyun kim, Nexell Co, <jhkim@nexell.co.kr>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#include <common.h>
#include <command.h>
#include <i2c.h>
#include <asm/arch/platform.h>

#define	DEBUG_I2C	(0)

#if	(DEBUG_I2C)
#define DBGOUT(msg...)		do { printf("i2c" msg); } while (0)
#else
#define DBGOUT(msg...)		do {} while (0)
#endif

/* gpio i2c 0 */
#ifdef CFG_IO_I2C0_SCL
	#define	I2C0_SCL	CFG_IO_I2C0_SCL
#else
	#define	I2C0_SCL	((PAD_GPIO_D + 2) | PAD_FUNC_ALT0)
#endif
#ifdef CFG_IO_I2C0_SDA
	#define	I2C0_SDA	CFG_IO_I2C0_SDA
#else
	#define	I2C0_SDA	((PAD_GPIO_D + 3) | PAD_FUNC_ALT0)
#endif
/* gpio i2c 1 */
#ifdef CFG_IO_I2C1_SCL
	#define	I2C1_SCL	CFG_IO_I2C1_SCL
#else
	#define	I2C1_SCL	((PAD_GPIO_D + 4) | PAD_FUNC_ALT0)
#endif
#ifdef CFG_IO_I2C1_SDA
	#define	I2C1_SDA	CFG_IO_I2C1_SDA
#else
	#define	I2C1_SDA	((PAD_GPIO_D + 5) | PAD_FUNC_ALT0)
#endif
/* gpio i2c 2 */
#ifdef CFG_IO_I2C2_SCL
	#define	I2C2_SCL	CFG_IO_I2C2_SCL
#else
	#define	I2C2_SCL	((PAD_GPIO_D + 6) | PAD_FUNC_ALT0)
#endif
#ifdef CFG_IO_I2C2_SDA
	#define	I2C2_SDA	CFG_IO_I2C2_SDA
#else
	#define	I2C2_SDA	((PAD_GPIO_D + 7) | PAD_FUNC_ALT0)
#endif
#ifdef CFG_IO_I2C3_SCL
	#define	I2C3_SCL	CFG_IO_I2C3_SCL
#endif
#ifdef CFG_IO_I2C3_SDA
	#define	I2C3_SDA	CFG_IO_I2C3_SDA
#endif


/*
 * I2C io maps
 */
struct i2c_params {
	int				device;
	unsigned int	sclpad;
	unsigned int	sdapad;
	int				no_stop;
	int				delay;
	unsigned int	speed;
};

static struct i2c_params i2c_info[3] = {
	{
		0, I2C0_SCL, I2C0_SDA, CONFIG_I2C0_NO_STOP,
		CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SPEED,
	},
	{
		1, I2C1_SCL, I2C1_SDA, CONFIG_I2C1_NO_STOP,
		CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SPEED,
	},
	{
		2, I2C2_SCL, I2C2_SDA, CONFIG_I2C2_NO_STOP,
		CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SPEED,
	},
#if defined (CFG_IO_I2C3_SCL) || defined (CFG_IO_I2C3_SDA)
	{
		3, I2C3_SCL, I2C3_SDA, CONFIG_I2C3_NO_STOP,
		CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SPEED,
	},
#endif	
};

/* set .data section, before u-boot is relocated */
static struct i2c_params * _pi2c __attribute__ ((section(".data"))) = NULL;
static int _busnum __attribute__ ((section(".data"))) = 0;

/*----------------------------------------------------------------------------
 * I2C u-boot
 */

#if	defined (CONFIG_I2C_GPIO_MODE)
	#include "nxp4330_gpio_i2c.c"
#else

#if defined(CONFIG_I2C_MULTI_BUS)
int i2c_set_bus_num (unsigned int bus)
{
	printf("%s: NOT IMPLEMENT\n", __func__);
	return -1;
}

unsigned int i2c_get_bus_num(void)
{
	printf("%s: NOT IMPLEMENT\n", __func__);
	return -1;
}
#endif

int i2c_set_bus_speed(unsigned int speed)
{
	printf("%s: NOT IMPLEMENT\n", __func__);
	return -1;
}

unsigned int i2c_get_bus_speed(void)
{
	printf("%s: NOT IMPLEMENT\n", __func__);
	return -1;
}

int i2c_write(u8 chip, u32 addr, int alen, u8 *buffer, int len)
{
	printf("%s: NOT IMPLEMENT\n", __func__);
	return -1;
}

int i2c_read (u8 chip, uint addr, int alen, u8 *buffer, int len)
{
	printf("%s: NOT IMPLEMENT\n", __func__);
	return -1;
}

void i2c_init(int speed, int slaveaddr)
{
	printf("%s: NOT IMPLEMENT\n", __func__);
}

/*
 * params in:
 * 		chip = slave addres
 * return:
 *		0 = detect else not detect
 */
int i2c_probe(u8 chip)
{
	struct i2c_params *i2c = _pi2c;
	if (NULL == _pi2c){
		i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);
		i2c = _pi2c = &i2c_info[_busnum];
	}
	printf("%s: NOT IMPLEMENT\n", __func__);
	return -1;
}
#endif /* CONFIG_I2C_GPIO_MODE */

/*
 *  To support non stop mode
 */
int do_i2c_mode (cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	char *cmd;

	cmd = argv[1];
	if (strcmp(cmd, "stop") == 0) {
		printf("Set i2c bus %d stop mode  \n", _busnum);
		i2c_info[_busnum].no_stop = 0;
		return 0;
	} else if (strcmp(cmd, "nostop") == 0) {
		printf("Set i2c bus %d nostop mode \n", _busnum);
		i2c_info[_busnum].no_stop = 1;
		return 0;
	} else {
		printf("Current i2c bus %d %s mode  \n",
			_busnum, i2c_info[_busnum].no_stop?"nostop":"stop");
	}
	return 1;
}

U_BOOT_CMD(
	i2cmod, 3, 1,	do_i2c_mode,
	"set I2C mode",
	"stop\n"
	"    - generate stop signal, when tx end (normal)\n"
	"i2cmod nostop\n"
	"    - skip stop signal, when tx end (restart)\n"
);




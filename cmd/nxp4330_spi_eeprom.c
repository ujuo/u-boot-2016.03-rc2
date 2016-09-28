/*
 * (C) Copyright 2000-2003
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
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

/*
 * Misc boot support
 */
#include <common.h>
#include <command.h>
#include <malloc.h>
#include <spi.h>
#include <asm/arch/nxp4330_boot.h>

extern int  eeprom_write (unsigned dev_addr, unsigned offset,
               uchar *buffer, unsigned cnt);
extern ssize_t spi_write (uchar *addr, int alen, uchar *buffer, int len);

#define POLY 0x04C11DB7L
unsigned int get_fcs(unsigned int fcs, unsigned char data)
{
	register int i;
	fcs ^= (unsigned int)data;
	for(i=0; i<8; i++)
	{
	 	if(fcs & 0x01) fcs ^= POLY; fcs >>= 1;
 	}
	return fcs;
}

/*
 * "update_eeprom <type> <mem> <length> ...\n"
 * "update_eeprom <type> <mem> <addr> <length> ...\n"
 */
int do_update_eeprom(cmd_tbl_t * cmdtp, int flag, int argc, char * const argv[])
{
	char *cmd = argv[1];
	unsigned char *p;
	unsigned int mem, addr, size;
	U8 offs[3] = { 0, };

	if (3 > argc)
		goto usage;

	if (0 != strcmp(argv[1], "raw") &&
		0 != strcmp(argv[1], "2ndboot") &&
		0 != strcmp(argv[1], "uboot")	&&
		0 != strcmp(argv[1], "clean") &&
		0 != strcmp(argv[1], "read"))
		goto usage;

#if defined(CONFIG_ENV_IS_IN_EEPROM)
	if (!strcmp(argv[1], "clean") &&
		!strcmp(argv[2], "env")) {

		char *buf = malloc(CONFIG_ENV_SIZE);
		addr = CONFIG_ENV_OFFSET;
		size = CONFIG_ENV_SIZE;

		/* clear buffer */
		memset((void*)buf, 0, CONFIG_ENV_SIZE);

		offs[0] = (addr >> 16);
		offs[1] = (addr >>  8);
		offs[2] = (addr & 0xFF);

		spi_write(offs, 3, (uchar*)buf, size);
		free(buf);
		return 0;
	}
#endif

	if (!strcmp(cmd, "uboot")) {
		struct boot_dev_head head;
		struct boot_dev_head *bh = &head;
		struct boot_dev_eeprom *bd = (struct boot_dev_eeprom *)&bh->bdi;
		int len = sizeof(head);
		unsigned int load = CONFIG_SYS_TEXT_BASE, CRC = 0;
		int i;

		spi_init_f();

		mem = simple_strtoul(argv[2], NULL, 16);
		if (argc > 4) {
			addr = simple_strtoul(argv[3], NULL, 16);
			size = simple_strtoul(argv[4], NULL, 16);
			if (6 == argc)
				load = simple_strtoul (argv[5], NULL, 16);
		} else {
			addr = CONFIG_UBOOT_OFFSET;
			size = simple_strtoul(argv[3], NULL, 16);
		}

		p = (unsigned char*)mem;
		for (i = 0; size > i; i++ )
			CRC = get_fcs(CRC, p[i]);

		memset(bh, 0xff, len);

		bh->load_addr = load;
		bh->jump_addr = bh->load_addr;
		bh->load_size = (int)size;
		bh->signature = SIGNATURE_ID;
		bd->addr_step = CONFIG_EEPROM_ADDRESS_STEP;
		bd->crc32 = CRC;

		p -= len;
		size += len;
		memcpy(p, bh, len);

		offs[0] = (addr >>  16);
		offs[1] = (addr >>   8);
		offs[2] = (addr & 0xFF);

		printf("update_eeprom uboot 0x%p to 0x%08x, size 0x%x\n", p, addr, size);

		return spi_write(offs, 3, (uchar*)p, size);
	}

	if (!strcmp(cmd, "2ndboot")) {
		unsigned int *buf;
		unsigned int CRC = 0;
		int i, ret = 0;

		buf = (unsigned int*)malloc(CONFIG_2NDBOOT_SIZE);
		p   = (unsigned char*)buf;

		spi_init_f();

		mem = simple_strtoul(argv[2], NULL, 16);
		if (argc > 4) {
			addr = simple_strtoul(argv[3], NULL, 16);
			size = simple_strtoul(argv[4], NULL, 16);
		} else {
			addr = CONFIG_2NDBOOT_OFFSET;
			size = simple_strtoul(argv[3], NULL, 16);
		}

		memset(p, 0xff, CONFIG_2NDBOOT_SIZE);
		memcpy(p, (uchar*)mem, size);

		for (i = 0; (CONFIG_2NDBOOT_SIZE-16) > i; i++ )
			CRC = get_fcs(CRC, p[i]);

		/* ADD CRC at Last 16K end */
		buf[(CONFIG_2NDBOOT_SIZE-16)/4] = CRC;

		offs[0] = (addr >> 16);
		offs[1] = (addr >>  8);
		offs[2] = (addr & 0xFF);
		size = CONFIG_2NDBOOT_SIZE;

		printf("update_eeprom 2ndboot 0x%p to 0x%08x, size 0x%x\n", p, addr, size);

		ret = spi_write(offs, 3, (uchar*)p, size);
		free(buf);

		return ret;
	}

	if (!strcmp(cmd, "raw")) {

		if (5 > argc)
			goto usage;

		spi_init_f();

		mem  = simple_strtoul(argv[2], NULL, 16);
		addr = simple_strtoul(argv[3], NULL, 16);
		size = simple_strtoul(argv[4], NULL, 16);

		p = (unsigned char*)mem;
		offs[0] = (addr >>  16);
		offs[1] = (addr >>   8);
		offs[2] = (addr & 0xFF);
		printf("update_eeprom raw 0x%p to 0x%08x, size 0x%x\n", p, addr, size);

		return spi_write(offs, 3, (uchar*)p, size);
	}
	
	if (!strcmp(cmd, "read")) {

		if (5 > argc)
			goto usage;

		spi_init_f();

		mem  = simple_strtoul(argv[2], NULL, 16);
		addr = simple_strtoul(argv[3], NULL, 16);
		size = simple_strtoul(argv[4], NULL, 16);

		p = (unsigned char*)mem;
		offs[0] = (addr >>  16);
		offs[1] = (addr >>   8);
		offs[2] = (addr & 0xFF);
		printf("update_eeprom read 0x%08x to 0x%p, size 0x%x\n", addr, p, size);

		return spi_read(offs, 3, (uchar*)p, size);
	}

	return 0;
usage:
	cmd_usage(cmdtp);
	return 1;
}

U_BOOT_CMD(
	update_eeprom, CONFIG_SYS_MAXARGS, 1,	do_update_eeprom,
	"update eemprom data\n",
	"<type> <mem> <length>\n"
	"update_eeprom <type> <mem> <addr> <length>\n"
	"    - type : 2ndboot | raw | uboot \n"
	"update_eeprom 2ndboot 'mem' 'length'\n"
	"    - update 'mem' data 'length' to 2ndboot address defined at config.h.\n"
	"update_eeprom 2ndboot 'mem' 'addr' 'length'\n"
	"    - update 'mem' data 'length' to device 'addr'.\n"
	"update_eeprom uboot 'mem' 'length'\n"
	"    - update 'mem' data 'length' to uboot address defined at config.h.\n"
	"update_eeprom uboot 'mem' 'addr' 'length'\n"
	"    - update 'mem' data 'length' to device 'addr'.\n"
	"update_eeprom raw 'mem' 'addr' 'length'\n"
	"    - update 'mem' data 'length' to device 'addr'.\n"
	"update_eeprom read 'mem' 'addr' 'length'\n"
	"    - read from device 'addr' data 'length' to 'mem'.\n"
	"Note.\n"
	"    - All numeric parameters are assumed to be hex.\n"
#if defined(CONFIG_ENV_IS_IN_EEPROM)
	"update_eeprom clean env\n"
	"    - clear environment data in eeprom\n"
#endif
);





/*
 * Copyright(c) 2011 MOSTiTECH co., ltd. by S.W. Kim <ksw@mostitech.com>
 * most2120 u-boot configuration file, using NEXELL's nxp2120
 *
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Marius Groeger <mgroeger@sysgo.de>
 * Gary Jennejohn <gj@denx.de>
 * David Mueller <d.mueller@elsoft.ch>
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

#ifndef __CONFIG_H
#define __CONFIG_H

/*
 * High Level Configuration Options
 * (easy to change)
 */
#define CONFIG_NXP2120		1		/* in a NEXELL's nxp2120 SoC */
#define CONFIG_NXP  		1		/* in a NEXELL's nxp Family  */
#define CONFIG_MOST2120		1		/* on a MOSTiTECH nxp2120 Board  */

#define CONFIG_CMDLINE_TAG		1	/* enable passing of ATAGs */
#define CONFIG_SETUP_MEMORY_TAGS	1
#define CONFIG_INITRD_TAG		1
#define CONFIG_EARLY_ETHINIT    1   /* Early init ETh device for macaddress */

#define MEMORY_BASE_ADDRESS	0x80000000
#define CONFIG_SYS_SDRAM_BASE	MEMORY_BASE_ADDRESS

/* input clock of PLL */
#define CONFIG_SYS_CLK_FREQ	12000000	/* the most2120 has 12MHz input clock */

#undef CONFIG_ENABLE_MMU
#ifdef CONFIG_ENABLE_MMU
#define virt_to_phys(x)	virt_to_phy_most2120(x)
#else
/*#define virt_to_phys(x)	(x)*/
#endif

#define CONFIG_MEMORY_UPPER_CODE

#undef CONFIG_USE_IRQ				/* we don't need IRQ/FIQ stuff */

#define CONFIG_INCLUDE_TEST

/*#define CONFIG_ZIMAGE_BOOT		1
#define CONFIG_IMAGE_BOOT		1

#define BOARD_LATE_INIT			1
*/
/*
 * Architecture magic and machine type
 */
#define MACH_TYPE		2120
#define UBOOT_MAGIC		(0x43090000 | MACH_TYPE)

/* Power Management is enabled */
#define CONFIG_PM

#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO

#undef CONFIG_SKIP_LOWLEVEL_INIT
#undef CONFIG_SKIP_RELOCATE_UBOOT
#undef CONFIG_USE_NOR_BOOT

/***************************************************************
 * Boot sound for most2120...
 **************************************************************/
#define CONFIG_BOOTSOUND
/*#define CONFIG_BOOTSOUND_AC97
#define CONFIG_BOOTSOUND_SUPERNOVA16
#define CONFIG_BOOTSOUND_SUPERNOVA32
*/
/***************************************************************
 * Boot stop can be done with JUST ctrl-U
 **************************************************************/
#define CONFIG_BOOTSTOP_CTRL_U

/***************************************************************
 * Size of malloc() pool
 **************************************************************/
#define CONFIG_SYS_MALLOC_LEN		(CONFIG_ENV_SIZE + 1024*1024)
#define CFG_GBL_DATA_SIZE	256 /*128*/	/* size in bytes reserved for initial data */
#define CFG_STACK_SIZE		512*1024 /*??*/

/****************************************************************
 * Hardware drivers
 ***************************************************************/
/*#define CONFIG_DRIVER_SMC911X	0*/	/* mv6400 has CS8900 */
#undef  CONFIG_DRIVER_SMC911X
#undef	CONFIG_DRIVER_CS8900

#define CONFIG_DRIVER_DM9000
#define CONFIG_DM9000_BASE	0x4000000
#define DM9000_IO	CONFIG_DM9000_BASE
#define DM9000_DATA	(CONFIG_DM9000_BASE + 4)

#define CFG_NET_ETHADDR_EMBEDDED_ADDR   16
#define CFG_FORCE_EMBEDDED_ETHEADDR 1

/**************************************************************
 * Timer
 **************************************************************/
/* Use timer chanel 4 for Tick count */
#define CFG_CLKTICK_TIMER_CH    4

/***************************************************************
 * select serial console configuration
 **************************************************************/
#define CONFIG_SERIAL1          1	/* we use SERIAL 1 on most2120 board */
#define CONFIG_SERIAL_USE_FIFO  1  /* USE FIFO */

#undef CONFIG_SYS_HUSH_PARSER			/* do not use "hush" command parser	*/
#ifdef CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT_HUSH_PS2	"> "
#endif

#define CONFIG_CMDLINE_EDITING

#define CONFIG_NXP2120_I2C		/* this board has H/W I2C */
#ifdef CONFIG_NXP2120_I2C
#define CONFIG_HARD_I2C		1
#define CONFIG_SYS_I2C_SPEED		400
#define CFG_I2C_SLAVE		0

#if !(defined(CONFIG_BOOTSOUND_SUPERNOVA16) || defined(CONFIG_BOOTSOUND_SUPERNOVA32))
#define CFG_I2C_NXP2120_PORT1   /* PORT1 or */
#else
#define CFG_I2C_NXP2120_PORT2   /* PORT2 */
#endif

#endif

/***************************************************************
 * Define nand hardware 
 ***************************************************************/
#define CONFIG_NAND_NXP2120_HW	1
/* 4,8 or 16. It would be 4 is enough for SLC. 8 would needed for MLC */
/* 16 sould not used as it exeeds nand's spare area */
#define CFG_NAND_ECC_MODE       4
#define CFG_NAND_ECC_LIMIT      4

/************************************************************
 * RTC
 ************************************************************/
/*#define CONFIG_RTC_NXP2120	1*/

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE

#define CONFIG_BAUDRATE		115200

/************************************************************
 * LCD/VGA/FB
 ************************************************************/
/*/#define CONFIG_NXP2120_FB	1*/

/***********************************************************
 * Command definition
 ***********************************************************/
#include <config_distro_defaults.h>
/*#define CONFIG_CMD_PING			1*/
#undef CONFIG_CMD_NFS
#undef CONFIG_CMD_SETGETDCR
#undef CONFIG_CMD_FLASH
#undef CONFIG_CMD_FPGA
#undef CONFIG_CMD_ITEST
#undef CONFIG_CMD_MISC
#undef CONFIG_CMD_SETGETDCR
#undef CONFIG_CMD_XIMG
#undef CONFIG_CMD_IMI
/*#undef CONFIG_CMD_LOADB*/	/* loadb			*/
#undef CONFIG_CMD_LOADS	/* loads			*/
#undef CONFIG_CMD_EXT2
#undef CONFIG_CMD_EXT4
#undef CONFIG_CMD_FAT
#undef CONFIG_CMD_MII
#undef CONFIG_CMD_FS_GENERIC
#undef CONFIG_CMD_DHCP
#undef CONFIG_DOS_PARTITION
#undef CONFIG_EFI_PARTITION
#undef CONFIG_SUPPORT_RAW_INITRD

#if 0 /* For smaller size, disable some features */
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_SPI
#define CONFIG_CMD_DATE
#endif
#define CONFIG_CMD_NAND			1
#if 0 /* We need no EEPROM anyway. */
#define CONFIG_CMD_EEPROM
#endif
/* No flash, no imls*/
#undef  CONFIG_CMD_IMLS
#define CONFIG_SYS_NO_FLASH			1

#define CONFIG_LOADADDR		0x80800000
/*#define CONFIG_CMD_I2C*/

/* this must be included AFTER the definition of CONFIG_COMMANDS (if any) */

/*#undef CONFIG_BOOTDELAY*/
/*#define CONFIG_BOOTDELAY	3*/
/*#define CONFIG_BOOTARGS    	"root=ramfs devfs=mount console=ttySA0,9600" */
/*#define CONFIG_ETHADDR		00:40:5c:26:0a:5b*/
#define CONFIG_NETMASK          255.255.255.0

#define CONFIG_ZERO_BOOTDELAY_CHECK
/*#define CONFIG_BOOT_RETRY_TIME	3*/
/*#define CONFIG_RESET_TO_RETRY*/

#define CONFIG_FAST_BBT		1
#define CONFIG_FAST_BBT_SIZE	0x600000
#define CONFIG_SW_MAPPED_FASTBOOT	1       /* ksw : new to fast boot */

/*#define CONFIG_NET_MULTI	1 */

#if (CONFIG_COMMANDS & CFG_CMD_KGDB)
#define CONFIG_KGDB_BAUDRATE	115200		/* speed to run kgdb serial port */
/* what's this ? it's not used anywhere */
#define CONFIG_KGDB_SER_INDEX	1		/* which serial port to use */
#endif

/*
 * Miscellaneous configurable options
 */
#define CONFIG_SYS_LONGHELP				/* undef to save memory		*/
#define CONFIG_SYS_PROMPT		"=> "	        /* Monitor Command Prompt	*/
#define CONFIG_SYS_CBSIZE		256		/* Console I/O Buffer Size	*/
#define CONFIG_SYS_PBSIZE		384		/* Print Buffer Size */
#define CONFIG_SYS_MAXARGS		16		/* max number of command args	*/
#define CONFIG_SYS_BARGSIZE		CFG_CBSIZE	/* Boot Argument Buffer Size	*/

#define CONFIG_SYS_MEMTEST_START	MEMORY_BASE_ADDRESS	/* memtest works on	*/
#define CONFIG_SYS_MEMTEST_END		MEMORY_BASE_ADDRESS + 0x7e00000		/* 128 MB in DRAM	*/

#undef CFG_CLKS_IN_HZ		/* everything, incl board info, in Hz */

#define CONFIG_SYS_LOAD_ADDR		(MEMORY_BASE_ADDRESS + 0x800000)	/* default load address	*/

/* the TImer 4 uses a counter of 30000 for 10 ms, so we need */
/* it to wrap 100 times (total 3000000) to get 1 sec. */
#define CFG_HZ			3000000		/* at PLL1 192MHz*/

/* valid baudrates */
#define CFG_BAUDRATE_TABLE	{ 9600, 19200, 38400, 57600, 115200 }

/*-----------------------------------------------------------------------
 * Stack sizes
 *
 * The stack sizes are set up in start.S using the settings below
 */
#define CONFIG_STACKSIZE	0x40000		/* regular stack 256KB */
#ifdef CONFIG_USE_IRQ
#define CONFIG_STACKSIZE_IRQ	(4*1024)	/* IRQ stack */
#define CONFIG_STACKSIZE_FIQ	(4*1024)	/* FIQ stack */
#endif

#define CONFIG_NR_DRAM_BANKS	1	   /* we have 2 bank of DRAM */
#define PHYS_SDRAM_1		MEMORY_BASE_ADDRESS /* SDRAM Bank #1 */
#define PHYS_SDRAM_1_SIZE_128M	0x08000000 /* 128 MB */
#define PHYS_SDRAM_1_SIZE_64M	0x04000000 /* 64 MB */
#define PHYS_SDRAM_2		(MEMORY_BASE_ADDRESS + PHYS_SDRAM_1_SIZE) /* SDRAM Bank #1 */
#define PHYS_SDRAM_2_SIZE	0x08000000 /* 128 MB */

#if DDR_16BIT_64M
#define PHYS_SDRAM_1_SIZE   PHYS_SDRAM_1_SIZE_64M
#else
#define PHYS_SDRAM_1_SIZE   PHYS_SDRAM_1_SIZE_128M
#endif

#define CONFIG_SYS_FLASH_BASE		0x00000000

/*-----------------------------------------------------------------------
 * FLASH and environment organization
 */
#define CONFIG_SYS_MAX_FLASH_BANKS	0	/* max number of memory banks */
#define CONFIG_SYS_MAX_FLASH_SECT	2048

/* timeout values are in ticks */
#define CONFIG_SYS_FLASH_ERASE_TOUT	(5*CFG_HZ) /* Timeout for Flash Erase */
#define CONFIG_SYS_FLASH_WRITE_TOUT	(5*CFG_HZ) /* Timeout for Flash Write */

#define CONFIG_ENV_ADDR		0
#define CONFIG_ENV_SIZE		0x4000	/* Total Size of Environment Sector */
#define CONFIG_SYS_MONITOR_BASE		CONFIG_SYS_FLASH_BASE	/* Monitor at beginning of flash */
#define CONFIG_SYS_MONITOR_LEN		(256 * 1024)	/* Reserve 256KiB */

/*
 * NXP2120 board specific data
 */

#define CONFIG_IDENT_STRING	" for most2120"

/* total memory required by uboot */
#define CFG_UBOOT_SIZE		(2*1024*1024)

/*#define CONFIG_SYS_ICACHE_OFF*/
#define CONFIG_SYS_DCACHE_OFF



/* base address for uboot */
#ifdef CONFIG_ENABLE_MMU
#define CONFIG_SYS_UBOOT_BASE		(0xcf700000) 
#else
#define CONFIG_SYS_UBOOT_BASE		0x85F00000/*0x85F00000*/ /*0x87F00000*/
#endif
#define CONFIG_SYS_PHY_UBOOT_BASE		0x85F00000/*MEMORY_BASE_ADDRESS + 0x5F00000*/
#define CONFIG_SYS_TEXT_BASE			0x85F00000/*0x85f00000*/ /*excute address*/		
#define CONFIG_SYS_INIT_SP_ADDR			(0x85F00000 - 0x100000)/*(GENERATED_GBL_DATA_SIZE+PHYS_SDRAM_1+2048)*/
/*
 * NAND should be 512bytes/page or 2048bytes/page
 */
#define CONFIG_ENV_OFFSET		(0x400000 - 256*1024)

/* NAND configuration */
#define CONFIG_SYS_MAX_NAND_DEVICE     1
#define CONFIG_SYS_NAND_BASE           (0x2C00000)
#define NAND_MAX_CHIPS          1
#define CONFIG_SYS_NAND_MAX_CHIPS 1
/*#define CONFIG_SYS_NAND_BASE_LIST	{CONFIG_SYS_NAND_BASE}*/

#define NAND_DISABLE_CE()	(__REG(NXP2120_NFCONTROL) &= ~NXP2120_NFCONT_NCSENB)
#define NAND_ENABLE_CE()	(__REG(NXP2120_NFCONTROL) |= NXP2120_NFCONT_NCSENB)

#define CFG_NAND_SKIP_BAD_DOT_I	1  /* ".i" read skips bad blocks   */
#define	CFG_NAND_WP		1
#define CFG_NAND_YAFFS_WRITE	1  /* support yaffs write */

#define NAND_bw    8 
#define NAND_tACS  0 
#define NAND_tCOS  1
#define NAND_tACC  7
#define NAND_tSACC 7
#define NAND_tOCH  1
#define NAND_tCAH  1
#define NAND_wm    1
#define NAND_rb    0
#define NAND_wb    0
 
/* Boot configuration (define only one of next) */
/*#define CONFIG_BOOT_NOR*/
#define CONFIG_BOOT_NAND
/*#define CONFIG_BOOT_MOVINAND
#define CONFIG_BOOT_ONENAND
#define CONFIG_BOOT_ONENAND_IROM*/

#define	CONFIG_NAND

/*
 * BL1 should be written in the block0 with 8 bit ecc parity codes
 * Enable this definition if you use iROM-NAND boot
 */
/*#define CONFIG_NAND_BL1_8BIT_ECC*/

/* Settings as above boot configuration */
#if defined(CONFIG_BOOT_NAND)
#define CONFIG_ENV_IS_IN_NAND
#define CFG_NAND_LARGEPAGE_SAVEENV
/*#define CFG_NAND_HWECC*/
#if defined(CFG_NAND_HWECC)
#define CONFIG_ECCTYPE_MANUAL 1
#endif
/*#define CFG_NAND_FLASH_BBT*/
#if 1
/*#define CONFIG_BOOTCOMMAND	"nand read 50008000 40000 3c0000;bootm 50008000"*/
#else
#define CONFIG_BOOTCOMMAND	"tftp c0008000 zImage.mv6410 ; bootm 50008000"
#define CONFIG_BOOTARGS     "root=/dev/nfs rw nfsroot=192.168.1.160:/mnt/rootfs_mv6410 ip=192.168.1.165:192.168.1.160:192.168.1.254:255.255.255.0:mv6410:eth0:off init=linuxrc console=ttySAC0,115200n81"
#endif
#elif defined(CONFIG_BOOT_MOVINAND)
#define CONFIG_ENV_IS_IN_MOVINAND
#define CONFIG_BOOTCOMMAND	"movi read kernel c0008000;movi read rootfs c0800000;bootm c0008000"
#elif defined(CONFIG_BOOT_ONENAND) || defined(CONFIG_BOOT_ONENAND_IROM)
#define CFG_ONENAND_BASE 	(0x70100000)
#define CFG_MAX_ONENAND_DEVICE	1
#define CONFIG_ENV_IS_IN_ONENAND
#define CONFIG_BOOTCOMMAND	"onenand read c0008000 40000 3c0000;bootm c0008000"
#else
# error Define one of CONFIG_BOOT_{NAND|MOVINAND|ONENAND|ONENAND_IROM}
#endif

#define	CONFIG_EXTRA_ENV_SETTINGS					\
	"netdev=eth0\0"							\
	"ipaddr=192.168.1.210\0"					\
	"serverip=192.168.1.46\0"					\
	"uboot=most2120/u-boot.bin\0"					\
	"kernel=most2120/uImage\0"					\
	"nfsroot=/opt/nxp-devel/root\0"				\
	"hostname=nxp2120\0"						\
	"filesys=squashfs\0"						\
	"verify=no\0"							\
	"kload_addr=80007FC0\0"                                         \
	"bootargs_base=setenv bootargs noinitrd console=ttyS0,115200\0"	\
	"bootargs_nfs=setenv bootargs ${bootargs} root=/dev/nfs rw nfsroot=${serverip}:${nfsroot} "	\
	"ip=${ipaddr}:${serverip}:${serverip}:255.255.255.0:${hostname}::off\0"	\
	"bootargs_fs=setenv bootargs ${bootargs} root=/dev/mtdblock2 rootfstype=${filesys}\0"\
	"bootcmd_net=run bootargs_base bootargs_nfs; "			\
	"tftpboot ${loadaddr} ${kernel};bootm\0"		\
	"gb=tftp ${loadaddr} ${uboot};go ${loadaddr}\0"    \
	"bootcmd_nand=run bootargs_base bootargs_fs;nboot.i ${kload_addr} 0 40000;bootm ${kload_addr}\0"

	
	/*"bootcmd=run bootcmd_nand\0"					\*/
#endif	/* __CONFIG_H */

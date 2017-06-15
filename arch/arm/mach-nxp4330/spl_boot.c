/*
 * Copyright (C) 2017 i4vine 
 *
 * All right reserved by JuYoung Ryu
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/proc-armv/ptrace.h>
#include <asm/arch/platform.h>
#include <spl.h>
#include <linux/mtd/mtd.h>
#include <nand.h>
#include <asm/io.h>
DECLARE_GLOBAL_DATA_PTR;
#if 0
void spl_board_prepare_for_linux(void)
{
	/* Nothing to do! */
}
#endif
u32 spl_boot_device(void)
{
	return BOOT_DEVICE_NAND;	
}

#ifdef CONFIG_SPL_BOARD_INIT
int spl_board_init(void)
{
	return 0;
}
#endif

void board_init_f(unsigned long bootflag)
{
	__attribute__((noreturn)) void (*uboot)(void);
//	unsigned short int *dbg1 = (unsigned short int *)0xC0016002;
//	unsigned short int *dbg2 = (unsigned short int *)0xC0016082;

//	*dbg1 = 0;
//	*dbg2 = 0;

	preloader_console_init();
	arch_cpu_init();
//	timer_init();
	pl01x_serial_init();

	if (!spl_start_uboot()){
		printf("kernel load\n");
		dram_init_banksize();
		board_init_r(NULL, 0);
	} else {
	/*	int *src __attribute__((unused));
		int *dst __attribute__((unused));
//		printf("init_f spl_load\n");
			nand_init();
		nand_spl_load_image(CONFIG_CMD_SPL_NAND_OFS,
			CONFIG_CMD_SPL_WRITE_SIZE,
			(void *)CONFIG_SYS_TEXT_BASE);
		for (dst = (int *)CONFIG_SYS_SPL_ARGS_ADDR,
				src = (int *)CONFIG_SYS_TEXT_BASE;
				src < (int *)(CONFIG_SYS_TEXT_BASE +
				CONFIG_CMD_SPL_WRITE_SIZE);
				src++, dst++) {
			writel(readl(src), dst);
		}
		//spl_nand_load_kernel();
		nand_spl_load_image(
				CONFIG_SYS_NAND_SPL_KERNEL_OFFS,
				2898280,(void *)0x40007FC0);
		//	printf("loadaddr 0x%X, size 0x%X err %d\n", spl_image.load_addr, spl_image.size,err);
		printf("kernel load\n");
*/
		printf("uboot load\n");
		spl_nand_load_image();
	
		dram_init();
		printf("jump 0x%X\n", CONFIG_SYS_TEXT_BASE);
		/* Jump to U-Boot image */
		uboot = (void *)CONFIG_SYS_TEXT_BASE;
		(*uboot)();
		/* Never returns Here */
	}

}

	
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
#if 1
static struct tag *params;
static unsigned char kernel_mode;

unsigned char boot_mode_sel(void)
{
	return kernel_mode;	
}

void spl_board_prepare_for_linux(void)
{
	unsigned long machid = 0xffffffff;
	unsigned long r2 = 0xffffffff;
//	char* commandline = "bootargs=noinitrd console=ttyAMA0,115200n8\0";
	char* cmdline;
	char *s;
	char * p;
	int fake = 0;	
	bd_t *bd = gd->bd;
	cmdline = getenv("bootargs_spl");
//	printf("\nusing: ATAGS\n");
	gd->bd->bi_boot_params = 0x40000100;
	gd->bd->bi_dram[0].start = 0x40000000;
	gd->bd->bi_dram[0].size = 0x20000000;	
	params = (struct tag *)bd->bi_boot_params;
//	printf("params 0x%X bd->biparams 0x%X\n", params,bd->bi_boot_params);
	
	params->hdr.tag = ATAG_CORE;
	params->hdr.size = tag_size (tag_core);
	params->u.core.flags = 0;
	params->u.core.pagesize = 0;
	params->u.core.rootdev = 0;
	params = tag_next (params);
//	printf("params next 0x%X\n", params);
	
//	printf("cmdline %s len %ld\n", cmdline,strlen(cmdline));

	params->hdr.tag = ATAG_CMDLINE;
	params->hdr.size =
		(sizeof (struct tag_header) + strlen (cmdline) + 1 + 4) >> 2;
	strcpy (params->u.cmdline.cmdline, cmdline);
	params = tag_next (params);
//	printf("params next 0x%X\n", params);
	
	params->hdr.tag = ATAG_MEM;
	params->hdr.size = tag_size ( tag_mem32 );
	params->u.mem.start = bd->bi_dram[0].start;
	params->u.mem.size = bd->bi_dram[0].size;
	params = tag_next (params);
//	printf("params next 0x%X\n", params);

	params->hdr.tag = ATAG_NONE;
	params->hdr.size = 0;

	
#ifdef CONFIG_MACH_TYPE
	machid = CONFIG_MACH_TYPE;
//	printf("machid=%d\n", machid);
#endif
	//machid = gd->bd->bi_arch_number;
	r2 = gd->bd->bi_boot_params;
	
	spl_image.entry_point = spl_image.load_addr + 0x40;
	printf("entry_point 0x%X loadaddr 0x%X\n", spl_image.entry_point,spl_image.load_addr);
//	printf("Entering kernel arg pointer: 0x%X, machid 0x%X\n", r2,machid);
//	typedef void (*image_entry_arg_t)(int, int, void *)
//		__attribute__ ((noreturn));
//	image_entry_arg_t image_entry =
//		(image_entry_arg_t) spl_image.entry_point;
//	image_entry = 0x40008000;

/*
	void (*kernel_entry)(int zero, int arch, unsigned int params);

	//images.ep = 0x40008000;
//	kernel_entry = (void (*)(int, int, uint))0x40008000;

	kernel_entry = (void*)0x40008000;
	printf("## Transferring control to Linux (at address %08lx)" \
		"...\n", (ulong) kernel_entry);
	bootstage_mark(BOOTSTAGE_ID_RUN_OS);
	printf("\nStarting kernel ...%s\n\n", fake ?
		"(fake run for tracing)" : "");
	bootstage_mark_name(BOOTSTAGE_ID_BOOTM_HANDOFF, "start_kernel");	
	cleanup_before_linux();
	printf("image entry \n");	
	kernel_entry(0, machid, r2);

*/
	//image_entry(0, machid, r2);

	/* Nothing to do! */
}
#endif
u32 spl_boot_device(void)
{
	return BOOT_DEVICE_NAND;	
}

#ifdef CONFIG_SPL_BOARD_INIT
extern struct spl_image_info spl_image;
void spl_board_init(void)
{
	memset(&spl_image, 0, sizeof(spl_image));
	spl_image.flags = SPL_COPY_PAYLOAD_ONLY;
}
#endif

void board_init_f(unsigned long bootflag)
{
	__attribute__((noreturn)) void (*uboot)(void);
//	unsigned short int *dbg1 = (unsigned short int *)0xC0016002;
//	unsigned short int *dbg2 = (unsigned short int *)0xC0016082;

//	*dbg1 = 0;
//	*dbg2 = 0;
	unsigned char *val=(unsigned char*)0xC00A1000;
	*val = 0x00;
	arch_cpu_init();	
	nxp4330_spl_gpio_init();	
	preloader_console_init();


//	timer_init();
//	pl01x_serial_init();
	init_sound();
	play_bootsound();

	if (0x0 == *val){
		kernel_mode = 0x00;
//		printf("uboot load\n");
		spl_nand_load_image();
		
		dram_init();
		printf("jump 0x%X\n", CONFIG_SYS_TEXT_BASE);
		/* Jump to U-Boot image */
		uboot = (void *)CONFIG_SYS_TEXT_BASE;
		(*uboot)();
		/* Never returns Here */		
	}else{
		kernel_mode = 0x01;
		printf("kernel load\n");
		dram_init_banksize();
		board_init_r(NULL, 0);		
	}
}

	

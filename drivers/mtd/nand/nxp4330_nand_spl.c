/*
 * Copyright(c) 2017 I4VINE. by Juyoung Ryu. 
 *
 * SPDX-License-Identifier:     GPL-2.0+
 *
 * (C) Copyright 2010
 * KOO Bon-Gyu, Nexell Co, <freestyle@nexell.co.kr>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <common.h>
#include <errno.h>
#include <nand.h>

#include <asm/io.h>
#include <asm/arch/platform.h>
#include <asm/arch/mach-api.h>
#include <asm/arch/gpio.h>
#include <asm/gpio.h>

#include <asm/io.h>
#include <asm/errno.h>

#include "nxp4330_nand_ecc_spl.h"

#if	(0)
#define TM_DBGOUT(msg...)		{ printf(msg); }
#else
#define TM_DBGOUT(msg...)		do {} while (0)
#endif

#if	(0)
#define DBGOUT(msg...)		do { printf(msg); } while (0)
#else
#define DBGOUT(msg...)		do {} while (0)
#endif

#define ERROUT(msg...)		{ 					\
		printf(msg); }

#define CLEAR_RnB(r)							\
	r = NX_MCUS_GetInterruptPending(0);			\
	if (r) {									\
		NX_MCUS_ClearInterruptPending(0); 		\
		NX_MCUS_GetInterruptPending(0); 		\
	}

/*------------------------------------------------------------------------------
 * nand interface
 */
static void nand_select_chip(struct mtd_info *mtd, int chipnr)
{
	DBGOUT("%s, chipnr=%d\n", __func__, chipnr);

#if defined(CFG_NAND_OPTIONS)
	struct nand_chip *chip = mtd->priv;
	chip->options |= CFG_NAND_OPTIONS;
#endif

	if (chipnr > 4) {
		ERROUT("not support nand chip index %d\n", chipnr);
		return;
	}

	if (-1 == chipnr) {
		NX_MCUS_SetNFCSEnable(CFALSE);		// nand chip select control disable
	} else {
		NX_MCUS_SetNFBank(chipnr);
		NX_MCUS_SetNFCSEnable(CTRUE);
	}
}

#define MASK_CLE	0x10	/* NFCM   + 2C00_0000 */
#define MASK_ALE	0x18	/* NFADDR + 2C00_0000 */

static void nand_cmd_ctrl(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	struct nand_chip *chip = mtd->priv;
	void __iomem* addr = chip->IO_ADDR_W;
	int ret = 0;

	if (cmd == NAND_CMD_NONE)
		return;

	if (ctrl & NAND_CLE)
	{
		if (cmd != NAND_CMD_STATUS &&
			cmd != NAND_CMD_READID &&
			cmd != NAND_CMD_RESET)
			CLEAR_RnB(ret);

//		printf("command: %02x\n",(unsigned char)cmd);
		writeb(cmd, addr + MASK_CLE);
	}
	else if (ctrl & NAND_ALE)
	{
//		printf("address: %02x\n",(unsigned char)cmd);
		writeb(cmd, addr + MASK_ALE);
	}
}

struct nand_timings {
	uint32_t tACS;			// tACS
	uint32_t tCAH;			// tCAH
	uint32_t tCOS;			// tCOS
	uint32_t tOCH;			// tOCH
	uint32_t tACC;			// tACC
};

struct nand_timing_mode5_params {
	uint32_t tCS;
	uint32_t tCH;
	uint32_t tCLS_tALS;
	uint32_t tCLH_tALH;
	uint32_t tWP;
	uint32_t tWH;
	uint32_t tWC;
	uint32_t tDS;
	uint32_t tDH;
	uint32_t tCEA;		// max
	uint32_t tREA;		// max
	uint32_t tRP;
	uint32_t tREH;
	uint32_t tRC;
	uint32_t tCOH;
} NAND_TM5_PARAM[6] =
{//	tCS tCH tCLS tCLH tWP tWH  tWC tDS tDH tCEA tREA tRP tREH  tRC tCOH
	{70, 20,  50,  20, 50, 30, 100, 40, 20, 100,  40, 50,  30, 100,  0},		// mode 0
	{35, 10,  25,  10, 25, 15,  45, 20, 10,  45,  30, 25,  15,  50, 15},		// mode 1
	{25, 10,  15,  10, 17, 15,  35, 15,  5,  30,  25, 17,  15,  35, 15},		// mode 2
	{25,  5,  10,   5, 15, 10,  30, 10,  5,  25,  20, 15,  10,  30, 15},		// mode 3
	{20,  5,  10,   5, 12, 10,  25, 10,  5,  25,  20, 12,  10,  25, 15},		// mode 4
	{15,  5,  10,   5, 10,  7,  20,  7,  5,  25,  16, 10,   7,  20, 15}			// mode 5
};



/**
 * nand_calc_timing_mode - calculate based on the timing mode
 * @mode: timing mode. (0 ~ 5)
 * @clkhz: BCLK clock rate in hz
 * @timings: value to set the timing register [Returns]
 */
static int nand_calc_timing_mode (uint32_t mode, uint32_t clkhz,
			struct nand_timings *timings)
{
	uint32_t nclk; 
	uint32_t tCS, tCH, tCLS, tCLH, tWP, tWH, tWC, tDS, tDH, tCEA, tREA, tRP, tREH, tRC, tCOH;
	uint32_t tRCS, tWCS0, tWCS1, tRACC, tWACC, tRCH, tWCH0, tWCH1;
	uint32_t tCOS, tACC, tOCH;
	struct nand_timing_mode5_params *pntmp;

	// error check
	if(mode >= 6)
		mode = 0;
	if(clkhz < 1000000)		// BCLK is minimum > 1MHz
		clkhz = 1000000;
	if(!timings)
		return -1;

	// varient convertion
	nclk = 1000000000/(clkhz/1000);	// convert to pico second
	pntmp = &NAND_TM5_PARAM[mode];
	tCS = pntmp->tCS*1000000;
	tCH = pntmp->tCH*1000000;
	tCLS = pntmp->tCLS_tALS*1000000;
	tCLH = pntmp->tCLH_tALH*1000000;
	tWP = pntmp->tWP*1000000;
	tWH = pntmp->tWH*1000000;
	tWC = pntmp->tWC*1000000;
	tDS = pntmp->tDS*1000000;
	tDH = pntmp->tDH*1000000;
	tCEA = pntmp->tCEA*1000000;
	tREA = pntmp->tREA*1000000;
	tRP = pntmp->tRP*1000000;
	tREH = pntmp->tREH*1000000;
	tRC = pntmp->tRC*1000000;
	tCOH = pntmp->tCOH*1000000;

	TM_DBGOUT("nclk: %u, mode: %u\n", nclk, mode);
	TM_DBGOUT("tCS: %u, tCH: %u tCLS: %u, tCLH: %u, tWP: %u, tWH: %u, tWC: %u\n",
		tCS, tCH, tCLS, tCLH, tWP, tWH, tWC);
	TM_DBGOUT("tDS: %u, tDH: %u, tCEA: %u, tREA: %u,\n  tRP: %u, tREH: %u, tRC: %u, tCOH: %u\n", 
		tDS, tDH, tCEA, tREA, tRP, tREH, tRC, tCOH);

	// timing calculation
	tRCS = (tCEA-tREA)/nclk;	//(tCEA-tREA)/nclk
	tWCS0 = (tCS-tWP)/nclk;		//(tCS-tWP)/nclk
	tWCS1 = (tCLS-tWP)/nclk;	//(tCLS-tWP)/nclk
	tRACC = ((tREA+nclk*2000)>tRP?(tREA+nclk*2000):tRP)/nclk;	//MAX(tREA+nclk*2, tRP)/nclk
	tWACC = ((tWP>tDS)?tWP:tDS)/nclk;	//MAX(tWP,tDS)/nclk
	tRCH = ((tRC-tRP)>tREH?(tRC-tRP):tREH)/nclk-tRCS;	//MAX(tRC-tRP,tREH)/nclk-tRCS
	tWCH0 = ((tWC-tWP)>tWH?(tWC-tWP):tWH)/nclk-(tWCS0>tWCS1?tWCS0:tWCS1);//MAX(tWC-tWP, tWH)/nclk - MAX(tWCS0, tWCS1)
	tWCH1 = ((tCH>tCLH?tCH:tCLH)>tDH?(tCH>tCLH?tCH:tCLH):tDH)/nclk;		//MAX(tCH,tCLH,tDH)/nclk

	TM_DBGOUT("tRCS: %u, tWCS0: %u, tWCS1: %u, tRACC: %u, tWACC: %u, tRCH: %u, tWCH0: %u, tWCH1: %u\n",
		tRCS, tWCS0, tWCS1, tRACC, tWACC, tRCH, tWCH0, tWCH1);

	// convertion to clock base asynchronous nand controller state machine
	tCOS = (tRCS>tWCS0?tRCS:tWCS0)>tWCS1?(tRCS>tWCS0?tRCS:tWCS0):tWCS1;//MAX(tRCS, tWCS0, tWCS1);
	tACC = tRACC>tWACC?tRACC:tWACC;		//MAX(tRACC, tWACC);
	tOCH = (tRCH>tWCH0?tRCH:tWCH0)>tWCH1?(tRCH>tWCH0?tRCH:tWCH0):tWCH1;//MAX(tRCH, tWCH0, tWCH1);

	TM_DBGOUT("tCOS: %u, tACC: %u, tOCH: %u\n", tCOS, tACC, tOCH);

	// convert to register value
	tCOS += 999;	// round up tCOS
	tACC += 999;	// round up tACC
	tOCH += 999;	// round up tOCH

	// fillup paramter	
	timings->tACS = 0;
	timings->tCOS = tCOS/1000;
	timings->tACC = tACC/1000;
	timings->tOCH = tOCH/1000;
	timings->tCAH = 0;

	TM_DBGOUT("  fill - tCOS: %u, tACC: %u, tOCH: %u\n", tCOS/1000, tACC/1000, tOCH/1000);

	return 0;
}

static int nand_onfi_timing_set(struct mtd_info *mtd, uint32_t mode)
{
	struct clk *clk;
	uint32_t clkhz;
	struct nand_timings tmgs;
	int ret;


	clk = clk_get (NULL, CORECLK_NAME_BCLK), clkhz = clk_get_rate(clk), clk_put(clk);
//	TM_DBGOUT(" BCLK: %u HZ\n", clkhz);

	// setting - nand flash

	// setting - nand controller timming
	NX_MCUS_GetNANDBUSConfig
	(
		0,
		&tmgs.tACS,
		&tmgs.tCAH,
		&tmgs.tCOS,
		&tmgs.tOCH,
		&tmgs.tACC
	);
//	TM_DBGOUT("[BEFORE]  tACS: %u, tCAH: %u, tCOS: %u, tOCH: %u, tACC: %u\n", 
//		tmgs.tACS, tmgs.tCAH, tmgs.tCOS, tmgs.tOCH, tmgs.tACC);

	ret = nand_calc_timing_mode (mode, clkhz, &tmgs);
	if (ret < 0)
		return -1;

	NX_MCUS_SetNANDBUSConfig
	(
		0,
		tmgs.tACS,
		tmgs.tCAH,
		tmgs.tCOS,
		tmgs.tOCH,
		tmgs.tACC
	);

	NX_MCUS_GetNANDBUSConfig
	(
		0,
		&tmgs.tACS,
		&tmgs.tCAH,
		&tmgs.tCOS,
		&tmgs.tOCH,
		&tmgs.tACC
	);
	TM_DBGOUT("[AFTER]  tACS: %u, tCAH: %u, tCOS: %u, tOCH: %u, tACC: %u\n", 
		tmgs.tACS, tmgs.tCAH, tmgs.tCOS, tmgs.tOCH, tmgs.tACC);

//	printf("time set \n");
	return 0;
}

/* timing set */
static int nexell_nand_timing_set(struct mtd_info *mtd)
{
#if 1
	struct nand_chip *chip = mtd->priv;
	uint32_t ret = ONFI_TIMING_MODE_UNKNOWN, mode;

#ifdef CONFIG_SYS_NAND_ONFI_DETECTION
	ret = onfi_get_async_timing_mode(chip);
#endif
	if (ret == ONFI_TIMING_MODE_UNKNOWN)
	{
		NX_MCUS_SetNANDBUSConfig
		(
			 0, /* NF */
			 CFG_SYS_NAND_TACS,              // tACS  ( 0 ~ 3 )
			 CFG_SYS_NAND_TCAH,              // tCAH  ( 0 ~ 3 )
			 CFG_SYS_NAND_TCOS,              // tCOS  ( 0 ~ 3 )
			 CFG_SYS_NAND_TCOH,              // tCOH  ( 0 ~ 3 )
			 CFG_SYS_NAND_TACC               // tACC  ( 1 ~ 16)
		);

		return 0;
	}

	mode = fls(ret) - 1;
//	TM_DBGOUT("ONFI TIMING MODE (%d) \n", mode);

	nand_onfi_timing_set (mtd, mode);
#endif
	return 0;
}


/*
 * Enable NAND write protect
 */
static void nxp_wp_enable(void)
{
	nxp_gpio_set_value(CFG_IO_NAND_nWP, 0);
}

/*
 * Disable NAND write protect
 */
static void nxp_wp_disable(void)
{
	nxp_gpio_set_value(CFG_IO_NAND_nWP, 1);
}

int nand_dev_ready(struct mtd_info *mtd)
{
	int ret = 0;
//	printf("nand_dev_ready\n");
	CLEAR_RnB(ret);
	DBGOUT("[%s, RnB=%d]\n", ret?"READY":"BUSY", NX_MCUS_IsNFReady());
	return ret;
}

static void nx_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	struct nand_chip *chip = mtd->priv;
	uint8_t *p;
	int i;

	p = buf;
	for (i=0; i< len >> 2; i++, p+=4)
		*((uint32_t *)p) = readl(chip->IO_ADDR_R);
	if (len & 3) {
		for (i=0; i< (len & 3); i++, p++)
			*p = readb(chip->IO_ADDR_R);
	}
}

static void nx_write_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	struct nand_chip *chip = mtd->priv;
	uint8_t *p;
	int i;
	
	p = buf;
	for (i=0; i< len >> 2; i++, p+=4)
		writel(*((uint32_t *)p), chip->IO_ADDR_W);
	if (len & 3) {
		for (i=0; i< (len & 3); i++, p++)
			writeb(*p, chip->IO_ADDR_W);
	}
}


static void nand_dev_init(struct mtd_info *mtd)
{
	NX_MCUS_SetAutoResetEnable(CTRUE);
	NX_MCUS_ClearInterruptPending(0);
	NX_MCUS_SetInterruptEnableAll(CFALSE);
	NX_MCUS_SetNFBank(0);
	NX_MCUS_SetNFCSEnable(CFALSE);

	nxp_gpio_direction_output (CFG_IO_NAND_nWP, 1);
}

/*------------------------------------------------------------------------------
 * u-boot nand module
 */
#if defined (CONFIG_MTD_NAND_ECC_BCH)
static uint8_t *verify_page;
static int nand_bch_write_page(struct mtd_info *mtd, struct nand_chip *chip,
			   const uint8_t *buf, int oob_required, int page, int cached, int raw)
{
#ifdef CONFIG_MTD_NAND_VERIFY_WRITE
	int ret = 0;
#endif
	int status;

	DBGOUT("%s page %d, raw=%d\n", __func__, page, raw);
	chip->cmdfunc(mtd, NAND_CMD_SEQIN, 0x00, page);

	/* not verify */
	if (raw)
		chip->ecc.write_page_raw(mtd, chip, buf, oob_required);
	else
		chip->ecc.write_page(mtd, chip, buf, oob_required);

	/*
	 * Cached progamming disabled for now, Not sure if its worth the
	 * trouble. The speed gain is not very impressive. (2.3->2.6Mib/s)
	 */
	cached = 0;

	if (!cached || !(chip->options & NAND_CACHEPRG)) {

		chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);
		status = chip->waitfunc(mtd, chip);
		/*
		 * See if operation failed and additional status checks are
		 * available
		 */
		if ((status & NAND_STATUS_FAIL) && (chip->errstat))
			status = chip->errstat(mtd, chip, FL_WRITING, status,
					       page);

		if (status & NAND_STATUS_FAIL)
			return -EIO;
	} else {
		chip->cmdfunc(mtd, NAND_CMD_CACHEDPROG, -1, -1);
		status = chip->waitfunc(mtd, chip);
	}

#ifdef CONFIG_MTD_NAND_VERIFY_WRITE
	if (raw)
		return 0;

	/* Send command to read back the data */
	chip->cmdfunc(mtd, NAND_CMD_READ0, 0, page);
	chip->ecc.read_page(mtd, chip, (uint8_t *)verify_page, oob_required, page);
	if (ret < 0)
		return -EIO;

	if (memcmp (verify_page, buf, mtd->writesize))
	{
		ERROUT ("%s fail verify %d page\n", __func__, page);
		return -EIO;
	}

	chip->cmdfunc(mtd, NAND_CMD_STATUS, -1, -1);
#endif
	return 0; // mtd->ecc_stats.corrected - stats.corrected ? -EUCLEAN : 0
}

static int nand_ecc_layout_swbch(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct nand_ecclayout *layout = &chip->ecc.layout[0];
	struct nand_oobfree *oobfree  = layout->oobfree;
	int ecctotal = chip->ecc.total;
	int oobsize	 = mtd->oobsize;
	unsigned char buf[mtd->writesize];

	printf("sw bch ecc %d bit, oob %2d, bad '0,1', ecc %d~%d (%d), free %d~%d (%d) ",
		ECC_BCH_BITS, oobsize, oobfree->offset+oobfree->length, oobsize-1, ecctotal,
		oobfree->offset, oobfree->length + 1, oobfree->length);
	memset(buf,0,sizeof(buf));
	verify_page = &buf;//kzalloc(mtd->writesize, GFP_KERNEL);
	if (!verify_page)
		return -ENOMEM;

	return 0;
}
#endif

#ifdef CONFIG_SYS_NAND_SELECT_DEVICE
int nand_ecc_layout_check(struct mtd_info *mtd)
{
	int ret = 0;
#if defined (CONFIG_MTD_NAND_ECC_HW)
	ret = nand_ecc_layout_hwecc(mtd);
#elif defined (CONFIG_MTD_NAND_ECC_BCH)
	ret = nand_ecc_layout_swbch(mtd);
#endif
	return ret;
}
#endif

static int nand_spl_ecc_alloc_buffer(struct mtd_info *mtd)
{
	int ret = 0;
//	unsigned char buf[mtd->writesize];
//	unsigned char buf1[NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE]={0};

#ifdef CONFIG_NAND_RANDOMIZER
	memset(buf,0,sizeof(buf));
	pages_per_block_mask = (mtd->erasesize/mtd->writesize) - 1;

	randomize_buf = &buf;//kzalloc(mtd->writesize, GFP_KERNEL);
	if (!randomize_buf) {
		ERROUT("randomize buffer alloc failed\n");
	}
	printf("    [%s:%d] randomize_buf: %p, mtd->writesize: %d, pages_per_block_mask: %x\n",
	//	__func__, __LINE__, randomize_buf, mtd->writesize, pages_per_block_mask);


	// kfree ...
#endif

#ifdef CONFIG_MTD_NAND_VERIFY_WRITE
		memset(buf1,0,sizeof(buf1));
	verify_buf = &buf1;// kmalloc(NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE, GFP_KERNEL);

	// kfree
#endif
	
	return ret;
}

#ifdef CONFIG_SYS_NAND_SELECT_DEVICE
int nand_ecc_post_scan(struct mtd_info *mtd)
{
	int ret = 0;

	ret = nand_ecc_layout_check(mtd);
#ifdef CONFIG_MTD_NAND_ECC_HW
	nand_spl_ecc_alloc_buffer(mtd);
#endif

	return ret;
}
#endif

/* Wait for the ready pin, after a command. The timeout is caught later. */
void nand_wait_ready(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	u32 timeo = (CONFIG_SYS_HZ * 20) / 1000;
	u32 time_start;

	time_start = get_timer(0);
	/* Wait until command is processed or timeout occurs */
	while (get_timer(time_start) < timeo) {
		if (chip->dev_ready)
			if (chip->dev_ready(mtd))
				break;
	}
}
EXPORT_SYMBOL_GPL(nand_wait_ready);


/**
 * nand_command_lp - [DEFAULT] Send command to NAND large page device
 * @mtd: MTD device structure
 * @command: the command to be sent
 * @column: the column address for this command, -1 if none
 * @page_addr: the page address for this command, -1 if none
 *
 * Send command to NAND device. This is the version for the new large page
 * devices. We don't have the separate regions as we have in the small page
 * devices. We must emulate NAND_CMD_READOOB to keep the code compatible.
 */
 void nxp4330_nand_command(struct mtd_info *mtd, unsigned int command,
			    int column, int page_addr)
{
	register struct nand_chip *chip = mtd->priv; 

	/* Emulate NAND_CMD_READOOB */
	if (command == NAND_CMD_READOOB) {
		column += mtd->writesize;
		command = NAND_CMD_READ0;
	}

	/* Command latch cycle */
	chip->cmd_ctrl(mtd, command, NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);

	if (column != -1 || page_addr != -1) {
		int ctrl = NAND_CTRL_CHANGE | NAND_NCE | NAND_ALE;

		/* Serially input address */
		if (column != -1) {
			/* Adjust columns for 16 bit buswidth */
			if (chip->options & NAND_BUSWIDTH_16 &&
					!nand_opcode_8bits(command))
				column >>= 1;
			chip->cmd_ctrl(mtd, column, ctrl);
			ctrl &= ~NAND_CTRL_CHANGE;
			chip->cmd_ctrl(mtd, column >> 8, ctrl);
		}
		if (page_addr != -1) {
			chip->cmd_ctrl(mtd, page_addr, ctrl);
			chip->cmd_ctrl(mtd, page_addr >> 8,
				       NAND_NCE | NAND_ALE);
			/* One more address cycle for devices > 128MiB */
			if (chip->chipsize > (128 << 20))
				chip->cmd_ctrl(mtd, page_addr >> 16,
					       NAND_NCE | NAND_ALE);
		}
	}
	chip->cmd_ctrl(mtd, NAND_CMD_NONE, NAND_NCE | NAND_CTRL_CHANGE);

	/*
	 * Program and erase have their own busy handlers status, sequential
	 * in and status need no delay.
	 */
	switch (command) {

	case NAND_CMD_CACHEDPROG:
	case NAND_CMD_PAGEPROG:
	case NAND_CMD_ERASE1:
	case NAND_CMD_ERASE2:
	case NAND_CMD_SEQIN:
	case NAND_CMD_RNDIN:
	case NAND_CMD_STATUS:
		return;

	case NAND_CMD_RESET:
#if 0
		if (chip->dev_ready)
			break;
		udelay(chip->chip_delay);
		chip->cmd_ctrl(mtd, NAND_CMD_STATUS,
			       NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
		chip->cmd_ctrl(mtd, NAND_CMD_NONE,
			       NAND_NCE | NAND_CTRL_CHANGE);
		/* EZ-NAND can take upto 250ms as per ONFi v4.0 */
		nand_wait_status_ready(mtd, 250);
		return;
#endif
		break;
	case NAND_CMD_RNDOUT:
		/* No ready / busy check necessary */
		chip->cmd_ctrl(mtd, NAND_CMD_RNDOUTSTART,
			       NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
		chip->cmd_ctrl(mtd, NAND_CMD_NONE,
			       NAND_NCE | NAND_CTRL_CHANGE);
		return;

	case NAND_CMD_READ0:
		chip->cmd_ctrl(mtd, NAND_CMD_READSTART,
			       NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
		chip->cmd_ctrl(mtd, NAND_CMD_NONE,
			       NAND_NCE | NAND_CTRL_CHANGE);

		/* This applies to read commands */
	default:
		/*
		 * If we don't have access to the busy pin, we apply the given
		 * command delay.
		 */
		if (!chip->dev_ready) {
			udelay(chip->chip_delay);
			return;
		}
	}

	/*
	 * Apply this short delay always to ensure that we do wait tWB in
	 * any case on any machine.
	 */
	ndelay(100);

	nand_wait_ready(mtd);

}

/*

 * calling from nand_init (drivers/mtd/nand/nand.c)
 */
#ifdef CONFIG_SYS_NAND_SELF_INIT
static struct nand_chip nand_chip[CONFIG_SYS_MAX_NAND_DEVICE];
void board_nand_init(void)
{
	struct nand_chip *chip = &nand_chip[0];
#else
int board_nand_init(struct nand_chip *chip)
{
	memset(chip,0,sizeof(struct nand_chip));	
#endif
	struct mtd_info  *mtd = &nand_info[0];
	int ret = 0;
	mtd->priv = chip;
	DBGOUT("board_nand_init mtd 0x%X nand_info 0x%X\n",mtd,&nand_info[0]);
	nand_dev_init(mtd);

	/*
	* nand callbacks
	*/
	chip->IO_ADDR_R 	= (void __iomem *)CONFIG_SYS_NAND_BASE;
	chip->IO_ADDR_W 	= (void __iomem *)CONFIG_SYS_NAND_BASE;
	chip->cmd_ctrl 		= nand_cmd_ctrl;
	chip->cmdfunc = nxp4330_nand_command;
	chip->dev_ready 	= nand_dev_ready;
	chip->select_chip 	= nand_select_chip;
	chip->read_buf 	= nx_read_buf;
	//chip->write_buf 	= nx_write_buf;
	chip->chip_delay 	= 15;
	chip->options       = 0;	
	chip->options		&= ~NAND_NEED_READRDY;
	//chip->options |= NAND_HWECC_READECC_FIRST | NAND_HWECC_WRITE_ORGECCCODE	
#if defined (CONFIG_MTD_NAND_ECC_BCH)
	chip->write_page	= nand_bch_write_page;
#endif

	/*
	 * error correct mode
	 */
#if   defined (CONFIG_MTD_NAND_ECC_HW)
	ret = nand_hw_ecc_init_device(mtd);

	printf("NAND ecc: Hardware (delay %d)\n", chip->chip_delay);
#elif defined (CONFIG_MTD_NAND_ECC_BCH)
	chip->ecc.mode = NAND_ECC_SOFT_BCH;

	/* refer to nand_ecc.c */
	switch (ECC_BCH_BITS) {
	case  4: chip->ecc.bytes =   7; chip->ecc.size  =  512; break;
	case  8: chip->ecc.bytes =  13; chip->ecc.size  =  512; break;
    case 12: chip->ecc.bytes =  20; chip->ecc.size  =  512; break;
	case 16: chip->ecc.bytes =  26; chip->ecc.size  =  512; break;
	case 24: chip->ecc.bytes =  42; chip->ecc.size  = 1024; break;
	case 40: chip->ecc.bytes =  70; chip->ecc.size  = 1024; break;
	//case 60: chip->ecc.bytes = 105; chip->ecc.size  = 1024; break;	/* not test */
	default:
		printf("Fail: not supoort bch ecc %d mode !!!\n", ECC_BCH_BITS);
		return -1;
	}
	printf("NAND ecc: Software BCH %d \n", ECC_BCH_BITS);
#else
	chip->ecc.mode = NAND_ECC_SOFT;
	printf("NAND ecc: Software \n");
#endif

	nexell_nand_timing_set(mtd);
#ifdef CONFIG_SYS_NAND_SELF_INIT
	return;
#else
	return ret;
#endif
}



static int nand_is_bad_block(struct mtd_info *mtd, int block)
{
	struct nand_chip *chip = mtd->priv;
	u_char bb_data[2];

	chip->cmdfunc(mtd, NAND_CMD_READ0, (1 << chip->page_shift), block * (CONFIG_SYS_NAND_BLOCK_SIZE/CONFIG_SYS_NAND_PAGE_SIZE));
	/*
	 * Read one byte (or two if it's a 16 bit chip).
	 */
	 DBGOUT("chip->options 0x%X\n", chip->options);
	 if (chip->options & NAND_BUSWIDTH_16) {
	 	 chip->read_buf(mtd, bb_data, 1);
	 	 DBGOUT("busw 16 bad[0] 0x%X bad[1] 0x%X\n", bb_data[0],bb_data[1]);
	 	 if (bb_data[0] != 0xff || bb_data[1] != 0xff)
	 	 	 return 1;
	 } else {
	 	 chip->read_buf(mtd, bb_data, 2);	
	 	 DBGOUT("busw 8 bad[0] 0x%X bad[1] 0x%X\n", bb_data[0],bb_data[1]);
	 	 if (bb_data[0] != 0xff)
	 	 	 return 1;
	 }
	 DBGOUT("nand_is_no bad_block \n");	
	 return 0;
}
extern unsigned char boot_mode_sel(void);
int spl_start_uboot(void)
{
	unsigned char mode=0;
	mode = boot_mode_sel();

	if(mode == 0x00){
//		printf("uboot mode 0x%X\n", mode);		
		return 1; //uboot
	} else {
		return 0;	//kernel
	}
//	int boot_mode = 0; //mode 0 : uboot, mode 1: kernel
//	if(boot_mode == 0)
//		return 1;
	return 0;
}

extern nand_info_t nand_info[CONFIG_SYS_MAX_NAND_DEVICE];
static int load_from_nand(int dev, unsigned int offs,unsigned int size, void *dst)
{
//	int i=0;
	unsigned int block,lastblock,page;
	struct mtd_info *mtd = &nand_info[dev];
//	size_t retlen;
	struct nand_chip *chip;
//	void *dst = (void*)CONFIG_SYS_NAND_U_BOOT_DST;//0x80f00000;
	chip = mtd->priv;
	
	block = offs/CONFIG_SYS_NAND_BLOCK_SIZE;
	lastblock = (offs+size-1)/CONFIG_SYS_NAND_BLOCK_SIZE;
	page = (offs % CONFIG_SYS_NAND_BLOCK_SIZE)/CONFIG_SYS_NAND_PAGE_SIZE;
//	printf("load addr 0x%X, block %d lastblock %d page %d\n", dst,block,lastblock,page);
	
	NX_MCUS_SetNFCSEnable(CTRUE);
//	printf("enable pagecnt %d\n",CONFIG_SYS_NAND_PAGE_COUNT);
	while(block <= lastblock){
		if(!nand_is_bad_block(mtd, block)){
			while(page < CONFIG_SYS_NAND_PAGE_COUNT) {
				nxp4330_nand_hw_ecc_read_page(mtd,chip,dst,0,((block*CONFIG_SYS_NAND_BLOCK_SIZE)/CONFIG_SYS_NAND_PAGE_SIZE)+page);
				dst += CONFIG_SYS_NAND_PAGE_SIZE;
				page ++;
			//	printf("dst 0x%X,block %d page %d\n", dst, block,page);
			//	printf("0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X ",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7]);
					
			}
			page = 0;
		} else {
			lastblock++;
			printf("badblock lastblock %d\n", lastblock);
		}
		block++;
//		printf("block %d\n", block);
	}
	NX_MCUS_SetNFCSEnable(CFALSE);
//	printf("load done\n");

	return 0;
}

int nand_spl_load_image(unsigned int offs, unsigned int size, void* dst)
{
	int i=0;

	if(size == 0x40){
		if(spl_start_uboot() ==  0){
		//	printf("kernel load\n");
		} else {
			printf("u-boot mode\n");
		//	printf("we don't need to load uboot header\n");
			return 0;
		}
	}
	unsigned int *dbg1 = (unsigned short int *)0x4C000000;		
	for(i=0; i<10; i++){
	//	printf("dbg 0x%x *dbg1 0x%X \n",dbg1,*dbg1);		
		*dbg1 = 0;
	//	printf("dbg 0x%x *dbg1 0x%X \n",dbg1,*dbg1);	
		dbg1++;

	}
	DBGOUT("nand_spl_laod_image size %d dst 0x%X\n", size,dst);

	
	load_from_nand(0,offs,size,(void*)dst);

	return 0;
	
}

void nand_deselect(void)
{
		
}

#ifdef CONFIG_SYS_NAND_SELECT_DEVICE
void board_nand_select_device(struct nand_chip *chip, int dev)
{
	struct mtd_info  *mtd = &nand_info[0];
	chip = mtd->priv;
	printf("select_device dev 0x%X mtd 0x%X, mtd->priv 0x%X, nand 0x%X\n",0, mtd,mtd->priv, chip);
	/* This is not optimal place of post scan, but we have to.
	  After BBT scanning we should have to something to for ECC,
	  mandatory work to do. */
	nand_ecc_post_scan(mtd);
}
#endif

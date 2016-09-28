/*
 * sunxi-nand mtd driver
 * Copyright (C) 2016 I4VINE Inc.,
 * All right reserved by ryu <jyryu@stcube.com> and following authurs
 * Copyright(c) 2016 STcube Inc.
 * All right reserved by Seungwoo Kim <ksw@stcube.com> and following authurs
 *
 * Copyright (C) 2013 Boris BREZILLON <b.brezillon.dev@gmail.com>
 *
 * Derived from:
 *	https://github.com/yuq/sunxi-nfc-mtd
 *	Copyright (C) 2013 Qiang Yu <yuq825@gmail.com>
 *
 *	https://github.com/hno/Allwinner-Info
 *	Copyright (C) 2013 Henrik Nordström <Henrik Nordström>
 *
 *	Copyright (C) 2013 Dmitriy B. <rzk333@gmail.com>
 *	Copyright (C) 2013 Sergey Lapin <slapin@ossfans.org>
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
 */
#include "sunxi_nand.h"

#ifndef BITS_PER_BYTE
#define BITS_PER_BYTE	8
#endif

#ifndef NSEC_PER_SEC
#define NSEC_PER_SEC		1000000000ULL
#endif

#if !STRUCT_NANDCHIP_HAVE_ONFI_TIMING
/* ONFI timing mode, used in both asynchronous and synchronous mode */
#define ONFI_TIMING_MODE_0              (1 << 0)
#define ONFI_TIMING_MODE_1              (1 << 1)
#define ONFI_TIMING_MODE_2              (1 << 2)
#define ONFI_TIMING_MODE_3              (1 << 3)
#define ONFI_TIMING_MODE_4              (1 << 4)
#define ONFI_TIMING_MODE_5              (1 << 5)
#define ONFI_TIMING_MODE_UNKNOWN        (1 << 6)

#define ONFI_FEATURE_ADDR_TIMING_MODE   0x1
#define ONFI_SUBFEATURE_PARAM_LEN       4

#if !NAND_SDR_TIMINGS_DEFINED
struct nand_sdr_timings {
        u32 tALH_min;
        u32 tADL_min;
        u32 tALS_min;
        u32 tAR_min;
        u32 tCEA_max;
        u32 tCEH_min;
        u32 tCH_min;
        u32 tCHZ_max;
        u32 tCLH_min;
        u32 tCLR_min;
        u32 tCLS_min;
        u32 tCOH_min;
        u32 tCS_min;
        u32 tDH_min;
        u32 tDS_min;
        u32 tFEAT_max;
        u32 tIR_min;
        u32 tITC_max;
        u32 tRC_min;
        u32 tREA_max;
        u32 tREH_min;
        u32 tRHOH_min;
        u32 tRHW_min;
        u32 tRHZ_max;
        u32 tRLOH_min;
        u32 tRP_min;
        u32 tRR_min;
        u64 tRST_max;
        u32 tWB_max;
        u32 tWC_min;
        u32 tWH_min;
        u32 tWHR_min;
        u32 tWP_min;
        u32 tWW_min;
};
#endif

static const struct nand_sdr_timings onfi_sdr_timings[] = {
	/* Mode 0 */
	{
		.tADL_min = 200000,
		.tALH_min = 20000,
		.tALS_min = 50000,
		.tAR_min = 25000,
		.tCEA_max = 100000,
		.tCEH_min = 20000,
		.tCH_min = 20000,
		.tCHZ_max = 100000,
		.tCLH_min = 20000,
		.tCLR_min = 20000,
		.tCLS_min = 50000,
		.tCOH_min = 0,
		.tCS_min = 70000,
		.tDH_min = 20000,
		.tDS_min = 40000,
		.tFEAT_max = 1000000,
		.tIR_min = 10000,
		.tITC_max = 1000000,
		.tRC_min = 100000,
		.tREA_max = 40000,
		.tREH_min = 30000,
		.tRHOH_min = 0,
		.tRHW_min = 200000,
		.tRHZ_max = 200000,
		.tRLOH_min = 0,
		.tRP_min = 50000,
		.tRST_max = 250000000000ULL,
		.tWB_max = 200000,
		.tRR_min = 40000,
		.tWC_min = 100000,
		.tWH_min = 30000,
		.tWHR_min = 120000,
		.tWP_min = 50000,
		.tWW_min = 100000,
	},
	/* Mode 1 */
	{
		.tADL_min = 100000,
		.tALH_min = 10000,
		.tALS_min = 25000,
		.tAR_min = 10000,
		.tCEA_max = 45000,
		.tCEH_min = 20000,
		.tCH_min = 10000,
		.tCHZ_max = 50000,
		.tCLH_min = 10000,
		.tCLR_min = 10000,
		.tCLS_min = 25000,
		.tCOH_min = 15000,
		.tCS_min = 35000,
		.tDH_min = 10000,
		.tDS_min = 20000,
		.tFEAT_max = 1000000,
		.tIR_min = 0,
		.tITC_max = 1000000,
		.tRC_min = 50000,
		.tREA_max = 30000,
		.tREH_min = 15000,
		.tRHOH_min = 15000,
		.tRHW_min = 100000,
		.tRHZ_max = 100000,
		.tRLOH_min = 0,
		.tRP_min = 25000,
		.tRR_min = 20000,
		.tRST_max = 500000000,
		.tWB_max = 100000,
		.tWC_min = 45000,
		.tWH_min = 15000,
		.tWHR_min = 80000,
		.tWP_min = 25000,
		.tWW_min = 100000,
	},
	/* Mode 2 */
	{
		.tADL_min = 100000,
		.tALH_min = 10000,
		.tALS_min = 15000,
		.tAR_min = 10000,
		.tCEA_max = 30000,
		.tCEH_min = 20000,
		.tCH_min = 10000,
		.tCHZ_max = 50000,
		.tCLH_min = 10000,
		.tCLR_min = 10000,
		.tCLS_min = 15000,
		.tCOH_min = 15000,
		.tCS_min = 25000,
		.tDH_min = 5000,
		.tDS_min = 15000,
		.tFEAT_max = 1000000,
		.tIR_min = 0,
		.tITC_max = 1000000,
		.tRC_min = 35000,
		.tREA_max = 25000,
		.tREH_min = 15000,
		.tRHOH_min = 15000,
		.tRHW_min = 100000,
		.tRHZ_max = 100000,
		.tRLOH_min = 0,
		.tRR_min = 20000,
		.tRST_max = 500000000,
		.tWB_max = 100000,
		.tRP_min = 17000,
		.tWC_min = 35000,
		.tWH_min = 15000,
		.tWHR_min = 80000,
		.tWP_min = 17000,
		.tWW_min = 100000,
	},
	/* Mode 3 */
	{
		.tADL_min = 100000,
		.tALH_min = 5000,
		.tALS_min = 10000,
		.tAR_min = 10000,
		.tCEA_max = 25000,
		.tCEH_min = 20000,
		.tCH_min = 5000,
		.tCHZ_max = 50000,
		.tCLH_min = 5000,
		.tCLR_min = 10000,
		.tCLS_min = 10000,
		.tCOH_min = 15000,
		.tCS_min = 25000,
		.tDH_min = 5000,
		.tDS_min = 10000,
		.tFEAT_max = 1000000,
		.tIR_min = 0,
		.tITC_max = 1000000,
		.tRC_min = 30000,
		.tREA_max = 20000,
		.tREH_min = 10000,
		.tRHOH_min = 15000,
		.tRHW_min = 100000,
		.tRHZ_max = 100000,
		.tRLOH_min = 0,
		.tRP_min = 15000,
		.tRR_min = 20000,
		.tRST_max = 500000000,
		.tWB_max = 100000,
		.tWC_min = 30000,
		.tWH_min = 10000,
		.tWHR_min = 80000,
		.tWP_min = 15000,
		.tWW_min = 100000,
	},
	/* Mode 4 */
	{
		.tADL_min = 70000,
		.tALH_min = 5000,
		.tALS_min = 10000,
		.tAR_min = 10000,
		.tCEA_max = 25000,
		.tCEH_min = 20000,
		.tCH_min = 5000,
		.tCHZ_max = 30000,
		.tCLH_min = 5000,
		.tCLR_min = 10000,
		.tCLS_min = 10000,
		.tCOH_min = 15000,
		.tCS_min = 20000,
		.tDH_min = 5000,
		.tDS_min = 10000,
		.tFEAT_max = 1000000,
		.tIR_min = 0,
		.tITC_max = 1000000,
		.tRC_min = 25000,
		.tREA_max = 20000,
		.tREH_min = 10000,
		.tRHOH_min = 15000,
		.tRHW_min = 100000,
		.tRHZ_max = 100000,
		.tRLOH_min = 5000,
		.tRP_min = 12000,
		.tRR_min = 20000,
		.tRST_max = 500000000,
		.tWB_max = 100000,
		.tWC_min = 25000,
		.tWH_min = 10000,
		.tWHR_min = 80000,
		.tWP_min = 12000,
		.tWW_min = 100000,
	},
	/* Mode 5 */
	{
		.tADL_min = 70000,
		.tALH_min = 5000,
		.tALS_min = 10000,
		.tAR_min = 10000,
		.tCEA_max = 25000,
		.tCEH_min = 20000,
		.tCH_min = 5000,
		.tCHZ_max = 30000,
		.tCLH_min = 5000,
		.tCLR_min = 10000,
		.tCLS_min = 10000,
		.tCOH_min = 15000,
		.tCS_min = 15000,
		.tDH_min = 5000,
		.tDS_min = 7000,
		.tFEAT_max = 1000000,
		.tIR_min = 0,
		.tITC_max = 1000000,
		.tRC_min = 20000,
		.tREA_max = 16000,
		.tREH_min = 7000,
		.tRHOH_min = 15000,
		.tRHW_min = 100000,
		.tRHZ_max = 100000,
		.tRLOH_min = 5000,
		.tRP_min = 10000,
		.tRR_min = 20000,
		.tRST_max = 500000000,
		.tWB_max = 100000,
		.tWC_min = 20000,
		.tWH_min = 7000,
		.tWHR_min = 80000,
		.tWP_min = 10000,
		.tWW_min = 100000,
	},
};
#endif

static inline struct sunxi_nand_chip *to_sunxi_nand(struct nand_chip *nand)
{
	struct sunxi_nfc *nfc = nand->priv;
	
	return nfc->chips[0];
}

static inline struct sunxi_nand_chip *mtd_to_sunxi_nand(struct mtd_info *mtd)
{
	struct nand_chip *nand = mtd->priv;
	
	return to_sunxi_nand(nand);
}

static inline struct sunxi_nfc *to_sunxi_nfc(struct mtd_info *mtd)
{
	struct nand_chip *nand = mtd->priv;
	
	return (struct sunxi_nfc *)nand->priv;
}

#if !(U_BOOT_NAND)
irqreturn_t sunxi_nfc_interrupt(int irq, void *dev_id)
{
	struct sunxi_nfc *nfc = dev_id;
	u32 st = readl(nfc->regs + NFC_REG_ST);
	u32 ien = readl(nfc->regs + NFC_REG_INT);

	if (!(ien & st))
		return IRQ_NONE;

	if ((ien & st) == ien)
		complete(&nfc->complete);

	writel(st & NFC_INT_MASK, nfc->regs + NFC_REG_ST);
	writel(~st & ien & NFC_INT_MASK, nfc->regs + NFC_REG_INT);

	return IRQ_HANDLED;
}

static int sunxi_nfc_wait_int(struct sunxi_nfc *nfc, u32 flags,
			      unsigned int timeout_ms)
{
	init_completion(&nfc->complete);

	writel(flags, nfc->regs + NFC_REG_INT);

	if (!timeout_ms)
		timeout_ms = NFC_DEFAULT_TIMEOUT_MS;

	if (!wait_for_completion_timeout(&nfc->complete,
					 msecs_to_jiffies(timeout_ms))) {
		dev_err(nfc->dev, "wait interrupt timedout\n");
		return -ETIMEDOUT;
	}

	return 0;
}
#else
/* Waiting function to polling nand flags? */
static int sunxi_nfc_wait_int(struct sunxi_nfc *nfc, u32 flags,
			      unsigned int timeout_ms)
{
	unsigned int st, ien;

	writel(flags, nfc->regs + NFC_REG_INT);
	while (1) {
		st = readl(nfc->regs + NFC_REG_ST);
		ien = readl(nfc->regs + NFC_REG_INT);
		if ((st & ien) == ien)
			break;
		ndelay(1);
		timeout_ms--;
	}
	return 0;
}
#endif

#if  !(U_BOOT_NAND)
static int sunxi_nfc_wait_cmd_fifo_empty(struct sunxi_nfc *nfc)
{
	unsigned long timeout = jiffies +
				msecs_to_jiffies(NFC_DEFAULT_TIMEOUT_MS);

	do {
		if (!(readl(nfc->regs + NFC_REG_ST) & NFC_CMD_FIFO_STATUS))
			return 0;
	} while (time_before(jiffies, timeout));

	dev_err(nfc->dev, "wait for empty cmd FIFO timedout\n");
	return -ETIMEDOUT;
}

int sunxi_nfc_rst(struct sunxi_nfc *nfc)
{
	unsigned long timeout = jiffies +
				msecs_to_jiffies(NFC_DEFAULT_TIMEOUT_MS);

	writel(0, nfc->regs + NFC_REG_ECC_CTL);
	writel(NFC_RESET, nfc->regs + NFC_REG_CTL);

	do {
		if (!(readl(nfc->regs + NFC_REG_CTL) & NFC_RESET))
			return 0;
	} while (time_before(jiffies, timeout));

	dev_err(nfc->dev, "wait for NAND controller reset timedout\n");
	return -ETIMEDOUT;
}

#else
static int sunxi_nfc_wait_cmd_fifo_empty(struct sunxi_nfc *nfc)
{
	unsigned long timeout = NFC_DEFAULT_TIMEOUT_MS * 1000;

	do {
		if (!(readl(nfc->regs + NFC_REG_ST) & NFC_CMD_FIFO_STATUS))
			return 0;
	} while (timeout--);

	dev_err(nfc->dev, "wait for empty cmd FIFO timedout\n");
	return -ETIMEDOUT;
}

int sunxi_nfc_rst(struct sunxi_nfc *nfc)
{
	unsigned long timeout = NFC_DEFAULT_TIMEOUT_MS * 1000;

	writel(0, nfc->regs + NFC_REG_ECC_CTL);
	writel(NFC_RESET, nfc->regs + NFC_REG_CTL);

	do {
		if (!(readl(nfc->regs + NFC_REG_CTL) & NFC_RESET))
			return 0;
	} while (timeout--);

	dev_err(nfc->dev, "wait for NAND controller reset timedout\n");
	return -ETIMEDOUT;
}
#endif

int sunxi_nfc_dev_ready(struct mtd_info *mtd)
{
	struct sunxi_nfc *nfc = to_sunxi_nfc(mtd);
	struct sunxi_nand_chip *sunxi_nand;
	struct sunxi_nand_pin *rb;
	unsigned long timeo;
	int ret;

	sunxi_nand = nfc->chips[nfc->current_chip];
	rb = &sunxi_nand->chip_sel.rb;
	timeo = (sunxi_nand->nand->state == FL_ERASING ? 400 : 20);

	switch (rb->type) {
	case CSRB_NATIVE:
		ret = !!(readl(nfc->regs + NFC_REG_ST) &
			 NFC_RB_STATE(rb->info.nativeid));
		if (ret)
			break;

		sunxi_nfc_wait_int(nfc, NFC_RB_B2R, timeo);
		ret = !!(readl(nfc->regs + NFC_REG_ST) &
			 NFC_RB_STATE(rb->info.nativeid));
		break;
	case CSRB_GPIO:
		ret = gpio_get_value(rb->info.gpio);
		break;
	case CSRB_NONE:
	default:
		ret = 0;
		dev_err(nfc->dev, "cannot check R/B NAND status!\n");
		break;
	}

	return ret;
}

void sunxi_nfc_select_chip(struct mtd_info *mtd, int chip)
{

	struct sunxi_nfc *nfc = to_sunxi_nfc(mtd);
	struct sunxi_nand_chip *sunxi_nand;
	struct sunxi_nand_chip_sel *sel;
	struct nand_chip *nand;
	u32 ctl;
	
	
	if ((chip < 0) || (chip >= nfc->num_of_chips))// && chip >= sunxi_nand->nsels)
		return;
    

	if (chip == nfc->current_chip)
		return;

	nfc->current_chip = chip;
	sunxi_nand = nfc->chips[chip];
	nand = sunxi_nand->nand;
	
	NAND_DBG("%s mtd 0x%X, sunxi_nand 0x%X, nfc 0x%X chip %d\n", 
		__func__, mtd, sunxi_nand, nfc,chip);

	ctl = readl(nfc->regs + NFC_REG_CTL) &
	      ~(NFC_PAGE_SHIFT_MSK | NFC_CE_SEL_MSK | NFC_RB_SEL_MSK | NFC_EN);

	if (chip >= 0) {
		
		sel = &sunxi_nand->chip_sel;
		//printf("chip >= 0 sel 0x%X\n", sel);
		ctl |= NFC_CE_SEL(sel->cs.info.nativeid) | NFC_EN |
		       NFC_PAGE_SHIFT(nand->page_shift - 10);
		if (sel->rb.type == CSRB_NONE) {
			nand->dev_ready = NULL;
		} else {
			nand->dev_ready = sunxi_nfc_dev_ready;
			if (sel->rb.type == CSRB_NATIVE)
				ctl |= NFC_RB_SEL(sel->rb.info.nativeid);
		}
		//mtd->writesize = 512;
		//printf("mtd->writesize %d, nfc->rec+ NFC_REG_SPARE_AREA %d\n", mtd->writesize, nfc->regs + NFC_REG_SPARE_AREA);
		writel(mtd->writesize, nfc->regs + NFC_REG_SPARE_AREA);

#if !(U_BOOT_NAND)
		if (nfc->clk_rate != sunxi_nand->clk_rate) {
			clk_set_rate(nfc->mod_clk, sunxi_nand->clk_rate);
			nfc->clk_rate = sunxi_nand->clk_rate;
		}
#endif
	}

	writel(sunxi_nand->timing_ctl, nfc->regs + NFC_REG_TIMING_CTL);
	writel(sunxi_nand->timing_cfg, nfc->regs + NFC_REG_TIMING_CFG);
	writel(ctl, nfc->regs + NFC_REG_CTL);
}


uint8_t sunxi_nfc_readbyte(struct mtd_info *mtd)
{
	uint8_t tmp;

	struct sunxi_nfc *nfc = to_sunxi_nfc(mtd);
	tmp = readl(nfc->regs);
	return tmp;
}

void sunxi_nfc_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	struct sunxi_nfc *nfc = to_sunxi_nfc(mtd);
	int ret;
	int cnt;
	int offs = 0;
	u32 tmp;

	while (len > offs) {
		cnt = min(len - offs, NFC_SRAM_SIZE);

		ret = sunxi_nfc_wait_cmd_fifo_empty(nfc);
		if (ret)
			break;

		writel(cnt, nfc->regs + NFC_REG_CNT);
		tmp = NFC_DATA_TRANS | NFC_DATA_SWAP_METHOD;
		writel(tmp, nfc->regs + NFC_REG_CMD);

		ret = sunxi_nfc_wait_int(nfc, NFC_CMD_INT_FLAG, 0);
		if (ret)
			break;

		if (buf)
			memcpy_fromio(buf + offs, nfc->regs + NFC_RAM0_BASE,
				      cnt);
		offs += cnt;
	}
}

void sunxi_nfc_write_buf(struct mtd_info *mtd, const uint8_t *buf,
				int len)
{
	struct sunxi_nfc *nfc = to_sunxi_nfc(mtd);
	int ret;
	int cnt;
	int offs = 0;
	u32 tmp;

	while (len > offs) {
		cnt = min(len - offs, NFC_SRAM_SIZE);

		ret = sunxi_nfc_wait_cmd_fifo_empty(nfc);
		if (ret)
			break;

		writel(cnt, nfc->regs + NFC_REG_CNT);
		memcpy_toio(nfc->regs + NFC_RAM0_BASE, buf + offs, cnt);
		tmp = NFC_DATA_TRANS | NFC_DATA_SWAP_METHOD |
		      NFC_ACCESS_DIR;
		writel(tmp, nfc->regs + NFC_REG_CMD);

		ret = sunxi_nfc_wait_int(nfc, NFC_CMD_INT_FLAG, 0);
		if (ret)
			break;

		offs += cnt;
	}
}

uint8_t sunxi_nfc_read_byte(struct mtd_info *mtd)
{
	uint8_t ret;

	sunxi_nfc_read_buf(mtd, &ret, 1);

	return ret;
}



void sunxi_nfc_cmd_ctrl(struct mtd_info *mtd, int dat,
			       unsigned int ctrl)
{
	struct sunxi_nfc *nfc = to_sunxi_nfc(mtd);
	int ret;
	u32 tmp;

	ret = sunxi_nfc_wait_cmd_fifo_empty(nfc);
	if (ret)
		return;

	if (ctrl & NAND_CTRL_CHANGE) {
		tmp = readl(nfc->regs + NFC_REG_CTL);
		if (ctrl & NAND_NCE)
			tmp |= NFC_CE_CTL;
		else
			tmp &= ~NFC_CE_CTL;
		writel(tmp, nfc->regs + NFC_REG_CTL);
	}

	if (dat == NAND_CMD_NONE)
		return;

	if (ctrl & NAND_CLE) {
		writel(NFC_SEND_CMD1 | dat, nfc->regs + NFC_REG_CMD);
	} else {
		writel(dat, nfc->regs + NFC_REG_ADDR_LOW);
		writel(NFC_SEND_ADR, nfc->regs + NFC_REG_CMD);
	}

	sunxi_nfc_wait_int(nfc, NFC_CMD_INT_FLAG, 0);
}

static void sunxi_nfc_hw_ecc_enable(struct mtd_info *mtd)
{
	struct sunxi_nfc *nfc = to_sunxi_nfc(mtd);
	struct sunxi_nand_chip *sunxi_nand;
	struct sunxi_nand_hw_ecc *data;
	u32 ecc_ctl;

	sunxi_nand = nfc->chips[nfc->current_chip];
	data = sunxi_nand->nand->ecc.priv;

	ecc_ctl = readl(nfc->regs + NFC_REG_ECC_CTL);
	ecc_ctl &= ~(NFC_ECC_MODE_MSK | NFC_ECC_PIPELINE |
		     NFC_ECC_BLOCK_SIZE_MSK);
	ecc_ctl |= NFC_ECC_EN | NFC_ECC_MODE(data->mode) | NFC_ECC_EXCEPTION;

	writel(ecc_ctl, nfc->regs + NFC_REG_ECC_CTL);
}

static void sunxi_nfc_hw_ecc_disable(struct mtd_info *mtd)
{
	struct sunxi_nfc *nfc = to_sunxi_nfc(mtd);

	writel(readl(nfc->regs + NFC_REG_ECC_CTL) & ~NFC_ECC_EN,
	       nfc->regs + NFC_REG_ECC_CTL);
}

static inline void sunxi_nfc_user_data_to_buf(u32 user_data, u8 *buf)
{
	buf[0] = user_data;
	buf[1] = user_data >> 8;
	buf[2] = user_data >> 16;
	buf[3] = user_data >> 24;
}


/**
 * nand_check_erased_buf - check if a buffer contains (almost) only 0xff data
 * @buf: buffer to test
 * @len: buffer length
 * @bitflips_threshold: maximum number of bitflips
 *
 * Check if a buffer contains only 0xff, which means the underlying region
 * has been erased and is ready to be programmed.
 * The bitflips_threshold specify the maximum number of bitflips before
 * considering the region is not erased.
 * Note: The logic of this function has been extracted from the memweight
 * implementation, except that nand_check_erased_buf function exit before
 * testing the whole buffer if the number of bitflips exceed the
 * bitflips_threshold value.
 *
 * Returns a positive number of bitflips or -ERROR_CODE.
 */
int nand_check_erased_buf(void *buf, int len, int bitflips_threshold)
{
	const unsigned char *bitmap = buf;
	int bitflips = 0;
	int weight;
	int longs;

	for (; len && ((unsigned long)bitmap) % sizeof(long); len--, bitmap++) {
		weight = hweight8(*bitmap);
		bitflips += BITS_PER_BYTE - weight;
		if (unlikely(bitflips > bitflips_threshold))
			return -EINVAL;
	}


	for (longs = len / sizeof(long); longs; longs--, bitmap += sizeof(long)) {
		BUG_ON(longs >= INT_MAX / BITS_PER_LONG);
	//	weight = hweight_long(*((unsigned long *)bitmap));
		weight = hweight32(*((unsigned long *)bitmap));
		bitflips += BITS_PER_LONG - weight;
		if (unlikely(bitflips > bitflips_threshold))
			return -EINVAL;
	}

	len %= sizeof(long);

	for (; len > 0; len--, bitmap++) {
		weight = hweight8(*bitmap);
		bitflips += BITS_PER_BYTE - weight;
		if (unlikely(bitflips > bitflips_threshold))
			return -EINVAL;
	}

	return bitflips;
}
EXPORT_SYMBOL(nand_check_erased_buf);

/**
 * nand_check_erased_ecc_chunk - check if an ECC chunk contains (almost) only
 *	0xff data
 * @data: data buffer to test
 * @datalen: data length
 * @ecc: ECC buffer
 * @ecclen: ECC length
 * @extraoob: extra OOB buffer
 * @extraooblen: extra OOB length
 * @bitflips_threshold: maximum number of bitflips
 *                    
 * Check if a data buffer and its associated ECC and OOB data contains only
 * 0xff pattern, which means the underlying region has been erased and is
 * ready to be programmed.
 * The bitflips_threshold specify the maximum number of bitflips before
 * considering the region as not erased.
 *
 * Returns a positive number of bitflips or -ERROR_CODE.
 */
int nand_check_erased_ecc_chunk(void *data, int datalen, void *ecc, int ecclen,
	void *extraoob, int extraooblen, int bitflips_threshold)
{
	int bitflips = 0;
	int ret;

	ret = nand_check_erased_buf(data, datalen, bitflips_threshold);
	if (ret < 0)
		return ret;

	bitflips += ret;
	bitflips_threshold -= ret;

	ret = nand_check_erased_buf(ecc, ecclen, bitflips_threshold);
	if (ret < 0)
		return ret;

	bitflips += ret;
	bitflips_threshold -= ret;

	ret = nand_check_erased_buf(extraoob, extraooblen, bitflips_threshold);
	if (ret < 0)
		return ret;

	memset(data, 0xff, datalen);
	memset(ecc, 0xff, ecclen);
	memset(extraoob, 0xff, extraooblen);

	return bitflips + ret;
}
EXPORT_SYMBOL(nand_check_erased_ecc_chunk);


static int sunxi_nfc_hw_ecc_read_chunk(struct mtd_info *mtd,
				       u8 *data, int data_off,
				       u8 *oob, int oob_off,
				       int *cur_off,
				       unsigned int *max_bitflips)
{
	struct sunxi_nfc *nfc = to_sunxi_nfc(mtd);
	struct sunxi_nand_chip *sunxi_nand;
	struct nand_chip *nand;
	struct nand_ecc_ctrl *ecc;
	u32 status;
	int ret;

	sunxi_nand = nfc->chips[nfc->current_chip];
	nand = sunxi_nand->nand;
	ecc = &nand->ecc;
	
	if (*cur_off != data_off)
		nand->cmdfunc(mtd, NAND_CMD_RNDOUT, data_off, -1);

	sunxi_nfc_read_buf(mtd, NULL, ecc->size);

	if (data_off + ecc->size != oob_off)
		nand->cmdfunc(mtd, NAND_CMD_RNDOUT, oob_off, -1);

	ret = sunxi_nfc_wait_cmd_fifo_empty(nfc);
	if (ret)
		return ret;

	writel(NFC_DATA_TRANS | NFC_DATA_SWAP_METHOD | NFC_ECC_OP,
	       nfc->regs + NFC_REG_CMD);

	ret = sunxi_nfc_wait_int(nfc, NFC_CMD_INT_FLAG, 0);
	if (ret)
		return ret;

	status = readl(nfc->regs + NFC_REG_ECC_ST);
	ret = NFC_ECC_ERR_CNT(0, readl(nfc->regs + NFC_REG_ECC_ERR_CNT(0)));

	memcpy_fromio(data, nfc->regs + NFC_RAM0_BASE, ecc->size);

	nand->cmdfunc(mtd, NAND_CMD_RNDOUT, oob_off, -1);
	sunxi_nfc_read_buf(mtd, oob, ecc->bytes + 4);

	if (status & NFC_ECC_ERR(0)) {
		ret = nand_check_erased_ecc_chunk(data,	ecc->size,
						  oob, ecc->bytes + 4,
						  NULL, 0, ecc->strength);
	} else {
		/*
		 * The engine protects 4 bytes of OOB data per chunk.
		 * Retrieve the corrected OOB bytes.
		 */
		sunxi_nfc_user_data_to_buf(readl(nfc->regs + NFC_REG_USER_DATA(0)),
					   oob);
	}

	if (ret < 0) {
		mtd->ecc_stats.failed++;
	} else {
		mtd->ecc_stats.corrected += ret;
		*max_bitflips = max_t(unsigned int, *max_bitflips, ret);
	}

	*cur_off = oob_off + ecc->bytes + 4;

	return 0;
}

static void sunxi_nfc_hw_ecc_read_extra_oob(struct mtd_info *mtd,
					    u8 *oob, int *cur_off)
{
	struct sunxi_nfc *nfc = to_sunxi_nfc(mtd);
	struct sunxi_nand_chip *sunxi_nand;
	struct nand_chip *nand;
	struct nand_ecc_ctrl *ecc;
	int offset;
	int len;


	sunxi_nand = nfc->chips[nfc->current_chip];
	nand = sunxi_nand->nand;
	ecc = &nand->ecc;
	offset = ((ecc->bytes + 4) * ecc->steps);
	len = mtd->oobsize - offset;
	
	if (len <= 0)
		return;

	if (*cur_off != offset)
		nand->cmdfunc(mtd, NAND_CMD_RNDOUT,
			      offset + mtd->writesize, -1);

	sunxi_nfc_read_buf(mtd, oob + offset, len);

	*cur_off = mtd->oobsize + mtd->writesize;
}

static inline u32 sunxi_nfc_buf_to_user_data(const u8 *buf)
{
	return buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24);
}

static int sunxi_nfc_hw_ecc_write_chunk(struct mtd_info *mtd,
					const u8 *data, int data_off,
					const u8 *oob, int oob_off,
					int *cur_off)
{
	struct sunxi_nfc *nfc = to_sunxi_nfc(mtd);
	struct sunxi_nand_chip *sunxi_nand;
	struct nand_chip *nand;
	struct nand_ecc_ctrl *ecc;
	int ret;

	sunxi_nand = nfc->chips[nfc->current_chip];
	nand = sunxi_nand->nand;
	ecc = &nand->ecc;

	if (data_off != *cur_off)
		nand->cmdfunc(mtd, NAND_CMD_RNDIN, data_off, -1);

	sunxi_nfc_write_buf(mtd, data, ecc->size);

	/* Fill OOB data in */
	writel(sunxi_nfc_buf_to_user_data(oob),
	       nfc->regs + NFC_REG_USER_DATA(0));

	if (data_off + ecc->size != oob_off)
		nand->cmdfunc(mtd, NAND_CMD_RNDIN, oob_off, -1);

	ret = sunxi_nfc_wait_cmd_fifo_empty(nfc);
	if (ret)
		return ret;

	writel(NFC_DATA_TRANS | NFC_DATA_SWAP_METHOD |
	       NFC_ACCESS_DIR | NFC_ECC_OP,
	       nfc->regs + NFC_REG_CMD);

	ret = sunxi_nfc_wait_int(nfc, NFC_CMD_INT_FLAG, 0);
	if (ret)
		return ret;

	*cur_off = oob_off + ecc->bytes + 4;

	return 0;
}

static void sunxi_nfc_hw_ecc_write_extra_oob(struct mtd_info *mtd,
					     u8 *oob, int *cur_off)
{
	struct sunxi_nfc *nfc = to_sunxi_nfc(mtd);
	struct sunxi_nand_chip *sunxi_nand;
	struct nand_chip *nand;
	struct nand_ecc_ctrl *ecc;
	int offset;
	int len;

	
	sunxi_nand = nfc->chips[nfc->current_chip];
	nand = sunxi_nand->nand;
	ecc = &nand->ecc;
	offset = ((ecc->bytes + 4) * ecc->steps);
	len = mtd->oobsize - offset;
	
	if (len <= 0)
		return;

	if (*cur_off != offset)
		nand->cmdfunc(mtd, NAND_CMD_RNDIN,
			      offset + mtd->writesize, -1);

	sunxi_nfc_write_buf(mtd, oob + offset, len);

	*cur_off = mtd->oobsize + mtd->writesize;
}

static int sunxi_nfc_hw_ecc_read_page(struct mtd_info *mtd,
				      struct nand_chip *chip, uint8_t *buf,
#if 1
				      int oob_required,
#endif
				      int page)
{
	struct nand_ecc_ctrl *ecc = &chip->ecc;
	unsigned int max_bitflips = 0;
	int ret, i, cur_off = 0;

	sunxi_nfc_hw_ecc_enable(mtd);

	for (i = 0; i < ecc->steps; i++) {
		int data_off = i * ecc->size;
		int oob_off = i * (ecc->bytes + 4);
		u8 *data = buf + data_off;
		u8 *oob = chip->oob_poi + oob_off;

		ret = sunxi_nfc_hw_ecc_read_chunk(mtd, data, data_off, oob,
						  oob_off + mtd->writesize,
						  &cur_off, &max_bitflips);
		if (ret)
			return ret;
	}

#if 1
	if (oob_required)
		sunxi_nfc_hw_ecc_read_extra_oob(mtd, chip->oob_poi, &cur_off);
#endif

	sunxi_nfc_hw_ecc_disable(mtd);

	return max_bitflips;
}
#if 1
static int
#else
static void
#endif
sunxi_nfc_hw_ecc_write_page(struct mtd_info *mtd,
				       struct nand_chip *chip,
				       const uint8_t *buf
#if 1
						,
				       int oob_required
#if 0
				       int page
#endif
#endif
				       )
{
	struct nand_ecc_ctrl *ecc = &chip->ecc;
	int ret, i, cur_off = 0;

	sunxi_nfc_hw_ecc_enable(mtd);

	for (i = 0; i < ecc->steps; i++) {
		int data_off = i * ecc->size;
		int oob_off = i * (ecc->bytes + 4);
		const u8 *data = buf + data_off;
		const u8 *oob = chip->oob_poi + oob_off;

		ret = sunxi_nfc_hw_ecc_write_chunk(mtd, data, data_off, oob,
						   oob_off + mtd->writesize,
						   &cur_off);
#if 1
		if (ret)
			return ret;
#endif
	}

#if 1
	if (oob_required)
		sunxi_nfc_hw_ecc_write_extra_oob(mtd, chip->oob_poi, &cur_off);
#endif

	sunxi_nfc_hw_ecc_disable(mtd);

#if 1
	return 0;
#endif
}

static int sunxi_nfc_hw_syndrome_ecc_read_page(struct mtd_info *mtd,
					       struct nand_chip *chip,
					       uint8_t *buf,
#if 1
					       int oob_required,
					       int page
#endif					       
					       )
{
	struct nand_ecc_ctrl *ecc = &chip->ecc;
	unsigned int max_bitflips = 0;
	int ret, i, cur_off = 0;

	sunxi_nfc_hw_ecc_enable(mtd);

	for (i = 0; i < ecc->steps; i++) {
		int data_off = i * (ecc->size + ecc->bytes + 4);
		int oob_off = data_off + ecc->size;
		u8 *data = buf + (i * ecc->size);
		u8 *oob = chip->oob_poi + (i * (ecc->bytes + 4));

		ret = sunxi_nfc_hw_ecc_read_chunk(mtd, data, data_off, oob,
						  oob_off, &cur_off,
						  &max_bitflips);
		if (ret)
			return ret;
	}

#if 1
	if (oob_required)
		sunxi_nfc_hw_ecc_read_extra_oob(mtd, chip->oob_poi, &cur_off);
#endif

	sunxi_nfc_hw_ecc_disable(mtd);

	return max_bitflips;
}
#if 1
static int
#else
static void
#endif
sunxi_nfc_hw_syndrome_ecc_write_page(struct mtd_info *mtd,
						struct nand_chip *chip,
						const uint8_t *buf
#if 1	
						,
						int oob_required
#endif
#if 0
						int page
#endif
						)
{
	struct nand_ecc_ctrl *ecc = &chip->ecc;
	int ret, i, cur_off = 0;

	sunxi_nfc_hw_ecc_enable(mtd);

	for (i = 0; i < ecc->steps; i++) {
		int data_off = i * (ecc->size + ecc->bytes + 4);
		int oob_off = data_off + ecc->size;
		const u8 *data = buf + (i * ecc->size);
		const u8 *oob = chip->oob_poi + (i * (ecc->bytes + 4));

		ret = sunxi_nfc_hw_ecc_write_chunk(mtd, data, data_off,
						   oob, oob_off, &cur_off);
#if 1		
		if (ret)
			return ret;
#endif
	}

#if 1
	if (oob_required)
		sunxi_nfc_hw_ecc_write_extra_oob(mtd, chip->oob_poi, &cur_off);
#endif

	sunxi_nfc_hw_ecc_disable(mtd);

#if 1
	return 0;
#endif
}

static const s32 tWB_lut[] = {6, 12, 16, 20};
static const s32 tRHW_lut[] = {4, 8, 12, 20};

static int _sunxi_nand_lookup_timing(const s32 *lut, int lut_size, u32 duration,
		u32 clk_period)
{
	u32 clk_cycles = DIV_ROUND_UP(duration, clk_period);
	int i;

	for (i = 0; i < lut_size; i++) {
		if (clk_cycles <= lut[i])
			return i;
	}

	/* Doesn't fit */
	return -EINVAL;
}

#define sunxi_nand_lookup_timing(l, p, c) \
			_sunxi_nand_lookup_timing(l, ARRAY_SIZE(l), p, c)

static int sunxi_nand_chip_set_timings(struct sunxi_nand_chip *chip,
				       const struct nand_sdr_timings *timings)
{
	struct sunxi_nfc *nfc = chip->nand->priv;
	u32 min_clk_period = 0;
	s32 tWB, tADL, tWHR, tRHW, tCAD;

	/* T1 <=> tCLS */
	if (timings->tCLS_min > min_clk_period)
		min_clk_period = timings->tCLS_min;

	/* T2 <=> tCLH */
	if (timings->tCLH_min > min_clk_period)
		min_clk_period = timings->tCLH_min;

	/* T3 <=> tCS */
	if (timings->tCS_min > min_clk_period)
		min_clk_period = timings->tCS_min;

	/* T4 <=> tCH */
	if (timings->tCH_min > min_clk_period)
		min_clk_period = timings->tCH_min;

	/* T5 <=> tWP */
	if (timings->tWP_min > min_clk_period)
		min_clk_period = timings->tWP_min;

	/* T6 <=> tWH */
	if (timings->tWH_min > min_clk_period)
		min_clk_period = timings->tWH_min;

	/* T7 <=> tALS */
	if (timings->tALS_min > min_clk_period)
		min_clk_period = timings->tALS_min;

	/* T8 <=> tDS */
	if (timings->tDS_min > min_clk_period)
		min_clk_period = timings->tDS_min;

	/* T9 <=> tDH */
	if (timings->tDH_min > min_clk_period)
		min_clk_period = timings->tDH_min;

	/* T10 <=> tRR */
	if (timings->tRR_min > (min_clk_period * 3))
		min_clk_period = DIV_ROUND_UP(timings->tRR_min, 3);

	/* T11 <=> tALH */
	if (timings->tALH_min > min_clk_period)
		min_clk_period = timings->tALH_min;

	/* T12 <=> tRP */
	if (timings->tRP_min > min_clk_period)
		min_clk_period = timings->tRP_min;

	/* T13 <=> tREH */
	if (timings->tREH_min > min_clk_period)
		min_clk_period = timings->tREH_min;

	/* T14 <=> tRC */
	if (timings->tRC_min > (min_clk_period * 2))
		min_clk_period = DIV_ROUND_UP(timings->tRC_min, 2);

	/* T15 <=> tWC */
	if (timings->tWC_min > (min_clk_period * 2))
		min_clk_period = DIV_ROUND_UP(timings->tWC_min, 2);

	/* T16 - T19 + tCAD */
	tWB  = sunxi_nand_lookup_timing(tWB_lut, timings->tWB_max,
					min_clk_period);
	if (tWB < 0) {
		dev_err(nfc->dev, "unsupported tWB\n");
		return tWB;
	}

	tADL = DIV_ROUND_UP(timings->tADL_min, min_clk_period) >> 3;
	if (tADL > 3) {
		dev_err(nfc->dev, "unsupported tADL\n");
		return -EINVAL;
	}

	tWHR = DIV_ROUND_UP(timings->tWHR_min, min_clk_period) >> 3;
	if (tWHR > 3) {
		dev_err(nfc->dev, "unsupported tWHR\n");
		return -EINVAL;
	}

	tRHW = sunxi_nand_lookup_timing(tRHW_lut, timings->tRHW_min,
					min_clk_period);
	if (tRHW < 0) {
		dev_err(nfc->dev, "unsupported tRHW\n");
		return tRHW;
	}

	/*
	 * TODO: according to ONFI specs this value only applies for DDR NAND,
	 * but Allwinner seems to set this to 0x7. Mimic them for now.
	 */
	tCAD = 0x7;

	/* TODO: A83 has some more bits for CDQSS, CS, CLHZ, CCS, WC */
	chip->timing_cfg = NFC_TIMING_CFG(tWB, tADL, tWHR, tRHW, tCAD);

	/*
	 * ONFI specification 3.1, paragraph 4.15.2 dictates that EDO data
	 * output cycle timings shall be used if the host drives tRC less than
	 * 30 ns.
	 */
	chip->timing_ctl = (timings->tRC_min < 30000) ? NFC_TIMING_CTL_EDO : 0;

	/* Convert min_clk_period from picoseconds to nanoseconds */
	min_clk_period = DIV_ROUND_UP(min_clk_period, 1000);

	/*
	 * Convert min_clk_period into a clk frequency, then get the
	 * appropriate rate for the NAND controller IP given this formula
	 * (specified in the datasheet):
	 * nand clk_rate = 2 * min_clk_rate
	 */
	chip->clk_rate = (2 * NSEC_PER_SEC) / min_clk_period;

	return 0;
}


/* return the supported asynchronous timing mode. */
static inline int onfi_get_async_timing_mode(struct nand_chip *chip)
{
#if STRUCT_NANDCHIP_HAVE_ONFI_TIMING
	if (!chip->onfi_version)
		return ONFI_TIMING_MODE_UNKNOWN;
	return le16_to_cpu(chip->onfi_params.async_timing_mode);
#else
	return ONFI_TIMING_MODE_UNKNOWN;
#endif
}

#if !STRUCT_NANDCHIP_HAVE_ONFI_TIMING
/*
const struct nand_sdr_timings *onfi_async_timing_mode_to_sdr_timings(int mode)
{
	if (mode < 0 || mode >= ARRAY_SIZE(onfi_sdr_timings))
		return ERR_PTR(-EINVAL);

	return &onfi_sdr_timings[mode];	
}
*/
#endif

int sunxi_nand_chip_init_timings(struct sunxi_nand_chip *chip)
{
	struct mtd_info *mtd = &nand_info[0];
	const struct nand_sdr_timings *timings;
	int ret=0;
	int mode;

	mode = onfi_get_async_timing_mode(chip->nand);
	if (mode == ONFI_TIMING_MODE_UNKNOWN) {
#if STRUCT_NANDCHIP_HAVE_ONFI_TIMING
		mode = chip->nand.onfi_timing_mode_default;
#else
		mode = chip->onfi_timing_mode_default;
#endif
	} else {
		uint8_t feature[ONFI_SUBFEATURE_PARAM_LEN] = {};
		int i;

		mode = fls(mode) - 1;
		if (mode < 0)
			mode = 0;

		feature[0] = mode;
		
		chip->nand->select_chip(mtd, i);
#if STRUCT_NANDCHIP_HAVE_ONFI_TIMING			
		ret = chip->nand.onfi_set_features(mtd,	&chip->nand,
					ONFI_FEATURE_ADDR_TIMING_MODE,
					feature);
#else
			
#endif
		chip->nand->select_chip(mtd, -1);
			
		if (ret)
			return ret;
	}

	timings = onfi_async_timing_mode_to_sdr_timings(mode);

	if (IS_ERR(timings))
		return PTR_ERR(timings);

	return sunxi_nand_chip_set_timings(chip, timings);
}

static int sunxi_nand_hw_common_ecc_ctrl_init(struct mtd_info *mtd,
					      struct nand_ecc_ctrl *ecc)
{
	static const u8 strengths[] = { 16, 24, 28, 32, 40, 48, 56, 60, 64 };
	
	struct sunxi_nand_hw_ecc *data;
	struct nand_ecclayout *layout;
	int nsectors;
	int ret;
	int i;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	/* Add ECC info retrieval from DT */
	for (i = 0; i < ARRAY_SIZE(strengths); i++) {
		if (ecc->strength <= strengths[i])
			break;
	}

	if (i >= ARRAY_SIZE(strengths)) {
		dev_err(mtd->dev, "unsupported strength\n");
		ret = -ENOTSUPP;
		goto err;
	}

	data->mode = i;

	/* HW ECC always request ECC bytes for 1024 bytes blocks */
	ecc->bytes = DIV_ROUND_UP(ecc->strength * fls(8 * 1024), 8);

	/* HW ECC always work with even numbers of ECC bytes */
	ecc->bytes = ALIGN(ecc->bytes, 2);

	layout = &data->layout;
	nsectors = mtd->writesize / ecc->size;

	if (mtd->oobsize < ((ecc->bytes + 4) * nsectors)) {
		ret = -EINVAL;
		goto err;
	}

	layout->eccbytes = (ecc->bytes * nsectors);

	ecc->layout = layout;
	ecc->priv = data;

	return 0;

err:
	kfree(data);

	return ret;
}

/*static void sunxi_nand_hw_common_ecc_ctrl_cleanup(struct nand_ecc_ctrl *ecc)
{
	kfree(ecc->priv);
}
*/
int sunxi_nand_hw_ecc_ctrl_init(struct mtd_info *mtd,
				       struct nand_ecc_ctrl *ecc)
{
	struct nand_ecclayout *layout;
	int nsectors;
	int i, j;
	int ret;

	ret = sunxi_nand_hw_common_ecc_ctrl_init(mtd, ecc);
	if (ret)
		return ret;

	ecc->read_page = sunxi_nfc_hw_ecc_read_page;
	ecc->write_page = sunxi_nfc_hw_ecc_write_page;
	layout = ecc->layout;
	nsectors = mtd->writesize / ecc->size;

	for (i = 0; i < nsectors; i++) {
		if (i) {
			layout->oobfree[i].offset =
				layout->oobfree[i - 1].offset +
				layout->oobfree[i - 1].length +
				ecc->bytes;
			layout->oobfree[i].length = 4;
		} else {
			/*
			 * The first 2 bytes are used for BB markers, hence we
			 * only have 2 bytes available in the first user data
			 * section.
			 */
			layout->oobfree[i].length = 2;
			layout->oobfree[i].offset = 2;
		}

		for (j = 0; j < ecc->bytes; j++)
			layout->eccpos[(ecc->bytes * i) + j] =
					layout->oobfree[i].offset +
					layout->oobfree[i].length + j;
	}

	if (mtd->oobsize > (ecc->bytes + 4) * nsectors) {
		layout->oobfree[nsectors].offset =
				layout->oobfree[nsectors - 1].offset +
				layout->oobfree[nsectors - 1].length +
				ecc->bytes;
		layout->oobfree[nsectors].length = mtd->oobsize -
				((ecc->bytes + 4) * nsectors);
	}

	return 0;
}

static int sunxi_nand_hw_syndrome_ecc_ctrl_init(struct mtd_info *mtd,
						struct nand_ecc_ctrl *ecc)
{
	struct nand_ecclayout *layout;
	int nsectors;
	int i;
	int ret;

	ret = sunxi_nand_hw_common_ecc_ctrl_init(mtd, ecc);
	if (ret)
		return ret;

	ecc->prepad = 4;
	ecc->read_page = sunxi_nfc_hw_syndrome_ecc_read_page;
	ecc->write_page = sunxi_nfc_hw_syndrome_ecc_write_page;

	layout = ecc->layout;
	nsectors = mtd->writesize / ecc->size;

	for (i = 0; i < (ecc->bytes * nsectors); i++)
		layout->eccpos[i] = i;

	layout->oobfree[0].length = mtd->oobsize - i;
	layout->oobfree[0].offset = i;

	return 0;
}
/*
static void sunxi_nand_ecc_cleanup(struct nand_ecc_ctrl *ecc)
{
	switch (ecc->mode) {
	case NAND_ECC_HW:
	case NAND_ECC_HW_SYNDROME:
		sunxi_nand_hw_common_ecc_ctrl_cleanup(ecc);
		break;
	case NAND_ECC_NONE:
		kfree(ecc->layout);
	default:
		break;
	}
}
*/
int sunxi_nand_ecc_init(struct mtd_info *mtd, struct nand_ecc_ctrl *ecc)
{
	struct sunxi_nfc *nfc = to_sunxi_nfc(mtd);
	struct sunxi_nand_chip *sunxi_nand;
	struct nand_chip *nand;
	int ret;

	sunxi_nand = nfc->chips[0]; //?? nfc->current_chip
	nand = sunxi_nand->nand;

	if (!ecc->size) {
#if STRUCT_NANDCHIP_HAVE_ONFI_TIMING
		ecc->size = nand->ecc_step_ds;
		ecc->strength = nand->ecc_strength_ds;
#else
		ecc->size = sunxi_nand->ecc_step_ds;
		ecc->strength = sunxi_nand->ecc_strength_ds;
//		ecc->size = nand->ecc_step_ds;
//		ecc->strength = nand->ecc_strength_ds;
#endif
	}
	NAND_DBG("ecc->size %d ecc->strength %d ecc->mode %d\n", ecc->size,ecc->strength, ecc->mode);
	if (!ecc->size || !ecc->strength)
		return -EINVAL;

	switch (ecc->mode) {
	case NAND_ECC_SOFT_BCH:
		break;
	case NAND_ECC_HW:
		ret = sunxi_nand_hw_ecc_ctrl_init(mtd, ecc);
		if (ret)
			return ret;
		break;
	case NAND_ECC_HW_SYNDROME:
		ret = sunxi_nand_hw_syndrome_ecc_ctrl_init(mtd, ecc);
		if (ret)
			return ret;
		break;
	case NAND_ECC_NONE:
		ecc->layout = kzalloc(sizeof(*ecc->layout), GFP_KERNEL);
		if (!ecc->layout)
			return -ENOMEM;
		ecc->layout->oobfree[0].length = mtd->oobsize;
	case NAND_ECC_SOFT:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}



extern u32 gpio_g_pioMemBase; // = (u32)CSP_OSAL_PHY_2_VIRT(CSP_PIN_PHY_ADDR_BASE , CSP_PIN_PHY_ADDR_SIZE);

#define PIOC_REGS_BASE gpio_g_pioMemBase

#define __REG(x)                        (*(volatile unsigned int *)(x))

#define PIO_REG_CFG(n, i)               ((volatile unsigned int *)(PIOC_REGS_BASE + ((n)-1)*0x24 + ((i)<<2) + 0x00))
#define PIO_REG_DLEVEL(n, i)            ((volatile unsigned int *)(PIOC_REGS_BASE + ((n)-1)*0x24 + ((i)<<2) + 0x14))
#define PIO_REG_PULL(n, i)              ((volatile unsigned int *)(PIOC_REGS_BASE + ((n)-1)*0x24 + ((i)<<2) + 0x1C))
#define PIO_REG_DATA(n)                   ((volatile unsigned int *)(PIOC_REGS_BASE + ((n)-1)*0x24 + 0x10))

#define PIO_REG_CFG_VALUE(n, i)          __REG(PIOC_REGS_BASE + ((n)-1)*0x24 + ((i)<<2) + 0x00)
#define PIO_REG_DLEVEL_VALUE(n, i)       __REG(PIOC_REGS_BASE + ((n)-1)*0x24 + ((i)<<2) + 0x14)
#define PIO_REG_PULL_VALUE(n, i)         __REG(PIOC_REGS_BASE + ((n)-1)*0x24 + ((i)<<2) + 0x1C)
#define PIO_REG_DATA_VALUE(n)            __REG(PIOC_REGS_BASE + ((n)-1)*0x24 + 0x10)


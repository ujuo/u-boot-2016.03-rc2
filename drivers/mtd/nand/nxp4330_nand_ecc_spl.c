/**
 * Copyright(C) 2014 STcube Inc.
 * All right reserved by Seungwoo Kim <ksw@stcube.com>
 * This code is based on Nexell's original NAND ECC code,
 * and add work around their HW ECC's quirk behavior.
 * See below their original copy right.
 *
 * (C) Copyright 2009
 * KOO Bon-Gyu, Nexell Co, <freestyle@nexell.co.kr>
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
 **/
#define	U_BOOT_NAND		(1)

#if (U_BOOT_NAND)
#include <common.h>
#include <asm/errno.h>
#include <asm/io.h>
#include <malloc.h>
#include <linux/mtd/nand.h>
#include <nand.h>
#include <asm/arch/platform.h>
#else
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/mtd/nand.h>
#include <asm/io.h>
#include <mach/platform.h>
#include <mach/soc.h>
#endif

#if	(0)
#define DBGOUT(msg...)		printf(msg)
#else
#define DBGOUT(msg...)		do {} while (0)
#endif
#if	(1)
#define ECCERR(msg...)		printf(msg)
#else
#define ECCERR(msg...)		do {} while (0)
#endif

#define ERROUT(msg...)		{ 					\
		printf(msg); }


/* ksw : for ECCCODE debuging */
//#define DEBUG_ECCCODE	1
/* ksw : nand hardware's bug work around. */
#define NAND_HW_ECC_WORKAROUND	0
/**
	WORKAROUND for HW 4bit nand ecc decoder.
	
	When I found Writing ECCcode is same as read oobdata,
	writing this code to ORGECC and read_buf always fail to ECC machine.
	Then I check NFCNT value that I found reading count always 
	1 byte less.
	So I write NFCTRL's DATACNT to 512 not 511, then decoding is not done 
	until one byte more read. This counter incremented write operation also,
	so I decided to workaround like following code.
	
	1. reset NFCTRL, write ORGdata.
	2. NFCTRL's DATACNT to 512 and start decoding.
	3. 512byte read from NAND.
	4. chip_en = -1, 1byte read from NAND. chip_en = 0;
	5. Then Decoding/ECC calculating would be completed soon. 

**/
#if defined (CONFIG_MTD_NAND_ECC_HW) || defined (CONFIG_SYS_NAND_HW_ECC)

#define	NAND_READ_RETRY		(1)

#include "nxp4330_nand_ecc.h"
#ifdef CONFIG_NAND_RANDOMIZER
#include "nx_randomizer.h"
static uint8_t *randomize_buf;
static uint32_t pages_per_block_mask;
#endif
#ifdef CONFIG_MTD_NAND_VERIFY_WRITE
static uint8_t *verify_buf;
#endif

#define NXP_NAND_TYPE_UNKNOWN	0x0
#define NXP_NAND_TYPE_SLC	0x1
#define NXP_NAND_TYPE_MLC	0x2

int nand_type = NXP_NAND_TYPE_UNKNOWN;                
int addr_cycle = 4;
/* Define default oob placement schemes for large and small page devices */
#if 0
static struct nand_ecclayout nand_oob_8 = {
	.eccbytes = 3,
	.eccpos = {0, 1, 2},
	.oobfree = {
		{.offset = 3,
		 .length = 2},
		{.offset = 6,
		 .length = 2} }
};

static struct nand_ecclayout nand_oob_16 = {
	.eccbytes = 6,
	.eccpos = {0, 1, 2, 3, 6, 7},
	.oobfree = {
		{.offset = 8,
		 . length = 8} }
};

static struct nand_ecclayout nand_oob_64 = {
	.eccbytes = 24,
	.eccpos = {
		   40, 41, 42, 43, 44, 45, 46, 47,
		   48, 49, 50, 51, 52, 53, 54, 55,
		   56, 57, 58, 59, 60, 61, 62, 63},
	.oobfree = {
		{.offset = 2,
		 .length = 38} }
};
#endif
static struct nand_ecclayout nand_oob_128 = {
	.eccbytes = 28,
	.eccpos = {
			100, 101, 102, 103,
		   104, 105, 106, 107, 108, 109, 110, 111,
		   112, 113, 114, 115, 116, 117, 118, 119,
		   120, 121, 122, 123, 124, 125, 126, 127},
	.oobfree = {
		{.offset = 2,
		 .length = 98} }
};

/*
 * u-boot nand hw ecc
 */
static int iNX_BCH_VAR_K	 = ECC_PAGE_SIZE;			/* 512 or 1024 */
static int iNX_BCH_VAR_M	 = 13;			/* 13 or 14 */
static int iNX_BCH_VAR_T	 = ECC_HW_BITS;	/* 4, 8, 12, 16, 24, 40, 60 ... */
static int iNX_BCH_VAR_TMAX  = 24;			/* eccsize == 512 ? 24 : 60 */
static int INX_BCH_INDEX    = 0;			/* 4 => 0, 8 => 1, 12 => 2, 16 => 3, 24 => 4, 40 => 5, 80 => 6 */
static char var_r_table[8]  = { 6, 12, 19, 25, 38, 41, 69, 104 };
/* 
static uint8_t invalid_ecc_4bit[] = {0x97, 0x38, 0x79, 0xab, 0x9d, 0x49, 0xd0};
static uint8_t invalid_ecc_8bit[] = {0x97, 0x38, 0x79, 0xab, 0x9d, 0x49, 0xd0, 0x00, 0x00, 0x00, 0x00, 0x00};
*/

static struct NX_MCUS_RegisterSet * const _pNCTRL =
	(struct NX_MCUS_RegisterSet *)IO_ADDRESS(PHY_BASEADDR_MCUSTOP_MODULE);

static void ecc_reset_decoder(void)
{
	DBGOUT("ecc_reset_decoder enter\n");
	_pNCTRL->NFCONTROL |= NX_NFCTRL_ECCRST;
	// disconnect syndrome path
	_pNCTRL->NFECCAUTOMODE = (_pNCTRL->NFECCAUTOMODE & ~(NX_NFACTRL_ELP)) | NX_NFACTRL_SYN;
}

static void ecc_decode_enable(int eccsize)	/* 512 or 1024 */
{
	int iNX_BCH_VAR_R = var_r_table[INX_BCH_INDEX]; //(((iNX_BCH_VAR_M * iNX_BCH_VAR_T)/8) - 1);
	DBGOUT("ecc_decode_enable enter\n");
	// connect syndrome path
	_pNCTRL->NFECCAUTOMODE = (_pNCTRL->NFECCAUTOMODE & ~(NX_NFACTRL_ELP | NX_NFACTRL_SYN));

	// run ecc
	_pNCTRL->NFECCCTRL =
		(1 << NX_NFECCCTRL_RUNECC_W)   |	   // run ecc
		(0 << NX_NFECCCTRL_ELPLOAD)   |
		(NX_NF_DECODE << NX_NFECCCTRL_DECMODE_W)	|
		(0 << NX_NFECCCTRL_ZEROPAD)	|
		((iNX_BCH_VAR_T & 0x7F) << NX_NFECCCTRL_ELPNUM)		|
		((iNX_BCH_VAR_R & 0xFF) << NX_NFECCCTRL_PDATACNT)	|
#if NAND_HW_ECC_WORKAROUND
		(((eccsize) & 0x3FF)  << NX_NFECCCTRL_DATACNT); /* ksw:work around for read correct. (eccsize-1 => eccsize)*/
#else
		(((eccsize-1) & 0x3FF)  << NX_NFECCCTRL_DATACNT);
#endif
}

static void ecc_write_ecc_decode(unsigned int *ecc, int eccbyte)
{
	volatile U32 *pNFORGECC = _pNCTRL->NFORGECC;
	volatile int i, len;
	DBGOUT("ecc_write_ecc_decode enter\n");
	/* align 4byte */
	len = DIV_ROUND_UP(eccbyte, sizeof(U32));

	for(i = 0; len > i; i++) {
		*pNFORGECC++ = *ecc++; 
	}
}

static void ecc_wait_for_decode(void)
{
	while (0 ==(_pNCTRL->NFECCSTATUS & NX_NFECCSTATUS_DECDONE));
	{ ; }
}

static unsigned int ecc_decode_error(void)
{
	return (int)(_pNCTRL->NFECCSTATUS & NX_NFECCSTATUS_ERROR);
}

static void ecc_start_correct(int eccsize)
{
	int iNX_BCH_VAR_R = var_r_table[INX_BCH_INDEX]; //(((iNX_BCH_VAR_M * iNX_BCH_VAR_T)/8) - 1);

	// load elp
	_pNCTRL->NFECCCTRL =
		(0 << NX_NFECCCTRL_RUNECC_W)   |
		(1 << NX_NFECCCTRL_ELPLOAD)    |	   // load elp
		(NX_NF_DECODE << NX_NFECCCTRL_DECMODE_W)	|
		(0 << NX_NFECCCTRL_ZEROPAD)	|
 		((iNX_BCH_VAR_T & 0x07F) << NX_NFECCCTRL_ELPNUM )	|
		((iNX_BCH_VAR_R & 0x0FF) << NX_NFECCCTRL_PDATACNT)	|
#if NAND_HW_ECC_WORKAROUND
	 	(((eccsize) & 0x3FF) << NX_NFECCCTRL_DATACNT); /* ksw : It may or may not */
#else
		(((eccsize-1) & 0x3FF) << NX_NFECCCTRL_DATACNT);
#endif
}

static void ecc_wait_for_correct(void)
{
	while (_pNCTRL->NFECCSTATUS & NX_NFECCSTATUS_BUSY)
	{ ; }
}

static int ecc_get_err_location(unsigned int *pLocation)
{
	volatile U32 *pELoc = _pNCTRL->NFERRLOCATION;
	volatile int len = ECC_HW_BITS/2;
	volatile int err, i;

	// it's not error correctable
	if (((_pNCTRL->NFECCSTATUS & NX_NFECCSTATUS_NUMERR) >>  4) !=
		((_pNCTRL->NFECCSTATUS & NX_NFECCSTATUS_ELPERR) >> 16))
		return -1;

	for (i = 0; len > i; i++) {
		register U32 regvalue = *pELoc++;
		*pLocation++ = (regvalue>>0  & 0x3FFF)^0x7;
		*pLocation++ = (regvalue>>14 & 0x3FFF)^0x7;
	}

	err = (_pNCTRL->NFECCSTATUS & NX_NFECCSTATUS_NUMERR) >> 4;
	return err;
}

static void ecc_setup_encoder(void)
{
	int iNX_BCH_VAR_R = var_r_table[INX_BCH_INDEX]; //(((iNX_BCH_VAR_M * iNX_BCH_VAR_T)/8) - 1);

    NX_MCUS_SetNANDRWDataNum(iNX_BCH_VAR_K);
    NX_MCUS_SetParityCount(iNX_BCH_VAR_R);
    NX_MCUS_SetNumOfELP(iNX_BCH_VAR_T);
}

static void ecc_encode_enable(void)
{
	NX_MCUS_SetNFDecMode(NX_MCUS_DECMODE_ENCODER);
	NX_MCUS_RunECCEncDec();
}

static void ecc_read_ecc_encode(unsigned int *ecc, int eccbyte)
{
	volatile U32 *pNFECC = _pNCTRL->NFECC;
	volatile int i, len;

	/* align 4byte */
	len = DIV_ROUND_UP(eccbyte, sizeof(U32));

	for(i = 0; len > i; i++) {
		*ecc++ = *pNFECC++;
	}
}

static void ecc_wait_for_encode(void)
{
	while ( 0==(_pNCTRL->NFECCSTATUS & NX_NFECCSTATUS_ENCDONE) )
	{ ; }
}

/*
 * u-boot nand hw ecc interface
 */

/* ECC related defines */
#define	ECC_HW_MAX_BYTES		((106/32)*32 + 32) 	/* 128 */

static int nand_sw_ecc_verify_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	int i;
	struct nand_chip *chip = mtd->priv;

	for (i = 0; len > i; i++)
		if (buf[i] != readb(chip->IO_ADDR_R))
			return -EFAULT;
	return 0;
}

static unsigned char eccbuf[256];
static uint32_t  eccbuff[ECC_HW_MAX_BYTES/4];
static int errpos[4];

static int nxp4330_nand_read_oob(struct mtd_info *mtd, struct nand_chip *chip,
			     int page)
{
	//int i;
	//printf("nand_read_oob oobsize=%d page_shift=%d page=%d chipsize=%d\n", mtd->oobsize, chip->page_shift, page, chip->chipsize);
	//printf("chip=%X mtd=%X\n", chip, mtd);
	chip->oob_poi = &eccbuf[0];
	if (chip->page_shift > 9) {
		chip->cmdfunc (mtd, NAND_CMD_READ0, 1 << chip->page_shift, page);
	} else {
		chip->cmdfunc (mtd, NAND_CMD_READOOB, 0, page);
	}

	chip->read_buf(mtd, chip->oob_poi, mtd->oobsize);
/*	for (i=0; i<128;i++) {
		if (i%16 == 0)
			printf("\n%05d:", i);
		printf("%02X ", chip->oob_poi[i]);
	}
	printf("\n"); */
	return 0;
}

static int nand_hw_ecc_read_page(struct mtd_info *mtd, struct nand_chip *chip,
				uint8_t *buf, int oob_required, int page)
{
	int i, k, n, ret = 0, retry = 0;

	int eccsteps = chip->ecc.steps;
	int eccbytes = chip->ecc.bytes;
	int eccsize  = chip->ecc.size;
	int eccrange = 8 * eccsize;
//	printf("nand_hw_ecc_read_page enter\n");
	uint8_t  *ecccode = (uint8_t*)eccbuff;
	uint32_t *eccpos = chip->ecc.layout->eccpos;
	
	chip->oob_poi = &eccbuf[0];
	
	uint8_t  *p = buf;//, dummy;

	uint32_t *errdat;
	int err = 0, errcnt = 0;
	uint32_t corrected = 0, failed = 0;
	uint32_t max_bitflips = 0;
	int is_erasedpage = 0;

	DBGOUT("page %d mtd->writesize %d\n", page,mtd->writesize);
	DBGOUT("mtd->oobsize %d eccmode=%d, eccbytes=%d, eccstep=%d chip->ecc.steps %d\n",mtd->oobsize,ECC_HW_BITS, eccbytes, eccsteps,chip->ecc.steps);
	do {
		/* reset value */
		eccsteps = chip->ecc.steps;
		p = buf;
#ifndef NO_ISSUE_MTD_BITFLIP_PATCH	/* freestyle@2013.09.26 */
		corrected = failed = 0;
#endif

		if (512 <= mtd->writesize) {
			chip->ecc.read_oob(mtd, chip, page);
			//nxp4330_nand_read_oob(mtd, chip, page);
			chip->cmdfunc(mtd, NAND_CMD_READ0, 0x00, page);
		} else {
			chip->cmdfunc(mtd, NAND_CMD_READOOB, 0, page);
			chip->read_buf(mtd, chip->oob_poi, mtd->oobsize);
			chip->cmdfunc(mtd, NAND_CMD_READ0, 0, page);
		}

		for (n = 0; eccsteps; eccsteps--, p += eccsize) {	

			memset (eccbuff, 0x00, sizeof eccbuff);

			for (i = 0; i < eccbytes; i++, n++) {
				ecccode[i] = chip->oob_poi[eccpos[n]];
#if defined(DEBUG_ECCCODE)
				printf("ecccode[%d] = %x eccpos=%d\n", i, ecccode[i], eccpos[n]);
#endif
			}

			/* set hw ecc */
			ecc_reset_decoder();	/* discon syndrome */
			ecc_write_ecc_decode((unsigned int*)ecccode, eccbytes);
			ecc_decode_enable(eccsize);

			/* read data */
#if !defined(DEBUG_ECCCODE)
			chip->read_buf(mtd, p, eccsize);
#else
//			for (i=0; i< eccsize; i++, p++) {
//				*p = readb(chip->IO_ADDR_R);
//				DBGOUT("r%d:cnt=%x\n", i, _pNCTRL->NFCNT);
//			}
#endif

#if NAND_HW_ECC_WORKAROUND

			chip->select_chip(mtd, -1);
			dummy = readb(chip->IO_ADDR_R); /* ksw : work around for decoder.*/
			chip->select_chip(mtd, 0);
#endif
//			DBGOUT("RE:cnt=%x\n", _pNCTRL->NFCNT);
			
			ecc_wait_for_decode();
			
			err = ecc_decode_error();

			if (err) { /* err : Temporary read check */
				ERROUT("ecc_decode_error\n");
				/* check erase status */
				for (i = 0 ; eccbytes > i; i++)
					if (0xFF != ecccode[i]) break;
				if (i == eccbytes) {
					is_erasedpage = 1;
					continue;
				}

				ecc_start_correct(eccsize);
				ecc_wait_for_correct();

#if (0)
				if (((_pNCTRL->NFECCSTATUS & NX_NFECCSTATUS_ELPERR) >>  16) >= chip->ecc.strength)
					ERROUT("  page: %d, step:%d, numerr: %d, elperr: %d\n", page, 
							(chip->ecc.steps-eccsteps),
							((_pNCTRL->NFECCSTATUS & NX_NFECCSTATUS_NUMERR) >>  4),
							((_pNCTRL->NFECCSTATUS & NX_NFECCSTATUS_ELPERR) >> 16));
#endif

				/* correct Error */
				errcnt = ecc_get_err_location((unsigned int *)errpos);
				if (0 >= errcnt) {
					ERROUT("page %d step %2d ecc error, can't %s ...\n",
						page, (chip->ecc.steps-eccsteps), 0==errcnt?"detect":"correct");
					failed++;
					ret = -EBADMSG;
					ERROUT("read retry page %d, retry: %d \n", page, retry);
					goto retry_rd;	/* EXIT */
				} else {
					ERROUT("page %d step %2d, ecc error %2d\n", page, (chip->ecc.steps-eccsteps), errcnt);
					for (k = 0; errcnt > k; k++) {
						errdat = (uint32_t*)p;
						ERROUT("offs = 0x%04x: 0x%4x -> ",
							((chip->ecc.steps-eccsteps)*eccsize)+((errpos[k]/32)*4), errdat[errpos[k]/32]);
						/* Error correct */
						if (errpos[k] >= eccrange) 		/* skip ecc error in oob */
							continue;
						errdat[errpos[k] / 32] ^= 1U<<(errpos[k] % 32);
						ERROUT("0x%4x\n", errdat[errpos[k]/32]);
					}

					#if !(U_BOOT_NAND)
					corrected += errcnt;
					#endif
					max_bitflips = max_t(unsigned int, max_bitflips, errcnt);
				}
			}
		}

#ifdef CONFIG_NAND_RANDOMIZER
		if (!no_nand_randomize && !is_erasedpage)
		{
			randomizer_page (page & pages_per_block_mask, buf, mtd->writesize);
			ERROUT("  page: %d ------->    derandomize\n", page);
		}
#endif

		mtd->ecc_stats.corrected += corrected;
		if (failed > 0)
			mtd->ecc_stats.failed++;

		DBGOUT("DONE nand_hw_ecc_read_page, ret=%d\n", ret);
		return max_bitflips;

retry_rd:
		retry++;
	} while (NAND_READ_RETRY > retry);

	mtd->ecc_stats.corrected += corrected;
	if (failed > 0)
		mtd->ecc_stats.failed++;

	printf("FAIL nand_hw_ecc_read_page, ret=%d, retry=%d\n", ret, retry);
	return ret;
}

int nxp4330_nand_hw_ecc_read_page(struct mtd_info *mtd, struct nand_chip *chip,uint8_t *buf, int oob_required, int page)
{
	int ret;
	ret = nand_hw_ecc_read_page(mtd,chip,buf,oob_required,page);
	return ret;
}

int nand_hw_ecc_write_page(struct mtd_info *mtd, struct nand_chip *chip,
				  const uint8_t *buf, int oob_required)
{
	int i, n;
	int eccsteps = chip->ecc.steps;
	int eccbytes = chip->ecc.bytes;
	int eccsize  = chip->ecc.size;
	
	uint8_t  *ecccode = (uint8_t*)eccbuff;
	uint32_t *eccpos   = chip->ecc.layout->eccpos;
	uint8_t  *p = (uint8_t *)buf;

	DBGOUT("nand_hw_ecc_write_page\n");

    ecc_setup_encoder();

	/* write data and get ecc */
	for (n = 0; eccsteps; eccsteps--, p += eccsize) {
		memset (eccbuff, 0x00, sizeof eccbuff);

		ecc_encode_enable();

		chip->write_buf(mtd, p, eccsize);

		/* get ecc code from ecc register */
		ecc_wait_for_encode();
		ecc_read_ecc_encode((uint32_t *)ecccode, eccbytes);

		/* set oob with ecc */
		for (i = 0; i < eccbytes; i++, n++) {
			chip->oob_poi[eccpos[n]] = ecccode[i];
#if defined(DEBUG_ECCCODE)
			DBGOUT("ecccode[%d] = %x\n", i, ecccode[i]);
#endif
		}
	}

	/* write oob */
	chip->write_buf(mtd, chip->oob_poi, mtd->oobsize);
	
	return 0;
}

int nand_hw_write_page(struct mtd_info *mtd, struct nand_chip *chip,
			uint32_t offset, int data_len, const uint8_t *buf,
		int oob_required, int page, int cached, int raw)
			   /* const uint8_t *buf, int oob_required, int page, int cached, int raw) */
{
#ifdef CONFIG_MTD_NAND_VERIFY_WRITE
	struct mtd_ecc_stats stats;
	int ret = 0;
#endif
	int status;
	uint8_t *funcbuf = (uint8_t *)buf;
	DBGOUT("nand_hw_write_page enter\n");
#ifdef CONFIG_NAND_RANDOMIZER
	if (!no_nand_randomize && randomize_buf) {
		memcpy (randomize_buf, buf, mtd->writesize);

		randomizer_page (page & pages_per_block_mask, randomize_buf, mtd->writesize);

		funcbuf = randomize_buf;
	}
#endif

	DBGOUT("nand_hw_write_page page %d, raw=%d\n",page, raw);
	chip->cmdfunc(mtd, NAND_CMD_SEQIN, 0x00, page);

	/* for hynix H27UBG8T2BTR */
	//ndelay(200);

	/* not verify */
	if (raw)
		status = chip->ecc.write_page_raw(mtd, chip, funcbuf, oob_required);
	else
		status = chip->ecc.write_page(mtd, chip, funcbuf, oob_required);

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
	ret = chip->ecc.read_page(mtd, chip, verify_buf, oob_required, page);
	if (ret < 0)
	{
		ERROUT ("  read page (%d) for write-verify failed!\n", page);
		return -EIO; //		return ret;
	}

	if (memcmp (verify_buf, buf, mtd->writesize))
	{
		ERROUT ("fail verify %d page\n", page);
		return -EIO;
	}

	chip->cmdfunc(mtd, NAND_CMD_STATUS, -1, -1);
#endif
	return 0; // mtd->ecc_stats.corrected - stats.corrected ? -EUCLEAN : 0
}

/**
 * nand_read_page_hwecc - [REPLACEABLE] hardware ECC based page read function
 * @mtd: mtd info structure
 * @chip: nand chip info structure
 * @buf: buffer to store read data
 * @oob_required: caller requires OOB data read to chip->oob_poi
 * @page: page number to read
 *
 * Not for syndrome calculating ECC controllers which need a special oob layout.
 */
static int nand_read_page_hwecc(struct mtd_info *mtd, struct nand_chip *chip,
				uint8_t *buf, int oob_required, int page)
{
	int i, eccsize = chip->ecc.size;
	int eccbytes = chip->ecc.bytes;
	int eccsteps = chip->ecc.steps;
	uint8_t *p = buf;
	uint8_t *ecc_calc = chip->buffers->ecccalc;
	uint8_t *ecc_code = chip->buffers->ecccode;
	uint32_t *eccpos = chip->ecc.layout->eccpos;
	unsigned int max_bitflips = 0;
	DBGOUT("nand_read_page_hwecc\n");
	for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {
		chip->ecc.hwctl(mtd, NAND_ECC_READ);
		chip->read_buf(mtd, p, eccsize);
		chip->ecc.calculate(mtd, p, &ecc_calc[i]);
	}
	chip->read_buf(mtd, chip->oob_poi, mtd->oobsize);

	for (i = 0; i < chip->ecc.total; i++)
		ecc_code[i] = chip->oob_poi[eccpos[i]];

	eccsteps = chip->ecc.steps;
	p = buf;

	for (i = 0 ; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {
		int stat;

		stat = chip->ecc.correct(mtd, p, &ecc_code[i], &ecc_calc[i]);
		if (stat < 0) {
			mtd->ecc_stats.failed++;
		} else {
			mtd->ecc_stats.corrected += stat;
			max_bitflips = max_t(unsigned int, max_bitflips, stat);
		}
	}
	return max_bitflips;
}


/*
 * Check if the chip configuration meet the datasheet requirements.

 * If our configuration corrects A bits per B bytes and the minimum
 * required correction level is X bits per Y bytes, then we must ensure
 * both of the following are true:
 *
 * (1) A / B >= X / Y
 * (2) A >= X
 *
 * Requirement (1) ensures we can correct for the required bitflip density.
 * Requirement (2) ensures we can correct even when all bitflips are clumped
 * in the same sector.
 */
static bool nand_ecc_strength_good(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct nand_ecc_ctrl *ecc = &chip->ecc;
	int corr, ds_corr;

	if (ecc->size == 0 || chip->ecc_step_ds == 0)
		/* Not enough information */
		return true;

	/*
	 * We get the number of corrected bits per page to compare
	 * the correction density.
	 */
	corr = (mtd->writesize * ecc->strength) / ecc->size;
	ds_corr = (mtd->writesize * chip->ecc_strength_ds) / chip->ecc_step_ds;

	return corr >= ds_corr && ecc->strength >= chip->ecc_strength_ds;
}

#if 0
int nand_write_page(struct mtd_info *mtd, struct nand_chip *chip,
		uint32_t offset, int data_len, const uint8_t *buf,
		int oob_required, int page, int cached, int raw)
{
	return 0;
}
	
int nand_read_page_hwecc_oob_first(struct mtd_info *mtd,
	struct nand_chip *chip, uint8_t *buf, int oob_required, int page)
{
	return 0;
}

int nand_write_page_hwecc(struct mtd_info *mtd, struct nand_chip *chip,
				  const uint8_t *buf, int oob_required)
{	
	return 0;
}

int nand_read_page_raw(struct mtd_info *mtd, struct nand_chip *chip,
			      uint8_t *buf, int oob_required, int page)
{
	return 0;
}

int nand_write_page_raw(struct mtd_info *mtd, struct nand_chip *chip,
				const uint8_t *buf, int oob_required)
{
	return 0;
}

int nand_read_oob_std(struct mtd_info *mtd, struct nand_chip *chip,
			     int page)
{
	return 0;
}

int nand_write_oob_std(struct mtd_info *mtd, struct nand_chip *chip,
			      int page)
{
	return 0;
}

int nand_read_subpage(struct mtd_info *mtd, struct nand_chip *chip,
			uint32_t data_offs, uint32_t readlen, uint8_t *bufpoi,
			int page)
{
	return 0;
}

int nand_write_subpage_hwecc(struct mtd_info *mtd,
				struct nand_chip *chip, uint32_t offset,
				uint32_t data_len, const uint8_t *buf,
				int oob_required)
{
	return 0;
}

int panic_nand_write(struct mtd_info *mtd, loff_t to, size_t len,
			    size_t *retlen, const uint8_t *buf)
{
	return 0;
}

int nand_read_oob(struct mtd_info *mtd, loff_t from,
			 struct mtd_oob_ops *ops)
{
	return 0;
}

 int nand_write_oob(struct mtd_info *mtd, loff_t to,
			  struct mtd_oob_ops *ops)
{
	return 0;
}

void nand_sync(struct mtd_info *mtd)
{
}

int nand_block_isreserved(struct mtd_info *mtd, loff_t ofs)
{
	return 0;
}

int nand_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
	return 0;
}
#endif

int nand_ecc_layout_hwecc(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct nand_ecclayout *layout = chip->ecc.layout;
	struct nand_oobfree *oobfree  = chip->ecc.layout->oobfree;
	uint32_t *eccpos = chip->ecc.layout->eccpos;
	int ecctotal = chip->ecc.total;
	int oobsize	 = mtd->oobsize;
	int i = 0, n = 0;
	int ret = 0;

	if (512 > mtd->writesize) {
		printf("NAND ecc: page size %d not support hw ecc\n",
			mtd->writesize);
		chip->ecc.mode 			= NAND_ECC_SOFT;
		chip->ecc.read_page 	= NULL;
		chip->ecc.read_subpage 	= NULL;
		chip->ecc.write_page 	= NULL;
		chip->ecc.layout		= NULL;
		//chip->verify_buf		= nand_sw_ecc_verify_buf;

		if ( chip->buffers &&
			!(chip->options & NAND_OWN_BUFFERS)) {
			kfree(chip->buffers);
			chip->buffers = NULL;
		}
//		ret = nxp4330_nand_scan_tail(mtd);
		printf("NAND ecc: Software \n");
		return ret;
	}

	if (ecctotal > oobsize)  {
		printf("\n");
		printf("==================================================\n");
		printf("error: %d bit hw ecc mode requires ecc %d byte	\n", ECC_HW_BITS, ecctotal);
		printf("       it's over the oob %d byte for page %d byte	\n", oobsize, mtd->writesize);
		printf("==================================================\n");
		printf("\n");
		return -EINVAL;
	}

	/*
	 * set ecc layout
	 */
	if (16 >= mtd->oobsize) {
		for (i = 0, n = 0; ecctotal>i; i++, n++) {
			if (5 == n) n += 1;	// Bad marker
			eccpos[i] = n;
		}
		oobfree->offset  = n;
		oobfree->length  = mtd->oobsize - ecctotal - 1;
		layout->oobavail = oobfree->length;

    	mtd->oobavail = oobfree->length;
		printf("hw ecc %2d bit, oob %3d, bad '5', ecc 0~4,6~%d (%d), free %d~%d (%d)\n",
			ECC_HW_BITS, oobsize, ecctotal+1-1, ecctotal, oobfree->offset,
			oobfree->offset+oobfree->length-1, oobfree->length);
	} else {

		oobfree->offset  = 2;
		oobfree->length  = mtd->oobsize - ecctotal - 2;
		layout->oobavail = oobfree->length;

		n = oobfree->offset + oobfree->length;
		for (i = 0; i < ecctotal; i++, n++)
			eccpos[i] = n;

    	mtd->oobavail = oobfree->length;
		printf("hw ecc %2d bit, oob %3d, bad '0,1', ecc %d~%d (%d), free 2~%d (%d)\n",
			ECC_HW_BITS, oobsize, oobfree->offset+oobfree->length, n-1,
			ecctotal, oobfree->length+2-1, oobfree->length);
	}

	/* must reset mtd */
	mtd->ecclayout = chip->ecc.layout;
	mtd->oobavail  = chip->ecc.layout->oobavail;
	DBGOUT("ecclayout 0x%X oobavail %d\n", mtd->ecclayout,mtd->oobavail);
	return ret;
}


int nand_hw_ecc_init_device(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	int eccbyte = 0, eccsize = ECC_PAGE_SIZE, eccidx;
	NX_MCUS_ECCMODE eccmode;
#if defined(CONFIG_MTD_NAND_ECC_HW)
	int i;
	u_char tmp, mfg, devid,sizeb;
//	struct nand_flash_dev *type = NULL;
	int cellinfo;
	int plane, plane_size, nand_size,erase_size;	
	int page_size;	
#endif
	/*
	 * HW ECC bytes:
	 *
	 *  4 bit ecc need " 4 * 13 =  52" bit (  6.5B) ecc code per  512 Byte
	 *  8 bit ecc need " 8 * 13 = 104" bit ( 13.0B) ecc code per  512 Byte
     * 12 bit ecc need "12 * 13 = 156" bit ( 19.5B) ecc code per  512 Byte
	 * 16 bit ecc need "16 * 13 = 208" bit ( 26.0B) ecc code per  512 Byte
	 * 24 bit ecc need "24 * 13 = 312" bit ( 39.0B) ecc code per  512 Byte
	 * 24 bit ecc need "24 * 14 = 336" bit ( 42.0B) ecc code per 1024 Byte
	 * 40 bit ecc need "40 * 14 = 560" bit ( 70.0B) ecc code per 1024 Byte
	 * 60 bit ecc need "60 * 14 = 840" bit (105.0B) ecc code per 1024 Byte
	 *
	 *  Page  512 Byte +  16 Byte
	 *  Page 2048 Byte +  64 Byte
	 *  Page 4096 Byte + 128 Byte
     *
     *  Page 8192 Byte + 436 Byte (MLC)
	 */
	switch (ECC_HW_BITS) {
	case  4: eccbyte =   7, eccidx = 13, eccmode = NX_MCUS_4BITECC;
			INX_BCH_INDEX = 0;
			if (512 != eccsize) goto _ecc_fail;
			break;
	case  8: eccbyte =  13, eccidx = 13,  eccmode = NX_MCUS_8BITECC;
			INX_BCH_INDEX = 1;
			if (512 != eccsize) goto _ecc_fail;
			break;
    case 12: eccbyte =  20, eccidx = 13,  eccmode = NX_MCUS_12BITECC;
    		INX_BCH_INDEX = 2;
    		if (512 != eccsize) goto _ecc_fail;
    		break;
	case 16: eccbyte =  26, eccidx = 13,  eccmode = NX_MCUS_16BITECC;
			INX_BCH_INDEX = 3;
			if (512 != eccsize) goto _ecc_fail;
			break;
	case 24: 
			if (eccsize == 512) {
				eccbyte = 39, eccidx = 13, eccmode = NX_MCUS_24BITECC_512;
				INX_BCH_INDEX = 4;
			} else {
				eccbyte = 42, eccidx = 14, eccmode = NX_MCUS_24BITECC;
				INX_BCH_INDEX = 5;
			}
			break;
	case 40: eccbyte =  70, eccidx = 14,  eccmode = NX_MCUS_40BITECC;
			INX_BCH_INDEX = 6;
			if (1024 != eccsize) goto _ecc_fail;
			break;
	case 60: eccbyte = 105, eccidx = 14,  eccmode = NX_MCUS_60BITECC;
			INX_BCH_INDEX = 7;
			if (1024 != eccsize) goto _ecc_fail;
			break;
	default:
		goto _ecc_fail;
		break;
	}


	iNX_BCH_VAR_M	 = eccidx;			/* 13 or 14 */
	iNX_BCH_VAR_T	 = ECC_HW_BITS;	/* 4, 8, 12, 16, 24, 40, 60 ... */
	iNX_BCH_VAR_TMAX = (eccsize == 512 ? 24 : 60);
	DBGOUT("%s ecc %d bit, eccsize=%d, eccbyte=%d, eccindex=%d\n",
		__func__, ECC_HW_BITS, eccsize, eccbyte, eccidx);

	
	chip->ecc.mode 			= NAND_ECC_HW;
	chip->ecc.size 			= eccsize;			/* per 512 or 1024 bytes */
	chip->ecc.bytes 		= eccbyte;
	chip->ecc.layout		= NULL;//&nand_ecc_oob;
	chip->ecc.read_page 	= nand_hw_ecc_read_page;
	chip->ecc.write_page 	= nand_hw_ecc_write_page;
	chip->write_page		= nand_hw_write_page;
	chip->ecc.strength		= ((eccbyte * 8 / fls (8*eccsize)) * 80 / 100);
	NX_MCUS_ResetNFECCBlock();
	NX_MCUS_SetECCMode(eccmode);
	
//#if defined(CONFIG_MTD_NAND_ECC_HW)
//	nxp2120_CreateLookupTable();

	chip->ecc.mode		= NAND_ECC_HW_OOB_FIRST;
	chip->ecc.read_oob = nxp4330_nand_read_oob;
/*	chip->ecc.hwctl	= nxp2120_nand_enable_hwecc;
	chip->ecc.calculate	= nxp2120_nand_calculate_ecc;
	chip->ecc.correct	= nxp2120_nand_correct_data;
	chip->ecc.read_page = nxp4330_nand_hw_ecc_read_page;
	chip->ecc.write_page = nand_hw_ecc_write_page;
	chip->ecc.read_oob = nxp2120_nand_read_oob;*/
	/*nand->ecc.write_oob = nxp2120_nand_write_oob;*/
	
	
//	chip->options |= NAND_HWECC_READECC_FIRST | NAND_HWECC_WRITE_ORGECCCODE;

	NX_MCUS_SetNFCSEnable(CTRUE);
	chip->cmd_ctrl(mtd, NAND_CMD_READID, NAND_CLE);
	chip->cmd_ctrl(mtd, 0x00, NAND_ALE);
	//chip->cmdfunc(mtd, NAND_CMD_READID, 0, -1);
	nand_dev_ready(0);
	mfg = readb(chip->IO_ADDR_R); /* Maf. ID */
	tmp = readb(chip->IO_ADDR_R); /* Device ID */
	devid = tmp;
	DBGOUT("mfg 0x%X tmp 0x%X\n",mfg,tmp);
/*	for (i = 0; nand_flash_ids[i].name != NULL; i++) {
		//printf("id:%x \n",nand_flash_ids[i].dev_id &0xFF);
		if (tmp == (nand_flash_ids[i].dev_id & 0xFF)) {
			type = &nand_flash_ids[i];
			break;
		}
	}*/

	cellinfo = readb(chip->IO_ADDR_R);	/* 3rd byte */
	tmp = readb(chip->IO_ADDR_R);		/* 4th byte */
	sizeb = readb(chip->IO_ADDR_R);		/* 5th byte */

	NX_MCUS_SetNFCSEnable(CFALSE);
	DBGOUT("1.mfg:%x 2.devid:%x 3.cellinfo %x 4.tmp %x 5.sizeb %x\n",mfg,devid,cellinfo,tmp,sizeb);

	if (((tmp&0x3)==0) || ((tmp&0x3)==0x01) || ((tmp&0x3)==0x02)|| ((tmp&0x3)==0x03)) {
		mtd->writesize = 1024 << (tmp & 0x03);
		if (((cellinfo >> 2) & 0x3) == 0) {
			nand_type = NXP_NAND_TYPE_SLC;
			chip->ecc.size = 512;
			chip->ecc.bytes	= 7;
			
			if ((1024 << (tmp & 0x3)) > 512) {
				//nand->ecc.read_page = s3c_nand_read_page_1bit;
				//nand->ecc.write_page = s3c_nand_write_page_1bit;
				//nand->ecc.read_oob = s3c_nand_read_oob_1bit;
				//nand->ecc.write_oob = s3c_nand_write_oob_1bit;
			//	chip->ecclayout = &nand_oob_64;

				chip->ecc.steps = 4;
			} else {				
			//	chip->ecclayout = &nand_oob_16;
				chip->ecc.steps = 1;
			}
		} else {
			nand_type = NXP_NAND_TYPE_MLC;
			chip->options |= NAND_NO_SUBPAGE_WRITE;	/* NOP = 1 if MLC */
			//chip->ecc.read_page = s3c_nand_read_page_4bit;
			//chip->ecc.write_page = s3c_nand_write_page_4bit;
			chip->ecc.size = 512;
			chip->ecc.bytes = 7;
			chip->ecc.mode = NAND_ECC_HW_OOB_FIRST;

		//	chip->ecclayout = &nand_oob_64;
			//nand->options &= ~NAND_HWECC_SPAREAREA;
		}
		/* calculate total size */
		//plane = 1 << (((sizeb >> 2) & 0x03) + 1);
		//plane_size = 8 << ((sizeb >> 4) & 0x07);
		
		//nand_size = plane * plane_size;
		DBGOUT("devid 0x%X\n",devid);
		switch(devid){
		case 0xF1://1Gb
			nand_size = 128;//*1024*1024;
			mtd->oobsize = (8 << ((tmp>>2) & 0x01)) * (mtd->writesize >> 9);
			break;
		case 0xDA://2Gb
			nand_size = 256;//*1024*1024;
			mtd->oobsize = (16 << ((tmp>>2) & 0x01)) * (mtd->writesize >> 9);
			break;
		case 0xDC://4Gb
			nand_size = 512;//*1024*1024;
			mtd->oobsize = (16 << ((tmp>>2) & 0x01)) * (mtd->writesize >> 9);
			DBGOUT("mtd->oobsize %d, (tmp>>2) 0x%x, ((tmp>>2) & 0x01) 0x%X,(16 << ((tmp>>2) & 0x01))) 0x%X, (mtd->writesize >> 9) 0x%X\n",
				mtd->oobsize,(tmp>>2),((tmp>>2) & 0x01),(16 << ((tmp>>2) & 0x01)),(mtd->writesize >> 9));
			break;
		}
		chip->chipsize = nand_size * 1024 * 1024;
		mtd->erasesize = (64 * 1024) << ((tmp>>4) & 0x03);		
		erase_size = (1 << (((tmp >> 4)&0x03)))*65536;
		
		switch(mtd->oobsize){
		case 128:
			chip->ecc.layout = &nand_oob_128;	
			break;
		}
		
		DBGOUT("ecc->layout 0x%X mtd->oobsize %d erase size=%d %d\n", chip->ecc.layout,mtd->oobsize,erase_size,mtd->erasesize);
		//printf("plane, planesize, nand_size = %d, %x, %x\n", plane, plane_size, nand_size);
		switch (tmp & 0x03) {
		case 0 : // 1KB page.
			addr_cycle = 3;
			if (nand_size > 64)
				addr_cycle = 4;
			chip->ecc.steps = 2;
			page_size = 1024;
			break;
		case 1 : // 2KB page
			// most large NAND has 2KB page, and if we have
			addr_cycle = 3;
			if (nand_size > 128) /* more than 128MByte we need one more addr cycle. ==> 256M >= size */
				addr_cycle = 4;
			chip->ecc.steps = 4;
			page_size = 2048;
			break;
		case 2 : // 4KB page
			addr_cycle = 3;
			if (nand_size > 256) /* more than 256MByte we need one more addr cycle. ==> 512M >= size */
				addr_cycle = 4;
			chip->ecc.steps = 8;
			page_size = 4096;
			break;
		case 3 : // 8KB page
			addr_cycle = 3;
			if (nand_size > 512) /* more than 1GByte */
				addr_cycle = 4;
			chip->ecc.steps = 16;
			page_size = 8192;
			break;
		}
//		type->pagesize=page_size;
//		type->erasesize=erase_size;
	} else {
		printf("You need to read Read id or to set default values\n");
	}
	chip->ecc.total = chip->ecc.bytes * chip->ecc.steps;
	DBGOUT("ecc total = %d\n", chip->ecc.total);
	chip->ecc.strength = 4;
	
//	mtd->writesize = 1024 << (tmp & 0x03);
//	tmp >>= 2;
//	mtd->oobsize = (16 << ((tmp>>2) & 0x01)) * (mtd->writesize >> 9);
//	tmp >>= 2;
	/* Calc blocksize. Blocksize is multiples of 64KiB */
//	mtd->erasesize = (64 * 1024) << ((tmp>>4) & 0x03);

	chip->page_shift = ffs(mtd->writesize) - 1;
	DBGOUT("mtd->writesize 0x%X, mtd->oobsize 0x%X, mtd->erasesize 0x%X, chip->pageshift %d\n",
		mtd->writesize,mtd->oobsize,mtd->erasesize,chip->page_shift);
	
	nand_ecc_layout_hwecc(mtd);
	
	
	
	return 0;

_ecc_fail:
	printf("Fail: not support ecc %d bits for pagesize %d !!!\n", ECC_HW_BITS, eccsize);
	return -EINVAL;
}
#endif /* CONFIG_MTD_NAND_ECC_HW */


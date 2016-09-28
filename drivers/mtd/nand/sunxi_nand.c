/*
 * Copyright (C) 2016 I4VINE Inc.,
 * All right reserved by ryu <jyryu@stcube.com> and following authurs
 * (C) Copyright 2016 STcube Inc.,
 * All right reserved by Seungwoo Kim <ksw@stcube.com>
 *
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
/*
#include <common.h>
#include <errno.h>
#include <nand.h>

#include <asm/io.h>
#include <asm/gpio.h>

#include <asm/io.h>
#include <asm/errno.h>
*/

#include "sunxi_nand.h"

#define SUNXI_NFC_BASE_ADDRESS	0x01C03000
#define SUNXI_NFC_IO_ADDRESS	(SUNXI_NFC_BASE_ADDRESS + 0)

/*
 * calling from nand_init (drivers/mtd/nand/nand.c)
 */
int board_nand_init(struct nand_chip *chip)
{
	struct mtd_info  *mtd = &nand_info[0];
	struct sunxi_nfc *nfc;
	struct sunxi_nand_chip *sunxi_nand;
	struct nand_ecc_ctrl *ecc = &chip->ecc;

	int ret = 0;

	nand_clock_setup();
	
	nfc = kzalloc(sizeof(struct sunxi_nfc), GFP_KERNEL);
	nfc->regs = (void __iomem *)SUNXI_NFC_BASE_ADDRESS;
	nfc->num_of_chips = 1;
	nfc->chips[0] = kzalloc(sizeof(struct sunxi_nand_chip), GFP_KERNEL);

	nfc->chips[0]->nand = chip;
	nfc->chips[0]->mtd = mtd;

	nfc->chips[0]->chip_sel.cs.type = CSRB_NATIVE;
	nfc->chips[0]->chip_sel.cs.info.nativeid = 0;

	nfc->chips[0]->chip_sel.rb.type = CSRB_NATIVE;
	nfc->chips[0]->chip_sel.rb.info.nativeid = 0;
	sunxi_nand = nfc->chips[0];
	nfc->current_chip = -1;

	NAND_DBG("%s nand_chip 0x%X, sunxi_nand 0x%X mtd 0x%X nfc 0x%X\n",
		__func__,chip,nfc->chips[0], mtd, nfc);

	/*
	* nand callbacks
	*/
	chip->IO_ADDR_R 	= (void __iomem *)SUNXI_NFC_IO_ADDRESS;
	chip->IO_ADDR_W 	= (void __iomem *)SUNXI_NFC_IO_ADDRESS;
	chip->cmd_ctrl 		= sunxi_nfc_cmd_ctrl;
	chip->dev_ready 	= sunxi_nfc_dev_ready;
	chip->select_chip 	= sunxi_nfc_select_chip;
	chip->read_buf 		= sunxi_nfc_read_buf;
	chip->write_buf 	= sunxi_nfc_write_buf;
	chip->read_byte 	= sunxi_nfc_read_byte;
	//chip->write_buf 	= nx_write_buf;
	chip->chip_delay 	= 15;
	chip->options       = 0;
	chip->priv 			= nfc;

#if STRUCT_NANDCHIP_HAVE_ONFI_TIMING
	chip->ecc_step_ds = 1024;
	chip->ecc_strength_ds = 8;
#else
	sunxi_nand->ecc_step_ds = 1024;
	sunxi_nand->ecc_strength_ds = 8;
#endif

	/*
	 * error correct mode
	 */
#if defined (CONFIG_MTD_NAND_ECC_HW)
	ret = sunxi_nand_ecc_init(mtd, ecc);

	//printk(KERN_INFO "NAND ecc: Hardware (delay %d)\n", chip->chip_delay);
#else
	chip->ecc.mode = NAND_ECC_SOFT;
	printk(KERN_INFO "NAND ecc: Software \n");
#endif

	sunxi_nand_chip_init_timings(sunxi_nand);

	return ret;
}


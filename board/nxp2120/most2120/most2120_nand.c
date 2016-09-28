/*
 *
 * Copyright(c) 2010 Seungwoo Kim, MOSTiTECH co., Ltd.
 * Seungwoo Kim <ksw@mostitech.com>
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
#include <linux/mtd/nand.h>
#include <asm/arch/regs.h>

int board_nand_init(struct nand_chip *nand)
{
	// Don't have to initialize hardware, hardware already ewnabled.
	nand->options = 0; /* no special options in */
#if !defined(CFG_NAND_HWECC)
	return nxp2120_nand_init(nand, 0);
#else
	return nxp2120_nand_init(nand, 2);
#endif
}

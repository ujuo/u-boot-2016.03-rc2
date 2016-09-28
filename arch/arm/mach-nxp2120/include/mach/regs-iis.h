/* linux/include/asm-arm/plat-s3c24xx/regs-s3c2412-iis.h
 *
 * Copyright 2010 MOSTiTECH co., ltd. 
 *  Seungwoo Kim <ksw@mostitech.com>
 * Copyright 2007 Simtec Electronics <linux@simtec.co.uk>
 *	http://armlinux.simtec.co.uk/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * S3C64XX IIS register definition
*/

#ifndef __ASM_ARCH_REGS_S3C64XX_IIS_H
#define __ASM_ARCH_REGS_S3C64XX_IIS_H

#define IISCON			(0x00)
#define IISMOD			(0x04)
#define IISFIC			(0x08)
#define IISPSR			(0x0C)
#define IISTXD			(0x10)
#define IISRXD			(0x14)

#define IISCON_FRXORSTATUS	(1 << 19)
#define IISCON_FRXORINTEN	(1 << 18)
#define IISCON_FTXURSTATUS	(1 << 17)
#define IISCON_FTXURINTEN	(1 << 16)
#define IISCON_FTXURINTEN	(1 << 16)
#define IISCON_LRINDEX		(1 << 11)
#define IISCON_TXFIFO_EMPTY	(1 << 10)
#define IISCON_RXFIFO_EMPTY	(1 << 9)
#define IISCON_TXFIFO_FULL	(1 << 8)
#define IISCON_RXFIFO_FULL	(1 << 7)
#define IISCON_TXDMA_PAUSE	(1 << 6)
#define IISCON_RXDMA_PAUSE	(1 << 5)
#define IISCON_TXCH_PAUSE	(1 << 4)
#define IISCON_RXCH_PAUSE	(1 << 3)
#define IISCON_TXDMA_ACTIVE	(1 << 2)
#define IISCON_RXDMA_ACTIVE	(1 << 1)
#define IISCON_IIS_ACTIVE	(1 << 0)

#define IISMOD_BLC_24BIT	(2 << 13)
#define IISMOD_BLC_8BIT	    (1 << 13)
#define IISMOD_BLC_16BIT	(0 << 13)
#define IISMOD_BLC_MASK 	(3 << 13)
#define IISMOD_CDCLK_EXTERNAL	(1 << 12)
#define IISMOD_CDCLK_INTERNAL	(0 << 12)
#define IISMOD_CDCLK_MASK	(1 << 12)
#define IISMOD_MASTER_PCLK	(0 << 10)
#define IISMOD_MASTER_CLKAUDIO	(1 << 10)
#define IISMOD_SLAVE_PCLK	(2 << 10)
#define IISMOD_SLAVE_CLKAUDIO	(3 << 10)
#define IISMOD_MASTER_SLAVE_MASK    (2 << 10)
#define IISMOD_MASTER_MASK	(3 << 10)
#define IISMOD_MODE_TXONLY	(0 << 8)
#define IISMOD_MODE_RXONLY	(1 << 8)
#define IISMOD_MODE_TXRX	(2 << 8)
#define IISMOD_MODE_MASK	(3 << 8)
#define IISMOD_LR_LLOW		(0 << 7)
#define IISMOD_LR_RLOW		(1 << 7)
#define IISMOD_LR_MASK		(1 << 7)
#define IISMOD_SDF_IIS		(0 << 5)
#define IISMOD_SDF_MSB		(1 << 5)
#define IISMOD_SDF_LSB		(2 << 5)
#define IISMOD_SDF_MASK		(3 << 5)
#define IISMOD_RCLK_256FS	(0 << 3)
#define IISMOD_RCLK_512FS	(1 << 3)
#define IISMOD_RCLK_384FS	(2 << 3)
#define IISMOD_RCLK_768FS	(3 << 3)
#define IISMOD_RCLK_MASK 	(3 << 3)
#define IISMOD_BCLK_32FS	(0 << 1)
#define IISMOD_BCLK_48FS	(1 << 1)
#define IISMOD_BCLK_16FS	(2 << 1)
#define IISMOD_BCLK_24FS	(3 << 1)
#define IISMOD_BCLK_MASK	(3 << 1)

#define IISPSR_PSREN		(1 << 15)
#define IISPSR_PSVAL(x)     (((x) & 0x3F) << 8)
#define IISPSR_PSVALA_MASK  (0x3F << 8)

#define IISFIC_TXFLUSH		(1 << 15)
#define IISFIC_RXFLUSH		(1 << 7)
#define IISFIC_TXCOUNT(x)	(((x) >>  8) & 0xf)
#define IISFIC_RXCOUNT(x)	(((x) >>  0) & 0xf)



#endif /* __ASM_ARCH_REGS_S3C64XX_IIS_H */


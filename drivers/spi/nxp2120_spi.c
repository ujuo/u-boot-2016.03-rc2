/*
 * Copyright (C) 2013, STCube Inc.
 * All right reserved by Seungwoo Kim <ksw@stcube.com>
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
 *
 */

#include <common.h>
#include <malloc.h>
#include <spi.h>
#include <asm/io.h>

/* NXP2120 SPI flash reader/writer.
   Experimental implementation by Seungwoo Kim.

	2013.  8. 14. Ver 0.5 initial stage.
*/

/*
  NXP2120 GPIO REGISTERS
*/

#define NXP2120_GPIO_A_BASE	0xC000A000
#define NXP2120_GPIOA_OUT			(NXP2120_GPIO_A_BASE + 0x00)
#define NXP2120_GPIOA_OUTENB		(NXP2120_GPIO_A_BASE + 0x04)
#define NXP2120_GPIOA_ALTFUNSEL0	(NXP2120_GPIO_A_BASE + 0x20)
#define NXP2120_GPIOA_ALTFUNSEL1	(NXP2120_GPIO_A_BASE + 0x24)

#define NXP2120_FRM_GPIO_A	28
#define NXP2120_CLK_GPIO_A	29
#define NXP2120_RXD_GPIO_A	30
#define NXP2120_TXD_GPIO_A	31
/*
  NXP2120 CONTROL REGISTERS
*/
#define NXP2120_SPIREG(x)	(x + 0xC0007800)
#define NXP2120_SPISSPCONT0	NXP2120_SPIREG(0x00)
#define NXP2120_SPISSPCONT1	NXP2120_SPIREG(0x02)
#define NXP2120_SPISSPDATA	NXP2120_SPIREG(0x04)
#define NXP2120_SPISSPSTAT	NXP2120_SPIREG(0x06)
#define NXP2120_SPI_RX_BRCV_SZ	NXP2120_SPIREG(0x08)
#define NXP2120_SPI_CLKENB	NXP2120_SPIREG(0x40)
#define NXP2120_SPI_CLKGEN	NXP2120_SPIREG(0x44)

/* SSPCONT0 BITS */
#define SSPCONT0_BURST_RCV	(1 << 15)
#define SSPCONT0_BRCV_EN	(1 << 13)
#define SSPCONT0_DMA_EN		(1 << 12)
#define SSPCONT0_PIOMODE    (0 << 12)
#define SSPCONT0_SPI_ENB	(1 << 11)
#define SSPCONT0_SPI_DISB	(0 << 11)
#define SSPCONT0_FIFO_CLR	(1 << 10)
#define SSPCONT0_USE_EXTCLK	(1 << 9)
#define SSPCONT0_USE_INTCLK (0 << 9)
#define SSPCONT0_NUMBIT_MASK	(0xF << 5)
#define SSPCONT0_NUMBIT(X)	((X-1) << 5)
#define SSPCONT0_DIVCNT_MASK	(0x1F << 0)
#define SSPCONT0_DIVCNT(X)	((X-1) << 0)
/* SSPCONT1 BITS */
#define SSPCONT1_BYTESWAP	(1 << 5)
#define SSPCONT1_NO_BYTESWAP	(0 << 5)
#define SSPCONT1_SLAVESEL	(1 << 4)
#define SSPCONT1_MASTERSEL	(0 << 4)
#define SSPCONT1_SCLK_POL_NORMAL	(1 << 3)
#define SSPCONT1_SCLK_POL_INVERT	(0 << 3)
#define SSPCONT1_SCLKSH_FORMATB		(1 << 2)
#define SSPCONT1_SCLKSH_FORMATA		(0 << 2)
#define SSPCONT1_MODE_SPP	(0 << 0)
#define SSPCONT1_MODE_SPI	(1 << 0)
/* SSPSTAT BITS */
#define SSPSTAT_IRQEENB		(1 << 15)
#define SSPSTAT_IRQWENB		(1 << 14)
#define SSPSTAT_IRQRENB		(1 << 13)
#define SSPSTAT_TXSHEMPTY	(1 << 8)
#define SSPSTAT_IRQE		(1 << 6)
#define SSPSTAT_IRQW		(1 << 5)
#define SSPSTAT_IRQR		(1 << 4)
#define SSPSTAT_WFFFULL		(1 << 3)
#define SSPSTAT_WFFEMPTY	(1 << 2)
#define SSPSTAT_RFFFULL		(1 << 1)
#define SSPSTAT_RFFEMPTY	(1 << 0)
/* CLKENB BITS */
#define SPI_CLKENB_PCLKMODE (1 << 3)
#define SPI_CLKENB_CLKGENENB	(1 << 2)
/* CLKGEN BITS */
#define SPI_CLKGEN_CLKDIV(X)	((X-1) << 5)
#define SPI_CLKGEN_CLKSRCSEL(X)	(X << 2)

struct nxp_spi_slave {
	struct spi_slave slave;
	unsigned long	base;
	int				bytecnt;
};

static inline struct nxp_spi_slave *to_nxp_spi_slave(struct spi_slave *slave)
{
	return container_of(slave, struct nxp_spi_slave, slave);
}

static inline u32 reg_read32(unsigned int addr)
{
	return *(volatile unsigned int*)addr;
}

static inline void reg_write32(unsigned int addr, u32 val)
{
	*(volatile unsigned int*)addr = val;
}

static inline u16 reg_read16(unsigned int addr)
{
	return *(volatile unsigned short int*)addr;
}

static inline void reg_write16(unsigned int addr, u16 val)
{
	*(volatile unsigned short int*)addr = val;
}

void spi_cs(int enable)
{
	u32 val;
	
	val = reg_read32(NXP2120_GPIOA_OUT);
	if (enable) {
		val &= ~(1 << 28);
	} else {
		val |= 1 << 28;
	}
	reg_write32(NXP2120_GPIOA_OUT, val);
}

int spi_xfer(struct spi_slave *slave, unsigned int bitlen, const void *dout,
		void *din, unsigned long flags)
{
	int n_bytes = (bitlen) / 8;
	//int x_bytes;
	struct nxp_spi_slave *nxp = to_nxp_spi_slave(slave);
	u8 *out_l, *in_l;
	u16 val;
	int do_end = 0;
	int i;

	if (bitlen % 8)
		n_bytes++;

	if (flags & SPI_XFER_BEGIN) {
		val = reg_read16(NXP2120_SPISSPCONT0);
		val &= ~SSPCONT0_FIFO_CLR;
		reg_write16(NXP2120_SPISSPCONT0, val | SSPCONT0_FIFO_CLR );
		reg_write16(NXP2120_SPISSPCONT0, val );		
		spi_cs(1);
	}
	in_l = (u8 *)din;
	out_l = (u8 *)dout;
	
	//x_bytes = n_bytes + nxp->bytecnt;
	//if (dout != NULL)
	//	printf("cmd:%x len=%d nb=%d\n", dout, bitlen, n_bytes);
	//if (din != NULL)
	//	printf("in: len=%d nb=%d\n", bitlen, n_bytes);
	
	//if (flags & SPI_XFER_END) {
	do_end = 1;
	//}
	
	i = 0;
	while (n_bytes > 0) {
		if (out_l != NULL) {
			reg_write16(NXP2120_SPISSPDATA, *out_l);
			out_l++;
		} else {
			reg_write16(NXP2120_SPISSPDATA, 0);
		}
		n_bytes--;
		i++;
		if (do_end) {
			val = reg_read16(NXP2120_SPISSPCONT0);
			reg_write16(NXP2120_SPISSPCONT0, val | SSPCONT0_SPI_ENB );
			do_end = 0;
		}
		while ((reg_read16(NXP2120_SPISSPSTAT) & 1) != 0) ;
		
		if (in_l != NULL) {
			*in_l = reg_read16(NXP2120_SPISSPDATA);
			in_l++;
		} else {
			reg_read16(NXP2120_SPISSPDATA);
		}
	}
	i = 0;
	while ((reg_read16(NXP2120_SPISSPSTAT) & 4) == 0) {
		// Do something...
		i++;
		if (i > 0x200000) break;
	}
	i = 0;
	while ((reg_read16(NXP2120_SPISSPSTAT) & 0x100) == 0) {
		i++;
		if (i > 0x200) break;
	}
	// Now we want to xmit buffer empty and xmit shift register empty 
	if (flags & SPI_XFER_END) {	
		val = reg_read16(NXP2120_SPISSPCONT0);
		val &= ~SSPCONT0_SPI_ENB;
		reg_write16(NXP2120_SPISSPCONT0, val );
		spi_cs(0);
	}

	// For safety
	while ((reg_read16(NXP2120_SPISSPSTAT) & 1) == 0) {
		reg_read16(NXP2120_SPISSPDATA);
	}

	return 0;
}

void spi_init(void)
{
	u32 val;
	// Setup GPIO
	val = reg_read32(NXP2120_GPIOA_OUTENB);
	val |= 1 << 28;
	reg_write32(NXP2120_GPIOA_OUTENB, val);
	val = reg_read32(NXP2120_GPIOA_ALTFUNSEL1);
	// Upper 4bits are SPI
	val &= 0x00FFFFFF;
	val |= 0x54000000; // ALT FUNCTION 1 for all 4bits but 28th bit.
	reg_write32(NXP2120_GPIOA_ALTFUNSEL1, val);
	// Setup CLKENB
	reg_write32(NXP2120_SPI_CLKENB, SPI_CLKENB_PCLKMODE );
	// Setup CLKGEN -- 192MHz / 4 = 48MHz
	reg_write32(NXP2120_SPI_CLKGEN, (SPI_CLKGEN_CLKDIV(4) | SPI_CLKGEN_CLKSRCSEL(1)) );
	// Setup CONT0
	val = 	SSPCONT0_PIOMODE | SSPCONT0_SPI_DISB | SSPCONT0_FIFO_CLR |
			SSPCONT0_USE_INTCLK | SSPCONT0_NUMBIT(8);
	val |= SSPCONT0_DIVCNT(2); // 24MHz
	reg_write16(NXP2120_SPISSPCONT0 , val);
	// Now Clear FIFO_CLR bit
	val &= ~SSPCONT0_FIFO_CLR;
	reg_write16(NXP2120_SPISSPCONT0 , val);
	// Supply operation clock
	reg_write32(NXP2120_SPI_CLKENB, SPI_CLKENB_PCLKMODE | SPI_CLKENB_CLKGENENB );
	// Seup CONT1
	val =  	SSPCONT1_NO_BYTESWAP | SSPCONT1_MASTERSEL |
			SSPCONT1_SCLK_POL_INVERT | SSPCONT1_SCLKSH_FORMATB | SSPCONT1_MODE_SPI;
	reg_write16(NXP2120_SPISSPCONT1, val);
}

struct spi_slave *spi_setup_slave(unsigned int bus, unsigned int cs,
			unsigned int max_hz, unsigned int mode)
{
	unsigned int ctrl_reg;
	struct nxp_spi_slave *nxps;

	nxps = malloc(sizeof(struct nxp_spi_slave));
	if (!nxps)
		return NULL;

	/* NXP SSP/SSI is only 1 */ 
	nxps->base = NXP2120_SPIREG(0x0);
	nxps->bytecnt = 0;
	spi_init();

	return &nxps->slave;
}

void spi_free_slave(struct spi_slave *slave)
{
	free(slave);
}

int spi_claim_bus(struct spi_slave *slave)
{
	struct nxp_spi_slave *nxps = to_nxp_spi_slave(slave);

	// Do nothing for NXP SPI?
	return 0;
}

void spi_release_bus(struct spi_slave *slave)
{
	/* TODO: Shut the controller down */
}

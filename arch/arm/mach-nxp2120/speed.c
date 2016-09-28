/*
 * (C) Copyright 2001-2004
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * (C) Copyright 2002
 * David Mueller, ELSOFT AG, d.mueller@elsoft.ch
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

/* This code should work for both the S3C2400 and the S3C2410
 * as they seem to have the same PLL and clock machinery inside.
 * The different address mapping is handled by the s3c24xx.h files below.
 */

#include <common.h>
#include <asm/arch/regs.h>

//------------------------------------------------------------------------------
// PLL 0/1 setting
//------------------------------------------------------------------------------
// Fout = (M*Fin) / (P*S), where P = PDIV, M = MDIV, S = 2^SDIV
//------------------------------------------------------------------------------
#define	CFG_SYS_PLL_PMSFUNC(_FIN_, _P_, _M_, _S_)			\
	((_M_) * ((_FIN_)/((_P_)*(1UL<<(_S_)))))
/* 12MHz */
#define CFG_SYS_PLLFIN  CONFIG_SYS_CLK_FREQ

static U32  pre_fclk = 0;
static U32  pre_mclk = 0;

u_int cpu_get_clock_hz(int clk)
{
	unsigned int clkmode[2], pllreg[2], clock_hz = 0;
	unsigned int pll[4], sel, dvo, div2, div3, fclk, mclk, bclk, pclk;
	int i;

	struct NX_CLKPWR_RegisterSet *const pReg = (struct NX_CLKPWR_RegisterSet*)PHY_BASEADDR_CLKPWR_MODULE;

	clkmode[0] 	= pReg->CLKMODEREG0;
	clkmode[1] 	= pReg->CLKMODEREG1;
	pllreg [0]	= pReg->PLLSETREG[0];
	pllreg [1]	= pReg->PLLSETREG[1];

	for ( i=0 ; 2 > i; i++ ) {
		unsigned int 	pdiv, mdiv, sdiv;
        unsigned int     P_bitpos, M_bitpos, S_bitpos;
        unsigned int     P_mask, M_mask, S_mask;

		const unsigned int PLL0_PDIV_BIT_POS 	= 18;
		const unsigned int PLL0_MDIV_BIT_POS 	=  8;
		const unsigned int PLL0_SDIV_BIT_POS 	=  0;
        const unsigned int PLL0_PDIV_MASK       = 0xFF;
        const unsigned int PLL0_MDIV_MASK       = 0x3FF;
        const unsigned int PLL0_SDIV_MASK       = 0xFF;

		const unsigned int PLL1_PDIV_BIT_POS 	= 18;
		const unsigned int PLL1_MDIV_BIT_POS 	=  8;
		const unsigned int PLL1_SDIV_BIT_POS 	=  0;
        const unsigned int PLL1_PDIV_MASK       = 0x3F;
        const unsigned int PLL1_MDIV_MASK       = 0x3FF;
        const unsigned int PLL1_SDIV_MASK       = 0xFF;

        if ( 0 == i ) {
            P_bitpos = PLL0_PDIV_BIT_POS;
            P_mask   = PLL0_PDIV_MASK;
            M_bitpos = PLL0_MDIV_BIT_POS;
            M_mask   = PLL0_MDIV_MASK;
            S_bitpos = PLL0_SDIV_BIT_POS;
            S_mask   = PLL0_SDIV_MASK;
        } else {
            P_bitpos = PLL1_PDIV_BIT_POS;
            P_mask   = PLL1_PDIV_MASK;
            M_bitpos = PLL1_MDIV_BIT_POS;
            M_mask   = PLL1_MDIV_MASK;
            S_bitpos = PLL1_SDIV_BIT_POS;
            S_mask   = PLL1_SDIV_MASK;
        }

		pdiv	= (pllreg[i] >> P_bitpos) & P_mask;
		mdiv	= (pllreg[i] >> M_bitpos) & M_mask;
		sdiv	= (pllreg[i] >> S_bitpos) & S_mask;

		pll[i]	= CFG_SYS_PLL_PMSFUNC( CFG_SYS_PLLFIN, pdiv, mdiv, sdiv );
	}

	dvo 	= (clkmode[0]>>( 0+ 0)) & 0xF;
	sel		= (clkmode[0]>>( 4+ 0)) & 0x3;	//	NX_ASSERT( 2 > sel );
	div2	= (clkmode[0]>>( 8+ 0)) & 0xF;

	fclk = pll[sel] / (dvo+1);
	pll[3] = fclk;

	dvo	    = (clkmode[1]>>(  0+ 0)) & 0xF;
	sel		= (clkmode[1]>>(  4+ 0)) & 0x3;//	NX_ASSERT( 2 != sel );
	div2	= (clkmode[1]>>(  8+ 0)) & 0xF;
	div3	= (clkmode[1]>>( 12+ 0)) & 0xF;

	mclk = pll[sel] / (dvo  + 1);
	bclk = mclk     / (div2 + 1);
	pclk = bclk     / (div3 + 1);

	switch (clk) {
	case 0:	clock_hz =	pll[0];	break;
	case 1:	clock_hz =	pll[1];	break;
	case 2:	clock_hz =	fclk;	break;
	case 3:	clock_hz =	mclk;	break;
	case 4:	clock_hz =	bclk;	break;
	case 5:	clock_hz =	pclk;	break;
	default:
		printf("error: unknown clock [%d], (0~6)\n", clk);
		break;
	}
	return clock_hz;
}

void cpu_clk_info(void)
{
	unsigned int clkmode[2], pllreg[2];
	unsigned int pll[4], sel, dvo, div2, div3, fclk, mclk, bclk, pclk, sync;
	int i;

	struct NX_CLKPWR_RegisterSet *const pReg = (struct NX_CLKPWR_RegisterSet*)PHY_BASEADDR_CLKPWR_MODULE;
	
	pre_fclk = cpu_get_clock_hz(2);
	pre_mclk = cpu_get_clock_hz(3);

	printf("%s: %s Frequency Configuration : \n", "NXP2120", "DONE");

#if (0)
	printf( "\n" );
	printf( "< CLKPWR register dump >\n" );
	printf( "CLKMODE[0] = 0x%08X\n", pReg->CLKMODE[0]);
	printf( "CLKMODE[1] = 0x%08X\n", pReg->CLKMODE[1]);
	printf( "PLLSET [0] = 0x%08X\n", pReg->PLLSET[0]);
	printf( "PLLSET [1] = 0x%08X\n", pReg->PLLSET[1]);
	printf( "\n" );
#endif

	clkmode[0] 	= pReg->CLKMODEREG0;
	clkmode[1] 	= pReg->CLKMODEREG1;
	pllreg [0]	= pReg->PLLSETREG[0];
	pllreg [1]	= pReg->PLLSETREG[1];
	sync		= pReg->PWRMODE & (1<<28);

	for ( i=0 ; 2 > i ; i++ )
	{
		unsigned int 	pdiv, mdiv, sdiv;
        unsigned int     P_bitpos, M_bitpos, S_bitpos;
        unsigned int     P_mask, M_mask, S_mask;

		const unsigned int PLL0_PDIV_BIT_POS 	= 18;
		const unsigned int PLL0_MDIV_BIT_POS 	=  8;
		const unsigned int PLL0_SDIV_BIT_POS 	=  0;
        const unsigned int PLL0_PDIV_MASK       = 0xFF;
        const unsigned int PLL0_MDIV_MASK       = 0x3FF;
        const unsigned int PLL0_SDIV_MASK       = 0xFF;

		const unsigned int PLL1_PDIV_BIT_POS 	= 18;
		const unsigned int PLL1_MDIV_BIT_POS 	=  8;
		const unsigned int PLL1_SDIV_BIT_POS 	=  0;
        const unsigned int PLL1_PDIV_MASK       = 0x3F;
        const unsigned int PLL1_MDIV_MASK       = 0x3FF;
        const unsigned int PLL1_SDIV_MASK       = 0xFF;

        if ( 0 == i )
        {
            P_bitpos = PLL0_PDIV_BIT_POS;
            P_mask   = PLL0_PDIV_MASK;
            M_bitpos = PLL0_MDIV_BIT_POS;
            M_mask   = PLL0_MDIV_MASK;
            S_bitpos = PLL0_SDIV_BIT_POS;
            S_mask   = PLL0_SDIV_MASK;
        }
        else
        {
            P_bitpos = PLL1_PDIV_BIT_POS;
            P_mask   = PLL1_PDIV_MASK;
            M_bitpos = PLL1_MDIV_BIT_POS;
            M_mask   = PLL1_MDIV_MASK;
            S_bitpos = PLL1_SDIV_BIT_POS;
            S_mask   = PLL1_SDIV_MASK;
        }

		pdiv	= (pllreg[i] >> P_bitpos) & P_mask;
		mdiv	= (pllreg[i] >> M_bitpos) & M_mask;
		sdiv	= (pllreg[i] >> S_bitpos) & S_mask;

		pll[i]	= CFG_SYS_PLL_PMSFUNC( CFG_SYS_PLLFIN, pdiv, mdiv, sdiv );

		printf( "PLL%d [P = %2d, M=%2d, S=%2d] = %d Hz. \n", i, pdiv, mdiv, sdiv, pll[i] );
	}

	dvo 	= (clkmode[0]>>( 0+ 0)) & 0xF;
	sel		= (clkmode[0]>>( 4+ 0)) & 0x3;	//	NX_ASSERT( 2 > sel );
	div2	= (clkmode[0]>>( 8+ 0)) & 0xF;
	printf( "CPU : Core clock = PLL%d / %d = %d, AXI Bus Clock = Core clock / %d = %d, [%s]\n",
		sel, dvo+1, pll[sel] / (dvo+1), div2+1, (pll[sel] / (dvo+1)) / (div2+1),
		sync ? "SYNC" : "ASYNC");

	fclk = pll[sel] / (dvo+1);
	pll[3] = fclk;

	dvo	    = (clkmode[1]>>(  0+ 0)) & 0xF;
	sel		= (clkmode[1]>>(  4+ 0)) & 0x3;//	NX_ASSERT( 2 != sel );
	div2	= (clkmode[1]>>(  8+ 0)) & 0xF;
	div3	= (clkmode[1]>>( 12+ 0)) & 0xF;

	mclk = pll[sel] / (dvo  + 1);
	bclk = mclk     / (div2 + 1);
	pclk = bclk     / (div3 + 1);

	printf( "BUS : MCLK = %s / %d = %d, BCLK = MCLK / %d = %d, PCLK = BCLK / %d = %d\n",
						((sel==3) ? "FCLK" : ((sel==1) ? "PLL1" : "PLL0")),
						dvo+1, 	mclk,
                		div2+1, bclk,
                		div3+1, pclk );

	/* PLL1 Power down status */
	printf("PLL1 Power : %s \n", (clkmode[0] & (1 << 30)) ? "down" : "normal" );
	printf("FCLK: %d Hz -> %d Hz, change [%s]\n", pre_fclk, fclk, (pre_fclk == fclk) ? "NONE" : "DONE");
	printf("MCLK: %d Hz -> %d Hz, change [%s]\n", pre_mclk, mclk, (pre_mclk == mclk) ? "NONE" : "DONE");
}

/*------------------------------------------------------------------------------
 * memory clock status.
 */
#define MEMCFG			*(volatile unsigned long *)(0xc0014800)
#define MEMTIME0		*(volatile unsigned long *)(0xc0014804)
#define MEMCTRL			*(volatile unsigned long *)(0xc0014808)
#define MEMACTPWD		*(volatile unsigned long *)(0xc001480C)
#define MEMTIME1		*(volatile unsigned long *)(0xc0014810)

#define PHYDELAYCTRL	*(volatile unsigned long *)(0xc0014894)
#define PHYDLLCTRL0		*(volatile unsigned long *)(0xc0014898)

void mem_clk_info(void)
{
	unsigned int config = MEMCFG;
	unsigned int time_0 = MEMTIME0;
	unsigned int time_1 = MEMTIME0;

	int CASLAT  = (int)((config >> 21) & 0x7);
	int ADDLAT  = (int)((config >> 18) & 0x7);
	int DLYLAT  = (int)((config >> 24) & 0x7);

	int TRCD	= (int)(((time_0 >>  0) & 0x0F) + 1);
	int TRP		= (int)(((time_0 >>  4) & 0x0F) + 1);
	int TRAS	= (int)(((time_0 >> 12) & 0xFF) + 1);
	int TMRD	= (int)(((time_1 >> 16) & 0x0F) + 1);
	int TWR		= (int)(((time_1 >> 24) & 0x0F) + 1);
	int TWTR	= (int)(((time_1 >> 28) & 0x0F) + 1);
	int TRTP	= (int)(((time_1 >> 20) & 0x0F) + 1);
	int TRFC	= (int)(((time_0 >> 24) & 0xFF) + 1);

	printf("Memory Clock Configuration :\n");
	printf("CLAT =%4d, ALAT =%4d, DLAT =%4d \n", CASLAT, ADDLAT, DLYLAT);
	printf("TRCD =%4d, TRP  =%4d, TRAS =%4d, TMRD =%4d \n", TRCD, TRP , TRAS, TMRD);
	printf("TWR  =%4d, TWTR =%4d, TRTP =%4d, TRFC =%4d \n", TWR , TWTR, TRTP, TRFC);
}

void cpu_pll_info(void)
{
	cpu_clk_info();
	mem_clk_info();
	printf("\n");
}

int print_cpuinfo(void)
{
	unsigned int reg_sp, reg_pc;

	asm("mov %0, sp":"=r" (reg_sp));
	asm("mov %0, pc":"=r" (reg_pc));

	printf("U-Boot PC [0x%x], SP [0x%x] \n", reg_pc, reg_sp);

	cpu_pll_info();

	return 0;
}

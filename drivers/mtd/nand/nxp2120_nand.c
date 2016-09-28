/*
 * (C) Copyright 2006 DENX Software Engineering
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

#if defined(CONFIG_CMD_NAND)
#include <nand.h>
#include <asm/arch/regs.h>

#include <asm/io.h>
#include <asm/errno.h>

#define true 1
#define false 0
#define NXP_NAND_DEBUG		1
#define MTD_DEBUG_LEVEL		1

#if NXP_NAND_DEBUG
u32 *cfg;
#endif
	
#if NXP_NAND_DEBUG
#define DEBUG(x, args...)	do {if (x >= (*cfg)) printf(args);} while(0)
#else
#define DEBUG(x, args...)	do {}	while (0)
#endif

/* When NAND is used as boot device, below is set to 1. */
int boot_nand = 0;

/* Nand flash definition values by jsgood */
#define NXP_NAND_TYPE_UNKNOWN	0x0
#define NXP_NAND_TYPE_SLC	0x1
#define NXP_NAND_TYPE_MLC	0x2

#undef	NXP_NAND_DEBUG

#define ECC_TYPE_AUTO   0
#define ECC_TYPE_1BIT   1
#define ECC_TYPE_4BIT   2
#define ECC_TYPE_8BIT   3
#define ECC_TYPE_12BIT  4
#define ECC_TYPE_16BIT  5

/* Nand flash global values by jsgood */
int cur_ecc_mode = 0;
int nand_type = NXP_NAND_TYPE_UNKNOWN;
int addr_cycle = 4;
int ecc_type = ECC_TYPE_AUTO;

static char BASEADDR_POLY[64*1024] = { 0, };

//------------------------------------------------------------------------------
// BCH variables:
//------------------------------------------------------------------------------
//	k : number of information
//	m : dimension of Galois field.
//	t : number of error that can be corrected.
//	n : length of codeword = 2^m - 1
//	r : number of parity bit = m * t
//------------------------------------------------------------------------------
#define NX_BCH_VAR_K		(512 * 8)
#define NX_BCH_VAR_M		(13)
#define NX_BCH_VAR_T		(CFG_NAND_ECC_MODE)		// 4 or 8 or 16

#define NX_BCH_VAR_N		(((1<<NX_BCH_VAR_M)-1))
#define NX_BCH_VAR_R		(NX_BCH_VAR_M * NX_BCH_VAR_T)

#define NX_BCH_VAR_TMAX		(16)
#define NX_BCH_VAR_RMAX		(NX_BCH_VAR_M * NX_BCH_VAR_TMAX)

#define NX_BCH_VAR_R32		((NX_BCH_VAR_R   +31)/32)
#define NX_BCH_VAR_RMAX32	((NX_BCH_VAR_RMAX+31)/32)


typedef struct tag_POLYNOMIALS
{
	short BCH_AlphaToTable[8192];
	short BCH_IndexOfTable[8192];
	int elp[(NX_BCH_VAR_TMAX*2)+1][(NX_BCH_VAR_TMAX*2)*2]; 	// Error locator polynomial (ELP)
	int B  [(NX_BCH_VAR_TMAX*2)+1][(NX_BCH_VAR_TMAX*2)*2];	// Scratch polynomial
} POLYNOMIALS;

/* Nand flash oob definition for 4bit 512byte page size by S.W. Kim.
   nxp2120 generate 
      4bit for 6.5Bytes(52bit)
      8bit for 13bytes(104bit)
      16bit for 26bytes(208bit)
      24bit for 42bytes(336bit)
 
   512byte nand must leave 5th byte should be free. */

static struct nand_ecclayout nxp_nand_oob_4bit_16 = {
	.eccbytes = 7,
	.oobfree = { {2, 16-2-7}, },
	.eccpos = {
		   9, 10, 11, 12, 13, 14, 15
	}
};

/* Nand flash oob definition for 4bit 2k page size by Seungwoo Kim */
static struct nand_ecclayout nxp_nand_oob_4bit_64 = {
	.eccbytes = 28,
	.oobfree = { {2, 64 - 2- 28}, },
	.eccpos = {
	     /*32, 33, 34, 35, */
		   36, 37, 38, 39, 
		   40, 41, 42, 43, 44, 45, 46, 47, 
		   48, 49, 50, 51, 52, 53, 54, 55, 
		   56, 57, 58, 59, 
		   60, 61, 62, 63
	}
};

/* Nand flash oob definition for 4bit 4k page size by Seungwoo Kim */
static struct nand_ecclayout nxp_nand_oob_4bit_128 = {
	.eccbytes = 56,
	.oobfree = { {2, 128-2-56}, },
	.eccpos = {
		/* 64, 65, 66, 67, 68, 69, 70, 71, */
		   72, 73, 74, 75, 76, 77, 78, 79,
		   80, 81, 82, 83, 84, 85, 86, 87,
		   88, 89, 90, 91, 92, 93, 94, 95,
		   96, 97, 98, 99,100,101,102,103,
		  104,105,106,107,108,109,110,111,
		  112,113,114,115,116,117,118,119,
		  120,121,122,123,124,125,126,127
	}
};

/* Nand flash oob definition for 8bit 512byte page size by Seungwoo Kim.
   5th byte should be free. So actualy no free block for 8bit ECC of 512bytes. */
static struct nand_ecclayout nxp_nand_oob_8bit_16 = {
	.eccbytes = 13,
	.oobfree = { {0, 0}, },
	.eccpos = {
		   2, 3, 4, 6, 7, 8, 9, 10,
		   11, 12, 13, 14, 15
	}
};

/* Nand flash oob definition for 8bit 2k page size by Seungwoo Kim */
static struct nand_ecclayout nxp_nand_oob_8bit_64 = {
	.eccbytes = 52,
	.oobfree = { {2, 64-2-52}, },
	.eccpos = {
		   12, 13, 14, 15, 16, 17, 18, 19,
		   20, 21, 22, 23, 24, 25, 26, 27,
		   28, 29, 30, 31, 32, 33, 34, 35,
		   36, 37, 38, 39, 40, 41, 42, 43,
		   44, 45, 46, 47, 48, 49, 50, 51,
		   52, 53, 54, 55, 56, 57, 58, 59,
		   60, 61, 62, 63
	}
};

/* Nand flash oob definition for MLC 4k page size by Seungwoo Kim */
static struct nand_ecclayout nxp_nand_oob_8bit_128 = {
	.eccbytes = 104,
	.oobfree = { {2, 128-2-104}, },
	.eccpos = {
		   24, 25, 26, 27, 28, 29, 30, 31,
		   32, 33, 34, 35, 36, 37, 38, 39,
		   40, 41, 42, 43, 44, 45, 46, 47,
		   48, 49, 50, 51, 52, 53, 54, 55,
		   56, 57, 58, 59, 60, 61, 62, 63,
		   64, 65, 66, 67, 68, 69, 70, 71,
		   72, 73, 74, 75, 76, 77, 78, 79,
		   80, 81, 82, 83, 84, 85, 86, 87,
		   88, 89, 90, 91, 92, 93, 94, 95,
		   96, 97, 98, 99,100,101,102,103,
		  104,105,106,107,108,109,110,111,
		  112,113,114,115,116,117,118,119,
		  120,121,122,123,124,125,126,127
	}
};

#if defined(NXP_NAND_DEBUG)
/*
 * Function to print out oob buffer for debugging
 * ksw: from s3c code by jsgood.
 */
static void print_oob(const char *header, struct mtd_info *mtd)
{
	int i;
	struct nand_chip *chip = mtd->priv;

	printk("%s:\t", header);

	for(i = 0; i < 64; i++)
		printk("%02x ", chip->data_poi[i]);

	printk("\n");
}
#endif

static int nxp2120_nand_device_ready(struct mtd_info *mtdinfo);

/*!
 * This function is used by the upper layer to write command to NAND Flash for
 * different operations to be carried out on NAND Flash
 *
 * @param       mtd             MTD structure for the NAND Flash
 * @param       command         command for NAND Flash
 * @param       column          column offset for the page read
 * @param       page_addr       page to be read from NAND Flash
 */
static void nxp2120_nand_command(struct mtd_info *mtd, unsigned command,
			     int column, int page_addr)
{
	struct nand_chip *this = mtd->priv;

	DEBUG(MTD_DEBUG_LEVEL3,
	      "%s (cmd = 0x%x, col = 0x%x, page = 0x%x)\n",
	      __func__,command, column, page_addr);

	/* Check command if command is "sequence in" and the device is 512byte/sector
	   then It would be nice to setting device's pointer to 'A' area.(According to Samsung's manual)
	 */
	 if (command == NAND_CMD_SEQIN) {
	     if (this->page_shift <= 9) {
	         /* Issue Pointer Reset to 'A'(0~255) area */
	         writeb(NAND_CMD_READ0, NXP2120_NFCMD);
	     }
	 }
	
	/*
	 * Write out the command to the device.
	 */
	writeb(command, NXP2120_NFCMD);
	/*
	 * Write out column address, if necessary
	 */
	if (command == NAND_CMD_READID) {
		 writeb(0, NXP2120_NFADDR);
		 return;
	}
	if (column != -1) {
		writeb(column& 0xFF , NXP2120_NFADDR); //page_addr == -1);
		if (this->page_shift > 9) { /* over 512 byte pagesize NAND, should need second addr cycle */
			writeb((column >> 8) & 0xFF, NXP2120_NFADDR);	/* another col addr cycle for 2k page */
		}
	}
	/*
	 * Write out page address, if necessary
	 */
	if (page_addr != -1) {
		writeb((page_addr & 0xff), NXP2120_NFADDR);	/* paddr_0 - p_addr_7 */
		writeb((page_addr >> 8) & 0xFF, NXP2120_NFADDR);
		if (addr_cycle >= 4) { /* 2048 or 4096 page size */
			writeb((page_addr >> 16) & 0xFF, NXP2120_NFADDR);
		}	
	}

	/*
	 * Command post-processing step
	 */
	switch (command) {

	case NAND_CMD_RESET:
		break;

	case NAND_CMD_READOOB:
	case NAND_CMD_READ0:
		if (this->page_shift > 9) {
			/* send read confirm command */
			writeb(NAND_CMD_READSTART, NXP2120_NFCMD);
		}
		break;
	case NAND_CMD_RNDOUT:
	    if (this->page_shift > 9) {
			/* send read confirm command */
			writeb(NAND_CMD_RNDOUTSTART, NXP2120_NFCMD);
		}
	    break;

	case NAND_CMD_READID:
		break;

	case NAND_CMD_PAGEPROG:
		break;

	case NAND_CMD_STATUS:
		break;

	case NAND_CMD_ERASE2:
		break;
	}
	//printk("cmd=%x\n", command);
	if (NAND_CMD_STATUS != command) {
	    while (!nxp2120_nand_device_ready(mtd)) ;
	} else {
	    /* We got some delay for STATUS? */
	}
}

/*!
 * This function is used by upper layer for select and deselect of the NAND
 * chip
 *
 * @param       mtd     MTD structure for the NAND Flash
 * @param       chip    val indicating select or deselect
 */
static void nxp2120_nand_select_chip(struct mtd_info *mtd, int chip)
{
	if (chip > 0) {
		DEBUG(MTD_DEBUG_LEVEL0,
		      "ERROR:  Illegal chip select (chip = %d)\n", chip);
		return;
	}

	switch (chip) {
	case -1:
		/* Disable the NFC clock */
		NAND_DISABLE_CE();
		break;
	case 0:
		/* Enable the chip 0 */
		NAND_ENABLE_CE();
		break;

	default:
		break;
	}
}
/*
 * Function for checking device ready pin
 * Written by jsgood
 */
static int nxp2120_nand_device_ready(struct mtd_info *mtdinfo)
{
    unsigned int ux=20000;
    unsigned int reg;

    while (ux--) {
        if ((reg = readl(NXP2120_NFCONTROL)) & NXP2120_NFCONT_IRQPEND) {
            writel(reg | NXP2120_NFCONT_IRQCLEAR, NXP2120_NFCONTROL);
           /* printk("%s ready = %d\n",__func__, ux);*/
            return 1;
        }
        if (ux < 19801){//19930) {
            //printk("reg=%x %d\n", reg, ux);
            if (0 == (reg & NXP2120_NFCONT_RNB))
                continue;
            break;
        }
    }
    //printk("not ready\n");
	return 1; /* Fail to get ready but on going ready*/
}

/*
 * We don't use bad block table
 */
static int nxp2120_nand_scan_bbt(struct mtd_info *mtdinfo)
{
	return nand_default_bbt(mtdinfo);
}

static void nxp2120_nand_write_buf(struct mtd_info *mtd, const u_char *buf, int len)
{
    int i;
    u32 *bb;
    int len4;

    len4 = len / 4;
    len = len % 4;
  
    bb = (u32 *)buf;
    for (i=0; i<len4; i++, bb++) {
        writel(*bb, NXP2120_NFDATA);
    }
    buf = (u_char *) bb;
    while (len) {
        writeb(*buf, NXP2120_NFDATA);
        buf++;
        len--;
    }
}

static void nxp2120_nand_read_buf(struct mtd_info *mtd, u_char *buf, int len)
{
    int i;
    u32 *bb;
    int len4, t;

    len4 = len / 4;
    len = len % 4;
  
    //printf("buf addr = %x len4=%x len-%x\n", buf, len4, len);
    bb = (u32 *)buf;
    for (i=0; i<len4; i++, bb++) {
        *bb = readl(NXP2120_NFDATA);
    }
    buf = (u_char *) bb;
    while (len) {
        *buf = readb(NXP2120_NFDATA);
        buf++;
        len--;
    }
}

#if defined(CFG_NAND_HWECC)

/*
 * Function for checking ENCDONE in NFECCSTATUS
 * Written by ksw
 */
static void nxp2120_nand_wait_enc(void)
{
	while (!(readl(NXP2120_NFECCSTATUS) & NXP2120_NFECCSTATUS_NFECCENCDONE)) {}
}

/*
 * Function for checking DECDONE in NFECCSTATUS
 * Written by ksw
 */
static void nxp2120_nand_wait_dec(void)
{
	while (!(readl(NXP2120_NFECCSTATUS) & NXP2120_NFECCSTATUS_NFECCDECDONE)) {}
}

/*
 * ksw : These code from NXP's u-boot
 *
 */
//------------------------------------------------------------------------------
int	nxp2120_GetErrorStatus(void)
{
	if (readl(NXP2120_NFECCSTATUS) & NXP2120_NFECCSTATUS_NFCHECKERROR)
		return 1;
	return 0;
}

//------------------------------------------------------------------------------
// Generate GF(2**NX_BCH_VAR_M) from the primitive polynomial p(X) in p[0]..p[NX_BCH_VAR_M]
// The lookup table looks like:
// index -> polynomial form:   pAlphaTo[ ] contains j = alpha**i;
// polynomial form -> index form:  pIndexOf[j = alpha**i] = i
// pAlphaTo[1] = 2 is the primitive element of GF(2**NX_BCH_VAR_M)
//------------------------------------------------------------------------------
void nxp2120_CreateLookupTable(void)
{
	int i;
	int mask;	// Register states
	unsigned int p = 0x25AF;	// Primitive polynomials

	POLYNOMIALS *pPoly = (POLYNOMIALS *)BASEADDR_POLY;
	short *   pAlphaTo = &(pPoly->BCH_AlphaToTable[0]);
	short *   pIndexOf = &(pPoly->BCH_IndexOfTable[0]);

	// Galois field implementation with shift registers
	// Ref: L&C, Chapter 6.7, pp. 217
	mask = 1;
	pAlphaTo[NX_BCH_VAR_M] = 0;
	for ( i=0 ; i<NX_BCH_VAR_M ; i++ )	{
		pAlphaTo[ i ] = mask;
		pIndexOf[ pAlphaTo[i] ] = i;

		if ( p & (1U<<i) )
			pAlphaTo[NX_BCH_VAR_M] ^= mask;

		mask <<= 1 ;
	}

	pIndexOf[ pAlphaTo[NX_BCH_VAR_M] ] = NX_BCH_VAR_M;
	mask >>= 1;
	for ( i=NX_BCH_VAR_M+1 ; i<NX_BCH_VAR_N ; i++ ) {
		if ( pAlphaTo[i-1] >= mask )
			pAlphaTo[i] = pAlphaTo[NX_BCH_VAR_M] ^ ((pAlphaTo[i-1] ^ mask) << 1);
		else
			pAlphaTo[i] = pAlphaTo[i-1] << 1;

		pIndexOf[pAlphaTo[i]] = i;
	}
	pIndexOf[0] = -1;
}

//------------------------------------------------------------------------------
#define NX_BCH_AlphaTo( _i_ )		((int)(pPoly->BCH_AlphaToTable[ (_i_) ]))
#define NX_BCH_IndexOf( _i_ )		((int)(pPoly->BCH_IndexOfTable[ (_i_) ]))

static int	NX_BCH_MODULAR(int index)
{
	register int modular = NX_BCH_VAR_N;

	while ( index >= modular )
		index -= modular;

	return index;
}
//------------------------------------------------------------------------------
int	nxp2120_GetErrorLocation(int *pOddSyn, int *pLocation, int *ErrCnt)
{
	register int i, j, elp_sum ;
	int count;
	int r;				// Iteration steps
	int Delta; 			// Discrepancy value

//	int elp[(NX_BCH_VAR_TMAX*2)+1][(NX_BCH_VAR_TMAX*2)+2]; 	// Error locator polynomial (ELP)
//	int B  [(NX_BCH_VAR_TMAX*2)+1][(NX_BCH_VAR_TMAX*2)+2];	// Scratch polynomial
	POLYNOMIALS *pPoly = (POLYNOMIALS *)BASEADDR_POLY;

	int L[(NX_BCH_VAR_TMAX*2)+1];		// Degree of ELP
	int reg[(NX_BCH_VAR_TMAX*1)+1];	// Register state
	int	s[(NX_BCH_VAR_TMAX*2)];

	for( i=0 ; i<NX_BCH_VAR_T ; i++ )
		s[i*2] = pOddSyn[i];

	// Even syndrome = (Odd syndrome) ** 2
	for( i=1,j=0 ; i<(NX_BCH_VAR_T*2) ; i+=2, j++ )
	{
		if( s[j] == 0 )		s[i] = 0;
		else				s[i] = NX_BCH_AlphaTo( NX_BCH_MODULAR( 2 * NX_BCH_IndexOf(s[j]) ) );
	}

	// Initialization of pPoly->elp, pPoly->B and register
	for( i=0 ; i<=(NX_BCH_VAR_T*2) ; i++ )
	{
		L[i] = 0 ;
		for( j=0 ; j<=(NX_BCH_VAR_T*2) ; j++ )
		{
			pPoly->elp[i][j] = 0 ;
			pPoly->B[i][j] = 0 ;
		}
	}

	for( i=0 ; i<=NX_BCH_VAR_T ; i++ )
	{
		reg[i] = 0 ;
	}

	pPoly->elp[1][0] = 1 ;
	pPoly->elp[1][1] = s[0] ;

	L[1] = 1 ;
	if( s[0] != 0 )
		pPoly->B[1][0] = NX_BCH_AlphaTo( NX_BCH_MODULAR( NX_BCH_VAR_N - NX_BCH_IndexOf(s[0]) ) );
	else
		pPoly->B[1][0] = 0;

	for( r=3 ; r<=(NX_BCH_VAR_T*2)-1 ; r=r+2 )
	{
		// Compute discrepancy
		Delta = s[r-1] ;
		for( i=1 ; i<=L[r-2] ; i++ )
		{
			if( (s[r-i-1] != 0) && (pPoly->elp[r-2][i] != 0) )
				Delta ^= NX_BCH_AlphaTo( NX_BCH_MODULAR( NX_BCH_IndexOf(s[r-i-1]) + NX_BCH_IndexOf(pPoly->elp[r-2][i]) ) );
		}

		if( Delta == 0 )
		{
			L[r] = L[r-2] ;
			for( i=0 ; i<=L[r-2] ; i++ )
			{
				pPoly->elp[r][i] = pPoly->elp[r-2][i];
				pPoly->B[r][i+2] = pPoly->B[r-2][i] ;
			}
		}
		else
		{
			// Form new error locator polynomial
			for( i=0 ; i<=L[r-2] ; i++ )
			{
				pPoly->elp[r][i] = pPoly->elp[r-2][i] ;
			}

			for( i=0 ; i<=L[r-2] ; i++ )
			{
				if( pPoly->B[r-2][i] != 0 )
					pPoly->elp[r][i+2] ^= NX_BCH_AlphaTo( NX_BCH_MODULAR( NX_BCH_IndexOf(Delta) + NX_BCH_IndexOf(pPoly->B[r-2][i]) ) );
			}

			// Form new scratch polynomial and register length
			if( 2 * L[r-2] >= r )
			{
				L[r] = L[r-2] ;
				for( i=0 ; i<=L[r-2] ; i++ )
				{
					pPoly->B[r][i+2] = pPoly->B[r-2][i];
				}
			}
			else
			{
				L[r] = r - L[r-2];
				for( i=0 ; i<=L[r-2] ; i++ )
				{
					if( pPoly->elp[r-2][i] != 0 )
						pPoly->B[r][i] = NX_BCH_AlphaTo( NX_BCH_MODULAR( NX_BCH_IndexOf(pPoly->elp[r-2][i]) + NX_BCH_VAR_N - NX_BCH_IndexOf(Delta) ) );
					else
						pPoly->B[r][i] = 0;
				}
			}
		}
	}

	if( L[(NX_BCH_VAR_T*2)-1] > NX_BCH_VAR_T )
	{
		if(ErrCnt)
			*ErrCnt = L[(NX_BCH_VAR_T*2)-1];
		return -1;
	}
	else
	{
		// Chien's search to find roots of the error location polynomial
		// Ref: L&C pp.216, Fig.6.1
		for( i=1 ; i<=L[(NX_BCH_VAR_T*2)-1] ; i++ )
			reg[i] = pPoly->elp[(NX_BCH_VAR_T*2)-1][i];

		count = 0;
		for( i=1 ; i<=NX_BCH_VAR_N ; i++ )
		{
			elp_sum = 1;
			for( j=1 ; j<=L[(NX_BCH_VAR_T*2)-1] ; j++ )
			{
				if( reg[j] != 0 )
				{
					reg[j] = NX_BCH_AlphaTo( NX_BCH_MODULAR( NX_BCH_IndexOf(reg[j]) + j ) );
					elp_sum ^= reg[j] ;
				}
			}

			if( !elp_sum )		// store root and error location number indices
			{
				// Convert error location from systematic form to storage form
				pLocation[count] = NX_BCH_VAR_N - i;

				if (pLocation[count] >= NX_BCH_VAR_R)
				{
					// Data Bit Error
					pLocation[count] = pLocation[count] - NX_BCH_VAR_R;
					pLocation[count] = (NX_BCH_VAR_K-1) - pLocation[count];
				}
				else
				{
					// ECC Error
					pLocation[count] = pLocation[count] + NX_BCH_VAR_K;
				}

				if( pLocation[count] < 0 ) {
					if(ErrCnt)
						*ErrCnt = L[(NX_BCH_VAR_T*2)-1];
					return -1;
				}
				//if( pLocation[count] >= 0 )
				count++;
			}
		}

		if(ErrCnt)
			*ErrCnt = L[(NX_BCH_VAR_T*2)-1];

		if( count == L[(NX_BCH_VAR_T*2)-1] )	// Number of roots = degree of pPoly->elp hence <= NX_BCH_VAR_T errors
		{
			return 0;
		}
		else	// Number of roots != degree of ELP => >NX_BCH_VAR_T errors and cannot solve
		{
			return -1;
		}

		/*
		if( count != L[(NX_BCH_VAR_T*2)-1] )
		{
			NX_DEBUG_MSG( "\n\n\t\t ERROR -> count = " );
			NX_DEBUG_DEC( count );
			NX_DEBUG_MSG( ", L = " );
			NX_DEBUG_DEC( L[(NX_BCH_VAR_T*2)-1] );
			NX_DEBUG_MSG( "\n\n" );

			if( count < 4 )		return -1;
		}

		return count;
		*/
	}
}
//------------------------------------------------------------------------------
void nxp2120_SetResetECC(int EccMode)
{
	const U32 BIT_SIZE	= 2;
	const U32 BIT_POS	= 28;
	const U32 BIT_MASK	= ((1<<BIT_SIZE)-1) << BIT_POS;

	register U32 regval;

	EccMode /= 8;	// NFECCMODE[1:0] = 0(4), 1(8), 2(16)

	regval  = __REG(NXP2120_NFCONTROL);
	regval &= ~(BIT_MASK);	// Unmask bits.
	regval |= (EccMode << BIT_POS);

	// Reset H/W BCH decoder.
	__REG(NXP2120_NFCONTROL) = regval | NXP2120_NFCONT_ECCRST;
}

//------------------------------------------------------------------------------
void nxp2120_GetGenECC(unsigned int *pECC, int EccMode)
{
	int i, num;
	volatile U32 *pRegECC = (volatile U32 *)NXP2120_NFECC0;

	switch (EccMode) {
	case  4: num = 2;	break;
	case  8: num = 4;	break;
	case 16: num = 7;	break;
	case 24:
	default:
		printf("not support ECC %d bit\n", EccMode);
		return;
	}

	for (i=0 ; i<num ; i++, pECC++) {
		*pECC = *pRegECC++;
		printf("pECC[%d]=%x\n", i, *pECC);
	}
}

void nxp2120_SetOriECC(unsigned int *pECC, int EccMode)
{
	int i, num;
	volatile U32 *pRegOrgECC = (volatile U32 *)NXP2120_NFORGECC0;

	switch (EccMode) {
	case  4: num = 2;	break;
	case  8: num = 4;	break;
	case 16: num = 7;	break;
	case 24:
	default:
		printf("not support ECC %d bit\n", EccMode);
		return;
	}

	for (i=0 ; num > i; i++) {
	    printf("orgecc[%d]=%x\n", i, *pECC);
		*pRegOrgECC++ = *pECC++;
	}
}

//------------------------------------------------------------------------------
void nxp2120_GetOddSyndrome(int *pSyndrome)
{
	const U32 BIT_SIZE	= 13;
	const U32 BIT_POS	= 13;
	const U32 BIT_MASK	= ((1UL<<BIT_SIZE)-1);

	register volatile U32 *pReg;
	register U32 regval;
	int i;

	//NX_ASSERT( NULL != pSyndrome );

	pReg = (volatile U32 *)NXP2120_NFSYNDROME0;

	for ( i=0 ; i<(NX_BCH_VAR_T/2) ; i++ ) {
		regval = *pReg++;
		*pSyndrome++ = (int)(regval & BIT_MASK);		// Syndrome <= NFSYNDROME[i][12: 0]
		*pSyndrome++ = (int)(regval >> BIT_POS);		// Syndrome <= NFSYNDROME[i][25:13]
	}
}


/*
 * This function is called before encoding ecc codes to ready ecc engine.
 * Written by jsgood/Seungwoo Kim
 */
static void nxp2120_nand_enable_hwecc(struct mtd_info *mtd, int mode)
{
	cur_ecc_mode = mode;
	
	nxp2120_SetResetECC(CFG_NAND_ECC_MODE);
}

/*
 * This function is called immediately after encoding ecc codes.
 * This function returns encoded ecc codes.
 * Written by jsgood/Seungwoo Kim
 */
static int nxp2120_nand_calculate_ecc(struct mtd_info *mtd, const u_char *dat, u_char *ecc_code)
{
    if (cur_ecc_mode == NAND_ECC_READ) {
        if (dat == NULL)
            nxp2120_SetOriECC((unsigned int *)ecc_code, CFG_NAND_ECC_MODE);
    } else {
        nxp2120_nand_wait_enc();
        nxp2120_GetGenECC((unsigned int *)ecc_code, CFG_NAND_ECC_MODE);
    }
    
    return 0;
}

static int nxp2120_CheckEmptyPage(struct mtd_info *mtd, unsigned char *read_ecc)
{
    int n, num;
    int lastmask = 0xFFFFFFFF;

    /* Temporary... All ECC values are 0xff, it would be empty page */
    switch (CFG_NAND_ECC_MODE) {
	case  4: num = 7;	lastmask = 0x00FFFFF; break;
	case  8: num = 13;	break;
	case 16: num = 28;	break;
	case 24:
	default:
		printf("not support ECC %d bit\n", CFG_NAND_ECC_MODE);
		return 1;
	}
    //printf("%s : enter nxp2120_CheckEmptyPage num=%d\n", __func__, num);
    for (n=0; num > n; n++) {
        //printf("read_ecc[%d]=%x\n", n, read_ecc[n]);
       	if (read_ecc[n] != 0xFF) return 0; /* Not empty */
    }
    return 1; /* Empty */
}
/*
 * This function determines whether read data is good or not.
 * If SLC, must write ecc codes to controller before reading status bit.
 * If MLC, status bit is already set, so only reading is needed.
 * If status bit is good, return 0.
 * If correctable errors occured, do that.
 * If uncorrectable errors occured, return -1.
 * Written by jsgood
 */
static int nxp2120_nand_correct_data(struct mtd_info *mtd, u_char *dat, u_char *read_ecc, u_char *calc_ecc)
{
	int ret = 0;
	int	o_syn[CFG_NAND_ECC_MODE], e_pos[CFG_NAND_ECC_MODE];
	int err = 0;
	int n;
	unsigned int *e_dat;
	u_char  *p = dat;

	/* Wait decoding complete */
	nxp2120_nand_wait_dec();

	/* Check Error status first */
	if (nxp2120_GetErrorStatus()) {
	    /* Check this is empry page, then we don't have to correct this */
	    if (nxp2120_CheckEmptyPage(mtd, read_ecc))
	        return 0;

	    nxp2120_GetOddSyndrome(&o_syn[0]);

	    ret = nxp2120_GetErrorLocation(&o_syn[0], &e_pos[0], &err);

		if (0 > ret) {
			
			printf("nand ecc detect errors, can't correct (err:%d) mtd->erasesize=%d\n", err, mtd->erasesize);
			for (n=0; n < 7; n++)
			{
				printf("%02X, ", read_ecc[n]);
			}
			printf("\n");
			

			ret = -EIO;
			return ret; /* temporarily */
		} else {
			DEBUG(MTD_DEBUG_LEVEL1,"nand ecc %2d err corrected, bit: ", err);
			for (n=0 ; err > n; n++) {
				printf("%d", e_pos[n]);
				if (n != err-1)
					printf(", ");

				if (4096 > e_pos[n]) {
					e_dat = (uint32_t*)p;
					e_dat[e_pos[n] / 32] ^= 1U<<(e_pos[n] % 32);
				}
			}
			ret = err > CFG_NAND_ECC_LIMIT ? -EBADMSG: 0;
			printf("\n");
		}
	}

	return ret;
}

static int nxp2120_nand_read_page_hwecc_oob_first(struct mtd_info *mtd,
	struct nand_chip *chip, uint8_t *buf, int oob_required, int page)
{
	int i, eccsize = chip->ecc.size;
	int eccbytes = chip->ecc.bytes;
	int eccsteps = chip->ecc.steps;
	uint8_t *p = buf;
	uint8_t *ecc_code = chip->buffers->ecccode;
	uint32_t *eccpos = chip->ecc.layout->eccpos;
	uint8_t *ecc_calc = chip->buffers->ecccalc;
	unsigned int max_bitflips = 0;

	/* Read the OOB area first */
	DEBUG(MTD_DEBUG_LEVEL3,"nand read page %d\n", page);
	
	if (chip->page_shift > 9)
	{
		chip->cmdfunc (mtd, NAND_CMD_READ0, 1 << chip->page_shift, page);
	}
	else 
	{
		chip->cmdfunc (mtd, NAND_CMD_READOOB, 0, page);
	}
				
/*	chip->cmdfunc(mtd, NAND_CMD_READOOB, 0, page);*/
	chip->read_buf(mtd, chip->oob_poi, mtd->oobsize);
	
	/* delay??*/
	
	
	chip->cmdfunc(mtd, NAND_CMD_READ0, 0, page);

	for (i = 0; i < chip->ecc.total; i++)
		ecc_code[i] = chip->oob_poi[eccpos[i]];

	for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {
		int stat;

		chip->ecc.hwctl(mtd, NAND_ECC_READ);
		
		/* write_orgecc   */
		{
			int j;
			u_char eccstep_code[(eccbytes+1)/2];

			for (j = 0; j < eccbytes; j++)
				eccstep_code[j] = ecc_code[i+j];
			for ( ; j < (eccbytes+1)/2 ; j++)
				eccstep_code[j] = 0;

			chip->ecc.calculate(mtd, NULL, eccstep_code);
		}
		chip->read_buf(mtd, p, eccsize);
		stat = chip->ecc.correct(mtd, p, &ecc_code[i], NULL);
		if (stat < 0) {
			mtd->ecc_stats.failed++;
		} else {
			mtd->ecc_stats.corrected += stat;
			max_bitflips = max_t(unsigned int, max_bitflips, stat);
		}
	}
	return max_bitflips;
}

#endif

static int nxp2120_nand_read_oob(struct mtd_info *mtd, struct nand_chip *chip,
			     int page)
{
	if (chip->page_shift > 9) {
		chip->cmdfunc (mtd, NAND_CMD_READ0, 1 << chip->page_shift, page);
	} else {
		chip->cmdfunc (mtd, NAND_CMD_READOOB, 0, page);
	}

	chip->read_buf(mtd, chip->oob_poi, mtd->oobsize);
	
	return 0;
}
/*
 * Board-specific NAND initialization. The following members of the
 * argument are board-specific (per include/linux/mtd/nand.h):
 * - IO_ADDR_R?: address to read the 8 I/O lines of the flash device
 * - IO_ADDR_W?: address to write the 8 I/O lines of the flash device
 * - hwcontrol: hardwarespecific function for accesing control-lines
 * - dev_ready: hardwarespecific function for  accesing device ready/busy line
 * - enable_hwecc?: function to enable (reset)  hardware ecc generator. Must
 *   only be provided if a hardware ECC is available
 * - eccmode: mode of ecc, see defines
 * - chip_delay: chip dependent delay for transfering data from array to
 *   read regs (tR)
 * - options: various chip options. They can partly be set to inform
 *   nand_scan about special functionality. See the defines for further
 *   explanation
 * Members with a "?" were not set in the merged testing-NAND branch,
 * so they are not set here either.
 */
int nxp2120_nand_init(struct nand_chip *nand, int ecc_type_override)
{
#if defined(CFG_NAND_HWECC)
	int i;
	u_char tmp, mfg, devid;
	struct nand_flash_dev *type = NULL;
#endif
	int cellinfo;
	int sizeb;
	int plane, plane_size, nand_size,erase_size;
	int page_size;
	struct	NX_MCUS_RegisterSet *nand_reg = (struct NX_MCUS_RegisterSet *)PHY_BASEADDR_MCUS_MODULE;
	cfg = (unsigned int*)0x80001000;
	*cfg = 5;
	
	nand_reg->NFTACS = NAND_tACS;
	nand_reg->NFTCOS = NAND_tCOS;    ///< F0h			: Nand Timing for tCOS Register
	nand_reg->NFTACCL = NAND_tACC;	 ///< F4h			: Nand Timing for tACC Register
	nand_reg->NFTACCH = NAND_tSACC;	 ///< F8h			: Nand Timing for tACC Register
	nand_reg->NFTOCH = NAND_tOCH;   ///< FCh			: Nand Timing for tOCH Register
	nand_reg->NFTCAH = NAND_tCAH;   ///

	ecc_type = ecc_type_override;

	// Some Register initialization needed.
	nand->IO_ADDR_R		= (void __iomem *)(NXP2120_NFDATA);
	nand->IO_ADDR_W		= (void __iomem *)(NXP2120_NFDATA);
	//nand->hwcontrol		= NULL;
	nand->dev_ready		= nxp2120_nand_device_ready;
	nand->scan_bbt		= nxp2120_nand_scan_bbt;
	nand->cmdfunc           = nxp2120_nand_command;
	nand->options		= 0;
	nand->select_chip       = nxp2120_nand_select_chip;
//	nand->write_buf         = nxp2120_nand_write_buf;
	nand->read_buf         = nxp2120_nand_read_buf;

#if defined(CONFIG_NAND_FLASH_BBT)
		nand->options 		|= NAND_USE_FLASH_BBT;
#else
/*		nand->options		|= NAND_SKIP_BBTSCAN;
		nand->block_bad		= 0;*/
#endif

#if defined(CFG_NAND_HWECC)
	nxp2120_CreateLookupTable();

	nand->ecc.mode		= NAND_ECC_HW_OOB_FIRST;
	nand->ecc.hwctl	= nxp2120_nand_enable_hwecc;
	nand->ecc.calculate	= nxp2120_nand_calculate_ecc;
	nand->ecc.correct	= nxp2120_nand_correct_data;
	nand->ecc.read_page = nxp2120_nand_read_page_hwecc_oob_first;
	/*nand->ecc.write_page = nxp2120_nand_write_page_hwecc_oob_first;*/
	nand->ecc.read_oob = nxp2120_nand_read_oob;
	/*nand->ecc.write_oob = nxp2120_nand_write_oob;*/
	
	
	//nand->options |= NAND_HWECC_READECC_FIRST | NAND_HWECC_WRITE_ORGECCCODE;

	NAND_ENABLE_CE();
	
	nxp2120_nand_command(0, NAND_CMD_READID, 0x00, -1);
	nxp2120_nand_device_ready(0);

	mfg = readb(nand->IO_ADDR_R); /* Maf. ID */
	tmp = readb(nand->IO_ADDR_R); /* Device ID */
	devid = tmp;
	
	for (i = 0; nand_flash_ids[i].name != NULL; i++) {
		printf("id:%x \n",nand_flash_ids[i].dev_id &0xFF);
		if (tmp == (nand_flash_ids[i].dev_id & 0xFF)) {
			type = &nand_flash_ids[i];
			printf("ok\n");
			break;
		}
	}
	cellinfo = readb(nand->IO_ADDR_R);	/* 3rd byte */
	tmp = readb(nand->IO_ADDR_R);		/* 4th byte */
	/*sizeb = readb(nand->IO_ADDR_R);*/		/* 5th byte */

	NAND_DISABLE_CE();
	printf("mfg:%x devid:%x cellinfo %x tmp %x sizeb %x type->pagesize %d\n",mfg,devid,cellinfo,tmp,sizeb,type->pagesize);
	printf("pagesize %d chipsize %d erasesize %d\n", type->pagesize, type->chipsize,type->erasesize);
	if (!type->pagesize) {
		if (((cellinfo >> 2) & 0x3) == 0) {
			nand_type = NXP_NAND_TYPE_SLC;
			nand->ecc.size = 512;
			nand->ecc.bytes	= 7;
			
			if ((1024 << (tmp & 0x3)) > 512) {
				//nand->ecc.read_page = s3c_nand_read_page_1bit;
				//nand->ecc.write_page = s3c_nand_write_page_1bit;
				//nand->ecc.read_oob = s3c_nand_read_oob_1bit;
				//nand->ecc.write_oob = s3c_nand_write_oob_1bit;
				nand->ecclayout = &nxp_nand_oob_4bit_64;
				nand->ecc.steps = 4;
			} else {				
				nand->ecclayout = &nxp_nand_oob_4bit_16;
				nand->ecc.steps = 1;
			}
		} else {
			nand_type = NXP_NAND_TYPE_MLC;
			nand->options |= NAND_NO_SUBPAGE_WRITE;	/* NOP = 1 if MLC */
			//nand->ecc.read_page = s3c_nand_read_page_4bit;
			//nand->ecc.write_page = s3c_nand_write_page_4bit;
			nand->ecc.size = 512;
			nand->ecc.bytes = 7;
			nand->ecc.mode = NAND_ECC_HW_OOB_FIRST;
			nand->ecclayout = &nxp_nand_oob_4bit_64;
			//nand->options &= ~NAND_HWECC_SPAREAREA;
		}
		/* calculate total size */
		//plane = 1 << (((sizeb >> 2) & 0x03) + 1);
		//plane_size = 8 << ((sizeb >> 4) & 0x07);
		
		//nand_size = plane * plane_size;
		nand_size = type->chipsize;
		erase_size = (1 << (((tmp >> 4)&3)))*65536;
		printf("erase size=%d\n", erase_size);
		//printf("plane, planesize, nand_size = %d, %x, %x\n", plane, plane_size, nand_size);
		switch (tmp & 0x03) {
		case 0 : // 1KB page.
			addr_cycle = 3;
			if (nand_size > 64)
				addr_cycle = 4;
			nand->ecc.steps = 2;
			page_size = 1024;
			break;
		case 1 : // 2KB page
			// most large NAND has 2KB page, and if we have
			addr_cycle = 3;
			if (nand_size > 128) /* more than 128MByte we need one more addr cycle. ==> 256M >= size */
				addr_cycle = 4;
			nand->ecc.steps = 4;
			page_size = 2048;
			break;
		case 2 : // 4KB page
			addr_cycle = 3;
			if (nand_size > 256) /* more than 256MByte we need one more addr cycle. ==> 512M >= size */
				addr_cycle = 4;
			nand->ecc.steps = 8;
			page_size = 4096;
			break;
		case 3 : // 8KB page
			addr_cycle = 3;
			if (nand_size > 512) /* more than 1GByte */
				addr_cycle = 4;
			nand->ecc.steps = 16;
			page_size = 8192;
			break;
		}
		type->pagesize=page_size;
		type->erasesize=erase_size;
	} else {
	    page_size = type->pagesize;
		nand_type = NXP_NAND_TYPE_SLC;
		nand->ecc.size = 512;
		//nand->cellinfo = 0; //ksw we may don't need cellinfo anymore
		nand->ecc.bytes = 7;
		nand->ecclayout = &nxp_nand_oob_4bit_16;
		addr_cycle = 3;
		if (type->chipsize >= 0x40) {
			addr_cycle = 4;
		}
		nand->ecc.steps = type->pagesize / nand->ecc.size;
	}
	printf("addr cycle:%d page size :%d ecc steps:%d nand type :%x\n", addr_cycle, page_size, nand->ecc.steps, nand_type);
#if CONFIG_ECCTYPE_MANUAL
	if (ecc_type != ECC_TYPE_AUTO) {
	    switch (ecc_type) {
#if 0
	      case ECC_TYPE_1BIT:
			nand->ecc.bytes = 4;
			if (page_size == 512) {
				nand->ecc.layout = &nxp_nand_oob_16;
			} if (page_size == 2048) {
				nand->ecc.layout = &nxp_nand_oob_64;
			} else {
				nand->ecc.layout = &nxp_nand_oob_64;
			}
			break;
#endif
	      case ECC_TYPE_4BIT:
			nand->ecc.bytes = 7;
			nand->ecc.strength = 4;
			//nand->options &= ~NAND_HWECC_SPAREAREA; /* disable SPAREAREA ECC for large eccarea */
			if (page_size == 2048) {
				nand->ecc.layout = &nxp_nand_oob_4bit_64;
			} if (page_size == 4096) {
				nand->ecc.layout = &nxp_nand_oob_4bit_128;
			} else {
				nand->ecc.layout = &nxp_nand_oob_4bit_64;
			}

			break;
#if defined(CONFIG_NAND_BL1_8BIT_ECC)
		  case ECC_TYPE_8BIT:
			nand->ecc.bytes = 13;
			nand->options |= NAND_HWECC_WRITE_ECCCODE;
			nand->options &= ~NAND_HWECC_SPAREAREA; /* disable SPAREAREA ECC for large eccarea */
			if (page_size == 2048) {
				nand->ecc.layout = &nxp_nand_oob_64_8bit;
			} if (page_size == 4096) {
				nand->ecc.layout = &nxp_nand_oob_128_8bit;
			} else {
				nand->ecc.layout = &nxp_nand_oob_64_8bit;
			}
			break;
		  case ECC_TYPE_12BIT: 
		  case ECC_TYPE_16BIT:
		    nand->options |= NAND_HWECC_WRITE_ECCCODE;
		    nand->options &= ~NAND_HWECC_SPAREAREA;
			/* don't supported yet... */
			break;
#endif
		default:
			;
		}
	}
#endif

#else
	NAND_ENABLE_CE();
	
	nxp2120_nand_command(0, NAND_CMD_READID, 0x00, -1);
	nxp2120_nand_device_ready(0);

	NAND_DISABLE_CE();
	
	
	
	nand->ecc.mode = NAND_ECC_SOFT;
	nand->ecc.read_oob = nxp2120_nand_read_oob;
#endif
        return 0;
}
#endif /* (CONFIG_COMMANDS & CFG_CMD_NAND) */

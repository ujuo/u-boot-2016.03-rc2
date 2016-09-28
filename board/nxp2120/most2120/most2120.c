/*
 * Copyright(c) 2011 STcube Inc.,
 * Seungwoo Kim <ksw@stcube.com>
 *
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Marius Groeger <mgroeger@sysgo.de>
 *
 * (C) Copyright 2002
 * David Mueller, ELSOFT AG, <d.mueller@elsoft.ch>
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
#include <asm/arch/regs.h>
#include <asm/arch/regs-iis.h>
#if defined(CONFIG_BOOTSOUND)
#include "DDING.h"
#endif

#define PLL_CALC    0

struct	NX_PWM_RegisterSet {
		struct {
			volatile U16	PWM_PREPOL;			///< 0x00		: Prescaler Register
			volatile U16	PWM_DUTY[2];		///< 0x02, 0x04 : Duty Cycle Register
			volatile U16	PWM_PERIOD[2];		///< 0x06, 0x08 : Period Cycle Register
			volatile U16	RESERVED0[3];		///< 0xA, 0xC, 0xE : Reserved Region
		} PWM2[2];

		volatile U32		RESERVED1[0x08];	///< 0x20~0x3C	: Reserved Region
		volatile U32		PWM_CLKENB;			///< 0x40		: Clock Enable Register
		volatile U32		PWM_CLKGEN;			///< 0x44		: Clock Generater Register
};

static inline void delay(unsigned long loops)
{
	__asm__ volatile ("1:\n" "subs %0, %1, #1\n" "bne 1b":"=r" (loops):"0"(loops));
}

void enable_sndclk(void)
{
    struct	NX_AUDIO_RegisterSet *i2s_reg = (struct NX_AUDIO_RegisterSet *)PHY_BASEADDR_AUDIO_MODULE;

    //udelay(100000);

    /* Enable CLKAUDIO for I2S... */
    i2s_reg->CLKENB = 0x0C; /* PCKALWAYS & enable CLK */
	i2s_reg->CLKGEN[0][0] = 4 | ((16-1) << 5); /* CLKSOURCE PLL1 and 1/16 */
	i2s_reg->I2S_CONFIG = 1; /* Set slave mode */
}

int board_init(void)
{
	DECLARE_GLOBAL_DATA_PTR;

	struct NX_GPIO_RegisterSet *gpioa = (struct NX_GPIO_RegisterSet*)PHY_BASEADDR_GPIO_A_MODULE;
	struct NX_GPIO_RegisterSet *gpiob = (struct NX_GPIO_RegisterSet*)PHY_BASEADDR_GPIO_B_MODULE;
	struct NX_GPIO_RegisterSet *gpioc = (struct NX_GPIO_RegisterSet*)PHY_BASEADDR_GPIO_C_MODULE;
	struct NX_MCUS_RegisterSet *mcus = (struct NX_MCUS_RegisterSet *)PHY_BASEADDR_MCUS_MODULE;
	struct	NX_PWM_RegisterSet *pwm = (struct	NX_PWM_RegisterSet *)PHY_BASEADDR_PWM_MODULE;

	gd->bd->bi_arch_number = MACH_TYPE;
	gd->bd->bi_boot_params = (PHYS_SDRAM_1 + 0x100);

	printf("Test board initialize!\n");

	/* Setup PWM first to set GPIOB2 to High */
	//pwm->PWM2[0].PWM_PREPOL = pwm->PWM2[0].PWM_PREPOL | 0x0000; /* Bit 7 high */
	/* Setup GPIO A for all GPIO ... Don't do anything for GPIO  */
	/* Setup GPIO as DM9000 works : All GPIOB port to ALT1 function. */
#ifndef CONFIG_BOOTSOUND_AC97
	gpiob->GPIOxALTFN[0] = 0x55555555; // 0x55555505
	gpiob->GPIOxALTFN[1] = 0x55555555;
#else
    /* if AC97 we should setup B8~B12 for AC97  ALT1 -> ALT2 */
    gpiob->GPIOxALTFN[0] = 0x56AA5555; // 0x55555505
    gpiob->GPIOxALTFN[1] = 0x55555555;
#endif
	/* Setup GPIO C port to ALT1 function except C16. */
	gpioc->GPIOxALTFN[0] = 0x55555555;
	gpioc->GPIOxALTFN[1] = 0x55555554;
	/* Setup MPU-S registers for Correct BUS IO */
	mcus->MEMBW	|= 0x02; /* CS1 should 16bit */
	/* Now all LED back on for LED test */
	/* PWM0 pin to output and set 1 */
	//gpiob->GPIOxOUT = 0x00000004; /* GPIO_B2 */
	//gpiob->GPIOxOUTENB = 0x00000004; /* GPIO_B2 */
	/* PortA pin output and set 1 */
#ifndef CONFIG_BOOTSOUND_AC97
	gpioa->GPIOxOUT = 0x001FFFFF; // Formerly 0x000fffff
#else
	gpioa->GPIOxOUT = 0x00000000;
#endif
	gpioa->GPIOxOUTENB = 0x001FFFF0; // Formerly 0x000fffff, LSB 4Bit INPUT for HIT N + Supernova

	enable_sndclk();

	return 0;
}

int dram_init(void)
{
	DECLARE_GLOBAL_DATA_PTR;
	
	gd->bd->bi_dram[0].start = PHYS_SDRAM_1;
	gd->bd->bi_dram[0].size = PHYS_SDRAM_1_SIZE;
#if CONFIG_NR_DRAM_BANKS == 2
	gd->bd->bi_dram[1].start = PHYS_SDRAM_2;
	gd->bd->bi_dram[1].size = PHYS_SDRAM_2_SIZE;
#endif

#if CONFIG_NR_DRAM_BANKS == 2
	gd->ram_size = get_ram_size((long *)PHYS_SDRAM_1, (PHYS_SDRAM_1_SIZE+PHYS_SDRAM_2_SIZE));
#else
	gd->ram_size = get_ram_size((long *)PHYS_SDRAM_1, (PHYS_SDRAM_1_SIZE));
#endif

	return 0;
}

#ifdef CONFIG_DISPLAY_BOARDINFO
int checkboard(void)
{
	printf("Board:   most2120\n");

	return (0);
}
#endif

#ifdef  CONFIG_SW_MAPPED_FASTBOOT
int getfastboot(void) {
	unsigned int sw;
	struct	NX_GPIO_RegisterSet *const gpioc = (struct	NX_GPIO_RegisterSet*)PHY_BASEADDR_GPIO_C_MODULE;
	sw = gpioc->GPIOxPAD; /* GPIOC_16 is fast boot */

	if ((sw & 0x00010000) != 0) {
		return 1; /* ksw:Temporarily */
	}
	return 0;
}
#endif

#ifdef CONFIG_BOOTSOUND
/* Some 8960 registers here */
#define WM8960_LINVOL		0x0
#define WM8960_RINVOL		0x1
#define WM8960_LOUT1		0x2
#define WM8960_ROUT1		0x3
#define WM8960_CLOCK1		0x4
#define WM8960_DACCTL1		0x5
#define WM8960_DACCTL2		0x6
#define WM8960_IFACE1		0x7
#define WM8960_CLOCK2		0x8
#define WM8960_IFACE2		0x9
#define WM8960_LDAC		0xa
#define WM8960_RDAC		0xb

#define WM8960_RESET		0xf
#define WM8960_3D		0x10
#define WM8960_ALC1		0x11
#define WM8960_ALC2		0x12
#define WM8960_ALC3		0x13
#define WM8960_NOISEG		0x14
#define WM8960_LADC		0x15
#define WM8960_RADC		0x16
#define WM8960_ADDCTL1		0x17
#define WM8960_ADDCTL2		0x18
#define WM8960_POWER1		0x19
#define WM8960_POWER2		0x1a
#define WM8960_ADDCTL3		0x1b
#define WM8960_APOP1		0x1c
#define WM8960_APOP2		0x1d

#define WM8960_LINPATH		0x20
#define WM8960_RINPATH		0x21
#define WM8960_LOUTMIX		0x22

#define WM8960_ROUTMIX		0x25
#define WM8960_MONOMIX1		0x26
#define WM8960_MONOMIX2		0x27
#define WM8960_LOUT2		0x28
#define WM8960_ROUT2		0x29
#define WM8960_MONO		0x2a
#define WM8960_INBMIX1		0x2b
#define WM8960_INBMIX2		0x2c
#define WM8960_BYPASS1		0x2d
#define WM8960_BYPASS2		0x2e
#define WM8960_POWER3		0x2f
#define WM8960_ADDCTL4		0x30
#define WM8960_CLASSD1		0x31

#define WM8960_CLASSD3		0x33
#define WM8960_PLL1		0x34
#define WM8960_PLL2		0x35
#define WM8960_PLL3		0x36
#define WM8960_PLL4		0x37


/*
 * WM8960 Clock dividers
 */
#define WM8960_SYSCLKDIV 		0
#define WM8960_DACDIV			1
#define WM8960_ADCDIV			2
#define WM8960_OPCLKDIV			3
#define WM8960_DCLKDIV			4
#define WM8960_TOCLKSEL			5
#define WM8960_SYSCLKSEL		6

#define WM8960_SYSCLK_DIV_1		(0 << 1)
#define WM8960_SYSCLK_DIV_2		(2 << 1)

#define WM8960_SYSCLK_MCLK		(0 << 0)
#define WM8960_SYSCLK_PLL		(1 << 0)

#define WM8960_DAC_DIV_1		(0 << 3)
#define WM8960_DAC_DIV_1_5		(1 << 3)
#define WM8960_DAC_DIV_2		(2 << 3)
#define WM8960_DAC_DIV_3		(3 << 3)
#define WM8960_DAC_DIV_4		(4 << 3)
#define WM8960_DAC_DIV_5_5		(5 << 3)
#define WM8960_DAC_DIV_6		(6 << 3)

#define WM8960_ADC_DIV_1		(0 << 6)
#define WM8960_ADC_DIV_1_5		(1 << 6)
#define WM8960_ADC_DIV_2		(2 << 6)
#define WM8960_ADC_DIV_3		(3 << 6)
#define WM8960_ADC_DIV_4		(4 << 6)
#define WM8960_ADC_DIV_5_5		(5 << 6)
#define WM8960_ADC_DIV_6		(6 << 6)

#define WM8960_DCLK_DIV_1_5		(0 << 6)
#define WM8960_DCLK_DIV_2		(1 << 6)
#define WM8960_DCLK_DIV_3		(2 << 6)
#define WM8960_DCLK_DIV_4		(3 << 6)
#define WM8960_DCLK_DIV_6		(4 << 6)
#define WM8960_DCLK_DIV_8		(5 << 6)
#define WM8960_DCLK_DIV_12		(6 << 6)
#define WM8960_DCLK_DIV_16		(7 << 6)

#define WM8960_TOCLK_F19		(0 << 1)
#define WM8960_TOCLK_F21		(1 << 1)

#define WM8960_OPCLK_DIV_1		(0 << 0)
#define WM8960_OPCLK_DIV_2		(1 << 0)
#define WM8960_OPCLK_DIV_3		(2 << 0)
#define WM8960_OPCLK_DIV_4		(3 << 0)
#define WM8960_OPCLK_DIV_5_5		(4 << 0)
#define WM8960_OPCLK_DIV_6		(5 << 0)

#define WM8960_BCLK_DIV_1		(0 << 0)
#define WM8960_BCLK_DIV_1_5		(1 << 0)
#define WM8960_BCLK_DIV_2		(2 << 0)
#define WM8960_BCLK_DIV_3		(3 << 0)
#define WM8960_BCLK_DIV_4	    (4 << 0)
#define WM8960_BCLK_DIV_5_5		(5 << 0)
#define WM8960_BCLK_DIV_6		(6 << 0)
#define WM8960_BCLK_DIV_8		(7 << 0)
#define WM8960_BCLK_DIV_11		(8 << 0)
#define WM8960_BCLK_DIV_12		(9 << 0)
#define WM8960_BCLK_DIV_16	    (10 << 0)
#define WM8960_BCLK_DIV_22		(11 << 0)
#define WM8960_BCLK_DIV_24		(12 << 0)
#define WM8960_BCLK_DIV_32		(13 << 0)

extern int i2c_write(uchar chip, uint addr, int alen, uchar *buf, int len);

void snd_soc_write(void *p, int addr, int val)
{
    uchar buf[2];
    buf[0] = ((addr << 1) & 0xFE) | ((val >> 8) & 0x01);
    buf[1] = val & 0xFF;
    i2c_write(0x1A, (unsigned int)buf[0], 1, &buf[1], 1);
    udelay(100);
}

#define codec NULL

#if 0
static const u16 wm8960_def_reg[56] = {
	0x0097, 0x0097, 0x0000, 0x0000,
	0x0000, 0x0008, 0x0000, 0x000a,
	0x01c0, 0x0000, 0x00ff, 0x00ff,
	0x0000, 0x0000, 0x0000, 0x0000,
	0x0000, 0x007b, 0x0100, 0x0032,
	0x0000, 0x00c3, 0x00c3, 0x01c0,
	0x0000, 0x0000, 0x0000, 0x0000,
	0x0000, 0x0000, 0x0000, 0x0000,
	0x0100, 0x0100, 0x0050, 0x0050,
	0x0050, 0x0050, 0x0000, 0x0000,
	0x0000, 0x0000, 0x0040, 0x0000,
	0x0000, 0x0050, 0x0050, 0x0000,
	0x0002, 0x0037, 0x004d, 0x0080,
	0x0008, 0x0031, 0x0026, 0x00e9,
};
#endif

void start_dma(char *p, int len, int channel)
{
    struct	NX_DMA_RegisterSet *dma = (struct NX_DMA_RegisterSet *)(PHY_BASEADDR_DMA_MODULE + OFFSET_OF_DMA_MODULE * channel);

    dma->DMASRCADDR = (unsigned int)p;
    dma->DMADSTADDR = PHY_BASEADDR_AUDIO_MODULE;
    dma->DMALENGTH = len-1;
    dma->DMAREQID = NXP2120_DMAREQ_ID_AUDIO_PCMOUT;
    dma->DMAMODE = 0; /* clear out DMAMODE */
#if defined(CONFIG_BOOTSOUND_SUPERNOVA32)
    /* Set Destination mode */
    dma->DMAMODE = dma->DMAMODE | NXP2120_DMAMODE_DSTNOTINC | NXP2120_DMAMODE_DESTIMODE_IO | NXP2120_DMAMODE_DSTIOSIZE_32;
    /* Set Source mode */
    dma->DMAMODE = dma->DMAMODE | NXP2120_DMAMODE_SRCNOTREQCHK | NXP2120_DMAMODE_SRCIOMODE_MEM | NXP2120_DMAMODE_SRCIOSIZE_32;
#else
    /* Set Destination mode */
    dma->DMAMODE = dma->DMAMODE | NXP2120_DMAMODE_DSTNOTINC | NXP2120_DMAMODE_DESTIMODE_IO | NXP2120_DMAMODE_DSTIOSIZE_16;
    /* Set Source mode */
    dma->DMAMODE = dma->DMAMODE | NXP2120_DMAMODE_SRCNOTREQCHK | NXP2120_DMAMODE_SRCIOMODE_MEM | NXP2120_DMAMODE_SRCIOSIZE_16;
#endif
    /* Start DMA */
    dma->DMAMODE = dma->DMAMODE | NXP2120_DMAMODE_RUN;
}

#if PLL_CALC
/* PLL divisors */
struct _pll_div {
	u32 pre_div:1;
	u32 n:4;
	u32 k:24;
};

struct _pll_div pll_div;
/* The size in bits of the pll divide multiplied by 10
 * to allow rounding later */
#define FIXED_PLL_SIZE ((1 << 24) * 10)

static int pll_factors(unsigned int source, unsigned int target,
		       struct _pll_div *pll_div)
{
	unsigned long long Kpart;
	unsigned int K, Ndiv, Nmod;

	printf("WM8960 PLL: setting %dHz->%dHz\n", source, target);

	/* Scale up target to PLL operating frequency */
	target *= 8;

	Ndiv = target / source;
	//printk("Ndiv = %d\n", Ndiv);
	if (Ndiv < 6) {
		source >>= 1;
		pll_div->pre_div = 1;
		Ndiv = target / source;
	} else
		pll_div->pre_div = 0;

	if ((Ndiv < 6) || (Ndiv > 12)) {
		printf("WM8960 PLL: Unsupported N=%d\n", Ndiv);
		return -1;
	}

	pll_div->n = Ndiv;
	Nmod = target % source;
	Kpart = FIXED_PLL_SIZE * (long long)Nmod;

	printf("Kpart = %ld\n", Kpart);

	Kpart /= source;
	printf("Kpart2 = %d\n", Kpart);

	K = Kpart & 0xFFFFFFFF;

	/* Check if we need to round */
	if ((K % 10) >= 5)
		K += 5;

	/* Move down to proper range now rounding is done */
	K /= 10;

	pll_div->k = K;

	//printk("K = %d\n", K);

	printf("WM8960 PLL: N=%x K=%x pre_div=%d\n",
		 pll_div->n, pll_div->k, pll_div->pre_div);

	return 0;
}
#endif

void play_bootsound(void)
{
    int datalen;
    uint regs;
    unsigned short int *d16, *dd;
    unsigned int *d32;
    int flen, i, done = 0;
    unsigned int clksel0;
    struct	NX_AUDIO_RegisterSet *i2s_reg = (struct NX_AUDIO_RegisterSet *)PHY_BASEADDR_AUDIO_MODULE;
    struct	NX_DMA_RegisterSet *dma = (struct	NX_DMA_RegisterSet *)PHY_BASEADDR_DMA_MODULE;
	struct  NX_GPIO_RegisterSet *gpioa = (struct NX_GPIO_RegisterSet*)PHY_BASEADDR_GPIO_A_MODULE;
	struct  NX_UART_RegisterSet *uart1 = (struct NX_UART_RegisterSet *)PHY_BASEADDR_UART1_MODULE;
	int ch;
	int rbytes;
	int msg[] =  { 'Q', 'U', 'I', 'E', 'T', 0 };

	regs = uart1->FSTATUS;

	rbytes = (regs & NXP2120_FSTATUS_RX_FIFO_FULL) == 0 ? (regs & 0x0F) : 16;

	i = 0;
	while (msg[i] != 0 && rbytes > 0)
	{
		ch = uart1->RHB;

		if (ch == msg[i])
			i++;

		rbytes--;
	}
	if (msg[i] == 0)
	{
		return;
	}

    datalen = sizeof(data) / 2;
    d16 = (unsigned short int *)data;

    //return;
    //udelay(100000);
    printf("Start audio playback\n");

    /* First enable GPIO for clk generation */
    /* Already done from board_init()       */

    /* Check I2C is configured. */
#ifndef CONFIG_NXP2120_I2C
#error Sound support need I2C support!
#endif

#if !defined(CONFIG_BOOTSOUND_AC97)
    /* Setup audio hardware */
    i2s_reg->I2S_CTRL = NXP2120_I2S_CTRL_I2S_EN; /* I2S Enable */
	i2s_reg->AC97_CTRL = 0; /* AC97 Controller disable */
	i2s_reg->AUDIO_BUFF_CTRL = 0;
	i2s_reg->AUDIO_BUFF_CONFIG =  0;
	i2s_reg->I2S_CONFIG = NXP2120_I2S_CONFIG_CTRL_MODE_SLAVE | NXP2120_I2S_CONFIG_SYNC_PERIOD_64FS | NXP2120_I2S_CONFIG_IF_MODE_I2S;
	//i2s_reg->AC97_CONFIG |= 1; /* FRONT_EN should be 1 for USING I2S enable */
	i2s_reg->I2S_CTRL = NXP2120_I2S_CTRL_I2S_EN | NXP2120_I2S_CTRL_I2SLINK_RUN;

#if !(defined(CONFIG_BOOTSOUND_SUPERNOVA16) || defined(CONFIG_BOOTSOUND_SUPERNOVA32))
    //udelay(100000);
    /* Reset all register first */
    snd_soc_write(codec, WM8960_RESET, 0);

#if 0
    /* As paranoia, Set all registers to default value */
    for (i=0; i<56; i++) {
        if (i != WM8960_RESET)
            snd_soc_write(codec, i, wm8960_def_reg[i]);
    }
#endif

    /* Now we should enable WM8960 and set WM8960 as correct status as we wanted. */
    snd_soc_write(codec, WM8960_POWER1, 0x1F2); //0x1F2
    /* Speaker Power Enable */
    snd_soc_write(codec, WM8960_POWER2, 0x7F);
    snd_soc_write(codec, WM8960_IFACE2, 0x040);  /* ADCLRC is GPIO1 */
    snd_soc_write(codec, WM8960_ADDCTL2, 0x044); /* LRCM On/ADCRC input/HPSWEN/HPSWPOL=1 */
    /* Set GPIO to GPIO JD */
    snd_soc_write(codec, WM8960_ADDCTL4, 0x002); /* GPIO1 is JD */
    /* Set LINVOL/RINVOL to set 0dB */
    snd_soc_write(codec, WM8960_LINVOL, 0x140 | 27); /* Set to 0dB */
    snd_soc_write(codec, WM8960_RINVOL, 0x140 | 27); /* Set to 0dB */
    /* Set ADCVOL to 0dB */
    snd_soc_write(codec, WM8960_LADC, 0x100 | 194); /* Set to 0dB */
    snd_soc_write(codec, WM8960_RADC, 0x100 | 194); /* Set to 0dB */
    /* Set MICBOOST to 20dB/no Mute */
    snd_soc_write(codec, WM8960_LINPATH, 0x100 | (0x02 << 4) | 0x08); /* Boost 20dB, Boost connect */
    snd_soc_write(codec, WM8960_RINPATH, 0x100 | (0x02 << 4) | 0x08); /* Boost 20dB, Boost connect */
    /* Set LOUT1/ROUT1 volume to set 0dB */
    snd_soc_write(codec, WM8960_LOUT1, 0x1F9);
    snd_soc_write(codec, WM8960_ROUT1, 0x1F9);
    /* Set output mixer DAC mute to off */
    snd_soc_write(codec, WM8960_LOUTMIX, 0x180);
    snd_soc_write(codec, 0x23, 0x180);
    snd_soc_write(codec, 0x24, 0x180);
    snd_soc_write(codec, WM8960_ROUTMIX, 0x180);
    /* Set LOUT2/ROUT2 volume to set 0dB */
    snd_soc_write(codec, WM8960_LOUT2, 0x1F9);
    snd_soc_write(codec, WM8960_ROUT2, 0x1F9);
    /* Power on Mixer */
    snd_soc_write(codec, WM8960_POWER3, 0x03C); // 0x03C for L,R micamp on

#if 1
    /* set codec DAI configuration */
    snd_soc_write(codec, WM8960_IFACE1 , 0x42); /* Set master mode for generate bitclk and fs(lrclk) */
    /* set cpu DAI configuration */
    /* set CPU generate reference clock for Codec's PLL. CLOCK ouput enable should turn on.  */
    /* Set prescaler to 16 to generate 12MHz around clock */
    clksel0 = i2s_reg->CLKGEN[0][0];
	clksel0 &= ~NXP2120_AUDIO_CLKGEN0_CLKDIV0_MASK;
	clksel0 |= (16-1) << 5;
	i2s_reg->CLKGEN[0][0] = clksel0;

	clksel0 = i2s_reg->CLKGEN[1][0];
	clksel0 &= ~(NXP2120_AUDIO_CLKGEN1_CLKDIV1_MASK | NXP2120_AUDIO_CLKGEN1_CLKSRCSEL1_MASK);
	clksel0 |= NXP2120_AUDIO_CLKGEN1_OUTCLKENB1;
	clksel0 |= (3 << 2); // BITCLK from external source
	i2s_reg->CLKGEN[1][0] = clksel0;

#if PLL_CALC
	pll_factors(12000000, 12288000, &pll_div);
#endif

    /* set codec sysclock from PLL source */
    snd_soc_write(codec, WM8960_CLOCK1, 0x00);

    /* Now All the bit rate and rootclock should be set. */
    /* set codec sysclock from PLL source */
    snd_soc_write(codec, WM8960_POWER2 , 0x7E);
    /* These PLL values should be calculated correctly. */
    snd_soc_write(codec, WM8960_PLL2, 0x31);
    snd_soc_write(codec, WM8960_PLL3, 0x26);
    snd_soc_write(codec, WM8960_PLL4, 0xE9);
    snd_soc_write(codec, WM8960_PLL1, 0x28);
    snd_soc_write(codec, WM8960_POWER2 , 0x7F);

    /* set codec speaker DCLK for SYSCLK/2, and BCLK as  1/12 */
    snd_soc_write(codec, WM8960_CLOCK2, 0x040 | 9); // SYSCLK/2, BCLK_DIV_12

    //udelay(400000);
    dd = 0x81000000;
    for (i=0; i<datalen; i++) {
        *dd++ = *d16;
        *dd++ = *d16++;
    }
    dd = 0x81000000;

    /* set codec DAC div factor to ddiv/adiv. */
    snd_soc_write(codec, WM8960_CLOCK1, 0x0DD); // As 16KHz...
#endif
    snd_soc_write(codec, WM8960_POWER1, 0xf2);
    snd_soc_write(codec, WM8960_POWER2, 0x1ff);
    snd_soc_write(codec, WM8960_DACCTL1, 0); /* Mute off */
    /* Speaker Output Driver enable */
    snd_soc_write(codec, WM8960_CLASSD1, 0xC0); /* F7 Disables R speaker*/
//    udelay(400000);
#else // Now supernova16 or 32 is defined.
	// Just wait some time...
	udelay(10000); // first delay 100ms

#if defined(CONFIG_BOOTSOUND_SUPERNOVA16)
	dd = 0x81000000;
    for (i=0; i<datalen; i++, d16++) {
        *dd++ = *d16;
        *dd++ = *d16;
        *dd++ = *d16;
        *dd++ = *d16;
        *dd++ = *d16;
        *dd++ = *d16;
    }
    datalen = datalen * 3; // 48KHz output
    
#endif
#if defined(CONFIG_BOOTSOUND_SUPERNOVA32)
	d32 = 0x81000000;
    for (i=0; i<datalen; i++,d16++) {
        *d32++ = ((unsigned int)(*d16)) << 8;
        *d32++ = ((unsigned int)(*d16)) << 8;
        *d32++ = ((unsigned int)(*d16)) << 8;
        *d32++ = ((unsigned int)(*d16)) << 8;
        *d32++ = ((unsigned int)(*d16)) << 8;
        *d32++ = ((unsigned int)(*d16)) << 8;
    }
    datalen = datalen * 3 * 2; // 32bit data and 48Khz data
#endif
    dd = 0x81000000;
	// and CLKGEN from external source...
	clksel0 = i2s_reg->CLKGEN[0][0];
	clksel0 &= ~NXP2120_AUDIO_CLKGEN0_CLKDIV0_MASK;
	clksel0 |= (16-1) << 5;
	i2s_reg->CLKGEN[0][0] = clksel0;

	clksel0 = i2s_reg->CLKGEN[1][0];
	clksel0 &= ~(NXP2120_AUDIO_CLKGEN1_CLKDIV1_MASK | NXP2120_AUDIO_CLKGEN1_CLKSRCSEL1_MASK);
	clksel0 |= NXP2120_AUDIO_CLKGEN1_OUTCLKENB1;
	clksel0 |= (3 << 2); // BITCLK from external source
	i2s_reg->CLKGEN[1][0] = clksel0;
#endif

#if 1
    /* Setup DMA */
    if (datalen * 4 > 65536)
    	start_dma(dd, 65536, 0);
    else
    	start_dma(dd, datalen*4, 0);
    dma->DMAMODE =  dma->DMAMODE | NXP2120_DMAMODE_INTPEND; // Cleear IRQPEND
    /* Start Audio transfer */
    i2s_reg->I2S_CONFIG = i2s_reg->I2S_CONFIG | NXP2120_I2S_CONFIG_I2S_OUT_EN; /* I2S Out Enable */
	i2s_reg->AUDIO_BUFF_CTRL = i2s_reg->AUDIO_BUFF_CTRL | NXP2120_AUDIO_BUFF_CTRL_PCMOBUF_EN; /* PCMOUT buffer enable */

    /* All register setup then start FIFO and play sound */
    while (!done) {
        /* Wait DMA is done. */
        if (0 != (dma->DMAMODE & NXP2120_DMAMODE_INTPEND)) break;
    }
    if (datalen * 4 > 65536) {
    	start_dma(dd+65536/2, datalen * 4 - 65536, 0);
    	while (!done) {
    		/* Wait DMA is done. */
        	if (0 != (dma->DMAMODE & NXP2120_DMAMODE_INTPEND)) break;
        }
    }
#endif

#else // AC97_BOOTSOUND
    /* Setup audio hardware for AC97 */
    i2s_reg->I2S_CTRL = 0; /* I2S disable */
	i2s_reg->AC97_CTRL = 7; /* AC97 Controller Enable */
	i2s_reg->AUDIO_BUFF_CTRL = 0;
	i2s_reg->AUDIO_BUFF_CONFIG =  0; /* ALL 16bit */
	i2s_reg->I2S_CONFIG = 0;
	i2s_reg->AC97_CONFIG = 1; /* FRONT_EN should be 1 for ac97 stereo enable */

    //udelay(100000);
    /* Reset all register first */
    snd_soc_write(codec, WM8960_RESET, 0);

#if 0
    /* As paranoia, Set all registers to default value */
    for (i=0; i<56; i++) {
        if (i != WM8960_RESET)
            snd_soc_write(codec, i, wm8960_def_reg[i]);
    }
#endif

    /* Now we should enable WM8960 and set WM8960 as correct status as we wanted. */
    snd_soc_write(codec, WM8960_POWER1, 0x1F2); //0x1F2
    /* Speaker Power Enable */
    snd_soc_write(codec, WM8960_POWER2, 0x7F);
    snd_soc_write(codec, WM8960_IFACE2, 0x040);  /* ADCLRC is GPIO1 */
    snd_soc_write(codec, WM8960_ADDCTL2, 0x044); /* LRCM On/ADCRC input/HPSWEN/HPSWPOL=1 */
    /* Set GPIO to GPIO JD */
    snd_soc_write(codec, WM8960_ADDCTL4, 0x002); /* GPIO1 is JD */
    /* Set LINVOL/RINVOL to set 0dB */
    snd_soc_write(codec, WM8960_LINVOL, 0x140 | 27); /* Set to 0dB */
    snd_soc_write(codec, WM8960_RINVOL, 0x140 | 27); /* Set to 0dB */
    /* Set ADCVOL to 0dB */
    snd_soc_write(codec, WM8960_LADC, 0x100 | 194); /* Set to 0dB */
    snd_soc_write(codec, WM8960_RADC, 0x100 | 194); /* Set to 0dB */
    /* Set MICBOOST to 20dB/no Mute */
    snd_soc_write(codec, WM8960_LINPATH, 0x100 | (0x02 << 4) | 0x08); /* Boost 20dB, Boost connect */
    snd_soc_write(codec, WM8960_RINPATH, 0x100 | (0x02 << 4) | 0x08); /* Boost 20dB, Boost connect */
    /* Set LOUT1/ROUT1 volume to set 0dB */
    snd_soc_write(codec, WM8960_LOUT1, 0x1F9);
    snd_soc_write(codec, WM8960_ROUT1, 0x1F9);
    /* Set output mixer DAC mute to off */
    snd_soc_write(codec, WM8960_LOUTMIX, 0x180);
    snd_soc_write(codec, 0x23, 0x180);
    snd_soc_write(codec, 0x24, 0x180);
    snd_soc_write(codec, WM8960_ROUTMIX, 0x180);
    /* Set LOUT2/ROUT2 volume to set 0dB */
    snd_soc_write(codec, WM8960_LOUT2, 0x1F9);
    snd_soc_write(codec, WM8960_ROUT2, 0x1F9);
    /* Power on Mixer */
    snd_soc_write(codec, WM8960_POWER3, 0x03C); // 0x03C for L,R micamp on

#if 1
    /* set codec DAI configuration */
    snd_soc_write(codec, WM8960_IFACE1 , 0x02); /* Set slave mode  and I2S mode */
    /* set cpu DAI configuration */
    /* set CPU generate reference clock for Codec's PLL. CLOCK ouput enable should turn on.  */
    /* Set prescaler to 16 to generate 12MHz around clock */
    clksel0 = i2s_reg->CLKGEN[0][0];
	clksel0 &= ~NXP2120_AUDIO_CLKGEN0_CLKDIV0_MASK;
	clksel0 |= (16-1) << 5;
	i2s_reg->CLKGEN[0][0] = clksel0;

	clksel0 = i2s_reg->CLKGEN[1][0];
	clksel0 &= ~(NXP2120_AUDIO_CLKGEN1_CLKDIV1_MASK | NXP2120_AUDIO_CLKGEN1_CLKSRCSEL1_MASK | NXP2120_AUDIO_CLKGEN1_OUTCLKENB1);
	//clksel0 |= NXP2120_AUDIO_CLKGEN1_OUTCLKENB1; // We don't need to output clock for AC97
	clksel0 |= (3 << 2); // BITCLK from external source
	i2s_reg->CLKGEN[1][0] = clksel0;

    /* set codec sysclock from SYSCLK source */
    snd_soc_write(codec, WM8960_CLOCK1, 0x00);

    /* Now All the bit rate and rootclock should be set. */
    snd_soc_write(codec, WM8960_POWER2 , 0x7F);

    /* set codec speaker DCLK for SYSCLK/2, and BCLK as  1/12 */
    snd_soc_write(codec, WM8960_CLOCK2, 0x040 | 9); // SYSCLK/2, BCLK_DIV_12

    //udelay(400000);
    dd = (unsigned short *)0x81000000;
    for (i=0; i<datalen; i++) {
        *dd++ = *d16;
        *dd++ = *d16++;
    }
    dd = (unsigned short *)0x81000000;

    /* set codec DAC div factor to ddiv/adiv. */
    snd_soc_write(codec, WM8960_CLOCK1, 0x000); // As 16KHz... 256 * 16000 =
#endif
    snd_soc_write(codec, WM8960_POWER1, 0xf2);
    snd_soc_write(codec, WM8960_POWER2, 0x1ff);
    snd_soc_write(codec, WM8960_DACCTL1, 0); /* Mute off */
    /* Speaker Output Driver enable */
    snd_soc_write(codec, WM8960_CLASSD1, 0xC0); /* F7 Disables R speaker*/
//    udelay(400000);
#if 1
    /* Setup DMA */
    start_dma(dd, datalen*4, 0);
    dma->DMAMODE =  dma->DMAMODE | NXP2120_DMAMODE_INTPEND; // Cleear IRQPEND
    /* Start Audio transfer */
	i2s_reg->AUDIO_BUFF_CTRL = i2s_reg->AUDIO_BUFF_CTRL | NXP2120_AUDIO_BUFF_CTRL_PCMOBUF_EN; /* PCMOUT buffer enable */

    /* All register setup then start FIFO and play sound */
    while (!done) {
        /* Wait DMA is done. */
        if (0 != (dma->DMAMODE & NXP2120_DMAMODE_INTPEND)) break;
    }
#endif
#endif
    /* Do anything to clean up? */
    snd_soc_write(codec, WM8960_CLASSD1, 0x00); /* 0 Disables all speakers*/
}
#endif


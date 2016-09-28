/*
 * Copyright(c) 2011 MOSTiTECH co., ltd. 
 * All right reserved by Seungwoo Kim <ksw@mostitech.com>
 *
 */
 /* All necessary bootloader needed stuf is exist in this file. */
#ifndef __NXP2120_H__
#define __NXP2120_H__

/************************************************
 * NAME	    : nxp2120.h
 *
 * Based on nxp2120 User's manual Rev 0.90
 ************************************************/

typedef unsigned char U8;
typedef unsigned  short int U16;
typedef unsigned int U32;

//------------------------------------------------------------------------------
// Module declaration.
//------------------------------------------------------------------------------
#ifndef NX_CHIP_MODULE_DECLARATION
#define	NX_CHIP_MODULE_DECLARATION( _name_, _num_, _addr_, _offset_ )	\
	enum {																\
		NUMBER_OF_ ## _name_ ## _MODULE		= _num_,					\
		PHY_BASEADDR_ ## _name_ ## _MODULE	= _addr_,					\
		OFFSET_OF_ ## _name_ ## _MODULE		= _offset_					\
	};
#endif	// NX_CHIP_MODULE_DECLARATION

//------------------------------------------------------------------------------
//							(	Name , Number,	Addr	,	Offset	)
//------------------------------------------------------------------------------
NX_CHIP_MODULE_DECLARATION( ALIVE		, 1, 0xC0019000, 0x00000000 )
NX_CHIP_MODULE_DECLARATION( CLKPWR		, 1, 0xC000F000, 0x00000000 )
NX_CHIP_MODULE_DECLARATION( INTC		, 1, 0xC0000800, 0x00000000 )
NX_CHIP_MODULE_DECLARATION( DMA			, 8, 0xC0000000, 0x00000080 )
NX_CHIP_MODULE_DECLARATION( RTC			, 1, 0xC0019400, 0x00000000 )
NX_CHIP_MODULE_DECLARATION( TIMER		, 5, 0xC0001800, 0x00000080 )
NX_CHIP_MODULE_DECLARATION( ECID		, 1, 0xC001F800, 0x00000000 )

NX_CHIP_MODULE_DECLARATION( MCUD		, 1, 0xC0014800, 0x00000000 )
NX_CHIP_MODULE_DECLARATION( MCUS		, 1, 0xC0015800, 0x00000000 )

NX_CHIP_MODULE_DECLARATION( MLC			, 1, 0xC0004000, 0x00000400 )
NX_CHIP_MODULE_DECLARATION( DPC			, 1, 0xC000307C, 0x00000400 )
NX_CHIP_MODULE_DECLARATION( ROTATOR		, 1, 0xC0004800, 0x00000000 )
NX_CHIP_MODULE_DECLARATION( SCALER		, 1, 0xC0003800, 0x00000000 )
NX_CHIP_MODULE_DECLARATION( VIP			, 2, 0xC0002800, 0x0000E000 )
NX_CHIP_MODULE_DECLARATION( DEINTERLACE	, 1, 0xC0014000, 0x00000000 )
NX_CHIP_MODULE_DECLARATION( CSC			, 1, 0xC0009000, 0x00000000 )
NX_CHIP_MODULE_DECLARATION( GRP3D		, 1, 0xC001A000, 0x00000000 )

NX_CHIP_MODULE_DECLARATION( MPEG		, 1, 0xC0006800, 0x00000000 )
NX_CHIP_MODULE_DECLARATION( H264		, 1, 0xC0006000, 0x00000000 )

NX_CHIP_MODULE_DECLARATION( AUDIO		, 1, 0xC000D800, 0x00000000 )

NX_CHIP_MODULE_DECLARATION( SDHC		, 1, 0xC0009800, 0x00003000 )
NX_CHIP_MODULE_DECLARATION( UART0		, 1, 0xC0016000, 0x00000000 )
NX_CHIP_MODULE_DECLARATION( UART1		, 1, 0xC0016080, 0x00000000 )
NX_CHIP_MODULE_DECLARATION( SSPSPI		, 1, 0xC0007800, 0x00000800 )
NX_CHIP_MODULE_DECLARATION( I2C0		, 1, 0xC000E000, 0x00000000 )
NX_CHIP_MODULE_DECLARATION( I2C1		, 1, 0xC000E800, 0x00000000 )
NX_CHIP_MODULE_DECLARATION( MPEGTSIF	, 1, 0xC0007000, 0x00000000 )
NX_CHIP_MODULE_DECLARATION( OHCI		, 1, 0xC0019800, 0x00000000 )
NX_CHIP_MODULE_DECLARATION( EHCI		, 1, 0xC000D000, 0x00000000 )
NX_CHIP_MODULE_DECLARATION( UDC			, 1, 0xC0018000, 0x00000000 )

NX_CHIP_MODULE_DECLARATION( GPIO		, 3, 0xC000A000, 0x00000040 )
NX_CHIP_MODULE_DECLARATION( GPIO_A		, 1, 0xC000A000, 0x00000040 )
NX_CHIP_MODULE_DECLARATION( GPIO_B		, 1, 0xC000A040, 0x00000040 )
NX_CHIP_MODULE_DECLARATION( GPIO_C		, 1, 0xC000A080, 0x00000040 )
NX_CHIP_MODULE_DECLARATION( GPIO_D		, 1, 0xC000A0C0, 0x00000040 )
NX_CHIP_MODULE_DECLARATION( GPIO_E		, 1, 0xC000A100, 0x00000040 )
NX_CHIP_MODULE_DECLARATION( ADC			, 1, 0xC0005000, 0x00000000 )
NX_CHIP_MODULE_DECLARATION( PWM			, 1, 0xC000C000, 0x00000000 )
NX_CHIP_MODULE_DECLARATION( PPM			, 1, 0xC000A800, 0x00000000 )
NX_CHIP_MODULE_DECLARATION( SPDIF		, 1, 0xC0011800, 0x00000000 )

/* For clock power registers */
struct	NX_CLKPWR_RegisterSet {
		volatile U32 CLKMODEREG0;				///< 0x00 : Clock Mode Register 0
		volatile U32 CLKMODEREG1;				///< 0x04 : Clock Mode Register 1
		volatile U32 PLLSETREG[2];				///< 0x08 ~ 0x0C : PLL Setting Register
		volatile U8 __Reserved00[0x40-0x10];	///< 0x10 ~ 0x3C : Reserved Region
		volatile U32 GPIOWAKEUPRISEENB;			///< 0x40 : GPIO Rising Edge Detect Enable Register
		volatile U32 GPIOWAKEUPFALLENB;			///< 0x44 : GPIO Falling Edge Detect Enable Register
		volatile U32 GPIORSTENB;				///< 0x48 : GPIO Reset Enable Register
		volatile U32 GPIOWAKEUPENB;				///< 0x4C : GPIO Wakeup Source Enable
		volatile U32 GPIOINTENB;				///< 0x50 : Interrupt Enable Register
		volatile U32 GPIOINTPEND;				///< 0x54 : Interrupt Pend Register
		volatile U32 RESETSTATUS;				///< 0x58 : Reset Status Register
		volatile U32 INTENABLE;					///< 0x5C : Interrupt Enable Register
		volatile U32 INTPEND;					///< 0x60 : Interrupt Pend Register
		volatile U32 PWRCONT;					///< 0x64 : Power Control Register
		volatile U32 PWRMODE;					///< 0x68 : Power Mode Register
		volatile U32 __Reserved01;				///< 0x6C : Reserved Region
		volatile U32 SCRATCH[3];				///< 0x70 ~ 0x78	: Scratch Register
		volatile U32 SYSRSTCONFIG;				///< 0x7C : System Reset Configuration Register.
		volatile U8 __Reserved02[0x100-0x80];	///< 0x80 ~ 0xFC	: Reserved Region
		volatile U32 PADSTRENGTHGPIO[5][2];		///< 0x100, 0x104 : GPIOA Pad Strength Register
												///< 0x108, 0x10C : GPIOB Pad Strength Register
												///< 0x110, 0x114 : GPIOC Pad Strength Register
												///< 0x118, 0x11C : GPIOD Pad Strength Register
												///< 0x120, 0x124 : GPIOE Pad Strength Register
		volatile U32 __Reserved03[2];			///< 0x128 ~ 0x12C: Reserved Region
		volatile U32 PADSTRENGTHBUS;			///< 0x130		: Bus Pad Strength Register
};

#define NXP2120_PWRCONT_SWRSTENB    (1 << 3)
#define NXP2120_PWRCONT_RTCWKENB    (1 << 1)
#define NXP2120_PWRMODE_CHGPLL      (1 << 15)
#define NXP2120_PWRMODE_SWRST       (1 << 12)
#define NXP2120_PWRMODE_LASTPWRSTOP      (1 << 5)
#define NXP2120_PWRMODE_LASTPWRIDLE      (1 << 4)
#define NXP2120_PWRMODE_STOP        (1 << 1)
#define NXP2120_PWRMODE_IDLE        (1 << 0)

/* for UART */
struct	NX_UART_RegisterSet	{
		volatile U16	LCON;			///< 0x00 : Line Control Register
		volatile U16	UCON;			///< 0x02 : Control Register
		volatile U16	FCON;			///< 0x04 : FIFO Control Register
		volatile U16	MCON;			///< 0x06 : Modem Control Register
		volatile U16	TRSTATUS;		///< 0x08 : Tx/Rx Status Register
		volatile U16	ESTATUS;		///< 0x0a : Error Status Register
		volatile U16	FSTATUS;		///< 0x0c : FIFO Status Register
		volatile U16	MSTATUS;		///< 0x0e : Modem Status Register
		volatile U16	THB;			///< 0x10 : Transmit Buffer Register
		volatile U16	RHB;			///< 0x12 : Receive Buffer Register
		volatile U16	BRD;			///< 0x14 : Baud Rate Divisor Register
		volatile U16	TIMEOUT;		///< 0x16 : Receive TimeOut Register
		volatile U16	INTCON;			///< 0x18 : Interrupt Control Register
		volatile U16	__Reserved[0x13];///< 0x1A ~ 0x3E : Reserved Region
		volatile U32	CLKENB;			///< 0x40 : Clock Enable Register
		volatile U32	CLKGEN;			///< 0x44 : Clock Generate Register
};

#define NXP2120_LCON_SYNC_PENDCLR       (1<<7)
#define NXP2120_LCON_SIR_MODE           (1<<6)
#define NXP2120_LCON_PARITY_MASK        (7<<3)
#define NXP2120_LCON_PARITY_ODD         (4<<3)
#define NXP2120_LCON_PARITY_EVEN        (5<<3)
#define NXP2120_LCON_PARITY_NONE        (0<<3)
#define NXP2120_LCON_ONE_STOP_BIT       (0<<2)
#define NXP2120_LCON_TWO_STOP_BIT       (1<<2)
#define NXP2120_LCON_WORDLEN_MASK       (3<<0)
#define NXP2120_LCON_WORDLEN_5BIT       (0<<0)
#define NXP2120_LCON_WORDLEN_6BIT       (1<<0)
#define NXP2120_LCON_WORDLEN_7BIT       (2<<0)
#define NXP2120_LCON_WORDLEN_8BIT       (3<<0)

#define NXP2120_UCON_TX_INT             (1<<9)
#define NXP2120_UCON_RX_INT             (1<<8)
#define NXP2120_UCON_RX_TIMEOUT         (1<<7)
#define NXP2120_UCON_RX_ERRSTATUS       (1<<6)
#define NXP2120_UCON_LOOPBACK_MODE      (1<<5)
#define NXP2120_UCON_SEND_BREAK         (1<<4)
#define NXP2120_UCON_TRANS_MODE_MASK    (3<<2)
#define NXP2120_UCON_TRANS_DISABLE      (0<<2)
#define NXP2120_UCON_TRANS_IRQ_POLLING  (1<<2)
#define NXP2120_UCON_TRANS_DMA_REQ      (2<<2)
#define NXP2120_UCON_RECV_MODE_MASK     (3<<0)
#define NXP2120_UCON_RECV_DISABLE       (0<<0)
#define NXP2120_UCON_RECV_IRQ_POLLING   (1<<0)
#define NXP2120_UCON_RECV_DMA_REQ       (2<<0)

#define NXP2120_FCON_TX_FIFO_TRIG_MASK  (3<<6)
#define NXP2120_FCON_TX_FIFO_TRIG_0BYTE (0<<6)
#define NXP2120_FCON_TX_FIFO_TRIG_4BYTE (1<<6)
#define NXP2120_FCON_TX_FIFO_TRIG_8BYTE (2<<6)
#define NXP2120_FCON_TX_FIFO_TRIG_12BYTE (3<<6)
#define NXP2120_FCON_RX_FIFO_TRIG_MASK  (3<<4)
#define NXP2120_FCON_RX_FIFO_TRIG_0BYTE (0<<4)
#define NXP2120_FCON_RX_FIFO_TRIG_4BYTE (1<<4)
#define NXP2120_FCON_RX_FIFO_TRIG_8BYTE (2<<4)
#define NXP2120_FCON_RX_FIFO_TRIG_12BYTE (3<<4)
#define NXP2120_FCON_TX_FIFO_RESET      (1<<2)
#define NXP2120_FCON_RX_FIFO_RESET      (1<<1)
#define NXP2120_FCON_FIFO_ENABLE        (1<<0)

#define NXP2120_MCON_HALF_CH_ENB        (1<<7)
#define NXP2120_MCON_SCRXENB            (1<<6)
#define NXP2120_MCON_SCTXENB            (1<<5)
#define NXP2120_MCON_AFC                (1<<4)
#define NXP2120_MCON_DTR_ACTIVE         (1<<1)
#define NXP2120_MCON_RTS_ACTIVE         (1<<0)

#define NXP2120_TRSTATUS_TX_EMPTY       (1<<2)
#define NXP2120_TRSTATUS_TX_BUF_EMPTY   (1<<1)
#define NXP2120_TRSTATUS_RX_BUF_READY   (1<<0)

#define NXP2120_ESTATUS_BREAK_DETECT   (1<<3)
#define NXP2120_ESTATUS_FRAME_ERROR    (1<<2)
#define NXP2120_ESTATUS_PARITY_ERROR   (1<<1)
#define NXP2120_ESTATUS_OVERRUN_ERROR  (1<<0)

#define NXP2120_FSTATUS_RX_FIFO_ERROR  (1<<10)
#define NXP2120_FSTATUS_TX_FIFO_FULL   (1<<9)
#define NXP2120_FSTATUS_RX_FIFO_FULL   (1<<8)
#define NXP2120_FSTATUS_TX_FIFO_COUNT  (0xF<<4)
#define NXP2120_FSTATUS_RX_FIFO_COUNT  (0xF<<0)

#define NXP2120_MSTATUS_DELTA_DCD      (1<<7)
#define NXP2120_MSTATUS_DELTA_RI       (1<<6)
#define NXP2120_MSTATUS_DELTA_DSR      (1<<5)
#define NXP2120_MSTATUS_DELTA_CTS      (1<<4)
#define NXP2120_MSTATUS_DCD            (1<<3)
#define NXP2120_MSTATUS_RI             (1<<2)
#define NXP2120_MSTATUS_DSR            (1<<1)
#define NXP2120_MSTATUS_CTS            (1<<0)

#define NXP2120_UART_CLKENB_PCLKMODE   (1<<3)
#define NXP2120_UART_CLKENB_CLKGENB    (1<<2)
#define NXP2120_UART_CLKENB_RESERVED   (3<<0)

#define NXP2120_UART_CLKGEN_SRC_SEL_MASK (0x07 << 2)
#define NXP2120_UART_CLKGEN_SRC_PLL0   (0x0 << 2)
#define NXP2120_UART_CLKGEN_SRC_PLL1   (0x1 << 2)

/* For Timer */
struct	NX_TIMER_RegisterSet {
		volatile U32 TMRCOUNT;			///< 0x00 : Timer counter register
		volatile U32 TMRMATCH;			///< 0x04 : Timer match register
		volatile U32 TMRCONTROL;		///< 0x08 : Timer control register
		volatile U32 _RESERVED[0x0D];	///< 0x0C ~ 0x3C : Reserved region
		volatile U32 TMRCLKENB;			///< 0x40 : Timer clock generation enable register
		volatile U32 TMRCLKGEN;			///< 0x44 : Timer clock generation control register
};

#define NXP2120_TMRCONTROL_LDCNT      (1 << 6)
#define NXP2120_TMRCONTROL_INTPEND    (1 << 5)
#define NXP2120_TMRCONTROL_INTENB     (1 << 4)
#define NXP2120_TMRCONTROL_RUN        (1 << 3)
#define NXP2120_TMRCONTROL_WDENB      (1 << 2)
#define NXP2120_TMRCONTROL_SELTCLK_MASK    (3 << 0)
#define NXP2120_TMRCONTROL_SELTCLK_NODIV   (3 << 0)
#define NXP2120_TMRCONTROL_SELTCLK_DIV_2   (0 << 0)
#define NXP2120_TMRCONTROL_SELTCLK_DIV_4   (1 << 0)
#define NXP2120_TMRCONTROL_SELTCLK_DIV_8   (2 << 0)
#define NXP2120_TMRCLKENB_TCLKMODE_ALWAYS  (1 << 3)
#define NXP2120_TMRCLKENB_CLKGEN_ENABLE    (1 << 2)
#define NXP2120_TMRCLKGEN_OUTCLK_DISABLE   (1 << 15)
#define NXP2120_TMRCLKGEN_CLKDIV0_MASK     (0x3F << 5)
#define NXP2120_TMRCLKGEN_CLKDIV0(x)     ((x-1) << 5)
#define NXP2120_TMRCLKGEN_CLKSRCSEL_MASK   (0x7 << 2)
#define NXP2120_TMRCLKGEN_CLKSRCSEL_PLL0   (0 << 2)
#define NXP2120_TMRCLKGEN_CLKSRCSEL_PLL1   (1 << 2)
#define NXP2120_TMRCLKGEN_OUTCLK_INVERT    (1 << 1)

/* For GPIO */
struct	NX_GPIO_RegisterSet	{
		volatile U32 GPIOxOUT;			///< 0x00	: Output Register
		volatile U32 GPIOxOUTENB;		///< 0x04	: Output Enable Register
		volatile U32 GPIOxDETMODE[2];	///< 0x08	: Event Detect Mode Register
		volatile U32 GPIOxINTENB;		///< 0x10	: Interrupt Enable Register
		volatile U32 GPIOxDET;			///< 0x14	: Event Detect Register
		volatile U32 GPIOxPAD;			///< 0x18	: PAD Status Register
		volatile U32 GPIOxPUENB;		///< 0x1C	: Pull Up Enable Register
		volatile U32 GPIOxALTFN[2];		///< 0x20	: Alternate Function Select Register
		volatile U32 __Reserved[5];		///< 0x28	:
		volatile U32 GPIOxDETENB;		///< 0x3C	: IntPend Detect Enable Register
};

/* For DMA */
struct	NX_DMA_RegisterSet {
		volatile U32	DMASRCADDR;		///< 0x00 : Source Address Register
		volatile U32	DMADSTADDR;		///< 0x04 :	Destination Address Register
		volatile U16	DMALENGTH;		///< 0x08 :	Transfer Length Register
		volatile U16	DMAREQID;		///< 0x0A : Peripheral ID Register
		volatile U32	DMAMODE;		///< 0x0C : DMA Mode Register
		volatile U32	DMASRCADDR_WB;	///< 0x10 : Source Address Write Back Register
		volatile U32	DMADSTADDR_WB;	///< 0x24 :	Destination Address Write Back Register
		volatile U16	DMALENGTH_WB;	///< 0x18 :	Transfer Length Write Back Register
		volatile U16	DMAREQID_WB;	///< 0x1A : Peripheral ID Write Back Register
		volatile U32	DMAMODE_WB;		///< 0x1C : DMA Mode Write Back Register
		volatile U32	DMACMDWAIT;		///< 0x20 : Command Wait Register
		volatile U32	DMACMDSTOP;		///< 0x24 : Command Stop Register
	//	volatile U32	DMACMDBUSY;		///< 0x28 : Command Busy Register
		volatile U32	Reserved;		///< 0x28 : Reserved
		volatile U32	DMACMDEMPTY;	///< 0x2C : Command Empty Space Register
};

#define NXP2120_DMAMODE_STOP            (1<<20)
#define NXP2120_DMAMODE_RUN             (1<<19)
#define NXP2120_DMAMODE_INTENB          (1<<18)
#define NXP2120_DMAMODE_INTPEND         (1<<17)
#define NXP2120_DMAMODE_BUSY            (1<<16)
#define NXP2120_DMAMODE_DSTNOTREQCHK    (1<<13)
#define NXP2120_DMAMODE_DSTNOTINC       (1<<12)
#define NXP2120_DMAMODE_DESTIMODE       (1<<10)
#define NXP2120_DMAMODE_DESTIMODE_IO    (1<<10)
#define NXP2120_DMAMODE_DESTIMODE_MEM   (0<<10)
#define NXP2120_DMAMODE_DSTIOSIZE_MASK  (3<<8)
#define NXP2120_DMAMODE_DSTIOSIZE_8     (0<<8)
#define NXP2120_DMAMODE_DSTIOSIZE_16    (1<<8)
#define NXP2120_DMAMODE_DSTIOSIZE_32    (2<<8)
#define NXP2120_DMAMODE_SRCNOTREQCHK    (1<<5)
#define NXP2120_DMAMODE_SRCNOTINC       (1<<4)
#define NXP2120_DMAMODE_SRCIOMODE       (1<<2)
#define NXP2120_DMAMODE_SRCIOMODE_IO    (1<<2)
#define NXP2120_DMAMODE_SRCIOMODE_MEM   (0<<2)
#define NXP2120_DMAMODE_SRCIOSIZE_MASK  (3<<0)
#define NXP2120_DMAMODE_SRCIOSIZE_8     (0<<0)
#define NXP2120_DMAMODE_SRCIOSIZE_16    (1<<0)
#define NXP2120_DMAMODE_SRCIOSIZE_32    (2<<0)
#define NXP2120_DMACMDWAIT_CMDWAIT      (1<<0)
#define NXP2120_DMACMDSTOP_CMDSTOP      (1<<0)

#define NXP2120_DMAREQ_ID_UART0_TX      0
#define NXP2120_DMAREQ_ID_UART0_RX      1
#define NXP2120_DMAREQ_ID_UART1_TX      2
#define NXP2120_DMAREQ_ID_UART1_RX      3
#define NXP2120_DMAREQ_ID_USB_EP1      12
#define NXP2120_DMAREQ_ID_USB_EP2      13
#define NXP2120_DMAREQ_ID_USB_EP3      14
#define NXP2120_DMAREQ_ID_SD0_RW       16
#define NXP2120_DMAREQ_ID_MPEG_TS      17
#define NXP2120_DMAREQ_ID_SPISSP0_TX   18
#define NXP2120_DMAREQ_ID_SPISSP0_RX   19
#define NXP2120_DMAREQ_ID_AUDIO_PCMOUT 24
#define NXP2120_DMAREQ_ID_AUDIO_SPDIF  25
#define NXP2120_DMAREQ_ID_AUDIO_PCMIN  26
#define NXP2120_DMAREQ_ID_AUDIO_MICIN  27
#define NXP2120_DMAREQ_ID_AUDIO_ADC0   28
#define NXP2120_DMAREQ_ID_AUDIO_ADC1   29
#define NXP2120_DMAREQ_ID_I2C0         33
#define NXP2120_DMAREQ_ID_I2C1         34
#define NXP2120_DMAREQ_ID_AES_READ     43
#define NXP2120_DMAREQ_ID_AES_WRITE    44
#define NXP2120_DMAREQ_ID_SPDIF_RX     45

/* For I2C */
struct	NX_I2C_RegisterSet {
		volatile U32 ICCR;					///< 0x00 : I2C Control Register
		volatile U32 ICSR;					///< 0x04 : I2C Status Register
		volatile U32 IAR;					///< 0x08 : I2C Address Register
		volatile U32 IDSR;					///< 0x0C : I2C Data Register
		volatile U32 QCNT_MAX;				///< 0x10 : I2C Quarter Period Register
		volatile U32 __Reserved0[4];		///< 0x14, 0x18, 0x1C, 0x20 : Reserved region
		volatile U32 PEND;					///< 0x24 : I2C IRQ PEND Register
		volatile U8	__Reserved1[0x100-0x28]; ///< 0x28 ~ 0xFC : Reserved region
		volatile U32 CLKENB;				///< 0x100 : Clock Enable Register.
};

#define NXP2120_ICCR_ACKGEN                 (1 << 7)
#define NXP2120_ICCR_CLKSRC_PCLK_DIV_256    (1 << 6)
#define NXP2120_ICCR_CLKSRC_PCLK_DIV_16     (0 << 6)
#define NXP2120_ICCR_IRQENB                 (1 << 5)
#define NXP2120_ICSR_ST_ENB                 (1 << 12)
#define NXP2120_ICSR_SLAVE_MATCH            (1 << 10)
#define NXP2120_ICSR_GENERAL_CALL           (1 << 9)
#define NXP2120_ICSR_SLV_RX_STOP            (1 << 8)
#define NXP2120_ICSR_MASTER                 (1 << 7)
#define NXP2120_ICSR_TX                     (1 << 6)
#define NXP2120_ICSR_BUSY                   (1 << 5)
#define NXP2120_ICSR_I2C_START              (1 << 5)
#define NXP2120_ICSR_TXRX_ENABLE            (1 << 4)
#define NXP2120_ICSR_ARBIT_FAIL             (1 << 3)
#define NXP2120_ICSR_ACK_STATUS             (1 << 0)
#define NXP2120_I2C_PEND_OP_HOLD            (1 << 1)
#define NXP2120_I2C_PEND_PEND               (1 << 0)
#define NXP2120_I2C_CLKENB_PCLKMODE         (1 << 3)

/* For AUDIO */
struct	NX_AUDIO_RegisterSet {
		volatile U16	AC97_CTRL;						///< 0x00 : AC97 Control Register
		volatile U16	AC97_CONFIG;					///< 0x02 : AC97 Configuration Register
		volatile U16	I2S_CTRL;						///< 0x04 : I2S	Control Register
		volatile U16	I2S_CONFIG;						///< 0x06 : I2S	Configuration Register
		volatile U16	AUDIO_BUFF_CTRL;				///< 0x08 : Audio Buffer Control Register
		volatile U16	AUDIO_BUFF_CONFIG;				///< 0x0A : Audio Buffer Configuration Register
		volatile U16	AUDIO_IRQ_ENA;					///< 0x0C : Audio Interrupt Enable Register
		volatile U16	AUDIO_IRQ_PEND;					///< 0x0E : Audio Interrupt Pending Register
		volatile U16	AC97_CODEC_ADDR;				///< 0x10 : AC97 Codec Address Register
		volatile U16	AC97_CODEC_WDATA;				///< 0x12 : AC97 Codec Write Data Register
		volatile U16	AC97_CODEC_RDATA;				///< 0x14 : AC97 Codec Read Data Register
		volatile U16	AUDIO_STATUS0;					///< 0x16 : Audio Status0 Register
		volatile U16	AUDIO_STATUS1;					///< 0x18 : Audio Status1 Register
		volatile U16	RESERVED_0[51];					///< 0x1A ~ 0x7E : Reserve
		volatile U16	AC97_CODEC_REG[64];				///< 0x80 ~ 0xFF : Codec Register Map
		volatile U32	__Reserved00[(0x3C0-0x100)/4];	///< 0x100 ~ 0x3BF : Reserved Region
		volatile U32	CLKENB;							///< 0x3C0 : Clock Enable Register
		volatile U32	CLKGEN[2][2];					///< 0x3C4 : Clock Generater Register 0
														///< 0x3C8 : Reserved
														///< 0x3CC : Clock Generater Register 1
														///< 0x3D0 : Reserved
};
#define NXP2120_AC97_CTRL_WARM_RST             (1 << 3)
#define NXP2120_AC97_CTRL_ACLINK_RUN           (1 << 2)
#define NXP2120_AC97_CTRL_CTRL_RST             (1 << 1)
#define NXP2120_AC97_CTRL_COLD_RST             (1 << 0)

#define NXP2120_AC97_CONFIG_ADC2_EN            (1 << 7)
#define NXP2120_AC97_CONFIG_ADC1_EN            (1 << 6)
#define NXP2120_AC97_CONFIG_MIC_EN             (1 << 5)
#define NXP2120_AC97_CONFIG_PCMIN_EN           (1 << 4)
#define NXP2120_AC97_CONFIG_SPDIF_EN           (1 << 3)
#define NXP2120_AC97_CONFIG_CLFE_EN            (1 << 2)
#define NXP2120_AC97_CONFIG_REAR_EN            (1 << 1)
#define NXP2120_AC97_CONFIG_FRONT_EN           (1 << 0)

#define NXP2120_I2S_CTRL_I2SLINK_RUN           (1 << 1)
#define NXP2120_I2S_CTRL_I2S_EN                (1 << 0)

#define NXP2120_I2S_CONFIG_IF_MODE_MASK        (3 << 6)
#define NXP2120_I2S_CONFIG_IF_MODE_I2S         (0 << 6)
#define NXP2120_I2S_CONFIG_IF_MODE_LEFTJUSTIFIED  (2 << 6)
#define NXP2120_I2S_CONFIG_IF_MODE_RIGHTJUSTIFIED  (3 << 6)
#define NXP2120_I2S_CONFIG_SYNC_PERIOD_MASK    (3 << 4)
#define NXP2120_I2S_CONFIG_SYNC_PERIOD_32FS    (0 << 4)
#define NXP2120_I2S_CONFIG_SYNC_PERIOD_48FS    (1 << 4)
#define NXP2120_I2S_CONFIG_SYNC_PERIOD_64FS    (2 << 4)
#define NXP2120_I2S_CONFIG_LOOPBACK            (1 << 3)
#define NXP2120_I2S_CONFIG_I2S_IN_EN           (1 << 2)
#define NXP2120_I2S_CONFIG_I2S_OUT_EN          (1 << 1)
#define NXP2120_I2S_CONFIG_CTRL_MODE_MASK      (1 << 0)
#define NXP2120_I2S_CONFIG_CTRL_MODE_MASTER    (0 << 0)
#define NXP2120_I2S_CONFIG_CTRL_MODE_SLAVE     (1 << 0)

#define NXP2120_AUDIO_BUFF_CTRL_ADC2BUF_EN   (1 << 5)
#define NXP2120_AUDIO_BUFF_CTRL_ADC1BUF_EN   (1 << 4)
#define NXP2120_AUDIO_BUFF_CTRL_MICBUF_EN    (1 << 3)
#define NXP2120_AUDIO_BUFF_CTRL_PCMIBUF_EN   (1 << 2)
#define NXP2120_AUDIO_BUFF_CTRL_SPDIFBUF_EN  (1 << 1)
#define NXP2120_AUDIO_BUFF_CTRL_PCMOBUF_EN   (1 << 0)

#define NXP2120_AUDIO_BUFF_CONFIG_PI_WIDTH_MASK (3 << 4)
#define NXP2120_AUDIO_BUFF_CONFIG_PI_WIDTH_16   (0 << 4)
#define NXP2120_AUDIO_BUFF_CONFIG_PI_WIDTH_18   (3 << 4)
#define NXP2120_AUDIO_BUFF_CONFIG_PI_WIDTH_20   (3 << 4)
#define NXP2120_AUDIO_BUFF_CONFIG_PI_WIDTH_24   (3 << 4)
#define NXP2120_AUDIO_BUFF_CONFIG_SP_WIDTH_MASK (3 << 4)
#define NXP2120_AUDIO_BUFF_CONFIG_SP_WIDTH_16   (0 << 4)
#define NXP2120_AUDIO_BUFF_CONFIG_SP_WIDTH_18   (3 << 4)
#define NXP2120_AUDIO_BUFF_CONFIG_SP_WIDTH_20   (3 << 4)
#define NXP2120_AUDIO_BUFF_CONFIG_PO_WIDTH_MASK (3 << 4)
#define NXP2120_AUDIO_BUFF_CONFIG_PO_WIDTH_16   (0 << 4)
#define NXP2120_AUDIO_BUFF_CONFIG_PO_WIDTH_18   (3 << 4)
#define NXP2120_AUDIO_BUFF_CONFIG_PO_WIDTH_20   (3 << 4)
#define NXP2120_AUDIO_BUFF_CONFIG_PO_WIDTH_24   (3 << 4)

#define NXP2120_AUDIO_IRQ_ENA_RDDONE_IRQ_EN     (1 << 6)
#define NXP2120_AUDIO_IRQ_ENA_ADC2OVR_IRQ_EN    (1 << 5)
#define NXP2120_AUDIO_IRQ_ENA_ADC1OVR_IRQ_EN    (1 << 4)
#define NXP2120_AUDIO_IRQ_ENA_MICOVR_IRQ_EN     (1 << 3)
#define NXP2120_AUDIO_IRQ_ENA_PIOVER_IRQ_EN     (1 << 2)
#define NXP2120_AUDIO_IRQ_ENA_SPUDR_IRQ_EN      (1 << 1)
#define NXP2120_AUDIO_IRQ_ENA_POUDR_IRQ_EN      (1 << 0)

#define NXP2120_AUDIO_IRQ_PEND_RDDONE_PEND      (1 << 6)
#define NXP2120_AUDIO_IRQ_PEND_ADC2OVR_PEND     (1 << 5)
#define NXP2120_AUDIO_IRQ_PEND_ADC1OVR_PEND     (1 << 4)
#define NXP2120_AUDIO_IRQ_PEND_MICOVR_PEND      (1 << 3)
#define NXP2120_AUDIO_IRQ_PEND_PIOVR_PEND       (1 << 2)
#define NXP2120_AUDIO_IRQ_PEND_SPUDR_PEND       (1 << 1)
#define NXP2120_AUDIO_IRQ_PEND_POUDR_PEND       (1 << 0)

#define NXP2120_AUDIO_STATUS0_CODEC_BUSY        (1 << 11)
#define NXP2120_AUDIO_STATUS0_CODEC_RDDONE      (1 << 10)
#define NXP2120_AUDIO_STATUS0_CODEC_WRDONE      (1 << 9)
#define NXP2120_AUDIO_STATUS0_CODEC_RDY         (1 << 8)
#define NXP2120_AUDIO_STATUS0_AC_FSM_MASK       (7 << 3)
#define NXP2120_AUDIO_STATUS0_I2S_FSM_MASK      (7 << 0)

#define NXP2120_AUDIO_STATUS1_ADC2BUF_RDY       (1 << 5)
#define NXP2120_AUDIO_STATUS1_ADC1BUF_RDY       (1 << 4)
#define NXP2120_AUDIO_STATUS1_MICBUF_RDY        (1 << 3)
#define NXP2120_AUDIO_STATUS1_PIBUF_RDY         (1 << 2)
#define NXP2120_AUDIO_STATUS1_SPBUF_RDY         (1 << 1)
#define NXP2120_AUDIO_STATUS1_POBUF_RDY         (1 << 0)

#define NXP2120_AUDIO_CLKENB_PCLKMODE           (1 << 3)
#define NXP2120_AUDIO_CLKENB_CLKGENENB          (1 << 2)

#define NXP2120_AUDIO_CLKGEN0_CLKDIV0_MASK      (0xFF << 5)
#define NXP2120_AUDIO_CLKGEN0_CLKSRCSEL0_MASK   (7 << 2)
#define NXP2120_AUDIO_CLKGEN0_OUTCLKNV0         (1 << 1)

#define NXP2120_AUDIO_CLKGEN1_OUTCLKENB1        (1 << 15)
#define NXP2120_AUDIO_CLKGEN1_CLKDIV1_MASK      (0xFF << 5)
#define NXP2120_AUDIO_CLKGEN1_CLKSRCSEL1_MASK   (7 << 2)
#define NXP2120_AUDIO_CLKGEN1_OUTCLKNV1         (1 << 1)


struct	NX_MCUS_RegisterSet {
	volatile U32 MEMBW			;	///< 00h			: Memory Bus Width Register
	volatile U32 MEMTIMEACS[2]	;	///< 04h, 08h		: Memory Timing for tACS Register
	volatile U32 MEMTIMECOS[2]	;	///< 0Ch, 10h		: Memory Timing for tCOS Register
	volatile U32 MEMTIMEACC[3]	;	///< 14h, 18h, 1Ch	: Memory Timing for tACC Register
	volatile U32 MEMTIMESACC[3]	;	///< 20h, 24h, 28h	: Memory Timing for tSACC Register
	volatile U32 MEMTIMEWACC[3]	;	///< 2Ch, 30h, 34h	: Memory Timing for tWACC Register
	volatile U32 MEMTIMECOH[2]	;	///< 38h, 3Ch		: Memory Timing for tCOH Register
	volatile U32 MEMTIMECAH[2]	;	///< 40h, 44h		: Memory Timing for tCAH Register
	volatile U32 MEMBURST		;	///< 48h			: Memory Burst Control Register
	volatile U32 __Reserved1[1]	;	///< 4Ch			: Reserved for future use
	volatile U32 MEMWAIT		;	///< 50h			: Memory Wait Control Register
	volatile U32 IDEDMATIMEOUT	;	///< 54h			: DMA Time-out Register
	volatile U32 IDEDMACTRL		;	///< 58h			: DMA Control Register
	volatile U32 IDEDMAPOL		;	///< 5Ch			: DMA Polarity Register
	volatile U32 IDEDMATIME0	;	///< 60h			: DMA Timing 0 Register
	volatile U32 IDEDMATIME1	;	///< 64h			: DMA Timing 1 Register
	volatile U32 IDEDMATIME2	;	///< 68h			: DMA Timing 2 Register
	volatile U32 IDEDMATIME3	;	///< 6Ch			: DMA Timing 3 Register
	volatile U32 IDEDMARST		;	///< 70h			: DMA Reset Register
	volatile U32 IDEDMATIME4	;	///< 74h			: DMA Timing 4 Register
	volatile U32 __Reserved2[1]	;	///< 78h			: Reserved for future use.
	volatile U32 NFCONTROL		;	///< 7Ch			: Nand Flash Control Register
	volatile U32 NFECC[7]		;	///< 80h ~ 98h		: Nand Flash ECC 0 ~ 6 Register
	volatile U32 NFORGECC[7]	;	///< 9Ch ~ B4h		: Nand Flash Origin ECC 0 ~ 6 Register
	volatile U32 NFCNT			;	///< B8h			: Nand Flash Data Count Register
	volatile U32 NFECCSTATUS	;	///< BCh			: Nand Flash ECC Status Register
	volatile U32 NFSYNDROME[8]	;	///< C0h ~ DCh		: Nand Flash ECC Syndrome Value 0 ~ 7 Register
	volatile U32 __Reserved3[3]	;	///< E0h ~ E8h		: Reserved for future use.
	volatile U32 NFTACS			;	///< ECh			: Nand Timing for tACS Register
	volatile U32 NFTCOS			;	///< F0h			: Nand Timing for tCOS Register
	volatile U32 NFTACCL		;	///< F4h			: Nand Timing for tACC Register
	volatile U32 NFTACCH		;	///< F8h			: Nand Timing for tACC Register
	volatile U32 NFTOCH			;	///< FCh			: Nand Timing for tOCH Register
	volatile U32 NFTCAH			;	///< 00h			: Nand Timing for tCAH Register
};

/*
 * NXP2120 NAND register definitions
 */
#define NXP2120_NFREG(x)      ((x) + 0xC001587C) // PHY_BASEADDR_MCUS_MODULE + 0x7C
#define NXP2120_NFCONTROL      NXP2120_NFREG(0x00)
#define NXP2120_NFECC0         NXP2120_NFREG(0x04)
#define NXP2120_NFECC1         NXP2120_NFREG(0x08)
#define NXP2120_NFECC2         NXP2120_NFREG(0x0C)
#define NXP2120_NFECC3         NXP2120_NFREG(0x10)
#define NXP2120_NFECC4         NXP2120_NFREG(0x14)
#define NXP2120_NFECC5         NXP2120_NFREG(0x18)
#define NXP2120_NFECC6         NXP2120_NFREG(0x1C)
#define NXP2120_NFORGECC0      NXP2120_NFREG(0x20)
#define NXP2120_NFORGECC1      NXP2120_NFREG(0x24)
#define NXP2120_NFORGECC2      NXP2120_NFREG(0x28)
#define NXP2120_NFORGECC3      NXP2120_NFREG(0x2C)
#define NXP2120_NFORGECC4      NXP2120_NFREG(0x30)
#define NXP2120_NFORGECC5      NXP2120_NFREG(0x34)
#define NXP2120_NFORGECC6      NXP2120_NFREG(0x38)
#define NXP2120_NFDATACNT      NXP2120_NFREG(0x3C)
#define NXP2120_NFECCSTATUS    NXP2120_NFREG(0x40)
#define NXP2120_NFSYNDROME0    NXP2120_NFREG(0x44)
#define NXP2120_NFSYNDROME1    NXP2120_NFREG(0x48)
#define NXP2120_NFSYNDROME2    NXP2120_NFREG(0x4C)
#define NXP2120_NFSYNDROME3    NXP2120_NFREG(0x50)
#define NXP2120_NFSYNDROME4    NXP2120_NFREG(0x54)
#define NXP2120_NFSYNDROME5    NXP2120_NFREG(0x58)
#define NXP2120_NFSYNDROME6    NXP2120_NFREG(0x5C)
#define NXP2120_NFSYNDROME7    NXP2120_NFREG(0x60)
#define NXP2120_NFDATA         (0x2C000000)
#define NXP2120_NFCMD          (0x2C000010)
#define NXP2120_NFADDR         (0x2C000018)

#define NXP2120_NFCONT_NCSENB				(1 << 31)
#define NXP2120_NFCONT_NFECCAUTORSTENB		(1 << 30)
#define NXP2120_NFCONT_NFECCMODE_MASK		(3 << 28)
#define NXP2120_NFCONT_NFECCMODE_4BIT		(0 << 28)
#define NXP2120_NFCONT_NFECCMODE_8BIT		(1 << 28)
#define NXP2120_NFCONT_NFECCMODE_16BIT		(2 << 28)
#define NXP2120_NFCONT_IRQPEND				(1 << 15)
#define NXP2120_NFCONT_IRQCLEAR				(1 << 15)
#define NXP2120_NFCONT_ECCRST				(1 << 11)
#define NXP2120_NFCONT_RNB					(1 << 9)
#define NXP2120_NFCONT_IRQENB				(1 << 8)
#define NXP2120_NFCONT_NFBOOTENB			(1 << 5)
#define NXP2120_NFCONT_NFTYPE_MASK			(3 << 3)
#define NXP2120_NFCONT_NFTYPE_SB_3ADDR		(0 << 3)
#define NXP2120_NFCONT_NFTYPE_SB_4ADDR		(1 << 3)
#define NXP2120_NFCONT_NFTYPE_LB_4ADDR		(2 << 3)
#define NXP2120_NFCONT_NFTYPE_LB_5ADDR		(3 << 3)
#define NXP2120_NFCONT_NFBANK_MASK			(3 << 0)
#define NXP2120_NFCONT_NFBANK_NCS0			(0 << 0)
#define NXP2120_NFCONT_NFBANK_NCS1			(1 << 0)
#define NXP2120_NFCONT_NFBANK_NCS2			(2 << 0)
#define NXP2120_NFCONT_NFBANK_NCS3			(3 << 0)

#define NXP2120_NFECCSTATUS_NFCHECKERROR (1 << 2)
#define NXP2120_NFECCSTATUS_NFECCDECDONE (1 << 1)
#define NXP2120_NFECCSTATUS_NFECCENCDONE (1 << 0)

#endif

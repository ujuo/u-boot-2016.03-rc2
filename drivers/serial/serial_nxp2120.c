/*
 * (C) Copyright 2011 MOSTiTECH co., ltd.
 * All right reserved by Seungwoo Kim <ksw@mostitech.com>
 * Most function rewritten for not using nexell's <prototype> function set.
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
#include <serial.h>

/* nexell soc headers */
#include <nxp2120.h>

DECLARE_GLOBAL_DATA_PTR;

#ifdef CONFIG_SERIAL1
#define PHYS_BASE (PHY_BASEADDR_UART0_MODULE)
#elif defined(CONFIG_SERIAL2)
#define PHYS_BASE (PHY_BASEADDR_UART1_MODULE)
#else
#error "Bad: you didn't configure serial ..."
#endif

extern u_int cpu_get_clock_hz(int clk);

//static int	g_inituart = 0;
//int	g_inituart = 0;
/*------------------------------------------------------------------------------
 * u-boot serial interface
 */
/*
#define DBG_ADDR	0x80001004
void dbg_halt(int addr,int val)
{
	*((int*)(DBG_ADDR+addr*4)) = val;
}
*/

int nxp2120_serial_init(void)
{
	int baudrate = CONFIG_BAUDRATE;
	int src_clk,div,mod;

	struct NX_UART_RegisterSet *uart = (struct NX_UART_RegisterSet*)PHYS_BASE;
	struct NX_GPIO_RegisterSet *gpiob = (struct NX_GPIO_RegisterSet*)PHY_BASEADDR_GPIO_B_MODULE;
//	struct NX_GPIO_RegisterSet *gpioc = (struct NX_GPIO_RegisterSet*)PHY_BASEADDR_GPIO_C_MODULE;
//	dbg_halt(0,(int)&g_inituart);
	
//	if(g_inituart)
//		return 0;
//	if(g_inituart == 0x54345434)
//		return 0;
	
//	dbg_halt(2,0x33);
	/* Maybe setup GPIO first */
//	gpioc->GPIOxALTFN[1] |= 0x10;
//	gpioc->GPIOxOUTENB &= 0xFFFEFFFF;
	gpiob->GPIOxALTFN[0] &= ~((0x03 << 2) | 0x03);
	gpiob->GPIOxALTFN[0] |= (0x01 << 2) | 0x01;
//	gpiob->GPIOxALTFN[0] |=  0x01;
    /* set LCON to 8bit, 1stop bit No parity configuration */
    uart->LCON = NXP2120_LCON_PARITY_NONE | NXP2120_LCON_ONE_STOP_BIT | NXP2120_LCON_WORDLEN_8BIT;
    /* Set UCON to int/polling mode RX/TX */
    uart->UCON = NXP2120_UCON_TRANS_IRQ_POLLING | NXP2120_UCON_RECV_IRQ_POLLING;
	/* determin using FIFO or not. */
#ifdef CONFIG_SERIAL_USE_FIFO
    uart->FCON |= NXP2120_FCON_FIFO_ENABLE | NXP2120_FCON_TX_FIFO_RESET | NXP2120_FCON_RX_FIFO_RESET;
#else
    uart->FCON &= ~NXP2120_FCON_FIFO_ENABLE;
#endif
    /* set Clock Enable Register */
    uart->CLKENB |= NXP2120_UART_CLKENB_PCLKMODE | NXP2120_UART_CLKENB_CLKGENB;
    /* set baudrate, by base register set close to 7.3728MHz */
    src_clk = cpu_get_clock_hz(1); /* PLL1 */
    div = src_clk / 7372800;
    mod = src_clk % 7372800;
    if (mod < 368640) { /* Then it can be usable as below +5% value... */
        div -= 1; /* As set value, it would be 1 less integer */
    } else
    if (mod > 7004160) { /* Then it can be usable as above -5% value...*/
        div = div + 1 - 1;
    } else {
        /* It would be some problem that Serial Port would not work properly. */
    }

    uart->CLKGEN = (div << 5) | NXP2120_UART_CLKGEN_SRC_PLL1;
    /* Now baudrate setting */
    uart->BRD = (7372800 / (baudrate * 16)) - 1;

#ifndef CONFIG_SERIAL2
	uart = (struct NX_UART_RegisterSet*)PHY_BASEADDR_UART1_MODULE;

    /* set LCON to 8bit, 1stop bit No parity configuration */
    uart->LCON = NXP2120_LCON_PARITY_NONE | NXP2120_LCON_ONE_STOP_BIT | NXP2120_LCON_WORDLEN_8BIT;
    /* Set UCON to int/polling mode RX/TX */
    uart->UCON = NXP2120_UCON_TRANS_IRQ_POLLING | NXP2120_UCON_RECV_IRQ_POLLING;
	/* determin using FIFO or not. */
    uart->FCON |= NXP2120_FCON_FIFO_ENABLE | NXP2120_FCON_TX_FIFO_RESET | NXP2120_FCON_RX_FIFO_RESET;

    uart->CLKGEN = (div << 5) | NXP2120_UART_CLKGEN_SRC_PLL1;

    /* set Clock Enable Register */
    uart->CLKENB |= NXP2120_UART_CLKENB_PCLKMODE | NXP2120_UART_CLKENB_CLKGENB;

    uart->BRD = (7372800 / (460800 * 16)) - 1;
#endif

    /* Uart initialize done. */
//	g_inituart = 0x54345434;
//	g_inituart = 1;
  	
	return 0;
}

void nxp2120_serial_putc(const char c)
{
    struct NX_UART_RegisterSet *uart = (struct NX_UART_RegisterSet*)PHYS_BASE;

#ifdef CONFIG_MODEM_SUPPORT
	if (be_quiet)
		return;
#endif

#ifdef CONFIG_SERIAL_USE_FIFO
    while ((uart->FSTATUS & NXP2120_FSTATUS_TX_FIFO_FULL)) ;
#else
	/* wait for room in the tx FIFO */
	while (!(uart->TRSTATUS & NXP2120_TRSTATUS_TX_BUF_EMPTY)) ;
#endif

#ifdef CONFIG_HWFLOW
	/* Wait for CTS up */
	while (hwflow && !(uart->MSTATUS & 0x1));
#endif

	uart->THB = c;

	/* If \n, also do \r */
	if (c == '\n')
		serial_putc('\r');
}

int nxp2120_serial_tstc(void)
{
	struct NX_UART_RegisterSet *uart = (struct NX_UART_RegisterSet*)PHYS_BASE;

#ifdef CONFIG_SERIAL_USE_FIFO
    return ((uart->FSTATUS & NXP2120_FSTATUS_RX_FIFO_COUNT) != 0);
#else
	return (uart->TRSTATUS & NXP2120_TRSTATUS_RX_BUF_READY) != 0;
#endif
}

int nxp2120_serial_getc(void)
{
	struct NX_UART_RegisterSet *uart = (struct NX_UART_RegisterSet*)PHYS_BASE;

	/* wait for character to arrive */
#ifdef CONFIG_SERIAL_USE_FIFO
    while ((uart->FSTATUS & NXP2120_FSTATUS_RX_FIFO_COUNT) == 0) ;
#else
	while ((uart->TRSTATUS & NXP2120_TRSTATUS_RX_BUF_READY) == 0) ;
#endif

	return uart->RHB & 0xff;
}

void nxp2120_serial_puts(const char *s)
{
    while (*s) {
		serial_putc(*s++);
	}
}

void nxp2120_serial_setbrg(void)
{
	return;
}

static struct serial_device nxp2120_serial_drv = {
	.name	= "nxp2120_serial",
	.start	= nxp2120_serial_init,
	.stop	= NULL,
	.setbrg	= nxp2120_serial_setbrg,
	.putc	= nxp2120_serial_putc,
	.puts	= default_serial_puts,
	.getc	= nxp2120_serial_getc,
	.tstc	= nxp2120_serial_tstc,
};



void nxp2120_serial_initialize(void)
{
	serial_register(&nxp2120_serial_drv);
}

__weak struct serial_device *default_serial_console(void)
{
	return &nxp2120_serial_drv;
}


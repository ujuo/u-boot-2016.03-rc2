/*
 *
 * (c) 2007 Pengutronix, Sascha Hauer <s.hauer@pengutronix.de>
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

#include <asm/arch/regs.h>


.globl lowlevel_init
lowlevel_init:
    /* Not many thing to do with lowlevel init, 
       because CPU CLOCK, and DRAM init already done
       from NANDBOOT code. */

   	/* Peri port setup */
    ldr r0, =0xC0000000
    orr r0, r0, #0x15
    mcr p15,0,r0,c15,c2,4       /* 256M(0xC0000000-0xCfffffff) */

#if 0 /* Test routine to turn on LED */
    ldr r6,=0xC000A000
    ldr r7,=0x00000000
    str r7,[r6, #0x20] /* setup GPIOA for GPIO */
    ldr r7,=0x0000000F
    str r7,[r6, #0x04] /* setup GPIOA Pad0~3 for output */
    ldr r7,=0x00000001
    str r7,[r6, #0x00] /* set GPIOA Pad0 for 1 */
#endif
    mov pc,lr /* go back to caller */


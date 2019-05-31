/*
 * Copyright (C) 2019 I4VINE
 * Author: Juyoung Ryu <jyryu@i4vine.com>	
 
 * (C) Copyright 2010
 * jung hyun kim, Nexell Co, <jhkim@nexell.co.kr>
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
#include <linux/version.h>
#define	PB_UART_FIFO_EMPTY        (0x02)
#define	PHY_BASEADDR_UART0			(0xC0016000)
#define PB_UART_PHYS_BASE			(0xC0016000)
#define PB_UART_VIRT_BASE			(0xF0016000)
#define PB_UART_THB					(0x10)
#define PB_UART_TRSTATUS			(0x08)
#define PB_UART_FIFO_EMPTY			(0x02)
/*#include <mach/platform.h>*/

		.macro	addruart,rp, rv, tmp

	/* If we move the adress using MMU, use this. */
		ldr	\rp, = PB_UART_PHYS_BASE		@ physical address
		ldr	\rv, = PB_UART_VIRT_BASE 		@ virtual address

		.endm

		.macro  senduart,rd,rx
		strb	\rd, [\rx, #PB_UART_THB]
		.endm

		.macro  busyuart,rd,rx
		.endm

		.macro  waituart,rd,rx
1001:
		ldr	\rd, [\rx, #PB_UART_TRSTATUS]
		tst	\rd, #PB_UART_FIFO_EMPTY
		beq	1001b
		.endm


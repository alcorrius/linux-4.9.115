/*
 * Copyright (C) 2019 I4VINE
 * Author: Juyoung Ryu <jyryu@i4vine.com>	
 *
 * Copyright (c) 2011 STCube co., ltd.
 * All right reserved by Seungwoo Kim <ksw@stcube.com>
 * Add direct register accessing code, and DMA driven IRQ code...
 *
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
#include <linux/device.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/seqlock.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial_reg.h>
#include <linux/serial.h>
#include <linux/version.h>
#include <linux/dma-mapping.h>
#include <asm/irq.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <asm/io.h>

#include "serial_nexell.h"

/* nexell soc headers */
//#include <mach/nx_soc.h>
//#include <mach/platform.h>
//#include <mach/devices.h>
//#include <mach/soc.h>


#define PHY_BASEADDR_UART0			(0xC0016000)
#define PHY_BASEADDR_UART1			(0xC0016080)
#define	CFG_UART_DEBUG_CLKDIV		(CFG_SYS_PLL1_FREQ/7372800)
#define UART_DEV_NAME	"nx-uart"

#define	IRQ_PHY_UART0		 	10
#define	IRQ_PHY_UART1		 	34
/* Debugging stuff */
#if (0)
#define DBGOUT(msg...)		{ printk(KERN_INFO "uart: " msg); }
#else
#define DBGOUT(msg...)		do {} while (0)
#endif

#if (0)
#define DBGINTR(msg...)		printk(msg)
#else
#define DBGINTR(msg...)		do {} while (0)
#endif

/* This is OLD Nexell implementation flag. Deafult is 0 */
#define OLD_IMPLEMENTATION  0
/* Using DMA for BUSY UART RX/TX */
#ifndef CONFIG_SERIAL_NXP2120_UART_PORT0_RXDMA
# define USE_DMA_CHAN0   0
#else
# define USE_DMA_CHAN0   1
#endif

#ifndef CONFIG_SERIAL_NXP2120L_UART_PORT1_RXDMA
# define USE_DMA_CHAN1   0
#else
# define USE_DMA_CHAN1   1
#endif
#define DMA_BUFFER_SIZE 32768
#define DMA_CHAN0_CHANNEL   4
#define DMA_CHAN1_CHANNEL   5
#define IRQ_ALWAYS          0

/* Register bit definition */
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

#define NXP2120_UCON_TX_INT_LEVEL       (1<<9)
#define NXP2120_UCON_TX_INT_PULSE       (0<<9)
#define NXP2120_UCON_RX_INT_LEVEL       (1<<8)
#define NXP2120_UCON_RX_INT_PULSE       (1<<9)
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

#define NXP2120_INTCON_MENB            (1<<7)
#define NXP2120_INTCON_ERRENB          (1<<6)
#define NXP2120_INTCON_RXENB           (1<<5)
#define NXP2120_INTCON_TXENB           (1<<4)
#define NXP2120_INTCON_MPEND           (1<<3)
#define NXP2120_INTCON_ERRPEND         (1<<2)
#define NXP2120_INTCON_RXPEND          (1<<1)
#define NXP2120_INTCON_TXPEND          (1<<0)

#define NXP2120_UART_CLKENB_PCLKMODE   (1<<3)
#define NXP2120_UART_CLKENB_CLKGENB    (1<<2)
#define NXP2120_UART_CLKENB_RESERVED   (3<<0)

#define NXP2120_UART_CLKGEN_SRC_SEL_MASK (0x07 << 2)
#define NXP2120_UART_CLKGEN_SRC_PLL0   (0x0 << 2)
#define NXP2120_UART_CLKGEN_SRC_PLL1   (0x1 << 2)

/* DMA REGISTER DMAMODE bit */
#define NXP2120_DMAMODE_STOP           (1 << 20)
#define NXP2120_DMAMODE_RUN            (1 << 19)
/*
 * 	Uart macros & functions
 */
#define	PORT_NEXELL			76
#define	UART_TERMINAL		"ttyS"
#define UART_BAUDRATE		115200

#define	UART_TXD_INTNUM		0	/* Tx interrupt bit */
#define	UART_RXD_INTNUM		1	/* Rx interrupt bit */
#define	UART_ERR_INTNUM		2	/* Error interrupt bit */
#define	UART_MOD_INTNUM		3	/* Modem interrupt bit */

#define UART_TXD_LEVEL		4	/* empty */
#define UART_RXD_LEVEL		12	/* 12byte */

#define UART_RXD_DUMMY		(0x10000000)

#define UART_ERR_OVERRUN	(1<<0)	/* overrun error */
#define UART_ERR_PARITY		(1<<1)	/* parity error */
#define UART_ERR_FRAME		(1<<2)	/* frame error */
#define UART_ERR_BREAK		(1<<3)	/* break error */

#define UART_INT_PEND		(0x18)

#define	UART_WAIT_READY		{ volatile int cnt; for (cnt=0; cnt <0xffff; cnt++); }

/* uart ops functions */
static struct uart_ops nx_uart_ops;

static unsigned int nx_uart_ops_tx_empty(struct uart_port *port);
static void nx_uart_ops_set_mctrl(struct uart_port *port, unsigned int mctrl);
static unsigned int nx_uart_ops_get_mctrl(struct uart_port *port);
static void nx_uart_ops_stop_tx(struct uart_port *port);
static void nx_uart_ops_start_tx(struct uart_port *port);
static void nx_uart_ops_stop_rx(struct uart_port *port);
static void nx_uart_ops_enable_ms(struct uart_port *port);
static void nx_uart_ops_break_ctl(struct uart_port *port, int break_state);
static int nx_uart_ops_startup(struct uart_port *port);
static void nx_uart_ops_shutdown(struct uart_port *port);
static void nx_uart_ops_set_termios(struct uart_port *port, struct ktermios *termios, struct ktermios *old);
static const char *nx_uart_ops_type(struct uart_port *port);
static void nx_uart_ops_release_port(struct uart_port *port);
static int nx_uart_ops_request_port(struct uart_port *port);
static void nx_uart_ops_config_port(struct uart_port *port, int flags);
static int nx_uart_ops_verify_port(struct uart_port *port, struct serial_struct *ser);

/* interrupt handler */
static irqreturn_t nx_uart_irq_handler(int irq, void *dev_id);



/* hrtimer routines for DMA/HRTIMER */
static int timer_init = 0;
static enum hrtimer_restart hrtimer_action(struct hrtimer *);
static struct hrtimer hrtimer;


struct nx_uart_port {
	struct uart_port           port;	/* Note : First type must be 'struct uart_port' */
	char                       *name;
	struct NX_UART_RegisterSet *regs;
	struct	NX_DMA_RegisterSet  *dma_regs;
	int	                        count;	/* open count */
	//struct work_struct         work;   /* work struct to work */
	struct dma_trans         * dma_tr;
	//int                        to_change;  /* From wq -> IRQ, change buffer signal */
	char                       *last_buf;
	unsigned int                last_addr;
	int                         buf_index;  /* What channel is currently running? */
	int                         dma_run;
	int                         buf_changed;
	unsigned int                buf_base[2];
	unsigned int                dma_phys[2];
	unsigned int                buf_size;
	int							zero_cnt;
	spinlock_t 			        lock;
};

/*
 * 	uart port info
 */
#if 1
#define NXP2120_NR_UARTS	2
static int  uart_count;
static struct uart_port ports[NXP2120_NR_UARTS];

#else
static struct nx_uart_port nx_ports[] = {
//#if defined(CONFIG_SERIAL_NXP2120_UART_PORT0)
	{
		.name = "UART0",
		.port = {
			.type 		= PORT_NEXELL,
			.iotype 	= UPIO_MEM,
			//.membase 	= (u_char __iomem *)ioremap_nocache(PHY_BASEADDR_UART0,0x80),
			.mapbase 	= PHY_BASEADDR_UART0,
			.irq 		= IRQ_PHY_UART0,
			.uartclk 	= CFG_UART_DEBUG_CLKFREQ,
			.fifosize 	= 16,
			.flags 		= UPF_BOOT_AUTOCONF,
			.ops 		= &nx_uart_ops,
			.line 		= 0,
		},
	//	.regs = (struct NX_UART_RegisterSet *)ioremap_nocache(PHY_BASEADDR_UART0,0x80),
	//	.dma_regs = (struct NX_DMA_RegisterSet *)ioremap_nocache(PHY_BASEADDR_DMA_MODULE + (OFFSET_OF_DMA_MODULE * DMA_CHAN0_CHANNEL),0x80),
	},
//#endif
//#if defined(CONFIG_SERIAL_NXP2120_UART_PORT1)
	{
		.name = "UART1",
		.port = {
			.type 		= PORT_NEXELL,
			.iotype 	= UPIO_MEM,
			//.membase 	= (u_char __iomem *)ioremap_nocache(PHY_BASEADDR_UART1,0x80),
			.mapbase 	= PHY_BASEADDR_UART1,
			.irq 		= IRQ_PHY_UART1,
			.uartclk 	= CFG_UART_DEBUG_CLKFREQ,
			.fifosize 	= 16,
			.flags 		= UPF_BOOT_AUTOCONF,
			.ops 		= &nx_uart_ops,
			.line 		= 1,
		},
	//	.regs = (struct NX_UART_RegisterSet *)ioremap_nocache(PHY_BASEADDR_UART1,0x80),
	//	.dma_regs = (struct NX_DMA_RegisterSet *)ioremap_nocache(PHY_BASEADDR_DMA_MODULE + (OFFSET_OF_DMA_MODULE * DMA_CHAN1_CHANNEL),0x80),
	},
//#endif


};
#endif





#if (USE_DMA_CHAN0 || USE_DMA_CHAN1)
//static struct workqueue_struct *nxp2120_serial_wq;

//static void nxp2120_serial_work_func(struct work_struct *work)
//{
//    struct nx_uart_port  *uart = container_of(work, struct nx_uart_port, work);
//}
static ktime_t time;
//static int		hrcnt = 0;

static enum hrtimer_restart hrtimer_action(struct hrtimer *timer)
{
    struct nx_uart_port *port;
    unsigned int cur_addr;
    int i, len;
    char *p;
    struct tty_struct *tty;
	unsigned long flags;


	//if (++hrcnt > 400)
	//{
	//	printk("hrcnt = %d\n", hrcnt);
	//	hrcnt = 0;
	//}
#if (USE_DMA_CHAN0)

	if (port->dma_run)
	{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
		tty = port->port.state->port.tty;
#else
		tty = port->port.info->port.tty;
#endif

	    /* Check out PORT1 and DMA channel 0 */
	    port = (struct nx_uart_port *)&nx_ports[0];
	    cur_addr = port->dma_regs->DMADSTADDR;
	    len = cur_addr - port->last_addr;
	    if (len > 0) {
	        p = port->last_buf;
	        for (i=0; i<len; i++) {
	            ch = *p++;
	            uart_insert_char(&port->port, 0, UART_LSR_OE, ch, 0);
	        }
	        port->last_buf += len;
	        port->last_addr = cur_addr;
	        tty_flip_buffer_push(tty);
	    }
	}
#endif
#if (USE_DMA_CHAN1)
    /* Check out PORT1 and DMA channel 1 */
    port = (struct nx_uart_port *)&nx_ports[1];

	if (port->dma_run)
	{
	    char ch;
	    int bytes, cp;
	    int regs,err;
	    unsigned long dmode;

		if (!(port->dma_regs->DMAMODE & (1 << 19)))
			printk("[%X]", port->dma_regs->DMAMODE);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
		tty = port->port.state->port.tty;
#else
		tty = port->port.info->port.tty;
#endif

		spin_lock_irqsave(&port->lock, flags);

	    cur_addr = port->dma_regs->DMADSTADDR;
	    regs = port->regs->FSTATUS;
		cp = 0;

		if (port->buf_changed) {
		    int index;
		    index = (port->buf_index == 0) ? 1: 0;
		    cp = port->buf_changed - port->dma_phys[index];
		    port->buf_changed = 0;
		    p = port->buf_base[index] + cp;
		    for (i=cp; i<DMA_BUFFER_SIZE; i++) {
		        ch = *p++;
		        uart_insert_char(&port->port, 0, UART_LSR_OE, ch, 0);
		   	}
		}
		len = cur_addr - port->last_addr;
	    if (len > 0) {
	    	char ch;
			//printk("port->last_buf = %x len=%d cur_addr=%x port->last_addr=%x dma_regs->DMALENGTH=%d bytes=%d\n", port->last_buf, len, cur_addr, port->last_addr, port->dma_regs->DMALENGTH, bytes);
	        p = port->last_buf;
	        for (i=0; i<len; i++) {
	            ch = *p++;
	            uart_insert_char(&port->port, 0, UART_LSR_OE, ch, 0);
	        }
	        port->last_buf += len;
	        port->last_addr = cur_addr;

	    	port->zero_cnt = 0;
	    }
	    else
	    	port->zero_cnt++;
#if 0
		bytes = (regs & NXP2120_FSTATUS_RX_FIFO_FULL) == 0 ? (regs & 0x0F) : 16;
	    if (port->zero_cnt > 1 && len == 0 && bytes > 0) {
		    dmode = port->dma_regs->DMAMODE;
		    dmode |= 1 << 20;
		    port->dma_regs->DMAMODE = dmode;

			while((port->dma_regs->DMAMODE & (1<<16)) && (port->dma_regs->DMAMODE & (1<<19)))	// DMA BUSY && RUN
				;

		    dmode = port->dma_regs->DMAMODE;
			dmode &= ~(1<<20);
		    port->dma_regs->DMAMODE = dmode;

		    dmode = port->regs->UCON;
		    dmode &= ~NXP2120_UCON_RECV_MODE_MASK;
		    dmode |= NXP2120_UCON_RECV_IRQ_POLLING;
		    port->regs->UCON = dmode;

		    for (i = 0 ; (port->regs->FSTATUS&0x0F) != 0 ; i++)	{
		        err = port->regs->ESTATUS;
		        ch = port->regs->RHB;
		        //printk("%03d", ch);
		        uart_insert_char(&port->port, 0, UART_LSR_OE, ch, 0);
		    }
		    dmode = port->regs->UCON;
		    dmode &= ~NXP2120_UCON_RECV_MODE_MASK;
		    dmode |= NXP2120_UCON_RECV_DMA_REQ;
		    port->regs->UCON = dmode;

		    dmode = port->dma_regs->DMAMODE;
		    dmode &= ~(1<<20);
		    dmode |= 1 << 19;
		    port->dma_regs->DMAMODE = dmode;

		    port->zero_cnt = 0;
		}
#endif
	    if ((bytes > 0) || (len > 0) || (cp > 0)) {
	        //printk("push %d bytes\n", bytes + len + cp);
	    }
	    tty_flip_buffer_push(tty);
		spin_unlock_irqrestore(&port->lock, flags);
	}
#endif

    hrtimer_start(timer, time, HRTIMER_MODE_REL);
    return HRTIMER_NORESTART;
}

static irqreturn_t uart_dma_irq_handler(int irq, void *dev_id)
{
    struct nx_uart_port *port = (struct nx_uart_port *)dev_id;
    struct dma_trans 	*tr  = port->dma_tr;
	unsigned long flags;
	unsigned long dmode;

    if (port->dma_run) {
        int len;
        int i;
        char ch;

		//printk("[%X]", port->dma_regs->DMAMODE);

		soc_dma_trans_stop(tr);
		spin_lock_irqsave(&port->lock, flags);

        if (port->buf_index) {
            port->buf_index = 0;
        } else {
            port->buf_index = 1;
        }
        //printk("enter uart_dma_irq_handler port->buf_index=%d \n", port->buf_index);
		//printk("UART : ESTATUS=%X UCON=%X , DMA DMAMODE=%X DMALENGTH=%X\n", port->regs->ESTATUS & 0x0F, port->regs->UCON, port->dma_regs->DMAMODE, port->dma_regs->DMALENGTH);
        /* Switch the buffers */
        tr->io2m.dstbase = port->dma_phys[port->buf_index];
        port->buf_changed = port->last_addr;
        port->last_addr = tr->io2m.dstbase;
        port->last_buf  = port->buf_base[port->buf_index];
        port->zero_cnt  = 0;
        //dmode = port->dma_regs->DMAMODE;
        //dmode |= 1 << 17;
        //port->dma_regs->DMAMODE = dmode;
        /* DMA transfer restart */
	    //soc_dma_set_mode(tr, DMA_MODE_NORMAL);
        soc_dma_transfer(tr);

		spin_unlock_irqrestore(&port->lock, flags);
    } else {
        return IRQ_NONE;
    }

    return IRQ_HANDLED;
}
#endif

static void init_uart_port(void)
{
#if OLD_IMPLEMENTATION
	int i = 0;
	int array = NX_UART_GetNumberOfModule();
	DBGOUT("%s (array:%d)\n", __func__, array);

	for (i=0; i < array; i++) {
		NX_UART_SetBaseAddress(i, (U32)ioremap_nocache(NX_UART_GetPhysicalAddress(i),0x80));
		/* Disable debug uart port clock */
#if !(PM_DBGOUT_ON) && !defined(CONFIG_DEBUG_LL_UART)
		NX_UART_OpenModule(i);
		NX_UART_SetClockDivisorEnable(i, CFALSE);
		NX_UART_SetClockPClkMode(i, NX_PCLKMODE_DYNAMIC);
#endif
	}
#else
	
    struct NX_UART_RegisterSet *uart0 = (struct NX_UART_RegisterSet *)ioremap_nocache(PHY_BASEADDR_UART0,0x80);
    struct NX_UART_RegisterSet *uart1 = (struct NX_UART_RegisterSet *)ioremap_nocache(PHY_BASEADDR_UART1,0x80);
	DBGOUT("%s (2)\n", __func__);

	/* Disable uart clken with PCLK_DYNAMIC */
	uart0->CLKENB &= ~(NXP2120_UART_CLKENB_PCLKMODE | NXP2120_UART_CLKENB_CLKGENB);
	uart1->CLKENB &= ~(NXP2120_UART_CLKENB_PCLKMODE | NXP2120_UART_CLKENB_CLKGENB);
#endif
}

static void setup_uart_port(unsigned int port)
{
#if OLD_IMPLEMENTATION
	DBGOUT("%s (port:%d)\n", __func__, port);

	/* Select module & initialize */
	NX_UART_OpenModule(port);

	/* Clock & Interrupt Disable */
	NX_UART_SetClockDivisorEnable(port, CFALSE);
	NX_UART_SetClockPClkMode(port, NX_PCLKMODE_DYNAMIC);

	NX_UART_ClearInterruptPendingAll(port);
   	NX_UART_SetInterruptEnableAll(port, CFALSE);

	// UART Mode : Tx, Rx Only
   	NX_UART_SetSIRMode(port, CFALSE);
	NX_UART_SetLoopBackMode(port, CFALSE);
	NX_UART_SetAutoFlowControl(port, CFALSE);
	NX_UART_SetHalfChannelEnable(port, CTRUE);

    NX_UART_SetSCTxEnb(port, CFALSE);
    NX_UART_SetSCRxEnb(port, CTRUE);

	// Frame Configuration : Data 8 - Parity 0 - Stop 1
	NX_UART_SetFrameConfiguration (port, NX_UART_PARITYMODE_NONE, 8, 1);

	// Tx Rx Operation : Default Polling
  	NX_UART_SetTxIRQType(port, NX_UART_IRQTYPE_PULSE);
   	NX_UART_SetTxOperationMode(port, NX_UART_OPERMODE_NORMAL);    	// Interrupt or Polling

   	NX_UART_SetRxIRQType(port, NX_UART_IRQTYPE_LEVEL);
   	NX_UART_SetRxOperationMode(port, NX_UART_OPERMODE_NORMAL);    	// Interrupt or Polling
   	if (port == 0)
   	    NX_UART_SetRxTimeOutEnb(port, CTRUE);							// NEED Enable for RX interrupt.
   	else
   	    NX_UART_SetRxTimeOutEnb(port, CTRUE);							// NEED Enable for RX interrupt.
   	NX_UART_SetIntEnbWhenExceptionOccur(port, CFALSE);

	NX_UART_SetSYNCPendClear(port);

	/* FCON: fifo control */
	NX_UART_SetTxFIFOTriggerLevel(port, UART_TXD_LEVEL);			// Tx empty irq
    NX_UART_ResetTxFIFO(port);

	NX_UART_SetRxFIFOTriggerLevel(port, UART_RXD_LEVEL);			// Rx full  irq
    NX_UART_ResetRxFIFO(port);
    NX_UART_SetFIFOEnb(port, CFALSE);

	/* MCON: modem control - skip */
    NX_UART_SetDTR(port, CFALSE);
    NX_UART_SetRTS(port, CFALSE);

	/* BRDn: baud rate divisor */
	NX_UART_SetBRD(port, NX_UART_MakeBRD(CFG_UART_DEBUG_BAUDRATE, CFG_UART_DEBUG_CLKFREQ));

	/* TIMEOUTREG: receive timeout */
    NX_UART_SetRxTimeOut(port, 0x1);

	/* INTSTATUSREG: Interrupt status */
    NX_UART_ClearInterruptPendingAll(port);

	/* UARTCLKGEN, UARTCLKENB */
	NX_UART_SetClockSource  (port, 0, CFG_UART_DEBUG_CLKSRC);
	NX_UART_SetClockDivisor (port, 0, CFG_UART_DEBUG_CLKDIV);
	if (port== 1) /* ksw : for test */
	    NX_UART_SetClockDivisor(port, 0, 2);
	NX_UART_SetClockDivisorEnable(port, CTRUE);
	NX_UART_SetClockPClkMode(port, NX_PCLKMODE_ALWAYS);
#else
    struct NX_UART_RegisterSet *uart;
	DBGOUT("%s (port:%d)\n", __func__, port);

	if (0 == port) {
	    uart = (struct NX_UART_RegisterSet *)ioremap_nocache(PHY_BASEADDR_UART0,0x80);
	} else {
	    uart = (struct NX_UART_RegisterSet *)ioremap_nocache(PHY_BASEADDR_UART1,0x80);
	}

	/* Clock & Interrupt Disable */
	uart->CLKENB = uart->CLKENB & ~(NXP2120_UART_CLKENB_PCLKMODE | NXP2120_UART_CLKENB_CLKGENB);
	uart->INTCON = uart->INTCON | 0x0F; /* All Four Pending Bits Clear, and disable IRQ all */

	// UART Mode : Tx, Rx Only
	uart->LCON = 0;
	// Tx Rx Operation : Default Polling
#if USE_DMA_CHAN0
    if (0 == port) {
        uart->UCON = NXP2120_UCON_RX_INT_PULSE | NXP2120_UCON_TRANS_IRQ_POLLING |
	                 NXP2120_UCON_RECV_DMA_REQ | NXP2120_UCON_RX_TIMEOUT;
	} else {
	    uart->UCON = NXP2120_UCON_RX_INT_LEVEL | NXP2120_UCON_TRANS_IRQ_POLLING |
	                 NXP2120_UCON_RECV_IRQ_POLLING | NXP2120_UCON_RX_TIMEOUT;
	}
#endif
#if USE_DMA_CHAN1
    if (1 == port) {
        uart->UCON = NXP2120_UCON_RX_INT_PULSE | NXP2120_UCON_TRANS_IRQ_POLLING |
	                 NXP2120_UCON_RECV_DMA_REQ;// | NXP2120_UCON_RX_TIMEOUT;
	} else {
	    uart->UCON = NXP2120_UCON_RX_INT_LEVEL | NXP2120_UCON_TRANS_IRQ_POLLING |
	                 NXP2120_UCON_RECV_IRQ_POLLING | NXP2120_UCON_RX_TIMEOUT;
	}
#endif
#if (!USE_DMA_CHAN0) && (!USE_DMA_CHAN1)
    uart->UCON = NXP2120_UCON_RX_INT_LEVEL | NXP2120_UCON_TRANS_IRQ_POLLING |
	             NXP2120_UCON_RECV_IRQ_POLLING | NXP2120_UCON_RX_TIMEOUT;
#endif
	uart->MCON = NXP2120_MCON_HALF_CH_ENB | NXP2120_MCON_SCRXENB;

	// Frame Configuration : Data 8 - Parity 0 - Stop 1 for default.
	uart->LCON = NXP2120_LCON_PARITY_NONE | NXP2120_LCON_ONE_STOP_BIT | NXP2120_LCON_WORDLEN_8BIT;

	uart->LCON = uart->LCON | NXP2120_LCON_SYNC_PENDCLR;

	/* FCON: fifo control */
	uart->FCON = NXP2120_FCON_TX_FIFO_TRIG_0BYTE | NXP2120_FCON_RX_FIFO_TRIG_4BYTE;//NXP2120_FCON_RX_FIFO_TRIG_8BYTE;
	uart->FCON = uart->FCON | NXP2120_FCON_TX_FIFO_RESET | NXP2120_FCON_RX_FIFO_RESET;

	/* MCON: modem control - skip */

	/* BRDn: baud rate divisor */
	uart->BRD = 4 - 1;

	/* TIMEOUTREG: receive timeout */
	uart->TIMEOUT = 0x01F; /* 0x1F is default */

	/* INTSTATUSREG: Interrupt status */
    uart->INTCON = 0 |  0x0F; // uart->INTCON turn off all the interrupt

	/* UARTCLKGEN, UARTCLKENB */
	uart->CLKGEN = NXP2120_UART_CLKGEN_SRC_PLL1 | ((CFG_UART_DEBUG_CLKDIV-1) << 5);
#ifdef CONFIG_MANGLED_BAUDRATE
	if (port== 1) { /* ksw : for test */
	    uart->CLKGEN = NXP2120_UART_CLKGEN_SRC_PLL1 | ((6-1) << 5);
	}
#endif
	uart->CLKENB = NXP2120_UART_CLKENB_PCLKMODE | NXP2120_UART_CLKENB_CLKGENB;
#endif

	UART_WAIT_READY;
}

/*---------------------------------------------------------------------------------------------
 * 	Uart ops functions
 --------------------------------------------------------------------------------------------*/

/*
 * return 0; tx buffer not empty
 * return 1: tx buffer empty
 */
static unsigned int nx_uart_ops_tx_empty(struct uart_port *port)
{
	unsigned long flags;
	unsigned int ret=0;
//	struct nx_uart_port *uart = (struct nx_uart_port *)port;
	struct NX_UART_RegisterSet *regs;	
	regs = (struct NX_UART_RegisterSet *)port->membase;		

	DBGOUT("%s (line:%d)\n", __func__, port->line);

	/* disable irq */
	spin_lock_irqsave(&port->lock,flags);

#if OLD_IMPLEMENTATION
	/* buffer empty */
	if ( (NX_UART_GetTxRxStatus(port->line) & NX_UART_TRANSMITTER_EMPTY) ) {
		ret = TIOCSER_TEMT;
	}
#else
	/* buffer empty */
#if 1
	if ( (regs->TRSTATUS & NX_UART_TRANSMITTER_EMPTY) ) {
		ret = TIOCSER_TEMT;
	}
#else
    /* NEXELL says we should check FIFO Empty ?? */
#endif
#endif

	/* enable irq */
	spin_unlock_irqrestore(&port->lock, flags);
	return ret;
}

/* modem control */
static void nx_uart_ops_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	DBGOUT("%s (line:%d) mctrl=%x\n", __func__, port->line, mctrl);

	/* not implementation */
}

/* modem control */
static unsigned int nx_uart_ops_get_mctrl(struct uart_port *port)
{
	unsigned int ret = (TIOCM_CTS | TIOCM_DSR | TIOCM_CAR);
	DBGOUT("nx_uart_ops_get_mctrl (line:%d)\n", port->line);

	/* not implementation */
	return ret;
}

static void nx_uart_ops_stop_tx(struct uart_port *port)
{
//    struct nx_uart_port *uart = (struct nx_uart_port *)port;
	struct NX_UART_RegisterSet *regs;	
	regs = (struct NX_UART_RegisterSet *)port->membase;		

	DBGINTR("%s (line:%d)\n", __func__, port->line);

#if OLD_IMPLEMENTATION
	if (NX_UART_GetInterruptEnable(port->line, UART_TXD_INTNUM))
		NX_UART_SetInterruptEnable(port->line, UART_TXD_INTNUM, CFALSE);
#else
    if (regs->INTCON & NXP2120_INTCON_TXENB)
		regs->INTCON =  regs->INTCON & ~NXP2120_INTCON_TXENB;
#endif
}

static void nx_uart_ops_start_tx(struct uart_port *port)
{
//    struct nx_uart_port *uart = (struct nx_uart_port *)port;
	struct NX_UART_RegisterSet *regs;	
	regs = (struct NX_UART_RegisterSet *)port->membase;		
 
    unsigned short reg;
	DBGINTR("%s (line:%d)\n", __func__, port->line);

#if OLD_IMPLEMENTATION
	if ( ! NX_UART_GetInterruptEnable(port->line, UART_TXD_INTNUM))
		NX_UART_SetInterruptEnable(port->line, UART_TXD_INTNUM, CTRUE);
#else
    if (0 == ((reg = regs->INTCON) & NXP2120_INTCON_TXENB)) {
        reg &= 0xF0;
		regs->INTCON =  reg | NXP2120_INTCON_TXENB;
	}
#endif
}

static void nx_uart_ops_stop_rx(struct uart_port *port)
{
//    struct nx_uart_port *uart = (struct nx_uart_port *)port;	
	struct NX_UART_RegisterSet *regs;	
	regs = (struct NX_UART_RegisterSet *)port->membase;		


	DBGOUT("%s (line:%d)\n", __func__, port->line);

#if OLD_IMPLEMENTATION
	if (NX_UART_GetInterruptEnable(port->line, UART_RXD_INTNUM))
		NX_UART_SetInterruptEnable(port->line, UART_RXD_INTNUM, CFALSE);
#else
    if (regs->INTCON & NXP2120_INTCON_RXENB)
		regs->INTCON =  regs->INTCON & ~NXP2120_INTCON_RXENB;

//    uart->dma_regs->DMAMODE |= 1 << 19;
#endif
}

static void nx_uart_ops_enable_ms(struct uart_port *port)
{
	DBGOUT("%s (line:%d)\n", __func__, port->line);
	/* not implementation */
}


static void nx_uart_ops_break_ctl(struct uart_port *port, int break_state)
{
	unsigned long flags;
	DBGOUT("%s (line:%d)\n", __func__, port->line);

	/* disable irq */
	spin_lock_irqsave(&port->lock, flags);

	/* not implementation */

	/* enable irq */
	spin_unlock_irqrestore(&port->lock, flags);
}

#if (USE_DMA_CHAN0 || USE_DMA_CHAN1)
static u64 nx_uart_dmamask = DMA_BIT_MASK(32);
#endif

static int nx_uart_ops_startup(struct uart_port *port)
{
//	struct nx_uart_port *uart = (struct nx_uart_port *)port;
	struct NX_UART_RegisterSet *regs;	
	regs = (struct NX_UART_RegisterSet *)port->membase;		

	unsigned long flags;
#if !IRQ_ALWAYS
	int ret;
#endif

	DBGOUT("%s (line:%d) \n", __func__, port->line);
	//printk("uart startup\n");


#if USE_DMA_CHAN0 && !USE_DMA_CHAN1
    if (0 == port->line) {
        /* Setup DMA ... */
        uart->dma_tr = soc_dma_request(DMA_CHAN0_CHANNEL, false);
	    if (!uart->dma_tr)  {
	        return -1; /* Error code */
	    }
	    uart->dma_tr->tr_type = DMA_TRANS_IO2MEM;
	    uart->dma_tr->srcbase = PHY_BASEADDR_UART0;
	    uart->dma_tr->src_id  = DMAINDEX_OF_UART0_MODULE_RX;
	    uart->dma_tr->src_bit = 8;
	    uart->dma_tr->length = DMA_BUFFER_SIZE;
	    uart->buf_index = 0;
	    uart->dma_tr->dstbase = uart->dma_phys[uart->buf_index];
        uart->last_addr = uart->dma_phys[uart->buf_index];
        uart->last_buf  = uart->buf_base[uart->buf_index];
        uart->zero_cnt  = 0;

	    /* Clear FIFO */
	    uart->regs->FCON = uart->regs->FCON | NXP2120_FCON_TX_FIFO_RESET | NXP2120_FCON_RX_FIFO_RESET;
	    soc_dma_set_mode(uart->dma_tr, DMA_MODE_NORMAL);
		soc_dma_transfer(uart->dma_tr);
    }
#endif
#if USE_DMA_CHAN1 && !USE_DMA_CHAN0
    if (1 == port->line) {
#if 1
	    /* Enable CLKGEN */
	    uart->regs->CLKGEN = uart->regs->CLKGEN | NXP2120_UART_CLKENB_CLKGENB;
	    /* Clear Pending Clear */
	    uart->regs->LCON = uart->regs->LCON | NXP2120_LCON_SYNC_PENDCLR;
        /* Set UART mode to IRQ_POLLING */
	    uart->regs->UCON &= ~NXP2120_UCON_RECV_MODE_MASK;
        uart->regs->UCON |= NXP2120_UCON_RECV_IRQ_POLLING;
        /* Clear FIFO */
	    uart->regs->FCON = uart->regs->FCON | NXP2120_FCON_RX_FIFO_RESET;
	    /* Enable UART mode to DMA */
        uart->regs->UCON &= ~NXP2120_UCON_RECV_MODE_MASK;
        uart->regs->UCON |= NXP2120_UCON_RECV_DMA_REQ;

        /* Clear FIFO with reading the values from the FIFO */
        //while (uart->regs->FSTATUS & 0x0F) {
        //    int ch;
        //    ch = uart->regs->RHB;
        //    printk("ch= %x fstatus = %x fcon = %x estatus = %x\n", ch, uart->regs->FSTATUS, uart->regs->FCON, uart->regs->ESTATUS);
        //}
#endif
        /* Setup DMA ... */
        uart->dma_tr = soc_dma_request(DMA_CHAN1_CHANNEL, false);
	    if (!uart->dma_tr)  {
	        return -1; /* Error code */
	    }
	    uart->dma_tr->tr_type = DMA_TRANS_IO2MEM;
	    uart->dma_tr->io2m.srcbase = PHY_BASEADDR_UART1;
	    uart->dma_tr->io2m.src_id  = DMAINDEX_OF_UART1_MODULE_RX;
	    uart->dma_tr->io2m.src_bit = 8;
	    uart->dma_tr->io2m.length = DMA_BUFFER_SIZE;
	    uart->buf_index = 0;
	    uart->dma_tr->io2m.dstbase = uart->dma_phys[uart->buf_index];

        uart->last_addr = uart->dma_phys[uart->buf_index];
        uart->last_buf  = uart->buf_base[uart->buf_index];
        uart->zero_cnt  = 0;

        uart->buf_changed = 0;
        uart->dma_regs->DMAMODE |= 1 << 18;
	    soc_dma_set_mode(uart->dma_tr, DMA_MODE_NORMAL);
		soc_dma_transfer(uart->dma_tr);
		uart->dma_run = 1;
    }
#endif
#if !IRQ_ALWAYS
#if USE_DMA_CHAN1 && !USE_DMA_CHAN0
	// Enable RX Interrupt
    if (port->line == 1) {/* ksw : for test */
		uart->regs->INTCON = 0;
    }
    else
    {
		uart->regs->INTCON = NXP2120_INTCON_RXENB;
    }
#endif
#if !USE_DMA_CHAN1 && USE_DMA_CHAN0
	// Enable RX Interrupt
    if (port->line == 0) {/* ksw : for test */
		uart->regs->INTCON = 0;
    }
    else
    {
		uart->regs->INTCON = NXP2120_INTCON_RXENB;
    }
#endif
	ret = request_irq(port->irq, nx_uart_irq_handler, 0, UART_DEV_NAME, port);
	if ( ret!=0 ) {
		dev_err(port->dev, "unable to grab irq%d\n",port->irq);
		return ret;
	}
#endif

	/* disable irq */
	spin_lock_irqsave(&port->lock, flags);

	if(0 == uart_count);
		setup_uart_port(port->line);

	uart_count++;

	/* enable irq */
	spin_unlock_irqrestore(&port->lock, flags);
	return 0;
}

static void nx_uart_ops_shutdown(struct uart_port *port)
{
//	struct nx_uart_port *uart = (struct nx_uart_port *)port;
	struct NX_UART_RegisterSet *regs;	
	regs = (struct NX_UART_RegisterSet *)port->membase;		

	unsigned long flags;

	DBGOUT("%s (line:%d, count:%d)\n", __func__, port->line, uart_count);
    //printk("called nx_uart_ops_shutdown\n");
	/* disable irq */
	spin_lock_irqsave(&port->lock,flags);

#if OLD_IMPLEMENTATION
   	NX_UART_SetInterruptEnableAll(port->line, CFALSE);
    NX_UART_ClearInterruptPendingAll(port->line);

	uart->count--;
	if (0 == uart->count) {
    	NX_UART_SetSYNCPendClear(port->line);
		NX_UART_SetClockDivisorEnable(port->line, CFALSE);
	}
#else
    regs->INTCON &= ~0xF0;
    regs->INTCON |= 0xF;

	uart_count--;
	if (0 == uart_count) {
	    regs->LCON |= NXP2120_LCON_SYNC_PENDCLR;
    	regs->CLKGEN &= ~NXP2120_UART_CLKENB_CLKGENB;
	}
#endif

	if (0 >	uart_count)
		uart_count = 0;

	/* enable irq */
	spin_unlock_irqrestore(&port->lock,flags);

#if !IRQ_ALWAYS
	/* release irq */
	free_irq(port->irq, port);
#endif

#if USE_DMA_CHAN1 && !USE_DMA_CHAN0
    if (1 == port->line) {
        /* Stop DMA ... */
        //uart->dma_regs->DMAMODE |= NXP2120_DMAMODE_STOP; /* Stop DMA */
        //while (uart->dma_regs->DMAMODE & NXP2120_DMAMODE_RUN) ; /* Check DMA stopped. */
        soc_dma_trans_stop(uart->dma_tr);
		if (uart->dma_tr)
            soc_dma_release(uart->dma_tr);
        uart->dma_run = 0;
    }
#endif

}

#define TTY_TRIGGER_ZERO	0x8000000
static void nx_uart_ops_set_termios(struct uart_port *port, struct ktermios *termios, struct ktermios *old)
{
	unsigned int data, parity, stop;
	unsigned int baud, quot;
	unsigned long flags;
	unsigned short reg;
	//unsigned short int *ptr;
//	struct nx_uart_port *uart = (struct nx_uart_port *)port;
	struct NX_UART_RegisterSet *regs;	
	regs = (struct NX_UART_RegisterSet *)port->membase;		
	DBGOUT("%s (line:%d)\n", __func__, port->line);

#if OLD_IMPLEMENTATION
	/* Data Bit */
	switch (termios->c_cflag & CSIZE) {
	case CS5:
		data =  5;
		break;
	case CS6:
		data =  6;
		break;
	case CS7:
		data =  7;
		break;
	default:
	case CS8:
		data =  8;
		break;
	}

	/* Parity Bit */
	if (termios->c_cflag & PARENB) {
		if (termios->c_cflag & PARODD)
			parity = NX_UART_PARITYMODE_ODD;
		else
			parity = NX_UART_PARITYMODE_EVEN;
	} else {
		parity = NX_UART_PARITYMODE_NONE;
	}

	/* Stop Bit */
	if (termios->c_cflag & CSTOPB)
		stop = 2;
	else
		stop = 1;

	/* Baudrate */
	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk/16);
	quot = uart_get_divisor(port, baud);

	printk("serial init:port=%d baud=%d parity=%d data=%d stop=%d clk=%d quot=%d\n",port->line,baud,parity,data,stop, port->uartclk/16, quot);
	/* disable irq */
	spin_lock_irqsave(&port->lock, flags);

	/* Update the per-port timeout */
	uart_update_timeout(port, termios->c_cflag, baud);

	/*
	 * Which character status flags are we interested in?
	 */
	port->read_status_mask = UART_ERR_OVERRUN;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= UART_ERR_FRAME | UART_ERR_PARITY;

	/*
	 * Which character status flags should we ignore?
	 */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= UART_ERR_OVERRUN;
	if (termios->c_iflag & IGNBRK && termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= UART_ERR_FRAME;

	/*
	 * Ignore all characters if CREAD is not set.
	 */
	if ((termios->c_cflag & CREAD) == 0)
		port->ignore_status_mask |= UART_RXD_DUMMY;

	DBGOUT("%s (baud:%d, parity:%d, data:%d, stop:%d)\n", __func__, baud , parity, data, stop);

	// set nexell uart register
	NX_UART_SetClockDivisorEnable(port->line, CFALSE);
	NX_UART_SetClockPClkMode(port->line, NX_PCLKMODE_DYNAMIC);
   	NX_UART_SetInterruptEnableAll(port->line, CFALSE);

	NX_UART_SetFrameConfiguration(port->line, parity, data, stop);
	NX_UART_SetBRD(port->line, NX_UART_MakeBRD(baud, CFG_UART_DEBUG_CLKFREQ));
    if (port->line == 1) {/* ksw : for test */
	    NX_UART_SetBRD(port->line, 51-1);
    }

   	NX_UART_ResetTxFIFO(port->line);
   	NX_UART_ResetRxFIFO(port->line);
   	NX_UART_SetFIFOEnb(port->line, CTRUE);

	// Enable RX Interrupt
	NX_UART_SetInterruptEnable(port->line, UART_MOD_INTNUM, CFALSE);
	NX_UART_SetInterruptEnable(port->line, UART_ERR_INTNUM, CFALSE);
	NX_UART_SetInterruptEnable(port->line, UART_RXD_INTNUM, CTRUE);
	NX_UART_SetInterruptEnable(port->line, UART_TXD_INTNUM, CTRUE); //CFALSE
   	NX_UART_ClearInterruptPendingAll(port->line);
   	NX_UART_SetSYNCPendClear(port->line);

//  NX_UART_SetInterruptEnableAll(port->line, CTRUE);

	NX_UART_SetClockDivisorEnable(port->line, CTRUE);
	NX_UART_SetClockPClkMode(port->line, NX_PCLKMODE_ALWAYS);
#else
	/* Data Bit */
	switch (termios->c_cflag & CSIZE) {
	case CS5:
		data =  NXP2120_LCON_WORDLEN_5BIT;
		break;
	case CS6:
		data =  NXP2120_LCON_WORDLEN_6BIT ;
		break;
	case CS7:
		data =  NXP2120_LCON_WORDLEN_7BIT;
		break;
	default:
	case CS8:
		data =  NXP2120_LCON_WORDLEN_8BIT;
		break;
	}

	/* Parity Bit */
	if (termios->c_cflag & PARENB) {
		if (termios->c_cflag & PARODD)
			parity = NXP2120_LCON_PARITY_ODD;
		else
			parity = NXP2120_LCON_PARITY_EVEN;
	} else {
		parity = NXP2120_LCON_PARITY_NONE;
	}

	/* Stop Bit */
	if (termios->c_cflag & CSTOPB)
		stop = NXP2120_LCON_TWO_STOP_BIT;
	else
		stop = NXP2120_LCON_ONE_STOP_BIT;

	if ((termios->c_cflag & CBAUD) == BOTHER) {
		int tval,minval,clkdiv;
		// Then we should set CLKGEN
		/* Base clock should be around 6M~8MHz.
		   so custom baudrate should have choice of some multiple of 16,
		   and values of N quot.
		 */
		tval = termios->c_ospeed * 16;
		minval = tval;
		clkdiv = 1;
		while (minval < 8000000) {
			minval = tval * clkdiv;
			if (minval > 6000000)
				break;
			clkdiv++;
		}
		clkdiv = 192000000 / minval;
		regs->CLKGEN = NXP2120_UART_CLKGEN_SRC_PLL1 | ((clkdiv-1) << 5);
		port->uartclk = minval;
		printk("BOTHER ospeed=%d\n", termios->c_ospeed); 
	} else {
		/* Normal CLKGEN */
		regs->CLKGEN = NXP2120_UART_CLKGEN_SRC_PLL1 | ((CFG_UART_DEBUG_CLKDIV-1) << 5);
		port->uartclk = CFG_UART_DEBUG_CLKFREQ;
	}
	/* Baudrate */
	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk/16);
	quot = uart_get_divisor(port, baud);

	printk("serial init:port=%d baud=%d parity=%d data=%d stop=%d clk=%d quot=%d trigger=%d\n",port->line,baud,parity,data,stop, port->uartclk/16, quot, (termios->c_iflag & TTY_TRIGGER_ZERO) ? 0 : 4);
	/* disable irq */
	spin_lock_irqsave(&port->lock, flags);

	/* Update the per-port timeout */
	uart_update_timeout(port, termios->c_cflag, baud);

	/*
	 * Which character status flags are we interested in?
	 */
	port->read_status_mask = UART_ERR_OVERRUN;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= UART_ERR_FRAME | UART_ERR_PARITY;

	/*
	 * Which character status flags should we ignore?
	 */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= UART_ERR_OVERRUN;
	if (termios->c_iflag & IGNBRK && termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= UART_ERR_FRAME;

	/*
	 * Ignore all characters if CREAD is not set.
	 */
	if ((termios->c_cflag & CREAD) == 0)
		port->ignore_status_mask |= UART_RXD_DUMMY;

	DBGOUT("%s (baud:%d, parity:%d, data:%d, stop:%d)\n", __func__, baud , parity, data, stop);

	// set nexell uart register
	regs->CLKENB =regs->CLKENB & ~NXP2120_UART_CLKENB_CLKGENB;
	regs->INTCON = 0x0F;

	reg = regs->LCON & ~0x3F;
	regs->LCON  = reg | data | parity | stop;

	regs->BRD = quot -1;
#ifdef CONFIG_MANGLED_BAUDRATE
    if (port->line == 1) {/* ksw : for test */
	    regs->BRD =  17-1;
    }
#endif
/* Clear FIFO with reading the values from the FIFO */
        //while (uart->regs->FSTATUS & 0x0F) {
        //    int ch;
        //    ch = uart->regs->RHB;
        //    printk("ch= %x fstatus = %x fcon = %x estatus = %x\n", ch, uart->regs->FSTATUS, uart->regs->FCON, uart->regs->ESTATUS);
        //}
    regs->FCON = regs->FCON | NXP2120_FCON_TX_FIFO_RESET | NXP2120_FCON_RX_FIFO_RESET;
    regs->FCON = regs->FCON | NXP2120_FCON_FIFO_ENABLE;

#if USE_DMA_CHAN1 && !USE_DMA_CHAN0
	// Enable RX Interrupt
    if (port->line == 1) {/* ksw : for test */
		uart->regs->INTCON = 0;

        if (termios->c_iflag & TTY_TRIGGER_ZERO)
    		uart->regs->FCON = (uart->regs->FCON & ~(NXP2120_FCON_RX_FIFO_TRIG_MASK)) | NXP2120_FCON_RX_FIFO_TRIG_0BYTE;
    	else
    		uart->regs->FCON = (uart->regs->FCON & ~(NXP2120_FCON_RX_FIFO_TRIG_MASK)) | NXP2120_FCON_RX_FIFO_TRIG_4BYTE;
    }
    else
    {
		uart->regs->INTCON = NXP2120_INTCON_RXENB;
    }
#endif

#if !USE_DMA_CHAN1 && USE_DMA_CHAN0
	// Enable RX Interrupt
    if (port->line == 0) {/* ksw : for test */
		uart->regs->INTCON = 0;
    }
    else
    {
		uart->regs->INTCON = NXP2120_INTCON_RXENB;
    }
#endif
#if !USE_DMA_CHAN1 && !USE_DMA_CHAN0
	// Enable RX Interrupt
	regs->INTCON = NXP2120_INTCON_RXENB;
#endif
	regs->INTCON = regs->INTCON | 0x0F; /* Clear Interrupt Pending all */
	regs->LCON = regs->LCON | NXP2120_LCON_SYNC_PENDCLR;

	regs->CLKENB = NXP2120_UART_CLKENB_PCLKMODE | NXP2120_UART_CLKENB_CLKGENB;
#endif

	UART_WAIT_READY;

	/* enable irq */
	spin_unlock_irqrestore(&port->lock, flags);
}

/* Custom UART control IOCTL */
struct uart_struc {
	short int subaddr;
	short int val;
};

#define UART_TRIGGER_LEVEL	_IOW('V',100, int)

static int nx_uart_ops_ioctl(struct uart_port *port, unsigned int cmd, unsigned long arg)
{
//	struct nx_uart_port *uart = (struct nx_uart_port *)port;
	struct NX_UART_RegisterSet *regs;	
	regs = (struct NX_UART_RegisterSet *)port->membase;		
printk("nx_uart_ops_ioctl : called cmd=%d\n", cmd);
    if (cmd == UART_TRIGGER_LEVEL)
    {
    	unsigned long	fcon;
    	int				level = *(int*)arg;

    	fcon = regs->FCON;

        if (arg == 0)
    		fcon = (fcon & ~(NXP2120_FCON_RX_FIFO_TRIG_MASK)) | NXP2120_FCON_RX_FIFO_TRIG_0BYTE;
    	else if (arg == 4)
    		fcon = (fcon & ~(NXP2120_FCON_RX_FIFO_TRIG_MASK)) | NXP2120_FCON_RX_FIFO_TRIG_4BYTE;
    	else if (arg == 8)
    		fcon = (fcon & ~(NXP2120_FCON_RX_FIFO_TRIG_MASK)) | NXP2120_FCON_RX_FIFO_TRIG_8BYTE;
    	else if (arg == 12)
    		fcon = (fcon & ~(NXP2120_FCON_RX_FIFO_TRIG_MASK)) | NXP2120_FCON_RX_FIFO_TRIG_12BYTE;
    	else
    		return EINVAL;

    	regs->FCON = fcon;
    }

	return 0;
}

static const char *nx_uart_ops_type(struct uart_port *port)
{
//	struct nx_uart_port *uart = (struct nx_uart_port *)port;
	DBGOUT("%s (line:%d)\n", __func__, port->line);

	return (port->type == PORT_NXP2120) ? "nxp2120_uart" : NULL;//uart->name;
}

static void nx_uart_ops_release_port(struct uart_port *port)
{
	DBGOUT("%s (line:%d)\n", __func__, port->line);
}

static int nx_uart_ops_request_port(struct uart_port *port)
{
	DBGOUT("%s (line:%d)\n", __func__, port->line);
	return 0;
}

static void nx_uart_ops_config_port(struct uart_port *port, int flags)
{
	DBGOUT("%s (line:%d, type:%s)\n", __func__, port->line, (char *)port->type);
	if (flags & UART_CONFIG_TYPE) {	
		if (nx_uart_ops_request_port(port))
			return;
		port->type = PORT_NXP2120;
	}
}

static int nx_uart_ops_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	DBGOUT("%s (line:%d)\n", __func__, port->line);
	if (port->type != PORT_NXP2120)
		return -EINVAL;
	if (port->irq != ser->irq)
		return -EINVAL;
	if (port->iotype != ser->io_type)
		return -EINVAL;
	if (port->mapbase != (unsigned long)ser->iomem_base)
		return -EINVAL;
//	 we don't want the core code to modify any port params 
	return 0;
}

static struct uart_ops nx_uart_ops = {
	.tx_empty		= nx_uart_ops_tx_empty,
	.set_mctrl		= nx_uart_ops_set_mctrl,
	.get_mctrl		= nx_uart_ops_get_mctrl,
	.stop_tx		= nx_uart_ops_stop_tx,
	.start_tx		= nx_uart_ops_start_tx,
	.stop_rx		= nx_uart_ops_stop_rx,
	.enable_ms		= nx_uart_ops_enable_ms,
	.break_ctl		= nx_uart_ops_break_ctl,
	.startup		= nx_uart_ops_startup,
	.shutdown		= nx_uart_ops_shutdown,
	.set_termios	= nx_uart_ops_set_termios,
	.type			= nx_uart_ops_type,
	.release_port	= nx_uart_ops_release_port,
	.request_port	= nx_uart_ops_request_port,
	.config_port	= nx_uart_ops_config_port,
	.verify_port	= nx_uart_ops_verify_port,
	//.ioctl			= nx_uart_ops_ioctl,
};

/*---------------------------------------------------------------------------------------------
 * 	Uart interrupt functions
 --------------------------------------------------------------------------------------------*/
static void nx_uart_rx_irq_handler(struct uart_port *port)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
	struct tty_struct *tty = port->state->port.tty;
	struct tty_port *tport = &port->state->port;		
#else
	struct tty_struct *tty = port->info->port.tty;
#endif
	unsigned int ch, flag;
	unsigned int err,reg;
	int max_count = 256;
//	struct nx_uart_port *uart = (struct nx_uart_port *)port;
	struct NX_UART_RegisterSet *regs;	
	regs = (struct NX_UART_RegisterSet *)port->membase;		

//	printk(" rxirq...");

	while (max_count-- > 0) {

#if OLD_IMPLEMENTATION

#else
#if (!USE_DMA_CHAN0) && (!USE_DMA_CHAN1)
        if (((reg = regs->FSTATUS) & 0x0F) == 0 && ((reg & NXP2120_FSTATUS_RX_FIFO_FULL) == 0)){
 //       	printk("regs->FSTATUS 0x%X reg & NXP2120_FSTATUS_RX_FIFO_FULL 0x%X\n", regs->FSTATUS,reg & NXP2120_FSTATUS_RX_FIFO_FULL );
			break;
		}
		err  = regs->ESTATUS & 0x0F;
        ch   = regs->RHB;
#endif
#endif
		flag = TTY_NORMAL;
		port->icount.rx++;

		/* error status */
		if (unlikely(err & (UART_ERR_OVERRUN |
					UART_ERR_PARITY	 |
					UART_ERR_FRAME   |
					UART_ERR_BREAK )) ) {
		//	printk("uart: rx error port:%d, ch=%c, err=%x rxfifocnt=%X\n", port->line, ch, err, regs->FSTATUS);

			/* break error */
			if (err & UART_ERR_BREAK) {
				port->icount.brk++;
				if (uart_handle_break(port))
					goto ignore_char;
			}
			/* parity error */
			if (err & UART_ERR_PARITY)
				port->icount.parity++;

			if (err & UART_ERR_FRAME)
				port->icount.frame++;

			if (err & UART_ERR_OVERRUN)
				port->icount.overrun++;

			/*
			 * Mask off conditions which should be ignored.
			 */
			err &= port->read_status_mask;

			if (err & UART_ERR_BREAK)
				flag = TTY_BREAK;
			else if (err & UART_ERR_PARITY)
				flag = TTY_PARITY;
			else if (err & UART_ERR_FRAME)
				flag = TTY_FRAME;
		}

		if (uart_handle_sysrq_char(port, ch))
			goto ignore_char;

		//uart_insert_char(port, err, UART_LSR_OE, ch, flag);
		tty_insert_flip_char(tport,ch,flag);
//		printk("ch %c\n", ch);

	ignore_char:
		continue;
	}
//	tty_flip_buffer_push(tty);
	tty_flip_buffer_push(tport);	
}

static void nx_uart_tx_irq_handler(struct uart_port *port)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
	struct circ_buf *xmit = &port->state->xmit;
#else
	struct circ_buf *xmit = &port->info->xmit;
#endif
//    struct nx_uart_port *uart = (struct nx_uart_port *)port;
	struct NX_UART_RegisterSet *regs;	
	regs = (struct NX_UART_RegisterSet *)port->membase;		

    unsigned char tmp;
	DBGINTR(" txirq[%d]", port->line);

#if OLD_IMPLEMENTATION
	if (port->x_char) {
		NX_UART_SendByte(port->line, port->x_char);
		port->icount.tx++;
		port->x_char=0;
		return;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		nx_uart_ops_stop_tx(port);
		return;
	}

	do {
		NX_UART_SendByte(port->line, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (NX_UART_GetTxFIFOCount(port->line) < port->fifosize);
#else
	if (port->x_char) {
		regs->THB = port->x_char;
		port->icount.tx++;
		port->x_char=0;
		return;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		nx_uart_ops_stop_tx(port);
		return;
	}

#if 0
	do {
		tmp =  xmit->buf[xmit->tail];
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
		uart->regs->THB = tmp; /* ksw : I hope this would prevent reasserting IRQ, send chars twice, maybe... */
		if (uart_circ_empty(xmit))
			break;
	} while ((((reg = uart->regs->FSTATUS) & NXP2120_FSTATUS_TX_FIFO_FULL) ? 16 : ((reg>> 4) & 0xF)) < port->fifosize);
#else
    while (0 == (regs->FSTATUS & NXP2120_FSTATUS_TX_FIFO_FULL)) { /* This will prevent XMIT data loss while IRQ is generated at somehow FIFO FULL */
        tmp =  xmit->buf[xmit->tail];
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
		regs->THB = tmp; /* ksw : I hope this would prevent reasserting IRQ, send chars twice, maybe... */
		if (uart_circ_empty(xmit))
			break;
    }
#endif
	//} while (NX_UART_GetTxFIFOCount(port->line) < port->fifosize);
#endif

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	if (uart_circ_empty(xmit))
		nx_uart_ops_stop_tx(port);

	DBGINTR(" txe...");
}

static void nx_uart_err_irq_handler(struct uart_port *port)
{
	//printk(KERN_ERR "%s (line:%d): uart error irq, not implement \n", __func__, port->line);
}

static void nx_uart_err_mod_handler(struct uart_port *port)
{
	printk(KERN_ERR "%s (line:%d): uart modem irq, not implement \n", __func__, port->line);
}

static irqreturn_t nx_uart_irq_handler(int irq, void *dev_id)
{
//	struct nx_uart_port *port = dev_id;
	struct uart_port *port = dev_id;
	struct NX_UART_RegisterSet *regs;	
	regs = (struct NX_UART_RegisterSet *)port->membase;		

	unsigned int ctrl, pend, mask, reg;
//	printk("uart irq(port:%d, irq:%d)", port->line, irq);

	/* get pend and mask num */
	ctrl = regs->INTCON & 0xff;
	mask = ((ctrl & 0xf0) >> 4);
	pend = ( ctrl & 0x0f);

#if OLD_IMPLEMENTATION

#else
//	printk("pend = %x, RxFiFo = %d\n", pend, regs->FSTATUS & 0x0F);

	reg = regs->FSTATUS;
	//if (((pend & NXP2120_INTCON_RXPEND) == 0) && ((reg & 0x0F) > 8)) {
	//    printk("uart err : PENDING(%x) BIT DOES NOT SET : %x\n", pend, reg);
	//    pend |= NXP2120_INTCON_RXPEND;
	//}
	//if (port->regs->INTCON & NXP2120_INTCON_RXENB)
	if ( (pend & NXP2120_INTCON_RXPEND) ) {	// 1
		nx_uart_rx_irq_handler((struct uart_port *)port);

		/* clear pend */
		reg = regs->INTCON & 0xF0;
        regs->INTCON = reg | NXP2120_INTCON_RXPEND;
        regs->LCON = regs->LCON | NXP2120_LCON_SYNC_PENDCLR;
	}

	if ( pend & NXP2120_INTCON_TXPEND) {	// 0

		nx_uart_tx_irq_handler((struct uart_port *)port);

		/* clear pend */
		reg = regs->INTCON & 0xF0;
        regs->INTCON = reg | NXP2120_INTCON_TXPEND;
		regs->LCON = regs->LCON | NXP2120_LCON_SYNC_PENDCLR;
	}

	if ( pend & NXP2120_INTCON_ERRPEND ) { 	// 2

		nx_uart_err_irq_handler((struct uart_port *)port);

		/* clear pend */
		reg = regs->INTCON & 0xF0;
        regs->INTCON = reg | NXP2120_INTCON_ERRPEND;
		regs->LCON = regs->LCON | NXP2120_LCON_SYNC_PENDCLR;
	}

	if ( pend & NXP2120_INTCON_MPEND ) { 	// 3

		if (mask & (1 << UART_MOD_INTNUM))
			nx_uart_err_mod_handler((struct uart_port *)port);

		/* clear pend */
		reg = regs->INTCON & 0xF0;
        regs->INTCON = reg | NXP2120_INTCON_MPEND;
		regs->LCON = regs->LCON | NXP2120_LCON_SYNC_PENDCLR;
	}

	//port->regs->INTCON = reg;
	//port->regs->LCON = port->regs->LCON | NXP2120_LCON_SYNC_PENDCLR;
#endif

	DBGINTR(" exit\n");
	return IRQ_HANDLED;
}


/*---------------------------------------------------------------------------------------------
 * 	Uart console functions
 --------------------------------------------------------------------------------------------*/
static void nx_uart_putchar(struct uart_port *port, int ch)
{
#if OLD_IMPLEMENTATION
	while ( !(NX_UART_GetTxRxStatus(port->line) & NX_UART_TX_BUFFER_EMPTY) ) {
		/* busy wait */;
	}
	NX_UART_SendByte(port->line, (U8)ch);
#else
//    struct nx_uart_port *uart = (struct nx_uart_port *)port;
	struct NX_UART_RegisterSet *regs;	
	regs = (struct NX_UART_RegisterSet *)port->membase;	
	while ( 0 == (regs->TRSTATUS & NX_UART_TX_BUFFER_EMPTY)) {
		/* busy wait */;
	}
	regs->THB = ch;
#endif
}

static void nx_console_write(struct console *co, const char *s, unsigned int count)
{
//	struct uart_port *port = &nx_ports[co->index].port;
	struct uart_port *port;
	struct NX_UART_RegisterSet *regs;
	unsigned long flags;
	int locked;
	port = &ports[co->index];
	regs = (struct NX_UART_RegisterSet *)port->membase;


/*	local_irq_save(flags);
	if (port->sysrq) {
		// bcm_uart_interrupt() already took the lock 
		locked = 0;
	} else if (oops_in_progress) {
		locked = spin_trylock(&port->lock);
	} else {
		spin_lock(&port->lock);
		locked = 1;
	}*/

	/* call helper to deal with \r\n */
	uart_console_write(port, s, count, nx_uart_putchar);
	
	while ( 0 == (regs->TRSTATUS & NX_UART_TX_BUFFER_EMPTY)) {
		// busy wait ;
	}
	
/*	if (locked)
		spin_unlock(&port->lock);
	local_irq_restore(flags);*/
	
}

static int __init nx_console_setup(struct console *co, char *options)
{
#if 1
	struct uart_port *port;
	int baud   =  UART_BAUDRATE;
	int bits   =  8;
	int parity = 'n';
	int flow   = 'n';

	
	if (co->index < 0 || co->index >= NXP2120_NR_UARTS)
		return -EINVAL;
	
	port = &ports[co->index];
	if (!port->membase)
		return -ENODEV;
	
	uart_count++;	
	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

#else
//	struct nx_uart_port *uart;
//	struct uart_port *port;

/*	int baud   =  UART_BAUDRATE;
	int bits   =  8;
	int parity = 'n';
	int flow   = 'n';

	DBGOUT("%s (id:%d, opt:%s)\n", __func__, (int)co->index, options);

	if (co->index == -1 || co->index >= ARRAY_SIZE(nx_ports))
		co->index = 0;

	uart = &nx_ports[co->index];
	port = &uart->port;

	// uart module initialize 
	setup_uart_port(co->index);

	uart->count++;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	DBGOUT("console_setup (options:%s, baud:%d, parity:%d, bits:%d, flow:0x%x)\n",
		options, baud, parity, bits, flow);*/
#endif

	return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct uart_driver uart_drv;

static struct console nx_console = {
	.name		= UART_TERMINAL,
	.write		= nx_console_write,
	.device		= uart_console_device,
	.setup		= nx_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,			/* don't modify */
	.data		= &uart_drv,
};

static int __init nx_console_init(void)
{
	printk("##################%s\n", __func__);

//	init_uart_port();
	register_console(&nx_console);

	return 0;
}
console_initcall(nx_console_init);


#if 1
static void nxp2120_early_write(struct console *con, const char *s, unsigned n)
{
	struct earlycon_device *dev = con->data;
	struct uart_port *port;	
	struct NX_UART_RegisterSet *regs;	
	port = &ports[con->index];
	regs = (struct NX_UART_RegisterSet *)port->membase;
	
	uart_console_write(&dev->port, s, n, nx_uart_putchar);
/*	while ( 0 == (regs->TRSTATUS & NX_UART_TX_BUFFER_EMPTY)) {
		// busy wait ;
	}*/
	
}

static int __init nxp2120_early_console_setup(struct earlycon_device *device,
					  const char *opt)
{
	if (!device->port.membase)
		return -ENODEV;

	device->con->write = nxp2120_early_write;

	return 0;
}

OF_EARLYCON_DECLARE(UART_DEV_NAME, "nexell,nxp2120-uart", nxp2120_early_console_setup);

#endif
/*---------------------------------------------------------------------------------------------
 * 	Uart module init/exit functions
 --------------------------------------------------------------------------------------------*/
static struct uart_driver uart_drv = {
	.owner          = THIS_MODULE,
	.driver_name    = UART_DEV_NAME,
	.dev_name       = UART_TERMINAL,
	.major          = TTY_MAJOR,
	.minor          = 64,
	.nr             = ARRAY_SIZE(ports),
	.cons           = &nx_console,
};

/*---------------------------------------------------------------------------------------------
 * 	Uart platform driver functions
 --------------------------------------------------------------------------------------------*/
static int nx_uart_driver_probe(struct platform_device *pdev)
{
//	struct nx_uart_port *uart = ( struct nx_uart_port *)&nx_ports[pdev->id].port;
//	struct uart_port *port = &uart->port;
	struct NX_UART_RegisterSet *regs;
	struct uart_port *port;
	struct device_node *np;
	struct resource *res_mem, *res_irq;
	struct clk *clk;
	unsigned int id;
	int irq;
	int ret;

	printk("%s enter\n", __func__);
	np = pdev->dev.of_node;
	printk("pdev->id %d np->name %s fullname %s, properties->name %s\n", pdev->id, np->name, np->full_name, np->properties->name);
	if(np){		
		if(0 != of_property_read_u32(np, "id", &id)){
			printk("failed to get dt uart id\n");
			return -EINVAL;
		}
		pdev->id = id;
		printk("pdev->id %d\n", pdev->id);
	}
	
	if(pdev->id < 0 || pdev->id >= NXP2120_NR_UARTS){
		return -EINVAL;
	}
	
	port = &ports[pdev->id];
	if(port->membase)
		return -EBUSY;
	
	memset(port, 0, sizeof(*port));
	
	res_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res_mem)
		return -ENODEV;
	
	port->mapbase = res_mem->start;
	port->membase = devm_ioremap_resource(&pdev->dev, res_mem);
	if (IS_ERR(port->membase))
		return PTR_ERR(port->membase);

	res_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res_irq)
		return -ENODEV;

/*	clk = pdev->dev.of_node ? of_clk_get(pdev->dev.of_node, 0) :
				  clk_get(&pdev->dev, "uartclk0");
	if (IS_ERR(clk))
		return -ENODEV;*/
	regs = (struct NX_UART_RegisterSet *)port->membase;
//	printk("===================uart %d regs 0x%X\n", pdev->id, regs);
//	regs->CLKENB &= ~(NXP2120_UART_CLKENB_PCLKMODE | NXP2120_UART_CLKENB_CLKGENB);

		
	
	port->iotype = UPIO_MEM;
	port->irq = res_irq->start;
	port->ops = &nx_uart_ops;
	port->flags = UPF_BOOT_AUTOCONF;
	port->dev = &pdev->dev;
	port->fifosize = 16;
	port->uartclk = clk_get_rate(clk) / 2;
	port->line = pdev->id;
	clk_put(clk);

	ret = uart_add_one_port(&uart_drv, port);
	if (ret) {
		ports[pdev->id].membase = NULL;
		return ret;
	}

	printk("UART port init line=%d\n", port->line);
	
		
#if 0
	port->dev = &pdev->dev;

	uart_add_one_port(&uart_drv, port);
	platform_set_drvdata(pdev, uart);

	printk("UART port init line=%d\n", port->line);
#if (USE_DMA_CHAN0 || USE_DMA_CHAN1)
	/* setup DMA buffers */
	if (USE_DMA_CHAN0 && (0 == port->line)) {
	    int index;
	    if (!port->dev->dma_mask)
	        port->dev->dma_mask = &nx_uart_dmamask;
	    if (!port->dev->coherent_dma_mask)
	        port->dev->coherent_dma_mask = 0xffffffff;
	    index = 0;
	    uart->buf_base[index] = (unsigned int)dma_alloc_writecombine(port->dev, DMA_BUFFER_SIZE, (dma_addr_t *)&uart->dma_phys[index], GFP_KERNEL);
	    printk("index=%d buf_base = %x dma_phys = %x\n", index, uart->buf_base[index], uart->dma_phys[index]);
	    memset((void *)uart->buf_base[index], 0, DMA_BUFFER_SIZE);
	    index = 1;
	    uart->buf_base[index] = (unsigned int)dma_alloc_writecombine(port->dev, DMA_BUFFER_SIZE, (dma_addr_t *)&uart->dma_phys[index], GFP_KERNEL);
	    printk("index=%d buf_base = %x dma_phys = %x\n", index, uart->buf_base[index], uart->dma_phys[index]);
	    memset((void *)uart->buf_base[index], 0, DMA_BUFFER_SIZE);
	    irq = PB_DMA_IRQ(DMA_CHAN0_CHANNEL);
	    ret = request_irq(irq, uart_dma_irq_handler, IRQF_DISABLED, "UART0_DMA", (void*)uart);
		uart->dma_run = 0;
		spin_lock_init(&uart->lock);
	}
	if (USE_DMA_CHAN1 && (1 == port->line)) {
	    int index;
	    if (!port->dev->dma_mask)
	        port->dev->dma_mask = &nx_uart_dmamask;
	    if (!port->dev->coherent_dma_mask)
	        port->dev->coherent_dma_mask = 0xffffffff;
	    index = 0;
	    uart->buf_base[index] = (unsigned int)dma_alloc_writecombine(port->dev, DMA_BUFFER_SIZE, (dma_addr_t *)&uart->dma_phys[index], GFP_KERNEL);
	    printk("index=%d buf_base = %x dma_phys = %x\n", index, uart->buf_base[index], uart->dma_phys[index]);
	    memset((void *)uart->buf_base[index], 0, DMA_BUFFER_SIZE);
	    index = 1;
	    uart->buf_base[index] = (unsigned int)dma_alloc_writecombine(port->dev, DMA_BUFFER_SIZE, (dma_addr_t *)&uart->dma_phys[index], GFP_KERNEL);
	    printk("index=%d buf_base = %x dma_phys = %x\n", index, uart->buf_base[index], uart->dma_phys[index]);
	    memset((void *)uart->buf_base[index], 0, DMA_BUFFER_SIZE);
	    uart->last_addr = uart->dma_phys[0];
	    uart->last_buf  = uart->buf_base[0];
        uart->zero_cnt  = 0;
	    irq = PB_DMA_IRQ(DMA_CHAN1_CHANNEL);
	    ret = request_irq(irq, uart_dma_irq_handler, IRQF_DISABLED, "UART1_DMA", (void*)uart);
		uart->dma_run = 0;
		spin_lock_init(&uart->lock);
	}
	//nxp2120_serial_wq = create_singlethread_workqueue("nxp2120_serial_wq");
	//if (!nxp2120_serial_wq)
	//	return -ENOMEM;
	//INIT_WORK(&uart->work, nxp2120_serial_work_func);
	if (!timer_init) {
	    time = ktime_set(0, 2500000); /* 2.5ms */

	    hrtimer_init(&hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	    hrtimer.function = &hrtimer_action;
	    hrtimer_start(&hrtimer, time, HRTIMER_MODE_REL);
	    timer_init = 1;
	}
#endif

#if IRQ_ALWAYS
#if USE_DMA_CHAN1 && !USE_DMA_CHAN0
	// Enable RX Interrupt
    if (port->line == 1) {/* ksw : for test */
		uart->regs->INTCON = 0;
    }
    else
    {
		uart->regs->INTCON = NXP2120_INTCON_RXENB;
    }
#endif
#if !USE_DMA_CHAN1 && USE_DMA_CHAN0
	// Enable RX Interrupt
    if (port->line == 0) {/* ksw : for test */
		uart->regs->INTCON = 0;
    }
    else
    {
		uart->regs->INTCON = NXP2120_INTCON_RXENB;
    }
#endif
#if !USE_DMA_CHAN1 && !USE_DMA_CHAN0
	uart->regs->INTCON = NXP2120_INTCON_RXENB;
#endif
	uart->regs->INTCON = uart->regs->INTCON | 0x0F; /* Clear Interrupt Pending all */
	ret = request_irq(port->irq, nx_uart_irq_handler, IRQF_DISABLED, UART_DEV_NAME, port);
	if ( ret!=0 ) {
		dev_err(port->dev, "unable to grab irq%d\n",port->irq);
		return -ENODEV;
	}
#endif
#endif

	return 0;
}

static int nx_uart_driver_remove(struct platform_device *pdev)
{
//	struct nx_uart_port *uart = platform_get_drvdata(pdev);
//	struct uart_port *port = &uart->port;
	struct uart_port *port;
	port = platform_get_drvdata(pdev);

	printk("%s (id:%d)\n", __func__, pdev->id);
	
	uart_remove_one_port(&uart_drv, port);	
	/* mark port as free */
	ports[pdev->id].membase = NULL;
#if 0
#if IRQ_ALWAYS
	free_irq(port->irq, port);
#endif

#if 0
#if (USE_DMA_CHAN0 || USE_DMA_CHAN1)
    if (USE_DMA_CHAN0 && (0 == port->line)) {
        /* Free DMA buffers */
        if (uart->dma_tr)
            soc_dma_release(uart->dma_tr);
        if (uart->buf_base[0])
            dma_free_writecombine(port->dev, DMA_BUFFER_SIZE, (void *)uart->buf_base[0], uart->dma_phys[0]);
        if (uart->buf_base[1])
            dma_free_writecombine(port->dev, DMA_BUFFER_SIZE, (void *)uart->buf_base[1], uart->dma_phys[1]);
    }
    if (USE_DMA_CHAN1 && (1 == port->line)) {
        /* Free DMA buffers */
        if (uart->dma_tr)
            soc_dma_release(uart->dma_tr);
        if (uart->buf_base[0])
            dma_free_writecombine(port->dev, DMA_BUFFER_SIZE, (void *)uart->buf_base[0], uart->dma_phys[0]);
        if (uart->buf_base[1])
            dma_free_writecombine(port->dev, DMA_BUFFER_SIZE, (void *)uart->buf_base[1], uart->dma_phys[1]);
    }
    //if (nxp2120_serial_wq)
	//	destroy_workqueue(nxp2120_serial_wq);
#endif
#endif
//	platform_set_drvdata(pdev, NULL);
	if (port) {
		uart_remove_one_port(&uart_drv, port);
	}
#endif
	return 0;
}

static int nx_uart_driver_suspend(struct platform_device *pdev, pm_message_t state)
{
//	struct nx_uart_port *uart = platform_get_drvdata(pdev);
//	struct uart_port *port = &uart->port;
//	PM_DBGOUT("+%s (line:%d)\n", __func__, port->line);

	struct uart_port *port;
	struct NX_UART_RegisterSet *regs;	
	regs = (struct NX_UART_RegisterSet *)port->membase;		
	
	port = platform_get_drvdata(pdev);

	if (port) {
		uart_suspend_port(&uart_drv, port);

#if OLD_IMPLEMENTATION
		NX_UART_ResetTxFIFO(port->line);
		NX_UART_ResetRxFIFO(port->line);
#else
        regs->FCON |= (NXP2120_FCON_TX_FIFO_RESET | NXP2120_FCON_RX_FIFO_RESET);
#endif
	}

//	PM_DBGOUT("-%s\n", __func__);
	return 0;
}

static int nx_uart_driver_resume(struct platform_device *pdev)
{
//	struct nx_uart_port *uart = platform_get_drvdata(pdev);
//	struct uart_port *port = &uart->port;
	struct uart_port *port;
	port = platform_get_drvdata(pdev);
//	PM_DBGOUT("+%s (line:%d)\n", __func__, port->line);

	if (port) {
		uart_resume_port(&uart_drv, port);
	}

//	PM_DBGOUT("-%s\n", __func__);
	return 0;
}

static const struct of_device_id nxp2120_match_table[] = {
	{ .compatible = "nexell,nxp2120-uart" },
	{}
};
MODULE_DEVICE_TABLE(of, nxp2120_match_table);

static struct platform_driver uart_plat_drv = {
	.probe      = nx_uart_driver_probe,
	.remove     = nx_uart_driver_remove,
	.suspend	= nx_uart_driver_suspend,
	.resume		= nx_uart_driver_resume,
	.driver		= {
		.name   = UART_DEV_NAME,
		.of_match_table = nxp2120_match_table,
	},
};

#if 1

static int _atoi(const char *name)
{
    int val = 0;

    for (;; name++) {
		switch (*name) {
	    	case '0' ... '9':
				val = 10*val+(*name-'0');
			break;
	    	default:
			return val;
		}
    }
}

static int serial_line   = 0;
static int serial_enable = 1;

static ssize_t serial_show(struct kobject *kobj, struct kobj_attribute *attr,
				char *buf)
{
	char *s = buf;
	DBGOUT("%s:\n", __func__);

	s += sprintf(s, "%d %s\n", serial_line, serial_enable ? "on" : "off");
	if (s != buf)
		*(s-1) = '\n';

	return (s - buf);
}

/*
 * #> echo 0 on  > /sys/_con/_con
 * #> echo 0 off > /sys/_con/_con
 */
static ssize_t serial_store(struct kobject *kobj, struct kobj_attribute *attr,
				const char *buf, size_t n)
{
	struct uart_port *port = NULL;
	char *ps, *pe;
	int len, line;

	ps = memchr(buf,  ' ', n);
	pe = memchr(buf, '\n', n);

	/* check argu */
	if (NULL == ps || NULL == pe)
		return n;

	/* get line num */
	line = _atoi(buf);

	if (line >= ARRAY_SIZE(ports) || 0 > line)
		return n;

	ps  = memchr(buf, 'o', n);
	len = pe - ps;

	port = &ports[line];

#if 0
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
	if (port->state->port.tty != get_current_tty())
		return n;
#else
	if (port->info->port.tty != get_current_tty())
		return n;
#endif
#endif

	/* start serial */
	if (len == 2 && !strncmp(ps, "on", len)) {
		serial_line   = line;
		serial_enable = 1;

		console_start(&nx_console);		// register_console(&nx_console);
#if 0
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
		tty_ldisc_flush(port->state->port.tty);
		start_tty(port->state->port.tty);
#else
		tty_ldisc_flush(port->info->port.tty);
		start_tty(port->info->port.tty);
#endif
#endif
	}

	/* stop serial */
	if (len == 3 && !strncmp(ps, "off", len)) {
		serial_line   = line;
		serial_enable = 0;

#if 0
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
		stop_tty(port->state->port.tty);
#else
		stop_tty(port->info->port.tty);
#endif
#endif
		console_stop(&nx_console);		// unregister_console(&nx_console);
	}

	return n;
}

static struct kobj_attribute serial_attr =
			__ATTR(serial, 0644, serial_show, serial_store);

static struct attribute * g[] = {
	&serial_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};

struct kobject * con_kobj;

static int __init nx_serial_driver_init(void)
{
	int ret;

	DBGOUT("%s\n", __func__);

	ret = uart_register_driver(&uart_drv);
	if (ret) {
		printk(KERN_ERR "serial: failed to register uart device (%d) \n", ret);
		return ret;
	}

	ret = platform_driver_register(&uart_plat_drv);
	if (ret != 0) {
		printk(KERN_ERR "serial: failed to register platform driver (%d) \n", ret);
		uart_unregister_driver(&uart_drv);
	}

	con_kobj = kobject_create_and_add("console", NULL);
	if (! con_kobj)
		return -ENOMEM;

	return sysfs_create_group(con_kobj, &attr_group);
}

static void __exit nx_serial_driver_exit(void)
{
	printk("%s\n", __func__);
	platform_driver_unregister(&uart_plat_drv);
	uart_unregister_driver(&uart_drv);
}

module_init(nx_serial_driver_init);
module_exit(nx_serial_driver_exit);
#endif

MODULE_DESCRIPTION("Serial driver for Nexell");
MODULE_LICENSE("GPL");



/*
 * Copyright (C) 2019 I4VINE
 * Author: Juyoung Ryu <jyryu@i4vine.com>	
 * Support
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

#include <linux/init.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/irqchip.h>

#include <asm/io.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <linux/vmalloc.h>
#include <asm/pgtable.h>

/* nexell soc headers */
//#include <mach/nx_soc.h>
#include "nx_clkpwr.h"
//#include <mach/platform.h>
//#include <mach/map_desc.h>
extern void lldebugout(const char *fmt, ...);
#if (0)
#define DBGOUT(msg...)		{ printk("cpu: " msg); }
#define _IOMAP()		{							\
	int i = 0;										\
	for (; i<ARRAY_SIZE(cpu_iomap_desc); i++) 		\
		printk(KERN_INFO "cpu: iomap[%2d]: p 0x%08x -> v 0x%08x len=0x%x\n", i,	\
			(u_int)(cpu_iomap_desc[i].pfn<<12),		\
			(u_int)(cpu_iomap_desc[i].virtual),		\
			(u_int)(cpu_iomap_desc[i].length));		\
	}
#else
#define DBGOUT(msg...)		do {} while (0)
#define _IOMAP()
#endif

//#define DBGOUT	lldebugout
#define	CFG_MEM_PHY_SYSTEM_BASE			0x80000000	/* System, must be at an evne 2MB boundary (head.S) */
#define	CFG_MEM_PHY_SYSTEM_SIZE			(0x0F000000)

/* check boot mode */
//extern u_int cpu_get_reserve_pm_mem(int phys);

void boot_mode(void)
{
	unsigned int  base = 1;//cpu_get_reserve_pm_mem(0); /* ksw: We need sleep mode now */
	unsigned int  csum = 0, size = 0;
	unsigned int *data = NULL;

	int  val = 0, xor = 0, i = 0;
	bool coldboot = true;
	unsigned int  scratch  = 0;

	if (! base) {
		DBGOUT("Boot mode : Cold boot ...\n");
		return;
	}

	csum 	 = *(unsigned int*)(base + 0x0);
	size 	 = *(unsigned int*)(base + 0x4);
	data 	 =  (unsigned int*)(base + 0x8);
/*	scratch  = NX_ALIVE_GetScratchReg();

	if (CFG_PWR_SLEEP_SIGNATURE == scratch) {

		if (size <= 0 || size >= 0x100) {
			coldboot = true;
		} else {
			for (i = 0; i<size; i++, data++)
				xor = (*data ^ 0xAA), val ^= xor;

			if (csum != val)
				coldboot = true;
			else
				coldboot = false;
		}
	}
*/
	DBGOUT("Boot mode : %s boot [0x%08x, 0x%08x, 0x%08x]...\n",
		coldboot==true?"Cold":"Warm", scratch, csum, val);
	//return (coldboot == false);
}
#if 0
static void nxp_cpu_shutdown(void)
{
	arch_halt();
}
#endif

#if 0
static inline void arch_halt(void)
{
    struct NX_UART_RegisterSet *uart;
	char	notify[] = "GOODBYE";
	int		i = 0;
	int		cnt = 0;

	setup_uart_port1();
	uart1_set_termios();

	uart = (struct NX_UART_RegisterSet *)IO_ADDRESS(PHY_BASEADDR_UART1);

    for (cnt = 0 ; cnt < 10 ; cnt++)
    {
    	i = 0;
	    while (0 == (uart->FSTATUS & NXP2120_FSTATUS_TX_FIFO_FULL))
	    {
			uart->THB = notify[i++];
			if (notify[i] == 0)
				break;
	    }
	    msleep(100);
	}
}
#endif

static inline void arch_reset(char mode, const char *cmd)
{
 //   arch_halt();

	printk(KERN_INFO "system reset: %s ...\n", cmd);

	NX_CLKPWR_SetSoftwareResetEnable(CTRUE);
	NX_CLKPWR_DoSoftwareReset();
}


static void nxp_cpu_reset(char str, const char *cmd)
{
	printk(KERN_INFO "system reset: %s ...\n", cmd);

//	if (nxp_board_reset)
//		nxp_board_reset(str, cmd);

	arch_reset(str, cmd);
}

#if 0

/*------------------------------------------------------------------------------
 * 	cpu initialize and io/memory map.
 * 	procedure: fixup -> map_io -> init_irq -> timer init -> init_machine
 */
static void __init cpu_fixup(struct tag *tags, char **cmdline, struct meminfo *mi)
{
	DBGOUT("%s\n", __func__);
#if 0
	/*
	 * system momory  = system_size + linear_size
	 */
    mi->nr_banks     	= 1;
	mi->bank[0].start 	= CFG_MEM_PHY_SYSTEM_BASE;
	mi->bank[0].size	= CFG_MEM_PHY_SYSTEM_SIZE; // + CFG_MEM_PHY_LINEAR_SIZE;
#endif
	printk("PHY_MEM_SIZE=%X\n", CFG_MEM_PHY_SYSTEM_SIZE);
}
#endif

#if 0
static struct map_desc nxp2120_io_desc[] __initdata = {
	{
		.virtual	= (unsigned long) NXP2120_REGS_VIRT_BASE,
		.pfn		= __phys_to_pfn(NXP2120_REGS_PHYS_BASE),
		.length		= NXP2120_REGS_SIZE,
		.type		= MT_DEVICE,
	},
};

void __init cpu_map_io(void)
{
	iotable_init(nxp2120_io_desc, ARRAY_SIZE(nxp2120_io_desc));
}
#endif


static void __init nxp2120_init_irq(void)
{
	irqchip_init();
}

static void __init cpu_init_machine(void)
{
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
	DBGOUT("=================cpu_init_machine done\n");
}

static const char * const cpu_dt_compat[] = {
	"nexell,nxp2120",
	NULL
};
/*------------------------------------------------------------------------------
 * Maintainer: Nexell Co., Ltd.
 */

DT_MACHINE_START(NXP2120, "nxp2120")
//	.fixup			=  cpu_fixup,
//	.map_io			=  cpu_map_io,
#if defined(CONFIG_CPU_NXP2120)
	.init_irq	= nxp2120_init_irq,
#endif
	//.init_early	    = nxp4330_init_early,
//	.timer			= &cpu_sys_timer,
	.init_machine	= cpu_init_machine,
	.dt_compat	= cpu_dt_compat,
	.restart	= nxp_cpu_reset,
MACHINE_END

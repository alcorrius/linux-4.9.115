 

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/irq.h>
#include <linux/gpio.h>

//#include <mach/nx_soc.h>
#include <mach/nx_mcus.h>
#include "nx_clkpwr.h"


typedef unsigned int U32;

struct timing {
	U32 BitWidth;
	U32 tACS;
	U32 tCAH;
	U32 tCOS;
	U32 tCOH;
	U32 tACC;
	U32 tSACC;
	U32 WaitMode;
	U32 BurstRead;
	U32 BurstWrite;
};
 

static int parse_static_timing_dt(struct device_node *np, struct timing *tm)
{
	if (of_property_read_u32(np, "bus_width", &tm->BitWidth)) {
		return -1;
	}
	if (of_property_read_u32(np, "tACS", &tm->tACS)) {
		return -1;
	}
	if (of_property_read_u32(np, "tCAH", &tm->tCAH)) {
		return -1;
	}
	if (of_property_read_u32(np, "tCOS", &tm->tCOS)) {
		return -1;
	}
	if (of_property_read_u32(np, "tCOH", &tm->tCOH)) {
		return -1;
	}
	if (of_property_read_u32(np, "tACC", &tm->tACC)) {
		return -1;
	}
	if (of_property_read_u32(np, "tSACC", &tm->tSACC)) {
		return -1;
	}
	if (of_property_read_u32(np, "wm", &tm->WaitMode)) {
		return -1;
	}
    if (of_property_read_u32(np, "rb", &tm->BurstRead)) {
		return -1;
	}
    if (of_property_read_u32(np, "wb", &tm->BurstWrite)) {
		return -1;
	}
	return 0;
}

static const struct of_device_id mcus_ids[] __initconst = {
	{ .compatible = "nexell,nxp2120-mcus", },
	{ /* centinel */},
};

static const struct of_device_id mcua_ids[] __initconst = {
	{ .compatible = "nexell,nxp2120-mcua", },
	{ /* centinel */},
};

static const struct of_device_id cpm_ids[] __initconst = {
	{ .compatible = "nexell,nxp2120,pll", },
	{ /* centinel */},
};
/* MCUD For Dynamic Memory. */
/* MCUS for Static Memory. */
// intialize soc bus status.
void __init bus_init(int flag)
{
	/*
	 * Update Fast Channel Arbiter
	 */
	#if (0)
	{
		NX_MCUD_SetFastArbiter(  0, (NX_MCUD_FAST)CFG_SYS_MCUD_FASTCH_00);
		NX_MCUD_SetFastArbiter(  1, (NX_MCUD_FAST)CFG_SYS_MCUD_FASTCH_01);
		NX_MCUD_SetFastArbiter(  2, (NX_MCUD_FAST)CFG_SYS_MCUD_FASTCH_02);
		NX_MCUD_SetFastArbiter(  3, (NX_MCUD_FAST)CFG_SYS_MCUD_FASTCH_03);
		NX_MCUD_SetFastArbiter(  4, (NX_MCUD_FAST)CFG_SYS_MCUD_FASTCH_04);
		NX_MCUD_SetFastArbiter(  5, (NX_MCUD_FAST)CFG_SYS_MCUD_FASTCH_05);
		NX_MCUD_SetFastArbiter(  6, (NX_MCUD_FAST)CFG_SYS_MCUD_FASTCH_06);
		NX_MCUD_SetFastArbiter(  7, (NX_MCUD_FAST)CFG_SYS_MCUD_FASTCH_07);
		NX_MCUD_SetFastArbiter(  8, (NX_MCUD_FAST)CFG_SYS_MCUD_FASTCH_08);
		NX_MCUD_SetFastArbiter(  9, (NX_MCUD_FAST)CFG_SYS_MCUD_FASTCH_09);
		NX_MCUD_SetFastArbiter( 10, (NX_MCUD_FAST)CFG_SYS_MCUD_FASTCH_10);
		NX_MCUD_SetFastArbiter( 11, (NX_MCUD_FAST)CFG_SYS_MCUD_FASTCH_11);
		NX_MCUD_SetFastArbiter( 12, (NX_MCUD_FAST)CFG_SYS_MCUD_FASTCH_12);
		NX_MCUD_SetFastArbiter( 13, (NX_MCUD_FAST)CFG_SYS_MCUD_FASTCH_13);
		NX_MCUD_SetFastArbiter( 14, (NX_MCUD_FAST)CFG_SYS_MCUD_FASTCH_14);
		NX_MCUD_SetFastArbiter( 15, (NX_MCUD_FAST)CFG_SYS_MCUD_FASTCH_15);
	}
	#endif

	/*
	 * Update Slow Channel Arbiter
	 */
	#if (0)
	{
		NX_MCUD_SetSlowArbiter(  0, (NX_MCUD_SLOW)CFG_SYS_MCUD_SLOWCH_00);
		NX_MCUD_SetSlowArbiter(  1, (NX_MCUD_SLOW)CFG_SYS_MCUD_SLOWCH_01);
		NX_MCUD_SetSlowArbiter(  2, (NX_MCUD_SLOW)CFG_SYS_MCUD_SLOWCH_02);
		NX_MCUD_SetSlowArbiter(  3, (NX_MCUD_SLOW)CFG_SYS_MCUD_SLOWCH_03);
		NX_MCUD_SetSlowArbiter(  4, (NX_MCUD_SLOW)CFG_SYS_MCUD_SLOWCH_04);
		NX_MCUD_SetSlowArbiter(  5, (NX_MCUD_SLOW)CFG_SYS_MCUD_SLOWCH_05);
		NX_MCUD_SetSlowArbiter(  6, (NX_MCUD_SLOW)CFG_SYS_MCUD_SLOWCH_06);
		NX_MCUD_SetSlowArbiter(  7, (NX_MCUD_SLOW)CFG_SYS_MCUD_SLOWCH_07);
		NX_MCUD_SetSlowArbiter(  8, (NX_MCUD_SLOW)CFG_SYS_MCUD_SLOWCH_08);
		NX_MCUD_SetSlowArbiter(  9, (NX_MCUD_SLOW)CFG_SYS_MCUD_SLOWCH_09);
		NX_MCUD_SetSlowArbiter( 10, (NX_MCUD_SLOW)CFG_SYS_MCUD_SLOWCH_10);
		NX_MCUD_SetSlowArbiter( 11, (NX_MCUD_SLOW)CFG_SYS_MCUD_SLOWCH_11);
		NX_MCUD_SetSlowArbiter( 12, (NX_MCUD_SLOW)CFG_SYS_MCUD_SLOWCH_12);
		NX_MCUD_SetSlowArbiter( 13, (NX_MCUD_SLOW)CFG_SYS_MCUD_SLOWCH_13);
		NX_MCUD_SetSlowArbiter( 14, (NX_MCUD_SLOW)CFG_SYS_MCUD_SLOWCH_14);
		NX_MCUD_SetSlowArbiter( 15, (NX_MCUD_SLOW)CFG_SYS_MCUD_SLOWCH_15);
	}
	#endif	
}

int __init soc_init(void)
{
//	printk("%s enter\n", __func__);
	bus_init(0);
//	pll_init();

//	pll_info();
	//boot_mode();
//	printk("%s exit\n", __func__);	
	return 0;
}
arch_initcall(soc_init);

int __init mcus_of_init(void)
{
	struct device_node *np, *child_node;
	struct resource res;
	void __iomem *base;
	struct timing tm;
	int ret;
//	printk("%s enter\n", __func__);
	NX_MCUS_Initialize();
	/* from of read base address and init */
	np = of_find_matching_node(NULL, mcus_ids);
	if (!np)
		return -ENODEV;

	if (of_address_to_resource(np, 0, &res))
		return -ENODEV;

	printk("mcus of init with base address 0x%X\n", res.start);
	base = ioremap(res.start, resource_size(&res));
	printk("mcus virtual addr=%x\n", base);

	NX_MCUS_SetBaseAddress((U32)base);

	/* Set Static memories's timing from of/dt */
	child_node = of_find_node_by_name(np, "static_0");
	ret = parse_static_timing_dt(child_node, &tm);
	NX_MCUS_SetStaticBUSConfig(
			NX_MCUS_SBUSID_STATIC0, tm.BitWidth,
			tm.tACS, tm.tCAH, tm.tCOS, tm.tCOH, tm.tACC, tm.tSACC,
			tm.WaitMode,tm.BurstRead, tm.BurstWrite );

	/*child_node = of_find_node_by_name(np, "static_1");
	ret = parse_static_timing_dt(child_node, &tm);
	NX_MCUS_SetStaticBUSConfig(
                NX_MCUS_SBUSID_STATIC1, tm.BitWidth,
        	tm.tACS, tm.tCAH, tm.tCOS, tm.tCOH, tm.tACC, tm.tSACC,
        	tm.WaitMode,tm.BurstRead, tm.BurstWrite );*/

	/* Are you sure set timing of nand here ? */
	child_node = of_find_node_by_name(np, "nand");
	ret = parse_static_timing_dt(child_node, &tm);
	NX_MCUS_SetStaticBUSConfig(
			NX_MCUS_SBUSID_NAND, tm.BitWidth,
			tm.tACS, tm.tCAH, tm.tCOS, tm.tCOH, tm.tACC, tm.tSACC,
			tm.WaitMode,tm.BurstRead, tm.BurstWrite );

	/* Now initialize CLKPWR */
	NX_CLKPWR_Initialize();

	np = of_find_matching_node(NULL, cpm_ids);
	if (!np)
		return -ENODEV;

	if (of_address_to_resource(np, 0, &res))
		return -ENODEV;

	printk("clockpower module of init with base address 0x%X\n", res.start);
	base = ioremap(res.start, resource_size(&res));
	NX_CLKPWR_SetBaseAddress((U32)base);
//	printk("%s exit\n", __func__);
	return 0;
}

arch_initcall(mcus_of_init);
/*
 * Copyright (C) 2019  I4VINE Inc.,
 * All right reserved by Seungwoo Kim  <ksw@i4vine.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <asm/tlbflush.h>

/* ksw : I only implement this nxp2120 pll to read information only. */

#include "dt-bindings/clk/nxp2120-clk.h"

/* ksw : we only have 2 plls and axiclk, pclk, bclk, mclk */

#define ID_CPU_PLL0	0
#define ID_CPU_PLL1	1
#define ID_FCLK		2
#define ID_HCLK		3
#define ID_MCLK		4
#define ID_BCLK		5
#define ID_PCLK		6
#define ID_XTI		7
#define ID_END		8

#define CLK_CPU_PLL0	"pll0"
#define CLK_CPU_PLL1	"pll1"
#define CLK_FCLK		"cpuclk"
#define CLK_HCLK		"hclk"
#define CLK_MCLK		"mclk"
#define CLK_BCLK		"bclk"
#define CLK_PCLK		"pclk"
#define CLK_XTI			"xticlk"

/* 
	normally, 
	AXICLK = CORECLOCK / 4
	MCLK = FCLK / 2
	BCLK = MCLK / 2
	PCLK = BCLK / 2
*/

#define PLL_LOCKING_TIME 100

struct nx_cpm_reg_t {
	volatile unsigned int clkmode0;				/* 0xC000F000 */
	volatile unsigned int clkmode1;				/* 0xC000F004 */
	volatile unsigned int pllset[2];			/* 0xC000F008 - 0x0C */
	unsigned int reserved0[(0x40-0x0C) / 4 - 1];
	volatile unsigned int gpio_wakeup_rise_enb;	/* 0cC000F040 */
	volatile unsigned int gpio_wakeup_fall_enb;	/* 0cC000F044 */
	volatile unsigned int gpio_rst_enb;			/* 0cC000F048 */
	volatile unsigned int gpio_wake_enb;		/* 0cC000F04C */
	volatile unsigned int gpio_int_enb;			/* 0cC000F050 */
	volatile unsigned int gpio_int_pend;		/* 0cC000F054 */
	volatile unsigned int reset_status;			/* 0cC000F058 */
	volatile unsigned int int_enb;				/* 0cC000F05C */
	volatile unsigned int int_pend;				/* 0cC000F060 */
	volatile unsigned int pwr_cont;				/* 0cC000F064 */
	volatile unsigned int pwr_mode;				/* 0cC000F068 */
	unsigned int reserved1;						/* 0cC000F06C */
	volatile unsigned int scratch0;				/* 0xC000F070 */
	volatile unsigned int scratch1;				/* 0xC000F074 */
	volatile unsigned int scratch2;				/* 0xC000F078 */
	volatile unsigned int sys_rst_config;		/* 0xC000F07C */
};

#define CLK_MODE0_PLL1_PWDN			(1 << 30)
#define CLK_MODE0_W_CLKDIV2CPU0(x)	(((x-1) << 8) & (0xF << 8))
#define CLK_MODE0_W_CLKSELCPU0(x)	(((x-1) << 4) & (0x3 << 4))
#define CLK_MODE0_W_CLKDIV1CPU0(x)	(((x-1) << 0) & (0xF << 0))
#define CLK_MODE0_R_CLKDIV2CPU0(x)	((((x) >> 8) & 0xF) + 1)
#define CLK_MODE0_R_CLKSELCPU0(x)	(((x) >> 4) & 0x3)
#define CLK_MODE0_R_CLKDIV1CPU0(x)	((((x) >> 0) & 0xF) + 1)


#define CLK_MODE1_W_CLKDIVPCLK(x)	(((x-1) << 12) & (0x0F << 12))
#define CLK_MODE1_W_CLKDIVBCLK(x)	(((x-1) << 8) & (0x0F << 8))
#define CLK_MODE1_W_CLKSELMCLK(x)	(((x) << 4) & (0x3 << 4))
#define CLK_MODE1_W_CLKDIVMCLK(x)	(((x-1) << 0) & (0xF << 0))
#define CLK_MODE1_R_CLKDIVPCLK(x)	((((x) >> 12) & 0x0F) + 1)
#define CLK_MODE1_R_CLKDIVBCLK(x)	((((x) >> 8) & 0x0F) + 1)
#define CLK_MODE1_R_CLKSELMCLK(x)	(((x) >> 4) & 0x3)
#define CLK_MODE1_R_CLKDIVMCLK(x)	((((x) >> 0) & 0xF) + 1)

#define PLLSET_W_PDIV(x)			(((x) << 18) & (0x3F << 18))
#define PLLSET_W_MDIV(x)			(((x) << 8) & (0x3FF << 8))
#define PLLSET_W_SDIV(x)			(((x) << 0) & (0xFF << 0))

#define PLLSET_R_PDIV(x)			(((x) >> 18) & 0x3F)
#define PLLSET_R_MDIV(x)			(((x) >> 8) & 0x3FF)
#define PLLSET_R_SDIV(x)			(((x) >> 0) & 0xFF)

struct pll_pms {
	long rate; /* unint Khz */
	int P;
	int M;
	int S;
};

struct clk_core {
	const char *name;
	int id, div, pll;
	unsigned long rate;
	struct clk_hw hw;
	struct pll_pms pms;
};

#define to_clk_core(_hw) container_of(_hw, struct clk_core, hw)

#define PMS_RATE(p, i) ((&p[i])->rate)
#define PMS_P(p, i) ((&p[i])->P)
#define PMS_M(p, i) ((&p[i])->M)
#define PMS_S(p, i) ((&p[i])->S)

#define PLL_S_BITPOS 0
#define PLL_M_BITPOS 8
#define PLL_P_BITPOS 18

static struct nx_cpm_reg_t *cpm_reg;

static void nx_pll_set_rate(int PLL, int P, int M, int S)
{
	/* We don't support pll set now */
	/* Don't do anything */
}

static unsigned long pll_round_rate(int pllno, unsigned long rate, int *p,
				    int *m, int *s)
{
	return 0;	
}

static unsigned long ref_clk = 12000000UL;

#define getquotient(v, d) (v / d)

#define DIV_CPUG0 0
#define DIV_BUS 1
#define DIV_MEM 2
#define DIV_G3D 3
#define DIV_VPU 4
#define DIV_DISP 5
#define DIV_HDMI 6
#define DIV_CPUG1 7
#define DIV_CCI4 8

#define DVO0 3
#define DVO1 9
#define DVO2 15
#define DVO3 21

static inline unsigned int pll_rate(unsigned int pllN, unsigned int xtal)
{
	struct nx_cpm_reg_t *reg = cpm_reg;
	unsigned int val, val1, nP, nM, nS;
	unsigned int temp = 0;

	val = reg->pllset[pllN];
	xtal /= 1000; /* Unit Khz */

	nP = PLLSET_R_PDIV(val);
	nM = PLLSET_R_MDIV(val);
	nS = PLLSET_R_SDIV(val);

	return (unsigned int)(nM * getquotient(xtal, (nP * 1 << nS)) * 1000);
}

#define PLLN_RATE(n) (pll_rate(n, ref_clk)) /* 0~ 1 */

/*
 *	core frequency clk interface
 */
static struct clk_core clk_pll_dev[] = {
	[ID_CPU_PLL0] =	{
		.id = ID_CPU_PLL0, .name = CLK_CPU_PLL0,
	},
	[ID_CPU_PLL1] =	{
		.id = ID_CPU_PLL1, .name = CLK_CPU_PLL1,
	},
	[ID_FCLK] = {.id = ID_FCLK,
		.name = CLK_FCLK,
		.div = 1},
	[ID_HCLK] = {.id = ID_HCLK,
		.name = CLK_HCLK,
		.div = 4},
	[ID_MCLK] = {.id = ID_MCLK,
		.name = CLK_MCLK,
		.div = 2},
	[ID_BCLK] = {.id = ID_BCLK,
		.name = CLK_BCLK,
		.div = 2},
	[ID_PCLK] = {.id = ID_PCLK,
		.name = CLK_PCLK,
		.div = 2},
	[ID_XTI] = {.id = ID_XTI,
		.div = 1},
};

static unsigned long get_mclk(void)
{
	int sel;
	unsigned int cm1;
	unsigned long rate;

	cm1 = cpm_reg->clkmode1;
	/* Get PLL No */
	sel = CLK_MODE1_R_CLKSELMCLK(cm1);
	/* Get PLL rate */
	if (3 == sel) {
		/* Get FLCK first */
		unsigned int cm0;

		cm0 = cpm_reg->clkmode0;
		sel = CLK_MODE0_R_CLKSELCPU0(cm0);
		/* Get PLL rate */
		rate = PLLN_RATE(sel);
		/* Div PLL with CLKDIV1 */
		rate = rate / CLK_MODE0_R_CLKDIV1CPU0(cm0);
	} else {
		rate = PLLN_RATE(sel);
	}
	rate = rate / CLK_MODE1_R_CLKDIVMCLK(cm1);

	return rate;
}

static unsigned long clk_pll_recalc_rate(struct clk_hw *hw, unsigned long rate)
{
	struct clk_core *clk_data = (struct clk_core *)to_clk_core(hw);
	int id = clk_data->id;

	switch (id) {
	case ID_CPU_PLL0:
		rate = PLLN_RATE(0);
		break;
	case ID_CPU_PLL1:
		rate = PLLN_RATE(1);
		break;
	case ID_FCLK:
		{
			int sel;
			unsigned int cm0;

			cm0 = cpm_reg->clkmode0;
			/* Get PLL No */
			sel = CLK_MODE0_R_CLKSELCPU0(cm0);
			/* Get PLL rate */
			rate = PLLN_RATE(sel);
			/* Div PLL with CLKDIV1 */
			rate = rate / CLK_MODE0_R_CLKDIV1CPU0(cm0);
		}
		break;
	case ID_HCLK:
		{
			int sel;
			unsigned int cm0;

			cm0 = cpm_reg->clkmode0;
			/* Get PLL No */
			sel = CLK_MODE0_R_CLKSELCPU0(cm0);
			/* Get PLL rate */
			rate = PLLN_RATE(sel);
			/* Div PLL with CLKDIV1 */
			rate = rate / CLK_MODE0_R_CLKDIV1CPU0(cm0) / CLK_MODE0_R_CLKDIV2CPU0(cm0);
		}
		break;
	case ID_MCLK:
		rate = get_mclk();
		break;
	case ID_BCLK:
		{
			unsigned int cm1;

			cm1 = cpm_reg->clkmode1;

			rate = get_mclk();
			rate = rate / CLK_MODE1_R_CLKDIVBCLK(cm1);
		}
		break;
	case ID_PCLK:
		{
			unsigned int cm1;

			cm1 = cpm_reg->clkmode1;

			rate = get_mclk();
			rate = rate / CLK_MODE1_R_CLKDIVBCLK(cm1) / CLK_MODE1_R_CLKDIVPCLK(cm1);	
		}
		break;
	case ID_XTI:
		{
			rate = ref_clk;
		}
		break;
	default:
		pr_info("Unknown clock ID [%d] ...\n", id);
		break;
	}

	pr_debug("%s: name %s id %d rate %ld\n", __func__, clk_data->name,
		 clk_data->id, rate);
	return rate;
}

static long clk_pll_round_rate(struct clk_hw *hw, unsigned long drate,
			       unsigned long *prate)
{
	struct clk_core *clk_data = (struct clk_core *)to_clk_core(hw);
	int id = clk_data->id;
	long rate = 0;

	rate = clk_pll_recalc_rate(hw, 0);

	return rate;
}

static int clk_pll_set_rate(struct clk_hw *hw, unsigned long drate,
			    unsigned long prate)
{
	struct clk_core *clk_data = (struct clk_core *)to_clk_core(hw);
	
	/* Don't do anything now.*/

	return 0;
}

static const struct clk_ops clk_pll_ops = {
	.recalc_rate = clk_pll_recalc_rate,
	.round_rate = clk_pll_round_rate,
	.set_rate = clk_pll_set_rate,
};

static const struct clk_ops clk_dev_ops = {
	.recalc_rate = clk_pll_recalc_rate,
};

static struct clk *clk_pll_clock_register(const char *name,
					  const char *parent_name,
					  struct clk_hw *hw,
					  const struct clk_ops *ops,
					  unsigned long flags)
{
	struct clk *clk;
	struct clk_init_data init;

	init.name = name;
	init.ops = ops;
	init.flags = flags;
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = parent_name ? 1 : 0;
	hw->init = &init;

	clk = clk_register(NULL, hw);
	if (IS_ERR(clk)) {
		pr_err("%s: failed to register pll clock %s\n", __func__,
		       init.name);
		return NULL;
	}

	if (clk_register_clkdev(clk, init.name, NULL))
		pr_err("%s: failed to register lookup for %s", __func__,
		       init.name);

	return clk;
}

static void __init clk_pll_sysclk_setup(struct device_node *np)
{
	struct clk *clk;
	unsigned long flags = 0; //CLK_IS_ROOT;/* | CLK_GET_RATE_NOCACHE; */
	unsigned long rate[2];
	int i;

	for (i = 0; i < 2; i++) {
		clk = clk_pll_clock_register(clk_pll_dev[i].name, NULL,
					     &clk_pll_dev[i].hw, &clk_pll_ops,
					     flags);
		if (NULL == clk)
			continue;
		rate[i] = clk_get_rate(clk);
	}
	pr_info("PLL : [0] = %10lu, [1] = %10lu\n",
	       rate[0], rate[1]);
}

static void __init clk_pll_of_clocks_setup(struct device_node *node)
{
	struct clk_core *clk_data = NULL;
	struct clk *clk;
	unsigned long flags = CLK_IS_BASIC;
	const char *parent_name = NULL;
	int i, div, pll;
	unsigned int cm0, cm1;

	cm0 = cpm_reg->clkmode0;
	cm1 = cpm_reg->clkmode1;
	
	for (i = ID_FCLK; ID_END > i; i++) {
		clk_data = &clk_pll_dev[i];
		switch (i) {
			case ID_FCLK:
				div = CLK_MODE0_R_CLKDIV1CPU0(cm0);
				pll = CLK_MODE0_R_CLKSELCPU0(cm0);
				break;
			case ID_HCLK:
				div = CLK_MODE0_R_CLKDIV2CPU0(cm0);
				pll = CLK_MODE0_R_CLKSELCPU0(cm0);
				break;
			case ID_MCLK:
				div = CLK_MODE1_R_CLKDIVMCLK(cm1);
				pll = CLK_MODE1_R_CLKSELMCLK(cm1);
				break;
			case ID_BCLK:
				div = CLK_MODE1_R_CLKDIVBCLK(cm1);
				pll = CLK_MODE1_R_CLKSELMCLK(cm1);
				break;
			case ID_PCLK:
				div = CLK_MODE1_R_CLKDIVPCLK(cm1);
				pll = CLK_MODE1_R_CLKSELMCLK(cm1);
				break;
			case ID_XTI:
				div = 1;
				pll = 0;
				break;
		}
		clk_data->div = div;
		clk_data->pll = pll;
		if (i != ID_XTI)
			parent_name = clk_pll_dev[pll].name;
		else
			parent_name = "none";

		clk = clk_pll_clock_register(clk_data->name, parent_name,
				       &clk_data->hw, &clk_dev_ops, flags);
		if (clk)
			clk_data->rate = clk_get_rate(clk);
	}
}

static void __init clk_pll_of_clocks_dump(struct device_node *np)
{
	struct clk_core *clk_data = clk_pll_dev;

	/* cpu 0, 1  : div 0, 7 */
	pr_info("(%d) PLL%d: CPU  FCLK = %10lu, HCLK = %9lu\n",
	       clk_data[ID_FCLK].div, clk_data[ID_FCLK].pll,
	       clk_data[ID_FCLK].rate, clk_data[ID_HCLK].rate);

	/* memory */
	pr_info("(%d) PLL%d: MEM  MCLK = %10lu, BCLK = %9lu, PCLK = %9lu\n",
	       clk_data[ID_MCLK].div, (clk_data[ID_MCLK].pll == 3) ? clk_data[ID_FCLK].pll : clk_data[ID_MCLK].pll,
	       clk_data[ID_MCLK].rate, clk_data[ID_BCLK].rate,
	       clk_data[ID_PCLK].rate);

}

static void __init clk_pll_of_setup(struct device_node *node)
{
	unsigned int pllin;
	struct resource regs;

	printk("%s enter\n", __func__);
	if (of_address_to_resource(node, 0, &regs) < 0) {
		pr_err("fail get clock pll regsister\n");
		return;
	}

	cpm_reg = (struct nx_cpm_reg_t *)ioremap(regs.start, resource_size(&regs));
	if (cpm_reg == NULL) {
		pr_err("fail get Clock control base address\n");
		return;
	}
	if (0 == of_property_read_u32(node, "ref-freuecny", &pllin))
		ref_clk = pllin;

	clk_pll_sysclk_setup(node);
	clk_pll_of_clocks_setup(node);
	clk_pll_of_clocks_dump(node);

	pr_info("CPU REF HZ: %lu hz (0x%08x:0x%p)\n", ref_clk, 0xc0010000,
		cpm_reg);
}

CLK_OF_DECLARE(nxp2120, "nexell,nxp2120,pll", clk_pll_of_setup);

/*
 * Copyright (C) 2016  Nexell Co., Ltd.
 * Author: Youngbok, Park <ybpark@nexell.co.kr>
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
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/syscore_ops.h>

#include <dt-bindings/clk/nxp2120-clk.h>

#define to_clk_dev(_hw) container_of(_hw, struct clk_dev, hw)

struct clk_dev_peri {
	const char *parent_name;
	const char *name;
	void __iomem *base;
	int id;
	bool enable;
	bool is_16bit;
	bool is_clkgen;
	long rate;
	u32 clkenbtype;
	u32 clkgentype;
	/* clock register */
	int src_mask0;
	int div_mask0;
	int src_mask1;
	int div_mask1;
	int clkgen_mask;
	/* clk src and div */
	int div_src_0;
	int div_val_0;
	bool clkgen_inv0;
	bool clkgen_outen0;
	int div_src_1;
	int div_val_1;
	bool clkgen_inv1;
	bool clkgen_outen1;
	int fix_src;
};

struct clk_dev {
	struct device_node *node;
	struct clk *clk;
	struct clk_hw hw;
	struct clk_dev_peri *peri;
	unsigned int rate;
	spinlock_t lock;
};

struct clk_dev_map {
	unsigned int con_enb;
	unsigned int con_gen[4];
};

#define MAX_DIVIDER ((1 << 8) - 1) /* 256, align 2 */

static inline void clk_dev_pclk(void *base, int on)
{
	struct clk_dev_map *reg = base;
	unsigned int val = 0;

	if (!on)
		return;

	val = readl(&reg->con_enb) & ~(1 << 3);
	val |= (1 << 3); // PCLK always.
	writel(val, &reg->con_enb);
}

static inline void clk_dev_bclk(void *base, int on)
{
	struct clk_dev_map *reg = base;
	unsigned int val = 0;

	if (!on)
		return;

	val = readl(&reg->con_enb) & ~(3 << 0);
	val |= (3 << 0); // BCLK always.
	writel(val, &reg->con_enb);
}

static inline void clk_dev_enb(void *base, int on)
{
	struct clk_dev_map *reg = base;
	unsigned int val = readl(&reg->con_enb) & ~(1 << 2);

	val |= ((on ? 1 : 0) << 2);
	writel(val, &reg->con_enb);
}


static inline void clk_dev_genclk(void *base, int on)
{
	struct clk_dev_map *reg = base;
	unsigned int val = readl(&reg->con_enb) & ~(1 << 2);

	val |= on ? (1 << 2) : 0; /* DEVICE CLOCK On*/
	writel(val, &reg->con_enb);
}

static inline void clk_dev_rate(void *base, int step, int src, int div, int div_mask)
{
	struct clk_dev_map *reg = base;
	unsigned int val = 0;

	val = readl(&reg->con_gen[step]);
	val &= ~(0x07 << 2);
	val |= (src << 2); /* source */
	val &= ~div_mask;
	val |= ((div - 1) << 5) & div_mask; /* divider */
	writel(val, &reg->con_gen[step]);
}

static inline void clk_dev_inv(void *base, int step, int inv)
{
	struct clk_dev_map *reg = base;
	unsigned int val = readl(&reg->con_gen[step]) & ~(1 << 1);

	val |= (inv << 1);
	writel(val, &reg->con_gen[step]);
}

static inline void clk_dev_outenb(void *base, int step, int on)
{
	struct clk_dev_map *reg = base;
	unsigned int val = readl(&reg->con_gen[step]) & ~(1 << 15);

	val |= ((on ? 1 : 0) << 15);
	writel(val, &reg->con_gen[step]);
}

static inline long clk_dev_divide(long rate, long request, int align,
				  int *divide, int step)
{
	int div = (rate / request);
	int max = MAX_DIVIDER & ~(align - 1);
	int adv = (div & ~(align - 1)) + align;

	if (!div) {
		if (divide)
			*divide = 1;
		return rate;
	}

	if (1 != div)
		div &= ~(align - 1);

	if (div != adv && abs(request - rate / div) > abs(request - rate / adv))
		div = adv;

	if (step == 2)
		div = (div > max ? div/2 : div);
	else
		div = (div > max ? max : div);

	if (divide)
		*divide = div;

	return (rate / div);
}

static long clk_dev_bus_rate(struct clk_dev_peri *peri)
{
	struct clk *clk;
	const char *name = NULL;
	long rate = 0;

	/* what mask it neede? */
	name = "bclk";

	if (name) {
		clk = clk_get(NULL, name);
		rate = clk_get_rate(clk);
		clk_put(clk);
	}

	return rate ?: -EINVAL;
}

static long clk_dev_pll_rate(int no)
{
	struct clk *clk;
	char name[16];
	long rate = 0;

	snprintf(name, sizeof(name), "pll%d", no);
	clk = clk_get(NULL, name);
	rate = clk_get_rate(clk);
	clk_put(clk);

	return rate;
}

static long dev_round_rate(struct clk_hw *hw, unsigned long rate)
{
	struct clk_dev_peri *peri = ((struct clk_dev *)to_clk_dev(hw))->peri;
	unsigned long request = rate, new_rate = 0;

	/* ksw : I don't implement this yet. */

	return new_rate;
}

static int dev_set_rate(struct clk_hw *hw, unsigned long rate)
{
	struct clk_dev_peri *peri = ((struct clk_dev *)to_clk_dev(hw))->peri;
	int i;

	rate = dev_round_rate(hw, rate);

#if 0
	for (i = 0; peri->clk_step > i; i++) {
		int s = (0 == i ? peri->div_src_0 : peri->div_src_1);
		int d = (0 == i ? peri->div_val_0 : peri->div_val_1);

		if (-1 == s)
			continue;

		/* change rate */
#ifdef CONFIG_EARLY_PRINTK
		if (!strcmp(peri->name, "uart0"))
			break;
#endif
		clk_dev_rate((void *)peri->base, i, s, d);

		pr_debug("clk: %s (%p) set_rate [%d] src[%d] div[%d]\n",
			 peri->name, peri->base, i, s, d);
	}
#endif
	peri->rate = rate;
	return rate;
}

static int clk_dev_is_enabled(struct clk_hw *hw)
{
	struct clk_dev_peri *peri = ((struct clk_dev *)to_clk_dev(hw))->peri;

	return peri->enable;
}

/*
 *	clock devices interface
 */
static int clk_dev_enable(struct clk_hw *hw)
{
	struct clk_dev_peri *peri = ((struct clk_dev *)to_clk_dev(hw))->peri;
	int i = 0, inv = 0;

	pr_debug("clk: %s enable (BCLK=%s, PCLK=%s)\n", peri->name,
		 peri->clkenbtype & CLKENB_MASK_BCLK ? "ON" : "PASS",
		 peri->clkenbtype & CLKENB_MASK_BCLK ? "ON" : "PASS");

	if (peri->clkenbtype & CLKENB_MASK_BCLK)
		clk_dev_bclk((void *)peri->base, 1);

	if (peri->clkenbtype & CLKENB_MASK_BCLK)
		clk_dev_pclk((void *)peri->base, 1);

	if (peri->is_clkgen) {
#if 0
		/* does we should output clock, and invert it ? */
		for (i = 0, inv = peri->invert_0; peri->clk_step > i;
			i++, inv = peri->invert_1)
			clk_dev_inv((void *)peri->base, i, inv);

		/* restore clock rate */
		for (i = 0; peri->clk_step > i; i++) {
			if (peri->fix_src < 0) {
				int s = (0 == i ? peri->div_src_0 : peri->div_src_1);
				int d = (0 == i ? peri->div_val_0 : peri->div_val_1);

				if (s == -1)
					continue;

				clk_dev_rate((void *)peri->base, i, s, d);
			} else {
				int s = peri->fix_src;
				int d = (0 == i ? peri->div_val_0 : peri->div_val_1);
				if(d == 0)
					d = 1;
				clk_dev_rate((void *)peri->base, i, s, d);
			}
		}
#endif
	}
	

	clk_dev_enb((void *)peri->base, 1);
	peri->enable = true;

	return 0;
}

static void clk_dev_disable(struct clk_hw *hw)
{
	struct clk_dev_peri *peri = ((struct clk_dev *)to_clk_dev(hw))->peri;

	pr_debug("clk: %s disable\n", peri->name);

	if (peri->clkenbtype & CLKENB_MASK_BCLK)
		clk_dev_bclk((void *)peri->base, 0);

	if (peri->clkenbtype & CLKENB_MASK_PCLK)
		clk_dev_pclk((void *)peri->base, 0);

	clk_dev_rate((void *)peri->base, 0, 7, 256, peri->div_mask0); /* for power save */
	clk_dev_enb((void *)peri->base, 0);

	peri->enable = false;
}

static unsigned long clk_dev_recalc_rate(struct clk_hw *hw, unsigned long rate)
{
	struct clk_dev_peri *peri = ((struct clk_dev *)to_clk_dev(hw))->peri;

	pr_debug("%s: name %s, (%lu)\n", __func__, peri->name, peri->rate);
	return peri->rate;
}

static long clk_dev_round_rate(struct clk_hw *hw, unsigned long drate,
			       unsigned long *prate)
{
	struct clk_dev_peri *peri = ((struct clk_dev *)to_clk_dev(hw))->peri;
	long rate = dev_round_rate(hw, drate);

	pr_debug("%s: name %s, (%lu, %lu)\n", __func__, peri->name, drate,
		 rate);
	return rate;
}

static int clk_dev_set_rate(struct clk_hw *hw, unsigned long drate,
			    unsigned long prate)
{
	struct clk_dev_peri *peri = ((struct clk_dev *)to_clk_dev(hw))->peri;
	int rate = dev_set_rate(hw, drate);

	pr_debug("%s: name %s, rate %lu:%d\n", __func__, peri->name, drate,
		 rate);
	return rate;
}

static const struct clk_ops clk_dev_ops = {
	.recalc_rate = clk_dev_recalc_rate,
	.round_rate = clk_dev_round_rate,
	.set_rate = clk_dev_set_rate,
	.enable = clk_dev_enable,
	.disable = clk_dev_disable,
	.is_enabled = clk_dev_is_enabled,
};

static const struct clk_ops clk_empty_ops = {};

struct clk *clk_dev_get_provider(struct of_phandle_args *clkspec, void *data)
{
	struct clk_dev *clk_data = data;

	pr_debug("%s: name %s\n", __func__, clk_data->peri->name);
	return clk_data->clk;
}

static void __init clk_dev_parse_device_data(struct device_node *np,
					     struct clk_dev *clk_data,
					     struct device *dev)
{
	struct clk_dev_peri *peri = clk_data->peri;
	struct resource res;
	unsigned int frequency = 0;
	u32 value;

	if (of_property_read_string(np, "clock-output-names", &peri->name)) {
		pr_info("clock node is missing 'clock-output-names'\n");
		return;
	}

	if (!of_property_read_string(np, "clock-names", &peri->parent_name))
		return;

	if (of_property_read_u32(np, "clk-enb-type", &peri->clkenbtype)) 
		return;

	if (of_property_read_u32(np, "clk-gen-type", &peri->clkgentype))
		return;

	if (!of_property_read_u32(np, "clock-frequency", &frequency))
		clk_data->rate = frequency;

#if 0
	if (!of_property_read_u32(np, "clock-src", &src))
		clk_data->clk_src0 = src;
	if (!of_property_read_u32(np, "clock-div", &div))
		clk_data->clk_div0 = div;

	if (!of_property_read_u32(np, "clock-src1", &src))
		clk_data->clk_src1 = src;
	if (!of_property_read_u32(np, "clock-div1", &div))
		clk_data->clk_div1 = div;

	if (!of_property_read_u32(np, "clock-output-inv", &src))
		clk_data->clk_src0 = src;
	if (!of_property_read_u32(np, "clock-output-enable", &div))
		clk_data->clk_div0 = div;
#endif
	if (0 == peri->clkgentype) {
		peri->is_clkgen = false;
	} else {
		peri->is_clkgen = true;
	}
	if (peri->is_clkgen) {
		int mask0,mask1;
		// We should get source
		mask0 = 0;
		mask1 = 0;
		switch (peri->clkgentype & 0x0F) {
			case CLKGEN_TYPE_PLL0_PLL1:
				mask0 = 3;
				break;
			case CLKGEN_TYPE_PCLK_PLL0_PLL1:
				mask0 = 7;
				break;
			case CLKGEN_TYPE_PLL0_PLL1_ICLK_IICLK:
				mask0 = 3 | BIT(3) | BIT(4);
				break;
			case CLKGEN_TYPE_PLL0_PLL1_ICLK_IICLK_AV_IAV:
				mask0 = BIT(0) | BIT(1) | BIT(3) | BIT(4) | BIT(5) | BIT(6);
				if (peri->clkgentype & 0xF00) {
					mask1 = mask1 = BIT(0) | BIT(1) | BIT(3) | BIT(4) | BIT(5) | BIT(6) | BIT(7);
				}
				break;
			case CLKGEN_TYPE_PLL0_PLL1_ICLK_IICLK_AV_IAV_GEN0:
				mask0 = BIT(0) | BIT(1) | BIT(3) | BIT(4) | BIT(5) | BIT(6);
				mask1 = BIT(0) | BIT(1) | BIT(3) | BIT(4) | BIT(5) | BIT(6) | BIT(7);
				break;
			case CLKGEN_TYPE_EXTCLK:
				mask0 = BIT(3);
				break;
			case CLKGEN_TYPE_PLL0_PLL1_XTI:
				mask0 = 0x7;
				break;
			case CLKGEN_TYPE_PLL0_PLL1_SVLK_PSVLCK_IPSVCLK_AVCLK_ISVCLK:
				mask0 = 0x7F;
				if (peri->clkgentype & 0xF00) {
					mask1 = 0x7F;
				}
				break;
			default:
				;
		}
		peri->src_mask0 = mask0;
		peri->clkgen_inv0 = !!(peri->clkgentype & CLKGEN_TYPE_OUTCLK_INV);
		peri->clkgen_outen0 = !!(peri->clkgentype & CLKGEN_TYPE_OUTCLK_ENB);
		peri->clkgen_mask = 1;
		if (peri->clkgentype & 0xF00) {
			switch (peri->clkgentype & 0xF00) {
				case CLKGEN_TYPE_GEN0L_GEN0H_GEN1:
					peri->clkgen_mask = 7;
					break;
				case CLKGEN_TYPE_GEN0_GEN1:
					peri->clkgen_mask = 5;
					break;
				case CLKGEN_TYPE_GEN0LH_GEN1LH:
					peri->clkgen_mask = 0xF;
					break;
			}
			peri->src_mask1 = mask1;
			peri->clkgen_inv1 = !!(peri->clkgentype & CLKGEN_TYPE_OUTCLK_INV);
			peri->clkgen_outen1 = !!(peri->clkgentype & CLKGEN_TYPE_OUTCLK_ENB);
		}
	}
	of_address_to_resource(np, 0, &res);

	peri->base = ioremap_nocache(res.start, resource_size(&res));
	if (!peri->base) {
		pr_err("Can't map registers for clock %s!\n", peri->name);
		return;
	}

	pr_debug("%8s: id=%2d, base=%p, enb=0x%04x, gen=0x%04x\n",
		 peri->name, peri->id, peri->base,
		 peri->clkenbtype, peri->clkgentype);
}

struct clk *clk_dev_clock_register(const char *name, const char *parent_name,
				   struct clk_hw *hw, const struct clk_ops *ops,
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
	pr_debug("Register clk %8s: parent %s\n", name, parent_name);

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

#ifdef CONFIG_PM_SLEEP
static int clk_syscore_suspend(void) { return 0; }

static void clk_syscore_resume(void) {}

static struct syscore_ops clk_syscore_ops = {
	.suspend = clk_syscore_suspend, .resume = clk_syscore_resume,
};
#endif /* CONFIG_PM_SLEEP */

static void __init clk_dev_of_setup(struct device_node *node)
{
	struct device_node *np;
	struct clk_dev *clk_data = NULL;
	struct clk_dev_peri *peri = NULL;
	struct clk *clk;
	int i = 0, size = (sizeof(*clk_data) + sizeof(*peri));
	int num_clks;

	num_clks = of_get_child_count(node);
	if (!num_clks) {
		pr_err("Failed to clocks count for clock generator!\n");
		return;
	}

	clk_data = kzalloc(size * num_clks, GFP_KERNEL);
	if (!clk_data) {
		WARN_ON(1);
		return;
	}
	peri = (struct clk_dev_peri *)(clk_data + num_clks);

	for_each_child_of_node(node, np) {
		clk_data[i].peri = &peri[i];
		clk_data[i].node = np;
		clk_dev_parse_device_data(np, &clk_data[i], NULL);
		of_clk_add_provider(np, clk_dev_get_provider, &clk_data[i++]);
	}

	for (i = 0; num_clks > i; i++, clk_data++) {
		unsigned long flags = 0;//CLK_IS_ROOT;
		const struct clk_ops *ops = &clk_dev_ops;

		if (peri[i].parent_name) {
			ops = &clk_empty_ops;
			flags = CLK_IS_BASIC;
		}

		clk = clk_dev_clock_register(peri[i].name, peri[i].parent_name,
					     &clk_data->hw, ops, flags);
		if (NULL == clk)
			continue;

		clk_data->clk = clk;
		if (clk_data->rate) {
			pr_debug("[%s set boot rate %u]\n", node->name,
				 clk_data->rate);
			clk_set_rate(clk, clk_data->rate);
		}
	}

#ifdef CONFIG_PM_SLEEP
	register_syscore_ops(&clk_syscore_ops);
#endif

	pr_debug("[%s:%d] %s (%d)\n", __func__, __LINE__, node->name, num_clks);
}
CLK_OF_DECLARE(nxp2120, "nexell,nxp2120,clocks", clk_dev_of_setup);

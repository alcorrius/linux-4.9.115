/*
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
#define DEBUG	1
#include <linux/kernel.h>
//#include <linux/ioport.h>
//#include <linux/interrupt.h>
//#include <linux/spinlock.h>
#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>      
#include <linux/device.h>
#include <linux/slab.h>     
//#include <asm/io.h>
#include <asm/exception.h>
#include <asm/irq.h>
//#include <asm/system.h>

#include <asm/mach/irq.h>

/* nexell soc headers */
//#include <mach/platform.h>

/* local */
/*static void __init gpio_init_irq(void);
static void __init dma_init_irq(void);
static void __init alive_init_irq(void);
*/
#if 0
/*----------------------------------------------------------------------------
 *  irq handler
 *  start  -> enable
 *  do IRQ -> mask_ack -> handler -> unmask->
 *  do IRQ -> mask_ack -> handler -> unmask-> ...
 *  end    -> disable
 ----------------------------------------------------------------------------*/
#define	REG_INTBASE	IO_ADDRESS(PHY_BASEADDR_INTC_MODULE)

#define REG_INTMODE_0		0x08	/* Interrupt Mode Register */
#define REG_INTMODE_1		0x0C 	/* Interrupt Mode Register */
#define REG_INTMASK_0		0x10	/* Interrupt Mask Register */
#define REG_INTMASK_1		0x14 	/* Interrupt Mask Register */
#define REG_PRIORDER		0x18 	/* Priority Order Register */
#define REG_INTPEND_0		0x20	/* Interrupt Pending Register */
#define REG_INTPEND_1		0x24 	/* Interrupt Pending Register */
#endif

typedef int CBOOL;
typedef unsigned int U32;
typedef int S32;
#define CTRUE	1
#define CFALSE	0

struct nx_intc_register_set
{
	volatile U32 __reserved[2];	///< 0x00 ~ 0x07 : Reserved
	volatile U32 intmode[2];	///< 0x08, 0x0C : Interrupt Mode Register
	volatile U32 intmask[2];	///< 0x10, 0x14 : Interrupt Mask Register
	volatile U32 priorder;		///< 0x18 : Priority Order Register
	volatile U32 __reserved01;	///< 0x1C : Reserved
	volatile U32 intpend[2];	///< 0x20, 0x24 : Interrupt Pending Register
	volatile U32 __reserved02;	///< 0x28 : Reserved
};


#define NUM_IRQS		56

struct nx_intc {
	void __iomem		*base;
	struct irq_domain	*dom;
};
static struct nx_intc *cpu_intc;


CBOOL NX_INTC_GetInterruptPending( S32 IntNum )
{
	struct nx_intc *intc = cpu_intc;
	struct nx_intc_register_set* pREG = (struct nx_intc_register_set*)intc->base;	
	int RegSel, RegBit;
	U32 regvalue;

	RegSel	= IntNum >> 5;		// 0 or 1
	RegBit	= IntNum & 0x1F;	// 0 ~ 31

	regvalue = pREG->intpend[RegSel];

	if( regvalue & ( 1UL << RegBit) )
	{
		return CTRUE;
	}

	return CFALSE;
}

void NX_INTC_SetInterruptEnable( S32 IntNum, CBOOL Enable )
{
	struct nx_intc *intc = cpu_intc;
	struct nx_intc_register_set* pREG = (struct nx_intc_register_set*)intc->base;	
	int RegSel, RegBit;
	U32 regvalue;

	Enable = (CBOOL)((U32)Enable ^ 1);	// Service:0, Masked:1

	RegSel	= IntNum >> 5;		// 0 or 1
	RegBit	= IntNum & 0x1F;	// 0 ~ 31

	regvalue = pREG->intmask[RegSel];
	regvalue &= ~( (U32)1		<< RegBit);
	regvalue |=	( (U32)Enable << RegBit);

	writel(regvalue, &pREG->intmask[RegSel]);
}


void NX_INTC_ClearInterruptPending( S32 IntNum )
{
	struct nx_intc *intc = cpu_intc;
	struct nx_intc_register_set* pREG = (struct nx_intc_register_set*)intc->base;	
	int RegSel, RegBit;
	U32 regvalue;

	RegSel	= IntNum >> 5;		// 0 or 1
	RegBit	= IntNum & 0x1F;	// 0 ~ 31

	regvalue = pREG->intpend[RegSel];
	regvalue = 1UL << RegBit;

	writel(regvalue, &pREG->intpend[RegSel]);
}


/* clear irq pend bit */
static void cpu_irq_ack(struct irq_data *irq)
{
	//printk("ack(%d)\n", irq->hwirq);
#if (1)
	NX_INTC_ClearInterruptPending(irq->hwirq);
#else
	soc_int_clr_pend(irqno);		/* intc : clear irq pend */
#endif
}

static void cpu_irq_enable(struct irq_data *irq)
{
	//printk("enable(%d) hw=%d\n", irq->irq, irq->hwirq);
#if (1)
	NX_INTC_ClearInterruptPending(irq->hwirq);
	NX_INTC_SetInterruptEnable(irq->hwirq, CTRUE);
	NX_INTC_GetInterruptPending(irq->hwirq);
#else
	soc_int_clr_pend(irqno);	 	/* intc : clear irq pend */
	soc_int_set_enable(irqno);		/* intc : enable irq */
	soc_int_get_pend(irqno);		/* Guarantee that irq Pending is cleared */
#endif
}

static void cpu_irq_disable(struct irq_data *irq)
{
	//printk("disable(%d)\n", irq->hwirq);
#if (1)
	NX_INTC_ClearInterruptPending(irq->hwirq);
	NX_INTC_SetInterruptEnable(irq->hwirq, CFALSE);
	NX_INTC_GetInterruptPending(irq->hwirq);
#else
	soc_int_clr_pend(irqno);	 	/* intc : clear irq pend */
	soc_int_set_disable(irqno);		/* intc : disable irq */
	soc_int_get_pend(irqno);		/* Guarantee that irq Pending is cleared */
#endif
}

/* disable irq: set mask bit & clear irq pend bit */
static void cpu_irq_mask(struct irq_data *irq)
{
	//printk("mask_ack(%d)\n", irq->hwirq);
#if (1)
//	NX_INTC_ClearInterruptPending(irq->irq);
	NX_INTC_SetInterruptEnable(irq->hwirq, CFALSE);
	NX_INTC_GetInterruptPending(irq->hwirq);
#else
	soc_int_clr_pend(irqno);		/* intc : clear irq pend */
	soc_int_set_disable(irqno);		/* intc : disable irq */
	soc_int_get_pend(irqno);		/* Guarantee that irq Pending is cleared */
#endif
}

/* enable irq: set unmask bit & clear irq pend bit */
static void cpu_irq_unmask(struct irq_data *irq)
{
	//printk("unmask(%d)\n", irq->hwirq);
#if (1)
	NX_INTC_ClearInterruptPending(irq->hwirq);
	NX_INTC_SetInterruptEnable(irq->hwirq, CTRUE);
	NX_INTC_GetInterruptPending(irq->hwirq);
#else
	soc_int_clr_pend(irqno);		/* intc : clear irq pend */
	soc_int_set_enable(irqno);		/* intc : enable irq */
	soc_int_get_pend(irqno);		/* Guarantee that irq Pending is cleared */
#endif
}

#if 1

static void __exception_irq_entry nx_intc_handle_irq(struct pt_regs *regs)
{
	struct nx_intc *intc = cpu_intc;
	struct nx_intc_register_set* pREG = (struct nx_intc_register_set*)intc->base;	
	u32 stat, irq;

	//printk("%s enter regs 0x%X\n", __func__, regs);
	for (;;) {
		irq = 0;
		stat = readl_relaxed(&pREG->intpend[0]);
		if (!stat) {
			stat = readl_relaxed(&pREG->intpend[1]);
			irq = 32;
		}
		if (stat == 0)
			break;
		irq += ffs(stat) - 1;
		//printk("irq %d\n", irq);
		handle_domain_irq(intc->dom, irq, regs);
	}
}

/* interrupt controller irq control */
static struct irq_chip cpu_irq_chip = {
	.name		= "IRQ",
	.irq_ack        = cpu_irq_ack,
	.irq_enable		= cpu_irq_enable,
	.irq_disable	= cpu_irq_disable,
	.irq_mask		= cpu_irq_mask,
	.irq_unmask		= cpu_irq_unmask,
};


static int nx_intc_map(struct irq_domain *d, unsigned int irq,
		    irq_hw_number_t hwirq)
{
	struct nx_intc *intc = d->host_data;
	struct irq_chip *chip = &cpu_irq_chip;

	irq_set_chip_and_handler(irq, chip, handle_level_irq);
//	irq_set_chip_data(irq, intc);
//	irq_domain_set_info(d,irq,hwirq,chip,d->host_data,handle_level_irq, NULL, NULL);
	irq_set_probe(irq);
	return 0;
}


static struct irq_domain_ops nx_intc_dom_ops = {
	.map = nx_intc_map, 
	.xlate = irq_domain_xlate_onecell,

};

static int __init nxp2120_irq_of_init(struct device_node *node,
			       struct device_node *parent)
{
	void __iomem *regs;
	struct nx_intc *intc;
	struct irq_chip *chip = &cpu_irq_chip;	
	int i=0;
	printk("%s enter\n", __func__);
	
	if (WARN(parent, "non-root nxp2120 intc not supported"))
		return -EINVAL;
	if (WARN(cpu_intc, "duplicate nxp2120 intc not supported"))
		return -EINVAL;

	regs = of_iomap(node, 0);
	if (WARN_ON(!regs))
		return -EIO;

	printk("irq regs 0x%X\n", regs);
	intc = kzalloc(sizeof(struct nx_intc), GFP_KERNEL);
	if (WARN_ON(!intc)) {
		iounmap(regs);
		return -ENOMEM;
	}
	intc->base = regs;

	/* Initialize soures, all masked */
	//intc_init_hw(intc);

	/* Ready to receive interrupts */
	cpu_intc = intc;
	

	/* Register our domain */
	//intc->dom = irq_domain_add_simple(node, NUM_IRQS, 0,
	//				 &nx_intc_dom_ops, intc);

	intc->dom = irq_domain_add_linear(node, NUM_IRQS, 
					 &nx_intc_dom_ops, intc);
	if (!intc->dom)
		panic("%s: unable to create IRQ domain\n", node->full_name);

	set_handle_irq(nx_intc_handle_irq);

	return 0;
}
#else

#define NUM_IRQS		32//56
static void nx_intc_handle_irq(struct irq_desc *desc)
{
	struct irq_domain *domain = irq_desc_get_handler_data(desc);
	struct irq_chip_generic *gc = domain->gc->gc[0];
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned long pending;
	int n;
	
	u32 stat, irq;

	printk("%s enter regs 0x%X\n", __func__, gc->reg_base);

	chained_irq_enter(chip, desc);
	
	for (;;) {
		irq = 0;
		stat = readl_relaxed(gc->reg_base+0x20);//&pREG->intpend[0]);
		if (!stat) {
			stat = readl_relaxed(gc->reg_base+0x24);//&pREG->intpend[1]);
			irq = 32;
		}
		if (stat == 0)
			break;
		irq += ffs(stat) - 1;
		printk("irq %d\n", irq);
		generic_handle_irq(irq_find_mapping(domain, irq));
	}
	
	chained_irq_exit(chip, desc);
}

static int nxp2120_intc_alloc(struct irq_domain *d, unsigned int virq,
			    unsigned int nr_irqs, void *data)
{
	struct irq_chip_generic *gc = d->gc->gc[0];
	struct irq_fwspec *fwspec = data;
	irq_hw_number_t hwirq;

	hwirq = fwspec->param[0];

	irq_map_generic_chip(d, virq, hwirq);
	irq_domain_set_info(d, virq, hwirq, &gc->chip_types->chip, gc,
			    handle_simple_irq, NULL, NULL);

	return 0;
}

static void nxp2120_intc_free(struct irq_domain *d, unsigned int virq,
			    unsigned int nr_irqs)
{
	struct irq_data *data = irq_domain_get_irq_data(d, virq);

	irq_domain_reset_irq_data(data);
}

struct irq_domain_ops nx_intc_dom_ops = {
	.map	= irq_map_generic_chip,
	.xlate	= irq_domain_xlate_onetwocell,
	.alloc  = nxp2120_intc_alloc,
	.free	= nxp2120_intc_free,
};




static int __init nxp2120_irq_of_init(struct device_node *node,
				  struct device_node *parent)
{
	unsigned int clr = IRQ_NOREQUEST | IRQ_NOPROBE | IRQ_NOAUTOEN;
	int nr_irqs, nr_exti, ret, i;
	struct irq_chip_generic *gc;
	struct irq_domain *domain;
	void *base;
	printk("===========================%s enter\n", __func__);
	base = of_iomap(node, 0);
	if (!base) {
		pr_err("%s: Unable to map registers\n", node->full_name);
		return -ENOMEM;
	}
	pr_info("%s: %d External IRQs detected\n", node->full_name, NUM_IRQS);
	domain = irq_domain_add_linear(node, NUM_IRQS,
				       &nx_intc_dom_ops, NULL);
	if (!domain) {
		pr_err("%s: Could not register interrupt domain.\n",
				node->name);
		ret = -ENOMEM;
		goto out_unmap;
	}

	ret = irq_alloc_domain_generic_chips(domain, NUM_IRQS, 1, "intc",
					     handle_level_irq, clr, 0, 0);
	if (ret) {
		pr_err("%s: Could not allocate generic interrupt chip.\n",
			node->full_name);
		goto out_free_domain;
	}

	gc = domain->gc->gc[0];
	gc->reg_base                         = base;
	gc->chip_types->type               = IRQ_TYPE_EDGE_BOTH;
	gc->chip_types->chip.name          = gc->chip_types[0].chip.name;
	gc->chip_types->chip.irq_ack       = cpu_irq_ack;
	gc->chip_types->chip.irq_mask      = cpu_irq_mask;
	gc->chip_types->chip.irq_unmask    = cpu_irq_unmask;
//	gc->chip_types->chip.irq_set_type  = stm32_irq_set_type;
//	gc->chip_types->chip.irq_set_wake  = stm32_irq_set_wake;
//	gc->chip_types->regs.ack           = EXTI_PR;
//	gc->chip_types->regs.mask          = EXTI_IMR;
	gc->chip_types->handler            = handle_level_irq;

	nr_irqs = of_irq_count(node);
	for (i = 0; i < nr_irqs; i++) {
		unsigned int irq = irq_of_parse_and_map(node, i);

		irq_set_handler_data(irq, domain);
		irq_set_chained_handler(irq, nx_intc_handle_irq);
	}

	return 0;

out_free_domain:
	irq_domain_remove(domain);
out_unmap:
	iounmap(base);
	return ret;
}
#endif

IRQCHIP_DECLARE(nxp2120_irq, "nexell,nxp2120-intc", nxp2120_irq_of_init);

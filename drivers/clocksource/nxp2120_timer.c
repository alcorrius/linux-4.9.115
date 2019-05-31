/*
 * Copyright (C) 2019  i4vine.
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

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/spinlock.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/version.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>

/// @brief	TIMER Module's Register List
typedef unsigned int U32;
typedef int CBOOL;
#define NX_ASSERT(x)
#define CTRUE	1
#define CFALSE	0
#define CLK_SOURCE_HZ (1*1000000)

struct	nx_timer_register_set
{
	volatile U32 tmrcount;			///< 0x00 : Timer counter register
	volatile U32 tmrmatch;			///< 0x04 : Timer match register
	volatile U32 tmrcontrol;		///< 0x08 : Timer control register
	volatile U32 _reserved[0x0D];	///< 0x0C ~ 0x3C : Reserved region
	volatile U32 tmrclkenb;			///< 0x40 : Timer clock generation enable register
	volatile U32 tmrclkgen;			///< 0x44 : Timer clock generation control register
};

/// @brief TIMER interrupt for interrupt interface
enum
{
	NX_TIMER_INT_COUNTER = 0		///< TIMER Counter interrupt.
};

/// @brief	timer clock source
typedef enum 
{
	NX_TIMER_CLOCK_TCLK		= 3,	///< TCLK = Source clock
	NX_TIMER_CLOCK_TCLK2	= 0,	///< TCLK = Source clock / 2
	NX_TIMER_CLOCK_TCLK4	= 1,	///< TCLK = Source clock / 4
	NX_TIMER_CLOCK_TCLK8	= 2		///< TCLK = Source clock / 8

} NX_TIMER_CLOCK;
	
typedef enum 
{
	NX_PCLKMODE_DYNAMIC = 0UL,		///< PCLK is provided only when CPU has access to registers of this module.
	NX_PCLKMODE_ALWAYS	= 1UL		///< PCLK is always provided for this module.
} NX_PCLKMODE ;

 /* timer data structs */
struct timer_info {
	void __iomem *clk_base;
	int ch;
	u_int 	clksrc;
	u_int 	clkdiv;
	u_int   clkfreq;
	u_int 	period;	
	int irq;
	const char *clock_name;
	struct clk *clk;
/*	unsigned long request;
	unsigned long rate;
	int tmux;
	int prescale;
	unsigned int tcount;
	unsigned int rcount;*/
};

struct timer_of_dev {
//	struct clk *pclk;
	struct timer_info timer_source;
	struct timer_info timer_event;
};
static struct timer_of_dev *timer_dev;
#define get_timer_dev() ((struct timer_of_dev *)timer_dev)


void NX_TIMER_Run(struct timer_info *t_info)
{
	struct timer_info *info = t_info;	
	struct nx_timer_register_set* pREG = (struct nx_timer_register_set*)info->clk_base;	
	const U32 INTPEND	= (1UL<<5);
	const U32 RUN		= (1UL<<3);

	U32 regvalue;

	regvalue  = pREG->tmrcontrol;

	regvalue &= ~INTPEND;
	regvalue |=	RUN;

	writel(regvalue,&pREG->tmrcontrol);
}

static inline void NX_TIMER_SetMatchCounter(struct timer_info *t_info, unsigned int period)
{
	struct timer_info *info = t_info;	
	struct nx_timer_register_set* pREG = (struct nx_timer_register_set*)info->clk_base;	
	unsigned int matchcnt = period;
	writel(matchcnt, &pREG->tmrmatch);
}


static inline void NX_TIMER_SetTimerCounter(struct timer_info *t_info, unsigned int tmrcnt)
{
	struct timer_info *info = t_info;	
	struct nx_timer_register_set* pREG = (struct nx_timer_register_set*)info->clk_base;	

	writel(tmrcnt, &pREG->tmrcount);
}

static inline void NX_TIMER_ClearInterruptPendingAll(struct timer_info *t_info)
{
	struct timer_info *info = t_info;	
	struct nx_timer_register_set* pREG = (struct nx_timer_register_set*)info->clk_base;	

	const	 U32 INTPEND_POS = 5;
	U32 regvalue;

	regvalue  = pREG->tmrcontrol;
	regvalue |= 1UL << INTPEND_POS;

//	pRegister->TMRCONTROL = regvalue;
	writel(regvalue, &pREG->tmrcontrol);
}

static inline void NX_TIMER_SetTClkDivider(struct timer_info *t_info, unsigned int clock)
{
	struct timer_info *info = t_info;	
	struct nx_timer_register_set* pREG = (struct nx_timer_register_set*)info->clk_base;	
	
	const U32 INTPEND	= (1UL<<5);
	const U32 SELTCLK	= (3UL<<0);

	U32 regvalue;

	regvalue = pREG->tmrcontrol;

	regvalue &= ~(INTPEND | SELTCLK);
	regvalue |=	(U32)clock;

	writel(regvalue, &pREG->tmrcontrol);
}

static inline void NX_TIMER_SetInterruptEnableAll(struct timer_info *t_info, int enable)
{
	struct timer_info *info = t_info;	
	struct nx_timer_register_set* pREG = (struct nx_timer_register_set*)info->clk_base;	

	const	U32 INTPEND_POS = 5;
	const	U32 INTENB_POS	= 4;
	U32 regvalue;
	
	regvalue  = pREG->tmrcontrol;

	regvalue &=	~(( 1UL << INTPEND_POS ) | ( 1UL << INTENB_POS ));
	regvalue |= (U32)enable << INTENB_POS ;

//	pRegister->TMRCONTROL = regvalue;
	writel(regvalue, &pREG->tmrcontrol);
}


static inline void NX_TIMER_SetWatchDogEnable(struct timer_info *t_info, int enable)
{
	struct timer_info *info = t_info;	
	struct nx_timer_register_set* pREG = (struct nx_timer_register_set*)info->clk_base;	
	
	const U32 INTPEND	= (1UL<<5);
	const U32 WDENB		= (1UL<<2);
	U32 regvalue;

	regvalue  = pREG->tmrcontrol;

	regvalue &= ~(INTPEND|WDENB);

	if( enable )	regvalue |= WDENB;

//	pRegister->TMRCONTROL = regvalue;
	writel(regvalue, &pREG->tmrcontrol);
}

static inline void NX_TIMER_Stop(struct timer_info *t_info)
{
	struct timer_info *info = t_info;	
	struct nx_timer_register_set* pREG = (struct nx_timer_register_set*)info->clk_base;	
	
	const U32 INTPEND	= (1UL<<5);
	const U32 RUN		= (1UL<<3);

	U32 regvalue;
	regvalue  = pREG->tmrcontrol;

	regvalue &= ~(INTPEND | RUN);

	writel(regvalue, &pREG->tmrcontrol);
}

static inline void NX_TIMER_SetClockDivisorEnable(struct timer_info *t_info, unsigned int enable)
{
	struct timer_info *info = t_info;	
	struct nx_timer_register_set* pREG = (struct nx_timer_register_set*)info->clk_base;	
	
	const U32 CLKGENENB_POS	= 2;
	U32 regvalue;

	regvalue  = pREG->tmrclkenb;

	regvalue &= ~(1UL		<< CLKGENENB_POS);
	regvalue |=	(U32)enable << CLKGENENB_POS;

	writel(regvalue, &pREG->tmrclkenb);
}

static inline void NX_TIMER_SetClockPClkMode(struct timer_info *t_info, int mode)
{
	struct timer_info *info = t_info;	
	struct nx_timer_register_set* pREG = (struct nx_timer_register_set*)info->clk_base;	
	
	U32 clkmode	=	0;
	const U32 TCLKMODE_POS	= 3;
	U32 regvalue;

	switch(mode)
	{
		case NX_PCLKMODE_DYNAMIC:	clkmode = 0;		break;
		case NX_PCLKMODE_ALWAYS:	clkmode = 1;		break;
		default: NX_ASSERT( 0 );
	}

	regvalue = pREG->tmrclkenb;

	regvalue &= ~(1UL<<TCLKMODE_POS);
	regvalue |= ( clkmode & 0x01 ) << TCLKMODE_POS;

	writel(regvalue, &pREG->tmrclkenb);
}


static inline void NX_TIMER_SetClockDivisor(struct timer_info *t_info)
{
	struct timer_info *info = t_info;	
	struct nx_timer_register_set* pREG = (struct nx_timer_register_set*)info->clk_base;	
	const U32 CLKDIV_POS	= 5;
	const U32 CLKDIV_MASK	= 0xFF;
	
	U32 regvalue;

	regvalue  = pREG->tmrclkgen;

	regvalue &= ~(CLKDIV_MASK << CLKDIV_POS);
	regvalue |= ( info->clkdiv - 1 ) << CLKDIV_POS;

//	pRegister->TMRCLKGEN = regvalue;
	writel(regvalue, &pREG->tmrclkgen);
}

static inline void NX_TIMER_SetClockSource(struct timer_info *t_info)
{
	struct timer_info *info = t_info;
	struct nx_timer_register_set* pREG = (struct nx_timer_register_set*)info->clk_base;
	const U32	CLKSRCSEL_POS	= 2;
	const U32	CLKSRCSEL_MASK	= 0x7;
	U32 regvalue;	
	regvalue  = pREG->tmrclkgen;

	regvalue &= ~(CLKSRCSEL_MASK << CLKSRCSEL_POS);
	regvalue |= ( info->clksrc << CLKSRCSEL_POS );

	writel(regvalue, &pREG->tmrclkgen);
}

U32	NX_TIMER_GetTimerCounter(struct timer_info *t_info)
{
	struct timer_info *info = t_info;	
	struct nx_timer_register_set* pREG = (struct nx_timer_register_set*)info->clk_base;	
	
	const U32 INTPEND	= (1UL<<5);
	const U32 LDCNT		= (1UL<<6);

	U32 regvalue;

	regvalue = pREG->tmrcontrol;

	regvalue &= ~INTPEND;
	regvalue |=	LDCNT;

	writel(regvalue, &pREG->tmrcontrol);

	return pREG->tmrcount;
}

CBOOL NX_TIMER_GetClockDivisorEnable(struct timer_info *t_info)
{
	struct timer_info *info = t_info;	
	struct nx_timer_register_set* pREG = (struct nx_timer_register_set*)info->clk_base;	
	
	const U32 CLKGENENB_POS	= 2;

	return (CBOOL)( (pREG->tmrclkenb >> CLKGENENB_POS ) & 0x01 );
}
#if 0
static void timer_source_resume(struct clocksource *cs)
{
	struct timer_of_dev *dev = get_timer_dev();
	struct timer_info *info = &dev->timer_event;

	u_long flags;

	printk("+%s\n", __func__);

	local_irq_save(flags);

	/* setup timer as free-running clocksource */
	if (! NX_TIMER_GetClockDivisorEnable(info)) {
		printk("timer: resume source timer clock reset ...\n");
		NX_TIMER_SetClockSource(info);
		NX_TIMER_SetClockDivisor(info);
		NX_TIMER_SetClockPClkMode(info, NX_PCLKMODE_ALWAYS);
		NX_TIMER_SetClockDivisorEnable(info, CTRUE);
	}

	NX_TIMER_Stop(info);
	NX_TIMER_SetWatchDogEnable(info, CFALSE);
	NX_TIMER_SetInterruptEnableAll(info, CFALSE);
	NX_TIMER_SetTClkDivider(info, NX_TIMER_CLOCK_TCLK);
	NX_TIMER_ClearInterruptPendingAll(info);

	/* source timer run */
	NX_TIMER_SetTimerCounter(info, 0x00000000);
	NX_TIMER_SetMatchCounter(info,info->period);
	NX_TIMER_Run(info);

	local_irq_restore(flags);

	printk("-%s\n", __func__);
}
#endif

static cycle_t timer_source_read(struct clocksource *cs)
{
	struct timer_of_dev *dev = get_timer_dev();
	return NX_TIMER_GetTimerCounter(&dev->timer_source);
}


#ifdef CONFIG_ARM

static inline u32 timer_read_count(void)
{
	struct timer_of_dev *dev = get_timer_dev();
	unsigned int cnt;
	cnt = NX_TIMER_GetTimerCounter(&dev->timer_event);
//	printk("%s enter cnt %d\n", cnt);
	return cnt;
}

static struct delay_timer nxp_delay_timer = {
	.read_current_timer = (unsigned long (*)(void))timer_read_count,
};
#endif
#if 0
static struct clocksource timer_clocksource = {
	.name = "source timer",
	.rating = 300,
	.read = timer_source_read,
	.mask = CLOCKSOURCE_MASK(32),
	.shift = 20,
	.flags = CLOCK_SOURCE_IS_CONTINUOUS,
/*	.suspend = timer_source_suspend,*/
	.resume = timer_source_resume,
};
#endif

static int timer_event_set_oneshot(struct clock_event_device *evt)
{
	struct timer_of_dev *dev = get_timer_dev();
	struct timer_info *info = &dev->timer_event;
	return 0;
}

static int timer_event_set_periodic(struct clock_event_device *evt)
{
	struct timer_of_dev *dev = get_timer_dev();
	struct timer_info *info = &dev->timer_event;
	int i;
	
	//printk("set periodic base=%p period=%x\n", info->clk_base, info->period);
	NX_TIMER_Stop(info);
	NX_TIMER_SetTimerCounter(info, 0);
	NX_TIMER_SetMatchCounter(info, info->period);
	NX_TIMER_Run(info);

	return 0;
}

static void timer_event_resume(struct clock_event_device *evt)
{
	struct timer_of_dev *dev = get_timer_dev();
	struct timer_info *info = &dev->timer_event;
	u_long flags;

	local_irq_save(flags);

	printk("+%s\n", __func__);

	if (! NX_TIMER_GetClockDivisorEnable(info)) {
		printk("timer: resume event timer clock reset ...\n");
		NX_TIMER_SetClockSource(info);
		NX_TIMER_SetClockDivisor(info);
		NX_TIMER_SetClockPClkMode(info, NX_PCLKMODE_ALWAYS);
		NX_TIMER_SetClockDivisorEnable(info, CTRUE);
	}

	NX_TIMER_Stop(info);
	NX_TIMER_SetWatchDogEnable(info, CFALSE);
	NX_TIMER_SetInterruptEnableAll(info, CFALSE);
	NX_TIMER_SetTClkDivider(info, NX_TIMER_CLOCK_TCLK);
	NX_TIMER_ClearInterruptPendingAll(info);

	/* resume event timer */
	NX_TIMER_SetInterruptEnableAll(info, CTRUE);

	local_irq_restore(flags);

	printk("-%s\n", __func__);
}


static int timer_event_shutdown(struct clock_event_device *evt)
{
	struct timer_of_dev *dev = get_timer_dev();
	struct timer_info *info = &dev->timer_event;

	//NX_TIMER_Stop(info);

	return 0;
}


static int timer_event_set_next(unsigned long period, struct clock_event_device *evt)
{
	struct timer_of_dev *dev = get_timer_dev();
	struct timer_info *info = &dev->timer_event;
	unsigned long flags;

	raw_local_irq_save(flags);

	NX_TIMER_Stop(info);
	NX_TIMER_ClearInterruptPendingAll(info);
	NX_TIMER_SetTimerCounter(info, 0);
	NX_TIMER_SetMatchCounter(info, period);
	NX_TIMER_Run(info);

	raw_local_irq_restore(flags);
	return 0;
}


static struct clock_event_device timer_clock_event = {
	.name = "event timer",
	.shift = 32,
	.features = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.set_state_shutdown = timer_event_shutdown,
	.set_state_periodic = timer_event_set_periodic,
	.set_state_oneshot = timer_event_set_oneshot,
	.tick_resume = timer_event_shutdown,
	.set_next_event = timer_event_set_next,
	.resume = timer_event_resume,
	.rating = 300, 
};


static irqreturn_t timer_event_handler(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;
	struct timer_of_dev *dev = get_timer_dev();
	struct timer_info *info = &dev->timer_event;
	
	NX_TIMER_ClearInterruptPendingAll(info);

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static struct irqaction timer_event_irqaction = {
	.name = "Event Timer IRQ",
	.flags = IRQF_TIMER, /* removed IRQF_DISABLED kernel 4.1.15 */
	.handler = timer_event_handler,
	.dev_id = &timer_clock_event,
};
#if 0
static int __init timer_source_of_init(struct device_node *np,struct timer_info *t_info)
{
//	struct timer_of_dev *dev = get_timer_dev();
	struct timer_info *info = t_info;
	struct clocksource *cs = &timer_clocksource;

//	clocksource_register_hz(cs, info->clkfreq);
	printk("%s enter\n", __func__);
/* setup timer as free-running clocksource */
	NX_TIMER_SetClockSource(info);
	printk("==================1\n", __func__);
	NX_TIMER_SetClockDivisor(info);
	NX_TIMER_SetClockPClkMode(info, NX_PCLKMODE_ALWAYS);
	NX_TIMER_SetClockDivisorEnable(info, CTRUE);
	NX_TIMER_Stop(info);
	printk("==================2\n", __func__);	
	NX_TIMER_SetWatchDogEnable(info, CFALSE);
	NX_TIMER_SetInterruptEnableAll(info, CFALSE);
	NX_TIMER_SetTClkDivider(info, NX_TIMER_CLOCK_TCLK);
	NX_TIMER_ClearInterruptPendingAll(info);	
	
	NX_TIMER_SetTimerCounter(info, 0x00000000);
	NX_TIMER_SetMatchCounter(info,info->period);
	clocksource_register_hz(cs, info->clkfreq);	
	NX_TIMER_Run(info);
	printk("%s exit\n", __func__);
	//pr_debug("timer.%d: source, %9lu(HZ:%d), mult:%u\n", info->ch, info->rate, HZ,
	//       cs->mult);
	return 0;
}

#endif
static int __init timer_event_of_init(struct device_node *np,struct timer_info *t_info)
{
//	struct timer_of_dev *dev = get_timer_dev();
	struct timer_info *info = t_info;
	struct clock_event_device *evt = &timer_clock_event;	
	int err;

//	clocksource_register_hz(cs, info->clkfreq);

/* setup timer as free-running clocksource */
	NX_TIMER_SetClockSource(info);
	NX_TIMER_SetClockDivisor(info);
	NX_TIMER_SetClockPClkMode(info, NX_PCLKMODE_ALWAYS);
	NX_TIMER_SetClockDivisorEnable(info, CTRUE);
	NX_TIMER_Stop(info);
	NX_TIMER_SetWatchDogEnable(info, CFALSE);
	NX_TIMER_SetInterruptEnableAll(info, CFALSE);
	NX_TIMER_SetTClkDivider(info, NX_TIMER_CLOCK_TCLK);
	NX_TIMER_ClearInterruptPendingAll(info);	
	
	/* Make irqs happen for the system timer */
	NX_TIMER_SetInterruptEnableAll(info, CTRUE);
	
	NX_TIMER_SetTimerCounter(info, 0x00000000);

	setup_irq(info->irq, &timer_event_irqaction);
	if (err) {
		pr_err("setup irq failed: %d\n", err);
		return err;
	}	
	clockevents_calc_mult_shift(evt, info->clkfreq, 5);
	info->period = info->clkfreq / HZ;
	evt->max_delta_ns = clockevent_delta2ns(0xffffffff, evt);
	evt->min_delta_ns = clockevent_delta2ns(0xf, evt);
	evt->cpumask = cpumask_of(0);	
	evt->irq = info->irq;//irq_of_parse_and_map(info, 0);
	
//	err = setup_irq(info->irq, &timer_event_irqaction);
	

	clockevents_register_device(evt);


	printk("timer.%d: , mult:%u\n", info->ch, evt->mult);
	return 0;
}

static int timer_parse_dt(struct device_node *np, struct timer_info *t_info)
{
	struct timer_info *info = t_info;	
//	struct nx_timer_register_set* pREG = (struct nx_timer_register_set*)info->clk_base;	
	
	int irq = -1;
	struct resource res;
	int ret = 0;


	ret = of_address_to_resource(np,0,&res);
	if (ret) {
		pr_err("failed to get base address\n");
		return -ENXIO;
	}
	printk("res.start 0x%X\n", res.start);
	
	info->clk_base = (struct nx_timer_register_set*)ioremap_nocache(res.start, resource_size(&res));
	if(!info->clk_base){
		pr_err("failed to ioremap\n");
		return -EBUSY;
	}
	printk("info->clk_base 0x%X\n", info->clk_base);
	
/*	info->clk_base = of_iomap(np, 0);
	if (!info->clk_base) {
		pr_err("Can't map registers for timer!");
		return -EINVAL;
	}	
	printk("remap timer base 0x%X\n", info->clk_base);
	*/
	if(0 != of_property_read_u32(np, "clksrc", &t_info->clksrc)){
		pr_err("failed to get dt clksrc\n");
		return -EINVAL;
	}
	printk("clk clksrc 0x%X\n",t_info->clksrc);
	
	if(0 != of_property_read_u32(np, "clkdiv", &t_info->clkdiv)){
		pr_err("failed to get dt clkdiv\n");
		return -EINVAL;
	}
	printk("clk clkdiv 0x%X\n",t_info->clkdiv);
	
	if(0 != of_property_read_u32(np, "clkfreq", &t_info->clkfreq)){
		pr_err("failed to get dt clkfreq\n");
		return -EINVAL;
	}
	printk("clk clkfreq 0x%X\n",t_info->clkfreq);

	if(0 != of_property_read_u32(np, "period", &t_info->period)){
		pr_err("failed to get dt period\n");
		return -EINVAL;
	}
	printk("clk period 0x%X\n",t_info->period);
	
	irq = irq_of_parse_and_map(np, 0);
	if (irq <= 0) {
		pr_err("Can't parse IRQ");
		return -EINVAL;
	}

/*	irq = of_irq_get(irq_np, 0);
	if(irq < 0){
		pr_err("failed to get irq num np->name %s\n", np->name);
	}*/
	printk("clk irq 0x%X\n",irq);
	
	info->irq = irq;
	
	return 0;
}

static int __init
timer_get_device_data(struct device_node *node, struct timer_of_dev *dev)
{
	struct timer_info *tsrc = &dev->timer_source;
	struct timer_info *tevt = &dev->timer_event;
	struct device_node *np_clksrc;
	struct device_node *np_clkevt;

	printk("%s enter\n", __func__);
//	np_clksrc = of_parse_phandle(node, "timer", 0);
//	printk("np name %s \n", np_clksrc->name);
	np_clkevt = of_parse_phandle(node, "timer", 1);
	printk("np name %s \n", np_clkevt->name);
	
/*	if(np_clksrc){
		timer_parse_dt(np_clksrc,tsrc);
	}else{
		pr_err("Can't get of_node for clksrc timer!");
		return -EINVAL;
	}*/
	
	if(np_clkevt){
		timer_parse_dt(np_clkevt,tevt);
	}else{
		pr_err("Can't get of_node for clkevt timer!");
		return -EINVAL;
	}	
	
//	timer_source_of_init(np_clksrc, tsrc);
	timer_event_of_init(np_clkevt, tevt);

	

/*	tsrc->clk = of_clk_get(node, 0);
	if (IS_ERR(tsrc->clk)) {
		pr_err("failed timer tsrc clock\n");
		return -EINVAL;
	}

	tevt->clk = of_clk_get(node, 1);
	if (IS_ERR(tevt->clk)) {
		pr_err("failed timer event clock\n");
		return -EINVAL;
	}

	dev->pclk = of_clk_get(node, 2);
	if (IS_ERR(dev->pclk))
		dev->pclk = NULL;
*/
	printk("%s : ch %d,%d irq %d\n", node->name, tsrc->ch,
		 tevt->ch, tevt->irq);

	return 0;
}


static void __init nxp2120_delay_timer_init(void)
{
	nxp_delay_timer.freq = CLK_SOURCE_HZ;
	register_current_timer_delay(&nxp_delay_timer);
}

static int __init nxp2120_timer_init(struct device_node *node)
{
	struct timer_of_dev *dev = NULL;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);

	timer_dev = dev;

	
	if (timer_get_device_data(node, dev))
		panic("unable to map timer cpu !!!\n");

//	nxp2120_delay_timer_init();
//	timer_source_of_init(node);
//	timer_event_of_init(node);
	
	
	return 0;
}
CLOCKSOURCE_OF_DECLARE(nxp2120, "nexell,nxp2120-tim", nxp2120_timer_init);
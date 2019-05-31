/*
 * (C) Copyright 2010
 * jung hyun kim, Nexell Co, <jhkim@nexell.co.kr>
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
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/device.h>
#include <linux/signal.h>
#include <linux/platform_device.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/pm.h>
#include <linux/usb/ehci_pdriver.h>
#include "ehci.h"
/* nexell soc headers */
//#include <mach/platform.h>
//#include <mach/devices.h>

/* Debugging stuff */
#if (1)
#define DBGOUT(msg...)	{ printk(KERN_INFO "ehci: " msg); }
#else
#define DBGOUT(msg...) 
#endif


static bool usb_hc_init = false;
#define DRIVER_DESC "EHCI nxp2120 driver"
static const char hcd_name[] = "ehci-nxp2120";
#define PHY_NUMBER 1
struct nxp2120_ehci_hcd {
	struct usb_phy *phy[PHY_NUMBER]; /* one PHY for each port */
	int nports;
};	
static struct hc_driver __read_mostly nxp2120_ehci_hc_driver;

static inline struct ehci_hcd *hcd_to_ehci(struct usb_hcd *hcd);
#define to_nxp2120_ehci(hcd) (struct nxp2120_ehci_hcd *)(hcd_to_ehci(hcd)->priv)


#if 0
static int set_host_mode(void)
{
	u_int UDC_BASE = ioremap_nocache(PHY_BASEADDR_UDC);
	u_int UDC_MODE = ioremap_nocache(0xC0018848);

	/* CLOCK_GEN : clock div=0, 3:use ext_clk */
	*(volatile unsigned int *)(UDC_BASE + 0x8C4) = (
													(0<<15) |		// clk output enabled
													(0<< 5) |		// clock divide is 1
													(3<< 2) |		// ext clock
													(0<< 1)			// nomal clock output(falling edge)
													);


	/* CLOCK_ENB : clock always, clk enable */
	*(volatile unsigned int *)(UDC_BASE + 0x8C0) = (
													(1<<3) |	// PCLK Op mode is always
													(1<<2) |	// Clock Generation Enable
													(3<<0)		// usbd clock mode is always
													);

	/* PHY power disable */
	*(volatile unsigned short *)(UDC_BASE + 0x052) = (1<<0);

	*(volatile unsigned short *)(UDC_BASE + 0x840) = (
													(0<<9) |	// XO bias, and PLL blocks remain powered in suspend mode.
													(0<<7) |	// Ref Clk Freq is 12MHz
													(1<<5) |	// XO block uses an external clock supplied on the XO pin
													(1<<4) |	// POR Enable
													(0<<3) |	// POR
													(1<<2) |	// suspendm enable
													(1<<1) |	// suspend normal op mode
													(1<<0)		// ext VBUS enable
													);			// 12MHz,

	*(volatile unsigned short *)(UDC_BASE + 0x052) = (
													(1<<0) |
													(1<<7)
													);			// phy block off, utmi reset

	/* Dummy delay : required over 10 usec */
	udelay(10);

	*(volatile unsigned short *)(UDC_BASE + 0x052) = 0;			// phy block on

	*(volatile unsigned short *)(UDC_BASE + 0x848) = (
													(0<<9) |	// pull-down resistance on D+ is disabled
													(0<<8) |	// pull-down resistance on D- is disabled
													(0<<7) |	// this bit actived if opmode is 2b11
													(0<<6) |	// this bit actived if opmode is 2b11
													(1<<5) |	// OTG block is powered down
													(1<<4) |	// Ext VBus is enable
													(1<<3) |	// VBus valid comparator is enabled.
													(0<<2) |	// disabled charging VBus through the  SRP pull-up resistance
													(0<<1) |	// disable dischrging VBus through the SRP pull-down resistance
													(0<<0)		// ID pin sampling is disbled, and IDDDIG output is not valid.
													);

	/* set usb host mode */
	*(volatile unsigned int *)(UDC_MODE) = (*(volatile unsigned int *)(UDC_MODE) | (1<<10));

	return 0;
}

static void start_usb_hc(struct device *dev, bool suspend)
{
	u_int HC_BASE = IO_ADDRESS(PHY_BASEADDR_EHCI);
	u_int IO_BASE = __PB_IO_MAP_REGS_VIRT;

#if defined(CONFIG_USB_GADGET_NXP2120) || defined(CONFIG_USB_GADGET_NXP2120_MODULE)
#error	"Not support usb device, when use usb host"
#endif

	if(true == usb_hc_init)
		return;

	DBGOUT("%s\n", __func__);

	/* set host mode */
	set_host_mode();

	/* set host module */
	*(volatile unsigned int *)(HC_BASE + 0x3C4) =
											0<<15 |	// Specifies the direction of the PADVCLK pad. You have to set this bit when CLKSRCSEL0/1 is 3 or 4. 0 : Enable(Output)                    1 : Reserved
											0<<5 |	// Specifies divider value of the source clock.
													//	Divider value = CLKDIV0 + 1
													//	Clock Divisor. For divide - by - N, enter an [N - 1] value.
													//	000000 ~ 111111(N-1) : Divide Value (N) = Divide by 1 to 64
													//	Ex) For divide-by-2 : [0001b]
											3<<2 |	// Clock Source Selection
													//	000 : Reserved			001 : Resrved
													//	010 : Reserved			011 : Ext Clock
													//	100 ~ 111 : Reserved
											0<<1;	// Specifies whether to invert the clock output.
													// 0 : Normal (Falling Edge)	1 : Invert (Rising Edge)


	*(volatile unsigned int *)(HC_BASE + 0x3C0) =
											1<<3 |	// PCLK Operating Mode 0 : Clock is enabled only when CPU accesses 1 : Always
											1<<2 |	// Clock Generation Enable 0 : Disable	1 : Enable
											3<<0;	// USBD Clock Enable
													// 00 : Disable		01 : Reserved		10 : Dynamic	11 : Always
	msleep(1);

	*(volatile unsigned int *)(HC_BASE + 0x0C0) =
											0<<9 |	// Common Block Power-Down Control :
														//	Function: This signal controls the power-down signals in the XO, Bias, and PLL blocks when the USB 2.0 nanoPHY is suspended.
														//	o 1: The XO, Bias, and PLL blocks are powered down in Suspend mode.
														//	o 0: The XO, Bias, and PLL blocks remain powered in Suspend mode.
											0<<7 |	// Reference Clock Frequency Select
													//	Function: This bus selects the USB 2.0 nanoPHY reference clock frequency.
													//	o 11: Reserved
													//	o 10: 48 MHz
													//	o 01: 24 MHz
													//	o 00: 12 MHz
											1<<5 |	// Reference Clock Select for PLL Block
													//	Function: This signal selects the reference clock source for the PLL block.
													//	o 11: The PLL uses CLKCORE as reference.
													//	o 10: The PLL uses CLKCORE as reference.
													//	o 01: The XO block uses an external clock supplied on the XO pin.
													//	o 00: The XO block uses the clock from a crystal.
											0<<4 |	// POR Enbale
													// 0 : Disable	1: Enable
											0<<3 |	// Power-On Reset
													// Function: This customer-specific signal resets all test registers and state machines in the USB 2.0 nanoPHY.
													// The POR signal must be asserted for a minimum of 10 ¥ìs.
											0<<2 |	// SuspendM Enbale
													// 0 : Disable	1: Enable
											0<<1;	// Suspend Assertion
													//	Function: Asserting this signal suspends the USB 2.0 nanoPHY by tristating the transmitter and powering down the USB 2.0 nanoPHY circuits.
													//	o 1: Normal operating mode
													//	o 0: Suspend mode
													//	USB 2.0 nanoPHY power-down behavior can be overridden using the test interface.
	msleep(1);

	*(volatile unsigned int *)(HC_BASE + 0x0C4) = 0;		// VBUS Interrupt Enbale
													// 0 : Disable	1: Enable
	msleep(1);

	*(volatile unsigned int *)(HC_BASE + 0x0C8) = 0;		// VBUS Interrupt Pending Enbale
													// 0 : Disable	1: Enable
	msleep(1);

	*(volatile unsigned int *)(HC_BASE + 0x0CC) =
											0<<9 |	// TermSel Enbale
													// 0 : Disable	1: Enable
											0<<8 |	// USB Termination Select
													//	Function: This controller signal sets FS or HS termination.
													//	o 1: Full-speed termination is enabled.
													//	o 0: High-speed termination is enabled.
											1<<7 |	// WordInterface Enbale
													// 0 : Disable	1: Enable
											1<<6 |	// UTMI Data Bus Width and Clock Select
													//	Function: This controller signal selects the data bus width of the UTMI data input and output paths.
													//	o 1: 16-bit data interface
													//	o 0: 8-bit data interface
													//	The USB 2.0 nanoPHY supports 8/16-bit data interfaces for all valid USB 2.0 nanoPHY speed modes.
											0<<5 |	// OPMode Enbale
													// 0 : Disable	1: Enable
											0<<3 |	// UTMI Operational Mode
													//	Function: This controller bus selects the UTMI operational mode.
													//	o 11: Normal operation without SYNC or EOP generation.
													//			If the XCVRSEL bus is not set to 2'b00 when OPMODE[1:0] is set to 2'b11,
													//			USB 2.0 nanoPHY behavior is undefined.
													//	o 10: Disable bit stuffing and NRZI encoding
													//	o 01: Non-driving
													//	o 00: Normal
											0<<2 |	// XCVRSel Enbale
													// 0 : Disable	1: Enable
											0<<0;	// Transceiver Select
													//	Function: This controller bus selects the HS, FS, or LS Transceiver.
													//	o 11: Sends an LS packet on an FS bus or receives an LS packet.
													//	o 10: LS Transceiver
													//	o 01: FS Transceiver
													//	o 00: HS Transceiver
	msleep(1);
	*(volatile unsigned int *)(HC_BASE + 0x0D0) = 0x320;	//

	msleep(1);
	*(volatile unsigned int *)(HC_BASE + 0x0D4) = 1<<15 | 1<<14 | 1<<13 | 0x20<<7 | 0x20<<1;	// default value

	msleep(1);
	*(volatile unsigned int *)(HC_BASE + 0x0D8) = 0xB37B;	// default value 0x6D78

	msleep(1);

	printk(KERN_INFO "%s: usb host EHCI version 0x%08x\n",
		EHCI_DEV_NAME, *(volatile unsigned int *)(IO_BASE + 0x00019800));

	usb_hc_init = true;
}

static void stop_usb_hc(struct device *dev)
{
	DBGOUT("%s\n", __func__);

	usb_hc_init = false;
}
#endif

/**
 * usb_hcd_nx_probe - initialize nexell-based HCDs
 * Context: !in_interrupt()
 *
 * Allocates basic resources for this USB host controller, and
 * then invokes the start() method for the HCD associated with it
 * through the hotplug entry's driver_data.
 *
 */
static int usb_hcd_nx_probe(struct platform_device *pdev)
{
	struct nxp2120_ehci_hcd *nxp2120_ehci;
	struct ehci_hcd *ehci;
	struct resource	*res;
	struct usb_hcd	*hcd;	
	int irq;
	int err;

	printk("%s\n", __func__);

	if (usb_disabled())
		return -ENODEV;

	hcd = usb_create_hcd(&nxp2120_ehci_hc_driver,
			     &pdev->dev, dev_name(&pdev->dev));
	if (!hcd) {
		dev_err(&pdev->dev, "Unable to create HCD\n");
		return -ENOMEM;
	}
	nxp2120_ehci = to_nxp2120_ehci(hcd);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "Unable to get IRQ resource\n");
		err = -ENODEV;		
		usb_put_hcd(hcd);
	}
	//hcd->irq = ret;
	
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Unable to get memory resource\n");
		err = -ENODEV;
		usb_put_hcd(hcd);
	}
	
	hcd->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(hcd->regs)) {
		err = PTR_ERR(hcd->regs);
		usb_put_hcd(hcd);
	}

	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);
	
	ehci = hcd_to_ehci(hcd);
	ehci->caps = hcd->regs;
	
	err = usb_add_hcd(hcd, irq, IRQF_SHARED);
	if (err) {
		dev_err(&pdev->dev, "Failed to add USB HCD\n");
			usb_put_hcd(hcd);
	}
	device_wakeup_enable(hcd->self.controller);
	platform_set_drvdata(pdev, hcd);

#ifdef CONFIG_NXP2120_USB_BUGFIX
	ehci->nxp2120_bugfix_count = 0;
	/* ksw : allocate maximum length buffer to all the way, and don't have to deallocate. */
	{
		int i, size = 16384;
		dma_addr_t new_dma_addr;

		//printk("self controller = %X\n", hcd->self.controller);
		for (i=0; i<MAX_NXP2120_BUGFIX_COUNT; i++) {
			ehci->nxp2120_bugfix[i].active = 0;
			ehci->nxp2120_bugfix[i].transfer_buffer = dma_alloc_coherent(hcd->self.controller, size, &new_dma_addr, GFP_KERNEL);
			//printk("dma_alloc = %X\n", ehci->nxp2120_bugfix[i].transfer_buffer);
			ehci->nxp2120_bugfix[i].transfer_dma = new_dma_addr;
			ehci->nxp2120_bugfix[i].transfer_buffer_size = size;
		}
	}
#endif
	
#if 0	
	if (pdev->resource[1].flags != IORESOURCE_IRQ) {
		pr_debug("resource[1] is not IORESOURCE_IRQ");
		return -ENOMEM;
	}

	hcd = usb_create_hcd(driver, &pdev->dev, EHCI_DEV_NAME);
	if (!hcd)
		return -ENOMEM;

	hcd->rsrc_start = pdev->resource[0].start;
	hcd->rsrc_len   = pdev->resource[0].end - pdev->resource[0].start + 1;

	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len, hcd_name)) {
		pr_debug("request_mem_region failed");
		ret = -EBUSY;
		goto err1;
	}

	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
	if (!hcd->regs) {
		pr_debug("ioremap failed");
		ret = -ENOMEM;
		goto err2;
	}

	start_usb_hc(&pdev->dev, false);

	ehci = hcd_to_ehci(hcd);
	ehci->caps = hcd->regs;
	ehci->regs = hcd->regs + HC_LENGTH(ehci,readl(&ehci->caps->hc_capbase));
#ifdef CONFIG_NXP2120_USB_BUGFIX
	ehci->nxp2120_bugfix_count = 0;
	/* ksw : allocate maximum length buffer to all the way, and don't have to deallocate. */
	{
		int i, size = 16384;
		dma_addr_t new_dma_addr;

		//printk("self controller = %X\n", hcd->self.controller);
		for (i=0; i<MAX_NXP2120_BUGFIX_COUNT; i++) {
			ehci->nxp2120_bugfix[i].active = 0;
			ehci->nxp2120_bugfix[i].transfer_buffer = dma_alloc_coherent(hcd->self.controller, size, &new_dma_addr, GFP_KERNEL);
			//printk("dma_alloc = %X\n", ehci->nxp2120_bugfix[i].transfer_buffer);
			ehci->nxp2120_bugfix[i].transfer_dma = new_dma_addr;
			ehci->nxp2120_bugfix[i].transfer_buffer_size = size;
		}
	}
#endif
	/* cache this readonly data; minimize chip reads */
	ehci->hcs_params = readl(&ehci->caps->hcs_params);

	ret = usb_add_hcd(hcd, pdev->resource[1].start,
			  IRQF_DISABLED | IRQF_SHARED);
	if (ret == 0) {
		platform_set_drvdata(pdev, hcd);
		return ret;
	}                                                     

	stop_usb_hc(&pdev->dev);

	iounmap(hcd->regs);


err2:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
err1:
	usb_put_hcd(hcd);
#endif
	return err;
}


/**
 * usb_hcd_nx_remove - shutdown processing for nexell-based HCDs
 * @dev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of usb_hcd_nx_probe(), first invoking
 * the HCD's stop() method.  It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 *
 */
static void usb_hcd_nx_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct nxp2120_ehci_hcd *ehci = to_nxp2120_ehci(hcd);
	DBGOUT("%s\n", __func__);

	usb_remove_hcd(hcd);
//	iounmap(hcd->regs);
//	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);

//	stop_usb_hc(&pdev->dev);
}

#if 0
static const struct hc_driver ehci_hc_driver = {
	.description =		hcd_name,
	.product_desc =		"Nexell EHCI ",
	.hcd_priv_size =	sizeof(struct ehci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq =			ehci_irq,
	.flags =		HCD_MEMORY | HCD_USB2,

	/*
	 * basic lifecycle operations
	 *
	 * FIXME -- ehci_init() doesn't do enough here.
	 * See ehci-ppc-soc for a complete implementation.
	 */
	.reset			= ehci_init,
	.start			= ehci_run,
	.stop			= ehci_stop,
	.shutdown		= ehci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		ehci_urb_enqueue,
	.urb_dequeue =		ehci_urb_dequeue,
	.endpoint_disable =	ehci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number =	ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data	= ehci_hub_status_data,
	.hub_control		= ehci_hub_control,
	.bus_suspend		= ehci_bus_suspend,
	.bus_resume			= ehci_bus_resume,
	.relinquish_port	= ehci_relinquish_port,
	.port_handed_over	= ehci_port_handed_over,
};
#endif

static int ehci_nx_driver_probe(struct platform_device *pdev)
{
	DBGOUT("%s\n", __func__);

	return usb_hcd_nx_probe(pdev);
}


static int ehci_nx_driver_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	
	DBGOUT("%s\n", __func__);

	usb_hcd_nx_remove(pdev);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef	CONFIG_PM
static int ehci_nx_driver_suspend(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	struct nxp2120_ehci_hcd *ehci = to_nxp2120_ehci(hcd);
	unsigned long flags;
	bool do_wakeup = device_may_wakeup(dev);
	int rc;
	printk("+%s\n", __func__);
	rc = ehci_suspend(hcd, do_wakeup);
	if (rc)
		return rc;	
#if 0
	if (time_before(jiffies, ehci->next_statechange))
		mdelay(100);

	/* Root hub was already suspended. Disable irq emission and
	 * mark HW unaccessible, bail out if RH has been resumed. Use
	 * the spinlock to properly synchronize with possible pending
	 * RH suspend or resume activity.
	 *
	 * This is still racy as hcd->state is manipulated outside of
	 * any locks =P But that will be a different fix.
	 */
	spin_lock_irqsave(&ehci->lock, flags);
	if (hcd->state != HC_STATE_SUSPENDED) {
		rc = -EINVAL;
		goto bail;
	}
	ehci_writel(ehci, 0, &ehci->regs->intr_enable);
	(void)ehci_readl(ehci, &ehci->regs->intr_enable);

	/* make sure snapshot being resumed re-enumerates everything */
	if (message.event == PM_EVENT_PRETHAW) {
		ehci_halt(ehci);
		ehci_reset(ehci);
	}

	clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

	stop_usb_hc(&pdev->dev);


bail:
	spin_unlock_irqrestore(&ehci->lock, flags);

	// could save FLADJ in case of Vaux power loss
	// ... we'd only use it to handle clock skew
#endif 
	printk("-%s (rc=0x%x)\n", __func__, rc);

	return rc;
}

static int ehci_nx_driver_resume(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	struct nxp2120_ehci_hcd *ehci = to_nxp2120_ehci(hcd);

	printk("+%s\n", __func__);
#if 0
	start_usb_hc(&pdev->dev, true);

	// maybe restore FLADJ

	if (time_before(jiffies, ehci->next_statechange))
		mdelay(100);

	/* Mark hardware accessible again as we are out of D3 state by now */
	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

	/* If CF is still set, we maintained PCI Vaux power.
	 * Just undo the effect of ehci_pci_suspend().
	 */
	if (ehci_readl(ehci, &ehci->regs->configured_flag) == FLAG_CF) {
		int	mask = INTR_MASK;

		if (!hcd->self.root_hub->do_remote_wakeup)
			mask &= ~STS_PCD;
		ehci_writel(ehci, mask, &ehci->regs->intr_enable);
		ehci_readl(ehci, &ehci->regs->intr_enable);
		return 0;
	}

	ehci_dbg(ehci, "lost power, restarting\n");
	usb_root_hub_lost_power(hcd->self.root_hub);

	/* Else reset, to cope with power loss or flush-to-storage
	 * style "resume" having let BIOS kick in during reboot.
	 */
	(void) ehci_halt(ehci);
	(void) ehci_reset(ehci);

	/* emptying the schedule aborts any urbs */
	spin_lock_irq(&ehci->lock);
	if (ehci->reclaim)
		end_unlink_async(ehci);
	ehci_work(ehci);
	spin_unlock_irq(&ehci->lock);

	ehci_writel(ehci, ehci->command, &ehci->regs->command);
	ehci_writel(ehci, FLAG_CF, &ehci->regs->configured_flag);
	ehci_readl(ehci, &ehci->regs->command);	/* unblock posted writes */

	/* here we "know" root ports should always stay powered */
	ehci_port_power(ehci, 1);

	hcd->state = HC_STATE_SUSPENDED;
#endif
	ehci_resume(hcd, false);
	printk("-%s\n", __func__);
	return 0;
}
#else
static int ehci_nx_driver_suspend(struct device *dev)
{
	printk("%s\n", __func__);
	return 0;
}

static int ehci_nx_driver_resume(struct device *dev)
{
	printk("%s\n", __func__);
	return 0;
}
#endif

#if CONFIG_PM
static const struct dev_pm_ops nxp2120_ehci_pm_ops = {
	.suspend	= ehci_nx_driver_suspend,
	.resume		= ehci_nx_driver_resume,
};
#endif

#ifdef CONFIG_OF
static const struct of_device_id nxp2120_ehci_match[] = {
	{ .compatible = "nexell,nxp2120-ehci" },	
	{},
};
MODULE_DEVICE_TABLE(of, nxp2120_ehci_match);
#endif

static struct platform_driver nxp2120_ehci_driver = {
	.probe		= ehci_nx_driver_probe,
	.remove		= ehci_nx_driver_remove,
	.shutdown	= usb_hcd_platform_shutdown,
	.driver		= {
	//	.owner	= THIS_MODULE,
		.name	= "nxp2120-ehci",
#if CONFIG_PM
		.pm = &nxp2120_ehci_pm_ops,
#endif
		.of_match_table = of_match_ptr(nxp2120_ehci_match),
	},
};


static const struct ehci_driver_overrides nxp2120_overrides __initconst = {
	.extra_priv_size = sizeof(struct nxp2120_ehci_hcd),

};

static int __init ehci_nxp2120_init(void)
{
	if (usb_disabled())
		return -ENODEV;

	pr_info("%s: " DRIVER_DESC "\n", hcd_name);
	ehci_init_driver(&nxp2120_ehci_hc_driver, &nxp2120_overrides);
	return platform_driver_register(&nxp2120_ehci_driver);
}
module_init(ehci_nxp2120_init);

static void __exit ehci_nxp2120_cleanup(void)
{
	platform_driver_unregister(&nxp2120_ehci_driver);
}
module_exit(ehci_nxp2120_cleanup);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_ALIAS("platform:nxp2120-ehci");
MODULE_AUTHOR("ryu & kim");
MODULE_LICENSE("GPL v2");
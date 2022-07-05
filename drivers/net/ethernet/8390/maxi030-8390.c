/* MAXI030's RTL8019AS Ethernet Driver
 *
 * Copyright (C) Lawrence Manning 2022, largely based largely on:
 *
 * Amiga Linux/m68k and Linux/PPC Zorro NS8390 Ethernet Driver
 *
 *  (C) Copyright 2021 Lawrence Manning, another Elitist 680x0 user
 *  (C) Copyright 1998-2000 by some Elitist 680x0 Users(TM)
 *
 *  ---------------------------------------------------------------------------
 *
 *  This program is based on all the other NE2000 drivers for Linux
 *
 *  ---------------------------------------------------------------------------
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License.  See the file COPYING in the main directory of the Linux
 *  distribution for more details.
 *
 *  ---------------------------------------------------------------------------
 *
 *  The Ariadne II and X-Surf are Zorro-II boards containing Realtek RTL8019AS
 *  Ethernet Controllers.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/jiffies.h>
#include <linux/platform_device.h>

#include <asm/irq.h>
#include <asm/amigaints.h>
#include <asm/amigahw.h>

#define EI_SHIFT(x)             (ei_local->reg_offset[x])
#define ei_inb(port)            in_8(port)
#define ei_outb(val, port)      out_8(port, val)
#define ei_inb_p(port)          in_8(port)
#define ei_outb_p(val, port)    out_8(port, val)

static const char version[] =
	"8390.c:v1.10cvs 9/23/94 Donald Becker (becker@cesdis.gsfc.nasa.gov)\n";

#include "lib8390.c"

#define DRV_NAME	"maxi030_8390"

#define NE_BASE		(dev->base_addr)
#define NE_CMD		(0x00 * 2)
#define NE_DATAPORT	(0x12 * 2)	/* NatSemi-defined port window offset */
#define NE_RESET	(0x1f * 2)	/* Issue a read to reset,
					 * a write to clear. */
#define NE_IO_EXTENT	(0x20 * 2)

#define NE_EN0_ISR	(0x07 * 2)
#define NE_EN0_DCFG	(0x0e * 2)

#define NE_EN0_RSARLO	(0x08 * 2)
#define NE_EN0_RSARHI	(0x09 * 2)
#define NE_EN0_RCNTLO	(0x0a * 2)
#define NE_EN0_RXCR	(0x0c * 2)
#define NE_EN0_TXCR	(0x0d * 2)
#define NE_EN0_RCNTHI	(0x0b * 2)
#define NE_EN0_IMR	(0x0f * 2)

#define NESM_START_PG	0x40	/* First page of TX buffer */
#define NESM_STOP_PG	0x80	/* Last page +1 of RX ring */

#define WORDSWAP(a)	((((a) >> 8) & 0xff) | ((a) << 8))

/* Page 3 */
#define E8390_PAGE3     0xc0    /* RTL8109AS specfic. */
#define EN3_CONFIG1     (0x04 * 2)

/* Hard reset the card.  This used to pause for the same period that a
 * 8390 reset command required, but that shouldn't be necessary.
 */
static void maxi030_8390_reset_8390(struct net_device *dev)
{
	unsigned long reset_start_time = jiffies;
	struct ei_device *ei_local = netdev_priv(dev);

	netif_dbg(ei_local, hw, dev, "resetting - t=%ld...\n", jiffies);


	writeb(readb(NE_BASE + NE_RESET), NE_BASE + NE_RESET);

	ei_status.txing = 0;
	ei_status.dmaing = 0;

	/* This check _should_not_ be necessary, omit eventually. */
	while ((readb(NE_BASE + NE_EN0_ISR) & ENISR_RESET) == 0)
		if (time_after(jiffies, reset_start_time + 2 * HZ / 100)) {
			netdev_warn(dev, "%s: did not complete\n", __func__);
			break;
		}
	writeb(ENISR_RESET, NE_BASE + NE_EN0_ISR);	/* Ack intr */
}

/* Grab the 8390 specific header. Similar to the block_input routine, but
 * we don't need to be concerned with ring wrap as the header will be at
 * the start of a page, so we optimize accordingly.
 */
static void maxi030_8390_get_8390_hdr(struct net_device *dev,
				   struct e8390_pkt_hdr *hdr, int ring_page)
{
	int nic_base = dev->base_addr;
	int cnt;
	unsigned char *ptrs;

	/* This *shouldn't* happen.
	 * If it does, it's the last thing you'll see
	 */
	if (ei_status.dmaing) {
		netdev_warn(dev,
			    "%s: DMAing conflict [DMAstat:%d][irqlock:%d]\n",
			    __func__, ei_status.dmaing, ei_status.irqlock);
		return;
	}

	ei_status.dmaing |= 0x01;
	writeb(E8390_NODMA + E8390_PAGE0 + E8390_START, nic_base + NE_CMD);
	writeb(ENISR_RDC, nic_base + NE_EN0_ISR);
	writeb(sizeof(struct e8390_pkt_hdr), nic_base + NE_EN0_RCNTLO);
	writeb(0, nic_base + NE_EN0_RCNTHI);
	writeb(0, nic_base + NE_EN0_RSARLO);		/* On page boundary */
	writeb(ring_page, nic_base + NE_EN0_RSARHI);
	writeb(E8390_RREAD+E8390_START, nic_base + NE_CMD);

	ptrs = (unsigned char *)hdr;
	for (cnt = 0; cnt < sizeof(struct e8390_pkt_hdr); cnt++)
		*ptrs++ = readb(NE_BASE + NE_DATAPORT);

	writeb(ENISR_RDC, nic_base + NE_EN0_ISR);	/* Ack intr */

	hdr->count = WORDSWAP(hdr->count);

	ei_status.dmaing &= ~0x01;
}

/* Block input and output, similar to the Crynwr packet driver.
 * If you are porting to a new ethercard, look at the packet driver source
 * for hints. The NEx000 doesn't share the on-board packet memory --
 * you have to put the packet out through the "remote DMA" dataport
 * using writeb.
 */
static void maxi030_8390_block_input(struct net_device *dev, int count,
				  struct sk_buff *skb, int ring_offset)
{
	int nic_base = dev->base_addr;
	char *buf = skb->data;
	unsigned char *ptrs;
	int cnt;

	/* This *shouldn't* happen.
	 * If it does, it's the last thing you'll see
	 */
	if (ei_status.dmaing) {
		netdev_err(dev, "%s: DMAing conflict [DMAstat:%d][irqlock:%d]\n",
			   __func__, ei_status.dmaing, ei_status.irqlock);
		return;
	}
	ei_status.dmaing |= 0x01;
	writeb(E8390_NODMA + E8390_PAGE0 + E8390_START, nic_base + NE_CMD);
	writeb(ENISR_RDC, nic_base + NE_EN0_ISR);
	writeb(count & 0xff, nic_base + NE_EN0_RCNTLO);
	writeb(count >> 8, nic_base + NE_EN0_RCNTHI);
	writeb(ring_offset & 0xff, nic_base + NE_EN0_RSARLO);
	writeb(ring_offset >> 8, nic_base + NE_EN0_RSARHI);
	writeb(E8390_RREAD+E8390_START, nic_base + NE_CMD);
	ptrs = (unsigned char *)buf;
	for (cnt = 0; cnt < count; cnt++)
		*ptrs++ = readb(NE_BASE + NE_DATAPORT);

	writeb(ENISR_RDC, nic_base + NE_EN0_ISR);	/* Ack intr */
	ei_status.dmaing &= ~0x01;
}

static void maxi030_8390_block_output(struct net_device *dev, int count,
				   const unsigned char *buf,
				   const int start_page)
{
	int nic_base = NE_BASE;
	unsigned long dma_start;
	unsigned char *ptrs;
	int cnt;

	/* Round the count up for word writes.  Do we need to do this?
	 * What effect will an odd byte count have on the 8390?
	 * I should check someday.
	 */
//	if (count & 0x01)
//		count++;

	/* This *shouldn't* happen.
	 * If it does, it's the last thing you'll see
	 */
	if (ei_status.dmaing) {
		netdev_err(dev, "%s: DMAing conflict [DMAstat:%d][irqlock:%d]\n",
			   __func__, ei_status.dmaing, ei_status.irqlock);
		return;
	}
	ei_status.dmaing |= 0x01;
	/* We should already be in page 0, but to be safe... */
	writeb(E8390_PAGE0+E8390_START+E8390_NODMA, nic_base + NE_CMD);

	writeb(ENISR_RDC, nic_base + NE_EN0_ISR);

	/* Now the normal output. */
	writeb(count & 0xff, nic_base + NE_EN0_RCNTLO);
	writeb(count >> 8,   nic_base + NE_EN0_RCNTHI);
	writeb(0x00, nic_base + NE_EN0_RSARLO);
	writeb(start_page, nic_base + NE_EN0_RSARHI);

	writeb(E8390_RWRITE + E8390_START, nic_base + NE_CMD);
	ptrs = (unsigned char *)buf;
	for (cnt = 0; cnt < count; cnt++)
		writeb(*ptrs++, NE_BASE + NE_DATAPORT);

	dma_start = jiffies;

	while ((readb(NE_BASE + NE_EN0_ISR) & ENISR_RDC) == 0)
		if (time_after(jiffies, dma_start + 2 * HZ / 100)) {
					/* 20ms */
			netdev_warn(dev, "timeout waiting for Tx RDC\n");
			maxi030_8390_reset_8390(dev);
			__NS8390_init(dev, 1);
			break;
		}

	writeb(ENISR_RDC, nic_base + NE_EN0_ISR);	/* Ack intr */
	ei_status.dmaing &= ~0x01;
}

static int maxi030_8390_open(struct net_device *dev)
{
	__ei_open(dev);
	return 0;
}

static int maxi030_8390_close(struct net_device *dev)
{
	struct ei_device *ei_local = netdev_priv(dev);

	netif_dbg(ei_local, ifdown, dev, "Shutting down ethercard\n");
	__ei_close(dev);
	return 0;
}

static int maxi030_8390_remove_one(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);

	unregister_netdev(dev);
	if (dev->irq)
		free_irq(dev->irq, dev);
#if 0
	release_mem_region(ZTWO_PADDR(dev->base_addr), NE_IO_EXTENT);
#endif
	free_netdev(dev);

	return 0;
}

static const struct net_device_ops maxi030_8390_netdev_ops = {
	.ndo_open		= maxi030_8390_open,
	.ndo_stop		= maxi030_8390_close,
	.ndo_start_xmit		= __ei_start_xmit,
	.ndo_tx_timeout		= __ei_tx_timeout,
	.ndo_get_stats		= __ei_get_stats,
	.ndo_set_rx_mode	= __ei_set_multicast_list,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_set_mac_address	= eth_mac_addr,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller	= __ei_poll,
#endif
};

static int maxi030_8390_init(struct net_device *dev,
			  const char *name, void __iomem *ioaddr, int irq)
{
	int i;
	int err;
	unsigned char SA_prom[32];
	unsigned char mac[ETH_ALEN] = {
		0x00, 0x11, 0x22, 0x33, 0x44, 0x55
	};
	int start_page, stop_page;
	static u32 maxi030_8390_offsets[16] = {
		0x00, 0x02, 0x04, 0x06, 0x08, 0x0a, 0x0c, 0x0e,
		0x10, 0x12, 0x14, 0x16, 0x18, 0x1a, 0x1c, 0x1e,
	};

	/* Reset card. Who knows what dain-bramaged state it was left in. */
	{
		unsigned long reset_start_time = jiffies;

		writeb(readb(ioaddr + NE_RESET), ioaddr + NE_RESET);

		while ((readb(ioaddr + NE_EN0_ISR) & ENISR_RESET) == 0)
			if (time_after(jiffies,
				       reset_start_time + 2 * HZ / 100)) {
				netdev_warn(dev, "not found (no reset ack)\n");
				return -ENODEV;
			}

		writeb(0xff, ioaddr + NE_EN0_ISR);	/* Ack all intr. */
	}

	/* Read the 16 bytes of station address PROM.
	 * We must first initialize registers,
	 * similar to NS8390_init(eifdev, 0).
	 * We can't reliably read the SAPROM address without this.
	 * (I learned the hard way!).
	 */
	{
		static const struct {
			u32 value;
			u32 offset;
		} program_seq[] = {
			{E8390_NODMA + E8390_PAGE3 + E8390_STOP, NE_CMD},
						/* Select page 3 */
			{0x80,  EN3_CONFIG1},	/* INT0 on */
			{E8390_NODMA + E8390_PAGE0 + E8390_STOP, NE_CMD},
						/* Select page 0 */
			{0x48,	NE_EN0_DCFG},	/* 0x48: Set byte-wide access */
			{0x00,	NE_EN0_RCNTLO},	/* Clear the count regs */
			{0x00,	NE_EN0_RCNTHI},
			{0x00,	NE_EN0_IMR},	/* Mask completion irq */
			{0xFF,	NE_EN0_ISR},
			{E8390_RXOFF, NE_EN0_RXCR}, /* 0x20 Set to monitor */
			{E8390_TXOFF, NE_EN0_TXCR}, /* 0x02 and loopback mode */
			{32,	NE_EN0_RCNTLO},
			{0x00,	NE_EN0_RCNTHI},
			{0x00,	NE_EN0_RSARLO},	/* DMA starting at 0x0000 */
			{0x00,	NE_EN0_RSARHI},
			{E8390_RREAD + E8390_START, NE_CMD},
		};
		for (i = 0; i < ARRAY_SIZE(program_seq); i++)
			writeb(program_seq[i].value,
				 ioaddr + program_seq[i].offset);
	}

        for (i = 0; i < 16; i++) {
                SA_prom[i] = readb(ioaddr + NE_DATAPORT);
                (void)readb(ioaddr + NE_DATAPORT);
        }


	/* We must set the 8390 for word mode. */
	writeb(0x48, ioaddr + NE_EN0_DCFG);
	start_page = NESM_START_PG;
	stop_page = NESM_STOP_PG;

	dev->base_addr = (unsigned long)ioaddr;
	dev->irq = irq;

	/* Install the Interrupt handler */
	i = request_irq(irq, __ei_interrupt, IRQF_SHARED, DRV_NAME, dev);
	if (i)
		return i;

	eth_hw_addr_set(dev, mac);

	pr_debug("Found ethernet address: %pM\n", dev->dev_addr);

	ei_status.name = name;
	ei_status.tx_start_page = start_page;
	ei_status.stop_page = stop_page;
	ei_status.word16 = 0;

	ei_status.rx_start_page = start_page + TX_PAGES;

	ei_status.reset_8390 = maxi030_8390_reset_8390;
	ei_status.block_input = maxi030_8390_block_input;
	ei_status.block_output = maxi030_8390_block_output;
	ei_status.get_8390_hdr = maxi030_8390_get_8390_hdr;
	ei_status.reg_offset = maxi030_8390_offsets;

	dev->netdev_ops = &maxi030_8390_netdev_ops;
	__NS8390_init(dev, 0);

	err = register_netdev(dev);
	if (err) {
		free_irq(dev->irq, dev);
		return err;
	}

	netdev_info(dev, "%s at 0x%08x, Ethernet Address %pM\n",
		    name, (unsigned int) ioaddr, dev->dev_addr);

	return 0;
}

static int maxi030_8390_init_one(struct platform_device *pdev)
{
	struct net_device *dev;
	struct resource *mem_res, *irq_res;
	int err;

	dev = ____alloc_ei_netdev(0);
	if (!dev)
		return -ENOMEM;

	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem_res) {
		printk("couldn't find MEM resource");
		return -ENXIO;
	}
#if 0
	if (!request_mem_region(mem_res->start, NE_IO_EXTENT, DRV_NAME)) {
		free_netdev(dev);
		netdev_err(dev, "request_mem_region failed");
		return -EBUSY;
	}
#endif

	irq_res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq_res) {
		printk("couldn't find IRQ resource");
		return -ENXIO;
	}

	err = maxi030_8390_init(dev, "MAXI030 RTL8019AS", (void __iomem *)(mem_res->start), irq_res->start);

	if (err) {
#if 0
		release_mem_region(mem_res->start, NE_IO_EXTENT);
#endif
		free_netdev(dev);
		return err;
	}
	platform_set_drvdata(pdev, dev);
	printk("dev set in pdev");
	return 0;
}

static struct platform_driver maxi030_8390_driver = {
	.probe		= maxi030_8390_init_one,
	.remove		= maxi030_8390_remove_one,
	.driver = {
		.name	= "maxi030_8390",
	},
};

static int __init maxi030_8390_init_module(void)
{
	return platform_driver_register(&maxi030_8390_driver);
}

static void __exit maxi030_8390_cleanup_module(void)
{
	platform_driver_unregister(&maxi030_8390_driver);
}

module_init(maxi030_8390_init_module);
module_exit(maxi030_8390_cleanup_module);

MODULE_LICENSE("GPL");

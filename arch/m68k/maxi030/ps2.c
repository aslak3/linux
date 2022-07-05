// SPDX-License-Identifier: GPL-2.0-only
/*
 * PS/2 controller interface for MAXI030
 *
 * Copyright (C) Lawrence Manning 2022
 *
 * Based on:
 *
 * Altera University Program PS2 controller driver
 *
 * Copyright (C) 2008 Thomas Chou <thomas@wytron.com.tw>
 *
 * Based on sa1111ps2.c, which is:
 * Copyright (C) 2002 Russell King
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/serio.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/of.h>

#define DRV_NAME "maxi030core_ps2serio"
#define NR_PORTS 2

#define REG_STATUS 0
#define REG_CONTROL 0
#define REG_TX_DATA 1
#define REG_RX_DATA REG_TX_DATA

#define CONTROL_INT_ENABLED 0x1

#define STATUS_RX_DATA_READY 0x1
#define STATUS_RX_PARITY_ERROR 0x2
#define STATUS_TX_DATA_EMPTY 0x4

#define STAT_TIMEOUT 2048//128

struct maxi030_ps2_port {
	struct serio *io;
	void __iomem *base;
	int is_open;
};

struct maxi030_ps2 {
	struct maxi030_ps2_port ports[NR_PORTS];
};

/*
 * Read all bytes waiting in the PS2 port.  There should be
 * at the most one, but we loop for safety.
 */
static irqreturn_t maxi030_ps2_rxint(int irq, void *dev_id)
{
	struct maxi030_ps2 *maxi030_ps2 = dev_id;
	unsigned char data, status;
	irqreturn_t handled = IRQ_HANDLED;//IRQ_NONE;
	int c;
	unsigned int flag = 0;
	
	for (c = 0; c < NR_PORTS; c++) {
		if (maxi030_ps2->ports[c].is_open) {
			status = readb(maxi030_ps2->ports[c].base + REG_STATUS);
			if (status & STATUS_RX_DATA_READY) {
				data = readb(maxi030_ps2->ports[c].base + REG_RX_DATA);
				serio_interrupt(maxi030_ps2->ports[c].io, data, flag);
				handled = IRQ_HANDLED;
			}
		}
	}
	
	return handled;
}

/*
 * Write a byte to the PS2 port.
 */
static int maxi030_ps2_write(struct serio *io, unsigned char val)
{
	struct maxi030_ps2_port *port = io->port_data;
	int timeout = STAT_TIMEOUT;
	unsigned char status;

	do {
		status = readb(port->base + REG_STATUS);
		cpu_relax();

		if (status & STATUS_TX_DATA_EMPTY) {
			writeb(val, port->base + REG_TX_DATA);
			return 0;
		}

	} while (--timeout);

	dev_err(&io->dev, "write timeout");
	return -ETIMEDOUT;
}

static int maxi030_ps2_open(struct serio *io)
{
	struct maxi030_ps2_port *port = io->port_data;
	unsigned char data;

	data = readb(port->base + REG_RX_DATA);

	writeb(CONTROL_INT_ENABLED, port->base + REG_CONTROL);

	port->is_open = 1;
	
	printk("PS/2 port open");

	return 0;
}

static void maxi030_ps2_close(struct serio *io)
{
	struct maxi030_ps2_port *port = io->port_data;

        writeb(0, port->base + REG_CONTROL);

	port->is_open = 0;

	printk("PS/2 port closed");
}

/*
 * Add one device to this driver.
 */
static int maxi030_ps2_probe(struct platform_device *pdev)
{
	struct maxi030_ps2 *maxi030_ps2;
	struct resource *res, *irq_res;
	struct serio *serios[NR_PORTS];
	int c, error;

	maxi030_ps2 = devm_kzalloc(&pdev->dev, sizeof(struct maxi030_ps2), GFP_KERNEL);
	if (!maxi030_ps2)
		return -ENOMEM;

	irq_res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq_res) {
		dev_err(&pdev->dev, "could not find IRQ resource");
		return -ENXIO;
	}

	error = request_irq(irq_res->start, maxi030_ps2_rxint, IRQF_TRIGGER_FALLING, dev_name(&pdev->dev),
		maxi030_ps2);
	if (error) {
		dev_err(&pdev->dev, "could not request IRQ %d", irq_res->start);
		return error;
	}
	dev_info(&pdev->dev, "IRQ %d", irq_res->start);
		
	for (c = 0; c < NR_PORTS; c++) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, c);
		if (!res) {
			dev_err(&pdev->dev, "Could not find memory resource for port %d", c);
			return -ENXIO;
		}
		maxi030_ps2->ports[c].base = (void __iomem *) res->start;

		serios[c] = kzalloc(sizeof(struct serio), GFP_KERNEL);
		if (!serios[c])
			return -ENOMEM;
      
		serios[c]->id.type			= SERIO_8042;
		serios[c]->write			= maxi030_ps2_write;
		serios[c]->open				= maxi030_ps2_open;
		serios[c]->close			= maxi030_ps2_close;
		strlcpy(serios[c]->name, dev_name(&pdev->dev), sizeof(serios[c]->name));
		strlcpy(serios[c]->phys, dev_name(&pdev->dev), sizeof(serios[c]->phys));
		serios[c]->port_data		= &maxi030_ps2->ports[c];
		serios[c]->dev.parent		= &pdev->dev;
		maxi030_ps2->ports[c].io	= serios[c];
		maxi030_ps2->ports[c].is_open	= 0;

		dev_info(&pdev->dev, "Port number %d base %08x", c, maxi030_ps2->ports[c].base);

		serio_register_port(serios[c]);
	}
	platform_set_drvdata(pdev, maxi030_ps2);
	
	printk("ps2 ports have been probed");

	return 0;
}

/*
 * Remove one device from this driver.
 */
static int maxi030_ps2_remove(struct platform_device *pdev)
{
	struct maxi030_ps2 *maxi030_ps2 = platform_get_drvdata(pdev);
	int c;

	for (c = 0; c < NR_PORTS; c++)
		serio_unregister_port(maxi030_ps2->ports[c].io);

	return 0;
}

/*
 * Our device driver structure
 */
static struct platform_driver maxi030_ps2_driver = {
	.probe		= maxi030_ps2_probe,
	.remove		= maxi030_ps2_remove,
	.driver	= {
		.name	= "maxi030core-ps2serio",
	},
};
module_platform_driver(maxi030_ps2_driver);

MODULE_DESCRIPTION("MAXI030 PS/2 controller driver");
MODULE_AUTHOR("Lawrence Manning <lawrence@aslak.net>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);

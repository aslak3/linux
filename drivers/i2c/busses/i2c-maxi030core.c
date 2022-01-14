// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  MAXI030 Core I2C Master
 *
 *  Copyright (C) 2021 Lawrence Manning <lawrence@aslak.net>
 *
 *  Based on:
 *
 *  Wondermedia I2C Master Mode Driver
 *
 *  Copyright (C) 2012 Tony Prisk <linux@prisktech.co.nz>
 *
 *  Derived from GPLv2+ licensed source:
 *  - Copyright (C) 2008 WonderMedia Technologies, Inc.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>

/* Register ofsets */
#define REG_ADDRESS		0x00
#define REG_WRITE_DATA		0x01
#define REG_READ_DATA		0x02
#define REG_CONTROL		0x03
#define REG_STATUS		REG_CONTROL

/* STATUS and CONTROL Bit fields */
#define STATUS_ACK_ERROR	0x02
#define STATUS_BUSY		0x01
#define CONTROL_LAST_BYTE	0x01

/* R/W */
#define ADDRESS_READ		0x80
#define ADDRESS_WRITE		0x00

#define WMT_I2C_TIMEOUT		(msecs_to_jiffies(1000))

struct maxi030core_i2c_dev {
	struct i2c_adapter	adapter;
	struct completion	complete;
	struct device		*dev;
	void __iomem		*base;
#if 0
	struct clk		*clk;
#endif
	int			mode;
#if 0
	int			irq;
#endif
	u16			cmd_status;
};

static int maxi030core_i2c_wait_bus_not_busy(struct maxi030core_i2c_dev *i2c_dev)
{
	unsigned long timeout;

	timeout = jiffies + WMT_I2C_TIMEOUT;
	while (readb(i2c_dev->base + REG_STATUS) & STATUS_BUSY) {
		if (time_after(jiffies, timeout)) {
			dev_warn(i2c_dev->dev, "timeout waiting for bus ready\n");
			return -EBUSY;
		}
		msleep(20);
	}

	if (readb(i2c_dev->base + REG_STATUS) & STATUS_ACK_ERROR) {
		return -EIO;
	}

	return 0;
}

#if 0
static int maxi030core_check_status(struct maxi030core_i2c_dev *i2c_dev)
{
	int ret = 0;

	if (readb(i2c_dev->base + REG_STATUS) & STATUS_BUSY)
		ret = -EBUSY;
	if (readb(i2c_dev->base + REG_STATUS) & STATUS_ACK_ERROR)
		ret = -EIO;

	return ret;
}
#endif

static int maxi030core_i2c_write(struct i2c_adapter *adap, struct i2c_msg *pmsg,
			 int last)
{
	struct maxi030core_i2c_dev *i2c_dev = i2c_get_adapdata(adap);
	int ret;
	int xfer_len = 0;

	/* Clear last byte from previous operation. */
	writeb(0, i2c_dev->base + REG_CONTROL);
	writeb(ADDRESS_WRITE | (pmsg->addr & 0x7f), i2c_dev->base + REG_ADDRESS);
	ret = maxi030core_i2c_wait_bus_not_busy(i2c_dev);
	if (ret)
		return ret;

	reinit_completion(&i2c_dev->complete);

	while (xfer_len < pmsg->len) {
		if (xfer_len == (pmsg->len - 1))
			writeb(CONTROL_LAST_BYTE, i2c_dev->base + REG_CONTROL);

		writeb(pmsg->buf[xfer_len], i2c_dev->base + REG_WRITE_DATA);
		ret = maxi030core_i2c_wait_bus_not_busy(i2c_dev);
		if (ret)
			return ret;

		xfer_len++;
	}

	return 0;
}

static int maxi030core_i2c_read(struct i2c_adapter *adap, struct i2c_msg *pmsg,
			int last)
{
	struct maxi030core_i2c_dev *i2c_dev = i2c_get_adapdata(adap);
	int ret;
	int xfer_len = 0;

	/* Clear last byte from previous operation. */
	writeb(0, i2c_dev->base + REG_CONTROL);
	writeb(ADDRESS_READ | (pmsg->addr & 0x7f), i2c_dev->base + REG_ADDRESS);
	ret = maxi030core_i2c_wait_bus_not_busy(i2c_dev);
	if (ret)
		return ret;

	reinit_completion(&i2c_dev->complete);

	while (xfer_len < pmsg->len) {
		if (xfer_len == (pmsg->len - 1))
			writeb(CONTROL_LAST_BYTE, i2c_dev->base + REG_CONTROL);

		/* Trigger read. */
		writeb(0, i2c_dev->base + REG_READ_DATA);
		ret = maxi030core_i2c_wait_bus_not_busy(i2c_dev);
		if (ret)
			return ret;
		pmsg->buf[xfer_len] = readb(i2c_dev->base + REG_READ_DATA);

		xfer_len++;
	}

	return 0;
}

static int maxi030core_i2c_xfer(struct i2c_adapter *adap,
			struct i2c_msg msgs[],
			int num)
{
	struct i2c_msg *pmsg;
	int i, is_last;
	int ret = 0;

	for (i = 0; ret >= 0 && i < num; i++) {
		is_last = ((i + 1) == num);

		pmsg = &msgs[i];
		if (pmsg->flags & I2C_M_RD)
			ret = maxi030core_i2c_read(adap, pmsg, is_last);
		else
			ret = maxi030core_i2c_write(adap, pmsg, is_last);
	}

	return (ret < 0) ? ret : i;
}

static u32 maxi030core_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm maxi030core_i2c_algo = {
	.master_xfer	= maxi030core_i2c_xfer,
	.functionality	= maxi030core_i2c_func,
};

#if 0
static irqreturn_t maxi030core_i2c_isr(int irq, void *data)
{
	struct maxi030core_i2c_dev *i2c_dev = data;

	/* save the status and write-clear it */
	i2c_dev->cmd_status = readw(i2c_dev->base + REG_ISR);
	writew(i2c_dev->cmd_status, i2c_dev->base + REG_ISR);

	complete(&i2c_dev->complete);

	return IRQ_HANDLED;
}
#endif

static int maxi030core_i2c_probe(struct platform_device *pdev)
{
#if 0
	struct device_node *np = pdev->dev.of_node;
#endif
	struct maxi030core_i2c_dev *i2c_dev;
	struct i2c_adapter *adap;
	struct resource *res;
	int err;
#if 0
	u32 clk_rate;
#endif
	i2c_dev = devm_kzalloc(&pdev->dev, sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	i2c_dev->base = (void __iomem *) res->start;
	if (IS_ERR(i2c_dev->base))
		return PTR_ERR(i2c_dev->base);

#if 0
	i2c_dev->irq = irq_of_parse_and_map(np, 0);
	if (!i2c_dev->irq) {
		dev_err(&pdev->dev, "irq missing or invalid\n");
		return -EINVAL;
	}
#endif
#if 0
	i2c_dev->clk = of_clk_get(np, 0);
	if (IS_ERR(i2c_dev->clk)) {
		dev_err(&pdev->dev, "unable to request clock\n");
		return PTR_ERR(i2c_dev->clk);
	}
	i2c_dev->mode = I2C_MODE_STANDARD;
	err = of_property_read_u32(np, "clock-frequency", &clk_rate);
	if (!err && (clk_rate == I2C_MAX_FAST_MODE_FREQ))
		i2c_dev->mode = I2C_MODE_FAST;
#endif

	i2c_dev->dev = &pdev->dev;
#if 0
	err = devm_request_irq(&pdev->dev, i2c_dev->irq, maxi030core_i2c_isr, 0,
							"i2c", i2c_dev);
	if (err) {
		dev_err(&pdev->dev, "failed to request irq %i\n", i2c_dev->irq);
		return err;
	}
#endif
	adap = &i2c_dev->adapter;
	i2c_set_adapdata(adap, i2c_dev);
	strlcpy(adap->name, "MAXI030 Core I2C adapter", sizeof(adap->name));
	adap->owner = THIS_MODULE;
	adap->algo = &maxi030core_i2c_algo;
	adap->dev.parent = &pdev->dev;
	adap->dev.of_node = pdev->dev.of_node;

	init_completion(&i2c_dev->complete);

	adap->nr = 1;
	err = i2c_add_numbered_adapter(adap);
	if (err)
		return err;

	platform_set_drvdata(pdev, i2c_dev);

	return 0;
}

static int maxi030core_i2c_remove(struct platform_device *pdev)
{
	struct maxi030core_i2c_dev *i2c_dev = platform_get_drvdata(pdev);

	i2c_del_adapter(&i2c_dev->adapter);

	return 0;
}

static struct platform_driver maxi030core_i2c_driver = {
	.probe		= maxi030core_i2c_probe,
	.remove		= maxi030core_i2c_remove,
	.driver		= {
		.name	= "maxi030core-i2c",
	},
};

module_platform_driver(maxi030core_i2c_driver);

MODULE_DESCRIPTION("MAXI030 Core I2C master-mode bus adapter");
MODULE_AUTHOR("Lawrence Manning <lawrence@aslak.net>");
MODULE_LICENSE("GPL");

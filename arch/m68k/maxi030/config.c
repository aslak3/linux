/*
 *  arch/m68k/maxi030/config.c
 *
 *  Copyright (C) 2015 William R Sowerbutts
 *  Copyright (C) 2021 Lawrence Manning
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file README.legal in the main directory of this archive
 * for more details.
 */

#include <linux/i2c.h>
#include <linux/console.h>
#include <linux/platform_device.h>
#include <linux/platform_data/serial-sccnxp.h>
#include <linux/ata_platform.h>
#include <linux/rtc.h>
#include <asm/bootinfo.h>
#include <asm/machdep.h>
#include <asm/maxi030hw.h>
#include <asm/hwtest.h>

extern void __init maxi030_init_IRQ(void);
extern irqreturn_t timer_handler(int irq, void *dev_id);

// QUART

static struct resource sc2692_resources[] = {
	DEFINE_RES_MEM(0x44010000, 0x10),
	DEFINE_RES_IRQ(IRQ_AUTO_2),
};

static struct sccnxp_pdata sc2692_info = {
	.reg_shift      = 0,
	.mctrl_cfg[0]   = MCTRL_SIG(DIR_OP, LINE_OP0),
	.mctrl_cfg[1]   = MCTRL_SIG(DIR_OP, LINE_OP1),
};
 
static struct platform_device sc2692 = {
	.name           = "sc2692",
	.resource       = sc2692_resources,
	.num_resources  = ARRAY_SIZE(sc2692_resources),
	.dev = {
		.platform_data  = &sc2692_info,
	},
};

// IDE

static struct resource maxi030ide_rsrc[] = {
	DEFINE_RES_MEM(0x44020000, 0x38),	/* CS1 */
	DEFINE_RES_MEM(0x44030000, 0x38),	/* CS3 */
//	DEFINE_RES_IRQ(IRQ_AUTO_3),		/* TODO: Inoperable at present. */
};

static struct pata_platform_info maxi030ide_info = {
	.ioport_shift = 2,
};

static struct platform_device maxi030ide = {
	.name           = "pata_platform",
	.resource       = maxi030ide_rsrc,
	.num_resources  = ARRAY_SIZE(maxi030ide_rsrc),
	.dev = {
		.platform_data = &maxi030ide_info,
	},
};

// NIC

static struct resource maxi030rtl8019_rsrc[] = {
	DEFINE_RES_MEM(0x44040000, 0x40),
	DEFINE_RES_IRQ(IRQ_AUTO_4),
};

static struct platform_device maxi030rtl8019 = {
	.name           = "maxi030_8390",
	.resource       = maxi030rtl8019_rsrc,
	.num_resources  = ARRAY_SIZE(maxi030rtl8019_rsrc),
};

// I2C

static struct resource maxi030corei2c_rsrc[] = {
	DEFINE_RES_MEM(0x44000007, 4),
};

static struct platform_device maxi030corei2c = {
	.name           = "maxi030core-i2c",
	.resource       = maxi030corei2c_rsrc,
	.num_resources  = ARRAY_SIZE(maxi030corei2c_rsrc),
};

// RTC and TEMP SENSOR

static struct i2c_board_info maxi030_i2c_info[] = {
	{
		I2C_BOARD_INFO("ds1307", 0x68),
	},
	{
		I2C_BOARD_INFO("lm75a", 0x4f),
	},
};

// PS2 SERIO

static struct resource maxi030coreps2serio_rsrc[] = {
	DEFINE_RES_MEM(0x44000003, 0x2),
	DEFINE_RES_MEM(0x44000005, 0x2),
	DEFINE_RES_IRQ(IRQ_AUTO_5),
};

static struct platform_device maxi030coreps2serio = {
	.name           = "maxi030core-ps2serio",
	.resource       = maxi030coreps2serio_rsrc,
	.num_resources  = ARRAY_SIZE(maxi030coreps2serio_rsrc),
};

static void maxi030_get_model(char *model)
{
	sprintf(model, "MAXI030");
}

int __init maxi030_parse_bootinfo(const struct bi_record *rec)
{
	return 1;
}

void maxi030_sched_init(void)
{
	if (request_irq(IRQ_AUTO_1, timer_handler, 0, "timer", NULL))
		panic("Couldn't register timer int");

	/* 100HZ timer in FPGA with clock @ 40MHz */
//	outb(6, MAXI030_CORE_TIMERCOUNTU);
//	outb(26, MAXI030_CORE_TIMERCOUNTM);
//	outb(126, MAXI030_CORE_TIMERCOUNTL);
	/* 100Hz timer in FPGA with clock @ 20MHz */
	outb(0x03, MAXI030_CORE_TIMERCOUNTU);
	outb(0x0d, MAXI030_CORE_TIMERCOUNTM);
	outb(0x40, MAXI030_CORE_TIMERCOUNTL);
	
	outb(0x01, MAXI030_CORE_TIMERCONTROL);
}

void __init config_maxi030(void)
{
	mach_init_IRQ = maxi030_init_IRQ;
	mach_sched_init = maxi030_sched_init;
	mach_get_model = maxi030_get_model;
//	mach_max_dma_address = 0x10000000; /* ?? */
}

int __init maxi030_platform_init(void)
{
	if (platform_device_register(&sc2692))
		panic("Unable to register UART device");
	if (platform_device_register(&maxi030ide))
		panic("Unable to register IDE device");
 	if (platform_device_register(&maxi030rtl8019))
		panic("Unable to register RTL8019 device");
	if (platform_device_register(&maxi030corei2c))
		panic("Unable to register MAXI030 Core I2C device");

	if (i2c_register_board_info(1, maxi030_i2c_info,
		ARRAY_SIZE(maxi030_i2c_info)))
	{
		panic("Unable to add I2C devices");
	}
	if (platform_device_register(&maxi030coreps2serio))
		panic("Unable to register MAXI030 PS2 serio device");

	return 0;
}

arch_initcall(maxi030_platform_init);

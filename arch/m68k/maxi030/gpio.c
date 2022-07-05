// SPDX-License-Identifier: GPL-2.0
/*
 * GPIO support for the LED on MAXI030
 * 
 * Copyright (C) 2022 Lawrence Manning
 *
 * Based on:
 *
 * arch/sh/boards/mach-x3proto/gpio.c
 *
 * Renesas SH-X3 Prototype Baseboard GPIO Support.
 *
 * Copyright (C) 2010 - 2012  Paul Mundt
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/gpio/driver.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/irqdomain.h>
#include <linux/io.h>

#define LED 0x44000000

static void maxi030core_gpio_set(struct gpio_chip *chip, unsigned int gpio,
	int value)
{
	outb(value & 1, LED);
}

struct gpio_chip maxi030core_gpio_chip = {
	.label			= "maxi030core-gpio",
	.set			= maxi030core_gpio_set,
	.base			= -1,
	.ngpio			= 1,
};

int __init maxi030core_gpio_setup(void)
{
	int ret;

	ret = gpiochip_add_data(&maxi030core_gpio_chip, NULL);
	if (unlikely(ret))
		return ret;

	pr_info("registering '%s' support, handling GPIOs %u -> %u",
		maxi030core_gpio_chip.label, maxi030core_gpio_chip.base,
		maxi030core_gpio_chip.base + maxi030core_gpio_chip.ngpio);;

	return 0;
}

core_initcall(maxi030core_gpio_setup);

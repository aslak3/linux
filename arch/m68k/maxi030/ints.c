// SPDX-License-Identifier: GPL-2.0
/*
 * Interrupt related bits for MAXI030.
 *
 * Copyright (C) 2022 Lawrence Manning */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <asm/traps.h>
#include <asm/machdep.h>
#include <asm/maxi030hw.h>

unsigned int maxi030core_irq_startup(struct irq_data *data);
void maxi030core_irq_shutdown(struct irq_data *data);
void maxi030core_irq_unmask(struct irq_data *data);
void maxi030core_irq_mask(struct irq_data *data);

irqreturn_t timer_handler(int irq, void *dev_id)
{
	outb(0x01, MAXI030_CORE_TIMERCONTROL);

	legacy_timer_tick(1);
	timer_heartbeat();
	
	return IRQ_HANDLED;
}

unsigned int maxi030core_irq_startup(struct irq_data *data)
{
	maxi030core_irq_unmask(data);
	return m68k_irq_startup(data);
}

void maxi030core_irq_shutdown(struct irq_data *data)
{
	maxi030core_irq_mask(data);
	m68k_irq_shutdown(data);
}

void maxi030core_irq_unmask(struct irq_data *data)
{
	uint8_t pass;

	pass = inb(MAXI030_CORE_INT_PASS);
	pass |= 1 << (data->irq - IRQ_AUTO_1);
	outb(pass, MAXI030_CORE_INT_PASS);
}

void maxi030core_irq_mask(struct irq_data *data)
{
	uint8_t pass;

	pass = inb(MAXI030_CORE_INT_PASS);
	pass &= ~(1 << (data->irq - IRQ_AUTO_1));
	outb(pass, MAXI030_CORE_INT_PASS);
}


void maxi030core_irq_eoi(struct irq_data *data)
{
}


/* MAXI030's FPGA has the name "core". A register is used to allow each int
 * through to the processor. 8 bit, 8 ints: 0=timer, 1=uart, 2=ide, 3=nic, 4=ps2 */
static struct irq_chip maxi030core_irq_chip = {
	.name 		= "maxi030core",
	.irq_startup	= maxi030core_irq_startup,
	.irq_shutdown	= maxi030core_irq_shutdown,
	.irq_mask	= maxi030core_irq_mask,
	.irq_unmask	= maxi030core_irq_unmask,
	.irq_eoi	= maxi030core_irq_eoi,
};

void __init maxi030_init_IRQ(void)
{
	/* Only the autovectored ints are used presently. */
	m68k_setup_irq_controller(&maxi030core_irq_chip, handle_fasteoi_irq, IRQ_AUTO_1, 7);
}

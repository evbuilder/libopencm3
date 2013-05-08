/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Thomas Otto <tommi@viadmin.org>
 * Copyright (C) 2010 Piotr Esden-Tempski <piotr@esden.net>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/flash.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/can.h>

struct can_tx_msg {
	u32 std_id;
	u32 ext_id;
	u8 ide;
	u8 rtr;
	u8 dlc;
	u8 data[8];
};

struct can_rx_msg {
	u32 std_id;
	u32 ext_id;
	u8 ide;
	u8 rtr;
	u8 dlc;
	u8 data[8];
	u8 fmi;
};

struct can_tx_msg can_tx_msg;
struct can_rx_msg can_rx_msg;

static void gpio_setup(void)
{
	/* Enable GPIO clocks */
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPCEN);
//	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);

	gpio_set(GPIOA, GPIO6); /* LED0 off */
	gpio_set(GPIOA, GPIO7); /* LED1 off */
//	gpio_set(GPIOB, GPIO0); /* LED2 off */
//	gpio_set(GPIOB, GPIO1); /* LED3 off */

	/* Set GPIO6/7 (in GPIO port C) to 'output push-pull' for the LEDs. */
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO6);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO7);
}

static void systick_setup(void)
{
	/* 72MHz / 8 => 9000000 counts per second */
	systick_set_clocksource(STK_CTRL_CLKSOURCE_AHB_DIV8);

	/* 9000000/9000 = 1000 overflows per second - every 1ms one interrupt */
	/* SysTick interrupt every N clock pulses: set reload to N-1 */
	systick_set_reload(8999);

	systick_interrupt_enable();

	/* Start counting. */
	systick_counter_enable();
}

static void can_setup(void)
{
	/* Enable peripheral clocks. */
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_AFIOEN);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_CANEN);

	/* Configure remapped CAN pins */
	gpio_primary_remap( 0 , AFIO_MAPR_CAN1_REMAP_PORTD);
	
	/* Configure CAN pin: RX (input pull-up). */
	gpio_set_mode(GPIOD, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_PULL_UPDOWN, GPIO0);
	gpio_set(GPIOD, GPIO0);

	/* Configure CAN pin: TX. */
	gpio_set_mode(GPIOD, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO1);
	
	/* NVIC setup. */
	nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
	nvic_set_priority(NVIC_USB_LP_CAN_RX0_IRQ, 1);

	/* Reset CAN. */
	can_reset(CAN1);

	/* CAN cell init. */
	if (can_init(CAN1,
		     false,           /* TTCM: Time triggered comm mode? */
		     true,            /* ABOM: Automatic bus-off management? */
		     false,           /* AWUM: Automatic wakeup mode? */
		     false,           /* NART: No automatic retransmission? */
		     false,           /* RFLM: Receive FIFO locked mode? */
		     false,           /* TXFP: Transmit FIFO priority? */
//		     CAN_BTR_SJW_1TQ, // 0         ts1 -- Baud = APB1clk/( (BRP + 1) * ( 1 + (ts1+1) + (ts2+1)) )
//		     CAN_BTR_TS1_3TQ, // 0x2 << 24 ts2 -- 36MHz / ( (12 + 1) * ( 1 + 3 + 4 ) = 346kHz
//		     CAN_BTR_TS2_4TQ, // 0x3 << 24 We want 125kbps... same as lata switches = 36/.125 = 288 = 18 * 16 =
//		     12,	      // (brp)     (17 + 1) * (1 + 7 + 8)
		     CAN_BTR_SJW_3TQ, CAN_BTR_TS1_7TQ, CAN_BTR_TS2_8TQ, 17,
		     false,
		     false))             /* BRP+1: Baud rate prescaler */
	{
		gpio_set(GPIOC, GPIO6);		/* LED0 on */
		gpio_clear(GPIOC, GPIO7);	/* LED1 off */

		/* Die because we failed to initialize. */
		while (1)
			__asm__("nop");
	}

	/* CAN filter 0 init. */
	can_filter_id_mask_32bit_init(CAN1,
				0,     /* Filter ID */
				0,     /* CAN ID */
				0,     /* CAN ID mask */
				0,     /* FIFO assignment (here: FIFO0) */
				true); /* Enable the filter. */

	/* Enable CAN RX interrupt. */
	can_enable_irq(CAN1, CAN_IER_FMPIE0);
}

void sys_tick_handler(void)
{
	static int temp32 = 0;
	static u8 data[8] = {0, 1, 2, 0, 0, 0, 0, 0};

	/* We call this handler every 1ms so 1000ms = 1s on/off. */
	if (++temp32 != 1000)
		return;

	temp32 = 0;

	/* Transmit CAN frame. */
	data[0]++;
	if (can_transmit(CAN1,
			 0,     /* (EX/ST)ID: CAN ID */
			 false, /* IDE: CAN ID extended? */
			 false, /* RTR: Request transmit? */
			 8,     /* DLC: Data length */
			 data) == -1)
	{
		gpio_clear(GPIOC, GPIO6);	/* LED0 off */
		gpio_set(GPIOC, GPIO7);		/* LED1 on */
	}
}

void usb_lp_can_rx0_isr(void)
{
	u32 id, fmi;
	bool ext, rtr;
	u8 length, data[8];

	can_receive(CAN1, 0, false, &id, &ext, &rtr, &fmi, &length, data);

	if (data[0] & 1)
		gpio_set(GPIOC, GPIO6);
	else
		gpio_clear(GPIOC, GPIO6);

	if (data[0] & 2)
		gpio_set(GPIOC, GPIO7);
	else
		gpio_clear(GPIOC, GPIO7);

	can_fifo_release(CAN1, 0);
}

int main(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();
	gpio_setup();
	can_setup();
	systick_setup();

	while (1); /* Halt. */

	return 0;
}

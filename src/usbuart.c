/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2012  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scs.h>

#include "general.h"
#include "cdcacm.h"
#include "usbuart.h"

#define USBUART_TIMER_FREQ_HZ 1000000U /* 1us per tick */
#define USBUART_RUN_FREQ_HZ 5000U /* 200us (or 100 characters at 2Mbps) */

#define FIFO_SIZE 128

/* RX Fifo buffer */
static uint8_t buf_rx1[FIFO_SIZE];
static uint8_t buf_rx2[FIFO_SIZE];
static uint8_t buf_rx3[FIFO_SIZE];
/* Fifo in pointer, writes assumed to be atomic, should be only incremented within RX ISR */
static uint8_t buf_rx1_in;
static uint8_t buf_rx2_in;
static uint8_t buf_rx3_in;
/* Fifo out pointer, writes assumed to be atomic, should be only incremented outside RX ISR */
static uint8_t buf_rx1_out;
static uint8_t buf_rx2_out;
static uint8_t buf_rx3_out;

static void usbuart_run(int USBUSART_TIM, uint8_t *buf_rx_out, uint8_t *buf_rx_in, uint8_t *buf_rx, int CDCACM_UART_ENDPOINT);

void usbuart_init(void)
{
	rcc_periph_clock_enable(RCC_USART1);
	rcc_periph_clock_enable(RCC_USART2);
	rcc_periph_clock_enable(RCC_USART3);

	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO9);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO2);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO10);

	/* Setup UART parameters. */
	usart_set_baudrate(USART1, 38400);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	usart_set_baudrate(USART2, 38400);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

	usart_set_baudrate(USART3, 38400);
	usart_set_databits(USART3, 8);
	usart_set_stopbits(USART3, USART_STOPBITS_1);
	usart_set_mode(USART3, USART_MODE_TX_RX);
	usart_set_parity(USART3, USART_PARITY_NONE);
	usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);

	/* Finally enable the USARTs. */
	usart_enable(USART1);
	usart_enable(USART2);
	usart_enable(USART3);

	/* Enable interrupts */
	USART1_CR1 |= USART_CR1_RXNEIE;
	nvic_set_priority(NVIC_USART1_IRQ, IRQ_PRI_USBUSART);
	nvic_enable_irq(NVIC_USART1_IRQ);

	USART2_CR1 |= USART_CR1_RXNEIE;
	nvic_set_priority(NVIC_USART2_IRQ, IRQ_PRI_USBUSART);
	nvic_enable_irq(NVIC_USART2_IRQ);

	USART3_CR1 |= USART_CR1_RXNEIE;
	nvic_set_priority(NVIC_USART3_IRQ, IRQ_PRI_USBUSART);
	nvic_enable_irq(NVIC_USART3_IRQ);

	/* Setup timer for running deferred FIFO processing */
	rcc_periph_clock_enable(RCC_TIM2);
	rcc_periph_clock_enable(RCC_TIM3);
	rcc_periph_clock_enable(RCC_TIM4);
	timer_reset(TIM2);
	timer_reset(TIM3);
	timer_reset(TIM4);
	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_mode(TIM4, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

	timer_set_prescaler(TIM2,
			rcc_apb2_frequency / USBUART_TIMER_FREQ_HZ * 2 - 1);
	timer_set_prescaler(TIM3,
			rcc_apb2_frequency / USBUART_TIMER_FREQ_HZ * 2 - 1);
	timer_set_prescaler(TIM4,
			rcc_apb2_frequency / USBUART_TIMER_FREQ_HZ * 2 - 1);

	timer_set_period(TIM2,
			USBUART_TIMER_FREQ_HZ / USBUART_RUN_FREQ_HZ - 1);
	timer_set_period(TIM3,
			USBUART_TIMER_FREQ_HZ / USBUART_RUN_FREQ_HZ - 1);
	timer_set_period(TIM4,
			USBUART_TIMER_FREQ_HZ / USBUART_RUN_FREQ_HZ - 1);

	/* Setup update interrupt in NVIC */
	nvic_set_priority(NVIC_TIM2_IRQ, IRQ_PRI_USBUSART_TIM);
	nvic_set_priority(NVIC_TIM3_IRQ, IRQ_PRI_USBUSART_TIM);
	nvic_set_priority(NVIC_TIM4_IRQ, IRQ_PRI_USBUSART_TIM);
	nvic_enable_irq(NVIC_TIM2_IRQ);
	nvic_enable_irq(NVIC_TIM3_IRQ);
	nvic_enable_irq(NVIC_TIM4_IRQ);

	/* turn the timer on */
	timer_enable_counter(TIM2);
	timer_enable_counter(TIM3);
	timer_enable_counter(TIM4);
}

/*
 * Runs deferred processing for usb uart rx, draining RX FIFO by sending
 * characters to host PC via CDCACM.  Allowed to read from FIFO in pointer,
 * but not write to it. Allowed to write to FIFO out pointer.
 */
static void usbuart_run(int USBUSART_TIM, uint8_t *buf_rx_out, uint8_t *buf_rx_in, uint8_t *buf_rx, int CDCACM_UART_ENDPOINT)
{
	/* forcibly empty fifo if no USB endpoint */
	if (cdcacm_get_config() != 1)
	{
		*buf_rx_out = *buf_rx_in;
	}

	/* if fifo empty, nothing further to do */
	if (*buf_rx_in == *buf_rx_out) {
		/* turn off LED, disable IRQ */
		timer_disable_irq(USBUSART_TIM, TIM_DIER_UIE);
		gpio_clear(LED_PORT_UART, LED_UART);
	}
	else
	{
		uint8_t packet_buf[CDCACM_PACKET_SIZE];
		uint8_t packet_size = 0;
		uint8_t buf_out = *buf_rx_out;

		/* copy from uart FIFO into local usb packet buffer */
		while (*buf_rx_in != buf_out && packet_size < CDCACM_PACKET_SIZE)
		{
			packet_buf[packet_size++] = buf_rx[buf_out++];

			/* wrap out pointer */
			if (buf_out >= FIFO_SIZE)
			{
				buf_out = 0;
			}

		}

		/* advance fifo out pointer by amount written */
		*buf_rx_out += usbd_ep_write_packet(usbdev,
				CDCACM_UART_ENDPOINT, packet_buf, packet_size);
		*buf_rx_out %= FIFO_SIZE;
	}
}

void usbuart_set_line_coding(struct usb_cdc_line_coding *coding, int USBUSART)
{
	usart_set_baudrate(USBUSART, coding->dwDTERate);

	if (coding->bParityType)
		usart_set_databits(USBUSART, coding->bDataBits + 1);
	else
		usart_set_databits(USBUSART, coding->bDataBits);

	switch(coding->bCharFormat) {
	case 0:
		usart_set_stopbits(USBUSART, USART_STOPBITS_1);
		break;
	case 1:
		usart_set_stopbits(USBUSART, USART_STOPBITS_1_5);
		break;
	case 2:
		usart_set_stopbits(USBUSART, USART_STOPBITS_2);
		break;
	}

	switch(coding->bParityType) {
	case 0:
		usart_set_parity(USBUSART, USART_PARITY_NONE);
		break;
	case 1:
		usart_set_parity(USBUSART, USART_PARITY_ODD);
		break;
	case 2:
		usart_set_parity(USBUSART, USART_PARITY_EVEN);
		break;
	}
}

static void usbuart_usb_out_cb(int USBUSART, usbd_device *dev, uint8_t ep, int CDCACM_UART_ENDPOINT)
{
	(void)ep;

	char buf[CDCACM_PACKET_SIZE];
	int len = usbd_ep_read_packet(dev, CDCACM_UART_ENDPOINT,
					buf, CDCACM_PACKET_SIZE);

	gpio_set(LED_PORT_UART, LED_UART);
	for(int i = 0; i < len; i++)
		usart_send_blocking(USBUSART, buf[i]);
	gpio_clear(LED_PORT_UART, LED_UART);
}

void usbuart1_usb_out_cb(usbd_device *dev, uint8_t ep)
{
    usbuart_usb_out_cb(USART1, dev, ep, 5);
}

void usbuart2_usb_out_cb(usbd_device *dev, uint8_t ep)
{
    usbuart_usb_out_cb(USART2, dev, ep, 1);
}

void usbuart3_usb_out_cb(usbd_device *dev, uint8_t ep)
{
    usbuart_usb_out_cb(USART3, dev, ep, 3);
}


#ifdef USBUART_DEBUG
int usbuart_debug_write(int tim, const char *buf, size_t len, uint8_t *buf_rx, uint8_t *buf_rx_in)
{
	for (size_t i = 0; i < len; i++) {
		if (buf[i] == '\n') {
			*buf_rx[(*buf_rx_in)++] = '\r';
			*buf_rx_in %= FIFO_SIZE;
		}
		buf_rx[(*buf_rx_in)++] = buf[i];
		*buf_rx_in %= FIFO_SIZE;
	}
	/* enable deferred processing if we put data in the FIFO */
	timer_enable_irq(tim, TIM_DIER_UIE);
	return len;
}
#endif

void usbuart_usb_in_cb(usbd_device *dev, uint8_t ep)
{
	(void) dev;
	(void) ep;
}

/*
 * Read a character from the UART RX and stuff it in a software FIFO.
 * Allowed to read from FIFO out pointer, but not write to it.
 * Allowed to write to FIFO in pointer.
 */
// USBUSART_ISR
static void usart_isr(int USBUSART, int USBUSART_TIM, uint8_t *buf_rx_out, uint8_t *buf_rx_in, uint8_t *buf_rx)
{
	uint32_t err = USART_SR(USBUSART);
	char c = usart_recv(USBUSART);
	if (err & (USART_SR_ORE | USART_SR_FE))
		return;

	/* Turn on LED */
	gpio_set(LED_PORT_UART, LED_UART);

	/* If the next increment of rx_in would put it at the same point
	* as rx_out, the FIFO is considered full.
	*/
	if (((*buf_rx_in + 1) % FIFO_SIZE) != *buf_rx_out)
	{
		/* insert into FIFO */
		buf_rx[(*buf_rx_in)++] = c;

		/* wrap out pointer */
		if (*buf_rx_in >= FIFO_SIZE)
		{
			*buf_rx_in = 0;
		}

		/* enable deferred processing if we put data in the FIFO */
		timer_enable_irq(USBUSART_TIM, TIM_DIER_UIE);
	}
}

void usart1_isr(void)
{
    usart_isr(USART1, TIM2, &buf_rx1_out, &buf_rx1_in, buf_rx1);
}

void usart2_isr(void)
{
    usart_isr(USART2, TIM3, &buf_rx2_out, &buf_rx2_in, buf_rx2);
}

void usart3_isr(void)
{
    usart_isr(USART3, TIM4, &buf_rx3_out, &buf_rx3_in, buf_rx3);
}


// USBUSART_TIM_ISR
void tim2_isr(void)
{
	/* need to clear timer update event */
	timer_clear_flag(TIM2, TIM_SR_UIF);

	/* process FIFO */
	usbuart_run(TIM2, &buf_rx1_out, &buf_rx1_in, buf_rx1, 5);
}

void tim3_isr(void)
{
	/* need to clear timer update event */
	timer_clear_flag(TIM3, TIM_SR_UIF);

	/* process FIFO */
	usbuart_run(TIM3, &buf_rx2_out, &buf_rx2_in, buf_rx2, 1);
}

void tim4_isr(void)
{
	/* need to clear timer update event */
	timer_clear_flag(TIM4, TIM_SR_UIF);

	/* process FIFO */
	usbuart_run(TIM4, &buf_rx3_out, &buf_rx3_in, buf_rx3, 3);
}

#ifdef ENABLE_DEBUG
enum {
	RDI_SYS_OPEN = 0x01,
	RDI_SYS_WRITE = 0x05,
	RDI_SYS_ISTTY = 0x09,
};

int rdi_write(int tim, int fn, const char *buf, size_t len)
{
	(void)fn;
	if (debug_bmp)
		return len - usbuart_debug_write(tim, buf, len);

	return 0;
}

struct ex_frame {
	union {
		int syscall;
		int retval;
	};
	const int *params;
	uint32_t r2, r3, r12, lr, pc;
};

void debug_monitor_handler_c(struct ex_frame *sp)
{
	/* Return to after breakpoint instruction */
	sp->pc += 2;

	switch (sp->syscall) {
	case RDI_SYS_OPEN:
		sp->retval = 1;
		break;
	case RDI_SYS_WRITE:
		sp->retval = rdi_write(sp->params[0], (void*)sp->params[1], sp->params[2]);
		break;
	case RDI_SYS_ISTTY:
		sp->retval = 1;
		break;
	default:
		sp->retval = -1;
	}

}

asm(".globl debug_monitor_handler\n"
    ".thumb_func\n"
    "debug_monitor_handler: \n"
    "    mov r0, sp\n"
    "    b debug_monitor_handler_c\n");

#endif

/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
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

/* This file implements the platform specific functions for the STM32
 * implementation.
 */
#ifndef __PLATFORM_H
#define __PLATFORM_H

#include "version.h"

#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/f1/memorymap.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/usb/usbd.h>

struct platform_timeout {
	uint32_t time;
};

uint32_t platform_time_ms(void);

extern uint8_t running_status;

void platform_timing_init(void);

void platform_init(void);

typedef struct platform_timeout platform_timeout;
void platform_timeout_set(platform_timeout *t, uint32_t ms);
bool platform_timeout_is_expired(platform_timeout *t);
void platform_delay(uint32_t ms);

const char *platform_target_voltage(void);
int platform_hwversion(void);
void platform_srst_set_val(bool assert);
bool platform_srst_get_val(void);
bool platform_target_get_power(void);
void platform_target_set_power(bool power);
void platform_request_boot(void);


#ifdef ENABLE_DEBUG
# define PLATFORM_HAS_DEBUG
# define USBUART_DEBUG
#endif

#define BOARD_IDENT       "Black Magic Probe (STLINK), (Firmware " FIRMWARE_VERSION ")"
#define UPD_IFACE_STRING  "@Internal Flash   /0x08000000/8*001Kg"

#define LED_PORT	GPIOA
/* Use PC13 for a "dummy" uart led. So we can observe the LED. */
#define LED_PORT_UART	GPIOC
#define LED_UART	GPIO13

#define USB_DRIVER      st_usbfs_v1_usb_driver
#define USB_IRQ	        NVIC_USB_LP_CAN_RX0_IRQ
#define USB_ISR	        usb_lp_can_rx0_isr
/* Interrupt priorities.  Low numbers are high priority.
 * For now USART2 preempts USB which may spin while buffer is drained.
 */
#define IRQ_PRI_USB		(2 << 4)
#define IRQ_PRI_USBUSART	(1 << 4)
#define IRQ_PRI_USBUSART_TIM	(3 << 4)
#define IRQ_PRI_USB_VBUS	(14 << 4)

#ifdef ENABLE_DEBUG
extern bool debug_bmp;
int usbuart_debug_write(const char *buf, size_t len);
# define DEBUG printf
#else
# define DEBUG(...)
#endif

extern uint16_t led_idle_run;
#define LED_IDLE_RUN            led_idle_run
#define SET_RUN_STATE(state)	{running_status = (state);}
#define SET_IDLE_STATE(state)	{gpio_set_val(LED_PORT, led_idle_run, state);}
#define SET_ERROR_STATE(x)

extern uint32_t detect_rev(void);

/* Use newlib provided integer only stdio functions */
#define sscanf siscanf
#define sprintf siprintf
#define vasprintf vasiprintf
#define snprintf sniprintf

#define gpio_set_val(port, pin, val) do {	\
	if(val)					\
		gpio_set((port), (pin));	\
	else					\
		gpio_clear((port), (pin));	\
} while(0)

static inline void _gpio_set(uint32_t gpioport, uint16_t gpios)
{
	GPIO_BSRR(gpioport) = gpios;
}
#define gpio_set _gpio_set

static inline void _gpio_clear(uint32_t gpioport, uint16_t gpios)
{
	GPIO_BRR(gpioport) = gpios;
}
#define gpio_clear _gpio_clear

static inline uint16_t _gpio_get(uint32_t gpioport, uint16_t gpios)
{
	return (uint16_t)GPIO_IDR(gpioport) & gpios;
}
#define gpio_get _gpio_get

#endif


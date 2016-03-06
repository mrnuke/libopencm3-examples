/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2011 Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2012-2013 Alexandru Gagniuc <mr.nuke.me@gmail.com>
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

/**
 * \addtogroup Examples
 *
 * Flashes the Red, Green and Blue diodes on the board, in order.
 *
 * RED controlled by PF1
 * Green controlled by PF3
 * Blue controlled by PF2
 */

#include "usb_to_serial_usbtmc.h"

#include <libopencm3/lm4f/systemcontrol.h>
#include <libopencm3/lm4f/rcc.h>
#include <libopencm3/lm4f/gpio.h>
#include <libopencm3/lm4f/uart.h>
#include <libopencm3/lm4f/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PLL_DIV_80MHZ		5
/* This is how the RGB LED is connected on the stellaris launchpad */
#define RGB_PORT	GPIOF
enum {
	LED_R	= GPIO1,
	LED_G	= GPIO3,
	LED_B	= GPIO2,
};

/*
 * Clock setup:
 * Take the main crystal oscillator at 16MHz, run it through the PLL, and divide
 * the 400MHz PLL clock to get a system clock of 80MHz.
 */
static void clock_setup(void)
{
	rcc_sysclk_config(OSCSRC_MOSC, XTAL_16M, PLL_DIV_80MHZ);
}

/*
 * GPIO setup:
 * Enable the pins driving the RGB LED as outputs.
 */
static void gpio_setup(void)
{
	/*
	 * Configure GPIOF
	 * This port is used to control the RGB LED
	 */
	periph_clock_enable(RCC_GPIOF);
	const uint32_t opins = (LED_R | LED_G | LED_B);

	gpio_mode_setup(RGB_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, opins);
	gpio_set_output_config(RGB_PORT, GPIO_OTYPE_PP, GPIO_DRIVE_2MA, opins);

}

void glue_data_received_cb(uint8_t * buf, uint16_t len)
{
	/* Blue LED indicates data coming in */
	gpio_set(RGB_PORT, LED_B);
	usbtmc_send_data(buf, len);
	gpio_clear(RGB_PORT, LED_B);
}

void glue_set_line_state_cb(uint8_t dtr, uint8_t rts)
{
	/* Green LED indicated one of the control lines are active */
	if (dtr || rts)
		gpio_set(RGB_PORT, LED_G);
	else
		gpio_clear(RGB_PORT, LED_G);

	uart_set_ctl_line_state(dtr, rts);
}

static void strn_terminate(char *str, const char *delimiters, size_t len)
{
	if (len == 0)
		return;

	str[len - 1] = '\0';
	str[strcspn(str, delimiters)] = '\0';
}

static const char *strscanfwd(const char *str, char delimiter)
{
	while (*str++ != delimiter) {
		if (*str == '\0')
			return NULL;
	}

	/* Scan past identical delimiters. */
	while (*str == delimiter)
		str++;

	return str;
}

static bool scpi_get_int(long *value, const char *str)
{
	char *endptr = NULL;

	errno = 0;
	*value = strtol(str, &endptr, 10);

	if (!endptr || *endptr || errno)
		return false;

	return true;
}

static bool conf488_set_baudrate(const char *cmd)
{
	long baud;

	if (!scpi_get_int(&baud, cmd))
		return false;

	if ((baud < 150) || (baud > 1000000)) {
		printf("Invalid baudrate: %ld\n", baud);
		return false;
	}

	printf("Changing baudrate to: %ld\n", baud);
	uart_configure(baud, UART_KEEP, UART_KEEP, UART_KEEP);
	return true;
}

static bool conf488_set_databits(const char *cmd)
{
	long databits;

	if (!scpi_get_int(&databits, cmd))
		return false;

	if ((databits < 7) || (databits > 8)) {
		printf("Invalid number of data bits: %ld\n", databits);
		return false;
	}

	printf("Changing data bits to: %ld\n", databits);
	uart_configure(UART_KEEP, databits, UART_KEEP, UART_KEEP);
	return true;
}

static bool conf488_set_parity(const char *cmd)
{
	enum uart_parity parity;
	if (!strcmp(cmd, "EVEN")) {
		parity = UART_PARITY_EVEN;
	} else if (!strcmp(cmd, "ODD")) {
		parity = UART_PARITY_ODD;
	} else if (!strcmp(cmd, "NONE")) {
		parity = UART_PARITY_NONE;
	} else {
		printf("Invalid parity: %s\n", cmd);
		return false;
	}

	printf("Changing parity to: %s\n", cmd);
	uart_configure(UART_KEEP, UART_KEEP, parity, UART_KEEP);
	return true;
}

static bool conf488_set_stopbits(const char *cmd)
{
	long stopbits;

	if (!scpi_get_int(&stopbits, cmd))
		return false;

	if ((stopbits < 1) || (stopbits > 2)) {
		printf("Invalid number of data bits: %ld\n", stopbits);
		return false;
	}

	printf("Changing stop bits to: %ld\n", stopbits);
	uart_configure(UART_KEEP, UART_KEEP, UART_KEEP, stopbits);
	return true;
}

static bool conf488_print_sexy(const char *cmd)
{
	unsigned int yeehaw;
	int scanno = sscanf(cmd, "%u", &yeehaw);
	printf("%s: sexy sexy sexy: %d: %u\n", cmd, scanno, yeehaw);
	return true;
}

struct scpi_hierarcy {
	const char *subcmd;
	const struct scpi_hierarcy *sub_hierarcy;
	bool (*execute)(const char *cmd);
};

static const struct scpi_hierarcy usb488_uart_hooks[] = {
	{"BAUD", NULL, conf488_set_baudrate },
	{"DATA", NULL, conf488_set_databits },
	{"STOP", NULL, conf488_set_stopbits },
	{"PAR",  NULL, conf488_set_parity },
	{"FLOW", NULL, conf488_print_sexy },
	{NULL, NULL, NULL},
};

static const struct scpi_hierarcy usb488_gpib_hooks[] = {
	{"ADDR", NULL, NULL },
	{NULL, NULL, NULL},
};

static const struct scpi_hierarcy usb488_hooks[] = {
	{
		.subcmd = "UART",
		.sub_hierarcy = usb488_uart_hooks,
	}, {
		.subcmd = "GPIB",
		.sub_hierarcy = usb488_gpib_hooks,
	}, {
		.subcmd = NULL,
	},
};

static const struct scpi_hierarcy usb488_hierarcy[] = {
	{
		.subcmd = "USB488",
		.sub_hierarcy = usb488_hooks,
	}, {
		.subcmd = NULL,
	},
};

static bool scpi_walk_hierarcy(const struct scpi_hierarcy *root, const char *cmd)
{
	const struct scpi_hierarcy *child;

	printf("Walking hierarcy for \"%s\"\n", cmd);

	child = root;
	while (child->subcmd) {
		if (!strncmp(cmd, child->subcmd, strlen(child->subcmd))) {
			printf ("Found subcommand \"%s\"\n", child->subcmd);
			if (child->execute) {
				cmd = strscanfwd(cmd, ' ');
				printf("Found leaf \"%s\"\n", cmd);
				return child->execute(cmd);
			}

			if (!child->sub_hierarcy)
				break;

			/* Walk the branch of the child node. */
			cmd = strscanfwd(cmd, ':');
			child = child->sub_hierarcy;
			continue;
		}

		/* Advance to next child in the list */
		child++;
	}

	printf("Failed to advance hierarcy\n");
	return false;
}

void usb488_rx_cmd_cb(void *buf, uint16_t len)
{
	int i;
	const char *config_magic = usb488_hierarcy->subcmd;
	const char *charbuf = buf;

	if (len > strlen(config_magic)
	    && !strncmp(buf, config_magic, strlen(config_magic))) {
		printf("USB488 configuration\n");
		strn_terminate(buf, "\r\n", len);
		scpi_walk_hierarcy(usb488_hierarcy, buf);
		return;
	}

	/* Red LED indicates data going out */
	gpio_set(RGB_PORT, LED_R);

	for (i = 0; i < len; i++) {
		uart_send_blocking(UART1, charbuf[i]);
	}

	gpio_clear(RGB_PORT, LED_R);
}

void glue_indicator_pulse(void)
{
	gpio_toggle(RGB_PORT, LED_B);
}

static void mainloop(void)
{
}

extern void console_uart_setup(void);

int main(void)
{
	gpio_enable_ahb_aperture();
	clock_setup();
	gpio_setup();
	console_uart_setup();

	usbtmc_init();
	uart_init();

	printf("Initialization complete, bitches\n");

	uart_configure(921600, 8, UART_PARITY_NONE, 1);

	while (1)
		mainloop();

	return 0;
}

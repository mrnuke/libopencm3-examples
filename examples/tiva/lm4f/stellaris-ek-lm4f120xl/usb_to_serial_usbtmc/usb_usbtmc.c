	/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2016 Alexandru Gagniuc <mr.nuke.me@gmail.com>
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

#include "usb_to_serial_usbtmc.h"

#include <stdlib.h>
#include <stdio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/lm4f/rcc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/lm4f/nvic.h>
#include <libopencm3/lm4f/usb.h>

/* FIXME: Move to something more common */
#define USB_CLASS_USBTMC		0xfe
#define USB_SUBCLASS_USBTMC		0x03
#define USB_TMC_PROTOCOL_USB488		0x01

/* USBTMC: Table 15: USBTMC bRequest values */
#define USBTMC_REQ_GET_CAPABILITIES	7
#define USBTMC_INDICATOR_PULSE		64


#define USB488_REQ_READ_STATUS_BYTE	128

enum usbtmc_status {
	USBTMC_STATUS_SUCCESS		= 0x01,
	USBTMC_STATUS_PENDING		= 0x02,
	USBTMC_STATUS_FAILED		= 0x81,
};

/* USBTMC: Table 37: GET_CAPABILITIES response format */
enum usbtmc_interface_capabilities {
	USBTMC_CAP_INDICATOR_PULSE	= 1 << 2,
	USBTMC_CAP_TALK_ONLY		= 1 << 1,
	USBTMC_CAP_LISTEN_ONLY		= 1 << 0,
};

/* USBTMC: Table 37: GET_CAPABILITIES response format */
enum usbtmc_dev_capabilities {
	USBTMC_CAP_TERM_CHAR		= 1 << 0,
};

/* USB488: Table 8: GET_CAPABILITIES response packet */
enum usb488_interface_capabilities {
	USB488_CAP_488_2		= 1 << 2,
	USB488_CAP_REN_CONTROL		= 1 << 1,
	USB488_CAP_TRIGGER		= 1 << 0,
};

/* USB488: Table 8: GET_CAPABILITIES response packet */
enum usb488_dev_capabilities {
	USB488_CAP_SCPI_COMPLIANT	= 1 << 3,
	USB488_CAP_SR1			= 1 << 2,
	USB488_CAP_RL1			= 1 << 1,
	USB488_CAP_DT1			= 1 << 0,
};

/* USBTMC: Table 37: GET_CAPABILITIES response format */
struct usbtmc_capabilities {
	uint8_t status;
	uint8_t reserved;
	uint16_t bcd_usbtmc;
	uint8_t interface_capabilities;
	uint8_t device_capabilities;
	uint8_t reserved_6[6];
} __attribute__((packed));

/* USB488: Table 8: GET_CAPABILITIES response packet */
struct usb488_capabilities {
	struct usbtmc_capabilities tmc_caps;
	uint16_t bcd_usb488;
	uint8_t interface_488_capabilities;
	uint8_t device_488_capabilities;
	uint8_t reserved[8];
} __attribute__((packed));

static const struct usb488_capabilities my_488_caps = {
	.tmc_caps = {
		.status = USBTMC_STATUS_SUCCESS,
		.bcd_usbtmc = 0x0200,
		.interface_capabilities = USBTMC_CAP_INDICATOR_PULSE,
	},
	.interface_488_capabilities = USB488_CAP_REN_CONTROL,
	.bcd_usb488 = 0x0200,
};

static const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0xc03e,
	.idProduct = 0xb007,
	.bcdDevice = 0x2000,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

static const struct usb_endpoint_descriptor data_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x01,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}, {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x81,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}};

static const struct usb_interface_descriptor comm_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_USBTMC,
	.bInterfaceSubClass = USB_SUBCLASS_USBTMC,
	.bInterfaceProtocol = USB_TMC_PROTOCOL_USB488,
	.iInterface = 0,

	.endpoint = data_endp,

	.extra = NULL,
	.extralen = 0,
}};

static const struct usb_endpoint_descriptor moar_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x03,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}, {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x83,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}};

static const struct usb_interface_descriptor moar_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_USBTMC,
	.bInterfaceSubClass = USB_SUBCLASS_USBTMC,
	.bInterfaceProtocol = USB_TMC_PROTOCOL_USB488,
	.iInterface = 0,

	.endpoint = moar_endp,

	.extra = NULL,
	.extralen = 0,
}};

static const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = comm_iface,
},{
	.num_altsetting = 1,
	.altsetting = moar_iface,
}};

static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 2,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static const char *usb_strings[] = {
	"libopencm3",
	"usb_to_serial_usbtmc",
	"none",
	"DEMO",
};

usbd_device *usbtmc_dev;
uint8_t usbd_control_buffer[128];
extern usbd_driver lm4f_usb_driver;

static int usb488_control_request(usbd_device * usbd_dev,
				  struct usb_setup_data *req, uint8_t **buf,
				  uint16_t *len,
				  void (**complete) (usbd_device * usbd_dev,
						     struct usb_setup_data *
						     req))
{
	uint8_t tag;
	static uint8_t ctrl_buf[64];

	(void)complete;
	(void)usbd_dev;

	switch (req->bRequest) {
	case USBTMC_REQ_GET_CAPABILITIES:
		*buf = (void *) &my_488_caps;
		*len = sizeof(my_488_caps);
		return 1;
	case USBTMC_INDICATOR_PULSE:
		glue_indicator_pulse();
		ctrl_buf[0] = USBTMC_STATUS_SUCCESS;
		*buf = ctrl_buf;
		*len = 1;
		return 1;
	case USB488_REQ_READ_STATUS_BYTE:
		tag =  req->wValue & 0x7f;
		ctrl_buf[0] = USBTMC_STATUS_SUCCESS;
		ctrl_buf[1] = tag;
		ctrl_buf[2] = 0; /* status byte */
		*buf = ctrl_buf;
		*len = 3;
		return 1;
	default:
		printf("Unhandled USB TMC request %x\n", req->bRequest);
		break;
	}
	return 0;
}

static inline uint32_t read_le32(void *data)
{
	uint8_t *raw = data;
	return (raw[0] | (raw[1] << 8) | (raw[2] << 16) | (raw[3] << 24));
}

static void usbtmc_data_rx_cb(usbd_device * usbd_dev, uint8_t ep)
{
	uint8_t buf[64];
	uint32_t size;
	static uint32_t num_expected = 0;

	uint16_t len = usbd_ep_read_packet(usbd_dev, ep, buf, 64);

	if (num_expected) {
		if (len > num_expected) {
			printf("Wanted %lu, but got %u\n", num_expected, len);
			usbd_ep_stall_set(usbd_dev, ep, 1);
			printf("Stallin expecting %lu bytes\n", num_expected);
			num_expected = 0;
			return;
		}
		num_expected -= len;
		usb488_rx_cmd_cb(buf, len);
	} else {
		size = read_le32(buf + 4);
		len -= 12;
		num_expected = size - MIN(size, len);
		usb488_rx_cmd_cb(buf + 12, MIN(size, len));
	}

	printf("Still expecting %lu bytes\n", num_expected);
}

void usbtmc_send_data(uint8_t * buf, uint16_t len)
{
	usbd_ep_write_packet(usbtmc_dev, 0x82, buf, len);
}

static void usbtmc_set_config(usbd_device * usbd_dev, uint16_t wValue)
{
	(void)wValue;

	usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64,
		      usbtmc_data_rx_cb);
	usbd_ep_setup(usbd_dev, 0x81, USB_ENDPOINT_ATTR_BULK, 64, NULL);
	usbd_ep_setup(usbd_dev, 0x03, USB_ENDPOINT_ATTR_BULK, 64,
		      usbtmc_data_rx_cb);
	usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_BULK, 64, NULL);

	usbd_register_control_callback(usbd_dev,
				       USB_REQ_TYPE_CLASS |
				       USB_REQ_TYPE_INTERFACE,
				       USB_REQ_TYPE_TYPE |
				       USB_REQ_TYPE_RECIPIENT,
				       usb488_control_request);
}

static void usb_pins_setup(void)
{
	/* USB pins are connected to port D */
	periph_clock_enable(RCC_GPIOD);
	/* Mux USB pins to their analog function */
	gpio_mode_setup(GPIOD, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO4 | GPIO5);
}

static void usb_ints_setup(void)
{
	uint8_t usbints;
	/* Gimme some interrupts */
	usbints = USB_INT_RESET | USB_INT_DISCON | USB_INT_RESUME | USB_INT_SUSPEND;	//| USB_IM_SOF;
	usb_enable_interrupts(usbints, 0xff, 0xff);
	nvic_enable_irq(NVIC_USB0_IRQ);
}

void usbtmc_init(void)
{
	usbd_device *usbd_dev;

	usb_pins_setup();

	usbd_dev = usbd_init(&lm4f_usb_driver, &dev, &config, usb_strings, 4,
			     usbd_control_buffer, sizeof(usbd_control_buffer));
	usbtmc_dev = usbd_dev;
	usbd_register_set_config_callback(usbd_dev, usbtmc_set_config);

	usb_ints_setup();
}

void usb0_isr(void)
{
	usbd_poll(usbtmc_dev);
}

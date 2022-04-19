/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <sys/printk.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   500

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
#define LED0	DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN	DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS	DT_GPIO_FLAGS(LED0_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led0 devicetree alias is not defined"
#define LED0	""
#define PIN	0
#define FLAGS	0
#endif

#define NUNCHUCK_I2C_ADDR 0x52

static int write_bytes(const struct device *i2c_dev, uint16_t addr,
		       uint8_t *data, uint32_t num_bytes)
{
	uint8_t wr_addr[2];
	struct i2c_msg msgs[2];

	/* FRAM address */
	wr_addr[0] = (addr >> 8) & 0xFF;
	wr_addr[1] = addr & 0xFF;

	/* Setup I2C messages */

	/* Send the address to write to */
	msgs[0].buf = wr_addr;
	msgs[0].len = 2U;
	msgs[0].flags = I2C_MSG_WRITE;

	/* Data to be written, and STOP after this. */
	msgs[1].buf = data;
	msgs[1].len = num_bytes;
	msgs[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	return i2c_transfer(i2c_dev, &msgs[0], 2, NUNCHUCK_I2C_ADDR);
}

void main(void)
{
	const struct device *led_dev;
	const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(lpi2c1));
	/* struct device *i2c_dev; */
	/* i2c_dev = device_get_binding(DT_BUS_LABEL(lpi2c1)); */
	bool led_is_on = true;
	int ret;
	uint8_t read_data[16];
	uint16_t write_data;

	led_dev = device_get_binding(LED0);
	if (led_dev == NULL) {
		printk("LED binding failed");
		return;
	}

	ret = gpio_pin_configure(led_dev, PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (ret < 0) {
		printk("LED init failed");
		return;
	}

	if (!device_is_ready(i2c_dev)){
		printk("I2C not ready\n");
		return;
	}

	read_data[0]=0xAE;
	write_data = 0xFA;

	printk("Starting loop...\n");
	ret = i2c_reg_write_byte(i2c_dev, NUNCHUCK_I2C_ADDR, 0xF0, 0x55);
	k_msleep(SLEEP_TIME_MS);
	ret = i2c_reg_write_byte(i2c_dev, NUNCHUCK_I2C_ADDR, 0xFB, 0x00);
	k_msleep(SLEEP_TIME_MS);
	while (1) {
		/* ret = write_bytes(i2c_dev, 0x00, data, 1); */
		/* ret = i2c_write(i2c_dev, data, 1, NUNCHUCK_I2C_ADDR); */
		/* ret = i2c_reg_read_byte(i2c_dev, NUNCHUCK_I2C_ADDR, 0xF0, read_data); */
		/* ret = i2c_read(i2c_dev, read_data, 6, NUNCHUCK_I2C_ADDR); */
		for(int i = 0xff-5; i<=0xff; ++i){
			ret = i2c_reg_read_byte(i2c_dev, NUNCHUCK_I2C_ADDR, i, read_data);
			printk("%x, ", read_data[0]);
		}
		printk("\n");
		/* if (ret) { */
		/* 	printk("Error writing: %d\n", ret); */
		/* } else { */
		/* 	printk("Write success: %d\n", read_data[0]); */
		/* } */
		/* for(int i = 0; i<6; ++i){ */
		/* 	printk("%d, ", data[i]); */
		/* } */
		/* printk("\n"); */
		gpio_pin_set(led_dev, PIN, (int)led_is_on);
		led_is_on = !led_is_on;
		k_msleep(SLEEP_TIME_MS);
		/* printk("Hello world\n"); */
	}
}

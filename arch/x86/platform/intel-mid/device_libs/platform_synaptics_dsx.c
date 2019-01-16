/*
 * platform_synaptics_dsx.c:
 * Synaptics dsx platform data initialization file for moorfield
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/input/synaptics_dsx.h>

#include <asm/intel-mid.h>
#include <linux/lnw_gpio.h>


#define TM1940 (1) /* I2C */
#define TM2448 (2) /* I2C */
#define TM2074 (3) /* SPI */
#define TM2780 (4) /* HID/I2C (RMI mode) */
#define SYNAPTICS_MODULE TM2448


#if (SYNAPTICS_MODULE == TM2448)
#define SYNAPTICS_I2C_DEVICE
#define DSX_I2C_ADDR 0x20
#define DSX_ATTN_GPIO 133
#define DSX_ATTN_MUX_NAME ""
#define DSX_POWER_GPIO -1
#define DSX_POWER_MUX_NAME ""
#define DSX_POWER_ON_STATE 1
#define DSX_POWER_DELAY_MS 160
#define DSX_RESET_GPIO -1
#define DSX_RESET_ON_STATE 0
#define DSX_RESET_DELAY_MS 100
#define DSX_RESET_ACTIVE_MS 20
#define DSX_IRQ_FLAGS (IRQF_TRIGGER_FALLING | IRQF_ONESHOT)
#define DSX_MAX_Y_FOR_2D -1 /* set to -1 if no virtual buttons */
static unsigned char bus_reg_name[] = "";
static unsigned int cap_button_codes[] = {};
static unsigned int vir_button_codes[] = {
		KEY_HOME, 100, 900, 100, 60,
		KEY_BACK, 300, 900, 100, 60
};

#elif (SYNAPTICS_MODULE == TM2780)
#define SYNAPTICS_I2C_DEVICE
#define SYNAPTICS_HID_DEVICE
#define DSX_HID_DEVICE_DESCRIPTOR_ADDR 0x0020
#define DSX_I2C_ADDR 0x2c
#define DSX_ATTN_GPIO 133
#define DSX_ATTN_MUX_NAME ""
#define DSX_POWER_GPIO -1
#define DSX_POWER_MUX_NAME ""
#define DSX_POWER_ON_STATE 1
#define DSX_POWER_DELAY_MS 160
#define DSX_RESET_GPIO -1
#define DSX_RESET_ON_STATE 0
#define DSX_RESET_DELAY_MS 100
#define DSX_RESET_ACTIVE_MS 20
#define DSX_IRQ_ON_STATE 0
#define DSX_IRQ_FLAGS (IRQF_TRIGGER_FALLING | IRQF_ONESHOT)
#define DSX_MAX_Y_FOR_2D -1
static unsigned char bus_reg_name[] = "";
static unsigned int cap_button_codes[] = {};
static unsigned int vir_button_codes[] = {};
#endif

static struct synaptics_dsx_button_map cap_button_map = {
	.nbuttons = ARRAY_SIZE(cap_button_codes),
	.map = cap_button_codes,
};

static struct synaptics_dsx_button_map vir_button_map = {
	.nbuttons = ARRAY_SIZE(vir_button_codes) / 5,
	.map = vir_button_codes,
};

static struct synaptics_dsx_board_data pdata = {
	.irq_gpio = DSX_ATTN_GPIO,
	.swap_axes = false,
	.x_flip = true,
	.y_flip = true,
	.irq_flags = DSX_IRQ_FLAGS,
	.power_gpio = DSX_POWER_GPIO,
	.power_on_state = DSX_POWER_ON_STATE,
	.power_delay_ms = DSX_POWER_DELAY_MS,
	.reset_gpio = DSX_RESET_GPIO,
	.reset_on_state = DSX_RESET_ON_STATE,
	.reset_delay_ms = DSX_RESET_DELAY_MS,
	.reset_active_ms = DSX_RESET_ACTIVE_MS,
	.max_y_for_2d = DSX_MAX_Y_FOR_2D,
	.bus_reg_name = bus_reg_name,
	.cap_button_map = &cap_button_map,
	.vir_button_map = &vir_button_map,
};

void *rmi4_platform_data(void *info)
{
	/* For mofd v1 PR2 synatics s3402 touchscreen */

	struct i2c_board_info *i2c_info = info;

	if (strcmp(i2c_info->type, "synaptics_7508") == 0) {
		pdata.swap_axes = true;
		pdata.x_flip = true;
		pdata.y_flip = false;
	}
	strlcpy(i2c_info->type, I2C_DRIVER_NAME, I2C_NAME_SIZE);
	pr_info("synaptics_dsx_i2c_rmi4_platform_data: %s\n", i2c_info->type);

	pdata.irq_gpio = get_gpio_by_name("ts_int");
	pdata.reset_gpio = get_gpio_by_name("ts_rst");

	return &pdata;
}

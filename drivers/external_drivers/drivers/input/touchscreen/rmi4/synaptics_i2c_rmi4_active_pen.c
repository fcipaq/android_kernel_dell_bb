/*
 * Synaptics DSX touchscreen driver
 *
 * Copyright (C) 2012 Synaptics Incorporated
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/i2c.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/synaptics_i2c_rmi4.h>
#include "synaptics_i2c_rmi4.h"

#define DRIVER_NAME "rmi4_apen"

#define MASK_16BIT		0xFFFF
#define MASK_8BIT		0xFF
#define MASK_7BIT		0x7F
#define MASK_5BIT		0x1F
#define MASK_4BIT		0x0F
#define MASK_3BIT		0x07
#define MASK_2BIT		0x03

#define ACTIVE_PEN_MAX_PRESSURE_16BIT 65535
#define ACTIVE_PEN_MAX_PRESSURE_8BIT 255

struct synaptics_rmi4_f12_query_8 {
	union {
		struct {
			unsigned char size_of_query9;
			struct {
				unsigned char data0_is_present:1;
				unsigned char data1_is_present:1;
				unsigned char data2_is_present:1;
				unsigned char data3_is_present:1;
				unsigned char data4_is_present:1;
				unsigned char data5_is_present:1;
				unsigned char data6_is_present:1;
				unsigned char data7_is_present:1;
			} __packed;
		};
		unsigned char data[2];
	};
};

struct apen_data {
	union {
		struct {
			unsigned char status_pen:1;
			unsigned char status_invert:1;
			unsigned char status_barrel:1;
			unsigned char status_reserved:5;
			unsigned char x_lsb;
			unsigned char x_msb;
			unsigned char y_lsb;
			unsigned char y_msb;
			unsigned char pressure_lsb;
			unsigned char pressure_msb;
		} __packed;
		unsigned char data[7];
	};
};

struct synaptics_rmi4_apen_handle {
	bool apen_present;
	unsigned char intr_mask;
	unsigned short query_base_addr;
	unsigned short control_base_addr;
	unsigned short data_base_addr;
	unsigned short command_base_addr;
	unsigned short apen_data_addr;
	unsigned short max_pressure;
	struct input_dev *apen_dev;
	struct apen_data *apen_data;
	struct rmi4_data *rmi4_data;
};

static struct synaptics_rmi4_apen_handle *apen;

DECLARE_COMPLETION(apen_remove_complete);

static void apen_lift(void)
{
	input_report_key(apen->apen_dev, BTN_TOUCH, 0);
	input_report_key(apen->apen_dev, BTN_TOOL_PEN, 0);
	input_report_key(apen->apen_dev, BTN_TOOL_RUBBER, 0);
	input_sync(apen->apen_dev);
	apen->apen_present = false;

	return;
}

static void apen_report(void)
{
	int retval;
	int x;
	int y;
	int pressure;
	static int invert = -1;
	struct rmi4_data *rmi4_data = apen->rmi4_data;
	struct i2c_client *client = rmi4_data->i2c_client;
	const struct rmi4_touch_calib *calib =
				&apen->rmi4_data->board->calib[apen->rmi4_data->touch_type];

	retval = rmi4_i2c_block_read(rmi4_data,
			apen->apen_data_addr,
			apen->apen_data->data,
			sizeof(apen->apen_data->data));
	if (retval < 0) {
		dev_err(&client->dev,
				"%s: Failed to read active pen data, retval=%d\n",
				__func__, retval);
		return;
	}

	if (apen->apen_data->status_pen == 0) {
		if (apen->apen_present) {
			apen_lift();
			invert = -1;
		}

		dev_dbg(&client->dev,
				"%s: No active pen data\n",
				__func__);

		return;
	}

	x = (apen->apen_data->x_msb << 8) | (apen->apen_data->x_lsb);
	y = (apen->apen_data->y_msb << 8) | (apen->apen_data->y_lsb);

	if ((x == -1) && (y == -1)) {
		dev_dbg(&client->dev,
				"%s: Active pen in range but no valid x & y\n",
				__func__);
		return;
	}

	if (calib->swap_axes)
		swap(x, y);
	if (calib->x_flip)
		x = apen->rmi4_data->sensor_max_x - x;
	if (calib->y_flip)
		y = apen->rmi4_data->sensor_max_y - y;

	if (invert != -1 && invert != apen->apen_data->status_invert)
		apen_lift();

	invert = apen->apen_data->status_invert;

	if (apen->max_pressure == ACTIVE_PEN_MAX_PRESSURE_16BIT) {
		pressure = (apen->apen_data->pressure_msb << 8) |
				apen->apen_data->pressure_lsb;
	} else {
		pressure = apen->apen_data->pressure_lsb;
	}

	input_report_key(apen->apen_dev, BTN_TOUCH, pressure > 0 ? 1 : 0);
	input_report_key(apen->apen_dev,
			apen->apen_data->status_invert > 0 ?
			BTN_TOOL_RUBBER : BTN_TOOL_PEN, 1);
	input_report_key(apen->apen_dev,
			BTN_STYLUS, apen->apen_data->status_barrel > 0 ?
			1 : 0);
	input_report_abs(apen->apen_dev, ABS_X, x);
	input_report_abs(apen->apen_dev, ABS_Y, y);
	input_report_abs(apen->apen_dev, ABS_PRESSURE, pressure);

	input_sync(apen->apen_dev);

	dev_dbg(&client->dev,
			"%s: Active pen:\n"
			"status = %d\n"
			"invert = %d\n"
			"barrel = %d\n"
			"x = %d\n"
			"y = %d\n"
			"pressure = %d\n",
			__func__,
			apen->apen_data->status_pen,
			apen->apen_data->status_invert,
			apen->apen_data->status_barrel,
			x, y, pressure);

	apen->apen_present = true;

	return;
}

static void apen_set_params(void)
{
	input_set_abs_params(apen->apen_dev, ABS_X, 0,
			apen->rmi4_data->sensor_max_x, 0, 0);
	input_set_abs_params(apen->apen_dev, ABS_Y, 0,
			apen->rmi4_data->sensor_max_y, 0, 0);
	input_set_abs_params(apen->apen_dev, ABS_PRESSURE, 0,
			apen->max_pressure, 0, 0);

	return;
}

static int apen_pressure(struct synaptics_rmi4_f12_query_8 *query_8)
{
	int retval;
	unsigned char ii;
	unsigned char data_reg_presence;
	unsigned char size_of_query_9;
	unsigned char *query_9;
	unsigned char *data_desc;
	struct rmi4_data *rmi4_data = apen->rmi4_data;

	data_reg_presence = query_8->data[1];

	size_of_query_9 = query_8->size_of_query9;
	query_9 = kmalloc(size_of_query_9, GFP_KERNEL);

	retval = rmi4_i2c_block_read(rmi4_data,
			apen->query_base_addr + 9,
			query_9,
			size_of_query_9);
	if (retval < 0)
		goto exit;

	data_desc = query_9;

	for (ii = 0; ii < 6; ii++) {
		if (!(data_reg_presence & (1 << ii)))
			continue; /* The data register is not present */
		data_desc++; /* Jump over the size entry */
		while (*data_desc & (1 << 7))
			data_desc++;
		data_desc++; /* Go to the next descriptor */
	}

	data_desc++; /* Jump over the size entry */
	/* Check for the presence of subpackets 1 and 2 */
	if ((*data_desc & (3 << 1)) == (3 << 1))
		apen->max_pressure = ACTIVE_PEN_MAX_PRESSURE_16BIT;
	else
		apen->max_pressure = ACTIVE_PEN_MAX_PRESSURE_8BIT;

exit:
	kfree(query_9);

	return retval;
}

static int apen_reg_init(void)
{
	int retval;
	unsigned char data_offset;
	unsigned char size_of_query8;
	struct synaptics_rmi4_f12_query_8 query_8;
	struct rmi4_data *rmi4_data = apen->rmi4_data;
	struct i2c_client *client = rmi4_data->i2c_client;

	retval = rmi4_i2c_block_read(rmi4_data,
			apen->query_base_addr + 7,
			&size_of_query8,
			sizeof(size_of_query8));
	if (retval < 0) {
        dev_err(&client->dev, "%s: rmi4_i2c_block_read1 failure, retval=%d\n",
                __func__, retval);
		return retval;
	}

	retval = rmi4_i2c_block_read(rmi4_data,
			apen->query_base_addr + 8,
			query_8.data,
			size_of_query8);
	if (retval < 0) {
        dev_err(&client->dev, "%s: rmi4_i2c_block_read2 failure, retval=%d\n",
                __func__, retval);
		return retval;
	}

	if ((size_of_query8 >= 2) && (query_8.data6_is_present)) {
		data_offset = query_8.data0_is_present +
				query_8.data1_is_present +
				query_8.data2_is_present +
				query_8.data3_is_present +
				query_8.data4_is_present +
				query_8.data5_is_present;
		apen->apen_data_addr = apen->data_base_addr + data_offset;
		retval = apen_pressure(&query_8);
		if (retval < 0)
			return retval;
	} else {
		dev_err(&client->dev,
				"%s: Active pen support unavailable\n",
				__func__);
		retval = -ENODEV;
	}

	return retval;
}

static int apen_scan_pdt(void)
{
	int retval;
	unsigned char ii;
	unsigned char page;
	unsigned char intr_count = 0;
	unsigned char intr_off;
	unsigned char intr_src;
	unsigned short addr;
	struct rmi4_fn_desc fd;
	struct rmi4_data *rmi4_data = apen->rmi4_data;
	struct i2c_client *client = rmi4_data->i2c_client;

	for (page = 0; page < RMI4_MAX_PAGE; page++) {
		for (addr = PDT_START_SCAN_LOCATION; addr > PDT_END_SCAN_LOCATION; addr -= PDT_ENTRY_SIZE) {
			addr |= (page << 8);

			retval = rmi4_i2c_block_read(rmi4_data,
					addr,
					(unsigned char *)&fd,
					sizeof(fd));
			if (retval < 0)
				return retval;

			addr &= ~(MASK_8BIT << 8);

			if (fd.fn_number) {
				dev_dbg(&client->dev,
						"%s: Found F%02x\n",
						__func__, fd.fn_number);
				switch (fd.fn_number) {
				case RMI4_TOUCHPAD_F12_FUNC_NUM:
					goto f12_found;
					break;
				}
			} else {
				break;
			}

			intr_count += (fd.intr_src_count & MASK_3BIT);
		}
	}

	dev_err(&client->dev,
			"%s: Failed to find F12\n",
			__func__);
	return -EINVAL;

f12_found:
	apen->query_base_addr = fd.query_base_addr | (page << 8);
	apen->control_base_addr = fd.ctrl_base_addr | (page << 8);
	apen->data_base_addr = fd.data_base_addr | (page << 8);
	apen->command_base_addr = fd.cmd_base_addr | (page << 8);

	retval = apen_reg_init();
	if (retval < 0) {
		dev_err(&client->dev,
				"%s: Failed to initialize active pen registers, retval=%d\n",
				__func__, retval);
		return retval;
	}

	apen->intr_mask = 0;
	intr_src = fd.intr_src_count;
	intr_off = intr_count % 8;
	for (ii = intr_off;
			ii < ((intr_src & MASK_3BIT) +
			intr_off);
			ii++) {
		apen->intr_mask |= 1 << ii;
	}

	addr = rmi4_data->fn01_ctrl_base_addr + 1;

	retval = rmi4_i2c_block_write(rmi4_data,
			addr,
			&(apen->intr_mask),
			sizeof(apen->intr_mask));
	if (retval < 0) {
		dev_err(&client->dev,
				"%s: Failed to set interrupt enable bit, retval=%d\n",
				__func__, retval);
		return retval;
	}

	return 0;
}

void synaptics_rmi4_apen_attn_check(struct rmi4_data *rmi4_data,
		unsigned char intr_mask)
{
	struct i2c_client *client = rmi4_data->i2c_client;
	if (!apen) {
		dev_info(&client->dev, "%s: apen uninitialized\n", __func__);
		return;
	}

	if (apen->intr_mask & intr_mask)
		apen_report();
	else
		dev_info(&client->dev, "no apen_report(): apen->intr_mask=%X, intr_mask=%X\n", apen->intr_mask, intr_mask);

	return;
}

int synaptics_rmi4_apen_init(struct rmi4_data *rmi4_data)
{
	int retval;
	struct i2c_client *client = rmi4_data->i2c_client;

	apen = kzalloc(sizeof(*apen), GFP_KERNEL);
	if (!apen) {
		dev_err(&client->dev,
				"%s: Failed to alloc mem for apen\n",
				__func__);
		retval = -ENOMEM;
		goto exit;
	}

	apen->apen_data = kzalloc(sizeof(*(apen->apen_data)), GFP_KERNEL);
	if (!apen->apen_data) {
		dev_err(&client->dev,
				"%s: Failed to alloc mem for apen_data\n",
				__func__);
		retval = -ENOMEM;
		goto exit_free_apen;
	}

	apen->rmi4_data = rmi4_data;

	retval = apen_scan_pdt();
	if (retval < 0)
		goto exit_free_apen_data;

	apen->apen_dev = input_allocate_device();
	if (apen->apen_dev == NULL) {
		dev_err(&client->dev,
				"%s: Failed to allocate active pen device\n",
				__func__);
		retval = -ENOMEM;
		goto exit_free_apen_data;
	}

	apen->apen_dev->name = DRIVER_NAME;
	apen->apen_dev->phys = "Synaptics Active Pen";
	apen->apen_dev->id.bustype = BUS_I2C;
	apen->apen_dev->dev.parent = &client->dev;
	input_set_drvdata(apen->apen_dev, rmi4_data);

	set_bit(EV_KEY, apen->apen_dev->evbit);
	set_bit(EV_ABS, apen->apen_dev->evbit);
	set_bit(BTN_TOUCH, apen->apen_dev->keybit);
	set_bit(BTN_TOOL_PEN, apen->apen_dev->keybit);
	set_bit(BTN_TOOL_RUBBER, apen->apen_dev->keybit);
	set_bit(BTN_STYLUS, apen->apen_dev->keybit);
#ifdef INPUT_PROP_DIRECT
	set_bit(INPUT_PROP_DIRECT, apen->apen_dev->propbit);
#endif

	apen_set_params();

	retval = input_register_device(apen->apen_dev);
	if (retval) {
		dev_err(&client->dev,
				"%s: Failed to register active pen device\n",
				__func__);
		goto exit_free_input_device;
	}

	return 0;

exit_free_input_device:
	input_free_device(apen->apen_dev);

exit_free_apen_data:
	kfree(apen->apen_data);

exit_free_apen:
	kfree(apen);
	apen = NULL;

exit:
	return retval;
}

static void synaptics_rmi4_apen_remove(struct rmi4_data *rmi4_data)
{
	if (!apen)
		goto exit;

	input_unregister_device(apen->apen_dev);
	kfree(apen->apen_data);
	kfree(apen);
	apen = NULL;

exit:
	complete(&apen_remove_complete);

	return;
}

static void synaptics_rmi4_apen_reset(struct rmi4_data *rmi4_data)
{
	apen_lift();

	apen_scan_pdt();

	apen_set_params();

	return;
}

static void synaptics_rmi4_apen_reinit(struct rmi4_data *rmi4_data)
{
	if (!apen)
		return;

	apen_lift();

	return;
}

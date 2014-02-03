/*
 * Elan I2C Touchpad diver
 *
 * Copyright (c) 2012 ELAN Microelectronics Corp.
 *
 * Author: (Jian-Jhong Ding) <jj_ding@emc.com.tw>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * Trademarks are the property of their respective owners.
 */

#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/elan_touchpad.h>

#define DRIVER_VERSION		"0.0.1"		//Joe modify

#define DRIVER_NAME		"elan_i2c"
#define HID_DESC_LENGTH		30
#define HID_REPORT_ID_OFFSET	2

/* Length of Elan touchpad information */
#define ETP_INF_LENGTH		2
#define ETP_MAX_FINGERS		5
#define ETP_REPORT_DESC_LENGTH	106
#define ETP_REPORT_LENGTH	30
#define ETP_FINGER_DATA_OFFSET	4
#define ETP_FINGER_DATA_LEN	5

#define ETP_REPORT_ID		0x5d

#define HID_CMD_REGISTER	0x0005
#define ETP_CMD_REGISTER	0x0300

#define CMD_RESET		0x0100
#define CMD_WAKE_UP		0x0800
#define CMD_SLEEP		0x0801
#define CMD_ENABLE_ABS		0x0001

#define REG_DESC		0x0001
#define REG_REPORT_DESC		0x0002
#define REG_FW_VERSION		0x0102
#define REG_XY_TRACE_NUM	0x0105
#define REG_X_AXIS_MAX		0x0106
#define REG_Y_AXIS_MAX		0x0107
#define REG_RESOLUTION		0x0108

/* The main device structure */
struct elan_i2c_data {
	struct i2c_client	*client;
	struct input_dev	*input;
	struct workqueue_struct *touchpad_wq;
	struct attribute_group attrs;
	unsigned int		max_x;
	unsigned int		max_y;
	unsigned int		width_x;
	unsigned int		width_y;
	unsigned int		irq;
	unsigned int		int_gpio;
	int touchpadp_status;
};

struct elan_i2c_data *elan_touchpad_data;
static int report_desc_length;

static ssize_t elan_i2c_get_status(struct device *dev, struct device_attribute *attr, char *buf);

static DEVICE_ATTR(touchpad_status, (S_IWUSR|S_IRUGO), elan_i2c_get_status, NULL);

static struct attribute *touchpad_attr[] = {
	&dev_attr_touchpad_status.attr,
	NULL
};

static int __elan_i2c_read_reg(struct i2c_client *client, u16 reg,
				u8 *val, u16 len)
{
	struct i2c_msg msgs[2];
	u8 buf[2];
	int ret;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;

	msgs[0].addr = elan_touchpad_data->client->addr;
	msgs[0].flags = elan_touchpad_data->client->flags & I2C_M_TEN;
	msgs[0].len = 2;
	msgs[0].buf = buf;

	msgs[1].addr = elan_touchpad_data->client->addr;
	msgs[1].flags = elan_touchpad_data->client->flags & I2C_M_TEN;
	msgs[1].flags |= I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = val;

	ret = i2c_transfer(elan_touchpad_data->client->adapter, msgs, 2);
	if (ret < 0)
		return ret;

	return ret != 2 ? -EIO : 0;
}

static int elan_i2c_read_reg(struct i2c_client *client, u16 reg, u8 *val)
{
	int retval;

	retval = __elan_i2c_read_reg(elan_touchpad_data->client, reg, val, ETP_INF_LENGTH);
	if (retval < 0) {
		dev_err(&client->dev, "reading register (0x%04x) failed!\n",
			reg);
		return retval;
	}
	
	printk("[Touchpad]: %s: 0x%x: 0x%x, 0x%x \n", __func__, reg, val[0], val[1] );
	
	return 0;
}

static int elan_i2c_write_reg_cmd(struct i2c_client *client, u16 reg, u16 cmd)
{
	struct i2c_msg msg;
	u8 buf[4];
	int ret;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;
	buf[2] = cmd & 0xff;
	buf[3] = (cmd >> 8) & 0xff;

	msg.addr = elan_touchpad_data->client->addr;
	msg.flags = elan_touchpad_data->client->flags & I2C_M_TEN;
	msg.len = 4;
	msg.buf = buf;

	ret = i2c_transfer(elan_touchpad_data->client->adapter, &msg, 1);
	if (ret < 0)
		return ret;

	return ret != 1 ? -EIO : 0;
}

static int elan_i2c_reset(struct i2c_client *client)
{
	return elan_i2c_write_reg_cmd(elan_touchpad_data->client, HID_CMD_REGISTER,
					CMD_RESET);
}

static int elan_i2c_wake_up(struct i2c_client *client)
{
	return elan_i2c_write_reg_cmd(elan_touchpad_data->client, HID_CMD_REGISTER,
					CMD_WAKE_UP);
}

static int elan_i2c_sleep(struct i2c_client *client)
{
	return elan_i2c_write_reg_cmd(elan_touchpad_data->client, HID_CMD_REGISTER,
					CMD_SLEEP);
}

static int elan_i2c_enable_absolute_mode(struct i2c_client *client)
{
	return elan_i2c_write_reg_cmd(elan_touchpad_data->client, ETP_CMD_REGISTER,
					CMD_ENABLE_ABS);
}

static int elan_i2c_get_desc(struct i2c_client *client, u8 *val)
{
	return __elan_i2c_read_reg(elan_touchpad_data->client, REG_DESC, val,
				   HID_DESC_LENGTH);
}

static int elan_i2c_get_report_desc(struct i2c_client *client, u8 *val)
{
	return __elan_i2c_read_reg(elan_touchpad_data->client, REG_REPORT_DESC, val,
				   report_desc_length);
}

static int elan_i2c_get_x_max(struct i2c_client *client, u8 *val)
{
	return elan_i2c_read_reg(elan_touchpad_data->client, REG_X_AXIS_MAX, val);
}

static int elan_i2c_get_y_max(struct i2c_client *client, u8 *val)
{
	return elan_i2c_read_reg(elan_touchpad_data->client, REG_Y_AXIS_MAX, val);
}

static int elan_i2c_get_trace_num(struct i2c_client *client, u8 *val)
{
	return elan_i2c_read_reg(elan_touchpad_data->client, REG_XY_TRACE_NUM, val);
}

static int elan_i2c_get_fw_version(struct i2c_client *client, u8 *val)
{
	return elan_i2c_read_reg(elan_touchpad_data->client, REG_FW_VERSION, val);
}

static int elan_i2c_get_resolution(struct i2c_client *client, u8 *val)
{
	return elan_i2c_read_reg(elan_touchpad_data->client, REG_RESOLUTION, val);
}

static int elan_i2c_initialize(struct i2c_client *client)
{
//	struct device *dev = &client->dev;
	u8 val[ETP_REPORT_DESC_LENGTH];
	int rc;
	int i;
	

	rc = elan_i2c_reset(elan_touchpad_data->client);
//	rc = elan_i2c_sleep(elan_touchpad_data->client);
	if (rc < 0) {
		printk(KERN_ERR "[Touchpad] %s:[%d]: device reset failed.\n", __func__, __LINE__);
		return -1;
	}

	rc = elan_i2c_get_desc(elan_touchpad_data->client, val);
	if (rc < 0) {
		printk(KERN_ERR "[Touchpad] %s:[%d]: couldn't get device descriptor.\n", __func__, __LINE__);
		return -1;
	}
	
	report_desc_length = (val[5] << 2 | val[4]);
	printk("[Touchpad]: report_desc_length = %d \n", report_desc_length);
	
	printk("[Touchpad]: HID_DESC_LENGTH \n");
	for (i = 0 ; i < HID_DESC_LENGTH; i++)
	{
		printk("0x%x ", val[i] );
	}
	
	printk(" \n" );
	
	
	rc = elan_i2c_get_report_desc(elan_touchpad_data->client, val);
	if (rc < 0) {
		printk(KERN_ERR "[Touchpad] %s:[%d]: fetching report descriptor failed.\n", __func__, __LINE__);
		return -1;
	}
	printk("[Touchpad]: ETP_REPORT_DESC_LENGTH \n");
	for (i = 0 ; i < report_desc_length; i++)
	{
		printk("0x%x ", val[i] );
	}
	
	printk(" \n" );
	return 0;
}

static void elan_i2c_report_absolute(struct elan_i2c_data *data,
				     u8 *packet)
{
//	struct input_dev *input = data->input;
	u8 *finger_data = &packet[ETP_FINGER_DATA_OFFSET];
	bool finger_on;
	int pos_x, pos_y;
	int area_x, area_y, pressure;
	int i;

	for (i = 0 ; i < ETP_MAX_FINGERS ; i++) {
		finger_on = (packet[3] >> (3 + i)) & 0x01;

		if (finger_on) {
			pos_x = ((finger_data[0] & 0xf0) << 4) | finger_data[1];
			pos_y = data->max_y -
				(((finger_data[0] & 0x0f) << 8) |
				   finger_data[2]);
			area_x = (finger_data[3] & 0x0f) * data->width_x;
			area_y = (finger_data[3] >> 4) * data->width_y;
			pressure = finger_data[4];

			finger_data += ETP_FINGER_DATA_LEN;

			input_mt_slot(elan_touchpad_data->input, i);
			input_mt_report_slot_state(elan_touchpad_data->input, MT_TOOL_FINGER, true);

			input_report_abs(elan_touchpad_data->input, ABS_MT_POSITION_X, pos_x);
			input_report_abs(elan_touchpad_data->input, ABS_MT_POSITION_Y, pos_y);
			input_report_abs(elan_touchpad_data->input, ABS_MT_PRESSURE, pressure);
			/* use x-axis value as TOOL_WIDTH */
			input_report_abs(elan_touchpad_data->input, ABS_TOOL_WIDTH,
					 finger_data[3] & 0x0f);
			input_report_abs(elan_touchpad_data->input, ABS_MT_TOUCH_MAJOR,
					 max(area_x, area_y));
			input_report_abs(elan_touchpad_data->input, ABS_MT_TOUCH_MINOR,
					 min(area_x, area_y));
		} else {
			input_mt_slot(elan_touchpad_data->input, i);
			input_mt_report_slot_state(elan_touchpad_data->input, MT_TOOL_FINGER,
						   false);
		}
	}

	input_report_key(elan_touchpad_data->input, BTN_LEFT, ((packet[3] & 0x01) == 1));
	input_mt_report_pointer_emulation(elan_touchpad_data->input, true);
	input_sync(elan_touchpad_data->input);
}

static int elan_i2c_check_packet(u8 *packet)
{
	u16 length = le16_to_cpu(packet[0]);
	u8 report_id = packet[HID_REPORT_ID_OFFSET];

	if (length != ETP_REPORT_LENGTH ||
	    report_id != ETP_REPORT_ID)
		return -1;

	return 0;
}

static irqreturn_t elan_i2c_isr(int irq, void *dev_id)
{
//	struct elan_i2c_data *data = dev_id;
	u8 packet[ETP_REPORT_LENGTH];
	int retval;
	
//	printk("[Touchpad] %s:[%d] ++  \n", __func__, __LINE__);
	
	retval = i2c_master_recv(elan_touchpad_data->client, packet, ETP_REPORT_LENGTH);
	if (retval != ETP_REPORT_LENGTH || elan_i2c_check_packet(packet)) {
		printk(KERN_ERR "[Touchpad] %s:[%d]: wrong packet data.\n", __func__, __LINE__);
		goto elan_isr_end;
	}

	elan_i2c_report_absolute(elan_touchpad_data, packet);

elan_isr_end:
	return IRQ_HANDLED;
}

/*
 * (value from firmware) * 10 + 790 = dpi
 * we also have to convert dpi to dots/mm (*10/254 to avoid floating point)
 */
static unsigned int elan_i2c_convert_res(unsigned int val)
{
	return (val * 10 + 790) * 10 / 254;
}

static int elan_i2c_input_dev_create(struct elan_i2c_data *data)
{
//	struct i2c_client *client = data->client;
//	struct input_dev *input;
	unsigned int x_res, y_res;
	u8 val[2];
	int ret;

	elan_touchpad_data->input = input_allocate_device();
	if (!elan_touchpad_data->input)
	{
		printk(KERN_ERR "[Touchpad] %s:[%d]: input allocate device failed!! \n", __func__, __LINE__);
		return -ENOMEM;
	}
	
	elan_touchpad_data->input->name = "Elan-I2C-Touchpad";
	elan_touchpad_data->input->id.bustype = BUS_I2C;
//	elan_touchpad_data->input->dev.parent = &elan_touchpad_data->client->dev;
	
//	printk("[Touchpad] %s:[%d]: elan_touchpad_data->input->dev.parent =%x \n", __func__, __LINE__, elan_touchpad_data->input->dev.parent);
	
	__set_bit(INPUT_PROP_POINTER, elan_touchpad_data->input->propbit);
	__set_bit(INPUT_PROP_BUTTONPAD, elan_touchpad_data->input->propbit);
	__set_bit(EV_KEY, elan_touchpad_data->input->evbit);
	__set_bit(EV_ABS, elan_touchpad_data->input->evbit);

	__set_bit(BTN_LEFT, elan_touchpad_data->input->keybit);

	__set_bit(BTN_TOUCH, elan_touchpad_data->input->keybit);
	__set_bit(BTN_TOOL_FINGER, elan_touchpad_data->input->keybit);
	__set_bit(BTN_TOOL_DOUBLETAP, elan_touchpad_data->input->keybit);
	__set_bit(BTN_TOOL_TRIPLETAP, elan_touchpad_data->input->keybit);
	__set_bit(BTN_TOOL_QUADTAP, elan_touchpad_data->input->keybit);
	__set_bit(BTN_TOOL_QUINTTAP, elan_touchpad_data->input->keybit);
	
	elan_i2c_get_x_max(elan_touchpad_data->client, val);
	data->max_x = (0x0f & val[1]) << 8 | val[0];

	elan_i2c_get_y_max(elan_touchpad_data->client, val);
	data->max_y = (0x0f & val[1]) << 8 | val[0];

	elan_i2c_get_trace_num(elan_touchpad_data->client, val);
	elan_touchpad_data->width_x = elan_touchpad_data->max_x / (val[0] - 1);
	elan_touchpad_data->width_y = elan_touchpad_data->max_y / (val[1] - 1);

	elan_i2c_get_resolution(elan_touchpad_data->client, val);
	x_res = elan_i2c_convert_res(val[0]);
	y_res = elan_i2c_convert_res(val[1]);

	elan_i2c_get_fw_version(elan_touchpad_data->client, val);
	
	printk("Elan I2C Trackpad Information:\n" \
		"    Firmware Version:  0x%02x%02x\n" \
		"    Max ABS X,Y:   %d,%d\n" \
		"    Width X,Y:   %d,%d\n" \
		"    Resolution X,Y:   %d,%d (dots/mm)\n",
		val[1], val[0],
		elan_touchpad_data->max_x, elan_touchpad_data->max_y, elan_touchpad_data->width_x,
		elan_touchpad_data->width_y, x_res, y_res);

	input_set_abs_params(elan_touchpad_data->input, ABS_X, 0, elan_touchpad_data->max_x, 0, 0);
	input_set_abs_params(elan_touchpad_data->input, ABS_Y, 0, elan_touchpad_data->max_y, 0, 0);
	input_abs_set_res(elan_touchpad_data->input, ABS_X, x_res);
	input_abs_set_res(elan_touchpad_data->input, ABS_Y, y_res);
	input_set_abs_params(elan_touchpad_data->input, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(elan_touchpad_data->input, ABS_TOOL_WIDTH, 0, 15, 0, 0);

	input_mt_init_slots(elan_touchpad_data->input, ETP_MAX_FINGERS);
	input_set_abs_params(elan_touchpad_data->input, ABS_MT_POSITION_X, 0, elan_touchpad_data->max_x, 0, 0);
	input_set_abs_params(elan_touchpad_data->input, ABS_MT_POSITION_Y, 0, elan_touchpad_data->max_y, 0, 0);
	input_abs_set_res(elan_touchpad_data->input, ABS_MT_POSITION_X, x_res);
	input_abs_set_res(elan_touchpad_data->input, ABS_MT_POSITION_Y, y_res);
	input_set_abs_params(elan_touchpad_data->input, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(elan_touchpad_data->input, ABS_MT_TOUCH_MAJOR, 0,
			     15 * max(elan_touchpad_data->width_x, elan_touchpad_data->width_y), 0, 0);
	input_set_abs_params(elan_touchpad_data->input, ABS_MT_TOUCH_MINOR, 0,
			     15 * min(elan_touchpad_data->width_x, elan_touchpad_data->width_y), 0, 0);

	/* Register the device in input subsystem */
	ret = input_register_device(elan_touchpad_data->input);
	if (ret) {
		printk(KERN_ERR "[Touchpad] %s:[%d]: input device register failed, %d\n", __func__, __LINE__, ret);
		goto err_free_device;
	}

	return 0;

err_free_device:
	input_free_device(elan_touchpad_data->input);
	return ret;
}


static ssize_t elan_i2c_get_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk(KERN_INFO "[Touchpad]:touchpad status=%d. \n",elan_touchpad_data->touchpadp_status);
	return sprintf(buf, "%d \n", elan_touchpad_data->touchpadp_status);
}


static int __devinit elan_i2c_probe(struct i2c_client *client,
			  const struct i2c_device_id *dev_id)
{
//	struct elan_i2c_data *data;
	struct elan_touchpad_i2c_platform_data *pdata;
	int ret;
	
	printk("[Touchpad] %s ++ \n", __func__);
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "[Touchpad] %s:[%d]: i2c check functionality error.\n", __func__, __LINE__);
		ret = -ENODEV;
		goto probe_err_check_functionality_failed;
	}

	elan_touchpad_data = kzalloc(sizeof(struct elan_i2c_data), GFP_KERNEL);
	if (elan_touchpad_data == NULL) {
		printk(KERN_ERR "[Touchpad] %s:[%d]: allocate ite_chip failed.\n", __func__, __LINE__);
		ret = -ENOMEM;
		goto probe_err_alloc_data_failed;
	}

	elan_touchpad_data->touchpad_wq = create_singlethread_workqueue("touchpad_wq");
	if (!elan_touchpad_data->touchpad_wq) {
		printk(KERN_ERR "[Touchpad] %s:[%d]: create workqueue failed.\n", __func__, __LINE__);
		ret = -ENOMEM;
		goto probe_err_create_wq_failed;
	}
	
	elan_touchpad_data->client = client;
	i2c_set_clientdata(client, elan_touchpad_data);
	pdata = client->dev.platform_data;
	if (likely(pdata != NULL)) {
		elan_touchpad_data->int_gpio = pdata->int_gpio;
		printk("[Touchpad] %s:[%d]: pdata != NULL \n", __func__, __LINE__);
	}

	printk("[Touchpad] %s:[%d]: int_gpio =%d \n", __func__, __LINE__,elan_touchpad_data->int_gpio);
	
	/*init interrupt pin*/
	ret = gpio_request(elan_touchpad_data->int_gpio, "touchpad-irq");
	if(ret < 0)
		printk(KERN_ERR "[Touchpad]: Failed to request GPIO%d (touchpad-irq) error=%d\n", elan_touchpad_data->int_gpio, ret);

	ret = gpio_direction_input(elan_touchpad_data->int_gpio);
	if (ret){
		printk(KERN_ERR "[Touchpad]: Failed to set interrupt direction, error=%d\n", ret);
		goto probe_err_gpio_direction_input_failed;
	}
	
	elan_touchpad_data->irq = gpio_to_irq(elan_touchpad_data->int_gpio);
	printk("[Touchpad]: intr_gpio=%d, irq=%d \n", elan_touchpad_data->int_gpio, elan_touchpad_data->irq);

//Joe	elan_touchpad_data->client = client;
//Joe	elan_touchpad_data->irq = client->irq;

	ret = elan_i2c_initialize(elan_touchpad_data->client);
	if (ret < 0)
	{
		printk(KERN_ERR "[Touchpad]: Failed to i2c initialize, error=%d\n", ret);
		goto probe_err_i2c_init;
	}

	ret = elan_i2c_input_dev_create(elan_touchpad_data);
	if (ret < 0)
	{
		printk(KERN_ERR "[Touchpad]: Failed to input device create, error=%d\n", ret);	
		goto probe_err_input_dev;
	}
	
//	ret = request_irq(elan_touchpad_data->irq, elan_i2c_isr, IRQF_TRIGGER_FALLING, elan_touchpad_data->client->name, elan_touchpad_data->client);
	
	ret = request_threaded_irq(elan_touchpad_data->irq, NULL, elan_i2c_isr,
				   IRQF_TRIGGER_FALLING,
				   elan_touchpad_data->client->name, elan_touchpad_data);
	if (ret < 0) {
		printk(KERN_ERR "[Touchpad]: Could not register interrupt for, irq=%d, ret=%d\n", elan_touchpad_data->irq, ret);
		goto probe_err_irq;
	}

	ret = elan_i2c_enable_absolute_mode(elan_touchpad_data->client);
	if (ret < 0) {
		printk(KERN_ERR "[Touchpad]: can't switch to absolute mode.\n");
		goto probe_err_switch_mode;
	}
	
	elan_i2c_sleep(elan_touchpad_data->client);
	msleep(10);
	
	ret = elan_i2c_wake_up(elan_touchpad_data->client);
	if (ret < 0) {
		printk(KERN_ERR "[Touchpad]: device wake up failed.\n");
		goto probe_err_switch_mode;
	}
	
	elan_touchpad_data->attrs.attrs = touchpad_attr;
    ret = sysfs_create_group(&client->dev.kobj, &elan_touchpad_data->attrs);
    if (ret) {
        printk(KERN_ERR "[Touchpad]: Not able to create the sysfs.\n");
    }

	device_init_wakeup(&elan_touchpad_data->client->dev, 1);
	
	elan_touchpad_data->touchpadp_status = 1;
	
	printk("[Touchpad]: Elan Touchpad Driver ver:%s . \n", DRIVER_VERSION);
			
	return 0;

probe_err_switch_mode:
	free_irq(elan_touchpad_data->irq, elan_touchpad_data);
probe_err_irq:
	input_unregister_device(elan_touchpad_data->input);
probe_err_input_dev:
probe_err_i2c_init:
probe_err_gpio_direction_input_failed:
	gpio_free(elan_touchpad_data->int_gpio);
probe_err_create_wq_failed:
	if (elan_touchpad_data->touchpad_wq)
		destroy_workqueue(elan_touchpad_data->touchpad_wq);
        
	kfree(elan_touchpad_data);
probe_err_alloc_data_failed:
probe_err_check_functionality_failed:
	
	elan_touchpad_data->touchpadp_status = 0;
	return ret;
}

static int __devexit elan_i2c_remove(struct i2c_client *client)
{
	free_irq(elan_touchpad_data->irq, elan_touchpad_data);
	input_unregister_device(elan_touchpad_data->input);
	gpio_free(elan_touchpad_data->int_gpio);
	if (elan_touchpad_data->touchpad_wq)
		destroy_workqueue(elan_touchpad_data->touchpad_wq);
        
	kfree(elan_touchpad_data);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int elan_i2c_suspend(struct device *dev)
{
//	struct i2c_client *client = to_i2c_client(dev);
	
	printk("[Touchpad] %s:[%d] ++  \n", __func__, __LINE__);
	
	if (device_may_wakeup(&elan_touchpad_data->client->dev))
	{
		printk("[Touchpad] %s:[%d] ++  \n", __func__, __LINE__);
		enable_irq_wake(elan_touchpad_data->irq);
	}

	elan_i2c_sleep(elan_touchpad_data->client);

	return 0;
}

static int elan_i2c_resume(struct device *dev)
{
//	struct i2c_client *client = to_i2c_client(dev);
	int ret;
	
	printk("[Touchpad] %s:[%d] ++  \n", __func__, __LINE__);

	ret = elan_i2c_wake_up(elan_touchpad_data->client);
	if (ret < 0)
		return ret;

	if (device_may_wakeup(&elan_touchpad_data->client->dev))
		enable_irq_wake(elan_touchpad_data->irq);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(elan_i2c_pm_ops,
			 elan_i2c_suspend, elan_i2c_resume);

static const struct i2c_device_id elan_i2c_id[] = {
	{ ELAN_TOUCHPAD_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, elan_i2c_id);

static struct i2c_driver elan_i2c_driver = {
	.driver = {
		.name	= ELAN_TOUCHPAD_NAME,
		.owner	= THIS_MODULE,
		.pm	= &elan_i2c_pm_ops,
	},
	.probe		= elan_i2c_probe,
	.remove		= __devexit_p(elan_i2c_remove),
	.id_table	= elan_i2c_id,
};

//module_i2c_driver(elan_i2c_driver);

static int __init elan_i2c_init(void)
{
	printk("[Touchpad]: %s ++ \n", __func__);
	return i2c_add_driver(&elan_i2c_driver);
}

static void __exit elan_i2c_exit(void)
{
	i2c_del_driver(&elan_i2c_driver);
}
module_init(elan_i2c_init);
module_exit(elan_i2c_exit);

MODULE_AUTHOR("JJ Ding <jj_ding@emc.com.tw>");
MODULE_DESCRIPTION("Elan I2C Touchpad driver");
MODULE_LICENSE("GPL");

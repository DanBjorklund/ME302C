/*
 * platform_accel.c: accel platform data initilization file
 *
 * (C) Copyright 2013
 * Author : cheng_kao
 *
 */

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include "platform_accel.h"

void *accel_platform_data(void *info)
{
//	int pin;
//	struct i2c_board_info *i2c_accel_info = (struct i2c_board_info *)info;
//	pin = get_gpio_by_name("accel_int");
//	if (pin == -1) {
//		printk("alp accel_platform_data get GPIO name fail, use default!!\n");
//		pin = 60;
//	}
//	i2c_accel_info->irq = pin;
	return NULL;
//	return &i2c_accel_info;
}

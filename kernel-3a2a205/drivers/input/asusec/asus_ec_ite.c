/*
 * drivers/i2c/chips/asusec.c
 *
 * Copyright (C) 2011 ITE Inc.
 *
 * Written by Donald Huang <donald.huang@ite.com.tw>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation (version 2 of the License only).
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * v1.1 2011.06.29 Donald use i2c_smbus_read_block_data instead of
 *                 i2c_smbus_read_i2c_block_data
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/input.h>
//#include <linux/leds.h>
#include <linux/earlysuspend.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <asm/gpio.h>
#include <linux/irq.h>

#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <asm/ioctl.h>
#include <linux/switch.h>
#include <linux/proc_fs.h>
#include <linux/wakelock.h>

#define __USING_I2C_SMBUS_RW_I2C_BLOCK_DATA__

#include "asus_ec_ite.h"
#include <linux/ite8566.h>
#include <asm/unaligned.h>

#include <linux/workqueue.h>

#define DRIVER_VERSION	"0.0.2"		//Joe modify 

u8 gBuffer[20];
u8 gFlash[3];
u8 gRepeat;
u32 irq_count1,irq_count2;

enum hid_i2c_command {
	ITE_HID_DESCR_LENGTH_CMD,
	ITE_HID_DESCR_CMD,
	ITE_HID_REPORT_DESCR_CMD,
	ITE_HID_INPUT_REPORT_CMD,
	ITE_HID_OUTPUT_REPORT_CMD,
	ITE_HID_POWER_ON_CMD,
	ITE_HID_RESET_CMD,
	ITE_HID_DATA_CMD,
	ITE_HID_PAD_BATTERY_CMD,
	ITE_HID_DOCK_BATTERY_CMD
};

#define ITE_I2C_COMMAND_TRIES	3
#define ITE_I2C_COMMAND_DELAY	50
#define ITE_REPORT_MAX_LENGTH	256
#define HID_DES_LENGTH_OFFSET	30
#define HID_DES_LENGTH_OFFSET	30
#define ITE_MAX_INPUT_LENGTH	12

static const u8 ite_hid_descriptor_cmd[] = {0xf1, 0x00};
static const u8 ite_hid_report_descr_cmd[] = {0xf2, 0x00};
static const u8 ite_hid_input_report_cmd[] = {0xf3, 0x00};
static const u8 ite_hid_output_report_cmd[] = {0xf4, 0x00};
static const u8 ite_hid_power_on_cmd[] = {0xf5, 0x00, 0x00, 0x08};
static const u8 ite_hid_reset_cmd[] = {0xf5, 0x00, 0x00, 0x01};
static const u8 ite_hid_data_cmd[] = {0xf6, 0x00};
static const u8 ite_hid_pad_battery_cmd[] = {0xF5, 0x00, 0x1C, 0x02, 0xF6, 0x00};
static const u8 ite_hid_dock_battery_cmd[] = {0xF5, 0x00, 0x1D, 0x02, 0xF6, 0x00};

static const unsigned char i2c_kbd_keycode[256] = {
          0,  0,  0,  0, 30, 48, 46, 32, 18, 33, 34, 35, 23, 36, 37, 38,
         50, 49, 24, 25, 16, 19, 31, 20, 22, 47, 17, 45, 21, 44,  2,  3,
          4,  5,  6,  7,  8,  9, 10, 11, 28,  1, 14, 15, 57, 12, 13, 26,
         27, 43, 43, 39, 40, 41, 51, 52, 53, 58, 59, 60, 61, 62, 63, 64,
         65, 66, 67, 68, 87, 88, 99, 70,119,110,102,104,111,107,109,106,
        105,108,103, 69, 98, 55, 74, 78, 96, 79, 80, 81, 75, 76, 77, 71,
         72, 73, 82, 83, 86,127,116,117,183,184,185,186,187,188,189,190,
        191,192,193,194,134,138,130,132,128,129,131,137,133,135,136,113,
        115,114,  0,  0,  0,121,  0, 89, 93,124, 92, 94, 95,  0,  0,  0,
        122,123, 90, 91, 85,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
          0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
          0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
          0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
          0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
         29, 42, 56,125, 97, 54,100,126,164,166,165,163,161,115,114,113,
        150,158,159,128,136,177,178,176,142,152,173,140
};

static uint8_t i2c_command = 0;

static ssize_t ite8566_show_disable(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t ite8566_set_disable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t ite8566_register_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t ite8566_register_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static DEVICE_ATTR(disable_kp, (S_IWUSR|S_IRUGO), ite8566_show_disable, ite8566_set_disable);
static DEVICE_ATTR(register, (S_IWUSR|S_IRUGO), ite8566_register_show, ite8566_register_store);

static struct attribute *ite8566_attr[] = {
	&dev_attr_register.attr,
	&dev_attr_disable_kp.attr,
	NULL
};

/*
 * The signals are AT-style: the low 7 bits are the keycode, and the top
 * bit indicates the state (1 for down, 0 for up).
 */
static inline u8 ite8566_whichkey(u8 event)
{
	return event & 0x7f;
}

static inline int ite8566_ispress(u8 event)
{
	return (event & 0x80) ? 0 : 1;
}

void hexdump(u8 *a,int n)
{
	int i,j;
	printk("%d bytes\n",n);
	
	for ( j=0; j<n; j+=16 ) {
		printk("%03x: ",j);
		for ( i=j; i<j+16; i++ ) {
			if((i%16)==0x8) {
				printk(" - ");
			}
			if ( i<n ) {
				printk("%02x ", a[i]&255 );
			} else {
				printk("   ");
			}
		}
		//printk("   ");
		printk("\n");
	} /* next j */
}

ssize_t ite8566_i2c_ioctl(struct inode *inode, struct file *filp,
unsigned int cmd, unsigned long arg)
{
	printk("[ITE]: %s: ite_i2c_ioctl:cmd=%x \n", __func__, cmd);
//	u8 *parg=arg;
//	int delay=0;
	
	switch (cmd)
	{
		case IOCTL_CHKREADY:
		case IOCTL_RESET:
		case IOCTL_WRITE:
		case IOCTL_PREREAD:
		case IOCTL_READ:
		case IOCTL_INITFW:
		case IOCTL_FINALIZEFW:
		case IOCTL_ITE_TEST:
		default:
			break;
	}
	return 0;
}

struct file_operations i2c_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = ite8566_i2c_ioctl,
	//open: ite_i2c_open2,
	//write: ite_i2c_write,
	//read: ite_i2c_read,
	//release: ite_i2c_release,
};

//XXXXXXXXXXXXXXX   i2c fops end

//check command type
u8 ite8566_checkCT(u8 cmd)
{
	return (cmd&0xf);
}

//check event type
u8 ite8566_checkET(u8 cmd)
{
	return (cmd&0xf);
}

void handle_keyboard(struct ite_chip *lm ,u8 *key_fifo)
{
	u8 event=0;
	u8 tflags=0;
		
	printk("[ITE]: %s \n",__func__);
	
	if(key_fifo[1]==0xE0) {
		tflags=1;
		//for 2 bytes scancode
		switch(key_fifo[2]) {
			case 0x48:
			case 0xc8:
				event = KEY_UP;
				break;
			case 0x4B:
			case 0xCB:
				event = KEY_LEFT;
				break;
			case 0x50:
			case 0xD0:
				event = KEY_DOWN;
				break;
			case 0x4D:
			case 0xCD:
				event = KEY_RIGHT;
				break;
			
			default:
				printk("[ITE]: Keymap no such define for scan code 0xE0 0x02%x \n",key_fifo[2]);	
		}
		
	} else {
		event=key_fifo[1];
	}	

	//if ((event = key_fifo[i++])) {
	if (1) {
		u8 key = ite8566_whichkey(event);
		u8 isdown = ite8566_ispress(event);
		//unsigned short keycode = lm->keymap[key];
		
		if(tflags) {
			isdown=ite8566_ispress(key_fifo[2]);
		}
		
		//dprintk("keycode 0x%02x key 0x%02x %s\n",
		//         keycode, key, isdown ? "down" : "up");
		
		//dprintk(" enabled=%x repeat_key=%x",lm->kp_enabled,lm->idev->repeat_key);
		if (ite_chip_data->kp_enabled) {
			ite_chip_data->input_dev->keybit[BIT_WORD(key)] = BIT_MASK(key);
		
			if(ite_chip_data->input_dev->repeat_key==key) {
				input_report_key(ite_chip_data->input_dev, key, 0);
			}
			input_report_key(ite_chip_data->input_dev, key, isdown);
			input_sync(ite_chip_data->input_dev);
			ite_chip_data->input_dev->repeat_key=key;
		}
		if (isdown)
			ite_chip_data->keys_down++;
		else
			ite_chip_data->keys_down--;
	}
}

void press_key(struct ite_chip *lm ,u8 key)
{
	u8 isdown = 0;//ite_ispress(event);
//	u8 p_key = ite_chip_data->input_dev->repeat_key;
	
	printk("[ITE]: %s:kb2:key=%x", __func__, key);
	
	//isdown = 0;
	//lm->idev->keybit[BIT_WORD(p_key)] = BIT_MASK(p_key);
	//input_report_key(lm->idev, p_key, isdown);
	//input_sync(lm->idev);
	
	
	isdown = 1;
	ite_chip_data->input_dev->keybit[BIT_WORD(key)] = BIT_MASK(key);
	input_event(ite_chip_data->input_dev,EV_MSC,MSC_SCAN,key);
	input_report_key(ite_chip_data->input_dev, key, isdown);
	input_sync(ite_chip_data->input_dev);
	ite_chip_data->input_dev->repeat_key=key;
	
	isdown = 0;
	ite_chip_data->input_dev->keybit[BIT_WORD(key)] = BIT_MASK(key);
	input_event(ite_chip_data->input_dev,EV_MSC,MSC_SCAN,key);
	input_report_key(ite_chip_data->input_dev, key, isdown);
	input_sync(ite_chip_data->input_dev);
	ite_chip_data->input_dev->repeat_key=key;
/*
        if(gRepeat != key) {
        	isdown = 0;
        }
        lm->idev->keybit[BIT_WORD(key)] = BIT_MASK(key);
        input_report_key(lm->idev, key, isdown);
        input_sync(lm->idev);
*/
}

void handle_keyboard2(struct ite_chip *lm ,u8 *key_fifo)
{
	u8 keycode1 = 0x00;
	u8 keycode2 = 0x00;
	
//	printk("[ITE]: key_fifo[3] = 0x%x, key_fifo[5] = 0x%x. \n", key_fifo[3], key_fifo[5]);
	
	if(key_fifo[3] == ASUSDEC_KEY_RELASE)
	{	//Modifier Keys release
		ite_chip_data->keypad_data.value = 0;
//		printk("[ITE]: %d \n", __LINE__);
		input_report_key(ite_chip_data->input_dev, ite_chip_data->modifier_repeat_key, ite_chip_data->keypad_data.value);
		input_sync(ite_chip_data->input_dev);
		ite_chip_data->modifier_repeat_key = KEY_RESERVED;
		
	}else{
		//Modifier Keys press
		ite_chip_data->keypad_data.value = 1;
//		printk("[ITE]: %d \n", __LINE__);
		
		if(key_fifo[3] == ASUSDEC_KEY_LEFT_CTRL)
			ite_chip_data->keypad_data.input_keycode = KEY_LEFTCTRL;
		else if(key_fifo[3] == ASUSDEC_KEY_LEFT_SHITE)
			ite_chip_data->keypad_data.input_keycode = KEY_LEFTSHIFT;
		else if(key_fifo[3] == ASUSDEC_KEY_LEFT_ALT)
			ite_chip_data->keypad_data.input_keycode = KEY_LEFTALT;
		else if(key_fifo[3] == ASUSDEC_KEY_LEFT_WINDOWS)
			ite_chip_data->keypad_data.input_keycode = KEY_HOMEPAGE;
		else if(key_fifo[3] == ASUSDEC_KEY_RIGHT_CTRL)
			ite_chip_data->keypad_data.input_keycode = KEY_RIGHTCTRL;
		else if(key_fifo[3] == ASUSDEC_KEY_RIGHT_SHITE)
			ite_chip_data->keypad_data.input_keycode = KEY_RIGHTSHIFT;
		else if(key_fifo[3] == ASUSDEC_KEY_RIGHT_ALT)
			ite_chip_data->keypad_data.input_keycode = KEY_RIGHTALT;
		else
			ite_chip_data->keypad_data.input_keycode = KEY_RESERVED;
		
		ite_chip_data->modifier_repeat_key = ite_chip_data->keypad_data.input_keycode;
		input_report_key(ite_chip_data->input_dev, ite_chip_data->keypad_data.input_keycode, ite_chip_data->keypad_data.value);
		input_sync(ite_chip_data->input_dev);
	}
	
	if(key_fifo[5] == ASUSDEC_KEY_RELASE)
	{ // the data is a break signal
		ite_chip_data->keypad_data.value = 0;
//		printk("[ITE]: %d \n", __LINE__);
		
	}else{
		ite_chip_data->keypad_data.value = 1;
//		printk("[ITE]: %d \n", __LINE__);
	}
		
	ite_chip_data->keypad_data.input_keycode = i2c_kbd_keycode[key_fifo[5]];
	
	
//	printk("[ITE]: keycode1 = 0x%x, %d. \n", keycode1, keycode1);
	
	if (ite_chip_data->kp_enabled) {
		
		if(key_fifo[5] == ASUSDEC_KEY_RELASE)
		{
			input_report_key(ite_chip_data->input_dev, ite_chip_data->input_dev->repeat_key, ite_chip_data->keypad_data.value);
			input_sync(ite_chip_data->input_dev);
		}
		else
		{
			input_report_key(ite_chip_data->input_dev, ite_chip_data->keypad_data.input_keycode, ite_chip_data->keypad_data.value);
			ite_chip_data->input_dev->repeat_key = ite_chip_data->keypad_data.input_keycode;
			input_sync(ite_chip_data->input_dev);
		}
	}
	
//	printk("[ITE]: %d\n",__LINE__);
	
}

void handle_keyboard_function_1(struct ite_chip *lm ,u8 *key_fifo)
{
	u8 key = key_fifo[3];
		
//	printk("[ITE]: %s ++ \n",__func__);
	if(key_fifo[3] == ASUSDEC_KEY_RELASE){
		ite_chip_data->keypad_data.value = 0;	
//		printk("[ITE]: %d\n",__LINE__);
	}else{
		ite_chip_data->keypad_data.value = 1;
//		printk("[ITE]: %d\n",__LINE__);
	}

//	printk("[ITE]: key_fifo[3] = 0x%x. \n", key_fifo[3]);
	
	if(key_fifo[3] == 0x10)
		ite_chip_data->keypad_data.input_keycode = ASUSDEC_KEY_LCDOFF;
	else if(key_fifo[3] == 0x12)
		ite_chip_data->keypad_data.input_keycode = ASUSDEC_KEY_TOUCHPAD;
	else if(key_fifo[3] == 0x18)
		ite_chip_data->keypad_data.input_keycode = ASUSDEC_KEY_POWER_4GEAR;
	else if(key_fifo[3] == 0x6F)
		ite_chip_data->keypad_data.input_keycode = KEY_BRIGHTNESSUP;
	else if(key_fifo[3] == 0x70)
		ite_chip_data->keypad_data.input_keycode = KEY_BRIGHTNESSDOWN;
	else if(key_fifo[3] == 0xB5)
		ite_chip_data->keypad_data.input_keycode = KEY_NEXTSONG;
	else if(key_fifo[3] == 0xB6)
		ite_chip_data->keypad_data.input_keycode = KEY_PREVIOUSSONG;
	else if(key_fifo[3] == 0xB7)
		ite_chip_data->keypad_data.input_keycode = KEY_STOPCD;
	else if(key_fifo[3] == 0xCD)
		ite_chip_data->keypad_data.input_keycode = KEY_PLAYPAUSE;
	else if(key_fifo[3] == 0xE2)
		ite_chip_data->keypad_data.input_keycode = KEY_MUTE;
	else if(key_fifo[3] == 0xE9)
		ite_chip_data->keypad_data.input_keycode = KEY_VOLUMEUP;
	else if(key_fifo[3] == 0xEA)
		ite_chip_data->keypad_data.input_keycode = KEY_VOLUMEDOWN;
	else
		ite_chip_data->keypad_data.input_keycode = KEY_RESERVED;
		
//	printk("[ITE]: input_keycode = 0x%x. \n",ite_chip_data->keypad_data.input_keycode);
	
	if(ite_chip_data->kp_enabled)
	{	
		input_report_key(ite_chip_data->input_dev, ite_chip_data->keypad_data.input_keycode, 1);
		input_sync(ite_chip_data->input_dev); 
		input_report_key(ite_chip_data->input_dev, ite_chip_data->keypad_data.input_keycode, 0);
		input_sync(ite_chip_data->input_dev); 
	}
	
//	printk("[ITE]: %s -- \n",__func__);
}


void handle_keyboard_function_2(struct ite_chip *lm ,u8 *key_fifo)
{
	u8 key = key_fifo[3];
		
//	printk("[ITE]: %s ++ \n",__func__);
	if(key_fifo[3] == ASUSDEC_KEY_RELASE){
		ite_chip_data->keypad_data.value = 0;	
//		printk("[ITE]: %d\n",__LINE__);
	}else{
		ite_chip_data->keypad_data.value = 1;
//		printk("[ITE]: %d\n",__LINE__);
	}

//	printk("[ITE]: key_fifo[3] = 0x%x. \n", key_fifo[3]);
	
	if(key_fifo[3] == 0x01)
		ite_chip_data->keypad_data.input_keycode = KEY_WLAN;
	else
		ite_chip_data->keypad_data.input_keycode = KEY_RESERVED;
		
//	printk("[ITE]: input_keycode = 0x%x. \n",ite_chip_data->keypad_data.input_keycode);
	
	if(ite_chip_data->kp_enabled)
	{	
		input_report_key(ite_chip_data->input_dev, ite_chip_data->keypad_data.input_keycode, 1);
		input_sync(ite_chip_data->input_dev); 
		input_report_key(ite_chip_data->input_dev, ite_chip_data->keypad_data.input_keycode, 0);
		input_sync(ite_chip_data->input_dev); 
	}
	
//	printk("[ITE]: %s -- \n",__func__);
}

void handle_keyboard_function_3(struct ite_chip *lm ,u8 *key_fifo)
{
	u8 key = key_fifo[3];
		
//	printk("[ITE]: %s ++ \n",__func__);
	if(key_fifo[3] == ASUSDEC_KEY_RELASE){
		ite_chip_data->keypad_data.value = 0;	
//		printk("[ITE]: %d\n",__LINE__);
	}else{
		ite_chip_data->keypad_data.value = 1;
//		printk("[ITE]: %d\n",__LINE__);
	}

//	printk("[ITE]: key_fifo[3] = 0x%x. \n", key_fifo[3]);
	
	if(key_fifo[3] == 0x82)
		ite_chip_data->keypad_data.input_keycode = KEY_SLEEP;
	else if(key_fifo[3] == 0xA1)
		ite_chip_data->keypad_data.input_keycode = KEY_CAMERA;
	else if(key_fifo[3] == 0xB5)
	{
//		printk("[ITE]: LCD & CRT change \n");
		ite_chip_data->keypad_data.input_keycode = KEY_RESERVED;
	}
	else
		ite_chip_data->keypad_data.input_keycode = KEY_RESERVED;
	
//	printk("[ITE]: input_keycode = 0x%x. \n",ite_chip_data->keypad_data.input_keycode);
	
	if(ite_chip_data->kp_enabled)
	{	
		input_report_key(ite_chip_data->input_dev, ite_chip_data->keypad_data.input_keycode, 1);
		input_sync(ite_chip_data->input_dev); 
		input_report_key(ite_chip_data->input_dev, ite_chip_data->keypad_data.input_keycode, 0);
		input_sync(ite_chip_data->input_dev); 
	}
	
//	printk("[ITE]: %s -- \n",__func__);
}

void handle_touchpad2(struct ite_chip *lm ,u8 *key_fifo)
{
	//int x_sign=0,y_sign=0,x_ov,y_ov,
	int left=0,right=0;
	signed char xp,yp;
	xp=key_fifo[4];
	yp=key_fifo[5];
	
	if(key_fifo[3]&0x01) {
		left=1;
		//input_report_key(lm->idev, BTN_LEFT, left);
	}
	
	if(key_fifo[3]&0x02) {
		right=1;
		//input_report_key(lm->idev, BTN_LEFT, left);
	}	
	
	//if((xp & yp & key_fifo[3]) !=0 ) {
	//input_report_key(lm->idev, BTN_TOUCH, 1);//20110901
	//input_report_key(lm->idev, BTN_TOUCH, 0);//20110901
	//hexdump(key_fifo,6);
	//dprintk("\n\r fifo [3]=%x [4]=%x [5]=%x ",key_fifo[3],key_fifo[4],key_fifo[5]);
	//dprintk("\n\r xp=%x yp=%x left=%x lm->idev=%x ",xp,yp,left,lm->idev);
	//input_report_key(lm->idev, BTN_MIDDLE, 0);//20110901
	//input_report_key(lm->idev, BTN_RIGHT, 0);//20110901
//Joe	printk("[ITE]: touchpad2:evbit=%x keybit=%x \n",ite_chip_data->input_dev->evbit[0],ite_chip_data->input_dev->keybit[0]);
	//if((xp|yp|left)) {
	input_report_rel(ite_chip_data->input_dev, REL_X, xp);
	input_report_rel(ite_chip_data->input_dev, REL_Y, yp);
	input_report_key(ite_chip_data->input_dev, BTN_LEFT, left);
	input_report_key(ite_chip_data->input_dev, BTN_MIDDLE, 0);
	input_report_key(ite_chip_data->input_dev, BTN_RIGHT,  right);
	input_sync(ite_chip_data->input_dev);
	//}
	//}
}

/*
 * We cannot use I2C in interrupt context, so we just schedule work.
 */
static irqreturn_t ite8566_irq(int irq, void *data)
{
//	struct ite_chip *ite_chip_data=data;
//	printk("[ITE]: %s ++ \n",__func__);
	schedule_work(&ite_chip_data->work);
	return IRQ_HANDLED;
}

//Joe static irqreturn_t ite_irq_level(int irq, void *data)
//Joe {
//Joe 	unsigned char buffer[32];
//Joe //	unsigned long flags;
//Joe 	//disable_irq(IRQ_EINT(18));
//Joe 	printk("[ITE]: %s ++ \n",__func__);
//Joe 	//ite_i2c_reschedule_work(ite_chip_data,0);
//Joe 	//spin_lock_irqsave(&ite_chip_data->lock2, flags);
//Joe 	ite_readbuf(ite_chip_data->client,buffer);
//Joe 	//spin_unlock_irqrestore(&ite_chip_data->lock2, flags);
//Joe 	//enable_irq(IRQ_EINT(18));
//Joe 	return IRQ_HANDLED;
//Joe }

static int ite8566_readbuf(struct i2c_client *client,u8 *buf)
{
	int ret;
	struct i2c_msg msg[1];
//	u8 buf1[15];
//	u8 buf2[15];
	
	//printk("\n\rite_readbuf:\n\r");
	msg[0].addr = ite_chip_data->client->addr;
	msg[0].flags = I2C_M_RD | I2C_M_RECV_LEN;
	msg[0].len = ITE_MAX_INPUT_LENGTH;
	msg[0].buf = buf;
	
	ret=i2c_transfer(client->adapter,msg,1);
//	hexdump(buf, ITE_MAX_INPUT_LENGTH);
	
	return 0;
}

static void ite8566_work(struct work_struct *work)
{
	u8 i2c_data[32];
	
//	printk("[ITE]: %s ++ \n",__func__);
	
	ite8566_readbuf(ite_chip_data->client,i2c_data);
	
//	if(buffer[0]==0x6) {
//		handle_touchpad2(ite_chip_data,buffer);
//	} else if(buffer[0]==0xB) {
//		handle_keyboard2(ite_chip_data,buffer);
//	} else {
//		printk("[ITE]: NO SUCH EVENT!! buffer[0]=%x \n",buffer[0]);
//	}
	
	if(i2c_data[2] == ASUSDEC_ID_KEYBOARD)
	{
		handle_keyboard2(ite_chip_data, i2c_data);
	} 
	else if(i2c_data[2] == ASUSDEC_ID_FUNCTION_1)
	{
		handle_keyboard_function_1(ite_chip_data, i2c_data);
	}
	else if(i2c_data[2] == ASUSDEC_ID_FUNCTION_2)
	{
		handle_keyboard_function_2(ite_chip_data, i2c_data);
	}
	else if(i2c_data[2] == ASUSDEC_ID_FUNCTION_3)
	{
		handle_keyboard_function_3(ite_chip_data, i2c_data);
	}
	else
	{
		printk("[ITE]: NO SUCH EVENT!! buffer[2]=0x%x \n", i2c_data[2]);
	}	
}

static struct i2c_driver ite_i2c_driver;

static ssize_t ite8566_show_disable(struct device *dev, struct device_attribute *attr, char *buf)
{
//	struct ite_chip *lm = dev_get_drvdata(dev);
	
	return sprintf(buf, "%u\n", !ite_chip_data->kp_enabled);
}

static ssize_t ite8566_set_disable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
//	struct ite_chip *lm = dev_get_drvdata(dev);
	int ret;
	unsigned long i;
	
	ret = strict_strtoul(buf, 10, &i);
	
	mutex_lock(&ite_chip_data->mutex_lock);
	ite_chip_data->kp_enabled = !i;
	mutex_unlock(&ite_chip_data->mutex_lock);
	
	return count;
}


void getHIDDescriptor(struct i2c_client *client)
{
	int ret;
	struct i2c_msg msg[2];
	u8 buf1[2]={0xf1,0x00};
	u8 buf2[30];
	int i;
	
	printk("[ITE]: %s addr:0x%x ++ \n",__func__, ite_chip_data->client->addr);
	
	msg[0].addr = ite_chip_data->client->addr;
	msg[0].flags = 0; //I2C_M_WR;
	msg[0].len = 2;
	msg[0].buf = &buf1[0];
	
	msg[1].addr = ite_chip_data->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = &buf2[0];
	msg[1].len = 30;
	
	ret=i2c_transfer(client->adapter,msg,2);
	
	for (i = 0; i < 30; i++)
	{
		printk("[ITE]: buf[%d]=0x%x \n", i, buf2[i]);
	}
	hexdump(buf2,30);	
}

void getReportDescriptor(struct i2c_client *client)
{
	int ret;
	struct i2c_msg msg[2];
	u8 buf1[2]={0xf2,0x00};
	u8 buf2[117];
	
	printk("[ITE]: %s addr:0x%x ++ \n",__func__, ite_chip_data->client->addr);
	
	msg[0].addr = ite_chip_data->client->addr;
	msg[0].flags = 0; //I2C_M_WR;
	msg[0].len = 2;
	msg[0].buf = &buf1[0];
	
	msg[1].addr = ite_chip_data->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = &buf2[0];
	msg[1].len = 117;
	
	ret=i2c_transfer(client->adapter,msg,2);
	hexdump(buf2,117);
}

void sendCmd(struct i2c_client *client,u8 *cmd,u8 rw)
{
	int ret;
	int len = cmd[1]+1;
	
	
	printk("[ITE]: %s: sendCmd=====> len=%x \n", __func__, len);
	//hexdump(cmd[1],len);	
	//test_flag =1;
	
	if(rw==0) {
		ret = i2c_smbus_write_i2c_block_data(client,cmd[0],len,&cmd[1]);
	} else {
	}
	
	if (ret < 0) {
		printk("[ITE]: failed writing the cmd=%x subcmd=%x \n",cmd[0],cmd[2]);
		//return ret;
	} else {
		printk("[ITE]: success writing the cmd=%x subcmd=%x \n",cmd[0],cmd[2]);
	}
}

int ite8566_i2c_event(struct input_dev *dev, unsigned int type, unsigned int code, int value)
{
	printk("[ITE]: %s ++ \n",__func__);
	
	input_sync(ite_chip_data->input_dev);
	return 0;
}

static int asus_ec_i2c_command(struct i2c_client *client, int  command,
				unsigned char *buf_recv, int data_len)
{
	const u8 *cmd = NULL;
	unsigned char *rec_buf = buf_recv;
	int ret;
	int tries = ITE_I2C_COMMAND_TRIES;
	int length = 0;
	struct i2c_msg msg[2];
	int msg_num = 0;

	switch (command) {
	case ITE_HID_DESCR_LENGTH_CMD:
	case ITE_HID_DESCR_CMD:
		cmd = ite_hid_descriptor_cmd;
		length = sizeof(ite_hid_descriptor_cmd);
		msg[1].len = data_len;
		msg_num = 2;
		break;

	case ITE_HID_REPORT_DESCR_CMD:
		cmd = ite_hid_report_descr_cmd;
		length = sizeof(ite_hid_report_descr_cmd);
		msg[1].len = data_len;
		msg_num = 2;
		break;
		
	case ITE_HID_POWER_ON_CMD:
		cmd = ite_hid_power_on_cmd;
		length = sizeof(ite_hid_power_on_cmd);
		msg_num = 1;
		break;
	
	case ITE_HID_RESET_CMD:
		cmd = ite_hid_reset_cmd;
		length = sizeof(ite_hid_reset_cmd);
		msg_num = 1;
		break;
	
	case ITE_HID_PAD_BATTERY_CMD:
		cmd = ite_hid_pad_battery_cmd;
		length = sizeof(ite_hid_pad_battery_cmd);
		msg[1].len = data_len;
		msg_num = 2;
		break;
		
	case ITE_HID_DOCK_BATTERY_CMD:
		cmd = ite_hid_dock_battery_cmd;
		length = sizeof(ite_hid_dock_battery_cmd);
		msg[1].len = data_len;
		msg_num = 2;
		break;
	
	default:
		dev_dbg(&client->dev, "%s command=%d unknow.\n",
			 __func__, command);
		return -1;
	}

	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = length;
	msg[0].buf = (char *) cmd;
	
	msg[1].addr = client->addr;
	msg[1].flags = client->flags & I2C_M_TEN;
	msg[1].flags |= I2C_M_RD;
	msg[1].buf = rec_buf;
	
	do {
		ret = i2c_transfer(client->adapter, msg, msg_num);
		
		if (ret > 0)
			break;
		tries--;
		printk("[ITE]: retrying elantech_i2c_command: %d (%d) \n", command, tries);
		
	} while (tries > 0);
	
	switch (command) {
		case ITE_HID_DESCR_LENGTH_CMD:
		case ITE_HID_DESCR_CMD:
		case ITE_HID_REPORT_DESCR_CMD:
		case ITE_HID_PAD_BATTERY_CMD:
		case ITE_HID_DOCK_BATTERY_CMD:
			hexdump(rec_buf, msg[1].len);
			break;
		default:
			break;
	}

	return ret;
}

/*
 * Return the battery average current
 * Or < 0 if something fails.
 */
//static int asusec_battery_current(struct ite_chip *id)
//{
//	sendCmd(ite_chip_data->client , ITE_CMD_BATTERY_AVG_CURRENT , CMD_W);
//	printk("[ITE]: asusec_battery_current=> %x, %x \n",ite_chip_data->current_uA , id->current_uA);
//	return ite_chip_data->current_uA;
//}

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
//Joe static int asusec_battery_voltage(struct ite_chip *id)
//Joe {
//Joe 	sendCmd(ite_chip_data->client , ITE_CMD_BATTERY_VOL , CMD_W);
//Joe 	printk("[ITE]: asusec_battery_voltage=> %x, %x \n",ite_chip_data->voltage_uV , id->voltage_uV);
//Joe 	return ite_chip_data->voltage_uV;
//Joe }

/*
 * Return the battery temperature in Celcius degrees
 * Or < 0 if something fails.
 */
//static int asusec_battery_temperature(struct ite_chip *id)
//{
//	sendCmd(ite_chip_data->client , ITE_CMD_BATTERY_TEMP, CMD_W);
//	printk("[ITE]: asusec_battery_temperature=> %x, %x\n",ite_chip_data->temp_C , id->temp_C);
//	return ite_chip_data->temp_C;
//}

/*
 * Return the battery Relative State-of-Charge
 * Or < 0 if something fails.
 */
//static int asusec_battery_rsoc(struct ite_chip *id)
//{
////	int ret;
////	int rsoc = 0;
//	
//	sendCmd(ite_chip_data->client , ITE_CMD_BATTERY_RSOC, CMD_W);
//	printk("[ITE]: asusec_battery_rsoc=> %x, %x \n",ite_chip_data->charge_rsoc , id->charge_rsoc);
//	return ite_chip_data->charge_rsoc >> 8;
//}

//#define to_asusec_device_info(x) container_of((x), struct ite_chip, bat);
//static int ite8566_i2c_read(struct i2c_client *client, const u8 command, unsigned char *buf_recv, int msg_num, int data_len)
//{
//	struct i2c_msg msg[2];
//	unsigned char *rec_buf = buf_recv;
//	int tries = ITE_I2C_COMMAND_TRIES;
//	int ret;
//	
//	msg[0].addr = client->addr;
//	msg[0].flags = client->flags & I2C_M_TEN;
//	msg[0].len = sizeof(command);
//	msg[0].buf = (char *) command;
//	
//	if(msg_num == 2)
//	{
//		msg[1].addr = client->addr;
//		msg[1].flags = client->flags & I2C_M_TEN;
//		msg[1].flags |= I2C_M_RD;
//		msg[1].len = data_len;
//		msg[1].buf = rec_buf;
//	}
//	
//	do {
//		ret = i2c_transfer(client->adapter, msg, msg_num);
//		
//		if (ret > 0)
//			break;
//		tries--;
//		printk("[ITE]: retrying ite8566_i2c_read: %d (%d) \n", command, tries);
//		
//	} while (tries > 0);
//}
 
 
static int ite8566_read_pad_battery_info(int *bat_status, int *bat_temp, int *bat_vol, int *bat_current, int *bat_capacity, int *bat_energy)
{
	int ret_val = 0;
	int rc;
	struct i2c_msg msg[2];
	const u8 *cmd = NULL;
	int length = 0;
	u8 buf_recv[14];
		
	printk("[ITE]: %s \n", __func__);
		
//	cmd = ite_hid_pad_battery_cmd;
//	length = sizeof(ite_hid_pad_battery_cmd);
//		
//	msg[0].addr = ite_chip_data->client->addr;
//	msg[0].flags = ite_chip_data->client->flags & I2C_M_TEN;
//	msg[0].len = length;
//	msg[0].buf = (char *) cmd;
//	
//	msg[1].addr = ite_chip_data->client->addr;
//	msg[1].flags = ite_chip_data->client->flags & I2C_M_TEN;
//	msg[1].flags |= I2C_M_RD;
//	msg[1].buf = &recbuf[0];
//	msg[1].len = 0x0E;
	
//	ret = i2c_transfer(ite_chip_data->client->adapter, msg, 2);

//	hexdump(recbuf,0x0E);

	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_PAD_BATTERY_CMD, buf_recv, 14);
	if (rc != 2)
	{
		printk(KERN_ERR "[ITE] %s:[%d]: Get ITE_HID_PAD_BATTERY_CMD failed \n", __func__, __LINE__);
		return false;
	}
	else {
		
		*bat_status = (buf_recv[3] << 8 ) | buf_recv[2];
		*bat_temp = (buf_recv[5] << 8 )| buf_recv[4];
		*bat_vol = (buf_recv[7] << 8 )| buf_recv[6];
		*bat_current = (buf_recv[9] << 8 )| buf_recv[8];
		*bat_capacity = (buf_recv[11] << 8 )| buf_recv[10];
		*bat_energy = (buf_recv[13] << 8 )| buf_recv[12];
	//	*bat_remaining_capacity = (buf_recv[16] << 8 )| buf_recv[15];
	//	*bat_avg_time_to_empty = (recbuf[13] << 8 )| recbuf[12];
	//	*bat_avg_time_to_full = (buf_recv[20] << 8 )| buf_recv[19];
		return 0;
	}
}
EXPORT_SYMBOL(ite8566_read_pad_battery_info);


static int ite8566_read_dock_battery_info(int *bat_status, int *bat_temp, int *bat_vol, int *bat_current, int *bat_capacity, int *bat_energy)
{
	int ret_val = 0;
	int rc;
	struct i2c_msg msg[2];
	const u8 *cmd = NULL;
	int length = 0;
	u8 buf_recv[14];
		
	printk("[ITE]: %s \n", __func__);
		
//	cmd = ite_hid_dock_battery_cmd;
//	length = sizeof(ite_hid_dock_battery_cmd);
//		
//	msg[0].addr = ite_chip_data->client->addr;
//	msg[0].flags = ite_chip_data->client->flags & I2C_M_TEN;
//	msg[0].len = length;
//	msg[0].buf = (char *) cmd;
//	
//	msg[1].addr = ite_chip_data->client->addr;
//	msg[1].flags = ite_chip_data->client->flags & I2C_M_TEN;
//	msg[1].flags |= I2C_M_RD;
//	msg[1].buf = &recbuf[0];
//	msg[1].len = 0x0E;
//	
//	ret = i2c_transfer(ite_chip_data->client->adapter, msg, 2);
//
//	hexdump(recbuf,0x0E);	

	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_DOCK_BATTERY_CMD, buf_recv, 14);
	if (rc != 2)
	{
		printk(KERN_ERR "[ITE] %s:[%d]: Get ITE_HID_DOCK_BATTERY_CMD failed \n", __func__, __LINE__);
		return false;
	} else {
		
		*bat_status = (buf_recv[3] << 8 ) | buf_recv[2];
		*bat_temp = (buf_recv[5] << 8 )| buf_recv[4];
		*bat_vol = (buf_recv[7] << 8 )| buf_recv[6];
		*bat_current = (buf_recv[9] << 8 )| buf_recv[8];
		*bat_capacity = (buf_recv[11] << 8 )| buf_recv[10];
		*bat_energy = (buf_recv[13] << 8 )| buf_recv[12];
	//	*bat_remaining_capacity = (buf_recv[16] << 8 )| buf_recv[15];
	//	*bat_avg_time_to_empty = (recbuf[13] << 8 )| recbuf[12];
	//	*bat_avg_time_to_full = (buf_recv[20] << 8 )| buf_recv[19];
		return 0;
	}
}
EXPORT_SYMBOL(ite8566_read_dock_battery_info);

static ssize_t ite8566_register_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret = 0;
    int bufsize = 55;
    uint8_t data[55] = { 0 }, loop_i;

    printk(KERN_INFO "[ITE] %s: i2c_command = %x\n", __func__, i2c_command);

    if (i2c_smbus_read_i2c_block_data(ite_chip_data->client, i2c_command, bufsize, &data[0]) < 0) {
        printk(KERN_WARNING "[ITE] %s: read fail\n", __func__);
        return ret;
    }

    ret += sprintf(buf, "command: %x\n", i2c_command);
    for (loop_i = 0; loop_i < bufsize; loop_i++) {
        ret += sprintf(buf + ret, "0x%2.2X ", data[loop_i]);
        if ((loop_i % 16) == 15)
            ret += sprintf(buf + ret, "\n");
    }
    ret += sprintf(buf + ret, "\n");
    return ret;
}

static ssize_t ite8566_register_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    char buf_tmp[6], length = 0;
    uint8_t veriLen = 0;
    uint8_t write_da[100];
    unsigned long result = 0;

    memset(buf_tmp, 0x0, sizeof(buf_tmp));
    memset(write_da, 0x0, sizeof(write_da));
	
	printk("[ITE] %s: ++ \n", __func__);
	
    if ((buf[0] == 'r' || buf[0] == 'w') && buf[1] == ':') {
        if (buf[2] == 'x') {
            uint8_t loop_i;
            uint16_t base = 5;
            memcpy(buf_tmp, buf + 3, 2);
            if (!strict_strtoul(buf_tmp, 16, &result))
                i2c_command = result;
            for (loop_i = 0; loop_i < 100; loop_i++) {
                if (buf[base] == '\n') {
                    if (buf[0] == 'w')
                        i2c_smbus_write_i2c_block_data(ite_chip_data->client, i2c_command, length, &write_da[0]);
                    printk(KERN_INFO "CMD: %x, %x, %d\n", i2c_command,
                        write_da[0], length);
                    for (veriLen = 0; veriLen < length; veriLen++)
                    {
                        printk(KERN_INFO "%x ", *((&write_da[0])+veriLen));
                    }

                    printk(KERN_INFO "\n");
                    return count;
                }
                if (buf[base + 1] == 'x') {
                    buf_tmp[4] = '\n';
                    buf_tmp[5] = '\0';
                    memcpy(buf_tmp, buf + base + 2, 2);
                    if (!strict_strtoul(buf_tmp, 16, &result))
                        write_da[loop_i] = result;
                    length++;
                }
                base += 4;
            }
        }
    }
    
    printk("[ITE] %s: -- \n", __func__);
    return count;
}

//Joe static int asusec_battery_get_property(struct power_supply *psy,
//Joe 										enum power_supply_property psp,
//Joe 										union power_supply_propval *val)
//Joe {
//Joe 	struct ite_chip *di = to_asusec_device_info(psy);
//Joe 	
//Joe 	printk("[ITE]: %s ++ \n",__func__);
//Joe 	
//Joe 	switch (psp) {
//Joe /*
//Joe 	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
//Joe 	case POWER_SUPPLY_PROP_PRESENT:
//Joe 		val->intval = asusec_battery_voltage(di);
//Joe 		if (psp == POWER_SUPPLY_PROP_PRESENT)
//Joe 			val->intval = val->intval <= 0 ? 0 : 1;
//Joe 		break;
//Joe */
//Joe 	case POWER_SUPPLY_PROP_CURRENT_NOW:
//Joe 		val->intval = asusec_battery_current(di);
//Joe 		break;
//Joe 		
//Joe 	case POWER_SUPPLY_PROP_CAPACITY:
//Joe 		val->intval = asusec_battery_rsoc(di);
//Joe 		break;
//Joe 		
//Joe 	case POWER_SUPPLY_PROP_TEMP:
//Joe 		val->intval = asusec_battery_temperature(di);
//Joe 		break;
//Joe 	default:
//Joe 		return -EINVAL;
//Joe 	}
//Joe 	
//Joe 	return 0;
//Joe }

//Joe static void asusec_powersupply_init(struct ite_chip *it)
//Joe {
//Joe 	it->bat.type = POWER_SUPPLY_TYPE_BATTERY;
//Joe 	it->bat.properties = asusec_battery_props;
//Joe 	it->bat.num_properties = ARRAY_SIZE(asusec_battery_props);
//Joe 	it->bat.get_property = asusec_battery_get_property;
//Joe 	it->bat.external_power_changed = NULL;
//Joe }

/*
 * asusec specific code
 */

//Joe static int asusec_read(u8 reg, int *rt_value, int b_single,
//Joe 						struct ite_chip *ite_chip_data)
//Joe {
//Joe 	struct i2c_client *client = ite_chip_data->client;
//Joe 	struct i2c_msg msg[1];
//Joe 	unsigned char data[2];
//Joe 	int err;
//Joe 	
//Joe 	printk("[ITE]: %s ++ \n",__func__);
//Joe 	
//Joe 	if (!client->adapter)
//Joe 		return -ENODEV;
//Joe 	
//Joe 	msg->addr = client->addr;
//Joe 	msg->flags = 0;
//Joe 	msg->len = 1;
//Joe 	msg->buf = data;
//Joe 	
//Joe 	data[0] = reg;
//Joe 	err = i2c_transfer(client->adapter, msg, 1);
//Joe 	
//Joe 	if (err >= 0) {
//Joe 		if (!b_single)
//Joe 			msg->len = 2;
//Joe 		else
//Joe 			msg->len = 1;
//Joe 		
//Joe 		msg->flags = I2C_M_RD;
//Joe 		err = i2c_transfer(client->adapter, msg, 1);
//Joe 		if (err >= 0) {
//Joe 			if (!b_single)
//Joe 				*rt_value = get_unaligned_be16(data);
//Joe 			else
//Joe 				*rt_value = data[0];
//Joe 			
//Joe 			return 0;
//Joe 		}
//Joe 	}
//Joe 	return err;
//Joe }

//Joe void do_timer_x(unsigned long data)
//Joe {
//Joe //	struct irq_chip *chip=get_irq_chip(ite_chip_data->client->irq);
//Joe 	
//Joe 	printk("[ITE]: do_timer: %s \n",chip->name);
//Joe 
//Joe 	if(gRepeat) {
//Joe 		press_key(ite_chip_data,i2c_kbd_keycode[gRepeat]);
//Joe 		ite_chip_data->timer.expires = msecs_to_jiffies(50);
//Joe 		add_timer(&ite_chip_data->timer);	
//Joe 	} else {
//Joe 		del_timer(&ite_chip_data->timer);
//Joe 	}
//Joe 	
//Joe 	//ite_chip_data->donald_renew=1;
//Joe 	//ite_i2c_reschedule_work(ite_chip_data,msecs_to_jiffies(125));	
//Joe 	//ite_chip_data->timer.expires = jiffies;
//Joe 	//add_timer(&ite_chip_data->timer);
//Joe 	
//Joe }

//Joe static struct ite_chip *ite_i2c_create(struct i2c_client *client)
//Joe {
//Joe 	int i;
//Joe 	
//Joe 	ite_chip_data = kzalloc(sizeof(struct ite_chip), GFP_KERNEL);
//Joe 	if (!ite_chip_data) {
//Joe 		return NULL;
//Joe 	}
//Joe 	
//Joe 	ite_chip_data->client = client;
//Joe 	
//Joe 	mutex_init(&ite_chip_data->lock);
//Joe 	INIT_WORK(&ite_chip_data->work,ite_work);
//Joe 	//INIT_DELAYED_WORK(&ite_chip_data->dwork, ite_work);
//Joe 	//spin_lock_init(&ite_chip_data->lock2);
//Joe 	
//Joe 	setup_timer(&ite_chip_data->timer,do_timer_x,(unsigned long)ite_chip_data);
//Joe 	ite_chip_data->timer.expires = jiffies + HZ;
//Joe 	add_timer(&ite_chip_data->timer);	
//Joe 	
//Joe 	return ite_chip_data;
//Joe }

static bool ite8566_i2c_hw_init(struct i2c_client *client)
{
	int rc;
	uint8_t buf_recv[ITE_REPORT_MAX_LENGTH];
	int hid_descr_len;
	int hid_report_len;

	/* Fetch the length of HID description */
	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_DESCR_LENGTH_CMD, buf_recv, HID_DES_LENGTH_OFFSET);
	if (rc != 2)
	{
		printk(KERN_ERR "[ITE] %s:[%d]: Get ITE_HID_DESCR_LENGTH_CMD failed \n", __func__, __LINE__);
		return false;
	}
	hid_descr_len = buf_recv[0];
	printk("[ITE] %s:[%d]: Get hid_descr_len = %d \n", __func__, __LINE__, hid_descr_len);
	mdelay(3);
		
	/* Fetch the lenght of HID report description */
	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_DESCR_CMD, buf_recv, hid_descr_len);
	if (rc != 2)
	{
		printk(KERN_ERR "[ITE] %s:[%d]: Get ITE_HID_DESCR_CMD failed \n", __func__, __LINE__);
		return false;
	}
	hid_report_len = (buf_recv[5]<< 2 | buf_recv[4]);
	printk("[ITE] %s:[%d]: Get hid_report_len = %d \n", __func__, __LINE__, hid_report_len );
	mdelay(3);
	
	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_POWER_ON_CMD, buf_recv, 0);
	if (rc != 1)
	{
		printk(KERN_ERR "[ITE] %s:[%d]: Get ITE_HID_POWER_ON_CMD failed \n", __func__, __LINE__);
		return false;
	}
	mdelay(3);
	
	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_RESET_CMD, buf_recv, 0);
	if (rc != 1)
	{
		printk(KERN_ERR "[ITE] %s:[%d]: Get ITE_HID_RESET_CMD failed \n", __func__, __LINE__);
		return false;
	}
	
	msleep(ITE_I2C_COMMAND_DELAY);
	
//	hid_report_len = 32;
	
	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_REPORT_DESCR_CMD, buf_recv, hid_report_len);
	if (rc != 2)
	{
		printk(KERN_ERR "[ITE] %s:[%d]: Get ITE_HID_REPORT_DESCR_CMD failed \n", __func__, __LINE__);
		return false;
	}
	
	msleep(10);
	
	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_PAD_BATTERY_CMD, buf_recv, 14);
	if (rc != 2)
	{
		printk(KERN_ERR "[ITE] %s:[%d]: Get ITE_HID_PAD_BATTERY_CMD failed \n", __func__, __LINE__);
		return false;
	}
	
//	msleep(1000);
//	
//	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_DOCK_BATTERY_CMD, buf_recv, 14);
//	if (rc != 2)
//	{
//		printk(KERN_ERR "[ITE] %s:[%d]: Get ITE_HID_DOCK_BATTERY_CMD failed \n", __func__, __LINE__);
//		return false;
//	}

	return true;
}

static int __devinit ite8566_probe(struct i2c_client *client,
								const struct i2c_device_id *id)
{
	struct asus_ec_i2c_platform_data *pdata;
	int err = 0, i;
	int major;
	
	printk("[ITE]: %s ++ \n",__func__);
	
	irq_count1=irq_count2=gRepeat =0;
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "[ITE] %s:[%d]: i2c check functionality error.\n", __func__, __LINE__);
		err = -ENODEV;
		goto probe_err_check_functionality_failed;
	}

	ite_chip_data = kzalloc(sizeof(struct ite_chip), GFP_KERNEL);
	if (ite_chip_data == NULL) {
		printk(KERN_ERR "[ITE] %s:[%d]: allocate ite_chip failed.\n", __func__, __LINE__);
		err = -ENOMEM;
		goto probe_err_alloc_data_failed;
	}

	ite_chip_data->asusec_wq = create_singlethread_workqueue("asusec_wq");
	if (!ite_chip_data->asusec_wq) {
		printk(KERN_ERR "[ITE] %s:[%d]: create workqueue failed.\n", __func__, __LINE__);
		err = -ENOMEM;
		goto probe_err_create_wq_failed;
	}
	
	ite_chip_data->client = client;
	i2c_set_clientdata(client, ite_chip_data);
	pdata = client->dev.platform_data;
	if (likely(pdata != NULL)) {
		ite_chip_data->int_gpio = pdata->int_gpio;
		printk("[ITE] %s:[%d]: pdata != NULL \n", __func__, __LINE__);
	}
	printk("[ITE] %s:[%d]: ite_chip_data->int_gpio =%d \n", __func__, __LINE__,ite_chip_data->int_gpio);
	
	//
	mutex_init(&ite_chip_data->mutex_lock);
	INIT_WORK(&ite_chip_data->work,ite8566_work);
		
	ite_chip_data->input_dev = input_allocate_device();
	if (ite_chip_data->input_dev == NULL) {
		err = -ENOMEM;
		printk(KERN_ERR "[ITE] %s:[%d]: Failed to allocate input device.\n", __func__, __LINE__);
		goto probe_err_input_dev_alloc_failed;
	}

	snprintf(ite_chip_data->input_phys_name, sizeof(ite_chip_data->input_phys_name), "%s/input-kp", dev_name(&client->dev));
	ite_chip_data->input_dev->name = "asus-ec";
	ite_chip_data->input_dev->id.bustype = BUS_I2C; //20110901add
	ite_chip_data->input_dev->phys = ite_chip_data->input_phys_name;	
	ite_chip_data->input_dev->event = ite8566_i2c_event;
	
	//Keypad
	for(i=0;i<ITE_KEYMAP_SIZE;i++) {
		set_bit(i,ite_chip_data->input_dev->keybit);
		ite_chip_data->keymap[i] = i;
	}

	ite_chip_data->input_dev->evbit[0] = BIT(EV_KEY) | BIT(EV_MSC);

	__set_bit(EV_KEY, ite_chip_data->input_dev->evbit);
	
	ite_chip_data->kp_enabled = true;
	ite_chip_data->attrs.attrs = ite8566_attr;
	err = sysfs_create_group(&client->dev.kobj, &ite_chip_data->attrs);
    if(err){
        printk(KERN_ERR "[ITE]: %s: Not able to create the sysfs\n", __func__);
		goto probe_err_device_create_file_failed;
    }
		
	err = input_register_device(ite_chip_data->input_dev);
	if(err < 0) {
		printk(KERN_ERR "[ITE]: %s: Error to register input device. \n", __func__);
		goto probe_err_input_register_device_failed;
	}

	/*init INTERRUPT pin*/
	err = gpio_request(ite_chip_data->int_gpio, "asus-ec-irq");
	if(err < 0)
		printk(KERN_ERR "Failed to request GPIO%d (asus-ec-interrupt) error=%d\n", ite_chip_data->int_gpio, err);

	err = gpio_direction_input(ite_chip_data->int_gpio);
	if (err){
		printk(KERN_ERR "Failed to set interrupt direction, error=%d\n", err);
		goto probe_err_gpio_direction_input_failed;
	}
	
	ite_chip_data->irq = gpio_to_irq(ite_chip_data->int_gpio);
	printk("[ITE]: intr_gpio=%d, irq=%d \n", ite_chip_data->int_gpio, ite_chip_data->irq);
	
	if(request_irq(ite_chip_data->irq, ite8566_irq, IRQF_TRIGGER_FALLING, ite_chip_data->client->name, ite_chip_data->client))
	{
		printk(KERN_ERR "[ITE]: %s: Can't allocate irq \n", __func__);
		goto probe_err_request_irq_failed;
	}
	//Joe for test ++
	major = register_chrdev(0, ASUS_EC_NAME, &i2c_fops);
	if (major < 0)
	{
		printk(KERN_ERR "[ITE]: %s: Couldn't register a device. \n", __func__);
		goto probe_err_register_chrdev_failed;
	}else {
		printk("[ITE]: %s: major num=%x . \n", __func__, major);
	}
	//Joe for test --
	
	
	if (!ite8566_i2c_hw_init(ite_chip_data->client)) {
		err = -EINVAL;
		printk(KERN_ERR "[ITE]: %s: ite_i2c_hw_init failed!! \n", __func__);
		goto probe_err_i2c_hw_init;
	}
		
//Joe	getHIDDescriptor(client);
//Joe	msleep(50);
//Joe 	getReportDescriptor(client);
//Joe #if 1
//Joe	i2c_set_clientdata(client, ite_chip_data);
//Joe	device_init_wakeup(&client->dev, 1);
//Joe	enable_irq_wake(client->irq);
//Joe #endif

	printk("[ITE]: ASUS EC Driver ver:%s . \n", DRIVER_VERSION);
		
	ite8566_read_pad_battery_info(&ite_chip_data->pad_bat_status, &ite_chip_data->pad_bat_temperature, &ite_chip_data->pad_bat_voltage,
							&ite_chip_data->pad_bat_current, &ite_chip_data->pad_bat_capacity, &ite_chip_data->pad_bat_energy);
	
	printk("[ITE]:: pad bat_status=0x%x \n",ite_chip_data->pad_bat_status);
	printk("[ITE]:: pad bat_temp=0x%x \n",ite_chip_data->pad_bat_temperature);
	printk("[ITE]:: pad bat_vol=0x%x \n",ite_chip_data->pad_bat_voltage);
	printk("[ITE]:: pad bat_current=0x%x \n",ite_chip_data->pad_bat_current);
	printk("[ITE]:: pad bat_capacity=0x%x \n",ite_chip_data->pad_bat_capacity);
	printk("[ITE]:: pad bat_energy=0x%x \n",ite_chip_data->pad_bat_energy);
		
//	ite8566_read_dock_battery_info(&ite_chip_data->dock_bat_status, &ite_chip_data->dock_bat_temperature, &ite_chip_data->dock_bat_voltage,
//							&ite_chip_data->dock_bat_current, &ite_chip_data->dock_bat_capacity, &ite_chip_data->dock_bat_energy);
//	
//	printk("[ITE]:: dock bat_status=0x%x \n",ite_chip_data->dock_bat_status);
//	printk("[ITE]:: dock bat_temp=0x%x \n",ite_chip_data->dock_bat_temperature);
//	printk("[ITE]:: dock bat_vol=0x%x \n",ite_chip_data->dock_bat_voltage);
//	printk("[ITE]:: dock bat_current=0x%x \n",ite_chip_data->dock_bat_current);
//	printk("[ITE]:: dock bat_capacity=0x%x \n",ite_chip_data->dock_bat_capacity);
//	printk("[ITE]:: dock bat_energy=0x%x \n",ite_chip_data->dock_bat_energy);

	return 0;

probe_err_i2c_hw_init:
probe_err_register_chrdev_failed:
probe_err_request_irq_failed:
	free_irq(ite_chip_data->irq, ite_chip_data);
probe_err_gpio_direction_input_failed:
	input_unregister_device(ite_chip_data->input_dev);
	gpio_free(ite_chip_data->int_gpio);
probe_err_input_register_device_failed:
	sysfs_remove_group(&client->dev.kobj, &ite_chip_data->attrs);
probe_err_device_create_file_failed:
	if (ite_chip_data->input_dev)
        input_free_device(ite_chip_data->input_dev);
probe_err_input_dev_alloc_failed:
probe_err_create_wq_failed:
	if (ite_chip_data->asusec_wq)
        destroy_workqueue(ite_chip_data->asusec_wq);
        
	kfree(ite_chip_data);
probe_err_alloc_data_failed:
probe_err_check_functionality_failed:
	
	 return err;

}

static int __devexit ite8566_remove(struct i2c_client *client)
{
//	struct ite_chip *lm = i2c_get_clientdata(client);
	
	disable_irq_wake(ite_chip_data->irq);
	free_irq(ite_chip_data->irq, ite_chip_data);
	cancel_work_sync(&ite_chip_data->work);
	
	input_unregister_device(ite_chip_data->input_dev);
	
	sysfs_remove_group(&client->dev.kobj, &ite_chip_data->attrs);
	
//Joe	platform_device_unregister(asusec_dev);
	
	kfree(ite_chip_data);
	
	return 0;
}

int asusec_command(struct i2c_client *client, unsigned int cmd, void *arg)
{
	printk("[ITE]: %s ++ \n", __func__);
	return 0;
}

static const struct i2c_device_id asusec_id[] = {
	{ ASUS_EC_NAME, 0},
	{ }
};

static struct i2c_driver ite_i2c_driver = {
	.driver = {
		.name	= ASUS_EC_NAME,
		.owner  = THIS_MODULE,
	},
	.probe		= ite8566_probe,
	.remove		= __devexit_p(ite8566_remove),
	.id_table	= asusec_id,
	.command	= asusec_command,
};
MODULE_DEVICE_TABLE(i2c, asusec_id);

static int __init ite8566_init(void)
{
	printk("[ITE]: %s ++ \n", __func__);
	return i2c_add_driver(&ite_i2c_driver);
}

static void __exit ite8566_exit(void)
{
	i2c_del_driver(&ite_i2c_driver);
}
module_init(ite8566_init);
module_exit(ite8566_exit);

MODULE_AUTHOR("Joe_CH Chen <joe_ch_chen@asus.com>");
MODULE_DESCRIPTION("ITE EC driver");
MODULE_LICENSE("GPL");

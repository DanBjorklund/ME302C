/*
 * ite.h - Configuration for ITE keypad driver.
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
 */

#ifndef __LINUX_ITE_H
#define __LINUX_ITE_H

#include <linux/types.h>

/*
 * Largest keycode that the chip can send, plus one,
 * so keys can be mapped directly at the index of the
 * ITE keycode instead of subtracting one.
 */
//#define ITE_KEYMAP_SIZE	(0x7f + 1)
#define ITE_KEYMAP_SIZE	256
//#define ITE_KEYMAP_SIZE	512

#define ITE_NUM_PWMS		3

struct asusec_platform_data {
	const unsigned short *keymap;
	const char *pwm_names[ITE_NUM_PWMS];
	const char *name; /* Device name. */
	
	int debounce_time; /* Time to watch for key bouncing, in ms. */
	int active_time; /* Idle time until sleep, in ms. */
	
	int size_x;
	int size_y;
	bool repeat;
	
};

u8 ITE_CMD_FLASH[6]     ={0x6 , 0x6 ,0x2 ,0x0,0x0,0x0};//cmd len subcmd addr2 addr1 addr0
//u8 ITE_CMD_FLASH_INITFW[6] = {0x6 , 0x6 ,0x1 ,0x49,0x54,0x45};//cmd len subcmd payload[0-2] = { "I", "T", "E"} = { 0x49,0x54,0x45}
u8 ITE_CMD_FLASH_INITFW[6] = {0x6 , 0x4 ,0x1 ,0x49,0x54,0x45};//cmd len subcmd payload[0-2] = { "I", "T", "E"} = { 0x49,0x54,0x45}
u8 ITE_CMD_FLASH_FINALIZEFW[3] = {0x6 , 0x1 ,0x4};//cmd len subcmd 
//u8 ITE_CMD_FLASH_FINALIZEFW[6] = {0x6 , 0x4 ,0x4 ,0x49,0x54,0x45};//cmd lensubcmd payload[0-2] = { "I", "T", "E"} = { 0x49,0x54,0x45}
//u8 ITE_CMD_INIT_EC[4]   ={0x3 , 0x2 ,0x0 ,0x1};//cmd len2 (subcmd payload)

u8 ITE_CMD_INIT_EC[4]   ={0x3 , 0x2 ,0x0 ,0x1};//cmd len2 (subcmd payload)
u8 ITE_CMD_INIT_KB[3]   ={0x4 , 0x1 ,0xf4};//cmd len1 subcmd
u8 ITE_CMD_INIT_TP[4]   ={0x5 , 0x2 ,0x3 ,0x3};//cmd len2 subcmd payload

u8 ITE_CMD_BATTERY[3]   ={0x2 , 0x2 ,0x1 }; //battery => len(2) cmd(2) subcmd(1)
//u8 ITE_CMD_BATTERY_RSOC[3]              ={0x2 , 0x2 ,0x0 }; //battery => len(2) cmd(2) subcmd(1) for Relative State of Charge
//u8 ITE_CMD_BATTERY_VOL[3]               ={0x2 , 0x2 ,0x1 }; //battery => len(2) cmd(2) subcmd(1) for voltage
//u8 ITE_CMD_BATTERY_AVG_CURRENT[3]       ={0x2 , 0x2 ,0x4}; //battery => len(2) cmd(2) subcmd(4) for average current
//u8 ITE_CMD_BATTERY_TEMP[3]              ={0x2 , 0x2 ,0xA }; //battery => len(2) cmd(2) subcmd(9) for temp K

u8 ITE_CMD_TEST[3]   ={0x2 , 0x2 ,0 }; //battery => len(2) cmd(2) subcmd(0)

/* Addresses to scan */
static const unsigned short normal_i2c[] = { 0x45, I2C_CLIENT_END };
//Joe I2C_CLIENT_INSMOD_1(asusec);

//Joe static int do_w_test(struct i2c_client *client);
static int ite8566_readbuf(struct i2c_client *client,u8 *buf);
//static int xp; // x axis
//static int yp; // y axis

#define EVENT_ITE_KEYBOARD	0x80
#define EVENT_ITE_TOUCHPAD	0x81

#define DEV_IOCTLID 0xD0
#define IOCTL_CHKREADY _IOW(DEV_IOCTLID, 24, int)
#define IOCTL_PREREAD _IOW(DEV_IOCTLID, 22, int)
#define IOCTL_PREWRITE _IOW(DEV_IOCTLID, 12, int)
#define IOCTL_READ _IOW(DEV_IOCTLID, 20, int)
#define IOCTL_WRITE _IOW(DEV_IOCTLID, 10, int)
#define IOCTL_FINALIZEFW _IOW(DEV_IOCTLID, 6, int)
#define IOCTL_INITFW _IOW(DEV_IOCTLID, 5, int)
#define IOCTL_RESET _IOW(DEV_IOCTLID, 0, int)
#define IOCTL_ITE_TEST _IOW(DEV_IOCTLID, 4, int)

#define CMD_W 0
#define CMD_R 1

//#define dprintk
#define dprintk printk


#define ASUSDEC_ID_KEYBOARD			0x01
#define ASUSDEC_ID_FUNCTION_1		0x03
#define ASUSDEC_ID_FUNCTION_2		0x04
#define ASUSDEC_ID_FUNCTION_3		0x05

#define ASUSDEC_KEY_RELASE			0x00

#define ASUSDEC_KEY_LEFT_CTRL		0x01
#define ASUSDEC_KEY_LEFT_SHITE		0x02
#define ASUSDEC_KEY_LEFT_ALT		0x04
#define ASUSDEC_KEY_LEFT_WINDOWS	0x08

#define ASUSDEC_KEY_RIGHT_CTRL		0x10
#define ASUSDEC_KEY_RIGHT_SHITE		0x20
#define ASUSDEC_KEY_RIGHT_ALT		0x40

#define ASUSDEC_KEY_POWER_4GEAR		KEY_F22
#define ASUSDEC_KEY_LCDOFF			KEY_F23
#define ASUSDEC_KEY_TOUCHPAD		KEY_F24



//static struct platform_device *asusec_dev; /* Device structure */


/* If the system has several batteries we need a different name for each
 * of them...
 */
//static DEFINE_IDR(battery_id);
//static DEFINE_MUTEX(battery_mutex);

struct ite_chip;
struct asusec_access_methods {
        int (*read)(u8 reg, int *rt_value, int b_single,
                struct ite_chip *ite_chip_data);
};

//Joe static enum power_supply_property asusec_battery_props[] = {
//Joe         POWER_SUPPLY_PROP_PRESENT,
//Joe         POWER_SUPPLY_PROP_VOLTAGE_NOW,
//Joe         POWER_SUPPLY_PROP_CURRENT_NOW,
//Joe         POWER_SUPPLY_PROP_CAPACITY,
//Joe         POWER_SUPPLY_PROP_TEMP,
//Joe };

struct asusdec_keypad{
	int value;
	int input_keycode;
	int extend;	
};

struct ite_chip {
	/* device lock */
	struct mutex mutex_lock;
	struct i2c_client *client;
	struct work_struct work;
	struct input_dev *input_dev;
	struct workqueue_struct *asusec_wq;
	struct attribute_group attrs;
	struct asusdec_keypad keypad_data;
	
	bool kp_enabled;
	bool pm_suspend;
	unsigned keys_down;
	char input_phys_name[32];
	unsigned short keymap[ITE_KEYMAP_SIZE];
	int size_x;
	int size_y;
	int debounce_time;
	int active_time;
	
	int id;
	int voltage_uV;
	int current_uA;
	int temp_C;
	int charge_rsoc;
//	struct asusec_access_methods   *bus;
//	struct power_supply     bat;
//	struct timer_list 	timer;
	int irq;
	int int_gpio;
	unsigned int modifier_repeat_key;
	
	int pad_bat_status;
	int pad_bat_temperature;
	int pad_bat_voltage;
	int pad_bat_current;
	int pad_bat_capacity;	//percentage
	int pad_bat_energy;
	int pad_bat_avg_time_to_empty;
	int pad_bat_remaining_capacity;
	int pad_bat_avg_time_to_full;
	
	int dock_bat_status;
	int dock_bat_temperature;
	int dock_bat_voltage;
	int dock_bat_current;
	int dock_bat_capacity;	//percentage
	int dock_bat_energy;
	int dock_bat_avg_time_to_empty;
	int dock_bat_remaining_capacity;
	int dock_bat_avg_time_to_full;

};

struct ite_chip *ite_chip_data;

#define client_to_ite(c)	container_of(c, struct ite_chip, client)
#define dev_to_ite(d)	container_of(d, struct ite_chip, client->dev)
#define work_to_ite(w)	container_of(w, struct ite_chip, work)
#define cdev_to_pwm(c)		container_of(c, struct ite_pwm, cdev)
#define work_to_pwm(w)		container_of(w, struct ite_pwm, work)

#define ITE_MAX_DATA 8

struct ite_i2c_data {
	struct i2c_client       *fake_client;
};
struct ite_i2c_data *ite_i2c_data_global;

void sendCmd(struct i2c_client *client,u8 *cmd,u8 rw);

#endif /* __LINUX_ITE_H */

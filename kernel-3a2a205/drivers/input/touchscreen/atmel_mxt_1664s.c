/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/i2c/atmel_mxt_1664s.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/slab.h>

// Jui add ++
#include <linux/gpio.h>
#include <linux/mutex.h>
#include <linux/debugfs.h>
// Jui add --

// Jui-Chuan add for early suspend and late resume ++
#include <linux/earlysuspend.h>
// Jui-Chuan add for early suspend and late resume --

#include <linux/wakelock.h>

/* Version */
#define MXT_VER_20		20
#define MXT_VER_21		21
#define MXT_VER_22		22

/* Slave addresses */
#define MXT_APP_LOW		0x4a
#define MXT_APP_HIGH		0x4b
#define MXT_BOOT_LOW		0x26
#define MXT_BOOT_HIGH		0x27

/* Firmware */
#define MXT_FW_NAME		"maxtouch.fw"

/* Registers */
#define MXT_FAMILY_ID		0x00
#define MXT_VARIANT_ID		0x01
#define MXT_VERSION		0x02
#define MXT_BUILD		0x03
#define MXT_MATRIX_X_SIZE	0x04
#define MXT_MATRIX_Y_SIZE	0x05
#define MXT_OBJECT_NUM		0x06
#define MXT_OBJECT_START	0x07

#define MXT_OBJECT_SIZE		6

/* Object types */
#define MXT_DEBUG_DIAGNOSTIC_T37	37
#define MXT_GEN_MESSAGE_T5		5
#define MXT_GEN_COMMAND_T6		6
#define MXT_GEN_POWER_T7		7
#define MXT_GEN_ACQUIRE_T8		8
#define MXT_GEN_DATASOURCE_T53		53
#define MXT_TOUCH_MULTI_T9		9
#define MXT_TOUCH_KEYARRAY_T15		15
#define MXT_TOUCH_PROXIMITY_T23		23
#define MXT_TOUCH_PROXKEY_T52		52
#define MXT_PROCI_GRIPFACE_T20		20
#define MXT_PROCG_NOISE_T22		22
#define MXT_PROCI_ONETOUCH_T24		24
#define MXT_PROCI_TWOTOUCH_T27		27
#define MXT_PROCI_GRIP_T40		40
#define MXT_PROCI_PALM_T41		41
#define MXT_PROCI_TOUCHSUPPRESSION_T42	42
#define MXT_PROCI_STYLUS_T47		47
#define MXT_PROCG_NOISESUPPRESSION_T48	48
#define MXT_SPT_COMMSCONFIG_T18		18
#define MXT_SPT_GPIOPWM_T19		19
#define MXT_SPT_SELFTEST_T25		25
#define MXT_SPT_CTECONFIG_T28		28
#define MXT_SPT_USERDATA_T38		38
#define MXT_SPT_DIGITIZER_T43		43
#define MXT_SPT_MESSAGECOUNT_T44	44
#define MXT_SPT_CTECONFIG_T46		46

/* MXT_GEN_COMMAND_T6 field */
#define MXT_COMMAND_RESET	0
#define MXT_COMMAND_BACKUPNV	1
#define MXT_COMMAND_CALIBRATE	2
#define MXT_COMMAND_REPORTALL	3
#define MXT_COMMAND_DIAGNOSTIC	5

/* MXT_GEN_POWER_T7 field */
#define MXT_POWER_IDLEACQINT	0
#define MXT_POWER_ACTVACQINT	1
#define MXT_POWER_ACTV2IDLETO	2

/* MXT_GEN_ACQUIRE_T8 field */
#define MXT_ACQUIRE_CHRGTIME	0
#define MXT_ACQUIRE_TCHDRIFT	2
#define MXT_ACQUIRE_DRIFTST	3
#define MXT_ACQUIRE_TCHAUTOCAL	4
#define MXT_ACQUIRE_SYNC	5
#define MXT_ACQUIRE_ATCHCALST	6
#define MXT_ACQUIRE_ATCHCALSTHR	7

/* MXT_TOUCH_MULTI_T9 field */
#define MXT_TOUCH_CTRL		0
#define MXT_TOUCH_XORIGIN	1
#define MXT_TOUCH_YORIGIN	2
#define MXT_TOUCH_XSIZE		3
#define MXT_TOUCH_YSIZE		4
#define MXT_TOUCH_BLEN		6
#define MXT_TOUCH_TCHTHR	7
#define MXT_TOUCH_TCHDI		8
#define MXT_TOUCH_ORIENT	9
#define MXT_TOUCH_MOVHYSTI	11
#define MXT_TOUCH_MOVHYSTN	12
#define MXT_TOUCH_NUMTOUCH	14
#define MXT_TOUCH_MRGHYST	15
#define MXT_TOUCH_MRGTHR	16
#define MXT_TOUCH_AMPHYST	17
#define MXT_TOUCH_XRANGE_LSB	18
#define MXT_TOUCH_XRANGE_MSB	19
#define MXT_TOUCH_YRANGE_LSB	20
#define MXT_TOUCH_YRANGE_MSB	21
#define MXT_TOUCH_XLOCLIP	22
#define MXT_TOUCH_XHICLIP	23
#define MXT_TOUCH_YLOCLIP	24
#define MXT_TOUCH_YHICLIP	25
#define MXT_TOUCH_XEDGECTRL	26
#define MXT_TOUCH_XEDGEDIST	27
#define MXT_TOUCH_YEDGECTRL	28
#define MXT_TOUCH_YEDGEDIST	29
#define MXT_TOUCH_JUMPLIMIT	30

/* MXT_PROCI_GRIPFACE_T20 field */
#define MXT_GRIPFACE_CTRL	0
#define MXT_GRIPFACE_XLOGRIP	1
#define MXT_GRIPFACE_XHIGRIP	2
#define MXT_GRIPFACE_YLOGRIP	3
#define MXT_GRIPFACE_YHIGRIP	4
#define MXT_GRIPFACE_MAXTCHS	5
#define MXT_GRIPFACE_SZTHR1	7
#define MXT_GRIPFACE_SZTHR2	8
#define MXT_GRIPFACE_SHPTHR1	9
#define MXT_GRIPFACE_SHPTHR2	10
#define MXT_GRIPFACE_SUPEXTTO	11

/* MXT_PROCI_NOISE field */
#define MXT_NOISE_CTRL		0
#define MXT_NOISE_OUTFLEN	1
#define MXT_NOISE_GCAFUL_LSB	3
#define MXT_NOISE_GCAFUL_MSB	4
#define MXT_NOISE_GCAFLL_LSB	5
#define MXT_NOISE_GCAFLL_MSB	6
#define MXT_NOISE_ACTVGCAFVALID	7
#define MXT_NOISE_NOISETHR	8
#define MXT_NOISE_FREQHOPSCALE	10
#define MXT_NOISE_FREQ0		11
#define MXT_NOISE_FREQ1		12
#define MXT_NOISE_FREQ2		13
#define MXT_NOISE_FREQ3		14
#define MXT_NOISE_FREQ4		15
#define MXT_NOISE_IDLEGCAFVALID	16

/* MXT_SPT_COMMSCONFIG_T18 */
#define MXT_COMMS_CTRL		0
#define MXT_COMMS_CMD		1

/* MXT_SPT_CTECONFIG_T28 field */
#define MXT_CTE_CTRL		0
#define MXT_CTE_CMD		1
#define MXT_CTE_MODE		2
#define MXT_CTE_IDLEGCAFDEPTH	3
#define MXT_CTE_ACTVGCAFDEPTH	4
#define MXT_CTE_VOLTAGE		5

#define MXT_VOLTAGE_DEFAULT	2700000
#define MXT_VOLTAGE_STEP	10000

/* Define for MXT_GEN_COMMAND_T6 */
#define MXT_BOOT_VALUE		0xa5
#define MXT_BACKUP_VALUE	0x55
#define MXT_BACKUP_TIME		25	/* msec */
#define MXT_RESET_TIME		65	/* msec */

#define MXT_FWRESET_TIME	175	/* msec */

/* Command to unlock bootloader */
#define MXT_UNLOCK_CMD_MSB	0xaa
#define MXT_UNLOCK_CMD_LSB	0xdc

/* Bootloader mode status */
#define MXT_WAITING_BOOTLOAD_CMD	0xc0	/* valid 7 6 bit only */
#define MXT_WAITING_FRAME_DATA	0x80	/* valid 7 6 bit only */
#define MXT_FRAME_CRC_CHECK	0x02
#define MXT_FRAME_CRC_FAIL	0x03
#define MXT_FRAME_CRC_PASS	0x04
#define MXT_APP_CRC_FAIL	0x40	/* valid 7 8 bit only */
#define MXT_BOOT_STATUS_MASK	0x3f

/* Touch status */
#define MXT_SUPPRESS		(1 << 1)
#define MXT_AMP			(1 << 2)
#define MXT_VECTOR		(1 << 3)
#define MXT_MOVE		(1 << 4)
#define MXT_RELEASE		(1 << 5)
#define MXT_PRESS		(1 << 6)
#define MXT_DETECT		(1 << 7)

/* Touch orient bits */
#define MXT_XY_SWITCH		(1 << 0)
#define MXT_X_INVERT		(1 << 1)
#define MXT_Y_INVERT		(1 << 2)

/* Touchscreen absolute values */
#define MXT_MAX_AREA		0xff

#define MXT_MAX_FINGER		10

// Jui-Chuan add for Self test ++
#define MXT_MSGR_T25_RUN_ALL_TESTS 0xFE
#define MXT_MSG_T25_STATUS 0x01
#define MXT_ADR_T25_CTRL 0x00
#define MXT_ADR_T25_CMD 0x01
char self_test_result[100];
int after_update_fw;
// Jui-Chuan add for Self test --

struct mxt_info {
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u8 matrix_xsize;
	u8 matrix_ysize;
	u8 object_num;
};

struct mxt_object {
	u8 type;
	u16 start_address;
	u8 size;
	u8 instances;
	u8 num_report_ids;

	/* to map object and message */
	u8 max_reportid;
};

struct mxt_message {
	u8 reportid;
	u8 message[7];
	u8 checksum;
};

struct mxt_finger {
	int status;
	int x;
	int y;
	int area;
	int pressure;
};

/* Each client has this additional data */
struct mxt_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	const struct mxt_platform_data *pdata;
	struct mxt_object *object_table;
	struct mxt_info info;
	struct mxt_finger finger[MXT_MAX_FINGER];
	unsigned int irq;
	unsigned int max_x;
	unsigned int max_y;


// 	platform data ++ 
	unsigned int x_line;
	unsigned int y_line;
	unsigned int x_size;
	unsigned int y_size;
	unsigned int blen;
	unsigned int threshold;
	unsigned int voltage;
	unsigned char orient;
//	unsigned long irqflags;
	int reset_gpio;
	int int_gpio;
// 	platform data --


	/* debugfs variables */
	struct dentry *debug_dir;
	int current_debug_datap;

	struct mutex debug_mutex;
	u16 *debug_data;
	u16 num_nodes;		/* Number of sensor nodes */

	/* for read write block */
	u16 msg_proc_addr; 
	u16 last_read_addr;

	// Jui-Chuan add for early suspend and late resume ++
	struct early_suspend early_suspend;
	// Jui-Chuan add for early suspend and late resume --

	int suspend_state;

	int numtouch;

	struct wake_lock wakelock;

	int tp_firmware_upgrade_proceed;

	struct workqueue_struct *mxt_wq;
	struct delayed_work touch_chip_firmware_upgrade_work;
	struct delayed_work touch_chip_firmware_upgrade_work_if_fw_broken;
	
};

static struct mxt_data *touch_chip; // Jui-Chuan add for global variant

#define MXT_ADR_T9_NUMTOUCH 0x0e
static int mxt_write_block(struct i2c_client *client,
                    u16 addr,
                    u16 length,
                    u8 *value);
void mxt_config_init(struct mxt_data *mxt){
	u8 T5[] = {255, 0, 168, 87, 41, 0, 0, 0,0};
	u8 T6[] = {0, 0, 0, 0, 0, 0};
	u8 T38[] = {0,0,0,0,0,0,0,0,0,
                    0,0,0,0,0,0,0,0,0,
                    0,0,0,0,0,0,0,0,0,
                    0,0,0,0,0,0,0,0,0,
                    0,0,0,0,0,0,0,0,0,
                    0,0,0,0,0,0,0,0,0,
                    0,0,0,0,0,0,0,0,0,0};
	u8 T7[] = {15, 255, 220, 87};
	u8 T8[] = {127, 0, 10, 10, 0, 0, 5, 80, 6, 192};
	u8 T9[] = {131, 0, 0, 30, 52, 0, 110, 50, 2, 1, 10, 15,8, 48, 5, 20, 20, 20, 255, 15, 255, 15, 7, 15, 10,
		   10, 202, 35, 138, 20, 20, 15, 45, 49, 0, 0};
	u8 T15[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	u8 T18[] = {0, 0};
	u8 T24[] = {0 ,0, 0, 0, 0, 
		    0, 0, 0, 0, 0, 
		    0, 0, 0, 0, 0, 
                    0, 0, 0, 0};
	u8 T25[] = {0, 0, 0, 0, 0,
		    0, 0, 0, 0, 0,
		    0, 0, 0, 0, 0};
	u8 T27[] = {0, 0, 0, 0, 0, 0, 0};
	u8 T40[] = {0, 0, 0, 0, 0};
	u8 T42[] = {0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0};
	u8 T43[] = {137, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0};
	u8 T46[] = {0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0};
	u8 T47[] = {0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0,
                    0, 0};

	u8 i= 0, max_objs= 0;
	u16 addr;
	struct mxt_object *obj_index;

	printk("[Atmel] mxt_config_init start\n");
	dev_dbg(&mxt->client->dev, "In function %s", __func__);
//	max_objs = mxt->device_info.num_objs;
	max_objs = mxt->info.object_num;
	obj_index = mxt->object_table;
	
	int instance = 0;

	for (i=0; i<max_objs; i++) {
//		addr = obj_index->chip_addr;
		addr = obj_index->start_address + (obj_index->size + 1) * instance;
		switch(obj_index->type) {
		case MXT_GEN_MESSAGE_T5:
			printk("[Atmel] mxt_config_init T5\n");
			mxt_write_block(mxt->client, addr,
                                        sizeof(T5), T5);
                        break;		
		case MXT_GEN_COMMAND_T6:
			printk("[Atmel] mxt_config_init T6\n");
                        mxt_write_block(mxt->client, addr,
                                        sizeof(T6), T6);
                        break;
		case MXT_GEN_POWER_T7:
			printk("[Atmel] mxt_config_init T7\n");
			mxt_write_block(mxt->client, addr,
					sizeof(T7), T7);
			break;
		case MXT_GEN_ACQUIRE_T8:
			printk("[Atmel] mxt_config_init T8\n");
			mxt_write_block(mxt->client, addr,
					sizeof(T8), T8);
			break;
		case MXT_TOUCH_MULTI_T9:
			printk("[Atmel] mxt_config_init T9\n");
			if (T9[MXT_ADR_T9_NUMTOUCH] != mxt->numtouch)
				T9[MXT_ADR_T9_NUMTOUCH] = mxt->numtouch;
			mxt_write_block(mxt->client, addr,
					sizeof(T9), T9);
			dev_dbg(&mxt->client->dev, "init multitouch object");
			break;
		case MXT_TOUCH_KEYARRAY_T15:
			printk("[Atmel] mxt_config_init T15\n");
			mxt_write_block(mxt->client, addr,
					sizeof(T15), T15);
			break;
		case MXT_SPT_COMMSCONFIG_T18:
			printk("[Atmel] mxt_config_init T18\n");
			mxt_write_block(mxt->client, addr,
					sizeof(T18), T18);
			break;
	//	case MXT_SPT_GPIOPWM_T19:
	//		mxt_write_block(mxt->client, addr,
	//				sizeof(T19), T19);
	//		break;
	//	case MXT_PROCI_GRIPFACE_T20:
	//		mxt_write_block(mxt->client, addr,
	//				sizeof(T20), T20);
	//		break;
	//	case MXT_PROCG_NOISE_T22:
	//		mxt_write_block(mxt->client, addr,
	//				sizeof(T22), T22);
	//		break;
	//	case MXT_TOUCH_PROXIMITY_T23:
	//		mxt_write_block(mxt->client, addr,
	//				sizeof(T23), T23);
	//		break;
		case MXT_PROCI_ONETOUCH_T24:
			printk("[Atmel] mxt_config_init T24\n");
			mxt_write_block(mxt->client, addr,
					sizeof(T24), T24);
			break;
		case MXT_SPT_SELFTEST_T25:
			printk("[Atmel] mxt_config_init T25\n");
			mxt_write_block(mxt->client, addr,
					sizeof(T25), T25);
			break;
		case MXT_PROCI_TWOTOUCH_T27:
			printk("[Atmel] mxt_config_init T27\n");
                        mxt_write_block(mxt->client, addr,
                                        sizeof(T27), T27);
                        break;
	//	case MXT_SPT_CTECONFIG_T28:
	//		mxt_write_block(mxt->client, addr,
	//				sizeof(T28), T28);
	//		break;
	//	case MXT_DEBUG_DIAGNOSTIC_T37:
	//		mxt_write_block(mxt->client, addr,
	//				sizeof(T37), T37);
	//		break;
		case MXT_SPT_USERDATA_T38:
			printk("[Atmel] mxt_config_init T38\n");
			mxt_write_block(mxt->client, addr,
					sizeof(T38), T38);
			break;
		case MXT_PROCI_GRIP_T40:
			printk("[Atmel] mxt_config_init T40\n");
			mxt_write_block(mxt->client, addr,
					sizeof(T40), T40);
			break;
		case MXT_PROCI_TOUCHSUPPRESSION_T42:
			printk("[Atmel] mxt_config_init T42\n");
			mxt_write_block(mxt->client, addr,
					sizeof(T42), T42);
			break;
		case MXT_SPT_DIGITIZER_T43:
			printk("[Atmel] mxt_config_init T43\n");
                        mxt_write_block(mxt->client, addr,
                                        sizeof(T43), T43);
                        break;
		case MXT_SPT_CTECONFIG_T46:
			printk("[Atmel] mxt_config_init T46\n");
			mxt_write_block(mxt->client, addr,
					sizeof(T46), T46);
			break;
		case MXT_PROCI_STYLUS_T47:
			printk("[Atmel] mxt_config_init T47\n");
			mxt_write_block(mxt->client, addr,
					sizeof(T47), T47);
			break;

#if 0
		case MXT_PROCI_ADAPTIVETHRESHOLD_T55:
			mxt_write_block(mxt->client, addr,
					sizeof(T55), T55);
			break;
		case MXT_PROCI_SHIELDLESS_T56:
			mxt_write_block(mxt->client, addr,
					sizeof(T56), T56);
			break;
		case MXT_PROCI_EXTRATOUCHSCREENDATA_T57:
			mxt_write_block(mxt->client, addr,
					sizeof(T57), T57);
			break;
		case MXT_SPT_TIMER_T61:
			mxt_write_block(mxt->client, addr,
					sizeof(T61), T61);
			break;
		case MXT_PROCG_NOISESUPPRESSION_T62:
			mxt_write_block(mxt->client, addr,
					sizeof(T62), T62);
			break;			
#endif
		default:
			break;
		}
		obj_index++;
	}
	dev_dbg(&mxt->client->dev, "config init Done.");

}

// Jui-Chuan add for early suspend and late resume ++
#ifdef CONFIG_HAS_EARLYSUSPEND
static void mxt_early_suspend(struct early_suspend *h);
static void mxt_late_resume(struct early_suspend *h);
#endif
// Jui-Chuan add for early suspend and late resume --

// Jui-Chuan add for MXT_BASE_ADDR ++
static u16 get_object_address(uint8_t object_type,
                              uint8_t instance,
                              struct mxt_object *object_table,
                              int max_objs)
{
        uint8_t object_table_index = 0;
        uint8_t address_found = 0;
        uint16_t address = 0;
        struct mxt_object *obj;

        while ((object_table_index < max_objs) && !address_found) {
                obj = &object_table[object_table_index];
                if (obj->type == object_type) {
                        address_found = 1;
                        /* Are there enough instances defined in the FW? */
                        if (obj->instances >= instance) {
				// change chip_addr to start_address
                                address = obj->start_address +
                                          (obj->size + 1) * instance;
                        } else {
                                return 0;
                        }
                }
                object_table_index++;
        }
        return address;
}

// Change "device_info.num_objs" to "info.object_num" 
#define MXT_BASE_ADDR(object_type, mxt)                                 \
        get_object_address(object_type, 0, mxt->object_table,           \
                           mxt->info.object_num) 


// Jui-Chuan add for MXT_BASE_ADDR --

static bool mxt_object_readable(unsigned int type)
{
	switch (type) {
	case MXT_GEN_MESSAGE_T5:
	case MXT_GEN_COMMAND_T6:
	case MXT_GEN_POWER_T7:
	case MXT_GEN_ACQUIRE_T8:
	case MXT_GEN_DATASOURCE_T53:
	case MXT_TOUCH_MULTI_T9:
	case MXT_TOUCH_KEYARRAY_T15:
	case MXT_TOUCH_PROXIMITY_T23:
	case MXT_TOUCH_PROXKEY_T52:
	case MXT_PROCI_GRIPFACE_T20:
	case MXT_PROCG_NOISE_T22:
	case MXT_PROCI_ONETOUCH_T24:
	case MXT_PROCI_TWOTOUCH_T27:
	case MXT_PROCI_GRIP_T40:
	case MXT_PROCI_PALM_T41:
	case MXT_PROCI_TOUCHSUPPRESSION_T42:
	case MXT_PROCI_STYLUS_T47:
	case MXT_PROCG_NOISESUPPRESSION_T48:
	case MXT_SPT_COMMSCONFIG_T18:
	case MXT_SPT_GPIOPWM_T19:
	case MXT_SPT_SELFTEST_T25:
	case MXT_SPT_CTECONFIG_T28:
	case MXT_SPT_USERDATA_T38:
	case MXT_SPT_DIGITIZER_T43:
	case MXT_SPT_CTECONFIG_T46:
		return true;
	default:
		return false;
	}
}

static bool mxt_object_writable(unsigned int type)
{
	switch (type) {
	case MXT_GEN_COMMAND_T6:
	case MXT_GEN_POWER_T7:
	case MXT_GEN_ACQUIRE_T8:
	case MXT_TOUCH_MULTI_T9:
	case MXT_TOUCH_KEYARRAY_T15:
	case MXT_TOUCH_PROXIMITY_T23:
	case MXT_TOUCH_PROXKEY_T52:
	case MXT_PROCI_GRIPFACE_T20:
	case MXT_PROCG_NOISE_T22:
	case MXT_PROCI_ONETOUCH_T24:
	case MXT_PROCI_TWOTOUCH_T27:
	case MXT_PROCI_GRIP_T40:
	case MXT_PROCI_PALM_T41:
	case MXT_PROCI_TOUCHSUPPRESSION_T42:
	case MXT_PROCI_STYLUS_T47:
	case MXT_PROCG_NOISESUPPRESSION_T48:
	case MXT_SPT_COMMSCONFIG_T18:
	case MXT_SPT_GPIOPWM_T19:
	case MXT_SPT_SELFTEST_T25:
	case MXT_SPT_CTECONFIG_T28:
	case MXT_SPT_DIGITIZER_T43:
	case MXT_SPT_CTECONFIG_T46:
		return true;
	default:
		return false;
	}
}

static void mxt_dump_message(struct device *dev,
				  struct mxt_message *message)
{
	dev_dbg(dev, "reportid:\t0x%x\n", message->reportid);
	dev_dbg(dev, "message1:\t0x%x\n", message->message[0]);
	dev_dbg(dev, "message2:\t0x%x\n", message->message[1]);
	dev_dbg(dev, "message3:\t0x%x\n", message->message[2]);
	dev_dbg(dev, "message4:\t0x%x\n", message->message[3]);
	dev_dbg(dev, "message5:\t0x%x\n", message->message[4]);
	dev_dbg(dev, "message6:\t0x%x\n", message->message[5]);
	dev_dbg(dev, "message7:\t0x%x\n", message->message[6]);
	dev_dbg(dev, "checksum:\t0x%x\n", message->checksum);

/*	printk("reportid:\t0x%x\n", message->reportid);
	printk("message1:\t0x%x\n", message->message[0]);
	printk("message2:\t0x%x\n", message->message[1]);
	printk("message3:\t0x%x\n", message->message[2]);
	printk("message4:\t0x%x\n", message->message[3]);
	printk("message5:\t0x%x\n", message->message[4]);
	printk("message6:\t0x%x\n", message->message[5]);
	printk("message7:\t0x%x\n", message->message[6]);
	printk("checksum:\t0x%x\n", message->checksum);
*/
}

static int mxt_check_bootloader(struct i2c_client *client,
				     unsigned int state)
{
	u8 val;

recheck:
	if (i2c_master_recv(client, &val, 1) != 1) {
		dev_err(&client->dev, "%s: i2c recv failed\n", __func__);
		return -EIO;
	}

	switch (state) {
	case MXT_WAITING_BOOTLOAD_CMD:
	case MXT_WAITING_FRAME_DATA:
		val &= ~MXT_BOOT_STATUS_MASK;
		break;
	case MXT_FRAME_CRC_PASS:
		if (val == MXT_FRAME_CRC_CHECK)
			goto recheck;
		break;
	default:
		return -EINVAL;
	}

	if (val != state) {
		dev_err(&client->dev, "Unvalid bootloader mode state\n");
		return -EINVAL;
	}

	return 0;
}

static int mxt_unlock_bootloader(struct i2c_client *client)
{
	u8 buf[2];

	buf[0] = MXT_UNLOCK_CMD_LSB;
	buf[1] = MXT_UNLOCK_CMD_MSB;

	if (i2c_master_send(client, buf, 2) != 2) {
		dev_err(&client->dev, "%s: i2c send failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int mxt_fw_write(struct i2c_client *client,
			     const u8 *data, unsigned int frame_size)
{
	if (i2c_master_send(client, data, frame_size) != frame_size) {
		dev_err(&client->dev, "%s: i2c send failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int __mxt_read_reg(struct i2c_client *client,
			       u16 reg, u16 len, void *val)
{
	struct i2c_msg xfer[2];
	u8 buf[2];

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;

	/* Write register */
	xfer[0].addr = client->addr;
	xfer[0].flags = 0;
	xfer[0].len = 2;
	xfer[0].buf = buf;

	/* Read data */
	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = len;
	xfer[1].buf = val;

	if (i2c_transfer(client->adapter, xfer, 2) != 2) {
		dev_err(&client->dev, "%s: i2c transfer failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int mxt_read_reg(struct i2c_client *client, u16 reg, u8 *val)
{
	return __mxt_read_reg(client, reg, 1, val);
}

static int mxt_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	u8 buf[3];

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;
	buf[2] = val;

	if (i2c_master_send(client, buf, 3) != 3) {
		dev_err(&client->dev, "%s: i2c send failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int mxt_read_object_table(struct i2c_client *client,
				      u16 reg, u8 *object_buf)
{
	return __mxt_read_reg(client, reg, MXT_OBJECT_SIZE,
				   object_buf);
}

static struct mxt_object *
mxt_get_object(struct mxt_data *data, u8 type)
{
	struct mxt_object *object;
	int i;
	//printk("[Atmel] data->info.object_num = %d\n",data->info.object_num);
	//printk("[Atmel] type = %d\n",type);
	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;
		//printk("[Atmel] compare object->type %d with input %d\n", object->type, type);
		if (object->type == type)
			return object;
	}

	dev_err(&data->client->dev, "Invalid object type\n");
	return NULL;
}

static int mxt_read_message(struct mxt_data *data,
				 struct mxt_message *message)
{
	struct mxt_object *object;
	u16 reg;

	object = mxt_get_object(data, MXT_GEN_MESSAGE_T5);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return __mxt_read_reg(data->client, reg,
			sizeof(struct mxt_message), message);
}

static int mxt_read_object(struct mxt_data *data,
				u8 type, u8 offset, u8 *val)
{
	struct mxt_object *object;
	u16 reg;

	object = mxt_get_object(data, type);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return __mxt_read_reg(data->client, reg + offset, 1, val);
}

static int mxt_write_object(struct mxt_data *data,
				 u8 type, u8 offset, u8 val)
{
	struct mxt_object *object;
	u16 reg;

	object = mxt_get_object(data, type);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return mxt_write_reg(data->client, reg + offset, val);
}

static void mxt_input_report(struct mxt_data *data, int single_id)
{
	struct mxt_finger *finger = data->finger;
	struct input_dev *input_dev = data->input_dev;
	int status = finger[single_id].status;
	int finger_num = 0;
	int id;

	for (id = 0; id < MXT_MAX_FINGER; id++) {
		if (!finger[id].status)
			continue;

		input_mt_slot(input_dev, id);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER,
				finger[id].status != MXT_RELEASE);

		if (finger[id].status != MXT_RELEASE) {
			finger_num++;
			input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR,
					finger[id].area);
			input_report_abs(input_dev, ABS_MT_POSITION_X,
					finger[id].x);
			input_report_abs(input_dev, ABS_MT_POSITION_Y,
					finger[id].y);
			input_report_abs(input_dev, ABS_MT_PRESSURE,
					finger[id].pressure);
		} else {
			finger[id].status = 0;
		}
	}

	input_report_key(input_dev, BTN_TOUCH, finger_num > 0);

	if (status != MXT_RELEASE) {
		input_report_abs(input_dev, ABS_X, finger[single_id].x);
		input_report_abs(input_dev, ABS_Y, finger[single_id].y);
		input_report_abs(input_dev,
				 ABS_PRESSURE, finger[single_id].pressure);
	}

	input_sync(input_dev);
}

static void mxt_input_touchevent(struct mxt_data *data,
				      struct mxt_message *message, int id)
{

	struct mxt_finger *finger = data->finger;
	struct device *dev = &data->client->dev;
	u8 status = message->message[0];
	int x;
	int y;
	int area;
	int pressure;
	//printk("[Atmel] mxt_input_touchevent status = %d\n",status);
	/* Check the touch is present on the screen */
	if (!(status & MXT_DETECT)) {
		if (status & MXT_RELEASE) {
			dev_dbg(dev, "[%d] released\n", id);

			finger[id].status = MXT_RELEASE;
			mxt_input_report(data, id);
		}
		return;
	}

	/* Check only AMP detection */
	if (!(status & (MXT_PRESS | MXT_MOVE)))
		return;

	x = (message->message[1] << 4) | ((message->message[3] >> 4) & 0xf);
	y = (message->message[2] << 4) | ((message->message[3] & 0xf));
	if (data->max_x < 1024)
		x = x >> 2;
	if (data->max_y < 1024)
		y = y >> 2;

	area = message->message[4];
	pressure = message->message[5];

	dev_dbg(dev, "[%d] %s x: %d, y: %d, area: %d\n", id,
		status & MXT_MOVE ? "moved" : "pressed",
		x, y, area);

	
	finger[id].status = status & MXT_MOVE ?
				MXT_MOVE : MXT_PRESS;
	finger[id].x = x;
	finger[id].y = y;
	finger[id].area = area;
	finger[id].pressure = pressure;

	mxt_input_report(data, id);
}

static irqreturn_t mxt_interrupt(int irq, void *dev_id)
{
	struct mxt_data *data = dev_id;
	struct mxt_message message;
	struct mxt_object *object;
	struct device *dev = &data->client->dev;
	int id;
	u8 reportid;
	u8 max_reportid;
	u8 min_reportid;
//	printk("[Atmel] mxt_interrupt \n");
	do {
//		printk("[Atmel] mxt_interrupt in do-while loop\n");
		if (mxt_read_message(data, &message)) {
			dev_err(dev, "Failed to read message\n");
			goto end;
		}

		reportid = message.reportid;

		/* whether reportid is thing of MXT_TOUCH_MULTI_T9 */
		object = mxt_get_object(data, MXT_TOUCH_MULTI_T9);
		if (!object)
			goto end;

		max_reportid = object->max_reportid;
		min_reportid = max_reportid - object->num_report_ids + 1;
		id = reportid - min_reportid;
		//printk("[Atmel] max_reportid = %d, min_reportid = %d, id = %d\n",max_reportid,min_reportid,id);
		if (reportid >= min_reportid && reportid <= max_reportid)
			mxt_input_touchevent(data, &message, id);
	

		/* whether reportid is thing of MXT_SPT_SELFTEST_T25 */
		/*object = mxt_get_object(data, MXT_SPT_SELFTEST_T25);
		if (!object)
                        goto end;
		max_reportid = object->max_reportid;
                min_reportid = max_reportid - object->num_report_ids + 1;
                id = reportid - min_reportid;
		if (reportid >= min_reportid && reportid <= max_reportid){
			if(message.message[0]==0xFE){	// Run all test OK !
				dev_info(dev,
                                        "maXTouch: Self-Test OK\n");		
				strcpy(self_test_result,"maXTouch: Self-Test OK");	
			}else{
				dev_err(dev,
                                	"maXTouch: Self-Test Failed [%02x]:"
	                                "{%02x,%02x,%02x,%02x,%02x}\n",
        	                        message.message[MXT_MSG_T25_STATUS],
                	                message.message[MXT_MSG_T25_STATUS + 0],
                        	        message.message[MXT_MSG_T25_STATUS + 1],
	                                message.message[MXT_MSG_T25_STATUS + 2],
        	                        message.message[MXT_MSG_T25_STATUS + 3],
                	                message.message[MXT_MSG_T25_STATUS + 4]
                                );	
				strcpy(self_test_result,"maXTouch: Self-Test Failed");
				
				if(message.message[0]==0xfd) 
					dev_err(dev,"maXTouch: The test code supplied in the CMD field is "
							"not associated with a valid test\n");
				if(message.message[0]==0xfc)
					dev_err(dev,"maXTouch: The test could not be completed due to an unrelated fault"
                                                        "(for example, an internal communications problem)\n");	
				if(message.message[0]==0x01)
					dev_err(dev,"maXTouch: AVdd is not present on at least one of the slave devices\n");
				if(message.message[0]==0x12)
                                        dev_err(dev,"maXTouch: The initial pin fault test failed following power-on or reset\n");
				if(message.message[0]==0x17)
                                        dev_err(dev,"maXTouch: The test failed because of a signal limit fault\n");
			}
		}*/else
                        mxt_dump_message(dev, &message);
		
	} while (reportid != 0xff);

end:
	return IRQ_HANDLED;
}

static int mxt_check_reg_init(struct mxt_data *data)
{
	printk(KERN_ERR "[Atmel] probe : mxt_check_reg_init\n");
	/*const struct mxt_platform_data *pdata = data->pdata;
	struct mxt_object *object;
	struct device *dev = &data->client->dev;
	int index = 0;
	int i, j, config_offset;


	if (!pdata->config) {
		printk(KERN_ERR "[Atmel] probe : mxt_check_reg_init : No cfg data defined, skipping reg init\n");
		dev_dbg(dev, "No cfg data defined, skipping reg init\n");
		return 0;
	}

	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;

		if (!mxt_object_writable(object->type))
			continue;

		for (j = 0;
		     j < (object->size + 1) * (object->instances + 1);
		     j++) {
			config_offset = index + j;
			if (config_offset > pdata->config_length) {
				dev_err(dev, "Not enough config data!\n");
				return -EINVAL;
			}
			mxt_write_object(data, object->type, j,
					 pdata->config[config_offset]);
		}
		index += (object->size + 1) * (object->instances + 1);
	}
	*/
	return 0;
}

static int mxt_make_highchg(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	struct mxt_message message;
	int count = 10;
	int error;

	/* Read dummy message to make high CHG pin */
	do {
		error = mxt_read_message(data, &message);
		if (error)
			return error;
	} while (message.reportid != 0xff && --count);

	if (!count) {
		dev_err(dev, "CHG pin isn't cleared\n");
		return -EBUSY;
	}

	return 0;
}

static void mxt_handle_pdata(struct mxt_data *data)
{
// Jui-Chuan Modified all "pdata" to "data" ++
//	const struct mxt_platform_data *pdata = data->pdata;
	u8 voltage;

	/* Set touchscreen lines */
	mxt_write_object(data, MXT_TOUCH_MULTI_T9, MXT_TOUCH_XSIZE,
			data->x_line);
	mxt_write_object(data, MXT_TOUCH_MULTI_T9, MXT_TOUCH_YSIZE,
			data->y_line);

	/* Set touchscreen orient */
	mxt_write_object(data, MXT_TOUCH_MULTI_T9, MXT_TOUCH_ORIENT,
			data->orient);

	/* Set touchscreen burst length */
	mxt_write_object(data, MXT_TOUCH_MULTI_T9,
			MXT_TOUCH_BLEN, data->blen);

	/* Set touchscreen threshold */
	mxt_write_object(data, MXT_TOUCH_MULTI_T9,
			MXT_TOUCH_TCHTHR, data->threshold);

	/* Set touchscreen resolution */
	mxt_write_object(data, MXT_TOUCH_MULTI_T9,
			MXT_TOUCH_XRANGE_LSB, (data->x_size - 1) & 0xff);
	mxt_write_object(data, MXT_TOUCH_MULTI_T9,
			MXT_TOUCH_XRANGE_MSB, (data->x_size - 1) >> 8);
	mxt_write_object(data, MXT_TOUCH_MULTI_T9,
			MXT_TOUCH_YRANGE_LSB, (data->y_size - 1) & 0xff);
	mxt_write_object(data, MXT_TOUCH_MULTI_T9,
			MXT_TOUCH_YRANGE_MSB, (data->y_size - 1) >> 8);

	/* Set touchscreen voltage */
	if (data->voltage) {
		if (data->voltage < MXT_VOLTAGE_DEFAULT) {
			voltage = (MXT_VOLTAGE_DEFAULT - data->voltage) /
				MXT_VOLTAGE_STEP;
			voltage = 0xff - voltage + 1;
		} else
			voltage = (data->voltage - MXT_VOLTAGE_DEFAULT) /
				MXT_VOLTAGE_STEP;

		mxt_write_object(data, MXT_SPT_CTECONFIG_T28,
				MXT_CTE_VOLTAGE, voltage);
	}
	
	//Jui added for 
	mxt_write_object(data, MXT_TOUCH_MULTI_T9,
			MXT_TOUCH_YEDGECTRL, 204);
	mxt_write_object(data, MXT_TOUCH_MULTI_T9,
			MXT_TOUCH_MOVHYSTN, 10);
// Jui-Chuan Modified all "pdata" to "data" --
}

static int mxt_get_info(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	struct mxt_info *info = &data->info;
	int error;
	u8 val;

	error = mxt_read_reg(client, MXT_FAMILY_ID, &val);
printk("[Atmel] mxt_get_info MXT_FAMILY_ID 0x%02x\n", val);
	if (error){
		queue_delayed_work(touch_chip->mxt_wq, &touch_chip->touch_chip_firmware_upgrade_work_if_fw_broken, 20*HZ);
		return error;
	}
	info->family_id = val;
	error = mxt_read_reg(client, MXT_VARIANT_ID, &val);
printk("[Atmel] mxt_get_info MXT_VARIANT_ID 0x%02x\n", val);
	if (error)
		return error;
	info->variant_id = val;
	error = mxt_read_reg(client, MXT_VERSION, &val);
printk("[Atmel] mxt_get_info MXT_VERSION 0x%02x\n", val);

	if(val == 17){
		queue_delayed_work(touch_chip->mxt_wq, &touch_chip->touch_chip_firmware_upgrade_work, 20*HZ);	
	}

	if (error)
		return error;
	info->version = val;
	error = mxt_read_reg(client, MXT_BUILD, &val);
printk("[Atmel] mxt_get_info MXT_BUILD 0x%02x\n", val);
	if (error)
		return error;
	info->build = val;
	error = mxt_read_reg(client, MXT_OBJECT_NUM, &val);
printk("[Atmel] mxt_get_info MXT_OBJECT_NUM 0x%02x\n", val);
	if (error)
		return error;

	info->object_num = val;

	return 0;
}

static int mxt_get_object_table(struct mxt_data *data)
{
	int error;
	int i;
	u16 reg;
	u8 reportid = 0;
	u8 buf[MXT_OBJECT_SIZE];

	for (i = 0; i < data->info.object_num; i++) {
		struct mxt_object *object = data->object_table + i;

		reg = MXT_OBJECT_START + MXT_OBJECT_SIZE * i;
		error = mxt_read_object_table(data->client, reg, buf);
		if (error)
			return error;

		object->type = buf[0];
		object->start_address = (buf[2] << 8) | buf[1];
		
#if 0		// Modified by Miracle. 2013/03/27. Refer to protocol guide.
		object->size = buf[3];
		object->instances = buf[4];
		object->num_report_ids = buf[5];
		if (object->num_report_ids) {
			reportid += object->num_report_ids *
					(object->instances + 1);
#else
		object->size = buf[3] + 1;
		object->instances = buf[4] + 1;
		object->num_report_ids = (u8)(object->instances * buf[5]);
		reportid += object->num_report_ids;

		printk("[Atmel] type = %d, size = %d, instances = %d, num_report = %d, report_id = %d\n", object->type, object->size,object->instances,object->num_report_ids,reportid);
#endif

			object->max_reportid = reportid;
		//}  // Modified by Jui-Chuan
	}

	return 0;
}

static int mxt_initialize(struct mxt_data *data)
{
	
	struct i2c_client *client = data->client;
	struct mxt_info *info = &data->info;
	int error;
	u8 val;
	printk("[Ateml] mxt1664 mxt_initialize\n");
	error = mxt_get_info(data);
	if (error)
		return error;
	printk("[Ateml] mxt1664 mxt_initialize mxt_get_info\n");
	data->object_table = kcalloc(info->object_num,
				     sizeof(struct mxt_object),
				     GFP_KERNEL);
	if (!data->object_table) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	/* Get object table information */
	error = mxt_get_object_table(data);
	if (error)
		return error;

	/* Check register init values */
	error = mxt_check_reg_init(data);
	if (error)
		return error;

	//mxt_handle_pdata(data);
	printk("[Ateml] mxt1664 mxt_initialize mxt_handle_pdata skip!\n");
	
//	if(after_update_fw == 1){
		mxt_config_init(data);
		after_update_fw = 0;
//	}
	/* Backup to memory */
	mxt_write_object(data, MXT_GEN_COMMAND_T6,
			MXT_COMMAND_BACKUPNV,
			MXT_BACKUP_VALUE);
	msleep(MXT_BACKUP_TIME);
	printk("[Ateml] mxt1664 mxt_initialize Backup to memory\n");
	/* Soft reset */
	mxt_write_object(data, MXT_GEN_COMMAND_T6,
			MXT_COMMAND_RESET, 1);
	//msleep(MXT_RESET_TIME);
	msleep(200);
	printk("[Ateml] mxt1664 mxt_initialize Soft reset\n");
	/* Update matrix size at info struct */
	error = mxt_read_reg(client, MXT_MATRIX_X_SIZE, &val);
	if (error)
		return error;
	info->matrix_xsize = val;
	printk("[Ateml] mxt1664 mxt_initialize Update matrix X size at info struct \n");	
	error = mxt_read_reg(client, MXT_MATRIX_Y_SIZE, &val);
	if (error)
		return error;
	info->matrix_ysize = val;
	printk("[Ateml] mxt1664 mxt_initialize Update matrix Y size at info struct\n");
	dev_info(&client->dev,
			"Family ID: %d Variant ID: %d Version: %d Build: %d\n",
			info->family_id, info->variant_id, info->version,
			info->build);

	dev_info(&client->dev,
			"Matrix X Size: %d Matrix Y Size: %d Object Num: %d\n",
			info->matrix_xsize, info->matrix_ysize,
			info->object_num);

	return 0;
}

static void mxt_calc_resolution(struct mxt_data *data)
{
	//Jui Modified all pdata to data ++ 
	//unsigned int max_x = data->pdata->x_size - 1; //original
	//unsigned int max_y = data->pdata->y_size - 1; //original
	unsigned int max_x = data->max_x - 1;
	unsigned int max_y = data->max_y - 1;
	
	if (data->orient & MXT_XY_SWITCH) {
		data->max_x = max_y;
		data->max_y = max_x;
	} else {
		data->max_x = max_x;
		data->max_y = max_y;
	}
	//Jui Modified all pdata to data --
}


static ssize_t mxt_Android_config(struct device *dev,
                    struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = touch_chip;
        mxt_config_init(data);
	printk("[Ateml] mxt1664 mxt_Android_config Change to Android Configuration\n");
        /* Backup to memory */
        mxt_write_object(data, MXT_GEN_COMMAND_T6,
                        MXT_COMMAND_BACKUPNV,
                        MXT_BACKUP_VALUE);
        msleep(MXT_BACKUP_TIME);
        printk("[Ateml] mxt1664 mxt_Android_config Backup to memory\n");
        /* Soft reset */
        mxt_write_object(data, MXT_GEN_COMMAND_T6,
                        MXT_COMMAND_RESET, 1);
        //msleep(MXT_RESET_TIME);
        msleep(200);
        printk("[Ateml] mxt1664 mxt_Android_config Soft reset\n");

}

static ssize_t mxt_int_show(struct device *dev,
                    struct device_attribute *attr, char *buf)
{
	int count = 0;
	int i;
#if 0		// 2013/05/03. Temperatory remove by Miracle.
	for(i=0;i<10000;i++)
	{
		int val = gpio_get_value(touch_chip->int_gpio);
		count += snprintf(buf + count, PAGE_SIZE - count,
                "touch int value = %d\n",val);
	}
#endif
	return count;

}


static ssize_t mxt_rst_show(struct device *dev,
                    struct device_attribute *attr, char *buf)
{
	int count = 0;
    int val = gpio_get_value(touch_chip->reset_gpio);
    count += snprintf(buf + count, PAGE_SIZE - count,
                "touch reset value = %d\n",val);
    return count;
}

static ssize_t mxt_object_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct mxt_object *object;
	int count = 0;
	int i, j;
	int error;
	u8 val;

	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;

		count += snprintf(buf + count, PAGE_SIZE - count,
				"Object[%d] (Type %d)\n",
				i + 1, object->type);
		if (count >= PAGE_SIZE)
			return PAGE_SIZE - 1;

		if (!mxt_object_readable(object->type)) {
			count += snprintf(buf + count, PAGE_SIZE - count,
					"\n");
			if (count >= PAGE_SIZE)
				return PAGE_SIZE - 1;
			continue;
		}

		// Modified by Miracle. 2013/03/27.
		// for (j = 0; j < object->size + 1; j++) {
		for (j = 0; j < object->size; j++) {
			error = mxt_read_object(data,
						object->type, j, &val);
			if (error)
				return error;

			count += snprintf(buf + count, PAGE_SIZE - count,
					"\t[%2d]: %02x (%d)\n", j, val, val);
			if (count >= PAGE_SIZE)
				return PAGE_SIZE - 1;
		}

		count += snprintf(buf + count, PAGE_SIZE - count, "\n");
		if (count >= PAGE_SIZE)
			return PAGE_SIZE - 1;
	}

	return count;
}

static int mxt_load_fw(struct device *dev, const char *fn)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	const struct firmware *fw = NULL;
	unsigned int frame_size;
	unsigned int pos = 0;
	int ret;
	int i;

	ret = request_firmware(&fw, fn, dev);
	if (ret) {
		dev_err(dev, "Unable to open firmware %s\n", fn);
		return ret;
	}

	/* Change to the bootloader mode */
	mxt_write_object(data, MXT_GEN_COMMAND_T6,
			MXT_COMMAND_RESET, MXT_BOOT_VALUE);
	//msleep(MXT_RESET_TIME);
	msleep(200);

	/* Change to slave address of bootloader */
	if (client->addr == MXT_APP_LOW)
		client->addr = MXT_BOOT_LOW;
	else
		client->addr = MXT_BOOT_HIGH;

	printk("[Atmel] slave address of bootloader = %x\n",client->addr);


	ret = mxt_check_bootloader(client, MXT_WAITING_BOOTLOAD_CMD);
	if (ret)
		goto out;

	/* Unlock bootloader */
	mxt_unlock_bootloader(client);

	while (pos < fw->size) {
#if 0	// Miracle. 2013/05/03. Double check wait frame data event.
		ret = mxt_check_bootloader(client,
						MXT_WAITING_FRAME_DATA);
		if (ret)
			goto out;
#else
		for(i=0; i < 3; i++) {
			ret = mxt_check_bootloader(client,
						MXT_WAITING_FRAME_DATA);
			if (!ret)
				break;
			msleep(500);
		}
		if(i==3)
			goto out;
#endif

		frame_size = ((*(fw->data + pos) << 8) | *(fw->data + pos + 1));

		/* We should add 2 at frame size as the the firmware data is not
		 * included the CRC bytes.
		 */
		frame_size += 2;

		printk("[Atmel] frame_size= %d bytes, fw->size= %zd bytes\n",frame_size,fw->size);


#if 0
		/* Write one frame to device */
		mxt_fw_write(client, fw->data + pos, frame_size);
//		msleep(150);	
		ret = mxt_check_bootloader(client,
						MXT_FRAME_CRC_PASS);

		if (ret)
			goto out;

		pos += frame_size;
#else
#define FRAME_SIZE		(60)
		for(;frame_size>0;) {
			/* Write one frame to device */
			if(frame_size > FRAME_SIZE) {
				mxt_fw_write(client, fw->data + pos, FRAME_SIZE);
				pos += FRAME_SIZE;
				frame_size -= FRAME_SIZE;
			}
			else {
				mxt_fw_write(client, fw->data + pos, frame_size);
				pos += frame_size;
				frame_size  = 0;
			}
//			msleep(50);	
		}

//		msleep(50);	
		ret = mxt_check_bootloader(client,
						MXT_FRAME_CRC_PASS);
		msleep(10);	

		if (ret)
			goto out;

#endif
		printk("[Atmel] Updated %d bytes / %zd bytes\n", pos, fw->size);

		dev_dbg(dev, "Updated %d bytes / %zd bytes\n", pos, fw->size);
	}

out:
	release_firmware(fw);


	printk("[Atmel] Do soft reset\n");
	/* Soft reset */
    mxt_write_object(data, MXT_GEN_COMMAND_T6,
            MXT_COMMAND_RESET, 1);
    //msleep(MXT_RESET_TIME);
    msleep(200);


	/* Change to slave address of application */
	if (client->addr == MXT_BOOT_LOW)
		client->addr = MXT_APP_LOW;
	else
		client->addr = MXT_APP_HIGH;

	printk("[Atmel] slave address of application = %x\n",client->addr);

	return ret;
}

static ssize_t mxt_update_fw_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int error;

	wake_lock(&data->wakelock);
	data->tp_firmware_upgrade_proceed = 1;


	disable_irq(data->irq);

	error = mxt_load_fw(dev, MXT_FW_NAME);
	if (error) {
		dev_err(dev, "The firmware update failed(%d)\n", error);
		count = error;
	} else {
		dev_dbg(dev, "The firmware update succeeded\n");

		/* Wait for reset */
		msleep(MXT_FWRESET_TIME);

		kfree(data->object_table);
		data->object_table = NULL;
		after_update_fw = 1;
		mxt_initialize(data);
	}

	enable_irq(data->irq);	

	data->tp_firmware_upgrade_proceed = 0;
        wake_unlock(&data->wakelock);

	error = mxt_make_highchg(data);
	if (error)
		return error;

	return count;
}

// Jui-Chuan add for Self test ++
static ssize_t mxt_self_test(struct device *dev,struct device_attribute *attr, char *buf){
	struct mxt_data *data = touch_chip;
        struct mxt_message message;
        struct mxt_object *object;
        
        int id;
        u8 reportid;
        u8 max_reportid;
        u8 min_reportid;
	
	int retval = 0;
	int error = 0;
	
	//printf("Start self test\n");
	/* Self test */

	disable_irq(touch_chip->irq); 
		

	retval = mxt_write_object(data, MXT_SPT_SELFTEST_T25, MXT_ADR_T25_CTRL, 0x03); // Enable and report message
	if (retval < 0)
			return -1;
	retval = mxt_write_object(data, MXT_SPT_SELFTEST_T25, MXT_ADR_T25_CMD, MXT_MSGR_T25_RUN_ALL_TESTS); // All test
	if (retval < 0)
                        return -1;


	while(true) {
                
		printk("[Atmel] touch_chip->int_gpio = %d\n",gpio_get_value(touch_chip->int_gpio));
		if(gpio_get_value(touch_chip->int_gpio)!=0){
			dev_err(dev, "Wait for low int_gpio\n");
                        continue;
		}

		if (mxt_read_message(data, &message)) {
                        dev_err(dev, "Failed to read message\n");
                        break;
                }

                reportid = message.reportid;
                /* whether reportid is thing of MXT_SPT_SELFTEST_T25 */
                object = mxt_get_object(data, MXT_SPT_SELFTEST_T25);
                if (!object)
                     break;
                max_reportid = object->max_reportid;
                min_reportid = max_reportid - object->num_report_ids + 1;
                id = reportid - min_reportid;
                if (reportid >= min_reportid && reportid <= max_reportid){
                        if(message.message[0]==0xFE){   // Run all test OK !
                                dev_info(dev,
                                        "maXTouch: Self-Test OK\n");
                                strcpy(self_test_result,"maXTouch: Self-Test OK");
                        }else{
                                dev_err(dev,
                                        "maXTouch: Self-Test Failed [%02x]:"
                                        "{%02x,%02x,%02x,%02x,%02x}\n",
                                        message.message[MXT_MSG_T25_STATUS],
                                        message.message[MXT_MSG_T25_STATUS + 0],
                                        message.message[MXT_MSG_T25_STATUS + 1],
                                        message.message[MXT_MSG_T25_STATUS + 2],
                                        message.message[MXT_MSG_T25_STATUS + 3],
                                        message.message[MXT_MSG_T25_STATUS + 4]
                                );
                                strcpy(self_test_result,"maXTouch: Self-Test Failed");

                                if(message.message[0]==0xfd)
                                        dev_err(dev,"maXTouch: The test code supplied in the CMD field is "
                                                        "not associated with a valid test\n");
                                if(message.message[0]==0xfc)
                                        dev_err(dev,"maXTouch: The test could not be completed due to an unrelated fault"
                                                        "(for example, an internal communications problem)\n");
                                if(message.message[0]==0x01)
                                        dev_err(dev,"maXTouch: AVdd is not present on at least one of the slave devices\n");
                                if(message.message[0]==0x12)
                                        dev_err(dev,"maXTouch: The initial pin fault test failed following power-on or reset\n");
                                if(message.message[0]==0x17)
                                        dev_err(dev,"maXTouch: The test failed because of a signal limit fault\n");
                        }
                }else
                        mxt_dump_message(dev, &message);

		break;
        }

	
	int count = 0;
        count += snprintf(buf + count, PAGE_SIZE - count, "%s\n",self_test_result);
        if (count >= PAGE_SIZE)
                        return PAGE_SIZE - 1;

	enable_irq(touch_chip->irq);

	error = mxt_make_highchg(data);
        if (error)
                return error;

        return count;	

}

// Jui-Chuan add for Self test --

// Jui-Chuan add for fine tune ++
static ssize_t mxt_object_write(struct device *dev,
                                        struct device_attribute *attr,
                                        const char *buf, size_t count){
	int object = 0;
	int offset = 0;
	int mesg = 0;
	struct mxt_data *data = touch_chip;
	int i;
	int c = 0;
	int index = 0;
	char** temp = (char**)kcalloc(3,sizeof(char*),GFP_KERNEL);
	for(i=0;i<3;i++) temp[i] = (char*)kcalloc(10,sizeof(char),GFP_KERNEL);
	
	i=0;
	while(index<=count){
		if(buf[index] != '+' && buf[index] != '\n') c++;
		else{
			strncpy(temp[i],buf+index-c,c);
			printk("temp[%d] = %s\n",i,temp[i]);
			i++;
			c = 0;
		}
		index++;
	}

	printk("i=%d  index=%d\n",i,index);
	sscanf(temp[0],"%d",&object);
	sscanf(temp[1],"%d",&offset);
	sscanf(temp[2],"%x",&mesg);

	//mxt_write_reg(data->client, MXT_BASE_ADDR(object, data) + offset, mesg);	

	mxt_write_object(data, object, offset, mesg);

	/* Backup to memory */
        mxt_write_object(data, MXT_GEN_COMMAND_T6,
                        MXT_COMMAND_BACKUPNV,
                        MXT_BACKUP_VALUE);
        msleep(MXT_BACKUP_TIME);


	/* Soft reset */
        mxt_write_object(data, MXT_GEN_COMMAND_T6,
                        MXT_COMMAND_RESET, 1);
        //msleep(MXT_RESET_TIME);
        msleep(200);

	for(i=0;i<3;i++) kfree(temp[i]);
	kfree(temp);

	return count;
}

// Jui-Chuan add for fine tune --

static ssize_t mxt_get_fw_ver(struct device *dev,struct device_attribute *attr, char *buf){
        struct mxt_data *data = touch_chip;
	u8 val = 0;
	int error = 0;	
	error = mxt_read_reg(data->client, MXT_VERSION, &val);

	int count = 0;
        count += snprintf(buf + count, PAGE_SIZE - count, "%x\n",val);
        if (count >= PAGE_SIZE)
                        return PAGE_SIZE - 1;

	return count;

}



static DEVICE_ATTR(get_fw_ver, 0444, mxt_get_fw_ver, NULL);


static DEVICE_ATTR(Change_to_Android, 0444, mxt_Android_config, NULL);

static DEVICE_ATTR(INT_value, 0444, mxt_int_show, NULL);
static DEVICE_ATTR(RST_value, 0444, mxt_rst_show, NULL);

static DEVICE_ATTR(object, 0444, mxt_object_show, NULL);
static DEVICE_ATTR(update_fw, 0664, NULL, mxt_update_fw_store);
// Jui-Chuan add for self test ++
static DEVICE_ATTR(self_test, 0444, mxt_self_test, NULL);
// Jui-Chuan add for self test --
// Jui-Chuan add for fine tune ++
static DEVICE_ATTR(write_object, 0664, NULL, mxt_object_write);
// Jui-Chuan add for fine tune --


static struct attribute *mxt_attrs[] = {
	&dev_attr_get_fw_ver.attr,

	&dev_attr_Change_to_Android.attr,
	&dev_attr_object.attr,
	&dev_attr_INT_value.attr,
	&dev_attr_RST_value.attr,
	&dev_attr_update_fw.attr,
	// Jui-Chuan add for self test ++
	&dev_attr_self_test.attr,
	// Jui-Chuan add for self test --
	// Jui-Chuan add for fine tune ++
	&dev_attr_write_object.attr,
	// Jui-Chuan add for fine tune --
	NULL
};

static const struct attribute_group mxt_attr_group = {
	.attrs = mxt_attrs,
};

// Jui-Chuan add for read write block ++
static int mxt_read_block(struct i2c_client *client,
                   u16 addr,
                   u16 length,
                   u8 *value)
{
        struct i2c_adapter *adapter = client->adapter;
        struct i2c_msg msg[2];
        __le16  le_addr;
        struct mxt_data *mxt;

        mxt = i2c_get_clientdata(client);

        if (mxt != NULL) {
                if ((mxt->last_read_addr == addr) &&
                        (addr == mxt->msg_proc_addr)) {
                        if  (i2c_master_recv(client, value, length) == length)
                                return length;
                        else
                                return -EIO;
                } else {
                        mxt->last_read_addr = addr;
                }
        }

        //mxt_debug(DEBUG_TRACE, "Writing address pointer & reading %d bytes "
        //        "in on i2c transaction...\n", length);

        le_addr = cpu_to_le16(addr);
        msg[0].addr  = client->addr;
        msg[0].flags = 0x00;
        msg[0].len   = 2;
        msg[0].buf   = (u8 *) &le_addr;

        msg[1].addr  = client->addr;
        msg[1].flags = I2C_M_RD;
        msg[1].len   = length;
        msg[1].buf   = (u8 *) value;
        if  (i2c_transfer(adapter, msg, 2) == 2)
                return length;
        else
                return -EIO;

}

static int mxt_write_block(struct i2c_client *client,
                    u16 addr,
                    u16 length,
                    u8 *value)
{
        int i;
        struct {
                __le16  le_addr;
                u8      data[256];

        } i2c_block_transfer;

        struct mxt_data *mxt;

        //mxt_debug(DEBUG_TRACE, "Writing %d bytes to %d...", length, addr);
        if (length > 256)
                return -EINVAL;
        mxt = i2c_get_clientdata(client);
        if (mxt != NULL)
                mxt->last_read_addr = -1;
        for (i = 0; i < length; i++)
                i2c_block_transfer.data[i] = *value++;
        i2c_block_transfer.le_addr = cpu_to_le16(addr);
        i = i2c_master_send(client, (u8 *) &i2c_block_transfer, length + 2);
        if (i == (length + 2))
                return length;
        else
                return -EIO;
}
// Jui-Chuan add for read write block --

// Jui-Chuan add for calibration ++
#define	MXT_USER_INFO_T38 38
#define MXT_BOOTLOADER_ADDRESS 0x25  //Refer to QTAN0051

static bool isInBootLoaderMode(struct i2c_client *client){
    	u8 buf[2];
	int ret;
	int identified;
	int retry = 2;
	int times;
	struct i2c_msg rmsg;

	rmsg.addr = MXT_BOOTLOADER_ADDRESS;
	rmsg.flags = I2C_M_RD;
	rmsg.len = 2;
	rmsg.buf = buf;
	
    /* Read 2 byte from boot loader I2C address to make sure touch chip is in bootloader mode */   
	for(times = 0; times < retry; times++ ){
	     ret = i2c_transfer(client->adapter, &rmsg, 1); 
	     if(ret >= 0)
		 	break;
		 	  	 
	     mdelay(25);
	}
	dev_err(&client->dev, "The touch is %s in bootloader mode.\n", (ret < 0 ? "not" : "indeed"));
	return ret >= 0;
}
#define MXT_ID_BLOCK_SIZE 7
#define I2C_M_WR 0
/* Fixed addresses inside maXTouch device */
#define	MXT_ADDR_INFO_BLOCK				0
#define	MXT_ADDR_OBJECT_TABLE				7
#define MXT_ID_BLOCK_SIZE                               7
#define	MXT_OBJECT_TABLE_ELEMENT_SIZE			6
static int recovery_from_bootMode(struct i2c_client *client){
    u8 buf[MXT_ID_BLOCK_SIZE];
    int ret;
    int identified;
    int retry = 40;
    int times;
    unsigned char data[] = {0x01, 0x01};
    struct i2c_msg wmsg;
    
    wmsg.addr = MXT_BOOTLOADER_ADDRESS;
    wmsg.flags = I2C_M_WR;
    wmsg.len = 2;
    wmsg.buf = data;
    dev_err(&client->dev, "---------Touch: Try to leave the bootloader mode!\n");
	/*Write two nosense bytes to I2C address "0x35" in order to force touch to leave the bootloader mode.*/
    i2c_transfer(client->adapter, &wmsg, 1);
    mdelay(10);
	
    /* Read Device info to check if chip is valid */
    for(times = 0; times < retry; times++ ){
        ret = mxt_read_block(client, MXT_ADDR_INFO_BLOCK, MXT_ID_BLOCK_SIZE, (u8 *) buf); 
	  if(ret >= 0)
	      break;

	  dev_err(&client->dev, "Retry addressing I2C address 0x%02X with %d times\n", client->addr,times+1); 	 
	  msleep(25);
    }	
	
    if(ret >= 0){
        dev_err(&client->dev, "---------Touch: Successfully leave the bootloader mode!\n");
		ret = 0;
    }    
    return ret;
}

// Jui-Chuan add for calibration --

static void mxt_start(struct mxt_data *data)
{
	// Jui-Chuan add for calibration ++ 
	char buf[2];
      	int wait_count = 10;
      	int error;
	//dummy read to wake up from deep sleep mode
	mxt_read_block(data->client, MXT_BASE_ADDR(MXT_SPT_COMMSCONFIG_T18, data), 1, buf);
	mdelay(25);
	error = mxt_read_block(data->client, MXT_BASE_ADDR(MXT_USER_INFO_T38, data), 1, buf);

	// Hardware reset ++
	printk("[Atmel] Do hardware reset\n");
	if(error < 0){
        	error = gpio_direction_output(data->reset_gpio, 0);
        	if (error){
                	printk("[Atmel] %s:Failed to set reset direction, error=%d\n", __func__, error);
                	gpio_free(data->reset_gpio);
        	}

        	msleep(1);
        	gpio_set_value(data->reset_gpio, 1);
        	msleep(100);
	}
        // Hardware reset --
	
	printk("[Atmel] Is in Bootloader Mode ?\n");
	if(error < 0 && isInBootLoaderMode(data->client)) // start boot loader recovery mode
            recovery_from_bootMode(data->client);	

	// Resume from Deep sleep mode 
	//mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_POWER_T7, mxt), 0x0F);
	//mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_POWER_T7, mxt)+1, 0xFF);
	mxt_write_reg(data->client, MXT_BASE_ADDR(MXT_GEN_POWER_T7, data), 0x0F);
        mxt_write_reg(data->client, MXT_BASE_ADDR(MXT_GEN_POWER_T7, data)+1, 0xFF);

	//mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt), 0);
	mxt_write_reg(data->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, data), 0);

	// Calibration
	printk("[Atmel] Do Calibration Start\n");	
	//mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRE_T8, mxt) + 6, 0x05);
	//mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRE_T8, mxt) + 7, 0x50);
	//mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRE_T8, mxt) + 8, 0x06);
	//mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRE_T8, mxt) + 9, 0xC0);

	mxt_write_reg(data->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRE_T8, data) + 6, 0x05);
        mxt_write_reg(data->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRE_T8, data) + 7, 0x50);
        mxt_write_reg(data->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRE_T8, data) + 8, 0x06);
        mxt_write_reg(data->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRE_T8, data) + 9, 0xC0);
	printk("[Atmel] Do Calibration End\n");

	// Jui-Chuan add for calibration --



	/* Touch enable */
	mxt_write_object(data,
			MXT_TOUCH_MULTI_T9, MXT_TOUCH_CTRL, 0x83);
}

static void mxt_stop(struct mxt_data *data)
{
	/* Touch disable */
	mxt_write_object(data,
			MXT_TOUCH_MULTI_T9, MXT_TOUCH_CTRL, 0);

	// Jui-Chuan add for Deep sleep mode ++

	// Go into Deep sleep Mode
	//mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_POWER_T7, mxt), 0);
	//mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_POWER_T7, mxt) + 1, 0);

	mxt_write_reg(data->client, MXT_BASE_ADDR(MXT_GEN_POWER_T7, data), 0);
        mxt_write_reg(data->client, MXT_BASE_ADDR(MXT_GEN_POWER_T7, data) + 1, 0);

	// Jui-Chuan add for Deep sleep mode --
}

static int mxt_input_open(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);

	mxt_start(data);

	return 0;
}

static void mxt_input_close(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);

	mxt_stop(data);
}


// Jui-Chuan add for raw data ++
#define MXT_ADR_T6_DIAGNOSTIC                           0x05
/* SPT_DEBUG_DIAGNOSTIC_T37 Register offsets from T37 base address */
#define MXT_ADR_T37_PAGE                                0x01
#define	MXT_ADR_T37_DATA				0x02
/* T6 Debug Diagnostics Commands */
#define	MXT_CMD_T6_PAGE_UP          0x01
#define	MXT_CMD_T6_PAGE_DOWN        0x02
#define	MXT_CMD_T6_DELTAS_MODE      0x10
#define	MXT_CMD_T6_REFERENCES_MODE  0x11
#define	MXT_CMD_T6_CTE_MODE         0x31
ssize_t debug_data_read(struct mxt_data *mxt, char *buf, size_t count,
                        loff_t *ppos, u8 debug_command)
{
        int i;
        u16 *data;
        u16 diagnostics_reg;
        int offset = 0;
        int size;
        int read_size;
        int error;
        char *buf_start;
        u16 debug_data_addr;
        u16 page_address;
        u8 page;
        u8 debug_command_reg;

	char *temp_buf = kmalloc(PAGE_SIZE, GFP_KERNEL);        
	ssize_t temp_len, temp_ret = 0;


        data = mxt->debug_data;
        if (data == NULL)
                return -EIO;

	printk("[Atmel] debug hello !\n");
	printk("[Atmel] debug counts = %d!\n", count);

	

        /* If first read after open, read all data to buffer. */
        if (mxt->current_debug_datap == 0) {
                // Jui-Chuan Modified for raw data ++
		diagnostics_reg =
                        MXT_BASE_ADDR(MXT_GEN_COMMAND_T6, mxt) +
                        MXT_ADR_T6_DIAGNOSTIC;
                if (count > (mxt->num_nodes * 2))
                        count = mxt->num_nodes;
                debug_data_addr =
                        MXT_BASE_ADDR(MXT_DEBUG_DIAGNOSTIC_T37, mxt) +
                        MXT_ADR_T37_DATA;
                page_address = MXT_BASE_ADDR(MXT_DEBUG_DIAGNOSTIC_T37, mxt) +
                               MXT_ADR_T37_PAGE; 
		
		
/*		struct mxt_object *temp_object;	
		u16 temp_reg;
		temp_object = mxt_get_object(mxt,MXT_GEN_COMMAND_T6);
		temp_reg = temp_object->start_address;	
		diagnostics_reg = temp_reg + MXT_ADR_T6_DIAGNOSTIC;

		if (count > (mxt->num_nodes * 2))
                        count = mxt->num_nodes;

		temp_object = mxt_get_object(mxt,MXT_DEBUG_DIAGNOSTIC_T37);
                temp_reg = temp_object->start_address;
		debug_data_addr = temp_reg + MXT_ADR_T37_DATA;
		page_address = temp_reg + MXT_ADR_T37_PAGE;
*/
		// Jui-Chuan Modified for raw data --
		printk("[Atmel] debug setting \n");
                // error = mxt_read_block(mxt->client, page_address, 1, &page);
		error = mxt_read_reg(mxt->client,page_address,&page);
                if (error < 0)
                        return error;
                //mxt_debug(DEBUG_TRACE, "debug data page = %d\n", page);
		

		printk("[Atmel] page = %d, debug_command = %d\n",page,debug_command);
		
		
                while (page != 0) {
                        //error = mxt_write_byte(mxt->client,
                        //                diagnostics_reg,
                        //                MXT_CMD_T6_PAGE_DOWN);
			//printk("[Atmel] in while loop start\n");	
			
			error = mxt_write_reg(mxt->client,
                                        diagnostics_reg,
                                        MXT_CMD_T6_PAGE_DOWN);

                        if (error < 0)
                                return error;
                        /* Wait for command to be handled; when it has, the
                           register will be cleared. */
                        debug_command_reg = 1;
			
                        while (debug_command_reg != 0) {
                                error = mxt_read_block(mxt->client,
                                                diagnostics_reg, 1,
                                                &debug_command_reg);

				//error = mxt_read_reg(mxt->client,
                                //                diagnostics_reg,
                                //                &debug_command_reg);

                                if (error < 0)
                                        return error;
                                //mxt_debug(DEBUG_TRACE,
                                //        "Waiting for debug diag command "
                                //        "to propagate...\n");
				
                        }
                        error = mxt_read_block(mxt->client, page_address, 1,
                                              &page);
			//error = mxt_read_reg(mxt->client, page_address,&page);

                        if (error < 0)
                                return error;
                        //mxt_debug(DEBUG_TRACE, "debug data page = %d\n", page);
			printk("[Atmel] in while loop\n");
                }

                /*
                 * Lock mutex to prevent writing some unwanted data to debug
                 * command register. User can still write through the char
                 * device interface though. TODO: fix?
                 */

                //mutex_lock(&mxt->debug_mutex);
                /* Configure Debug Diagnostics object to show deltas/refs */
                //error = mxt_write_byte(mxt->client, diagnostics_reg,
                //                debug_command);
	
		printk("[Atmel] debug_command = %d\n");
		
	
		error = mxt_write_reg(mxt->client, diagnostics_reg,
                                debug_command);
                /* Wait for command to be handled; when it has, the
                 * register will be cleared. */
                debug_command_reg = 1;
                while (debug_command_reg != 0) {
                        error = mxt_read_block(mxt->client,
                                        diagnostics_reg, 1,
                                        &debug_command_reg);
			//error = mxt_read_reg(mxt->client,
                        //                diagnostics_reg,
                        //                &debug_command_reg);

                        if (error < 0)
                                return error;
                        //mxt_debug(DEBUG_TRACE, "Waiting for debug diag command "
                        //        "to propagate...\n");
                }

                if (error < 0) {
                        printk(KERN_WARNING
                                "Error writing to maXTouch device!\n");
                        return error;
                }

                size = mxt->num_nodes * sizeof(u16);

		

                while (size > 0) {
                        read_size = size > 128 ? 128 : size;
			//read_size = 2;
                        //mxt_debug(DEBUG_TRACE,
                        //        "Debug data read loop, reading %d bytes...\n",
                        //        read_size);
                        error = mxt_read_block(mxt->client,
                                               debug_data_addr,
                                               read_size,
                                               (u8 *) &data[offset]);
			//error = mxt_read_reg(mxt->client,
                        //                       debug_data_addr,
                        //                       (u8 *) &data[offset]);

                        if (error < 0) {
                                printk(KERN_WARNING
                                        "Error reading debug data\n");
                                goto error;
                        }
                        offset += read_size/2;
                        size -= read_size;

                        /* Select next page */
                        //error = mxt_write_byte(mxt->client, diagnostics_reg,
                        //                MXT_CMD_T6_PAGE_UP);
			error = mxt_write_reg(mxt->client, diagnostics_reg,
                                        MXT_CMD_T6_PAGE_UP);
                        if (error < 0) {
                                printk(KERN_WARNING
                                        "Error writing to maXTouch device!\n");
                                goto error;
                        }
                }
                //mutex_unlock(&mxt->debug_mutex);
        }

        buf_start = buf;
        i = mxt->current_debug_datap;

        while (((buf - buf_start) < (count - 6)) &&
                (i < mxt->num_nodes)) {

                mxt->current_debug_datap++;
                if (debug_command == MXT_CMD_T6_REFERENCES_MODE)
                        buf += sprintf(buf, "%d: %5d\n", i,
                                       (u16) le16_to_cpu(data[i]));
                else if (debug_command == MXT_CMD_T6_DELTAS_MODE)
                        buf += sprintf(buf, "%d: %5d\n", i,
                                       (s16) le16_to_cpu(data[i]));
                i++;
        }

        return buf - buf_start;
error:
        mutex_unlock(&mxt->debug_mutex);
        return error;
}

ssize_t deltas_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
        return debug_data_read(file->private_data, buf, count, ppos,
                               MXT_CMD_T6_DELTAS_MODE);
}

ssize_t refs_read(struct file *file, char *buf, size_t count,
                        loff_t *ppos)
{
        return debug_data_read(file->private_data, buf, count, ppos,
                               MXT_CMD_T6_REFERENCES_MODE);
}

int debug_data_open(struct inode *inode, struct file *file)
{

        struct mxt_data *mxt;
        int i;
        mxt = inode->i_private;
        if (mxt == NULL)
                return -EIO;
        mxt->current_debug_datap = 0;
	
        //mxt->debug_data = kmalloc(mxt->device_info.num_nodes * sizeof(u16),
        //                          GFP_KERNEL);
	mxt->debug_data = kmalloc(mxt->num_nodes * sizeof(u16),
                                  GFP_KERNEL);
        if (mxt->debug_data == NULL)
                return -ENOMEM;
        //for (i = 0; i < mxt->device_info.num_nodes; i++)
	for (i = 0; i < mxt->num_nodes; i++)
                mxt->debug_data[i] = 7777;
        file->private_data = mxt;
	printk("[Atmel] debug_data_open\n");
        return 0;
}

int debug_data_release(struct inode *inode, struct file *file)
{
        struct mxt_data *mxt;
        mxt = file->private_data;
        kfree(mxt->debug_data);
	printk("[Atmel] debug_data_release\n");
        return 0;
}

static const struct file_operations delta_fops = {
        .owner = THIS_MODULE,
        .open = debug_data_open,
        .release = debug_data_release,
        .read = deltas_read,
};

static const struct file_operations refs_fops = {
        .owner = THIS_MODULE,
        .open = debug_data_open,
        .release = debug_data_release,
        .read = refs_read,
};
// Jui-Chuan add for raw data --

// Jui-Chuan add for read write memory ++

static int touch_chip_self_firmware_upgrade(struct work_struct *dat)
{
        struct i2c_client *client;
        client = touch_chip->client;

        int try_again = 2;
        int error = 0;


//        client->addr = 0x27;
//        error = mxt_check_bootloader(client, MXT_WAITING_BOOTLOAD_CMD);

            printk("[Atmel] We try FW update\n");
            struct device *dev = &client->dev;

            wake_lock(&touch_chip->wakelock);
            touch_chip->tp_firmware_upgrade_proceed = 1;
            disable_irq(touch_chip->irq);


            error = mxt_load_fw(dev, MXT_FW_NAME);
            while (error!=0 && try_again!=0) {
                try_again--;
                printk("[Atmel] try again %d\n",try_again);
                msleep(500);
                error = mxt_load_fw(dev, MXT_FW_NAME);
            }

            if (error) {
                    printk("[Atmel] The firmware update failed(%d)\n", error);
                    

            } else {
                printk("[Atmel] The firmware update succeeded\n");
                msleep(MXT_FWRESET_TIME);
                kfree(touch_chip->object_table);
                touch_chip->object_table = NULL;
                //after_update_fw = 1;
                mxt_initialize(touch_chip);
                //mxt_probe(client,mxt_id);

            }


            enable_irq(touch_chip->irq);
            touch_chip->tp_firmware_upgrade_proceed = 0;
            wake_unlock(&touch_chip->wakelock);

                msleep(200);

//        client->addr = 0x4b;
        return 0;
}


static int touch_chip_self_firmware_upgrade_if_fw_broken(struct work_struct *dat)
{
	struct i2c_client *client;	
	client = touch_chip->client; 

	int try_again = 2;
	int error = 0;

        printk("[Atmel] Check is in bootloader mode\n");
        client->addr = 0x27;
//        error = mxt_check_bootloader(client, MXT_WAITING_BOOTLOAD_CMD);

        u8 buf[2];
        buf[0] = 0x00;
        buf[1] = 0x00;
	if (i2c_master_send(client, buf, 2) == 2) {

            printk("[Atmel] We try FW update\n");
            struct device *dev = &client->dev;

            wake_lock(&touch_chip->wakelock);
            touch_chip->tp_firmware_upgrade_proceed = 1;
            disable_irq(touch_chip->irq);
	    
	    error = mxt_load_fw(dev, MXT_FW_NAME);
            while (error!=0 && try_again!=0) {
		try_again--;
		printk("[Atmel] try again %d\n",try_again);
		msleep(500);
		error = mxt_load_fw(dev, MXT_FW_NAME);
	    }
	    
            if (error) {
                    printk("[Atmel] The firmware update failed(%d)\n", error);
		    return -1;

            } else {
                printk("[Atmel] The firmware update succeeded\n");
                msleep(MXT_FWRESET_TIME);
                kfree(touch_chip->object_table);
                touch_chip->object_table = NULL;
                //after_update_fw = 1;
                mxt_initialize(touch_chip);
		//mxt_probe(client,mxt_id);

            }
            enable_irq(touch_chip->irq);
	    touch_chip->tp_firmware_upgrade_proceed = 0;
       	    wake_unlock(&touch_chip->wakelock);

		msleep(200);
        }
        client->addr = 0x4b;
	return 0;        
}


#define PASS_PROBE_ERROR 0
static int __devinit mxt_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{

	printk("Atmel mxt1664s probe start ! \n");
	printk("[Atmel][1] touch int value = %d\n",gpio_get_value(62));

	after_update_fw = 0;	
	
	//const struct mxt_platform_data *pdata = client->dev.platform_data;
	struct mxt_data *data;
	struct input_dev *input_dev;
	int error;

	// Jui-Chuan for pdata ++
	//if (!pdata)
	//	return -EINVAL;
	// Jui-Chuan for pdata --	

	touch_chip = kzalloc(sizeof(struct mxt_data), GFP_KERNEL); // allocate memory for touch_chip
	data = touch_chip;


	touch_chip->mxt_wq = create_singlethread_workqueue("mxt_wq");
	INIT_DELAYED_WORK(&touch_chip->touch_chip_firmware_upgrade_work, touch_chip_self_firmware_upgrade);
	INIT_DELAYED_WORK(&touch_chip->touch_chip_firmware_upgrade_work_if_fw_broken, touch_chip_self_firmware_upgrade_if_fw_broken);
//	queue_delayed_work(touch_chip->mxt_wq, &touch_chip->touch_chip_firmware_upgrade_work, 20*HZ);

	input_dev = input_allocate_device();
	if (!data || !input_dev) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		error = -ENOMEM;
		goto err_free_mem;
	}

	input_dev->name = "Atmel-touchscreen";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
	input_dev->open = mxt_input_open;
	input_dev->close = mxt_input_close;

	data->client = client;
	data->input_dev = input_dev;
	// Jui-Chuan for pdata ++
	//data->pdata = pdata;
	// Jui-Chuan for pdata --

	// Configuration from Appendix B Reference Configuration ++
	data->max_x = 4095;
	data->max_y = 4095;
	data->x_line = 30; //
	data->y_line = 52; //
	data->x_size = 50;
	data->y_size = 50;
	data->blen = 110;   //
	data->threshold = 50; //
	data->voltage = MXT_VOLTAGE_DEFAULT;
	data->orient = 1;

	data->numtouch = 10;
	// Configuration from Appendix B Reference Configuration --

	// Jui-Chuan for pdata ++
	data->reset_gpio = 88;
	data->int_gpio = 62;
	// Jui-Chuan for pdata --

	data->num_nodes = data->x_line * data->y_line;
	
        //--------------------------------------------------------------------------------------------
	/*init INTERRUPT pin*/
	error = gpio_request(data->int_gpio, "AtmelTouch-irq");
	if(error < 0)
		printk("[Atmel] %s:Failed to request GPIO%d (ElanTouch-interrupt) error=%d\n", __func__, data->int_gpio, error);

	error = gpio_direction_input(data->int_gpio);
	if (error){
		printk("[Atmel] %s:Failed to set interrupt direction, error=%d\n", __func__, error);
		gpio_free(data->int_gpio);
	}
	

	data->irq = gpio_to_irq(data->int_gpio);
	printk("[Atmel] %s: irq=%d \n", __func__, data->irq);
	
	/*init RESET pin*/
	error = gpio_request(data->reset_gpio, "AtmelTouch-reset");
	if (error < 0)
		printk("[Atmel] %s:Failed to request GPIO%d (ElanTouch-reset) error=%d\n", __func__, data->reset_gpio, error);

	// Hardware reset ++
	error = gpio_direction_output(data->reset_gpio, 0);
	if (error){
		printk("[Atmel] %s:Failed to set reset direction, error=%d\n", __func__, error);
		gpio_free(data->reset_gpio);
	}

	msleep(1);
	gpio_set_value(data->reset_gpio, 1);
	msleep(100);
	// Hardware reset --
        //--------------------------------------------------------------------------------------------
	printk("[Ateml] mxt1664 probe init INTERRUPT pin\n");

	client->irq = data->irq;

	
	mxt_calc_resolution(data);
	printk("[Ateml] mxt1664 probe mxt_calc_resolution\n");

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);

	/* For single touch */
	input_set_abs_params(input_dev, ABS_X,
			     0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_Y,
			     0, data->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE,
			     0, 255, 0, 0);

	/* For multi touch */
	input_mt_init_slots(input_dev, MXT_MAX_FINGER);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
			     0, MXT_MAX_AREA, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, data->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE,
			     0, 255, 0, 0);

	printk("[Ateml] mxt1664 probe set abs params\n");
	input_set_drvdata(input_dev, data);
	i2c_set_clientdata(client, data);

	// Jui-Chuan add ++
#if 0
	printk("[Atmel] Check is in bootloader mode\n");
	client->addr = 0x27;
	error = mxt_check_bootloader(client, MXT_WAITING_BOOTLOAD_CMD);
	if(!error){
	    u8 buf[2];
	    buf[0] = 0x00;
		buf[1] = 0x00;
        if (i2c_master_send(client, buf, 2) != 2) {
            	printk("[Atmel] Transfer fail 1\n");
        }else{
			printk("[Atmel] send 0000 to 0x27 success 1\n");
		}
		msleep(200);
		printk("[Atme] check bootloader again\n");
		error = mxt_check_bootloader(client, MXT_WAITING_BOOTLOAD_CMD);
		if(!error){
			printk("[Atmel] Chip is in bootloader mode\n");

	        	//printk("[Atmel] We try FW update\n");
				//struct device *dev = &client->dev;
				//error = mxt_load_fw(dev, MXT_FW_NAME);
		        //if (error) {
        		//        printk("[Atmel] The firmware update failed(%d)\n", error);
                
	        	//} else {
        	    //    	printk("[Atmel] The firmware update succeeded\n");

        		//}
		}
		printk("[Atmel] send 0000 to 0x27 again\n");
		if (i2c_master_send(client, buf, 2) != 2) {
        	printk("[Atmel] Transfer fail 2\n");
        }else{
			printk("[Atmel] Chip is in bootloader mode, check!\n");
		}
		msleep(200);
	}
	client->addr = 0x4b;
	// Hardware reset ++
    error = gpio_direction_output(data->reset_gpio, 0);
    if (error){
        printk("[Atmel] %s:Failed to set reset direction, error=%d\n", __func__, error);
        gpio_free(data->reset_gpio);
    }

    msleep(1);
    gpio_set_value(data->reset_gpio, 1);
    msleep(100);
    // Hardware reset --
	printk("[Atmel] Check is in Application mode\n");	
	u8 buf[2];
    buf[0] = 0x00;
    buf[1] = 0x00;
    if (i2c_master_send(client, buf, 2) != 2) {
       printk("[Atmel] Transfer fail 1\n");
    }else{
       printk("[Atmel] send 0000 to 0x4b success\n");
    }
    msleep(200);
	printk("[Atmel] Check is in Application mode again\n");
    if (i2c_master_send(client, buf, 2) != 2) {
       printk("[Atmel] Transfer fail 1\n");
    }else{
       printk("[Atmel] send 0000 to 0x4b success\n");
    }
    msleep(200);
#endif
	// Jui-Chuan add --



	printk("[Ateml] mxt1664 probe mxt_initialize START\n");
	error = mxt_initialize(data);
	if (error)
		goto err_free_object;



	printk("[Ateml] mxt1664 probe mxt_initialize END\n");
	error = request_threaded_irq(data->irq, NULL, mxt_interrupt,
			IRQF_TRIGGER_FALLING, client->dev.driver->name, data); // Modified
	printk("[Ateml] mxt1664 probe request_threaded_irq\n");
	if (error) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto err_free_object;
	}
	
	error = mxt_make_highchg(data);


	if (error)
		goto err_free_irq;
	printk("[Ateml] mxt1664 probe mxt_make_highchg\n");
	error = input_register_device(input_dev);
	if (error)
		goto err_free_irq;

	error = sysfs_create_group(&client->dev.kobj, &mxt_attr_group);
	if (error)
		goto err_unregister_device;


	// Jui-Chuan add for debug data ++
	data->debug_dir = debugfs_create_dir("maXTouch", NULL);
	if (data->debug_dir == ERR_PTR(-ENODEV)) {
		/* debugfs is not enabled. */
		printk(KERN_WARNING "debugfs not enabled in kernel\n");
	} else if (data->debug_dir == NULL) {
		printk(KERN_WARNING "error creating debugfs dir\n");
	} else {
		//mxt_debug(DEBUG_TRACE, "created \"maXTouch\" debugfs dir\n");

		debugfs_create_file("deltas", S_IRUSR, data->debug_dir, data,
				    &delta_fops);
		debugfs_create_file("refs", S_IRUSR, data->debug_dir, data,
				    &refs_fops);
	}
	// Jui-Chuan add for debug data --
	
	// Jui-Chuan add for early suspend and late resume ++
	#ifdef CONFIG_HAS_EARLYSUSPEND
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 21;
	data->early_suspend.suspend = mxt_early_suspend;
	data->early_suspend.resume = mxt_late_resume;
	register_early_suspend(&data->early_suspend);
	#endif

	data->suspend_state = 0;
	// Jui-Chuan add for early suspend and late resume --

#if 0	// 2013/05/03. Remove by Miracle. 
   for(i=0;i<200;i++)
        printk("[Atmel][8] touch int value = %d\n",gpio_get_value(62));
#endif
	
	return 0;

err_unregister_device:
	input_unregister_device(input_dev);
	input_dev = NULL;
err_free_irq:
	free_irq(client->irq, data);
err_free_object:
	kfree(data->object_table);
err_free_mem:
	input_free_device(input_dev);
	kfree(data);
	return error;
}

static int __devexit mxt_remove(struct i2c_client *client)
{
	struct mxt_data *data = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &mxt_attr_group);
	free_irq(data->irq, data);
	input_unregister_device(data->input_dev);
	kfree(data->object_table);
	kfree(data);

	return 0;
}

#ifdef CONFIG_PM
static int mxt_suspend(struct device *dev, pm_message_t mesg)
{
	printk("[Atmel] Go to mxt_suspend \n");
	struct i2c_client *client = to_i2c_client(dev);
	//struct mxt_data *data = i2c_get_clientdata(client);
	struct mxt_data *data = touch_chip;
	struct input_dev *input_dev = data->input_dev;

	if(data->suspend_state == 0){
		if(!data->tp_firmware_upgrade_proceed){
		
		mutex_lock(&input_dev->mutex);

		if (input_dev->users)
			mxt_stop(data);

		mutex_unlock(&input_dev->mutex);

			data->suspend_state = 1;
		}
	}

	return 0;
}

static int mxt_resume(struct device *dev)
{
	printk("[Atmel] Go to mxt_resume \n");
	struct i2c_client *client = to_i2c_client(dev);
	//struct mxt_data *data = i2c_get_clientdata(client);
	struct mxt_data *data = touch_chip;
	struct input_dev *input_dev = data->input_dev;

	if(data->suspend_state == 1){
		if(!data->tp_firmware_upgrade_proceed){
		/* Soft reset */
		mxt_write_object(data, MXT_GEN_COMMAND_T6,
				MXT_COMMAND_RESET, 1);

		msleep(MXT_RESET_TIME);

		mutex_lock(&input_dev->mutex);

		if (input_dev->users)
			mxt_start(data);

		mutex_unlock(&input_dev->mutex);

			data->suspend_state = 0;
		}
	}
	return 0;
}

static const struct dev_pm_ops mxt_pm_ops = {
	.suspend	= mxt_suspend,
	.resume		= mxt_resume,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mxt_early_suspend(struct early_suspend *es)
{
	
	struct mxt_data *mxt;
	
	//mxt = container_of(es, struct mxt_data, early_suspend);
	mxt = touch_chip;

	printk("Go to mxt_early_suspend\n");
	if(!mxt) printk("mxt is NULL\n");
	if (mxt_suspend(mxt->client, PMSG_SUSPEND) != 0)
		dev_err(&mxt->client->dev, "%s: failed\n", __func__);
	printk(KERN_WARNING "MXT Early Suspended\n");
}

static void mxt_late_resume(struct early_suspend *es)
{
	struct mxt_data *mxt;
	//mxt = container_of(es, struct mxt_data, early_suspend);
	mxt = touch_chip;
	
	printk("Go to mxt_late_resume\n");
	if (mxt_resume(mxt->client) != 0)
		dev_err(&mxt->client->dev, "%s: failed\n", __func__);
	printk(KERN_WARNING "MXT Early Resumed\n");
}

#endif
// Jui-Chuan add for early suspend and late resume --


#else
#define mxt_suspend NULL
#define mxt_resume NULL

#endif //#ifdef CONFIG_PM


static const struct i2c_device_id mxt_id[] = {
	{ "mxt1664s", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mxt_id);

static struct i2c_driver mxt_driver = {
	.driver = {
		.name	= "atmel_mxt_ts",
		.owner	= THIS_MODULE,
// Jui-Chuan mark for early suspend and late resume ++
#ifdef CONFIG_PM
		.pm	= &mxt_pm_ops,
#endif
// Jui-Chuan mark for early suspend and late resume --
	},
	.probe		= mxt_probe,
	.remove		= __devexit_p(mxt_remove),
	.id_table	= mxt_id,
// Jui-Chuan add for early suspend and late resume ++
	#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = mxt_suspend,
	.resume = mxt_resume,
	#endif
// Jui-Chuan add for early suspend and late resume --
};

module_i2c_driver(mxt_driver);

/* Module information */
MODULE_AUTHOR("Joonyoung Shim <jy0922.shim@samsung.com>");
MODULE_DESCRIPTION("Atmel maXTouch Touchscreen driver");
MODULE_LICENSE("GPL");


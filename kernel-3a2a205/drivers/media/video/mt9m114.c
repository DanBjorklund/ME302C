/*
 * Support for mt9m114 Camera Sensor.
 *
 * Copyright (c) 2012 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kmod.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>
//FW_BSP++
#include <linux/proc_fs.h>	//ASUS_BSP+++, add for ISP firmware update
#include "app_i2c_lib_icatch.h"
//FW_BSP--

//#include "i7002a.h"
#include "mt9m114.h"

#define to_mt9m114_sensor(sd) container_of(sd, struct mt9m114_device, sd)

static int first_on = 0;

//Add for ATD read camera status+++
int ATD_mt9m114_status = 0;  //Add for ATD read camera status
static int iso_setting = 0;
int sensorid = 0, retvalue_h = 0, retvalue_l = 0;
//Add build version -> user:3, userdebug:2, eng:1
extern int build_version;



u16 g_is_calibration = 0; //Add for calibration


static ssize_t mt9m114_show_status(struct device *dev,struct device_attribute *attr,char *buf)
{
	printk("%s: get mt9m114 status (%d) !!\n", __func__, ATD_mt9m114_status);
   	//Check sensor connect status, just do it  in begining for ATD camera status 

	return sprintf(buf,"%d\n", ATD_mt9m114_status);
}

static DEVICE_ATTR(mt9m114_status, S_IRUGO,mt9m114_show_status,NULL);

static struct attribute *mt9m114_attributes[] = {
	&dev_attr_mt9m114_status.attr,
	NULL
};
//Add for ATD read camera status---


//FW_BSP++
/****************************************************************************
 *						C O N S T A N T S										*
 ****************************************************************************/
#define FW_HEADER_SIZE 16
#define RES_3ACALI_HEADER_SIZE	8
#define RES_LSC_HEADER_SIZE		24
#define RES_LSCDQ_HEADER_SIZE	16


#define SPI_magic_number 49
#define EEPROM_magic_number 50
#define FW_HEADER_SIZE 16
static char DEFAULT_SPI_FILE_WITH_PATH[] = "/system/etc/camera_spi_init.txt";
static char EXTERNAL_SPI_FILE_WITH_PATH[] = "/data/media/0/camera_spi_init.txt";
static char *SPI_FILE_WITH_PATH = EXTERNAL_SPI_FILE_WITH_PATH;
static char  DEFAULT_FW_BIN_FILE_WITH_PATH[] = "/system/etc/firmware/camera/DEFAULT-ME302C-BOOT.BIN";
static char  NEW_FW_BIN_FILE_WITH_PATH[] = "/system/etc/firmware/camera/ME302C-BOOT.BIN";
static char  EXTERNAL_FW_BIN_FILE_WITH_PATH[] = "/data/media/0/ME302C-BOOT.BIN";
static char *FW_BIN_FILE_WITH_PATH = EXTERNAL_FW_BIN_FILE_WITH_PATH;
static char DEFAULT_CALIBOPT_FILE_WITH_PATH[] = "/system/bin/calibration_option.BIN";
static char FACTORY_CALIBOPT_FILE_WITH_PATH[] = "/factory/calibration_option.BIN";
static char *CALIBOPT_FILE_WITH_PATH = FACTORY_CALIBOPT_FILE_WITH_PATH;
static char IIIACALI_FILE_WITH_PATH[] = "/factory/3ACALI_F.BIN";
static char LSC_FILE_WITH_PATH[] = "/factory/LSC_F.BIN";
static char LSCDQ_FILE_WITH_PATH[] = "/factory/LSC_DQ_F.BIN";
bool FW_BIN_FILE=false, GET_SENSOR_ID=false, FIRST_BOOTING=true;
u8 *pIspFW_g=NULL, *pCalibOpt_g=NULL, *p3acali_g=NULL, *pLsc_g=NULL, *pLscdq_g=NULL;
int SPI_ret=0;
UINT32 bootbin_size_g=0;


static int mt9m114_s_power(struct v4l2_subdev *sd, int power);
extern int spca700xa_SPI_write(UINT8 *, UINT32);
extern int spca700xa_SPI_read(UINT8 *, UINT32);
//FW_BSP--


// iCatch i2c r/w +
int sensor_write_reg(struct i2c_client *client, u16 addr, u16 val)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[4];
	int retry = 0;
	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);
	data[2] = (u8) (val & 0xff);

	//msg.addr = client->addr;
	msg.addr = 0x3C;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;
	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		pr_err("%s : i2c transfer failed, retrying addr = %x val = %x\n", __FUNCTION__, addr, val);
		pr_err("%s : i2c transfer failed, msg.addr %x, err= 0x%x\n", __FUNCTION__, msg.addr, err);
//		msleep(3);
	} while (retry <= SENSOR_MAX_RETRIES);

	if(err == 0) {
		printk("%s(%d): i2c_transfer error, but return 0!?\n", __FUNCTION__, __LINE__);
		err = -EINVAL;
	}

	return err;
}

static int sensor_sequential_write_reg(struct i2c_client *client, unsigned char *data, u16 datasize)
{
	int err;
	struct i2c_msg msg;
	int retry = 0;

              return 0;
	if (datasize==0)
		return 0;
	if (!client->adapter)
		return -ENODEV;

	//msg.addr = client->addr;
	msg.addr = 0x3C;
	msg.flags = 0;
	msg.len = datasize;
	msg.buf = data;
	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		//pr_err("yuv_sensor : i2c transfer failed, retrying %x %x\n",
		      // addr, val);
		pr_err("yuv_sensor : i2c transfer failed, count %x \n",
		       msg.addr);
//		msleep(3);
	} while (retry <= SENSOR_MAX_RETRIES);

	return err;
}

int sensor_read_reg(struct i2c_client *client, u16 addr, u16 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[4];
	if (!client->adapter)
		return -ENODEV;

	//msg[0].addr = client->addr;
	msg[0].addr = 0x3C;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (addr >> 8);;
	data[1] = (u8) (addr & 0xff);

	//msg[1].addr = client->addr;
	msg[1].addr = 0x3C;
	msg[1].flags = I2C_M_RD;

	msg[1].len = 1;
	msg[1].buf = data + 2;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err != 2)
		return -EINVAL;

	memcpy(val, data+2, 1);
	*val=*val&0xff;

	return 0;
}

static int build_sequential_buffer(unsigned char *pBuf, u16 width, u16 value) {
	u32 count = 0;

	switch (width)
	{
	  case 0:
	  // possibly no address, some focusers use this
	  break;

	  // cascading switch
	  case 32:
	    pBuf[count++] = (u8)((value>>24) & 0xFF);
	  case 24:
	    pBuf[count++] = (u8)((value>>16) & 0xFF);
	  case 16:
	    pBuf[count++] = (u8)((value>>8) & 0xFF);
	  case 8:
	    pBuf[count++] = (u8)(value & 0xFF);
	    break;

	  default:
	    printk("Unsupported Bit Width %d\n", width);
	    break;
	}
	return count;

}

int sensor_write_table(struct i2c_client *client,
			      const struct sensor_reg table[])
{
	int err;
	const struct sensor_reg *next;
	u16 val;
	unsigned char data[10];
	u16 datasize = 0;

	//for (next = table; next->addr != SENSOR_TABLE_END; next++) {
	next = table;
	while (next->addr != SENSOR_TABLE_END) {
		if (next->addr == SENSOR_WAIT_MS) {
			msleep(next->val);
			next +=1;
			continue;
		}
		if (next->addr == SEQ_WRITE_START) {
			next += 1;
			while (next->addr !=SEQ_WRITE_END) {
				if (datasize==0) {
					datasize += build_sequential_buffer(&data[datasize], 16, next->addr);
					datasize += build_sequential_buffer(&data[datasize], 8, next->val);
				}
				else
					datasize += build_sequential_buffer(&data[datasize], 8, next->val);
				if (datasize==10) {
					sensor_sequential_write_reg(client, data, datasize);
					datasize = 0;
				}
				next += 1;
			}
			sensor_sequential_write_reg(client, data, datasize); //flush out the remaining buffer.
			datasize = 0;
		}
		else {
			val = next->val;

			err = sensor_write_reg(client, next->addr, val);
			if (err) {
				printk("%s(%d): isensor_write_reg ret= 0x%x\n", __FUNCTION__, __LINE__, err);
				return err;
			}
		}
		next += 1;
	}
	return 0;
}
// iCatch i2c r/w -


//ASUS_BSP+++, add for ISP firmware update, create proc file
#ifdef	CONFIG_PROC_FS
#define	i7002a_PROC_FILE	"driver/i7002a_front"
static struct proc_dir_entry *i7002a_proc_file;

static ssize_t i7002a_proc_read(char *page, char **start, off_t off, int count, 
            	int *eof, void *data)
{
	
}

static ssize_t i7002a_proc_write(struct file *filp, const char __user *buff, 
	            unsigned long len, void *data)
{
	char buffer[256];
	u16 addr, value, read_value;


	if (len > 256)
		len = 256;

        pr_info("i7002a_proc_write %s\n", buff);
	if (copy_from_user(buffer, buff, len)) {
		printk(KERN_INFO "%s: proc write to buffer failed.\n", __func__);
		return -EFAULT;
	}

	if(!strncmp("is_calibration",buffer,14)){ //Add for calibration
		sscanf(&buffer[15],"%d",&value);
		printk(KERN_INFO "%s: Calibration val for front %d.\n", __func__, value);
		g_is_calibration = value;				
	} else {
		pr_info("command not support\n");
	}

	return len;
}

void create_i7002a_proc_file(void)
{
    i7002a_proc_file = create_proc_entry(i7002a_PROC_FILE, 0666, NULL);
    if (i7002a_proc_file) {
		i7002a_proc_file->read_proc = i7002a_proc_read;
		i7002a_proc_file->write_proc = i7002a_proc_write;
    } else{
        pr_err("proc file create failed!\n");
    }
}

#endif
//ASUS_BSP---
//FW_BSP--




int sensor_s_color_effect(struct v4l2_subdev *sd, int effect)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int reg_val;
	int ret = 0;
	printk("%s mode = %d\n", __func__, effect);

	switch (effect) {
	case V4L2_COLORFX_NONE:
		sensor_write_reg(client, 0x7102, 0x00);//auto
		break;
	case V4L2_COLORFX_AQUA:
		sensor_write_reg(client, 0x7102, 0x01);//aqua
		break;
	case V4L2_COLORFX_NEGATIVE:
		sensor_write_reg(client, 0x7102, 0x02);//negative
		break;
	case V4L2_COLORFX_SEPIA:
		sensor_write_reg(client, 0x7102, 0x03);//sepia
		break;
	case V4L2_COLORFX_BW:
		sensor_write_reg(client, 0x7102, 0x04);//grayscale
		break;
	case V4L2_COLORFX_VIVID:
		sensor_write_reg(client, 0x7102, 0x05);//vivid
		break;
	case V4L2_COLORFX_EMBOSS:
		sensor_write_reg(client, 0x7102, 0x07);//vintage
		break;
	case V4L2_COLORFX_SKETCH:
		sensor_write_reg(client, 0x7102, 0x09);//lomo
		break;
	default:
		dev_err(&client->dev, "invalid col eff: %d", effect);
		return -ERANGE;
	}

	return 0;
}

int sensor_s_white_balance(struct v4l2_subdev *sd, int wb_mode)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int reg_val;
	int ret = 0;
	printk("%s mode = %d\n", __func__, wb_mode);

	switch (wb_mode) {
	case V4L2_WHITE_BALANCE_AUTO:
		sensor_write_reg(client, 0x710A, 0x00);//auto
		break;
	case V4L2_WHITE_BALANCE_DAYLIGHT:
		sensor_write_reg(client, 0x710A, 0x01);//daylight
		break;
	case V4L2_WHITE_BALANCE_CLOUDY:
		sensor_write_reg(client, 0x710A, 0x02);//cloudy
		break;
	case V4L2_WHITE_BALANCE_SHADE:
		sensor_write_reg(client, 0x710A, 0x03);//shade
		break;
	case V4L2_WHITE_BALANCE_FLUORESCENT:
		sensor_write_reg(client, 0x710A, 0x04);//fluorescent_L
		break;
	case V4L2_WHITE_BALANCE_FLUORESCENT_H:
		sensor_write_reg(client, 0x710A, 0x05);//fluorescent_H
		break;
	case V4L2_WHITE_BALANCE_INCANDESCENT:
		sensor_write_reg(client, 0x710A, 0x06);//tungsten
		break;
	default:
		dev_err(&client->dev, "invalid white balance mode: %d", wb_mode);
		return -ERANGE;
	}

	return 0;
}

int sensor_s_scene_mode(struct v4l2_subdev *sd, int scene_mode)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int reg_val;
	int ret = 0;
	printk("%s mode = %d\n", __func__, scene_mode);

	switch (scene_mode) {
	case V4L2_SCENE_MODE_NONE:
		sensor_write_reg(client, 0x7109, 0x00);//auto
		break;
	case V4L2_SCENE_MODE_BACKLIGHT:
		sensor_write_reg(client, 0x7109, 0x16);//backlight
		break;
	case V4L2_SCENE_MODE_BEACH_SNOW:
		sensor_write_reg(client, 0x7109, 0x0B);//snow
		break;
	case V4L2_SCENE_MODE_CANDLE_LIGHT:
		sensor_write_reg(client, 0x7109, 0x04);//candle light
		break;
	case V4L2_SCENE_MODE_FIREWORKS:
		sensor_write_reg(client, 0x7109, 0x05);//firework
		break;
	case V4L2_SCENE_MODE_LANDSCAPE:
		sensor_write_reg(client, 0x7109, 0x06);//landscape
		break;
	case V4L2_SCENE_MODE_NIGHT:
		sensor_write_reg(client, 0x7109, 0x07);//night
		break;
	case V4L2_SCENE_MODE_PARTY_INDOOR:
		sensor_write_reg(client, 0x7109, 0x09);//party
		break;
	case V4L2_SCENE_MODE_PORTRAIT:
		sensor_write_reg(client, 0x7109, 0x0A);//portrait
		break;
	case V4L2_SCENE_MODE_SPORTS:
		sensor_write_reg(client, 0x7109, 0x0C);//sport
		break;
	case V4L2_SCENE_MODE_SUNSET:
		sensor_write_reg(client, 0x7109, 0x0E);//sunset
		break;
	case V4L2_SCENE_MODE_TEXT:
		sensor_write_reg(client, 0x7109, 0x02);//text
		break;
	default:
		dev_err(&client->dev, "invalid snene mode: %d", scene_mode);
		return -ERANGE;
	}

	return 0;
}

int sensor_s_ev(struct v4l2_subdev *sd, int ev)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int reg_val;
	int ret = 0;
	printk("%s mode = %d\n", __func__, ev);

	switch (ev) {
	case 6:
		sensor_write_reg(client, 0x7103, 0x00);// +2.0
		break;
	case 5:
		sensor_write_reg(client, 0x7103, 0x01);// +1.7
		break;
	case 4:
		sensor_write_reg(client, 0x7103, 0x02);// +1.3
		break;
	case 3:
		sensor_write_reg(client, 0x7103, 0x03);// +1.0
		break;
	case 2:
		sensor_write_reg(client, 0x7103, 0x04);// +0.7
		break;
	case 1:
		sensor_write_reg(client, 0x7103, 0x05);// +0.3
		break;
	case 0:
		sensor_write_reg(client, 0x7103, 0x06);// +0
		break;
	case -1:
		sensor_write_reg(client, 0x7103, 0x07);// -0.3
		break;
	case -2:
		sensor_write_reg(client, 0x7103, 0x08);// -0.7
		break;
	case -3:
		sensor_write_reg(client, 0x7103, 0x09);// -1.0
		break;
	case -4:
		sensor_write_reg(client, 0x7103, 0x0A);// -1.3
		break;
	case -5:
		sensor_write_reg(client, 0x7103, 0x0B);// -1.7
		break;
	case -6:
		sensor_write_reg(client, 0x7103, 0x0C);// -2.0
		break;
	default:
		dev_err(&client->dev, "invalid ev: %d", ev);
		return -ERANGE;
	}

	return 0;
}

int sensor_s_flicker(struct v4l2_subdev *sd, int flicker)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int reg_val;
	int ret = 0;
	printk("%s mode = %d\n", __func__, flicker);

	switch (flicker) {
	case V4L2_CID_POWER_LINE_FREQUENCY_AUTO:
		sensor_write_reg(client, 0x7101, 0x00);//auto
		break;
	case V4L2_CID_POWER_LINE_FREQUENCY_50HZ:
		sensor_write_reg(client, 0x7101, 0x01);//50hz
		break;
	case V4L2_CID_POWER_LINE_FREQUENCY_60HZ:
		sensor_write_reg(client, 0x7101, 0x02);//60hz
		break;
	default:
		dev_err(&client->dev, "invalid flicker: %d", flicker);
		return -ERANGE;
	}

	return 0;
}

int sensor_s_iso(struct v4l2_subdev *sd, int iso)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int reg_val;
	int ret = 0;
	printk("%s mode = %d\n", __func__, iso);
	iso_setting = iso;

	switch (iso) {
	case SENSOR_ISO_AUTO:
		sensor_write_reg(client, 0x7110, 0x00);
		break;
	case SENSOR_ISO_50:
		sensor_write_reg(client, 0x7110, 0x01);
		break;
	case SENSOR_ISO_100:
		sensor_write_reg(client, 0x7110, 0x02);
		break;
	case SENSOR_ISO_200:
		sensor_write_reg(client, 0x7110, 0x03);
		break;
	case SENSOR_ISO_400:
		sensor_write_reg(client, 0x7110, 0x04);
		break;
	case SENSOR_ISO_800:
		sensor_write_reg(client, 0x7110, 0x05);
		break;
	default:
		dev_err(&client->dev, "invalid iso: %d", iso);
		return -ERANGE;
	}

	return 0;
}

int sensor_s_ae_metering_mode(struct v4l2_subdev *sd, int ae_mode)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int reg_val;
	int ret = 0;
	printk("%s mode = %d\n", __func__, ae_mode);

	switch (ae_mode) {
	case V4L2_EXPOSURE_METERING_AVERAGE:
		sensor_write_reg(client, 0x710E, 0x00);//multi
		break;
	case V4L2_EXPOSURE_METERING_SPOT:
		sensor_write_reg(client, 0x710E, 0x01);//spot
		break;
	case V4L2_EXPOSURE_METERING_CENTER_WEIGHTED:
		sensor_write_reg(client, 0x710E, 0x02);//center
		break;
	default:
		dev_err(&client->dev, "invalid ae_mode: %d", ae_mode);
		return -ERANGE;
	}

	return 0;
}
//ASUS_BSP++, add for calibration
int sensor_s_write_reg(struct v4l2_subdev *sd, int val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int reg_addr = 0x0;
	int reg_val = 0x0;
	int ret = 0;
//	printk("%s val:0x%x\n", __func__, val);

	reg_addr = val >>16;
	reg_val = val & 0xFFFF;
//	printk("reg_addr:0x%x, reg_val:0x%x\n", reg_addr, reg_val);

	ret = sensor_write_reg(client, reg_addr, reg_val);
	return 0;
}

int sensor_s_read_reg(struct v4l2_subdev *sd, int *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int reg_addr = 0x0;
	int reg_val = 0x0;
	int ret = 0;
//	printk("%s val:0x%x\n", __func__, *val);

	reg_addr = *val;	
	ret = sensor_read_reg(client, reg_addr, &reg_val);
	//reg_val = 128;
	*val = reg_val;
//	printk("val:0x%x, reg_val:0x%x\n", *val, reg_val);
	return 0;
}

int sensor_s_read_spi(struct v4l2_subdev *sd, int *val)
{
	UINT8 *ucStartAddr;
	UINT32 ulTransByteCnt = 0;
	UINT8 type = 0;
	struct file *fp = NULL;
	static char *CALIBRATION_FILE_WITH_PATH;
	struct inode *inode;
	mm_segment_t old_fs;
	loff_t offset = 0;
	int i,j;
	

//	printk("%s, val = %x\n", __func__, *val);
	type = (*val)>>28;
	ulTransByteCnt = (*val)&0x0FFFFFFF;
//	printk("%s, type = %x, val = %x, ulTransByteCnt = %x\n", __func__, type, *val, ulTransByteCnt);


	ucStartAddr = kmalloc(ulTransByteCnt, GFP_KERNEL);

	spca700xa_SPI_read(ucStartAddr, ulTransByteCnt);


	switch(type) {
		case 0:
			fp = filp_open(IIIACALI_FILE_WITH_PATH, O_RDWR | O_CREAT, S_IRUGO | S_IWUSR);
			CALIBRATION_FILE_WITH_PATH = IIIACALI_FILE_WITH_PATH;
			break;
		case 1:
			fp = filp_open(LSC_FILE_WITH_PATH, O_RDWR | O_CREAT, S_IRUGO | S_IWUSR);
			CALIBRATION_FILE_WITH_PATH = LSC_FILE_WITH_PATH;
			break;
		case 2:
			fp = filp_open(LSCDQ_FILE_WITH_PATH, O_RDWR | O_CREAT, S_IRUGO | S_IWUSR);
			CALIBRATION_FILE_WITH_PATH = LSCDQ_FILE_WITH_PATH;
			break;
	}


	if ( IS_ERR_OR_NULL(fp) ){
		printk("%s: open %s fail\n", __FUNCTION__, CALIBRATION_FILE_WITH_PATH);
	} else {
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		offset = 0;
		if (fp->f_op != NULL && fp->f_op->write != NULL){
			fp->f_op->write(fp,
				ucStartAddr,
				ulTransByteCnt,
				&offset);
		}else
			pr_err("%s: f_op might be null\n", __FUNCTION__);
		set_fs(old_fs);
		filp_close(fp, NULL);
	}	

	*val = 0;

	kfree(ucStartAddr);

	return 0;
}

//ASUS_BSP--, add for calibration

int sensor_g_exposure(struct v4l2_subdev *sd, int *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int exposure_num;
	u32 exposure_denum;
	u16 retvalue_hh, retvalue_h, retvalue_l;
	int ret = 0;

	ret = sensor_read_reg(client, 0x72B0, &retvalue_l); //num[7:0]
	exposure_num = retvalue_l;

	ret = sensor_read_reg(client, 0x72B1, &retvalue_l); //denum[7:0]
	ret = sensor_read_reg(client, 0x72B2, &retvalue_h); //denum[15:8]
	ret = sensor_read_reg(client, 0x72B3, &retvalue_hh); //denum[23:16]
	exposure_denum = (retvalue_hh<<16)|(retvalue_h<<8)|(retvalue_l);

	printk("%s, exposure time num %d denum %d\n", __func__, exposure_num, exposure_denum);

	*val = ((exposure_num << 24) | exposure_denum);

	return 0;
}

//For iCatch 3A information+++
int sensor_g_3a_info_ae1(struct v4l2_subdev *sd, int *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 retvalue_0, retvalue_1, retvalue_2, retvalue_3;
	int ret = 0;

	ret = sensor_read_reg(client, 0x72D8, &retvalue_0);
	ret = sensor_read_reg(client, 0x72D9, &retvalue_1);
	ret = sensor_read_reg(client, 0x72DA, &retvalue_2);
	ret = sensor_read_reg(client, 0x72DB, &retvalue_3);

	*val = ((retvalue_3 << 24) | (retvalue_2 << 16) | (retvalue_1 << 8) | retvalue_0);
	printk("%s, val 0x%x\n", __func__, *val);

	return 0;
}

int sensor_g_3a_info_ae2(struct v4l2_subdev *sd, int *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 retvalue_0, retvalue_1, retvalue_2, retvalue_3;
	int ret = 0;

	ret = sensor_read_reg(client, 0x72DC, &retvalue_0);
	ret = sensor_read_reg(client, 0x72DD, &retvalue_1);
	ret = sensor_read_reg(client, 0x72DE, &retvalue_2);
	ret = sensor_read_reg(client, 0x72DF, &retvalue_3);

	*val = ((retvalue_3 << 24) | (retvalue_2 << 16) | (retvalue_1 << 8) | retvalue_0);
	printk("%s, val 0x%x\n", __func__, *val);

	return 0;
}

int sensor_g_3a_info_awb1(struct v4l2_subdev *sd, int *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 retvalue_0, retvalue_1, retvalue_2, retvalue_3;
	int ret = 0;

	ret = sensor_read_reg(client, 0x72E0, &retvalue_0);
	ret = sensor_read_reg(client, 0x72E1, &retvalue_1);
	ret = sensor_read_reg(client, 0x72E2, &retvalue_2);
	ret = sensor_read_reg(client, 0x72E3, &retvalue_3);

	*val = ((retvalue_3 << 24) | (retvalue_2 << 16) | (retvalue_1 << 8) | retvalue_0);
	printk("%s, val 0x%x\n", __func__, *val);

	return 0;
}

int sensor_g_3a_info_awb2(struct v4l2_subdev *sd, int *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 retvalue_0, retvalue_1, retvalue_2, retvalue_3;
	int ret = 0;

	ret = sensor_read_reg(client, 0x72E4, &retvalue_0);
	ret = sensor_read_reg(client, 0x72E5, &retvalue_1);
	ret = sensor_read_reg(client, 0x72EE, &retvalue_2);
	ret = sensor_read_reg(client, 0x72EF, &retvalue_3);

	*val = ((retvalue_3 << 24) | (retvalue_2 << 16) | (retvalue_1 << 8) | retvalue_0);
	printk("%s, val 0x%x\n", __func__, *val);

	return 0;
}

int sensor_g_3a_info_af1(struct v4l2_subdev *sd, int *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 retvalue_0, retvalue_1, retvalue_2, retvalue_3;
	int ret = 0;

	ret = sensor_read_reg(client, 0x72E6, &retvalue_0);
	ret = sensor_read_reg(client, 0x72E7, &retvalue_1);
	ret = sensor_read_reg(client, 0x72E8, &retvalue_2);
	ret = sensor_read_reg(client, 0x72E9, &retvalue_3);

	*val = ((retvalue_3 << 24) | (retvalue_2 << 16) | (retvalue_1 << 8) | retvalue_0);
	printk("%s, val 0x%x\n", __func__, *val);

	return 0;
}

int sensor_g_3a_info_af2(struct v4l2_subdev *sd, int *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 retvalue_0, retvalue_1, retvalue_2, retvalue_3;
	int ret = 0;

	ret = sensor_read_reg(client, 0x72EA, &retvalue_0);
	ret = sensor_read_reg(client, 0x72EB, &retvalue_1);
	ret = sensor_read_reg(client, 0x72EC, &retvalue_2);
	ret = sensor_read_reg(client, 0x72ED, &retvalue_3);

	*val = ((retvalue_3 << 24) | (retvalue_2 << 16) | (retvalue_1 << 8) | retvalue_0);
	printk("%s, val 0x%x\n", __func__, *val);

	return 0;
}
//For iCatch 3A information---

//FW_BSP++
/*-------------------------------------------------------------------------
 *  Function Name : EXISP_7002SPICfg
 *  Description : external SPI configuration
 *  ucSPIMode: SPI mode
     ucSPIFreq: SPI frequency
     ulDmaAddr: SPCA7002 DMA start address
     ulTransByteCnt: buffer size to be wrote
 *  Return : status
 *------------------------------------------------------------------------*/
static UINT8 EXISP_7002SPICfg(UINT8 ucSPIMode, UINT8 ucSPIFreq, UINT32 ulDmaAddr, UINT32 ulTransByteCnt, struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	UINT8 status = SUCCESS;
	UINT8 ucStartBank, ucEndBank, ucRealEndBank, i;
	UINT32 udwBankEnValue=0;
	const UINT16 uwBanksize = 8192;
	const UINT32 udwMaxTransByteCnt = 0x50000;
	u16 ucReadData;

	static UINT8 regdata1[][3] = {
		{0x40, 0x10, 0x10}, /*SPI reset*/
		{0x40, 0xe0, 0x00}, /*SPI freq*/
		{0x40, 0xe1, 0x00}, /*SPI mode*/
		{0x40, 0x51, 0x01}, /*SPI enable*/
		{0x40, 0x11, 0x10}, /*DMA0 reset*/
		{0x41, 0x64, 0x01}, /*Read data from host*/
		{0x41, 0x70, 0x0f}, /*Byte Cnt Low*/
		{0x41, 0x71, 0x0f}, /*Byte Cnt Mid*/
		{0x41, 0x72, 0x0f}, /*Byte Cnt High*/
		{0x10, 0x8c, 0x00}, /*DMA master select FM*/
		{0x10, 0x80, 0x00}, /*DMA start addr Low*/
		{0x10, 0x81, 0x00}, /*DMA start addr Mid*/
		{0x10, 0x82, 0x00}, /*DMA start addr High*/
		{0x10, 0x84, 0x00}, /*DMA bank enable*/
		{0x10, 0x85, 0x00},
		{0x10, 0x86, 0x00},
		{0x10, 0x87, 0x00},
		{0x10, 0x88, 0x00},
		{0x00, 0x26, 0x00},
		{0x40, 0x03, 0x02}, /*Clear DMA0 INT status*/
	};

	regdata1[1][2] = (((UINT8)ucSPIFreq) & 0x7);
	regdata1[2][2] = (((UINT8)ucSPIMode) & 0xf);
	regdata1[5][2] = (1&0x1);

	if (udwMaxTransByteCnt < ulTransByteCnt)
		ulTransByteCnt = (udwMaxTransByteCnt-1);
	else
		ulTransByteCnt--;

	regdata1[6][2] = (ulTransByteCnt & 0xff);
	regdata1[7][2] = ((ulTransByteCnt >> 8) & 0xff);
	regdata1[8][2] = ((ulTransByteCnt >> 16) & 0xff);
	regdata1[9][2] = 0<<4;
	regdata1[10][2] = (ulDmaAddr & 0xff);
	regdata1[11][2] = ((ulDmaAddr >> 8) & 0xff);
	regdata1[12][2] = ((ulDmaAddr >> 16) & 0xff);

	ucStartBank = (ulDmaAddr&0xffffff)/uwBanksize;
	ucEndBank = ((ulDmaAddr&0xffffff)+ulTransByteCnt)/uwBanksize;
	ucRealEndBank = ucEndBank;

	if (ucEndBank > 31) {

		for (i = 32; i <= ucEndBank; i++)
			udwBankEnValue |= (1 << (i-32));

		regdata1[17][2] = (udwBankEnValue & 0xff);
		ucRealEndBank = 32;
		udwBankEnValue = 0;
	}

	for (i = ucStartBank; i <= ucRealEndBank; i++)
		udwBankEnValue |= (1 << i);

	regdata1[13][2] = (udwBankEnValue & 0xff);
	regdata1[14][2] = ((udwBankEnValue >> 8) & 0xff);
	regdata1[15][2] = ((udwBankEnValue >>16) & 0xff);
	regdata1[16][2] = ((udwBankEnValue >>24) & 0xff);

	sensor_read_reg(client, 0x0026, &ucReadData); /*Config the SPI pin GPIO/Function mode.*/
	ucReadData &= (~0xf);
	regdata1[18][2] = ucReadData;

	for (i = 0; i < sizeof(regdata1)/sizeof(regdata1[0]); i++)
		sensor_write_reg(client, ((regdata1[i][0]<<8)&0xFF00)+(regdata1[i][1]&0x00FF), regdata1[i][2]);

	return status;
}



/*-------------------------------------------------------------------------
 *  Function Name : EXISP_7002SPICfg_slave
 *  Description : external SPI configuration
 *  ucSPIMode: SPI mode
     ulDmaAddr: SPCA7002 DMA start address
     ulTransByteCnt: buffer size to be wrote
 *  Return : status
 *------------------------------------------------------------------------*/
static UINT8 EXISP_7002SPICfg_slave(UINT8 ucSPIMode, UINT32 ulDmaAddr, UINT32 ulTransByteCnt, struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	UINT8 status = SUCCESS;
	UINT8 ucStartBank, ucEndBank, ucRealEndBank, i;
	UINT32 udwBankEnValue=0;
	const UINT16 uwBanksize = 8192;
	const UINT32 udwMaxTransByteCnt = 0x50000;
	u16 ucReadData;

	static UINT8 regdata1[][3] = {
		{0x40, 0x10, 0x10}, /*SPI reset*/
		{0x40, 0xe1, 0x00}, /*SPI mode*/
		{0x40, 0x51, 0x01}, /*SPI enable*/
		{0x40, 0x11, 0x10}, /*DMA0 reset*/
		{0x41, 0x64, 0x01}, /*Read data from host*/
		{0x41, 0x70, 0x0f}, /*Byte Cnt Low*/
		{0x41, 0x71, 0x0f}, /*Byte Cnt Mid*/
		{0x41, 0x72, 0x0f}, /*Byte Cnt High*/
		{0x10, 0x8c, 0x00}, /*DMA master select FM*/
		{0x10, 0x80, 0x00}, /*DMA start addr Low*/
		{0x10, 0x81, 0x00}, /*DMA start addr Mid*/
		{0x10, 0x82, 0x00}, /*DMA start addr High*/
		{0x10, 0x84, 0x00}, /*DMA bank enable*/
		{0x10, 0x85, 0x00},
		{0x10, 0x86, 0x00},
		{0x10, 0x87, 0x00},
		{0x10, 0x88, 0x00},
		{0x00, 0x26, 0x00},
		{0x40, 0x03, 0x02}, /*Clear DMA0 INT status*/
	};

	regdata1[1][2] = (((UINT8)ucSPIMode) & 0xf);
	regdata1[4][2] = (1&0x1);

	if (udwMaxTransByteCnt < ulTransByteCnt)
		ulTransByteCnt = (udwMaxTransByteCnt-1);
	else
		ulTransByteCnt--;

	regdata1[5][2] = (ulTransByteCnt & 0xff);
	regdata1[6][2] = ((ulTransByteCnt >> 8) & 0xff);
	regdata1[7][2] = ((ulTransByteCnt >> 16) & 0xff);
	regdata1[8][2] = 0<<4;
	regdata1[9][2] = (ulDmaAddr & 0xff);
	regdata1[10][2] = ((ulDmaAddr >> 8) & 0xff);
	regdata1[11][2] = ((ulDmaAddr >> 16) & 0xff);

	ucStartBank = (ulDmaAddr&0xffffff)/uwBanksize;
	ucEndBank = ((ulDmaAddr&0xffffff)+ulTransByteCnt)/uwBanksize;
	ucRealEndBank = ucEndBank;

	if (ucEndBank > 31) {

		for (i = 32; i <= ucEndBank; i++)
			udwBankEnValue |= (1 << (i-32));

		regdata1[16][2] = (udwBankEnValue & 0xff);
		ucRealEndBank = 32;
		udwBankEnValue = 0;
	}

	for (i = ucStartBank; i <= ucRealEndBank; i++)
		udwBankEnValue |= (1 << i);

	regdata1[12][2] = (udwBankEnValue & 0xff);
	regdata1[13][2] = ((udwBankEnValue >> 8) & 0xff);
	regdata1[14][2] = ((udwBankEnValue >>16) & 0xff);
	regdata1[15][2] = ((udwBankEnValue >>24) & 0xff);

	sensor_read_reg(client, 0x0026, &ucReadData); /*Config the SPI pin GPIO/Function mode.*/
	ucReadData &= (~0xf);
	regdata1[17][2] = ucReadData;

	for (i = 0; i < sizeof(regdata1)/sizeof(regdata1[0]); i++)
		sensor_write_reg(client, ((regdata1[i][0]<<8)&0xFF00)+(regdata1[i][1]&0x00FF), regdata1[i][2]);

	return status;
}





void EXISP_SPIDataRead(UINT8 ucSPIMode, UINT8 ucSPIFreq, UINT32 ulTransByteCnt, struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	UINT16 uwTimeCnt = 0;
	UINT8 *ucStartAddr;
	u16 retValue;

	ucStartAddr = kmalloc(ulTransByteCnt, GFP_KERNEL);

	EXISP_7002SPICfg(ucSPIMode, ucSPIFreq, 0, ulTransByteCnt, sd);

	/*Trigger the 7002 SPI DMA0*/
	sensor_write_reg(client, 0x4160, 0x01);

	/*Polling the register 0x4000 firstly to check the empty state, 1: empty, 0: not empty*/
	sensor_read_reg(client, 0x40E6, &retValue);
	while ((retValue&0x02) != 0) {
		udelay(5000);
		uwTimeCnt++;
		if (1000 < uwTimeCnt) {
			printk("Check 7002 register 0x4000 Timeout.\n");
			return;
		}
		sensor_read_reg(client, 0x40E6, &retValue);
	}

	spca700xa_SPI_read(ucStartAddr, ulTransByteCnt);

	/* Restore SPCA7002 DMA setting */
	sensor_write_reg(client, 0x1084, 0);
	sensor_write_reg(client, 0x1085, 0);
	sensor_write_reg(client, 0x1086, 0);
	sensor_write_reg(client, 0x1087, 0);
	sensor_write_reg(client, 0x1088, 0);
	sensor_write_reg(client, 0x108c, 0);
	kfree(ucStartAddr);
}



/*-------------------------------------------------------------------------
 *  Function Name : EXISP_SPIDataWrite
 *  Description : write data to external ISP through SPI interface
 *  ucSPIMode: SPI mode
     ucStartAddr: buffer to be wrote
     ulTransByteCnt: buffer size to be wrote
     ulDmaAddr: SPCA7002 DMA start address
 *  Return : status
 *------------------------------------------------------------------------*/
static UINT8 EXISP_SPIDataWrite(UINT8 ucSPIMode, UINT8 *ucStartAddr, UINT32 ulTransByteCnt, UINT32 ulDmaAddr, struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	UINT8 status = SUCCESS;
	UINT16 uwTimeCnt = 0;
	u16 retValue;


	EXISP_7002SPICfg_slave(ucSPIMode, ulDmaAddr, ulTransByteCnt, sd);

	/*Trigger the 7002 SPI DMA0*/
	sensor_write_reg(client, 0x4160, 0x01);

	
	/* Enable data transaction */
	spca700xa_SPI_write(ucStartAddr, ulTransByteCnt);

	/* Read data from DMA */
//	EXISP_SPIDataRead(ucSPIMode, ucSPIFreq, ulTransByteCnt, sd);	

	/* Wait SPCA7002 DMA done */
	sensor_read_reg(client, 0x4003, &retValue);
	while ((retValue&0x02) != 0x02) {
		udelay(5000);
		uwTimeCnt++;
		if (1000 < uwTimeCnt) {
			printk("Wait 7002 DMA0 INT Timeout.\n");
			return;
		}
		sensor_read_reg(client, 0x4003, &retValue);
	}

	/* Restore SPCA7002 DMA setting */
	sensor_write_reg(client, 0x1084, 0);
	sensor_write_reg(client, 0x1085, 0);
	sensor_write_reg(client, 0x1086, 0);
	sensor_write_reg(client, 0x1087, 0);
	sensor_write_reg(client, 0x1088, 0);
	sensor_write_reg(client, 0x108c, 0);

	return status;
}



/*-------------------------------------------------------------------------
 *  Function Name : EXISP_LoadCode
 *  Description : Load code from host, support boot from host only
 *  ucFwIdx: Set which FW will be loaded
     is_calibration: calibration flag, calibration:1 normal:0
     pIspFw: external ISP FW pointer
     pCaliOpt: read from calibration_option.BIN
     p3acali: read from 3ACALI.BIN or 3ACALI_F.BIN
     pLsc: read from LSC.BIN or LSC_F.BIN
     pLscdq: read from LSC_DQ.BIN or LSC_DQ_F.BIN
 *  Return : status result
 *------------------------------------------------------------------------*/
UINT8 EXISP_LoadCode(
	UINT8 ucFwIdx,
	UINT8 is_calibration,
	UINT8 *pIspFw,
	UINT8 *pCalibOpt,
	UINT8 *p3acali,
	UINT8 *pLsc,
	UINT8 *pLscdq,
	struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	UINT8 ucRet = SUCCESS;
	UINT32 t1=0, t2=0, t3=0, tmrCnt=0, i=0;
	UINT32 checksumWrite=0, checksumRead=0;
	ispFwHeaderInfo_t *pFwInfo;
	ispLoadCodeRet_t loadCodeRet;
	u16 retvalue=0;

	if (pIspFw == NULL) {
		return LOADCODE_BOOT_FILE_ERR;
	}

	pFwInfo = (ispFwHeaderInfo_t *)pIspFw;
	for (i = 0; i < ucFwIdx; i++) {
		pIspFw += FW_HEADER_SIZE+pFwInfo->DmemFicdmemSize+pFwInfo->ImemSize;
		pFwInfo = (ispFwHeaderInfo_t *)pIspFw;
	}

	/* Modify length to 16-alignment */
	pFwInfo->DmemFicdmemSize = (pFwInfo->DmemFicdmemSize+15)&0xFFFFFFF0;
	pFwInfo->ImemSize = (pFwInfo->ImemSize+15)&0xFFFFFFF0;
	//printk(" %s, %d, DmemFicdmemSize=%d\n", __FUNCTION__, __LINE__, pFwInfo->DmemFicdmemSize);
	//printk(" %s, %d, ImemSize=%d\n", __FUNCTION__, __LINE__, pFwInfo->ImemSize);
	//printk("0.pIspFw =%x\n", pIspFw);
	printk("@@@ISP FW: Get BOOT.BIN Bin File End\n");

#if 1 //update golden or calibration resource
	printk("@@@ISP FW: Update Resource Start\n");
	if (is_calibration == 1) { //calibration
		//Nothing to do
	}
	else { //normal
		memset(&loadCodeRet, 0, sizeof(ispLoadCodeRet_t));

		/* pCalibOpt check */
		if (pCalibOpt == NULL) {
			loadCodeRet.retCalibOpt= LOADCODE_CALIB_OPT_FILE_ERR;
			goto _EXIT_;
		}
		/* p3acali check */
		if (p3acali == NULL) {
			loadCodeRet.ret3acli= LOADCODE_3ACALI_FILE_ERR;
		}
		/* pLsc check */
		if (pLsc == NULL) {
			loadCodeRet.retLsc= LOADCODE_LSC_FILE_ERR;
		}
		/* pLscdq check */
		if (pLscdq == NULL) {
			loadCodeRet.retLscdq = LOADCODE_LSC_DQ_FILE_ERR;
		}

		EXISP_UpdateCalibResStart(ucFwIdx, pIspFw, pFwInfo, &loadCodeRet, *pCalibOpt, p3acali, pLsc, pLscdq);
	}
	printk("@@@ISP FW: Update Resource End\n");
#endif

_EXIT_:
	if (is_calibration == 1) { //calibration
		printk("********** Load Golden Res **********\n");
	}
	else { //normal
		if (loadCodeRet.retCalibOpt == SUCCESS &&
			loadCodeRet.ret3acli == SUCCESS &&
			loadCodeRet.retLsc == SUCCESS &&
			loadCodeRet.retLscdq == SUCCESS) {
			printk("********** Load Calibration Res **********\n");
		}
		else {
			if (loadCodeRet.retCalibOpt != SUCCESS) {
				printk("********** Load Golden Res, retCalibOpt=%d **********\n", loadCodeRet.retCalibOpt);
			}
			else if (loadCodeRet.ret3acli != SUCCESS) {
				printk("********** Load Golden 3ACALI Res, ret3acli=%d **********\n", loadCodeRet.ret3acli);
			}
			else if (loadCodeRet.retLsc != SUCCESS || loadCodeRet.retLscdq == SUCCESS) {
				printk("********** Load Golden LSC Res, retLsc=%d, retLscdq=%d **********\n", loadCodeRet.retLsc, loadCodeRet.retLscdq);
			}
		}
	}

	/* CPU reset */
	//I2CDataWrite(0x1011, 1);
	sensor_write_reg(client, 0x1011, 1);

	/* Set imemmode*/
	t1 = pFwInfo->ImemSize/8192;
	t2 = t1-32;
	t3 = 1;
	if ((int)t2 < 0) {
		t1 = t3 << t1;
		--t1;
	}
	else {
		t3 <<= t2;
		t1 = -1U;
	}
	--t3;
	//I2CDataWrite(0x1070, t1);
	//I2CDataWrite(0x1071, t1>>8);
	//I2CDataWrite(0x1072, t1>>16);
	//I2CDataWrite(0x1073, t1>>24);
	//I2CDataWrite(0x1074, t3);
	sensor_write_reg(client, 0x1070, t1);
	sensor_write_reg(client, 0x1071, t1>>8);
	sensor_write_reg(client, 0x1072, t1>>16);
	sensor_write_reg(client, 0x1073, t1>>24);
	sensor_write_reg(client, 0x1074, t3);
	

	/* @@@ Start load code to SPCA7002 */

	/* Enable checksum mechanism */
	//I2CDataWrite(0x4280, 1);
	sensor_write_reg(client, 0x4280, 1);

	/* Wait Ready For Load Code interrupt */
	sensor_read_reg(client, SP7K_RDREG_INT_STS_REG_0, &retvalue);
//	while (!(I2CDataRead(SP7K_RDREG_INT_STS_REG_0)&0x02)) {
	while (!(retvalue&0x02)) {

		tmrCnt++;
		if (tmrCnt >= 10) {
			printk("@@@ISP FW: polling RFLC bit timeout\n");
			ucRet = FAIL;
			return ucRet;
		}
		mdelay(10);
	}
	//I2CDataWrite(SP7K_RDREG_INT_STS_REG_0, 0x02);
	sensor_write_reg(client, SP7K_RDREG_INT_STS_REG_0, 0x02);

	/* Load DMEM/FICDMEM bin file Start */
	printk("@@@ISP FW: Load DMEM/FICDMEM Bin File Start\n");
	pIspFw += FW_HEADER_SIZE;
	/* Reset checksum value */
	//I2CDataWrite(0x4284, 0x00);
	sensor_write_reg(client, 0x4284, 0x00);
	checksumWrite = 0;
	for (i = 0; i < pFwInfo->DmemFicdmemSize; i++) {
		checksumWrite += pIspFw[i];
	}
	checksumWrite &= 0xFF;

	/* Transmit DMEM/FICDMEM data */
	if(pFwInfo->DmemFicdmemSize <= 6*1024) { /* Data size <6K, load all bin file */
		ucRet = EXISP_SPIDataWrite(0/*SPI mode0*/, pIspFw, pFwInfo->DmemFicdmemSize, 0x0800, sd);
		//I2CDataWrite(0x1011, 0); /* Set CPU to normal operation */
		sensor_write_reg(client, 0x1011, 0);
	}
	else {
		ucRet = EXISP_SPIDataWrite(0/*SPI mode0*/, pIspFw, 6*1024, 0x0800, sd);
		//I2CDataWrite(0x1011, 0); /* Set CPU to normal operation */
		sensor_write_reg(client, 0x1011, 0);
		ucRet = EXISP_SPIDataWrite(0/*SPI mode0*/, pIspFw+(6*1024), pFwInfo->DmemFicdmemSize-(6*1024), 0x0800+(6*1024), sd);
	}

	/* Checksum value check */
	//checksumRead = I2CDataRead(0x4284);
	sensor_read_reg(client, 0x4284, &checksumRead);
	if (checksumWrite == checksumRead) {
		printk("@@@ISP FW: Checksum DMEM/FICDMEM test: OK, %x, %x\n", checksumRead, checksumWrite);
	}
	else {
		printk("@@@ISP FW: Checksum DMEM/FICDMEM test: FAIL, %x, %x\n", checksumRead, checksumWrite);
		ucRet = FAIL;
		return ucRet;
	}
	printk("@@@ISP FW: Load DMEM/FICDMEM Bin File End\n");
	/* Load DMEM/FICDMEM bin file End */

	/* Load IMEM bin file Start */
	printk("@@@ISP FW: Load IMEM Bin File Start\n");
	pIspFw += pFwInfo->DmemFicdmemSize;
	/* Reset checksum value */
	//I2CDataWrite(0x4284, 0x00);
	sensor_write_reg(client, 0x4284, 0x00);
	checksumWrite = 0;
	for (i = 0; i < pFwInfo->ImemSize; i++) {
		checksumWrite += pIspFw[i];
	}
	checksumWrite &= 0xFF;

	/* Transmit IMEM data */
	ucRet = EXISP_SPIDataWrite(0/*SPI mode0*/, pIspFw, pFwInfo->ImemSize, (320*1024)-pFwInfo->ImemSize, sd);

	/* Checksum value check */
	//checksumRead = I2CDataRead(0x4284);
	sensor_read_reg(client, 0x4284, &checksumRead);
	if (checksumWrite == checksumRead) {
		printk("@@@ISP FW: Checksum IMEM test: OK, %x, %x\n", checksumRead, checksumWrite);
	}
	else {
		printk("@@@ISP FW: Checksum IMEM test: FAIL, %x, %x\n", checksumRead, checksumWrite);
		ucRet = FAIL;
		return ucRet;
	}
	printk("@@@ISP FW: Load IMEM Bin File End\n");
	/* Load IMEM bin file End */

	/* @@@ End load code to SPCA7002 */

	/* Disable checksum mechanism */
	//I2CDataWrite(0x4280, 0);
	sensor_write_reg(client, 0x4280, 0);

	/* Write load code end register */
	//I2CDataWrite(0x1307, 0xA5);
	sensor_write_reg(client, 0x1307, 0xA5);

	return ucRet;
}



/*-------------------------------------------------------------------------
 *  Function Name : EXISP_UpdateCalibResStart
 *  Description : Update calibration data start
 *  ucFwIdx: Set which FW will be loaded
     pIspFw: external ISP FW pointer
     pFwInfo: external ISP FW header information
     pLoadCodeRet: load code status result
     ucCaliOpt: read from calibration_option.BIN
     p3acali: read from 3ACALI.BIN or 3ACALI_F.BIN
     pLsc: read from LSC.BIN or LSC_F.BIN
     pLscdq: read from LSC_DQ.BIN or LSC_DQ_F.BIN
 *  Return : none
 *------------------------------------------------------------------------*/
void EXISP_UpdateCalibResStart(
	UINT8 ucFwIdx,
	UINT8 *pIspFw,
	ispFwHeaderInfo_t *pFwInfo,
	ispLoadCodeRet_t *pLoadCodeRet,
	UINT8 ucCaliOpt,
	UINT8 *p3acali,
	UINT8 *pLsc,
	UINT8 *pLscdq)
{
	//printk("1.pIspFw =%x\n", pIspFw);

	if ((ucFwIdx == 0) && (ucCaliOpt != 0xFF)) { //rear sensor
		ucCaliOpt = ucCaliOpt & 0x03;
	}
	else if ((ucFwIdx == 1) && (ucCaliOpt != 0xFF)){ //front sensor
		ucCaliOpt = ucCaliOpt >> 2;
	}

	switch (ucCaliOpt) {
	case 1: /* load golden 3ACALI.BIN and calibrated LSC.BIN & LSC_DQ.BIN */
		if (pLsc == NULL || pLscdq == NULL) {
			pLoadCodeRet->retLsc = LOADCODE_LSC_FILE_ERR;
			pLoadCodeRet->retLscdq = LOADCODE_LSC_DQ_FILE_ERR;
			break;
		}
		pLoadCodeRet->retLsc = EXISP_UpdateCalibRes(1, pIspFw, pFwInfo, p3acali, pLsc, pLscdq);
		pLoadCodeRet->retLscdq = pLoadCodeRet->retLsc;
		break;
	case 2: /* load calibrated 3ACALI.BIN and golden LSC.BIN & LSC_DQ.BIN */
		if (p3acali == NULL) {
			pLoadCodeRet->ret3acli = LOADCODE_3ACALI_FILE_ERR;
			break;
		}
		pLoadCodeRet->ret3acli = EXISP_UpdateCalibRes(0, pIspFw, pFwInfo, p3acali, pLsc, pLscdq);
		break;
	case 3: /* load golden 3ACALI.BIN and golden LSC.BIN & LSC_DQ.BIN */
		//Nothing to do
		break;
	default: /* load calibrated 3ACALI.BIN and calibrated LSC.BIN & LSC_DQ.BIN */
		if (p3acali == NULL) {
			pLoadCodeRet->ret3acli = LOADCODE_3ACALI_FILE_ERR;
		}
		
		if (pLoadCodeRet->ret3acli == SUCCESS) {
			pLoadCodeRet->ret3acli = EXISP_UpdateCalibRes(0, pIspFw, pFwInfo, p3acali, pLsc, pLscdq);
		}		

		if (pLoadCodeRet->ret3acli == LOADCODE_GET_RES_NUM_ERR) {
			break;
		}
		else if (pLsc == NULL || pLscdq == NULL) {
			pLoadCodeRet->retLsc = LOADCODE_LSC_FILE_ERR;
			pLoadCodeRet->retLscdq = LOADCODE_LSC_DQ_FILE_ERR;
			break;
		}
		pLoadCodeRet->retLsc = EXISP_UpdateCalibRes(1, pIspFw, pFwInfo, p3acali, pLsc, pLscdq);
		pLoadCodeRet->retLscdq = pLoadCodeRet->retLsc;
		break;
	}
}



/*-------------------------------------------------------------------------
 *  Function Name : EXISP_UpdateCalibRes
 *  Description : Update calibration data from host, support boot from host only
 *  idx: Set which resource will be loaded
     pIspFw: external ISP FW pointer
     pFwInfo: external ISP FW header information
     p3acali: read from 3ACALI.BIN or 3ACALI_F.BIN
     pLsc: read from LSC.BIN or LSC_F.BIN
     pLscdq: read from LSC_DQ.BIN or LSC_DQ_F.BIN
 *  Return : status result
 *------------------------------------------------------------------------*/
UINT8 EXISP_UpdateCalibRes(
	UINT8 idx,
	UINT8 *pIspFw,
	ispFwHeaderInfo_t *pFwInfo,
	UINT8 *p3acali,
	UINT8 *pLsc,
	UINT8 *pLscdq)
{
	UINT32 iqOffset = 0, resNumCnt = 0, iqSize = 0, caliSize = 0, tempSize = 0, i;
	UINT32 start3acali = 0, startLsc = 0, startLscdq = 0, size3acali = 0, sizeLsc = 0, sizeLscdq = 0;
	UINT8 ucRet = SUCCESS;

	/* resource header and checksum check */
	ucRet = EXISP_ResCheck(idx, p3acali, pLsc, pLscdq);
	if (ucRet != SUCCESS) {
		goto _EXIT_;
	}

	/* find out where IQ.BIN is */
	pIspFw += FW_HEADER_SIZE + pFwInfo->DmemSize;
	//printk("2.pIspFw =%x\n", pIspFw);
	if (*(pIspFw+0x38) == 0x43 &&
		*(pIspFw+0x39) == 0x41 &&
		*(pIspFw+0x3A) == 0x4C &&
		*(pIspFw+0x3B) == 0x49) {
		iqOffset = ((*(pIspFw+0x51)<<8)&0x0000FF00) + (*(pIspFw+0x50)&0x000000FF);
	}
	else {
		iqOffset = ((*(pIspFw+0x31)<<8)&0x0000FF00) + (*(pIspFw+0x30)&0x000000FF);
	}
	//printk("iqOffset=%x\n", iqOffset);

	/* point to IQ.BIN start position */
	pIspFw += iqOffset;
	//printk("3.pIspFw =%x\n", pIspFw);

	/* parsing out the file size to get the start position of calibration data,
	FICDMEM file size should be 16 alignment */
	ucRet = EXISP_ResNumGet(&resNumCnt, pIspFw+0x10);
	if (ucRet != SUCCESS) {
		goto _EXIT_;
	}
	//printk("resNumCnt=%d\n", resNumCnt);
	//printk("5.pIspFw =%x\n", pIspFw);

	for (i = 0; i < resNumCnt; i++) {
		tempSize = *(pIspFw+14+(1+i)*0x10);
		tempSize += ((*(pIspFw+15+((1+i)*0x10)))<<8);
		//printk("tempSize=0x%02x\n", tempSize);
		if ((tempSize%0x10) != 0) {
			tempSize = ((tempSize+0xF)>>4)<<4;
		}
		//printk("size of IQ BIN files %d: 0x%02x\n", i, tempSize);
		iqSize += tempSize;
	}
	//printk("iqSize=%d\n", iqSize);
	//printk("6.pIspFw =%x\n", pIspFw);

	/* find out 3ACALI.BIN & LSC.BIN & LSC_DQ.BIN start position */
	start3acali = iqSize+(1+resNumCnt+3)*0x10;
	for (i = 0; i < 3; i++) {
		tempSize = *(pIspFw+14+(1+resNumCnt+i)*0x10);
		tempSize += ((*(pIspFw+15+(1+resNumCnt+i)*0x10))<<8);
		if (i == 0) {
			size3acali = tempSize;
		}
		else if (i == 1){
			sizeLsc = tempSize;
		}
		else {
			sizeLscdq = tempSize;
		}

		if ((tempSize%0x10) != 0) {
			tempSize = ((tempSize+0xF)>>4)<<4;
		}
		//printk("0.size of IQ BIN files %d: 0x%02x\n", i, tempSize);
		caliSize += tempSize;
		if (i == 0) {
			startLsc = start3acali + caliSize;
		}
		else if (i == 1) {
			startLscdq = start3acali + caliSize;
		}
	}
	//printk("start3acali=%x, size3acali=%d\n", start3acali, size3acali);
	//printk("startLsc=%x, sizeLsc=%d\n", startLsc, sizeLsc);
	//printk("startLscdq=%x, size=%d\n", startLscdq, sizeLscdq);

	/* update calibration data into FW buffer */
	if (idx == 0) {
		memcpy(pIspFw+start3acali, p3acali, size3acali);
	}
	else if (idx == 1) {
		memcpy(pIspFw+startLsc, pLsc, sizeLsc);
		memcpy(pIspFw+startLscdq, pLscdq, sizeLscdq);
	}

_EXIT_:

	return ucRet;
}



/*-------------------------------------------------------------------------
 *  Function Name : EXISP_ResCheck
 *  Description : check resource header and checksum
 *  idx: Set which resource will be loaded
     p3acali: read from 3ACALI.BIN or 3ACALI_F.BIN
     pLsc: read from LSC.BIN or LSC_F.BIN
     pLscdq: read from LSC_DQ.BIN or LSC_DQ_F.BIN
 *  Return : status result
 *------------------------------------------------------------------------*/
UINT8 EXISP_ResCheck(UINT8 idx, UINT8 *p3acali, UINT8 *pLsc, UINT8 *Lscdq)
{
	UINT8 ucRet = SUCCESS;
	UINT32 header_3ACALI = 0, header_LSC = 0, header_LSCDQ = 0;
	UINT32 chksuum_3ACALI = 0, chksuum_LSC = 0, chksuum_LSCDQ = 0, checksum = 0, dataSize = 0, i;

	/* Header check */
	if (idx == 0) { //3ACALI
		header_3ACALI = *(p3acali);
		header_3ACALI += (*(p3acali+1)<<8);
		header_3ACALI += (*(p3acali+2)<<16);
		header_3ACALI += (*(p3acali+3)<<24);
		if (header_3ACALI == 0xFFFFFFFF || header_3ACALI == 0x00000000) {
			printk("header_3ACALI=0x%04x\n", header_3ACALI);
			ucRet = LOADCODE_3ACALI_HEADER_ERR;
			goto _EXIT_;
		}
	}
	else if (idx == 1) { //LSC & LSC_DQ
		header_LSC = *(pLsc);
		header_LSC += (*(pLsc+1)<<8);
		header_LSC += (*(pLsc+2)<<16);
		header_LSC += (*(pLsc+3)<<24);
		if (header_LSC == 0xFFFFFFFF || header_LSC == 0x00000000) {
			printk("header_LSC=0x%04x\n", header_LSC);
			ucRet = LOADCODE_LSC_HEADER_ERR;
			goto _EXIT_;
		}

		header_LSCDQ = *(Lscdq);
		header_LSCDQ += (*(Lscdq+1)<<8);
		header_LSCDQ += (*(Lscdq+2)<<16);
		header_LSCDQ += (*(Lscdq+3)<<24);
		if (header_LSCDQ == 0xFFFFFFFF || header_LSCDQ == 0x00000000) {
			printk("header_LSCDQ=0x%04x\n", header_LSCDQ);
			ucRet = LOADCODE_LSC_DQ_HEADER_ERR;
			goto _EXIT_;
		}
	}

	/* Checksum check */
	if (idx == 0) { //3ACALI
		dataSize = *(p3acali+6);
		dataSize += (*(p3acali+7)<<8);
		checksum = *(p3acali+RES_3ACALI_HEADER_SIZE);
		checksum += (*(p3acali+RES_3ACALI_HEADER_SIZE+1)<<8);
		checksum += (*(p3acali+RES_3ACALI_HEADER_SIZE+2)<<16);
		checksum += (*(p3acali+RES_3ACALI_HEADER_SIZE+3)<<24);
		
		for (i = 0; i < dataSize-sizeof(UINT32); i++) {
			chksuum_3ACALI = chksuum_3ACALI + (*(p3acali+RES_3ACALI_HEADER_SIZE+sizeof(UINT32)+i));
		}
		if (chksuum_3ACALI != checksum) {
			printk("dataSize=%d, checksum=0x%04x, chksuum_3ACALI=0x%04x\n", dataSize, checksum, chksuum_3ACALI);
			ucRet = LOADCODE_3ACALI_CHKSUM_ERR;
			goto _EXIT_;
		}
	}
	else if (idx == 1) { //LSC & LSC_DQ
		dataSize = *(pLsc+6);
		dataSize += (*(pLsc+7)<<8);
		checksum = *(pLsc+RES_LSC_HEADER_SIZE-4);
		checksum += (*(pLsc+RES_LSC_HEADER_SIZE-3)<<8);
		checksum += (*(pLsc+RES_LSC_HEADER_SIZE-2)<<16);
		checksum += (*(pLsc+RES_LSC_HEADER_SIZE-1)<<24);
		for (i = 0; i < dataSize; i++) {
			chksuum_LSC = chksuum_LSC + (*(pLsc+RES_LSC_HEADER_SIZE+i));
		}
		if (chksuum_LSC != checksum) {
			printk("dataSize=%d, checksum=0x%04x, chksuum_LSC=0x%04x\n", dataSize, checksum, chksuum_LSC);
			ucRet = LOADCODE_LSC_CHKSUM_ERR;
			goto _EXIT_;
		}

		dataSize = *(Lscdq+6);
		dataSize += (*(Lscdq+7)<<8);
		checksum = *(Lscdq+RES_LSCDQ_HEADER_SIZE-4);
		checksum += (*(Lscdq+RES_LSCDQ_HEADER_SIZE-3)<<8);
		checksum += (*(Lscdq+RES_LSCDQ_HEADER_SIZE-2)<<16);
		checksum += (*(Lscdq+RES_LSCDQ_HEADER_SIZE-1)<<24);
		for (i = 0; i < dataSize; i++) {
			chksuum_LSCDQ = chksuum_LSCDQ + (*(Lscdq+RES_LSCDQ_HEADER_SIZE+i));
		}
		if (chksuum_LSCDQ != checksum) {
			printk("dataSize=%d, checksum=0x%04x, chksuum_LSCDQ=0x%04x\n", dataSize, checksum, chksuum_LSCDQ);
			ucRet = LOADCODE_LSC_DQ_CHKSUN_ERR;
			goto _EXIT_;
		}
	}

_EXIT_:

	return ucRet;
}



/*-------------------------------------------------------------------------
 *  Function Name : EXISP_ResNumGet
 *  Description : get the resource number in the IQ.BIN
 *  resNum: resource number
     pIspFw: external ISP FW pointer
 *  Return : status result
 *------------------------------------------------------------------------*/
UINT8 EXISP_ResNumGet(UINT32 *resNum, UINT8 *pIspFw)
{
	UINT8 i = 0, ucRet = SUCCESS;

	//printk("4.pIspFw =%x\n", pIspFw);
	while (1) {
		//printk("[%s] %d 0x%2x 0x%2x 0x%2x 0x%2x 0x%2x 0x%2x\n", __FUNCTION__, __LINE__,
//			*(pIspFw),*(pIspFw+1),*(pIspFw+2), *(pIspFw+3),*(pIspFw+4),*(pIspFw+5));
		if ((*(pIspFw) == 0x33) && (*(pIspFw+1) == 0x41) && (*(pIspFw+2) == 0x43) &&
			(*(pIspFw+3)==0x41) && (*(pIspFw+4)==0x4C) && (*(pIspFw+5)==0x49)) {
			break;
		}
		i++;
		pIspFw += 0x10;
		if (i > 30) {
			ucRet = LOADCODE_GET_RES_NUM_ERR;
			goto _EXIT_;
		}
	}

_EXIT_:
	*resNum = i;

	return ucRet;
}



static int spi_init_extra_parameter()
{
	struct file *fp = NULL;
	int ret = -1;
	u8 *pbootBuf = NULL;

	struct inode *inode;
	int bootbin_size = 0;

	mm_segment_t old_fs;
	printk("%s ++\n", __func__);

	fp = filp_open(SPI_FILE_WITH_PATH, O_RDONLY, 0);
	
	if ( !IS_ERR_OR_NULL(fp) ){
		pr_info("filp_open success fp:%p\n", fp);
		inode = fp->f_dentry->d_inode;
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		if(fp->f_op != NULL && fp->f_op->read != NULL){
			printk("Start to read %s\n", SPI_FILE_WITH_PATH);
		}
		set_fs(old_fs);
		filp_close(fp, NULL);
	} else if(PTR_ERR(fp) == -ENOENT) {
		pr_err(" \"%s\" not found error\n", SPI_FILE_WITH_PATH);
		SPI_FILE_WITH_PATH = DEFAULT_SPI_FILE_WITH_PATH;

	} else{
		pr_err(" \"%s\" open error\n", SPI_FILE_WITH_PATH);
		SPI_FILE_WITH_PATH = DEFAULT_SPI_FILE_WITH_PATH;
	}


	char *pFile = SPI_FILE_WITH_PATH;
	fp = filp_open(pFile, O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);

	if ( !IS_ERR_OR_NULL(fp) ){
		inode = fp->f_dentry->d_inode;
		bootbin_size = inode->i_size;
		if (bootbin_size > 0) {
			pbootBuf = kmalloc(bootbin_size, GFP_KERNEL);
			old_fs = get_fs();
			set_fs(KERNEL_DS);
			if(fp->f_op != NULL && fp->f_op->read != NULL){
				int byte_count = 0;
				printk("Start to read %s\n", SPI_FILE_WITH_PATH);
				byte_count = fp->f_op->read(fp, pbootBuf, bootbin_size, &fp->f_pos);
			}
			set_fs(old_fs);
		}
		filp_close(fp, NULL);
	} else {
		printk("No extra parameter\n");
		return 0;
	}
	
	if (bootbin_size > 0) {	
		ret = pbootBuf[0];
		kfree(pbootBuf);
	}

	return ret;
}
//FW_BSP--


static int mt9m114_init_common(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
//FW_BSP++
	int status;
//FW_BSP--


//FW_BSP++
	if (GET_SENSOR_ID==false) {
		ret = sensor_write_reg(client, 0x1011, 0x01);//cpu reset
		if(ret) {
			printk("%s, I2C transfer fail, break!!\n", __func__);
			mt9m114_s_power(sd, 0);
			return -EINVAL;
		}
		sensor_write_reg(client, 0x001C, 0x08);/* reset FM */
		sensor_write_reg(client, 0x001C, 0x00);
		sensor_write_reg(client, 0x1010, 0x02);
		sensor_write_reg(client, 0x1010, 0x00);


		// Start - Power on sensor & enable clock
		sensor_write_reg(client, 0x0084, 0x14);	// To sensor clock divider
		sensor_write_reg(client, 0x0034, 0xFF);	// Turn on all clock
		sensor_write_reg(client, 0x9030, 0x3F);
		sensor_write_reg(client, 0x9031, 0x04);
		sensor_write_reg(client, 0x9034, 0xF2);
		sensor_write_reg(client, 0x9035, 0x04);
		sensor_write_reg(client, 0x9033, 0x04);
		sensor_write_reg(client, 0x9032, 0x3C);
		mdelay(10);
		sensor_write_reg(client, 0x9033, 0x00);
		mdelay(10);
		sensor_write_reg(client, 0x9033, 0x04);
		sensor_write_reg(client, 0x9032, 0x3e);
		mdelay(10);
		sensor_write_reg(client, 0x9032, 0x3c);
		// End - Power on sensor & enable clock

		// Start - I2C Read
		sensor_write_reg(client, 0x9138, 0x30);	// Sub address enable
		sensor_write_reg(client, 0x9140, 0x90);	// Slave address
		sensor_write_reg(client, 0x9100, 0x03);	// Read mode
		sensor_write_reg(client, 0x9110, 0x00);	// Register addr MSB
		sensor_write_reg(client, 0x9112, 0x00);	// Register add LSB
		sensor_write_reg(client, 0x9104, 0x01);	// Trigger I2C read
		mdelay(5);
		sensor_read_reg(client, 0x9111, &retvalue_h);

		sensor_write_reg(client, 0x9110, 0x00);	// Register addr MSB
		sensor_write_reg(client, 0x9112, 0x01);	// Register addr LSB
		sensor_write_reg(client, 0x9104, 0x01);	// Trigger I2C read
		mdelay(5);
		sensor_read_reg(client, 0x9111, &retvalue_l);


		sensorid = ((retvalue_h<<8)|(retvalue_l))&0xffff;
		printk("%s Sensor ID = 0x%x\n", __func__, sensorid);
		GET_SENSOR_ID=true;
	}
	

	if (SPI_ENABLE==0) {
		ret = sensor_write_reg(client, 0x1011, 0x01);//cpu reset
		if(ret) {
			printk("%s, I2C transfer fail, break!!\n", __func__);
			return -EINVAL;
		}
		sensor_write_reg(client, 0x001C, 0x08);/* reset FM */
		sensor_write_reg(client, 0x001C, 0x00);
		sensor_write_reg(client, 0x1010, 0x02);
		sensor_write_reg(client, 0x1010, 0x00);
		sensor_write_reg(client, 0x1306, 0x01);//front camera
		sensor_write_reg(client, 0x1011, 0x00);
		msleep(70); //wait for iCatch load sensor code		
	}else {
		if ((FW_BIN_FILE==true)&&((FIRST_BOOTING==false)||(build_version==1))) {
			status = EXISP_LoadCode(0x01, g_is_calibration, pIspFW_g, pCalibOpt_g, p3acali_g, pLsc_g, pLscdq_g, sd);
			fw_update_status = ICATCH_FW_UPDATE_SUCCESS;
			sensor_write_reg(client, 0x1306, 0x01); /* Set which FW to be loaded */ 
		}
		FIRST_BOOTING=false;
	}
//FW_BSP--
	return 0;
}

static int power_up(struct v4l2_subdev *sd)
{
	struct mt9m114_device *dev = to_mt9m114_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	if (NULL == dev->platform_data) {
		dev_err(&client->dev, "no camera_sensor_platform_data");
		return -ENODEV;
	}

	//clk and gpio control are moved to power_ctrl function
	/* power control */
	ret = dev->platform_data->power_ctrl(sd, 1);
	if (ret)
		goto fail_power;
#if 0
	/* flis clock control */
	ret = dev->platform_data->flisclk_ctrl(sd, 1);
	if (ret)
		goto fail_clk;

	/* gpio ctrl */
	ret = dev->platform_data->gpio_ctrl(sd, 1);
	if (ret)
		dev_err(&client->dev, "gpio failed 1\n");
#endif

	return ret;
#if 0
fail_clk:
	dev->platform_data->flisclk_ctrl(sd, 0);
#endif
fail_power:
	dev->platform_data->power_ctrl(sd, 0);
	dev_err(&client->dev, "sensor power-up failed\n");

	return ret;
}

static int power_down(struct v4l2_subdev *sd)
{
	struct mt9m114_device *dev = to_mt9m114_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	if (NULL == dev->platform_data) {
		dev_err(&client->dev, "no camera_sensor_platform_data");
		return -ENODEV;
	}
#if 0
	ret = dev->platform_data->flisclk_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "flisclk failed\n");

	/* gpio ctrl */
	ret = dev->platform_data->gpio_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "gpio failed 1\n");
#endif
	/* power control */
	ret = dev->platform_data->power_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "vprog failed.\n");

	first_on = 0;

	return ret;
}

static int mt9m114_s_power(struct v4l2_subdev *sd, int power)
{
	if (power == 0)
		return power_down(sd);

	if (power_up(sd))
		return -EINVAL;

	return mt9m114_init_common(sd);
}

static int mt9m114_try_res(u32 *w, u32 *h)
{
	int i;

	//printk("%s\n", __func__);

	/*
	 * The mode list is in ascending order. We're done as soon as
	 * we have found the first equal or bigger size.
	 */
	for (i = 0; i < N_RES; i++) {
		if ((mt9m114_res[i].width >= *w) &&
		    (mt9m114_res[i].height >= *h))
			break;
	}

	/*
	 * If no mode was found, it means we can provide only a smaller size.
	 * Returning the biggest one available in this case.
	 */
	if (i == N_RES)
		i--;

	*w = mt9m114_res[i].width;
	*h = mt9m114_res[i].height;

	return 0;
}

static struct mt9m114_res_struct *mt9m114_to_res(u32 w, u32 h)
{
	int  index;

	//printk("%s\n", __func__);

	for (index = 0; index < N_RES; index++) {
		if ((mt9m114_res[index].width == w) &&
		    (mt9m114_res[index].height == h))
			break;
	}

	/* No mode found */
	if (index >= N_RES)
		return NULL;

	return &mt9m114_res[index];
}

static int mt9m114_try_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	return mt9m114_try_res(&fmt->width, &fmt->height);
}

static int mt9m114_res2size(unsigned int res, int *h_size, int *v_size)
{
	unsigned short hsize;
	unsigned short vsize;

	//printk("%s\n", __func__);

	switch (res) {
	case MT9M114_RES_QCIF:
		hsize = MT9M114_RES_QCIF_SIZE_H;
		vsize = MT9M114_RES_QCIF_SIZE_V;
		break;
	case MT9M114_RES_QVGA:
		hsize = MT9M114_RES_QVGA_SIZE_H;
		vsize = MT9M114_RES_QVGA_SIZE_V;
		break;
	case MT9M114_RES_VGA:
		hsize = MT9M114_RES_VGA_SIZE_H;
		vsize = MT9M114_RES_VGA_SIZE_V;
		break;
/*
	case MT9M114_RES_480P:
		hsize = MT9M114_RES_480P_SIZE_H;
		vsize = MT9M114_RES_480P_SIZE_V;
		break;
*/
	case MT9M114_RES_720P:
		hsize = MT9M114_RES_720P_SIZE_H;
		vsize = MT9M114_RES_720P_SIZE_V;
		break;
	case MT9M114_RES_960P:
		hsize = MT9M114_RES_960P_SIZE_H;
		vsize = MT9M114_RES_960P_SIZE_V;
		break;
	default:
		WARN(1, "%s: Resolution 0x%08x unknown\n", __func__, res);
		return -EINVAL;
	}

	if (h_size != NULL)
		*h_size = hsize;
	if (v_size != NULL)
		*v_size = vsize;
	return 0;
}

static int mt9m114_get_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	struct mt9m114_device *dev = to_mt9m114_sensor(sd);
	int width, height;
	int ret;

	printk("%s\n", __func__);

	ret = mt9m114_res2size(dev->res, &width, &height);
	if (ret)
		return ret;
	fmt->width = width;
	fmt->height = height;

	return 0;
}

static int mt9m114_set_mbus_fmt(struct v4l2_subdev *sd,
			      struct v4l2_mbus_framefmt *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9m114_device *dev = to_mt9m114_sensor(sd);
	struct mt9m114_res_struct *res_index;
	u32 width = fmt->width;
	u32 height = fmt->height;
	int ret;
	u16 testval, i, addr;

	printk("%s\n", __func__);

	mt9m114_try_res(&width, &height);
	res_index = mt9m114_to_res(width, height);

	/* Sanity check */
	if (unlikely(!res_index)) {
		WARN_ON(1);
		return -EINVAL;
	}

	switch (res_index->res) {
//TODO: add supported resolution later
/*
	case MT9M114_RES_QCIF:
		ret = mt9m114_write_reg_array(c, mt9m114_qcif_30_init);
		dev_info(&c->dev, "%s: set for qcif\n", __func__);
		break;
	case MT9M114_RES_QVGA:
		ret = mt9m114_write_reg_array(c, mt9m114_qvga_30_init);
		dev_info(&c->dev, "%s: set for qvga\n", __func__);
		break;
*/
	case MT9M114_RES_VGA:
		sensor_write_reg(client, 0x7106, 0x0B); //SET resolution 640x480
		dev_info(&client->dev, "%s: set for VGA\n", __func__);
		break;
/*
	case MT9M114_RES_480P:
		ret = mt9m114_write_reg_array(c, mt9m114_svga_30_init);
		dev_info(&c->dev, "%s: set for svga\n", __func__);
		break;
*/
	case MT9M114_RES_720P:
		sensor_write_reg(client, 0x7106, 0x04); //SET Preview resolution 1280x720
		dev_info(&client->dev, "%s: set for 720p\n", __func__);
		break;
	case MT9M114_RES_960P:
		sensor_write_reg(client, 0x7106, 0x00); //SET Preview resolution 1280x960
		dev_info(&client->dev, "%s: set for 960p\n", __func__);
		break;
	default:
		dev_err(&client->dev, "set resolution: %d failed!\n",
			res_index->res);
		return -EINVAL;
	}

	if (ret)
		return ret;

	sensor_write_reg(client, 0x7120, 0x00); //Enter Preview mode

	//wait interrupt 0x72F8.2
	for (i=0;i<200;i++) {
		sensor_read_reg(client, 0x72f8, &testval);
		//printk("testval=0x%X, i=%d ",testval,i);
		if (testval & 0x04) {
			sensor_write_reg(client, 0x72f8, 0x04);
			sensor_read_reg(client, 0x72f8, &testval);
			printk("Clear testval=0x%X, i=%d\n",testval,i);
			break;
		}
		msleep(5);
	}
	if(i == 200) {
		pr_info("Change to preview mode fail\n");

		pr_info("-- Dump iCatch register now --\n");
		for (addr=0x7200;addr<0x7280;addr++) {
			sensor_read_reg(client, addr, &testval);
			printk("addr=0x%x, value=0x%x \n",addr,testval);
		}
		for (addr=0x7005;addr<0x7007;addr++) {
			sensor_read_reg(client, addr, &testval);
			printk("addr=0x%x, value=0x%x \n",addr,testval);
		}
		for (addr=0x2030;addr<0x205C;addr++) {
			sensor_read_reg(client, addr, &testval);
			printk("addr=0x%x, value=0x%x \n",addr,testval);
		}
		for (addr=0x7070;addr<0x7076;addr++) {
			sensor_read_reg(client, addr, &testval);
			printk("addr=0x%x, value=0x%x \n",addr,testval);
		}
		sensor_read_reg(client, 0x72f8, &testval);
		printk("addr=0x72f8, value=0x%x \n",testval);
		pr_info("-- Dump iCatch register Down --\n");

		return -ENOMEM;
	}

//ASUS_BSP+++, RAW patch
	if (fmt->colorspace == V4L2_COLORSPACE_SRGB) {
		/*icatch RAW output*/
		printk("icatch RAW output\n");
		sensor_write_reg(client, 0x7160, 0x00);
		sensor_write_reg(client, 0x7161, 0x01);
#if 0
		printk("icatch test pattern - enable \n");
		sensor_write_reg(client, 0x90d0, 0x01);

		printk("icatch test pattern - color bar\n");
		sensor_write_reg(client, 0x90d6, 0x0a);
#endif
	}
//ASUS_BSP---, RAW patch

	dev->res = res_index->res;

	fmt->width = width;
	fmt->height = height;

	return 0;
}

static int mt9m114_g_focal(struct v4l2_subdev *sd, s32 *val)
{
	*val = (MT9M114_FOCAL_LENGTH_NUM << 16) | MT9M114_FOCAL_LENGTH_DEM;
	return 0;
}

static int mt9m114_g_fnumber(struct v4l2_subdev *sd, s32 *val)
{
	/* const f number for MT9M114 */
	*val = (MT9M114_F_NUMBER_DEFAULT_NUM << 16) | MT9M114_F_NUMBER_DEM;
	return 0;
}

static int mt9m114_g_fnumber_range(struct v4l2_subdev *sd, s32 *val)
{
	*val = (MT9M114_F_NUMBER_DEFAULT_NUM << 24) |
		(MT9M114_F_NUMBER_DEM << 16) |
		(MT9M114_F_NUMBER_DEFAULT_NUM << 8) | MT9M114_F_NUMBER_DEM;
	return 0;
}

/*
 * This returns EV.
 */
static int mt9m114_get_exposure_bias(struct v4l2_subdev *sd, s32 *value)
{
	*value = 0;

	return 0;
}

/*
 * This returns ISO sensitivity.
 */
static int sensor_g_iso(struct v4l2_subdev *sd, s32 *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int iso_value = 0;
	//int iso_value, iso_ratio = 0, iso = 0x0;
	u16 retvalue_h, retvalue_l;
	int ret = 0;

	switch (iso_setting)
	{
		case 0:
			ret = sensor_read_reg(client, 0x72B7, &retvalue_l); //iso[7:0]
			ret = sensor_read_reg(client, 0x72B8, &retvalue_h); //iso[15:8]
			iso_value = (retvalue_h<<8)|(retvalue_l);
/*
			iso_ratio = iso / 50;
			if (iso_ratio < 2)
				iso_value = 50;
			else if (iso_ratio < 4)
				iso_value = 100;
			else if (iso_ratio < 8)
				iso_value = 200;
			else if (iso_ratio < 16)
				iso_value = 400;
			else
				iso_value = 800;
*/
			break;
		case 1:
			iso_value = 50;
			break;
		case 2:
			iso_value = 100;
			break;
		case 3:
			iso_value = 200;
			break;
		case 4:
			iso_value = 400;
			break;
		case 5:
			iso_value = 800;
			break;
		default:
			iso_value = 100;
			break;
	}

	printk("%s, iso exif %d\n", __func__, iso_value);

	*value = iso_value;

	return 0;
}

//TODO: add camera parameter later
static struct mt9m114_control mt9m114_controls[] = {
	{
		.qc = {
			.id = V4L2_CID_AUTO_EXPOSURE_BIAS,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "exposure bias",
			.minimum = 0x0,
			.maximum = 0xffff,
			.step = 0x01,
			.default_value = 0x00,
			.flags = 0,
		},
		.query = mt9m114_get_exposure_bias, //For EXIF value, need to combine with other parameter
	},
	{
		.qc = {
			.id = V4L2_CID_ISO_SENSITIVITY,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "iso",
			.minimum = 0x0,
			.maximum = 0xffff,
			.step = 0x01,
			.default_value = 0x00,
			.flags = 0,
		},
		.query = sensor_g_iso, //For EXIF value, need to combine with other parameter
		.tweak = sensor_s_iso,
	},
	{
		.qc = {
			.id = V4L2_CID_COLORFX,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "color effect",
			.minimum = 0,
			.maximum = 15,
			.step = 1,
			.default_value = 0,
		},
		.tweak = sensor_s_color_effect,
	},
	{
		.qc = {
			.id = V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "white balance",
			.minimum = 0,
			.maximum = 9,
			.step = 1,
			.default_value = 0,
		},
		.tweak = sensor_s_white_balance,
	},
	{
		.qc = {
			.id = V4L2_CID_SCENE_MODE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "scene mode",
			.minimum = 0,
			.maximum = 13,
			.step = 1,
			.default_value = 0,
		},
		.tweak = sensor_s_scene_mode,
	},
	{
		.qc = {
			.id = V4L2_CID_EXPOSURE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "exposure",
			.minimum = -6,
			.maximum = 6,
			.step = 1,
			.default_value = 0,
		},
		.tweak = sensor_s_ev,
		.query = sensor_g_exposure,
	},
	{
		.qc = {
			.id = V4L2_CID_POWER_LINE_FREQUENCY,
			.type = V4L2_CTRL_TYPE_MENU,
			.name = "Light frequency filter",
			.minimum = 1,
			.maximum = 3,
			.step = 1,
			.default_value = 1,
		},
		.tweak = sensor_s_flicker,
	},
	{
		.qc = {
			.id = V4L2_CID_EXPOSURE_METERING,
			.type = V4L2_CTRL_TYPE_MENU,
			.name = "metering",
			.minimum = 0,
			.maximum = 2,
			.step = 1,
			.default_value = 1,
		},
		.tweak = sensor_s_ae_metering_mode,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCAL_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focal length",
			.minimum = MT9M114_FOCAL_LENGTH_DEFAULT,
			.maximum = MT9M114_FOCAL_LENGTH_DEFAULT,
			.step = 0x01,
			.default_value = MT9M114_FOCAL_LENGTH_DEFAULT,
			.flags = 0,
		},
		.query = mt9m114_g_focal,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number",
			.minimum = MT9M114_F_NUMBER_DEFAULT,
			.maximum = MT9M114_F_NUMBER_DEFAULT,
			.step = 0x01,
			.default_value = MT9M114_F_NUMBER_DEFAULT,
			.flags = 0,
		},
		.query = mt9m114_g_fnumber,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_RANGE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number range",
			.minimum = MT9M114_F_NUMBER_RANGE,
			.maximum =  MT9M114_F_NUMBER_RANGE,
			.step = 0x01,
			.default_value = MT9M114_F_NUMBER_RANGE,
			.flags = 0,
		},
		.query = mt9m114_g_fnumber_range,
	},
//ASUS_BSP++, add for calibration
	{
		.qc = {
			.id = CUSTOM_IOCTL_REG_SET,
		},
		.tweak = sensor_s_write_reg,
	},
	{
		.qc = {
			.id = CUSTOM_IOCTL_REG_GET,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "REG get",
			.minimum = 0,
			.maximum = 9,
			.step = 1,
			.default_value = 0,
		},
		.query = sensor_s_read_reg,
	},
	{
		.qc = {
			.id = CUSTOM_IOCTL_SPI_GET,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "SPI get",
			.minimum = 0,
			.maximum = 9,
			.step = 1,
			.default_value = 0,
		},
		.query = sensor_s_read_spi,
	},		
//ASUS_BSP--, add for calibration

//ASUS_BSP+++, for iCatch 3A information
	{
		.qc = {
			.id = V4L2_CID_PAN_RELATIVE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "AE1",
			.minimum = 0,
			.maximum = 6,
			.step = 1,
			.default_value = 0,
		},
		.query = sensor_g_3a_info_ae1,
	},
	{
		.qc = {
			.id = V4L2_CID_TILT_RELATIVE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "AE2",
			.minimum = 0,
			.maximum = 6,
			.step = 1,
			.default_value = 0,
		},
		.query = sensor_g_3a_info_ae2,
	},
	{
		.qc = {
			.id = V4L2_CID_PAN_RESET,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "AWB1",
			.minimum = 0,
			.maximum = 6,
			.step = 1,
			.default_value = 0,
		},
		.query = sensor_g_3a_info_awb1,
	},
	{
		.qc = {
			.id = V4L2_CID_TILT_RESET,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "AWB2",
			.minimum = 0,
			.maximum = 6,
			.step = 1,
			.default_value = 0,
		},
		.query = sensor_g_3a_info_awb2,
	},
	{
		.qc = {
			.id = V4L2_CID_PAN_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "AF1",
			.minimum = 0,
			.maximum = 6,
			.step = 1,
			.default_value = 0,
		},
		.query = sensor_g_3a_info_af1,
	},
	{
		.qc = {
			.id = V4L2_CID_TILT_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "AF2",
			.minimum = 0,
			.maximum = 6,
			.step = 1,
			.default_value = 0,
		},
		.query = sensor_g_3a_info_af2,
	},
//ASUS_BSP---, for iCatch 3A information
};

#define N_CONTROLS (ARRAY_SIZE(mt9m114_controls))

static struct mt9m114_control *mt9m114_find_control(__u32 id)
{
	int i;

	for (i = 0; i < N_CONTROLS; i++) {
		if (mt9m114_controls[i].qc.id == id)
			return &mt9m114_controls[i];
	}
	return NULL;
}

static int mt9m114_detect(struct mt9m114_device *dev, struct i2c_client *client)
{
	struct i2c_adapter *adapter = client->adapter;
	int ret = 0, i = 0, status;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s: i2c error", __func__);
		return -ENODEV;
	}

	if (sensorid!=0x2481) {
		if (SPI_ENABLE==1) {
			if (FW_BIN_FILE==true) {
				if (build_version!=1) {
					status = EXISP_LoadCode(0x01, g_is_calibration, pIspFW_g, pCalibOpt_g, p3acali_g, pLsc_g, pLscdq_g, &dev->sd);
					fw_update_status = ICATCH_FW_UPDATE_SUCCESS;
					sensor_write_reg(client, 0x1306, 0x01); /* Set which FW to be loaded */ 
				}

				printk("%s: Read Sensor ID again\n", __func__);
				msleep(300);
				for (i = 0; i<10; i++) {
					ret = sensor_read_reg(client, 0x72C8, &retvalue_l); //low byte of front sensor ID
					ret = sensor_read_reg(client, 0x72C9, &retvalue_h); //high byte of front sensor ID
					sensorid = ((retvalue_h<<8)|(retvalue_l))&0xffff;
					printk("%s Sensor ID = 0x%x\n", __func__, sensorid);	
					if (sensorid==0x2481) {
						break;	
					}
					msleep(5);				
				}
			}
		} else {
			printk("%s: Read Sensor ID again\n", __func__);
			msleep(300);
			for (i = 0; i<10; i++) {
				ret = sensor_read_reg(client, 0x72C8, &retvalue_l); //low byte of front sensor ID
				ret = sensor_read_reg(client, 0x72C9, &retvalue_h); //high byte of front sensor ID
				sensorid = ((retvalue_h<<8)|(retvalue_l))&0xffff;
				printk("%s Sensor ID = 0x%x\n", __func__, sensorid);	
				if (sensorid==0x2481) {
					break;	
				}
				msleep(5);
			}
		}
	}
	
	if(sensorid==0x2481) {
		ATD_mt9m114_status = 1;
	}else{
		ATD_mt9m114_status = 0;
		printk("%s Sensor ID is not match\n", __func__);
	}

	return 0;
}

static int
mt9m114_s_config(struct v4l2_subdev *sd, int irq, void *platform_data)
{
	struct mt9m114_device *dev = to_mt9m114_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct file *fp = NULL;
	mm_segment_t old_fs;
	struct inode *inode;
	int status, byte_count = 0;
	UINT32 size_CalibOpt=0, size_3acali=0, size_Lsc=0, size_Lscdq=0;	
	int ret;

	if (NULL == platform_data)
		return -ENODEV;

	dev->platform_data =
	    (struct camera_sensor_platform_data *)platform_data;

	if (dev->platform_data->platform_init) {
		ret = dev->platform_data->platform_init(client);
		if (ret) {
			dev_err(&client->dev, "mt9m114 platform init err\n");
			return ret;
		}
	}

//FW_BSP++

	fp = filp_open(FW_BIN_FILE_WITH_PATH, O_RDONLY, 0);
	
	if ( !IS_ERR_OR_NULL(fp) ){
		pr_info("filp_open success fp:%p\n", fp);
		inode = fp->f_dentry->d_inode;
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		if(fp->f_op != NULL && fp->f_op->read != NULL){
			printk("Start to read %s\n", FW_BIN_FILE_WITH_PATH);
		}
		set_fs(old_fs);
		filp_close(fp, NULL);
	} else {
		pr_err("iCatch \"%s\" not found error\n", FW_BIN_FILE_WITH_PATH);
		FW_BIN_FILE_WITH_PATH = NEW_FW_BIN_FILE_WITH_PATH;
		fp = filp_open(FW_BIN_FILE_WITH_PATH, O_RDONLY, 0);
		if ( !IS_ERR_OR_NULL(fp) ){
			pr_info("filp_open success fp:%p\n", fp);
			inode = fp->f_dentry->d_inode;
			old_fs = get_fs();
			set_fs(KERNEL_DS);
			if(fp->f_op != NULL && fp->f_op->read != NULL){
				printk("Start to read %s\n", FW_BIN_FILE_WITH_PATH);
			}
			set_fs(old_fs);
			filp_close(fp, NULL);
		} else {
			pr_err("iCatch \"%s\" open error\n", FW_BIN_FILE_WITH_PATH);
			FW_BIN_FILE_WITH_PATH = DEFAULT_FW_BIN_FILE_WITH_PATH;
		}
	}



	if (HW_ID == 0xFF){
		HW_ID = Read_HW_ID();
	}

	if (PROJECT_ID == 0xFF) {
		PROJECT_ID = Read_PROJ_ID();
	}	
	

	if (PROJECT_ID==PROJ_ID_ME302C) {
		switch (HW_ID) {
			case HW_ID_SR1:
			case HW_ID_SR2:
			case HW_ID_ER:		
				SPI_ENABLE=0;
				break;
			case HW_ID_PR:
			case HW_ID_MP:
				SPI_ENABLE=1;
				break;
			default:
				SPI_ENABLE=1;
		}
	}

	if (PROJECT_ID==PROJ_ID_ME372CG) {
		switch (HW_ID) {
			case HW_ID_SR1:	//for EVB
				SPI_ENABLE=0;
				break;				
			case HW_ID_SR2:	//for SR1
			case HW_ID_ER:		
			case HW_ID_PR:
			case HW_ID_MP:
				SPI_ENABLE=1;
				break;
			default:
				SPI_ENABLE=1;
		}
	}	

	if (PROJECT_ID==PROJ_ID_ME372C) {
		switch (HW_ID) {
			case HW_ID_SR1:	//for EVB
				SPI_ENABLE=0;
				break;				
			case HW_ID_SR2:	//for SR1
			case HW_ID_ER:		
			case HW_ID_PR:
			case HW_ID_MP:
				SPI_ENABLE=1;
				break;
			default:
				SPI_ENABLE=1;
		}
	}	
	

	if (PROJECT_ID==PROJ_ID_GEMINI) {
		switch (HW_ID) {
			case HW_ID_SR1:
				SPI_ENABLE=0;
				break;				
			case HW_ID_SR2:
			case HW_ID_ER:		
			case HW_ID_PR:
			case HW_ID_MP:
				SPI_ENABLE=1;
				break;
			default:
				SPI_ENABLE=1;
		}
	}	

	SPI_ret=spi_init_extra_parameter();

	if (SPI_ret==SPI_magic_number) {
		SPI_ENABLE=1;		
	} else if (SPI_ret==EEPROM_magic_number) {
		SPI_ENABLE=0;
	}


	if (SPI_ENABLE==1) {
			
			/* Calculate BOOT.BIN file size. */
			FW_BIN_FILE=false;
			fp = filp_open(FW_BIN_FILE_WITH_PATH, O_RDONLY, 0);

			if ( !IS_ERR_OR_NULL(fp) ){
				pr_info("filp_open success fp:%p\n", fp);
				inode = fp->f_dentry->d_inode;
				bootbin_size_g = inode->i_size;
				printk("%s: fp->f_dentry->d_inode->i_size=%d\n", __FUNCTION__, bootbin_size_g);
				if (bootbin_size_g > 0) {
					pIspFW_g = kmalloc(bootbin_size_g, GFP_KERNEL);
					old_fs = get_fs();
					set_fs(KERNEL_DS);
					if(fp->f_op != NULL && fp->f_op->read != NULL){
						byte_count = 0;
						printk("Start to read %s\n", FW_BIN_FILE_WITH_PATH);
						byte_count = fp->f_op->read(fp, pIspFW_g, bootbin_size_g, &fp->f_pos);
						printk("iCatch: BIN file size= %d bytes\n", bootbin_size_g);
					}
					set_fs(old_fs);
					FW_BIN_FILE=true;
				}
				filp_close(fp, NULL);


				fp = filp_open(CALIBOPT_FILE_WITH_PATH, O_RDONLY, 0);
				
				if ( !IS_ERR_OR_NULL(fp) ){
					pr_info("filp_open success fp:%p\n", fp);
					inode = fp->f_dentry->d_inode;
					old_fs = get_fs();
					set_fs(KERNEL_DS);
					if(fp->f_op != NULL && fp->f_op->read != NULL){
						printk("Start to read %s\n", CALIBOPT_FILE_WITH_PATH);
					}
					set_fs(old_fs);
					filp_close(fp, NULL);
				} else if(PTR_ERR(fp) == -ENOENT) {
					pr_err(" \"%s\" not found error\n", CALIBOPT_FILE_WITH_PATH);
					CALIBOPT_FILE_WITH_PATH = DEFAULT_CALIBOPT_FILE_WITH_PATH;
				
				} else{
					pr_err(" \"%s\" open error\n", CALIBOPT_FILE_WITH_PATH);
					CALIBOPT_FILE_WITH_PATH = DEFAULT_CALIBOPT_FILE_WITH_PATH;
				}


				
				fp = filp_open(CALIBOPT_FILE_WITH_PATH, O_RDONLY, 0);
				if ( !IS_ERR_OR_NULL(fp) ){
					pr_info("filp_open success fp:%p\n", fp);
					inode = fp->f_dentry->d_inode;
					size_CalibOpt = inode->i_size;
					printk("%s: fp->f_dentry->d_inode->i_size=%d\n", __FUNCTION__, size_CalibOpt);
					if (size_CalibOpt > 0) {
						pCalibOpt_g = kmalloc(size_CalibOpt, GFP_KERNEL);
						old_fs = get_fs();
						set_fs(KERNEL_DS);
						if(fp->f_op != NULL && fp->f_op->read != NULL){
							byte_count= 0;
							printk("Start to read %s\n", CALIBOPT_FILE_WITH_PATH);
							byte_count = fp->f_op->read(fp, pCalibOpt_g, size_CalibOpt, &fp->f_pos);
							printk("iCatch: BIN file size= %d bytes\n", size_CalibOpt);
						}
						set_fs(old_fs);
					}
					filp_close(fp, NULL);
				} else {
					pr_err("iCatch \"%s\" open error\n", CALIBOPT_FILE_WITH_PATH);
				}

				
				fp = filp_open(IIIACALI_FILE_WITH_PATH, O_RDONLY, 0);
				if ( !IS_ERR_OR_NULL(fp) ){
					pr_info("filp_open success fp:%p\n", fp);
					inode = fp->f_dentry->d_inode;
					size_3acali = inode->i_size;
					printk("%s: fp->f_dentry->d_inode->i_size=%d\n", __FUNCTION__, size_3acali);
					if (size_3acali > 0) {
						p3acali_g = kmalloc(size_3acali, GFP_KERNEL);
						old_fs = get_fs();
						set_fs(KERNEL_DS);
						if(fp->f_op != NULL && fp->f_op->read != NULL){
							byte_count= 0;
							printk("Start to read %s\n", IIIACALI_FILE_WITH_PATH);
							byte_count = fp->f_op->read(fp, p3acali_g, size_3acali, &fp->f_pos);
							printk("iCatch: BIN file size= %d bytes\n", size_3acali);
						}
						set_fs(old_fs);
					}
					filp_close(fp, NULL);
				} else {
					pr_err("iCatch \"%s\" open error\n", IIIACALI_FILE_WITH_PATH);
				}

				
				fp = filp_open(LSC_FILE_WITH_PATH, O_RDONLY, 0);
				if ( !IS_ERR_OR_NULL(fp) ){
					pr_info("filp_open success fp:%p\n", fp);
					inode = fp->f_dentry->d_inode;
					size_Lsc = inode->i_size;
					printk("%s: fp->f_dentry->d_inode->i_size=%d\n", __FUNCTION__, size_Lsc);
					if (size_Lsc > 0) {
						pLsc_g = kmalloc(size_Lsc, GFP_KERNEL);
						old_fs = get_fs();
						set_fs(KERNEL_DS);
						if(fp->f_op != NULL && fp->f_op->read != NULL){
							byte_count= 0;
							printk("Start to read %s\n", LSC_FILE_WITH_PATH);
							byte_count = fp->f_op->read(fp, pLsc_g, size_Lsc, &fp->f_pos);
							printk("iCatch: BIN file size= %d bytes\n", size_Lsc);
						}
						set_fs(old_fs);
					}
					filp_close(fp, NULL);
				} else {
					pr_err("iCatch \"%s\" open error\n", LSC_FILE_WITH_PATH);
				}

				
				fp = filp_open(LSCDQ_FILE_WITH_PATH, O_RDONLY, 0);
				if ( !IS_ERR_OR_NULL(fp) ){
					pr_info("filp_open success fp:%p\n", fp);
					inode = fp->f_dentry->d_inode;
					size_Lscdq = inode->i_size;
					printk("%s: fp->f_dentry->d_inode->i_size=%d\n", __FUNCTION__, size_Lscdq);
					if (size_Lscdq > 0) {
						pLscdq_g = kmalloc(size_Lscdq, GFP_KERNEL);
						old_fs = get_fs();
						set_fs(KERNEL_DS);
						if(fp->f_op != NULL && fp->f_op->read != NULL){
							byte_count= 0;
							printk("Start to read %s\n", LSCDQ_FILE_WITH_PATH);
							byte_count = fp->f_op->read(fp, pLscdq_g, size_Lscdq, &fp->f_pos);
							printk("iCatch: BIN file size= %d bytes\n", size_Lscdq);
						}
						set_fs(old_fs);
					}
					filp_close(fp, NULL);
				} else {
					pr_err("iCatch \"%s\" open error\n", LSCDQ_FILE_WITH_PATH);
				}					

		
			} else if(PTR_ERR(fp) == -ENOENT) {
				pr_err("iCatch \"%s\" not found error\n", FW_BIN_FILE_WITH_PATH);
				fw_update_status = ICATCH_FW_UPDATE_FAILED;
			} else{
				pr_err("iCatch \"%s\" open error\n", FW_BIN_FILE_WITH_PATH);
				fw_update_status = ICATCH_FW_UPDATE_FAILED;
			}
	}
//FW_BSP--

	
	ret = mt9m114_s_power(sd, 1);
	if (ret) {
		dev_err(&client->dev, "mt9m114 power-up err\n");
		//goto fail_detect;
	}

	/* config & detect sensor */
	ret = mt9m114_detect(dev, client);
	if (ret) {
		dev_err(&client->dev, "mt9m114_detect err s_config.\n");
		//goto fail_detect;
	}

	ret = dev->platform_data->csi_cfg(sd, 1);
	if (ret)
		goto fail_csi_cfg;

	ret = mt9m114_s_power(sd, 0);
	if (ret) {
		dev_err(&client->dev, "mt9m114 power down err\n");
		return ret;
	}

	dev->run_mode = CI_MODE_PREVIEW;
	dev->last_run_mode = CI_MODE_PREVIEW;

	return 0;

fail_csi_cfg:
	dev->platform_data->csi_cfg(sd, 0);
//fail_detect:
	mt9m114_s_power(sd, 0);
	dev_err(&client->dev, "sensor power-gating failed\n");
	return ret;
}

static int mt9m114_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	struct mt9m114_control *ctrl = mt9m114_find_control(qc->id);

	if (ctrl == NULL)
		return -EINVAL;
	*qc = ctrl->qc;
	return 0;
}

static int mt9m114_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct mt9m114_control *octrl = mt9m114_find_control(ctrl->id);
	int ret;

	if (octrl == NULL)
		return -EINVAL;

	ret = octrl->query(sd, &ctrl->value);
	if (ret < 0)
		return ret;

	return 0;
}

static int mt9m114_s_ctrl_old(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct mt9m114_control *octrl = mt9m114_find_control(ctrl->id);
	int ret;

	if (!octrl || !octrl->tweak)
		return -EINVAL;

	ret = octrl->tweak(sd, ctrl->value);
	if (ret < 0)
		return ret;

	return 0;
}

static int mt9m114_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9m114_device *dev = to_mt9m114_sensor(sd);
	u32 i = 0, testval = 0x00;

	printk("%s enable = %d\n", __func__, enable);

	if (enable) {
		//wait AE ready
		for (i=0;i<20;i++) {
			sensor_read_reg(client, 0x72c3, &testval);
			//printk("testval=0x%X, i=%d ",testval,i);
			if (testval & 0x01) {
				printk("%s AE ready\n", __func__);
				break;
			}
			msleep(5);
		}
		if(i == 20) {
			pr_info("Wait AE timeout\n");
		}

		if(first_on==1){
			sensor_write_reg(client, 0x7121, 0x01); //Streaming on
			printk("%s Set stream on command\n", __func__);
		}else{
			first_on = 1;
			sensor_write_reg(client, 0x7121, 0x01); //Streaming on
			printk("%s First stream on command\n", __func__);
		}
	} else {
		sensor_write_reg(client, 0x7121, 0x00); //Streaming off

		dev->last_run_mode = dev->run_mode;
		printk("%s ++ last_run_mode:0x%x\n", __func__, dev->last_run_mode);
	}

	return 0;
}

static int
mt9m114_enum_framesizes(struct v4l2_subdev *sd, struct v4l2_frmsizeenum *fsize)
{
	unsigned int index = fsize->index;

	if (index >= N_RES)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = mt9m114_res[index].width;
	fsize->discrete.height = mt9m114_res[index].height;

	return 0;
}

static int mt9m114_enum_frameintervals(struct v4l2_subdev *sd,
				       struct v4l2_frmivalenum *fival)
{
	unsigned int index = fival->index;
	int i;

	if (index >= N_RES)
		return -EINVAL;

	/* find out the first equal or bigger size */
	for (i = 0; i < N_RES; i++) {
		if ((mt9m114_res[i].width >= fival->width) &&
		    (mt9m114_res[i].height >= fival->height))
			break;
	}
	if (i == N_RES)
		i--;

	index = i;

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = 1;
	fival->discrete.denominator = mt9m114_res[index].fps;

	return 0;
}

static int
mt9m114_g_chip_ident(struct v4l2_subdev *sd, struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_MT9M114, 0);
}

static int mt9m114_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= MAX_FMTS)
		return -EINVAL;
	code->code = V4L2_MBUS_FMT_SGRBG10_1X10;

	return 0;
}

static int mt9m114_enum_frame_size(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh,
	struct v4l2_subdev_frame_size_enum *fse)
{

	unsigned int index = fse->index;


	if (index >= N_RES)
		return -EINVAL;

	fse->min_width = mt9m114_res[index].width;
	fse->min_height = mt9m114_res[index].height;
	fse->max_width = mt9m114_res[index].width;
	fse->max_height = mt9m114_res[index].height;

	return 0;
}

static struct v4l2_mbus_framefmt *
__mt9m114_get_pad_format(struct mt9m114_device *sensor,
			 struct v4l2_subdev_fh *fh, unsigned int pad,
			 enum v4l2_subdev_format_whence which)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);

	if (pad != 0) {
		dev_err(&client->dev,  "%s err. pad %x\n", __func__, pad);
		return NULL;
	}

	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(fh, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &sensor->format;
	default:
		return NULL;
	}
}

static int
mt9m114_get_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct mt9m114_device *snr = to_mt9m114_sensor(sd);
	struct v4l2_mbus_framefmt *format =
			__mt9m114_get_pad_format(snr, fh, fmt->pad, fmt->which);

	if (format == NULL)
		return -EINVAL;
	fmt->format = *format;

	return 0;
}

static int
mt9m114_set_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct mt9m114_device *snr = to_mt9m114_sensor(sd);
	struct v4l2_mbus_framefmt *format =
			__mt9m114_get_pad_format(snr, fh, fmt->pad, fmt->which);

	if (format == NULL)
		return -EINVAL;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		snr->format = fmt->format;

	return 0;
}

static int mt9m114_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mt9m114_device *snr = container_of(
		ctrl->handler, struct mt9m114_device, ctrl_handler);

	snr->run_mode = ctrl->val;

	return 0;
}

static int mt9m114_g_skip_frames(struct v4l2_subdev *sd, u32 *frames)
{
	int index;
	struct mt9m114_device *snr = to_mt9m114_sensor(sd);

	if (frames == NULL)
		return -EINVAL;

	for (index = 0; index < N_RES; index++) {
		if (mt9m114_res[index].res == snr->res)
			break;
	}

	if (index >= N_RES)
		return -EINVAL;

	*frames = mt9m114_res[index].skip_frames;

	return 0;
}
static const struct v4l2_subdev_video_ops mt9m114_video_ops = {
	.try_mbus_fmt = mt9m114_try_mbus_fmt,
	.s_mbus_fmt = mt9m114_set_mbus_fmt,
	.g_mbus_fmt = mt9m114_get_mbus_fmt,
	.s_stream = mt9m114_s_stream,
	.enum_framesizes = mt9m114_enum_framesizes,
	.enum_frameintervals = mt9m114_enum_frameintervals,
};

static struct v4l2_subdev_sensor_ops mt9m114_sensor_ops = {
	.g_skip_frames	= mt9m114_g_skip_frames,
};

static const struct v4l2_subdev_core_ops mt9m114_core_ops = {
	.g_chip_ident = mt9m114_g_chip_ident,
	.queryctrl = mt9m114_queryctrl,
	.g_ctrl = mt9m114_g_ctrl,
	.s_ctrl = mt9m114_s_ctrl_old,
	.s_power = mt9m114_s_power,
};

static const struct v4l2_subdev_pad_ops mt9m114_pad_ops = {
	.enum_mbus_code = mt9m114_enum_mbus_code,
	.enum_frame_size = mt9m114_enum_frame_size,
	.get_fmt = mt9m114_get_pad_format,
	.set_fmt = mt9m114_set_pad_format,
};

static const struct v4l2_subdev_ops mt9m114_ops = {
	.core = &mt9m114_core_ops,
	.video = &mt9m114_video_ops,
	.pad = &mt9m114_pad_ops,
	.sensor = &mt9m114_sensor_ops,
};

static const struct media_entity_operations mt9m114_entity_ops;


static int mt9m114_remove(struct i2c_client *client)
{
	struct mt9m114_device *dev;
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	dev = container_of(sd, struct mt9m114_device, sd);
	dev->platform_data->csi_cfg(sd, 0);
	if (dev->platform_data->platform_deinit)
		dev->platform_data->platform_deinit();
	v4l2_ctrl_handler_free(&dev->ctrl_handler);
	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&dev->sd.entity);
	kfree(dev);
	return 0;
}

static const struct v4l2_ctrl_ops ctrl_ops = {
	.s_ctrl = mt9m114_s_ctrl,
};

static const char * const ctrl_run_mode_menu[] = {
	NULL,
	"Video",
	"Still capture",
	"Continuous capture",
	"Preview",
};

static const struct v4l2_ctrl_config ctrl_run_mode = {
	.ops = &ctrl_ops,
	.id = V4L2_CID_RUN_MODE,
	.name = "run mode",
	.type = V4L2_CTRL_TYPE_MENU,
	.min = 1,
	.def = 4,
	.max = 4,
	.qmenu = ctrl_run_mode_menu,
};

static int mt9m114_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	struct mt9m114_device *dev;
	int ret;

	/* Setup sensor configuration structure */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&client->dev, "out of memory\n");
		return -ENOMEM;
	}

//FW_BSP++
	create_i7002a_proc_file(); //ASUS_BSP +++, add for ISP firmware update
//FW_BSP--	

	//Add for ATD read camera status+++
	dev->sensor_i2c_attribute.attrs = mt9m114_attributes;

	// Register sysfs hooks 
	ret = sysfs_create_group(&client->dev.kobj, &dev->sensor_i2c_attribute);
	if (ret) {
		dev_err(&client->dev, "Not able to create the sysfs\n");
		return ret;
	}
	//Add for ATD read camera status---

	v4l2_i2c_subdev_init(&dev->sd, client, &mt9m114_ops);
	if (client->dev.platform_data) {
		ret = mt9m114_s_config(&dev->sd, client->irq,
				       client->dev.platform_data);
		if (ret) {
			v4l2_device_unregister_subdev(&dev->sd);
			kfree(dev);
			return ret;
		}
	}

	/*TODO add format code here*/
	dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dev->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = v4l2_ctrl_handler_init(&dev->ctrl_handler, 1);
	if (ret) {
		mt9m114_remove(client);
		return ret;
	}

	dev->run_mode = v4l2_ctrl_new_custom(&dev->ctrl_handler,
					     &ctrl_run_mode, NULL);

	if (dev->ctrl_handler.error) {
		mt9m114_remove(client);
		return dev->ctrl_handler.error;
	}

	ret = media_entity_init(&dev->sd.entity, 1, &dev->pad, 0);
	if (ret) {
		mt9m114_remove(client);
		return ret;
	}

	/* set res index to be invalid */
	dev->res = -1;

	return 0;
}


MODULE_DEVICE_TABLE(i2c, mt9m114_id);

static struct i2c_driver mt9m114_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "mt9m114"
	},
	.probe = mt9m114_probe,
	.remove = __exit_p(mt9m114_remove),
	.id_table = mt9m114_id,
};

static __init int init_mt9m114(void)
{
	return i2c_add_driver(&mt9m114_driver);
}

static __exit void exit_mt9m114(void)
{
	i2c_del_driver(&mt9m114_driver);
}

module_init(init_mt9m114);
module_exit(exit_mt9m114);

MODULE_AUTHOR("ASUS SW3");
MODULE_LICENSE("GPL");

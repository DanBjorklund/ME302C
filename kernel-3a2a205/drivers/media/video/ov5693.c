/*
 * Support for ov5693 Camera Sensor.
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
#include <linux/switch.h>		//Add for ISP firmware version in setting
//FW_BSP--

//#include "i7002a.h"
#include "ov5693.h"

#define to_ov5693_sensor(sd) container_of(sd, struct ov5693_device, sd)

//Add for ATD read camera status+++
int ATD_ov5693_status = 0;  //Add for ATD read camera status

//Add build version -> user:3, userdebug:2, eng:1
extern int build_version;

struct v4l2_subdev *fw_sd;
static int first_on = 0;
static bool g_enable_roi_debug = false; //Add ROI display for debugging
u16 g_is_calibration = 0; //Add for calibration
static int iso_setting = 0;
int sensorid = 0, retvalue_h = 0, retvalue_l = 0;


static ssize_t ov5693_show_status(struct device *dev,struct device_attribute *attr,char *buf)
{
	printk("%s: get ov5693 status (%d) !!\n", __func__, ATD_ov5693_status);
   	//Check sensor connect status, just do it  in begining for ATD camera status 

	return sprintf(buf,"%d\n", ATD_ov5693_status);
}

static DEVICE_ATTR(ov5693_status, S_IRUGO,ov5693_show_status,NULL);

static struct attribute *ov5693_attributes[] = {
	&dev_attr_ov5693_status.attr,
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
#define ISP_SDEV_NAME			"camera"  //Add for ISP firmware version in setting

#define SENSOR_WAIT_MS          0 /* special number to indicate this is wait time require */
#define SENSOR_TABLE_END        1 /* special number to indicate this is end of table */
#define SENSOR_BYTE_WRITE       2
#define SENSOR_WORD_WRITE       3
#define SENSOR_MASK_BYTE_WRITE  4
#define SENSOR_MASK_WORD_WRITE  5
#define SEQ_WRITE_START         6
#define SEQ_WRITE_END           7

#define SENSOR_MAX_RETRIES      3 /* max counter for retry I2C access */

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
static char IIIACALI_FILE_WITH_PATH[] = "/factory/3ACALI.BIN";
static char LSC_FILE_WITH_PATH[] = "/factory/LSC.BIN";
static char LSCDQ_FILE_WITH_PATH[] = "/factory/LSC_DQ.BIN";
static char  FW_BIN_FILE_WITH_PATH_READ[] = "/data/BOOT-READ.BIN";
bool FW_BIN_FILE=false, GET_SENSOR_ID=false, FIRST_BOOTING=true;
u8 *pIspFW_g=NULL, *pCalibOpt_g=NULL, *p3acali_g=NULL, *pLsc_g=NULL, *pLscdq_g=NULL;
int SPI_ret=0;
UINT32 bootbin_size_g=0;


// i7002a firmware +++
#define SPI_CMD_BYTE_READ 	0x03
#define SPI_CMD_RD_ID 		0x9F
#define SPI_CMD_WRT_EN		0x06
#define SPI_CMD_BYTE_PROG 	0x02
#define SPI_CMD_RD_STS		0x05
#define SPI_CMD_BYTE_PROG_AAI	0xAD
#define SPI_CMD_WRT_STS_EN	0x50
#define SPI_CMD_WRT_STS 	0x01
#define SPI_CMD_WRT_DIS 	0x04
#define SPI_CMD_ERASE_ALL	0xC7
#define	SPI_CMD_SECTOR_ERASE		0x20
#define	SPI_CMD_32KB_BLOCK_ERASE	0x52
#define	SPI_CMD_64KB_BLOCK_ERASE	0xD8


#define update_write_byte 64

/* iCatch Camera Firmware Header
 * It locates on the end of the bin file.
 * Total: 32 bytes.
 * byte[0] ~ byte[7]: 0xFF's
 * byte[8] ~ byte[11]: Compensation for Overall Checksum
 * byte[12] ~ byte[15]: Overall Checksum

 * byte[16] ~ byte[20]: 0xFF's
 * byte[21]: Front Format
 * byte[22]: Rear Lane#
 * byte[23]: Front Lane#
 * byte[24] ~ byte[25]: Rear Sensor ID
 * byte[26] ~ byte[27]: Front sensor ID
 * byte[28] ~ byte[31]: FW Version
 */

#define BIN_FILE_HEADER_SIZE 32
#define NEED_UPDATE				0x0
#define UPDATE_UNNECESSARY		0x1

static int fw_page_count = -1;
static int total_page_count = -1;

/* Used for calculating the iCatch fw update progress */
static int g_page_count = -1;
static int g_total_page_count = -1;


//struct switch_dev   i7002a_sdev; //Add for ISP firmware version in setting
u32 version_num_in_bin = 0xffffffff;
static unsigned int version_num_in_isp = 0xffffff;

static int ov5693_s_power(struct v4l2_subdev *sd, int power);

extern int spca700xa_SPI_write(UINT8 *, UINT32);
extern int spca700xa_SPI_read(UINT8 *, UINT32);

struct switch_dev 	ISP_sdev;	//Add for ISP firmware version in setting
static int spi_init_extra_parameter();

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

	msg.addr = client->addr;
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

	msg.addr = client->addr;
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

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (addr >> 8);;
	data[1] = (u8) (addr & 0xff);

	msg[1].addr = client->addr;
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

//FW_BSP++
//i7002a firmware+++
int I2C_SPIInit(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
	//  I2CDataWrite(0x0026,0xc0);
	//  I2CDataWrite(0x4051,0x01); /* spien */
	//  I2CDataWrite(0x40e1,0x00); /* spi mode */
	//  I2CDataWrite(0x40e0,0x11); /* spi freq */
	struct sensor_reg SPI_init_seq[] = {
		{0x0026, 0xc0},
		{0x4051, 0x01},
		{0x40e1, 0x00},
		{0x40e0, 0x11},
		{SENSOR_TABLE_END, 0x0000}
	};

	ret = sensor_write_table(client, SPI_init_seq);
	if(ret) {
		printk("%s: init fail. ret= 0x%x\n", __FUNCTION__, ret);
	}
	return ret;
}

/*-------------------------------------------------------------------------
 *  File Name : I2C_SPIInit
 *  return SUCCESS: normal
           FAIL: if wait spi flash time out
 *------------------------------------------------------------------------*/
u32 I2C_SPIFlashPortWait(void)
{
    //u32 cnt = WAIT_COUNT;
#if 0
    while(I2CDataRead(0x40e6) != 0x00){
        cnt--;
        if(cnt == 0x00)
        {
            printf("serial flash port wait time out!!\n");
            return FAIL;
        }
    }
#endif
    return 0;
}

/*-------------------------------------------------------------------------
 *  File Name : I2C_SPIFlashPortWrite
 *  return SUCCESS: normal
           FAIL:    if wait spi flash time out
 *------------------------------------------------------------------------*/
u32 I2C_SPIFlashPortWrite(struct v4l2_subdev *sd, u32 wData)
{
    //hsI2CDataWrite(0x40e3,(u8)wData);
    struct i2c_client *client = v4l2_get_subdevdata(sd);
	
    sensor_write_reg(client, 0x40e3,(u8)wData);
    return I2C_SPIFlashPortWait();
}


u32 I2C_SPIFlashPortRead(struct v4l2_subdev *sd)
{
	u16 ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	// ret = hsI2CDataRead(0x40e4);
      sensor_read_reg(client, 0x40e4, &ret);
	/* polling SPI state machine ready */
#if 0
    if (I2C_SPIFlashPortWait() != SUCCESS) {
        return 0;
    }
#endif
	//ret = hsI2CDataRead(0x40e5);
      sensor_read_reg(client, 0x40e5, &ret);

    return (u32)ret;
}

u32 I2C_SPIFlashRead(
	struct v4l2_subdev *sd,
	u32 addr,
	u32 pages,
	u8 *pbuf
)
{
	u32 err = 0;
	u32 i, size=0;
	u32 pageSize = 0x100;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	addr = addr * pageSize;
	size = pages*pageSize;

	// I2CDataWrite(0x40e7,0x00);
	sensor_write_reg(client, 0x40e7, 0x00);
	// I2C_SPIFlashPortWrite(SPI_CMD_BYTE_READ);               /* Write one byte command*/
	sensor_write_reg(client, 0x40e3, SPI_CMD_BYTE_READ);
	// I2C_SPIFlashPortWrite((u8)(addr >> 16));               /* Send 3 bytes address*/
	// I2C_SPIFlashPortWrite((u8)(addr >> 8));
	// I2C_SPIFlashPortWrite((u8)(addr));
	sensor_write_reg(client, 0x40e3, (u8)(addr >> 16));
	sensor_write_reg(client, 0x40e3, (u8)(addr >> 8));
	sensor_write_reg(client, 0x40e3, (u8)(addr));

	for (i = 0; i < size ; i++) {
		*pbuf = I2C_SPIFlashPortRead(sd);
		if((i%256)==0)
			printk("%s: page count: 0x%x\n", __FUNCTION__, (i/256));
		pbuf ++;
	}

	sensor_write_reg(client, 0x40e7, 0x01);

	return err;
}

u32 I2C_SPIFlashReadId(struct v4l2_subdev *sd)
{
	u8 id[3];
	u32 ID;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	id[0] = 0;
	id[1] = 0;
	id[2] = 0;

	//hsI2CDataWrite(0x40e7,0x00);
	sensor_write_reg(client, 0x40e7,0x00);

	//err = I2C_SPIFlashPortWrite(SPI_CMD_RD_ID); /*read ID command*/
	sensor_write_reg(client, 0x40e3,SPI_CMD_RD_ID);

#if 0
	if (err != SUCCESS) {
		printf("Get serial flash ID failed\n");
		return 0;
	}
#endif

	id[0] = I2C_SPIFlashPortRead(sd);    /* Manufacturer's  ID */
	id[1] = I2C_SPIFlashPortRead(sd);    /* Device ID          */
	id[2] = I2C_SPIFlashPortRead(sd);    /* Manufacturer's  ID */

	//hsI2CDataWrite(0x40e7,0x01);
	sensor_write_reg(client, 0x40e7,0x01);

	printk("ID %2x %2x %2x\n", id[0], id[1], id[2]);

	ID = ((u32)id[0] << 16) | ((u32)id[1] << 8) | \
    ((u32)id[2] << 0);

	return ID;
}

static const u32 stSpiIdInfo[29] =
{
	/*EON*/
	0x001C3117,
	0x001C2016,
	0x001C3116,
	0x001C3115,
	0x001C3114,
	0x001C3113,
	/*Spansion*/
	0x00012018,
	0x00010216,
	0x00010215,
	0x00010214,
	/*ST*/
	0x00202018,
	0x00202017,
	0x00202016,
	0x00202015,
	0x00202014,
	/*MXIC*/
	0x00C22018,
	0x00C22017,
	0x00C22016,
	0x00C25e16,
	0x00C22015,
	0x00C22014,
	0x00C22013,
	/*Winbond*/
	0x00EF3017,
	0x00EF3016,
	0x00EF3015,
	0x00EF3014,
	0x00EF3013,
	0x00EF5013,
	/*Fail*/
	0x00000000,
};

static const u32 sstSpiIdInfo[6] =
{
	/*ESMT*/
	0x008C4016,
	/*SST*/
	0x00BF254A,
	0x00BF2541,
	0x00BF258E,
	0x00BF258D,
	/*Fail*/
	0x00000000,
};

u32
BB_SerialFlashTypeCheck(
	u32 id
)
{
	u32 i=0;
	u32 fullID = 1;
	u32 shift = 0, tblId, type = 0;

	/* check whether SST type serial flash */
	while( 1 ){
		tblId = sstSpiIdInfo[i] >> shift;
		if( id == tblId ) {
			printk("SST type serial flash\n");
			type = 2;
			break;
		}
		if( id == 0x00FFFFFF || id == 0x00000000) {
			return 0;
		}
		if( sstSpiIdInfo[i] == 0x00000000 ) {
			if( fullID ){
				fullID = 0;/* sarch partial ID */
				i = 0;
				shift = 16;
				id = id >> shift;
				continue;
			}
			type = 3;
			break;
		}
		i ++;
	}
	if( type == 2 )
		return type;

	i = 0;
	/* check whether ST type serial flash */
	while( 1 ){
		tblId = stSpiIdInfo[i] >> shift;
		if( id == tblId ) {
			printk("ST Type serial flash\n");
			type = 1;
			break;
		}
		if( id == 0x00FFFFFF || id == 0x00000000) {
			return 0;
		}
		if( stSpiIdInfo[i] == 0x00000000 ) {
			if( fullID ){
				fullID = 0;/* sarch partial ID */
				i = 0;
				shift = 16;
				id = id >> shift;
				continue;
			}
			type = 3;
			break;
		}
		i ++;
	}

	return type;
}

int I2C_SPIFlashWrEnable(struct v4l2_subdev *sd)
{
	int ret = 0;
	//hsI2CDataWrite(0x40e7,0x00);
	//I2C_SPIFlashPortWrite(SPI_CMD_WRT_EN);
	//hsI2CDataWrite(0x40e7,0x01);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct sensor_reg I2C_SPIFlashWrEnable_seq[] = {
		{0x40e7, 0x00},
		{0x40e3, SPI_CMD_WRT_EN},
		{0x40e7, 0x01},
		{SENSOR_TABLE_END, 0x0000}
	};

	ret = sensor_write_table(client, I2C_SPIFlashWrEnable_seq);

	if(ret) {
		printk("%s: fail. ret= 0x%x\n", __FUNCTION__, ret);
	}
	return ret;
}

u32 I2C_SPIStsRegRead(struct v4l2_subdev *sd)
{
	u32 ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	//hsI2CDataWrite(0x40e7,0x00);
	sensor_write_reg(client, 0x40e7,0x00);
	//I2C_SPIFlashPortWrite(SPI_CMD_RD_STS);
	sensor_write_reg(client, 0x40e3,SPI_CMD_RD_STS);
	ret = I2C_SPIFlashPortRead(sd);

	// hsI2CDataWrite(0x40e7,0x01);
	sensor_write_reg(client, 0x40e7,0x01);

	return ret;
}

void I2C_SPITimeOutWait(struct v4l2_subdev *sd, u32 poll, u32 *ptimeOut)
{
    /* MAX_TIME for SECTOR/BLOCK ERASE is 25ms */
    u32 sts;
    u32 time = 0;
    while (1) {
        sts = I2C_SPIStsRegRead(sd);
        if (!(sts & poll))	/* sfStatusRead() > 4.8us */ {
            break;
        }
        time ++;
        if( *ptimeOut < time ) {
            printk("iCatch: TimeOut %d, sts=0x%x, poll=0x%x\n",time,sts,poll);
            break;
        }
    }
}

int I2C_SPIStChipErase(struct v4l2_subdev *sd)
{
	u32 timeout;
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	printk("iCatch: ST Chip Erasing...\n");

	//hsI2CDataWrite(0x40e7,0x00);
	sensor_write_reg(client, 0x40e7,0x00);
	//I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS);
	sensor_write_reg(client, 0x40e3,SPI_CMD_WRT_STS);
	//I2C_SPIFlashPortWrite(0x02);
	sensor_write_reg(client, 0x40e3,0x02);
	//hsI2CDataWrite(0x40e7,0x01);
	sensor_write_reg(client, 0x40e7,0x01);

	ret = I2C_SPIFlashWrEnable(sd);
	if (ret) {
		printk("iCatch: ST Chip Erase fail, ret= 0x%x\n", ret);
		return ret;
	}

	//hsI2CDataWrite(0x40e7,0x00);
	sensor_write_reg(client, 0x40e7,0x00);
	//I2C_SPIFlashPortWrite(SPI_CMD_ERASE_ALL);
	sensor_write_reg(client, 0x40e3,SPI_CMD_ERASE_ALL);
	//hsI2CDataWrite(0x40e7,0x01);
	sensor_write_reg(client, 0x40e7,0x01);

	timeout = 0xffffffff;
	I2C_SPITimeOutWait(sd, 0x01, &timeout);
#if 0
	ros_thread_sleep(1);
#endif
	//hsI2CDataWrite(0x40e7,0x01);
	sensor_write_reg(client, 0x40e7,0x01);
	printk("iCatch: ST Chip Erased\n");
	return 0;
}

int I2C_SPISstChipErase(struct v4l2_subdev *sd)
{
	u32 timeout;
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	printk("iCatch: SST Chip Erasing...\n");

	ret = I2C_SPIFlashWrEnable(sd);
	if (ret) {
		printk("iCatch: SST Chip Erase fail, ret= 0x%x\n", ret);
		return ret;
	}

	//hsI2CDataWrite(0x40e7,0x00);
	//I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS_EN); /*Write Status register command*/
	//hsI2CDataWrite(0x40e7,0x01);
	sensor_write_reg(client, 0x40e7,0x00);
	sensor_write_reg(client, 0x40e3,SPI_CMD_WRT_STS_EN);
	sensor_write_reg(client, 0x40e7,0x01);

	//hsI2CDataWrite(0x40e7,0x00);
	//I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS);
	//I2C_SPIFlashPortWrite(0x02);
	//hsI2CDataWrite(0x40e7,0x01);
	sensor_write_reg(client, 0x40e7,0x00);
	sensor_write_reg(client, 0x40e3,SPI_CMD_WRT_STS);
	sensor_write_reg(client, 0x40e3,0x02);
	sensor_write_reg(client, 0x40e7,0x01);

	I2C_SPIFlashWrEnable(sd);

	//hsI2CDataWrite(0x40e7,0x00);
	//I2C_SPIFlashPortWrite(SPI_CMD_ERASE_ALL);
	//hsI2CDataWrite(0x40e7,0x01);
	sensor_write_reg(client, 0x40e7,0x00);
	sensor_write_reg(client, 0x40e3,SPI_CMD_ERASE_ALL);
	sensor_write_reg(client, 0x40e7,0x01);

	timeout = 0xffffffff;
	I2C_SPITimeOutWait(sd, 0x01, &timeout);
	//msleep(500);
	printk("iCatch: SST Chip Erased\n");
	return 0;
}

void writeUpdateProgresstoFile(int page_left, int total_page_num)
{
	struct file *fp_progress = NULL;
	mm_segment_t old_fs;
	loff_t offset = 0;
	char str_progress[4];
	int percentage = 0;

	percentage = 100 * (total_page_num - page_left + 1)/total_page_num;

	if(page_left % 32 == 1){
		printk("%s: page:0x%x; percentage= %d;\n", __FUNCTION__, page_left, percentage);
		fp_progress = filp_open("/data/isp_fw_update_progress", O_RDWR | O_CREAT, S_IRUGO | S_IWUGO);
		if ( IS_ERR_OR_NULL(fp_progress) ){
			printk("%s: open %s fail\n", __FUNCTION__, "/data/isp_fw_update_progress");
		} else {
			old_fs = get_fs();
			set_fs(KERNEL_DS);
			offset = 0;
			if (fp_progress->f_op != NULL && fp_progress->f_op->write != NULL){
				sprintf(str_progress, "%d\n", percentage);
				fp_progress->f_op->write(fp_progress,
					str_progress,
					strlen(str_progress),
					&offset);
			}else
				pr_err("%s: f_op might be null\n", __FUNCTION__);
			set_fs(old_fs);
			filp_close(fp_progress, NULL);
		}
	}
}


u32 seqI2CDataWrite(struct i2c_client *client, u32 a_InputAddr, u8* pBuf)
{
	int i, err;
	struct i2c_msg msg;
	unsigned char data[update_write_byte+2];
	int retry = 0;
	if (!client->adapter)
		return -ENODEV;

	data[0]=  (a_InputAddr & 0xff00)>>8; 
	data[1]=  a_InputAddr & 0x00ff; 
	for(i=0; i < update_write_byte; i++) {
		data[i+2]= *(pBuf+i) & 0x00ff;
	}
/*
	data[2]=  *(pBuf) & 0x00ff; 
	data[3]=  *(pBuf+1) & 0x00ff; 
	data[4]=  *(pBuf+2) & 0x00ff; 
	data[5]=  *(pBuf+3) & 0x00ff; 
*/
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = update_write_byte+2;
	msg.buf = data;
	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		pr_err("%s : i2c transfer failed, msg.addr %x, err= 0x%x\n", __FUNCTION__, msg.addr, err);
//		msleep(3);
	} while (retry <= SENSOR_MAX_RETRIES);

	if(err == 0) {
		printk("%s(%d): i2c_transfer error, but return 0!?\n", __FUNCTION__, __LINE__);
		err = -EINVAL;
	}

	return err;
}


u32 I2C_7002DmemWr(
	struct v4l2_subdev *sd,
	u32 bankNum,
	u32 byteNum,
	u8* pbuf
)
{
	u32 i, bank;
    int rc=0;	
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	bank = 0x40+bankNum;
	//I2CDataWrite(0x10A6,bank);
	sensor_write_reg(client, 0x10A6,bank);

	for(i=0;i<byteNum;i+=update_write_byte)
	{
		rc=seqI2CDataWrite(client, (0x1800+i),(pbuf+i)); /* sequentially write DMEM */
        if(rc < 0){
        	pr_err("%s sensor_write_reg_bytes fail rc=%d\n",__func__,rc);
        	return rc;
        }		
	}

	bank = 0x40 + ((bankNum+1)%2);
	//hsI2CDataWrite(0x10A6,bank);
	sensor_write_reg(client, 0x10A6,bank);

	return 0;
}

u32 I2C_SPIFlashWrite_DMA(
	struct v4l2_subdev *sd,
	u32 addr,
	u32 pages,
	u8 *pbuf
)
{
    u32 i, err = 0;
    u32 pageSize = 0x100, size;
    u32 rsvSec1, rsvSec2;
    u32 dmemBank = 0;
    u32 chk1=0;
    u16 temp, chk2=0;
	int rc=0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

    rsvSec1 = pages*pageSize - 0x7000;
    rsvSec2 = pages*pageSize - 0x1000;
    addr = addr * pageSize;

    /* Set DMA bytecnt as 256-1 */
    //I2CDataWrite(0x4170,0xff);
    //I2CDataWrite(0x4171,0x00);    
   //I2CDataWrite(0x4172,0x00);    
    sensor_write_reg(client, 0x4170,0xff);
    sensor_write_reg(client, 0x4171,0x00);
    sensor_write_reg(client, 0x4172,0x00);

    /* Set DMA bank & DMA start address */
    //I2CDataWrite(0x1084,0x01);
    //I2CDataWrite(0x1080,0x00);    
    //I2CDataWrite(0x1081,0x00);    
    //I2CDataWrite(0x1082,0x00);    
    sensor_write_reg(client, 0x1084,0x01);
    sensor_write_reg(client, 0x1080,0x00);
    sensor_write_reg(client, 0x1081,0x00);
    sensor_write_reg(client, 0x1082,0x00);

    /* enable DMA checksum and reset checksum */
    //I2CDataWrite(0x4280,0x01);
    //I2CDataWrite(0x4284,0x00);    
    //I2CDataWrite(0x4285,0x00);    
    //I2CDataWrite(0x4164,0x00);    
    sensor_write_reg(client, 0x4280,0x01);
    sensor_write_reg(client, 0x4284,0x00);
    sensor_write_reg(client, 0x4285,0x00);
    sensor_write_reg(client, 0x4164,0x00);

    size = pages * pageSize;
    for(i=0;i<size;i++)
    {
		if((i>=rsvSec2) || (i <rsvSec1))
		{
			chk1 += *(pbuf+i);
		}
		
		if(chk1>=0x10000)
		{
			chk1 -= 0x10000;
		}
	}

	g_total_page_count = (int)pages;

    while( pages ) {
		g_page_count = (int)pages;
		writeUpdateProgresstoFile(g_page_count, g_total_page_count);   
	
		if((pages%0x40)==0)
		{
			//printk("page:0x%x",pages);
		}
		
		if((addr>=rsvSec1) && (addr <rsvSec2))
		{
			addr += 0x1000;
			pbuf += 0x1000;
			pages -= 0x10;
			continue;
		}
		
		if((pages==1))
		{
			for (i = 0; i < pageSize ; i++) {
				printk("%2x ",*(pbuf+i));
				if((i%0x10)==0x0f) printk("\n");
			}
		}

    	dmemBank = pages % 2;
    	//I2CDataWrite(0x1081,dmemBank*0x20);
    	//I2CDataWrite(0x1084,(1<<dmemBank));    	
		sensor_write_reg(client, 0x1081,dmemBank*0x20);
		sensor_write_reg(client, 0x1084,(1<<dmemBank));
    	rc=I2C_7002DmemWr(sd,dmemBank,pageSize,pbuf);
		
		if(rc<0){
            pr_err("%s fail\n",__func__);
            return rc;
		}
		
     	I2C_SPIFlashWrEnable(sd);
     	//I2CDataWrite(0x40e7,0x00);
		sensor_write_reg(client, 0x40e7,0x00);
     	I2C_SPIFlashPortWrite(sd, SPI_CMD_BYTE_PROG);               /* Write one byte command*/
     	I2C_SPIFlashPortWrite(sd, (u8)(addr >> 16));               /* Send 3 bytes address*/
     	I2C_SPIFlashPortWrite(sd, (u8)(addr >> 8));
     	I2C_SPIFlashPortWrite(sd, (u8)(addr));

    	//I2CDataWrite(0x4160,0x01);
		sensor_write_reg(client, 0x4160,0x01);
    	//tmrUsWait(100);/* wait for DMA done */
		udelay(100);
    	//I2CDataWrite(0x40e7,0x01);
		sensor_write_reg(client, 0x40e7,0x01);
    	pbuf += pageSize;
    	addr += pageSize;
    	pages --;
    }
	
    //tmrUsWait(500);/* wait for DMA done */
    udelay(500);

    //temp = hsI2CDataRead(0x4285);
    sensor_read_reg(client, 0x4285, &temp);
    //chk2 = hsI2CDataRead(0x4284);
    sensor_read_reg(client, 0x4284, &chk2);
    chk2 = chk2 | (temp<<8);
    printk("checksum: 0x%x 0x%x\n",chk1,chk2);

    return err;
}

void I2C_SPISstStatusWrite(struct v4l2_subdev *sd, u8 dat)
{
	u32 timeout, poll;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	I2C_SPIFlashWrEnable(sd);

	//hsI2CDataWrite(0x40e7,0x00);
	//I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS_EN);
	//hsI2CDataWrite(0x40e7,0x01);
	sensor_write_reg(client, 0x40e7,0x00);
	sensor_write_reg(client, 0x40e3,SPI_CMD_WRT_STS_EN);
	sensor_write_reg(client, 0x40e7,0x01);

	// hsI2CDataWrite(0x40e7,0x00);
	//I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS);
	//I2C_SPIFlashPortWrite(dat);
	//hsI2CDataWrite(0x40e7,0x01);
	sensor_write_reg(client, 0x40e7,0x00);
	sensor_write_reg(client, 0x40e3,SPI_CMD_WRT_STS);
	printk("%s: dat=%d\n", __FUNCTION__, dat);
	sensor_write_reg(client, 0x40e3,dat);
	printk("%s: dat=%d; Done.\n", __FUNCTION__, dat);
	sensor_write_reg(client, 0x40e7,0x01);

	poll = 0x01;
#if 0
	if( spiDev.bus != SPI_1BIT_MODE ) {/* 1 bit mode */
		poll = 0x80;
	} else {
		poll = 0x01;
	}
#endif
    timeout = 100000;
    I2C_SPITimeOutWait(sd, poll, &timeout);
    //msleep(500);
    return;
}

u32 I2C_SPISstFlashWrite(
	struct v4l2_subdev *sd,
	u32 addr,
	u32 pages,
	u8 *pbuf
)
{
	u32 i, err = 0;
	u32 pageSize = 0x100;
	u32 timeout = 100000;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	addr = addr * pageSize;

	printk("iCatch: SST type writing...\n");
	I2C_SPISstStatusWrite(sd, 0x40);

	total_page_count = (int)pages;

	while( pages ) {
		fw_page_count = (int)pages;
		writeUpdateProgresstoFile(fw_page_count, total_page_count);

		I2C_SPIFlashWrEnable(sd);
		//hsI2CDataWrite(0x40e7,0x00);
		sensor_write_reg(client, 0x40e7,0x00);
		//I2C_SPIFlashPortWrite(SPI_CMD_BYTE_PROG_AAI); /* Write one byte command*/
		sensor_write_reg(client, 0x40e3,SPI_CMD_BYTE_PROG_AAI);
		//I2C_SPIFlashPortWrite((UINT8)(addr >> 16)); /* Send 3 bytes address*/
		sensor_write_reg(client, 0x40e3,(u8)(addr >> 16));
		//I2C_SPIFlashPortWrite((UINT8)(addr >> 8));
		sensor_write_reg(client, 0x40e3,(u8)(addr >> 8));
		//I2C_SPIFlashPortWrite((UINT8)(addr));
		sensor_write_reg(client, 0x40e3,(u8)(addr));
		//I2C_SPIFlashPortWrite(*pbuf);
		sensor_write_reg(client, 0x40e3,(u8)(*pbuf));
		pbuf++;
		//I2C_SPIFlashPortWrite(*pbuf);
		sensor_write_reg(client, 0x40e3,(u8)(*pbuf));
		pbuf++;
		//hsI2CDataWrite(0x40e7,0x01);
		sensor_write_reg(client, 0x40e7,0x01);
		timeout = 100000;
		I2C_SPITimeOutWait(sd, 0x01,&timeout);

		for (i = 2; i < pageSize ; i = i+2) {
			//hsI2CDataWrite(0x40e7,0x00);
			sensor_write_reg(client, 0x40e7,0x00);
			//I2C_SPIFlashPortWrite(SPI_CMD_BYTE_PROG_AAI);
			sensor_write_reg(client, 0x40e3,SPI_CMD_BYTE_PROG_AAI);
			// I2C_SPIFlashPortWrite(*pbuf);
			sensor_write_reg(client, 0x40e3,(u8)(*pbuf));
			pbuf++;
			// I2C_SPIFlashPortWrite(*pbuf);
			sensor_write_reg(client, 0x40e3,(u8)(*pbuf));
			pbuf++;
			// hsI2CDataWrite(0x40e7,0x01);
			sensor_write_reg(client, 0x40e7,0x01);
			timeout = 100000;
			I2C_SPITimeOutWait(sd, 0x01,&timeout);
		}

		// hsI2CDataWrite(0x40e7,0x00);
		sensor_write_reg(client, 0x40e7,0x00);
		//I2C_SPIFlashPortWrite(SPI_CMD_WRT_DIS);
		sensor_write_reg(client, 0x40e3,SPI_CMD_WRT_DIS);
		//hsI2CDataWrite(0x40e7,0x01);
		sensor_write_reg(client, 0x40e7,0x01);

		addr += pageSize;
		pages --;

		//hsI2CDataWrite(0x40e7,0x00);
		sensor_write_reg(client, 0x40e7,0x00);
		//I2C_SPIFlashPortWrite(SPI_CMD_WRT_DIS);
		sensor_write_reg(client, 0x40e3,SPI_CMD_WRT_DIS);
		//hsI2CDataWrite(0x40e7,0x01);
		sensor_write_reg(client, 0x40e7,0x01);
	}
	printk("iCatch: SST type writing Done.\n");
	return err;
}

/* get_one_page_from_i7002a():
 *   Dump the ISP page whose index is "which_page" to "pagebuf".
 *   mclk, power & rst are requisite for getting correct page data.
 */
void get_one_page_from_i7002a(struct v4l2_subdev *sd, int which_page, u8* pagebuf)
{
	int i = 0;
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	//I2CDataWrite(0x70c4,0x00);
	//I2CDataWrite(0x70c5,0x00);
	sensor_write_reg(client, 0x70c4,0x00);
	sensor_write_reg(client, 0x70c5,0x00);
	printk("%s: I2C_SPIInit", __FUNCTION__);
	ret = I2C_SPIInit(sd);
	if (ret) {
		printk("%s: get nothing. ret= %d", __FUNCTION__, ret);
		return;
	}
	printk("%s: I2C_SPIFlashReadId", __FUNCTION__);
	I2C_SPIFlashReadId(sd);
	printk("%s: I2C_SPIFlashRead", __FUNCTION__);
	I2C_SPIFlashRead(sd, which_page, 1, pagebuf);

#if 1 // dump to kmsg ?
	printk("page#%d:\n", which_page);
	for(i=0; i < 0x100; i++) {
		if(i%16 == 0)
			printk("[%04x]", i);
		printk("%02X ",  pagebuf[i]);
		if(i%16 == 15)
			printk("\n");
	}
#endif
}

unsigned int get_fw_version_in_isp(struct v4l2_subdev *sd)
{
	u8 tmp_page[0x100];
	unsigned int vn = 0xABCDEF;
	int i = 0;
	int retry = 3;
	bool b_ok;

	for (i = 0; i < retry; i++) {
		int j =0;
		b_ok = true;

		/* The fw veriosn is in the page with the index, 4095.*/
		get_one_page_from_i7002a(sd, 4095, tmp_page);

		/* The header format looks like:
		 * FF FF FF FF FF FF FF FF XX XX XX XX XX XX XX
		 * FF FF FF FF FF XX XX XX XX XX XX XX XX XX XX
		 */
		for (j = 0; j < 8; j++) {
			if (tmp_page[0x100 - BIN_FILE_HEADER_SIZE + j] != 0xFF) {
				printk("%s: tmp_page[0x%X]= %02X\n", __FUNCTION__,
					0x100 - BIN_FILE_HEADER_SIZE + j,
					tmp_page[0x100 - BIN_FILE_HEADER_SIZE + j]);
				b_ok = false;
				break;
			}
		}
		if (b_ok == true)
			break;
		else {
			printk("%s: wrong page data? Try again (%d).\n", __FUNCTION__, i);
			msleep(10);
		}
	}

	if (b_ok == true)
		vn = (tmp_page[0xFF - 1] <<16) | (tmp_page[0xFF - 2] << 8) | tmp_page[0xFF -3];
	printk("%s: vn=0x%X\n", __FUNCTION__, vn);
	return vn;
}

u32 I2C_SPISectorErase(
	struct v4l2_subdev *sd,
	u32 address,
	u32 stFlag
)
{
	u32 timeout;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	pr_info("addr:0x%x\n",address);
	if(!stFlag)
	{
		I2C_SPIFlashWrEnable(sd);
	
		//hsI2CDataWrite(0x40e7,0x00);
    	sensor_write_reg(client, 0x40e7,0x00);
		I2C_SPIFlashPortWrite(sd, SPI_CMD_WRT_STS_EN);				/*Write Status register command*/
		//hsI2CDataWrite(0x40e7,0x01);
    	sensor_write_reg(client, 0x40e7,0x01);
	}
	
	//hsI2CDataWrite(0x40e7,0x00);
    sensor_write_reg(client, 0x40e7,0x00);
	I2C_SPIFlashPortWrite(sd, SPI_CMD_WRT_STS);				/*Write Status register command*/
	I2C_SPIFlashPortWrite(sd, 0x02);
	//hsI2CDataWrite(0x40e7,0x01);
  	sensor_write_reg(client, 0x40e7,0x01);

	I2C_SPIFlashWrEnable(sd);

	//hsI2CDataWrite(0x40e7,0x00);
  	sensor_write_reg(client, 0x40e7,0x00);
	I2C_SPIFlashPortWrite(sd, SPI_CMD_SECTOR_ERASE);
	I2C_SPIFlashPortWrite(sd, address >> 16);	/* A23~A16 */
	I2C_SPIFlashPortWrite(sd, address >> 8);		/* A15~A08 */
	I2C_SPIFlashPortWrite(sd, address);			/* A07~A00 */
	//hsI2CDataWrite(0x40e7,0x01);
  	sensor_write_reg(client, 0x40e7,0x01);
	
	timeout = 5000000;
	I2C_SPITimeOutWait(sd, 0x01, &timeout);
	
	return 0;
}

u32 I2C_SPI32KBBlockErase(
	struct v4l2_subdev *sd,
	u32 address,
	u32 stFlag
)
{
	u32 timeout;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	pr_info("addr:0x%x\n",address);
	if(!stFlag)
	{
		I2C_SPIFlashWrEnable(sd);
	
		//hsI2CDataWrite(0x40e7,0x00);
    	sensor_write_reg(client, 0x40e7,0x00);
		I2C_SPIFlashPortWrite(sd, SPI_CMD_WRT_STS_EN);				/*Write Status register command*/
		//hsI2CDataWrite(0x40e7,0x01);
    	sensor_write_reg(client, 0x40e7,0x01);
	}

	//hsI2CDataWrite(0x40e7,0x00);
  	sensor_write_reg(client, 0x40e7,0x00);
	I2C_SPIFlashPortWrite(sd, SPI_CMD_WRT_STS);				/*Write Status register command*/
	I2C_SPIFlashPortWrite(sd, 0x02);
	//hsI2CDataWrite(0x40e7,0x01);
  	sensor_write_reg(client, 0x40e7,0x01);

	I2C_SPIFlashWrEnable(sd);

	//hsI2CDataWrite(0x40e7,0x00);
  	sensor_write_reg(client, 0x40e7,0x00);
	I2C_SPIFlashPortWrite(sd, SPI_CMD_32KB_BLOCK_ERASE);
	I2C_SPIFlashPortWrite(sd, address >> 16);	/* A23~A16 */
	I2C_SPIFlashPortWrite(sd, address >> 8);		/* A15~A08 */
	I2C_SPIFlashPortWrite(sd, address);			/* A07~A00 */
	//hsI2CDataWrite(0x40e7,0x01);
  	sensor_write_reg(client, 0x40e7,0x01);
	
	timeout = 5000000;
	I2C_SPITimeOutWait(sd, 0x01, &timeout);
	
	return 0;
}

u32 I2C_SPI64KBBlockErase(
	struct v4l2_subdev *sd,
	u32 address,
	u32 stFlag
)
{
	u32 timeout;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	pr_info("addr:0x%x\n",address);
	if(!stFlag)
	{
		I2C_SPIFlashWrEnable(sd);
	
		//hsI2CDataWrite(0x40e7,0x00);
  		sensor_write_reg(client, 0x40e7,0x00);
		I2C_SPIFlashPortWrite(sd, SPI_CMD_WRT_STS_EN);				/*Write Status register command*/
		//hsI2CDataWrite(0x40e7,0x01);
	  	sensor_write_reg(client, 0x40e7,0x01);
	}

	//hsI2CDataWrite(0x40e7,0x00);
  	sensor_write_reg(client, 0x40e7,0x00);
	I2C_SPIFlashPortWrite(sd, SPI_CMD_WRT_STS);				/*Write Status register command*/
	I2C_SPIFlashPortWrite(sd, 0x02);
	//hsI2CDataWrite(0x40e7,0x01);
  	sensor_write_reg(client, 0x40e7,0x01);

	I2C_SPIFlashWrEnable(sd);

	//hsI2CDataWrite(0x40e7,0x00);
  	sensor_write_reg(client, 0x40e7,0x00);
	I2C_SPIFlashPortWrite(sd, SPI_CMD_64KB_BLOCK_ERASE);
	I2C_SPIFlashPortWrite(sd, address >> 16);	/* A23~A16 */
	I2C_SPIFlashPortWrite(sd, address >> 8);		/* A15~A08 */
	I2C_SPIFlashPortWrite(sd, address);			/* A07~A00 */
	//hsI2CDataWrite(0x40e7,0x01);
  	sensor_write_reg(client, 0x40e7,0x01);
	
	timeout = 5000000;
	I2C_SPITimeOutWait(sd, 0x01, &timeout);
	
	return 0;
}

void
BB_EraseSPIFlash(
	struct v4l2_subdev *sd,
	u32 type,
	u32 spiSize
)
{
	u8 typeFlag=0;
	u32 i, temp1;
	if( type == 2 )/* SST */
	{
		typeFlag = 0;
	}
	else if( type == 1 || type == 3 )/* ST */
	{
		typeFlag = 1;
	}else{
            pr_err("%s type(%d) is not support\n",__func__,type);
            return;
       }
    
	/*printf("spiSize:0x%x\n",spiSize);*/
	if(spiSize == (512*1024))
	{
		/* skip 0x7B000 ~ 0x7EFF, to keep calibration data */
		temp1 = (spiSize / 0x10000)-1;
		for(i=0;i<temp1;i++)
		{
			I2C_SPI64KBBlockErase(sd,i*0x10000,typeFlag);
		}
		I2C_SPI32KBBlockErase(sd,temp1*0x10000,typeFlag);
		temp1 = temp1*0x10000 + 0x8000;
		for(i=temp1;i<spiSize-0x5000;i+=0x1000)
		{
			I2C_SPISectorErase(sd,i,typeFlag);
		}
		I2C_SPISectorErase(sd,spiSize-0x1000,typeFlag);
	}
	else if(spiSize == (1024*1024))
	{
		/* only erase 256*3KB */
		temp1 = ((spiSize*3/4) / 0x10000);
		for(i=0;i<temp1;i++)
		{
			I2C_SPI64KBBlockErase(sd,i*0x10000,typeFlag);
		}
		I2C_SPI32KBBlockErase(sd,(temp1+1),typeFlag);
		I2C_SPISectorErase(sd,spiSize-0x1000,typeFlag);
	}
}


void
BB_WrSPIFlash(struct v4l2_subdev *sd, char* binfile_path, bool erase_all)
{

	u32 id, type;
	u32 pages;

	u8 *pbootBuf = NULL;
	u8 bin_file_header[BIN_FILE_HEADER_SIZE];
	u8 checksum1_in_bin[2], checksum2_in_bin[2];
	u8 checksum1_in_isp[2], checksum2_in_isp[2];
	int firmware2_offset;
	u8 tmp_page[0x100];

	struct file *fp = NULL;
	mm_segment_t old_fs;
	struct inode *inode;
	int bootbin_size = 0;
	int i, ret = 0;

    //Add for ISP firmware version in setting	
    char *envp[3];
    char name_buf[64];
    int env_offset = 0;	

	struct ov5693_device *dev = to_ov5693_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);



	wake_lock(dev->fw_update_wakelock);
	
	ret = dev->platform_data->gpio_ctrl(sd, 1);
	if (ret) {
		dev_err(&client->dev, "ov5693 power-up err");
	}

	fw_update_status = ICATCH_FW_IS_BURNING;

	sensor_write_reg(client, 0x70c4,0x00);
	sensor_write_reg(client, 0x70c5,0x00);

	sensor_write_reg(client, 0x1011,0x01); /* CPU reset */
	sensor_write_reg(client, 0x001C,0x08); /* FM reset */
	sensor_write_reg(client, 0x001C,0x00);
	sensor_write_reg(client, 0x108C,0x00); /* DMA select */
	sensor_write_reg(client, 0x009a,0x00); /* CPU normal operation */

	/* Calculate BOOT.BIN file size. */
	fp = filp_open(binfile_path, O_RDONLY, 0);

	if ( !IS_ERR_OR_NULL(fp) ){
		pr_info("filp_open success fp:%p\n", fp);
		inode = fp->f_dentry->d_inode;
		bootbin_size = inode->i_size;
		printk("%s: fp->f_dentry->d_inode->i_size=%d\n", __FUNCTION__, bootbin_size);
		pbootBuf = kmalloc(bootbin_size, GFP_KERNEL);
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		if(fp->f_op != NULL && fp->f_op->read != NULL){
			int byte_count= 0;
			printk("Start to read %s\n", binfile_path);

			byte_count = fp->f_op->read(fp, pbootBuf, bootbin_size, &fp->f_pos);
#if 0
			if (byte_count <= 0) {
				printk("iCatch: EOF or error. last byte_count= %d;\n", byte_count);
				kfree(pbootBuf);
				fw_update_status = ICATCH_FW_UPDATE_FAILED;
				goto end;
			} else
#endif			
				printk("iCatch: BIN file size= %d bytes\n", bootbin_size);

#if 0
			for(i=0; i < bootbin_size; i++) {
				printk("%c", pbootBuf[i]);
			}
			printk("\n");
#endif
		}
		set_fs(old_fs);
		filp_close(fp, NULL);
	} else if(PTR_ERR(fp) == -ENOENT) {
		pr_err("iCatch \"%s\" not found error\n", binfile_path);
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		goto end;
	} else{
		pr_err("iCatch \"%s\" open error\n", binfile_path);
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		goto end;
	}

	for (i=0; i < BIN_FILE_HEADER_SIZE; i++)
	{
		bin_file_header[i] = pbootBuf[bootbin_size - BIN_FILE_HEADER_SIZE + i];
		printk("%s: bin_file_header[%d]= 0x%x\n", __FUNCTION__, i,bin_file_header[i]);
	}
	version_num_in_bin = (bin_file_header[30] << 16) | (bin_file_header[29] << 8) | bin_file_header[28];

	/* Get the checksum in bin file.
	 *   firmware2_offset
	 *     = fw1 header size
	 *     + fw1 DMEM FICDMEM size
	 *     + fw1 IMEM size
	 */
	memcpy(checksum1_in_bin, pbootBuf + 10, 2);

	firmware2_offset = 16 +
		((pbootBuf[3] << 24) | (pbootBuf[2] << 16) | (pbootBuf[1] << 8) | pbootBuf[0]) +
		((pbootBuf[7] << 24) | (pbootBuf[6] << 16) | (pbootBuf[5] << 8) | pbootBuf[4]);
	memcpy(checksum2_in_bin, pbootBuf + firmware2_offset + 10, 2);

	printk("%s: checksum in bin:%02X %02X; %02X %02X\n", __FUNCTION__,
		checksum1_in_bin[0],checksum1_in_bin[1],checksum2_in_bin[0], checksum2_in_bin[1]);

	ret = I2C_SPIInit(sd);
	if (ret) {
		printk("%s: SPI init fail. ret= 0x%x", __FUNCTION__, ret);
		kfree(pbootBuf);
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		goto end;
	}

	id = I2C_SPIFlashReadId(sd);

	if(id==0) {
		printk("read id failed\n");
		kfree(pbootBuf);
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		goto end;
	}

	type = BB_SerialFlashTypeCheck(id);
	if(type == 0) {
		printk("BB_SerialFlashTypeCheck(%d) failed\n", id);
		kfree(pbootBuf);
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		goto end;
	}

	pages = bootbin_size/0x100;

	printk("%s: pages:0x%x\n", __FUNCTION__, pages);

	if (erase_all == false) {
		printk("%s: Erase partial partition.\n", __FUNCTION__);
		BB_EraseSPIFlash(sd,type,bootbin_size);
	}

	/* Writing Flash here */
	if( type == 2 ) {
		flash_type = ICATCH_FLASH_TYPE_SST;
		printk("SST operation\n");
		if (erase_all == true) {
			printk("%s: Erase all partition.\n", __FUNCTION__);
			ret = I2C_SPISstChipErase(sd);
			if(ret) {
				printk("%s: SST erase fail.\n", __FUNCTION__);
				kfree(pbootBuf);
				fw_update_status = ICATCH_FW_UPDATE_FAILED;
				goto end;
			}
		}
		I2C_SPISstFlashWrite(sd, 0, pages, pbootBuf);
	} else if( type == 1 || type == 3 ) {
		flash_type = ICATCH_FLASH_TYPE_ST;
		printk("ST operation\n");
		if (erase_all == true) {
			printk("%s: Erase all partition.\n", __FUNCTION__);
			ret = I2C_SPIStChipErase(sd);
			if(ret) {
				printk("%s: ST erase fail.\n", __FUNCTION__);
				kfree(pbootBuf);
				fw_update_status = ICATCH_FW_UPDATE_FAILED;
				goto end;
			}
		}
		ret=I2C_SPIFlashWrite_DMA(sd, 0, pages, pbootBuf); //new burn ISP flow
        if(ret<0){
        	printk("I2C_SPIFlashWrite_DMA fail\n");
            fw_update_status = ICATCH_FW_UPDATE_FAILED;
			kfree(pbootBuf);
			goto end;			
        }
	} else {
		printk("type unknown: %d; Won't update iCatch FW.\n", type);
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		kfree(pbootBuf);
		goto end;
	}
	kfree(pbootBuf);

	/* Check the update reult. */
	/* Compare Check sum here */
	get_one_page_from_i7002a(sd, 0, tmp_page);
	memcpy(checksum1_in_isp, tmp_page + 10, 2);

	if (memcmp(checksum1_in_isp, checksum1_in_bin, 2) == 0) {
		/* checksum1 PASS */
		firmware2_offset = 16 +
			((tmp_page[3] << 24) | (tmp_page[2] << 16) | (tmp_page[1] << 8) | tmp_page[0]) +
			((tmp_page[7] << 24) | (tmp_page[6] << 16) | (tmp_page[5] << 8) | tmp_page[4]);

		get_one_page_from_i7002a(sd, firmware2_offset >> 8, tmp_page);
		memcpy(checksum2_in_isp, tmp_page + 10, 2);

		if (memcmp(checksum2_in_isp, checksum2_in_bin, 2) == 0) {
			/* checksum2 PASS */
			version_num_in_isp = get_fw_version_in_isp(sd);
			if (version_num_in_isp == version_num_in_bin) {
				/* version number PASS */
				fw_update_status = ICATCH_FW_UPDATE_SUCCESS;
				printk("%s: ICATCH FW UPDATE SUCCESS.\n", __FUNCTION__);
				//Add for ISP firmware version in setting
				snprintf(name_buf, sizeof(name_buf), "SWITCH_NAME=0x%x", version_num_in_isp);
				envp[env_offset++] = name_buf;
				envp[env_offset] = NULL;
				kobject_uevent_env(&ISP_sdev.dev->kobj, KOBJ_CHANGE, envp);				
			} else {
				/* version number FAIL */
				fw_update_status = ICATCH_FW_UPDATE_FAILED;
				printk("%s: check version FAIL: ISP(0x%06X) != BIN(0x%06X)\n", __FUNCTION__, version_num_in_isp, version_num_in_bin);
				version_num_in_isp = 0xABCDEF;
			}
		} else {
			/* checksum2 FAIL */
			fw_update_status = ICATCH_FW_UPDATE_FAILED;
			printk("%s: checksum2 FAIL: ISP(%02X %02X) != BIN(%02X %02X)\n",
				__FUNCTION__, checksum2_in_isp[0], checksum2_in_isp[1],
				checksum2_in_bin[0], checksum2_in_bin[1]);
			version_num_in_isp = 0xABCDEF;
		}
	} else {
		/* checksum1 FAIL */
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		printk("%s: checksum1 FAIL: ISP(%02X %02X) != BIN(%02X %02X)\n",
			__FUNCTION__, checksum1_in_isp[0], checksum1_in_isp[1],
			checksum1_in_bin[0], checksum1_in_bin[1]);
		version_num_in_isp = 0xABCDEF;
	}

end:
	ret = dev->platform_data->gpio_ctrl(sd, 0);
	if (ret) {
		dev_err(&client->dev, "ov5693 power down err");
	}	
	wake_unlock(dev->fw_update_wakelock);

}

u32 I2C_7002DmemRd(
	struct v4l2_subdev *sd,
	u32 bankNum,
	u32 byteNum,
	u8* pbuf
)
{
	u32 i, bank;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	bank = 0x40+bankNum;
	sensor_write_reg(client, 0x10A6,bank);

	for(i=0;i<byteNum;i+=4)
	{
		//seqI2CDataRead(client, (0x1800+i),(pbuf+i));
		sensor_read_reg(client, (0x1800+i), (pbuf+i));
	}
	
	bank = 0x40 + ((bankNum+1)%2);
	sensor_write_reg(client, 0x10A6,bank);
}

u32 I2C_SPIFlashRead_DMA(
	struct v4l2_subdev *sd,
	u32 addr,
	u32 pages,
	u8 *pbuf
)
{
    u8* pbufR;
    u32 ch, err = SUCCESS, dmemBank;
    u32 i, ret, count=0, size=0, bytes, offset;
    u32 pageSize = 0x100;

    addr = addr * pageSize;
    size = pages*pageSize;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	/* Set DMA bytecnt as 256-1 */
	sensor_write_reg(client, 0x4170,0xff);
	sensor_write_reg(client, 0x4171,0x00);
	sensor_write_reg(client, 0x4172,0x00);

	/* Set DMA bank & DMA start address */
	sensor_write_reg(client, 0x1084,0x01);
	sensor_write_reg(client, 0x1080,0x00);
	sensor_write_reg(client, 0x1081,0x00);
	sensor_write_reg(client, 0x1082,0x00);

	/* enable DMA checksum and reset checksum */
	sensor_write_reg(client, 0x4280,0x01);
	sensor_write_reg(client, 0x4284,0x00);
	sensor_write_reg(client, 0x4285,0x00);
	sensor_write_reg(client, 0x4164,0x01);

	while(pages)
	{
	sensor_write_reg(client, 0x40e7,0x00);
	I2C_SPIFlashPortWrite(sd, SPI_CMD_BYTE_READ);               /* Write one byte command*/
	I2C_SPIFlashPortWrite(sd, (u8)(addr >> 16));               /* Send 3 bytes address*/
	I2C_SPIFlashPortWrite(sd, (u8)(addr >> 8));
	I2C_SPIFlashPortWrite(sd, (u8)(addr));

	if( (pages%0x40)==0x00 )
	{
		printk("RE:0x%x\n",pages);
	}
	dmemBank = pages % 2;
	sensor_write_reg(client, 0x1081,dmemBank*0x20);
	sensor_write_reg(client, 0x1084,(1<<dmemBank));
	sensor_write_reg(client, 0x4160,0x01);
	udelay(100);
	sensor_write_reg(client, 0x40e7,0x01);
	I2C_7002DmemRd(sd, dmemBank,pageSize,pbuf);

		pbuf += pageSize;
		pages--;
		addr += pageSize;
	}

    return err;
}

void
BB_RdSPIFlash(struct v4l2_subdev *sd, int which_page, u32 sizes)
{
   	u32 i, id;
	u32 pages, size;

	u8* pbootBuf = NULL;
	u32 checksum1 = 0, checksum2 = 0;
	u32 oriChecksum1 = 0, oriChecksum2 = 0;
	u32 fwSize1, fwSize2;

	int ret;
	struct file *fp = NULL;
	loff_t pos = 0;
	mm_segment_t old_fs;

	struct ov5693_device *dev = to_ov5693_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	ret = dev->platform_data->gpio_ctrl(sd, 1);
	if (ret) {
		dev_err(&client->dev, "ov5693 power-up err");
	}

	sensor_write_reg(client, 0x70c4,0x00);
	sensor_write_reg(client, 0x70c5,0x00);

	//pbootBuf = osMemAlloc(0x100);
	pbootBuf = kmalloc(0x100, GFP_KERNEL);

	ret = I2C_SPIInit(sd);
	if (ret) {
		printk("%s: SPI init fail. ret= 0x%x", __FUNCTION__, ret);
		kfree(pbootBuf);
		return;
	}

	id = I2C_SPIFlashReadId(sd);
	if(id==0) {
		printk("read id failed\n");
		kfree(pbootBuf);
		return;
	}

	I2C_SPIFlashRead(sd, 0, 1, pbootBuf);

	fwSize1 = *(pbootBuf);
	fwSize1 += *(pbootBuf+1)<<8;
	fwSize1 += *(pbootBuf+2)<<16;
	fwSize1 += *(pbootBuf+3)<<24;
	fwSize1 += *(pbootBuf+4);
	fwSize1 += *(pbootBuf+5)<<8;
	fwSize1 += *(pbootBuf+6)<<16;
	fwSize1 += *(pbootBuf+7)<<24;

	oriChecksum1 = *(pbootBuf+10);
	oriChecksum1 += *(pbootBuf+11)<<8;

	printk("first firmware size:0x%x original checksum:0x%x\n",
		fwSize1, oriChecksum1);
	pages = ((fwSize1+0x10)/0x100);
	I2C_SPIFlashRead(sd, pages, 1, pbootBuf);

	fwSize2 = *(pbootBuf+0x0);
	fwSize2 += *(pbootBuf+0x1)<<8;
	fwSize2 += *(pbootBuf+0x2)<<16;
	fwSize2 += *(pbootBuf+0x3)<<24;
	fwSize2 += *(pbootBuf+0x4);
	fwSize2 += *(pbootBuf+0x5)<<8;
	fwSize2 += *(pbootBuf+0x6)<<16;
	fwSize2 += *(pbootBuf+0x7)<<24;

	oriChecksum2 = *(pbootBuf+10);
	oriChecksum2 += *(pbootBuf+11)<<8;
	printk("second firmware size:0x%x  original checksum:0x%x\n",
		fwSize2, oriChecksum2);

	//osMemFree(pbootBuf);
	kfree(pbootBuf);

	size = sizes*0x100;
	printk("pages:0x%x\n",sizes);
	pbootBuf = kmalloc(size, GFP_KERNEL);
	I2C_SPIFlashRead(sd,which_page,sizes,pbootBuf);
	//I2C_SPIFlashRead_DMA(sd,0,sizes,pbootBuf);

	printk("Write fw to file\n");
	/* save file for verification */
	fp = filp_open(FW_BIN_FILE_WITH_PATH_READ, O_RDWR|O_CREAT|O_LARGEFILE, 0644);
	if ( IS_ERR_OR_NULL(fp) ){
		filp_close(fp, NULL);
		printk("%s: open %s fail\n", __FUNCTION__, FW_BIN_FILE_WITH_PATH_READ);
	}

	printk("File is opened\n");
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;

	printk("File is writing\n");
	if (fp->f_op != NULL && fp->f_op->write != NULL){
		fp->f_op->write(fp,pbootBuf,size,&pos);
	}else{
		pr_err("%s: f_op might be null\n", __FUNCTION__);
	}

	printk("File is writed, close it\n");
	set_fs(old_fs);
	filp_close(fp, NULL);

	//osMemFree(pbootBuf);
	kfree(pbootBuf);

	ret = dev->platform_data->gpio_ctrl(sd, 0);
	if (ret) {
		dev_err(&client->dev, "ov5693 power down err");
	}
}
//i7002a fireware---



//ASUS_BSP+++, add for ISP firmware update, create proc file
#ifdef	CONFIG_PROC_FS
#define	i7002a_PROC_FILE	"driver/i7002a"
static struct proc_dir_entry *i7002a_proc_file;

static ssize_t i7002a_proc_read(char *page, char **start, off_t off, int count, 
            	int *eof, void *data)
{
		
	if (SPI_ENABLE==0) {
		return sprintf(page, "version_in_isp:%x\n", version_num_in_isp);
	} else {
		return sprintf(page, "version_in_bin:%x\n", version_num_in_bin);
	}
}

static ssize_t i7002a_proc_write(struct file *filp, const char __user *buff, 
	            unsigned long len, void *data)
{
	char buffer[256];
	struct i2c_client *client = v4l2_get_subdevdata(fw_sd);
	u16 addr, value, read_value, ret;
	int which_page;
	u32 size;
//FW_BSP++
	struct file *fp = NULL;
	mm_segment_t old_fs;
	struct inode *inode;
//FW_BSP--


	if (len > 256)
		len = 256;

        pr_info("i7002a_proc_write %s\n", buff);
	if (copy_from_user(buffer, buff, len)) {
		printk(KERN_INFO "%s: proc write to buffer failed.\n", __func__);
		return -EFAULT;
	}
	if ('f' == buffer[0]) {
		/* Update ISP firmware*/
		//i7002a_update_status = 0;

		FW_BIN_FILE_WITH_PATH = EXTERNAL_FW_BIN_FILE_WITH_PATH;

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
		} else  {
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
			} else{
				pr_err("iCatch \"%s\" open error\n", FW_BIN_FILE_WITH_PATH);
				FW_BIN_FILE_WITH_PATH = DEFAULT_FW_BIN_FILE_WITH_PATH;
			}
		
		}


		if (SPI_ENABLE==0) {
			pr_info("i7002a firmware update start\n");
			BB_WrSPIFlash(fw_sd, FW_BIN_FILE_WITH_PATH, false);			
		} else {
			pr_info("i7002a doesn't support firmware update in this HW version\n");
		}

	
	} else if(!strncmp("ri2c",buffer,4)){
		sscanf(&buffer[5],"%x",&addr);
		printk(KERN_INFO "%s: addr 0x%x.\n", __func__, addr);
		sensor_read_reg(client, addr, &read_value);
		printk("I2C read REG:0x%x is 0x%x \n", addr, read_value);
	} else if(!strncmp("wi2c",buffer,4)){
		sscanf(&buffer[5],"%x",&addr);
		sscanf(&buffer[10],"%x",&value);
		printk(KERN_INFO "%s: addr 0x%x, val 0x%x.\n", __func__, addr, value);
		sensor_write_reg(client, addr, value);
	} else if(!strncmp("d1",buffer,2)){ //Add ROI display for debugging
		pr_info("%s Enable ROI debug\n",__func__);
		g_enable_roi_debug = true;
	} else if(!strncmp("d0",buffer,2)){ //Add ROI display for debugging
		pr_info("%s Disable ROI debug\n",__func__);
		g_enable_roi_debug = false;
	} else if(!strncmp("is_calibration",buffer,14)){ //Add for calibration
		sscanf(&buffer[15],"%d",&value);
		printk(KERN_INFO "%s: Calibration val for rear: %d.\n", __func__, value);
		g_is_calibration = value;				
	} else if(!strncmp("erase_all",buffer,9)){ //Erase all partition after FW update
		pr_info("%s Erase all partition after FW update\n",__func__);

		if (SPI_ENABLE==0) {
			pr_info("i7002a firmware update start\n");
			BB_WrSPIFlash(fw_sd, FW_BIN_FILE_WITH_PATH, true);			
		} else {
			pr_info("i7002a doesn't support firmware update in this HW version\n");
		}
		
	} else if(!strncmp("read_fw",buffer,7)){ //Read fw in SPI flash for calibration
		sscanf(&buffer[8],"%x",&which_page);
		sscanf(&buffer[12],"%d",&size);
		pr_info("%s Read camera fw from SPI flash, page:0x%x size %d\n",__func__,which_page,size);
		BB_RdSPIFlash(fw_sd,which_page,size);
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

int sensor_s_focus_mode(struct v4l2_subdev *sd, int focus_mode)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int reg_val;
	int ret = 0;
	printk("%s mode = %d\n", __func__, focus_mode);

	switch (focus_mode) {
	case SENSOR_FOCUS_MODE_AUTO:
		sensor_write_reg(client, 0x7105, 0x00);//auto
		sensor_write_reg(client, 0x7188, 0x01);//ROI On
		break;
	case SENSOR_FOCUS_MODE_MACRO:
		sensor_write_reg(client, 0x7105, 0x01);//macro
		break;
	case SENSOR_FOCUS_MODE_INFINITY:
		sensor_write_reg(client, 0x7105, 0x02);//infinity
		sensor_write_reg(client, 0x7188, 0x00);//ROI Off
		break;
	case SENSOR_FOCUS_MODE_CONTINUOUS:
		printk("%s hack, set to auto\n", __func__);
		//sensor_write_reg(client, 0x7105, 0x03);//CAF
		sensor_write_reg(client, 0x7105, 0x00);//auto
		sensor_write_reg(client, 0x7188, 0x01);//ROI On
		break;
	default:
		dev_err(&client->dev, "invalid focus mode: %d", focus_mode);
		return -ERANGE;
	}

	return 0;
}

int sensor_s_roi_trigger(struct v4l2_subdev *sd, int trigger)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	printk("%s trigger %d\n", __func__, trigger);

	switch (trigger) {
	case SENSOR_ROI_TRIGGER_AF:
		sensor_write_reg(client, 0x714E, 0x00);//AE ROI Stop
		sensor_write_reg(client, 0x7146, 0x01);//AF ROI Trigger
		break;
	case SENSOR_ROI_TRIGGER_AE_AF:
		sensor_write_reg(client, 0x714E, 0x02);//AE ROI Trigger(Use TAF ROI)
		sensor_write_reg(client, 0x7146, 0x01);//AF ROI Trigger
		break;
	case SENSOR_ROI_TRIGGER_STOP:
		sensor_write_reg(client, 0x714E, 0x00);//AE ROI Stop
		sensor_write_reg(client, 0x714F, 0x01);//AF abort and stop in the CURRENT position
		break;
	default:
		dev_err(&client->dev, "invalid ROI trigger: %d", trigger);
		return -ERANGE;
	}

	return 0;
}

int sensor_s_focus_window(struct v4l2_subdev *sd, char *window)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int reg_val;
	int ret = 0;
	printk("%s\n", __func__);

	int x_left, x_right, y_top, y_bottom;
	int start_x, start_y, size;

	sscanf(window, "%d,%d,%d,%d",
		&x_left, &y_top, &x_right, &y_bottom);

	start_x = x_left;
	start_y = y_top;
	size = x_right - x_left;

	printk("%s, x:%d, y:%d, size:%d\n", __func__, start_x, start_y, size);

	//Only set ROI window, trigger later
	sensor_write_reg(client, 0x7188, 0x01);//AF ROI On
	sensor_write_reg(client, 0x7140, size>>8);//AF ROI Size_H
	sensor_write_reg(client, 0x7141, size&0xFF);//AF ROI Size_L
	sensor_write_reg(client, 0x7142, start_x>>8);//AF ROI X_H
	sensor_write_reg(client, 0x7143, start_x&0xFF);//AF ROI X_L
	sensor_write_reg(client, 0x7144, start_y>>8);//AF ROI Y_H
	sensor_write_reg(client, 0x7145, start_y&0xFF);//AF ROI Y_L

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
int sensor_g_focus_result(struct v4l2_subdev *sd, int *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 retvalue;
	int ret = 0;

	ret = sensor_read_reg(client, 0x72A0, &retvalue);
	if (retvalue==1) { //S_BUSY
		printk("%s, Focus is processing\n", __func__);
		*val = 0; //busy
		return 0;
	}

	ret = sensor_read_reg(client, 0x72A1, &retvalue);
	if (retvalue==0) {
		printk("%s, Focus Success\n", __func__);
		*val = 1; //Locked
	} else {
		printk("%s, Focus Fail\n", __func__);
		*val = 2; //failed to find
	}

	//Add ROI display for debugging
	if(true == g_enable_roi_debug){
		sensor_write_reg(client, 0x243f, 0x01);
	}

	return 0;
}

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


static int ov5693_init_common(struct v4l2_subdev *sd)
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
			ov5693_s_power(sd, 0);
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
		sensor_write_reg(client, 0x9032, 0x00);
		mdelay(10);
		sensor_write_reg(client, 0x9032, 0x30);
		mdelay(10);
		// End - Power on sensor & enable clock

		sensor_write_reg(client, 0x9008, 0x00);
		sensor_write_reg(client, 0x9009, 0x00);
		sensor_write_reg(client, 0x900A, 0x00);
		sensor_write_reg(client, 0x900B, 0x00);

		// Start - I2C Read
		sensor_write_reg(client, 0x9238, 0x30);	// Sub address enable
		sensor_write_reg(client, 0x9240, 0x20);	// Slave address
		sensor_write_reg(client, 0x9200, 0x03);	// Read mode
		sensor_write_reg(client, 0x9210, 0x30);	// Register addr MSB
		sensor_write_reg(client, 0x9212, 0x0A);	// Register add LSB
		sensor_write_reg(client, 0x9204, 0x01);	// Trigger I2C read
		mdelay(5);
		sensor_read_reg(client, 0x9211, &retvalue_h);

		sensor_write_reg(client, 0x9210, 0x30);	// Register addr MSB
		sensor_write_reg(client, 0x9212, 0x0B);	// Register addr LSB
		sensor_write_reg(client, 0x9204, 0x01);	// Trigger I2C read
		mdelay(5);
		sensor_read_reg(client, 0x9211, &retvalue_l);
	

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
		//Add for calibration ++
		printk("<<g_is_calibration :%d\n", g_is_calibration);
		if(g_is_calibration){
			sensor_write_reg(client, 0x1306, 0x02);//rear camera: 0, front camera: 1, calibration: 2
		}else{
			sensor_write_reg(client, 0x1306, 0x00);//rear camera: 0, front camera: 1, calibration: 2
		}
		//Add for calibration --
		sensor_write_reg(client, 0x1011, 0x00);
		msleep(70); //wait for iCatch load sensor code		
	}else {
		if ((FW_BIN_FILE==true)&&((FIRST_BOOTING==false)||(build_version==1))) {
			if(g_is_calibration){
				status = EXISP_LoadCode(0x02, g_is_calibration, pIspFW_g, pCalibOpt_g, p3acali_g, pLsc_g, pLscdq_g, sd); //rear camera: 0, front camera: 1, calibration: 2
			}
			else {
				status = EXISP_LoadCode(0x00, g_is_calibration, pIspFW_g, pCalibOpt_g, p3acali_g, pLsc_g, pLscdq_g, sd); //rear camera: 0, front camera: 1, calibration: 2
			}
			fw_update_status = ICATCH_FW_UPDATE_SUCCESS;
			if(g_is_calibration){
				sensor_write_reg(client, 0x1306, 0x02); /* Set which FW to be loaded */	
			} else {
				sensor_write_reg(client, 0x1306, 0x00); /* Set which FW to be loaded */	
			}
		}
		FIRST_BOOTING=false;
	}
//FW_BSP--
	return 0;
}

static int power_up(struct v4l2_subdev *sd)
{
	struct ov5693_device *dev = to_ov5693_sensor(sd);
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
	struct ov5693_device *dev = to_ov5693_sensor(sd);
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

static int ov5693_s_power(struct v4l2_subdev *sd, int power)
{
	if (power == 0)
		return power_down(sd);

	if (power_up(sd))
		return -EINVAL;

	return ov5693_init_common(sd);
}

static int ov5693_try_res(u32 *w, u32 *h)
{
	int i;

	//printk("%s\n", __func__);

	/*
	 * The mode list is in ascending order. We're done as soon as
	 * we have found the first equal or bigger size.
	 */
	for (i = 0; i < N_RES; i++) {
		if ((ov5693_res[i].width >= *w) &&
		    (ov5693_res[i].height >= *h))
			break;
	}

	/*
	 * If no mode was found, it means we can provide only a smaller size.
	 * Returning the biggest one available in this case.
	 */
	if (i == N_RES)
		i--;

	*w = ov5693_res[i].width;
	*h = ov5693_res[i].height;

	return 0;
}

static struct ov5693_res_struct *ov5693_to_res(u32 w, u32 h)
{
	int  index;

	//printk("%s\n", __func__);

	for (index = 0; index < N_RES; index++) {
		if ((ov5693_res[index].width == w) &&
		    (ov5693_res[index].height == h))
			break;
	}

	/* No mode found */
	if (index >= N_RES)
		return NULL;

	return &ov5693_res[index];
}

static int ov5693_try_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	return ov5693_try_res(&fmt->width, &fmt->height);
}

static int ov5693_res2size(unsigned int res, int *h_size, int *v_size)
{
	unsigned short hsize;
	unsigned short vsize;

	//printk("%s\n", __func__);

	switch (res) {
	case OV5693_RES_QCIF:
		hsize = OV5693_RES_QCIF_SIZE_H;
		vsize = OV5693_RES_QCIF_SIZE_V;
		break;
	case OV5693_RES_QVGA:
		hsize = OV5693_RES_QVGA_SIZE_H;
		vsize = OV5693_RES_QVGA_SIZE_V;
		break;
	case OV5693_RES_VGA:
		hsize = OV5693_RES_VGA_SIZE_H;
		vsize = OV5693_RES_VGA_SIZE_V;
		break;
/*
	case OV5693_RES_480P:
		hsize = OV5693_RES_480P_SIZE_H;
		vsize = OV5693_RES_480P_SIZE_V;
		break;
*/
	case OV5693_RES_720P:
		hsize = OV5693_RES_720P_SIZE_H;
		vsize = OV5693_RES_720P_SIZE_V;
		break;
	case OV5693_RES_960P:
		hsize = OV5693_RES_960P_SIZE_H;
		vsize = OV5693_RES_960P_SIZE_V;
		break;
	case OV5693_RES_1080P:
		hsize = OV5693_RES_1080P_SIZE_H;
		vsize = OV5693_RES_1080P_SIZE_V;
		break;
	case OV5693_RES_3M:
		hsize = OV5693_RES_3M_SIZE_H;
		vsize = OV5693_RES_3M_SIZE_V;
		break;
	case OV5693_RES_5M:
		hsize = OV5693_RES_5M_SIZE_H;
		vsize = OV5693_RES_5M_SIZE_V;
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

static int ov5693_get_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	struct ov5693_device *dev = to_ov5693_sensor(sd);
	int width, height;
	int ret;

	printk("%s\n", __func__);

	ret = ov5693_res2size(dev->res, &width, &height);
	if (ret)
		return ret;
	fmt->width = width;
	fmt->height = height;

	return 0;
}

static int ov5693_set_mbus_fmt(struct v4l2_subdev *sd,
			      struct v4l2_mbus_framefmt *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5693_device *dev = to_ov5693_sensor(sd);
	struct ov5693_res_struct *res_index;
	u32 width = fmt->width;
	u32 height = fmt->height;
	int ret;
	u16 testval, i, addr;

	printk("%s\n", __func__);

	ov5693_try_res(&width, &height);
	res_index = ov5693_to_res(width, height);

	/* Sanity check */
	if (unlikely(!res_index)) {
		WARN_ON(1);
		return -EINVAL;
	}

	switch (res_index->res) {
//TODO: add supported resolution later
/*
	case OV5693_RES_QCIF:
		ret = ov5693_write_reg_array(c, ov5693_qcif_30_init);
		dev_info(&c->dev, "%s: set for qcif\n", __func__);
		break;
	case OV5693_RES_QVGA:
		ret = ov5693_write_reg_array(c, ov5693_qvga_30_init);
		dev_info(&c->dev, "%s: set for qvga\n", __func__);
		break;
*/
	case OV5693_RES_VGA:
		//Add for calibration ++
		if(g_is_calibration){
			sensor_write_reg(client, 0x7106, 0x0F); //SET resolution 1280x960 with sensor full in
		}else{
			sensor_write_reg(client, 0x7106, 0x0B); //SET resolution 640x480
		}
		//Add for calibration --
		dev_info(&client->dev, "%s: set for VGA\n", __func__);
		break;
/*
	case OV5693_RES_480P:
		ret = ov5693_write_reg_array(c, ov5693_svga_30_init);
		dev_info(&c->dev, "%s: set for svga\n", __func__);
		break;
*/
	case OV5693_RES_720P:
		sensor_write_reg(client, 0x7106, 0x04); //SET resolution 1280x720
		dev_info(&client->dev, "%s: set for 720p\n", __func__);
		break;
	case OV5693_RES_960P:
		//Add for calibration ++
		if(g_is_calibration){
			sensor_write_reg(client, 0x7106, 0x0F); //SET resolution 1280x960 with sensor full in
			dev_info(&client->dev, "%s: calibration for 960p\n", __func__);
		} else {
			sensor_write_reg(client, 0x7106, 0x00); //SET resolution 1280x960
			dev_info(&client->dev, "%s: set for 960p\n", __func__);
		}
		//Add for calibration --
		break;
	case OV5693_RES_1080P:
		sensor_write_reg(client, 0x7106, 0x02); //SET resolution 1920x1080
		dev_info(&client->dev, "%s: set for 1080p\n", __func__);
		break;
	case OV5693_RES_3M:
		sensor_write_reg(client, 0x7106, 0x06); //SET resolution 2080x1560
		dev_info(&client->dev, "%s: Snapshot 5M\n", __func__);
		break;
	case OV5693_RES_5M:
		sensor_write_reg(client, 0x7106, 0x0A); //SET resolution 2592x1944
		dev_info(&client->dev, "%s: Snapshot 5M\n", __func__);
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

static int ov5693_g_focal(struct v4l2_subdev *sd, s32 *val)
{
	*val = (OV5693_FOCAL_LENGTH_NUM << 16) | OV5693_FOCAL_LENGTH_DEM;
	return 0;
}

static int ov5693_g_fnumber(struct v4l2_subdev *sd, s32 *val)
{
	/* const f number for OV5693 */
	*val = (OV5693_F_NUMBER_DEFAULT_NUM << 16) | OV5693_F_NUMBER_DEM;
	return 0;
}

static int ov5693_g_fnumber_range(struct v4l2_subdev *sd, s32 *val)
{
	*val = (OV5693_F_NUMBER_DEFAULT_NUM << 24) |
		(OV5693_F_NUMBER_DEM << 16) |
		(OV5693_F_NUMBER_DEFAULT_NUM << 8) | OV5693_F_NUMBER_DEM;
	return 0;
}

/*
 * This returns EV.
 */
static int ov5693_get_exposure_bias(struct v4l2_subdev *sd, s32 *value)
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
			else if (iso_ratio < 16)
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
static struct ov5693_control ov5693_controls[] = {
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
		.query = ov5693_get_exposure_bias, //For EXIF value, need to combine with other parameter
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
			.id = V4L2_CID_AUTO_FOCUS_RANGE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "AF Mode",
			.minimum = 0,
			.maximum = 9,
			.step = 1,
			.default_value = 0,
		},
		.tweak = sensor_s_focus_mode,
	},
	{
		.qc = {
			.id = V4L2_CID_IRIS_RELATIVE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "ROI Trigger",
			.minimum = 0,
			.maximum = 9,
			.step = 1,
			.default_value = 0,
		},
		.tweak = sensor_s_roi_trigger,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCUS_AUTO,
			.type = V4L2_CTRL_TYPE_STRING,
			.name = "Focus Window",
			.minimum = 0,
			.maximum = 255,
			.step = 1,
			.default_value = 0,
		},
		.tweak = sensor_s_focus_window,
	},
	{
		.qc = {
			.id = V4L2_CID_AUTO_FOCUS_STATUS,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Focus Result",
			.minimum = 0,
			.maximum = 9,
			.step = 1,
			.default_value = 0,
		},
		.query = sensor_g_focus_result,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCAL_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focal length",
			.minimum = OV5693_FOCAL_LENGTH_DEFAULT,
			.maximum = OV5693_FOCAL_LENGTH_DEFAULT,
			.step = 0x01,
			.default_value = OV5693_FOCAL_LENGTH_DEFAULT,
			.flags = 0,
		},
		.query = ov5693_g_focal,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number",
			.minimum = OV5693_F_NUMBER_DEFAULT,
			.maximum = OV5693_F_NUMBER_DEFAULT,
			.step = 0x01,
			.default_value = OV5693_F_NUMBER_DEFAULT,
			.flags = 0,
		},
		.query = ov5693_g_fnumber,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_RANGE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number range",
			.minimum = OV5693_F_NUMBER_RANGE,
			.maximum =  OV5693_F_NUMBER_RANGE,
			.step = 0x01,
			.default_value = OV5693_F_NUMBER_RANGE,
			.flags = 0,
		},
		.query = ov5693_g_fnumber_range,
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

#define N_CONTROLS (ARRAY_SIZE(ov5693_controls))

static struct ov5693_control *ov5693_find_control(__u32 id)
{
	int i;

	for (i = 0; i < N_CONTROLS; i++) {
		if (ov5693_controls[i].qc.id == id)
			return &ov5693_controls[i];
	}
	return NULL;
}

//FW_BSP++
static ssize_t isp_switch_name(struct switch_dev *sdev, char *buf)
{
	if (SPI_ENABLE==0) {
		printk("version_in_isp:%x\n", version_num_in_isp);

		return sprintf(buf, "%x\n", version_num_in_isp);
	} else {
		printk("version_in_bin:%x\n", version_num_in_bin);

		return sprintf(buf, "%x\n", version_num_in_bin);
	}
}

int get_fw_version_in_bin()
{
	u8 *pbootBuf = pIspFW_g;

	struct file *fp = NULL;
	mm_segment_t old_fs;
	struct inode *inode;	
	int bootbin_size = 0;
	int i, ret = 0;
	u8 bin_file_header[BIN_FILE_HEADER_SIZE];
	


	if (FW_BIN_FILE==false) {
		return 0;
	} else {
	
		for (i=0; i < BIN_FILE_HEADER_SIZE; i++)
		{
			bin_file_header[i] = pbootBuf[bootbin_size_g - BIN_FILE_HEADER_SIZE + i];
		}
		version_num_in_bin = (bin_file_header[30] << 16) | (bin_file_header[29] << 8) | bin_file_header[28];
		printk("%s: version_in_bin: %x\n", __FUNCTION__, version_num_in_bin);

		return 0;
	}
}

//FW_BSP--

static int ov5693_detect(struct ov5693_device *dev, struct i2c_client *client)
{
	struct i2c_adapter *adapter = client->adapter;
	int ret = 0, i = 0, status;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s: i2c error", __func__);
		return -ENODEV;
	}

	if (sensorid!=0x5690) {

		if (SPI_ENABLE==1) {
			if (FW_BIN_FILE==true) {
				if (build_version!=1) {
					if(g_is_calibration){
						status = EXISP_LoadCode(0x02, g_is_calibration, pIspFW_g, pCalibOpt_g, p3acali_g, pLsc_g, pLscdq_g, &dev->sd); //rear camera: 0, front camera: 1, calibration: 2
					}
					else {
						status = EXISP_LoadCode(0x00, g_is_calibration, pIspFW_g, pCalibOpt_g, p3acali_g, pLsc_g, pLscdq_g, &dev->sd); //rear camera: 0, front camera: 1, calibration: 2
					}
					fw_update_status = ICATCH_FW_UPDATE_SUCCESS;
					if(g_is_calibration){
						sensor_write_reg(client, 0x1306, 0x02); /* Set which FW to be loaded */ 
					} else {
						sensor_write_reg(client, 0x1306, 0x00); /* Set which FW to be loaded */ 
					}
				}

				printk("%s: Read Sensor ID again\n", __func__);
				msleep(100);
				for (i = 0; i<10; i++) {
					ret = sensor_read_reg(client, 0x72C6, &retvalue_l); //low byte of back sensor ID
					ret = sensor_read_reg(client, 0x72C7, &retvalue_h); //high byte of back sensor ID
					sensorid = ((retvalue_h<<8)|(retvalue_l))&0xffff;
					printk("%s Sensor ID = 0x%x\n", __func__, sensorid);
					if (sensorid==0x5690) {
						break;
					}
					msleep(5);
				}

			}
		} else {
			printk("%s: Read Sensor ID again\n", __func__);
			msleep(100);
			for (i = 0; i<10; i++) {
				ret = sensor_read_reg(client, 0x72C6, &retvalue_l); //low byte of back sensor ID
				ret = sensor_read_reg(client, 0x72C7, &retvalue_h); //high byte of back sensor ID
				sensorid = ((retvalue_h<<8)|(retvalue_l))&0xffff;
				printk("%s Sensor ID = 0x%x\n", __func__, sensorid);
				if (sensorid==0x5690) {
					break;
				}
				msleep(5);
			}
		}
	}

	
	if(sensorid==0x5690) {
		ATD_ov5693_status = 1;
		//FW_BSP++
		if (SPI_ENABLE==0) {
			version_num_in_isp = get_fw_version_in_isp(&dev->sd);
		}else {			
			get_fw_version_in_bin();
		}
		//FW_BSP--
	}else{
		ATD_ov5693_status = 0;
		printk("%s Sensor ID is not match\n", __func__);
	}

	return 0;
}

static int
ov5693_s_config(struct v4l2_subdev *sd, int irq, void *platform_data)
{
	struct ov5693_device *dev = to_ov5693_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	struct file *fp = NULL;
	mm_segment_t old_fs;
	struct inode *inode;	
	int status, byte_count= 0;
	UINT32 size_CalibOpt=0, size_3acali=0, size_Lsc=0, size_Lscdq=0;

	if (NULL == platform_data)
		return -ENODEV;

	dev->platform_data =
	    (struct camera_sensor_platform_data *)platform_data;

	if (dev->platform_data->platform_init) {
		ret = dev->platform_data->platform_init(client);
		if (ret) {
			dev_err(&client->dev, "ov5693 platform init err\n");
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
	} else  {
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
		} else  {
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
						byte_count= 0;
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
	
	ret = ov5693_s_power(sd, 1);
	if (ret) {
		dev_err(&client->dev, "ov5693 power-up err\n");
		//goto fail_detect;
	}

	/* config & detect sensor */
	ret = ov5693_detect(dev, client);
	if (ret) {
		dev_err(&client->dev, "ov5693_detect err s_config.\n");
		//goto fail_detect;
	}

//FW_BSP++
	ISP_sdev.name = ISP_SDEV_NAME;
	ISP_sdev.print_name = isp_switch_name;
	//ISP_sdev.print_state = isp_switch_state;
	if(switch_dev_register(&ISP_sdev) < 0){
		pr_err("switch_dev_register for camera ISP failed!\n");
	}
	switch_set_state(&ISP_sdev, 0);	
//FW_BSP--

	ret = dev->platform_data->csi_cfg(sd, 1);
	if (ret)
		goto fail_csi_cfg;

	ret = ov5693_s_power(sd, 0);
	if (ret) {
		dev_err(&client->dev, "ov5693 power down err\n");
		return ret;
	}

	dev->run_mode = CI_MODE_PREVIEW;
	dev->last_run_mode = CI_MODE_PREVIEW;

	return 0;

fail_csi_cfg:
	dev->platform_data->csi_cfg(sd, 0);
//fail_detect:
	ov5693_s_power(sd, 0);
	dev_err(&client->dev, "sensor power-gating failed\n");
	return ret;
}

static int ov5693_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	struct ov5693_control *ctrl = ov5693_find_control(qc->id);

	if (ctrl == NULL)
		return -EINVAL;
	*qc = ctrl->qc;
	return 0;
}

static int ov5693_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ov5693_control *octrl = ov5693_find_control(ctrl->id);
	int ret;

	if (octrl == NULL)
		return -EINVAL;

	ret = octrl->query(sd, &ctrl->value);
	if (ret < 0)
		return ret;

	return 0;
}

static int ov5693_s_ctrl_old(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ov5693_control *octrl = ov5693_find_control(ctrl->id);
	int ret;

	if (!octrl || !octrl->tweak)
		return -EINVAL;

	ret = octrl->tweak(sd, ctrl->value);
	if (ret < 0)
		return ret;

	return 0;
}

static int ov5693_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5693_device *dev = to_ov5693_sensor(sd);
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
ov5693_enum_framesizes(struct v4l2_subdev *sd, struct v4l2_frmsizeenum *fsize)
{
	unsigned int index = fsize->index;

	if (index >= N_RES)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = ov5693_res[index].width;
	fsize->discrete.height = ov5693_res[index].height;

	return 0;
}

static int ov5693_enum_frameintervals(struct v4l2_subdev *sd,
				       struct v4l2_frmivalenum *fival)
{
	unsigned int index = fival->index;
	int i;

	if (index >= N_RES)
		return -EINVAL;

	/* find out the first equal or bigger size */
	for (i = 0; i < N_RES; i++) {
		if ((ov5693_res[i].width >= fival->width) &&
		    (ov5693_res[i].height >= fival->height))
			break;
	}
	if (i == N_RES)
		i--;

	index = i;

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = 1;
	fival->discrete.denominator = ov5693_res[index].fps;

	return 0;
}

static int
ov5693_g_chip_ident(struct v4l2_subdev *sd, struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_OV5693, 0);
}

static int ov5693_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= MAX_FMTS)
		return -EINVAL;
	code->code = V4L2_MBUS_FMT_SGRBG10_1X10;

	return 0;
}

static int ov5693_enum_frame_size(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh,
	struct v4l2_subdev_frame_size_enum *fse)
{

	unsigned int index = fse->index;


	if (index >= N_RES)
		return -EINVAL;

	fse->min_width = ov5693_res[index].width;
	fse->min_height = ov5693_res[index].height;
	fse->max_width = ov5693_res[index].width;
	fse->max_height = ov5693_res[index].height;

	return 0;
}

static struct v4l2_mbus_framefmt *
__ov5693_get_pad_format(struct ov5693_device *sensor,
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
ov5693_get_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct ov5693_device *snr = to_ov5693_sensor(sd);
	struct v4l2_mbus_framefmt *format =
			__ov5693_get_pad_format(snr, fh, fmt->pad, fmt->which);

	if (format == NULL)
		return -EINVAL;
	fmt->format = *format;

	return 0;
}

static int
ov5693_set_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct ov5693_device *snr = to_ov5693_sensor(sd);
	struct v4l2_mbus_framefmt *format =
			__ov5693_get_pad_format(snr, fh, fmt->pad, fmt->which);

	if (format == NULL)
		return -EINVAL;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		snr->format = fmt->format;

	return 0;
}

static int ov5693_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov5693_device *snr = container_of(
		ctrl->handler, struct ov5693_device, ctrl_handler);

	snr->run_mode = ctrl->val;

	return 0;
}

static int ov5693_g_skip_frames(struct v4l2_subdev *sd, u32 *frames)
{
	int index;
	struct ov5693_device *snr = to_ov5693_sensor(sd);

	if (frames == NULL)
		return -EINVAL;

	for (index = 0; index < N_RES; index++) {
		if (ov5693_res[index].res == snr->res)
			break;
	}

	if (index >= N_RES)
		return -EINVAL;

	*frames = ov5693_res[index].skip_frames;

	return 0;
}
static const struct v4l2_subdev_video_ops ov5693_video_ops = {
	.try_mbus_fmt = ov5693_try_mbus_fmt,
	.s_mbus_fmt = ov5693_set_mbus_fmt,
	.g_mbus_fmt = ov5693_get_mbus_fmt,
	.s_stream = ov5693_s_stream,
	.enum_framesizes = ov5693_enum_framesizes,
	.enum_frameintervals = ov5693_enum_frameintervals,
};

static struct v4l2_subdev_sensor_ops ov5693_sensor_ops = {
	.g_skip_frames	= ov5693_g_skip_frames,
};

static const struct v4l2_subdev_core_ops ov5693_core_ops = {
	.g_chip_ident = ov5693_g_chip_ident,
	.queryctrl = ov5693_queryctrl,
	.g_ctrl = ov5693_g_ctrl,
	.s_ctrl = ov5693_s_ctrl_old,
	.s_power = ov5693_s_power,
};

static const struct v4l2_subdev_pad_ops ov5693_pad_ops = {
	.enum_mbus_code = ov5693_enum_mbus_code,
	.enum_frame_size = ov5693_enum_frame_size,
	.get_fmt = ov5693_get_pad_format,
	.set_fmt = ov5693_set_pad_format,
};

static const struct v4l2_subdev_ops ov5693_ops = {
	.core = &ov5693_core_ops,
	.video = &ov5693_video_ops,
	.pad = &ov5693_pad_ops,
	.sensor = &ov5693_sensor_ops,
};

static const struct media_entity_operations ov5693_entity_ops;


static int ov5693_remove(struct i2c_client *client)
{
	struct ov5693_device *dev;
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	dev = container_of(sd, struct ov5693_device, sd);
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
	.s_ctrl = ov5693_s_ctrl,
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

static int ov5693_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	struct ov5693_device *dev;
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
	dev->sensor_i2c_attribute.attrs = ov5693_attributes;

	// Register sysfs hooks 
	ret = sysfs_create_group(&client->dev.kobj, &dev->sensor_i2c_attribute);
	if (ret) {
		dev_err(&client->dev, "Not able to create the sysfs\n");
		return ret;
	}
	//Add for ATD read camera status---

	v4l2_i2c_subdev_init(&dev->sd, client, &ov5693_ops);
	if (client->dev.platform_data) {
		ret = ov5693_s_config(&dev->sd, client->irq,
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
		ov5693_remove(client);
		return ret;
	}

	dev->run_mode = v4l2_ctrl_new_custom(&dev->ctrl_handler,
					     &ctrl_run_mode, NULL);

	if (dev->ctrl_handler.error) {
		ov5693_remove(client);
		return dev->ctrl_handler.error;
	}

	ret = media_entity_init(&dev->sd.entity, 1, &dev->pad, 0);
	if (ret) {
		ov5693_remove(client);
		return ret;
	}

	/* set res index to be invalid */
	dev->res = -1;
	fw_sd = &dev->sd;

	return 0;
}


MODULE_DEVICE_TABLE(i2c, ov5693_id);

static struct i2c_driver ov5693_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "ov5693"
	},
	.probe = ov5693_probe,
	.remove = __exit_p(ov5693_remove),
	.id_table = ov5693_id,
};

static __init int init_ov5693(void)
{
	return i2c_add_driver(&ov5693_driver);
}

static __exit void exit_ov5693(void)
{
	i2c_del_driver(&ov5693_driver);
}

module_init(init_ov5693);
module_exit(exit_ov5693);

MODULE_AUTHOR("ASUS SW3");
MODULE_LICENSE("GPL");

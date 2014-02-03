/*
 * Support for mt9m114 Camera Sensor.
 *
 * Copyright (c) 2010 Intel Corporation. All Rights Reserved.
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

#ifndef __MT9M114_H__
#define __MT9M114_H__

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/spinlock.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <linux/v4l2-mediabus.h>
#include <media/media-entity.h>
#include <linux/atomisp_platform.h>
#include <linux/atomisp.h>
//FW_BSP+++
#include <linux/HWVersion.h>
extern int Read_HW_ID(void);
extern int Read_PROJ_ID(void);
static unsigned int HW_ID = 0xFF;
static unsigned int PROJECT_ID = 0xFF;
static unsigned int SPI_ENABLE = 0;
//FW_BSP---


#define V4L2_IDENT_MT9M114 0001 //Peter, what is this?

#define MSG_LEN_OFFSET		2
#define MAX_FMTS		1

//parameter define+++
#define SENSOR_ISO_AUTO		0
#define SENSOR_ISO_50		1
#define SENSOR_ISO_100		2
#define SENSOR_ISO_200		3
#define SENSOR_ISO_400		4
#define SENSOR_ISO_800		5
//parameter define---

//i2c r/w +++
#define SENSOR_WAIT_MS          0 /* special number to indicate this is wait time require */
#define SENSOR_TABLE_END        1 /* special number to indicate this is end of table */
#define SENSOR_BYTE_WRITE       2
#define SENSOR_WORD_WRITE       3
#define SENSOR_MASK_BYTE_WRITE  4
#define SENSOR_MASK_WORD_WRITE  5
#define SEQ_WRITE_START         6
#define SEQ_WRITE_END           7

#define SENSOR_MAX_RETRIES      3 /* max counter for retry I2C access */

struct sensor_reg {
	u16 addr;
	u16 val;
};
//i2c r/w --

//TODO: need to modify
#define MT9M114_FOCAL_LENGTH_NUM	208	/*2.08mm*/
#define MT9M114_FOCAL_LENGTH_DEM	100
#define MT9M114_F_NUMBER_DEFAULT_NUM	24
#define MT9M114_F_NUMBER_DEM	10

/*
 * focal length bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define MT9M114_FOCAL_LENGTH_DEFAULT 0xD00064

/*
 * current f-number bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define MT9M114_F_NUMBER_DEFAULT 0x18000a

/*
 * f-number range bits definition:
 * bits 31-24: max f-number numerator
 * bits 23-16: max f-number denominator
 * bits 15-8: min f-number numerator
 * bits 7-0: min f-number denominator
 */
#define MT9M114_F_NUMBER_RANGE 0x180a180a

//TODO: add more resolution later
/* Supported resolutions */
enum {
	MT9M114_RES_QCIF,
	MT9M114_RES_QVGA,
	MT9M114_RES_VGA,
	//MT9M114_RES_480P,
	MT9M114_RES_720P,
	MT9M114_RES_960P,
};
#define MT9M114_RES_960P_SIZE_H		1280
#define MT9M114_RES_960P_SIZE_V		960
#define MT9M114_RES_720P_SIZE_H		1280
#define MT9M114_RES_720P_SIZE_V		720
#define MT9M114_RES_480P_SIZE_H		768
#define MT9M114_RES_480P_SIZE_V		480
#define MT9M114_RES_VGA_SIZE_H		640
#define MT9M114_RES_VGA_SIZE_V		480
#define MT9M114_RES_QVGA_SIZE_H		320
#define MT9M114_RES_QVGA_SIZE_V		240
#define MT9M114_RES_QCIF_SIZE_H		176
#define MT9M114_RES_QCIF_SIZE_V		144


//FW_BSP++
enum iCatch_fw_update_status{
	ICATCH_FW_NO_CMD,
	ICATCH_FW_IS_BURNING,
	ICATCH_FW_UPDATE_SUCCESS,
	ICATCH_FW_UPDATE_FAILED,
};
static enum iCatch_fw_update_status fw_update_status = ICATCH_FW_NO_CMD;
//FW_BSP--


struct mt9m114_device {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;

	struct camera_sensor_platform_data *platform_data;
	int real_model_id;
	unsigned int res;
	int last_run_mode;

	struct mutex input_lock; /* serialize sensor's ioctl */
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *run_mode;

	struct attribute_group sensor_i2c_attribute; //Add for ATD read camera status+++
};

struct mt9m114_format_struct {
	u8 *desc;
	u32 pixelformat;
	struct regval_list *regs;
};

struct mt9m114_res_struct {
	u8 *desc;
	int res;
	int width;
	int height;
	int fps;
	int skip_frames;
	int row_time;
	bool used;
	struct regval_list *regs;
};

struct mt9m114_control {
	struct v4l2_queryctrl qc;
	int (*query)(struct v4l2_subdev *sd, s32 *value);
	int (*tweak)(struct v4l2_subdev *sd, int value);
};

/*
 * Modes supported by the mt9m114 driver.
 * Please, keep them in ascending order.
 */
static struct mt9m114_res_struct mt9m114_res[] = {
	{
	.desc	= "QCIF",
	.res	= MT9M114_RES_QCIF,
	.width	= 176,
	.height	= 144,
	.fps	= 30,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 0,
	},
	{
	.desc	= "QVGA",
	.res	= MT9M114_RES_QVGA,
	.width	= 320,
	.height	= 240,
	.fps	= 30,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 0,
	},
	{
	.desc	= "VGA",
	.res	= MT9M114_RES_VGA,
	.width	= 640,
	.height	= 480,
	.fps	= 30,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 0,
	},
/*
	{
	.desc	= "480p",
	.res	= MT9M114_RES_480P,
	.width	= 768,
	.height	= 480,
	.fps	= 30,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 0,
	},
*/
	{
	.desc	= "720p",
	.res	= MT9M114_RES_720P,
	.width	= 1280,
	.height	= 720,
	.fps	= 30,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 0,
	},
	{
	.desc	= "960P",
	.res	= MT9M114_RES_960P,
	.width	= 1280,
	.height	= 960,
	.fps	= 30,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 0,
	},
};
#define N_RES (ARRAY_SIZE(mt9m114_res))

static const struct i2c_device_id mt9m114_id[] = {
	{"mt9m114", 0},
	{}
};

#endif

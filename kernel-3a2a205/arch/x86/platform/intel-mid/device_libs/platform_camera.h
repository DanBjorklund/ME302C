/*
 * platform_camera.h: CAMERA platform library header file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _PLATFORM_CAMERA_H_
#define _PLATFORM_CAMERA_H_

#include <linux/atomisp_platform.h>

//Peter++
#include <linux/HWVersion.h>
extern int Read_HW_ID(void);
extern int Read_PROJ_ID(void);
static unsigned int HW_ID = 0xFF;
static unsigned int PROJECT_ID = 0xFF;
static unsigned int SPI_ENABLE = 0;


extern int intel_scu_ipc_msic_vemmc1(int);
//Peter--

extern const struct intel_v4l2_subdev_id v4l2_ids[] __attribute__((weak));

/* MFLD iCDK camera sensor GPIOs */

//Peter++
#define GP_CAMERA_ISP_POWER_1P2_EN	"ISP_1P2_EN" //iCatch 1.2V powwer enable pin, GPIO: 111
#define GP_CAMERA_ISP_RESET		"ISP_RST_N" //iCatch reset pin, GPIO: 161
#define GP_CAMERA_ISP_SUSPEND		"ISP_SUSPEND" //iCatch suspend pin, GPIO: 162
#define GP_CAMERA_ISP_INT		"ISP_INT" //iCatch interrupt pin, GPIO: 163
#if 0
/* Obsolete pin, maybe used by old MFLD iCDK */
#define GP_CAMERA_0_POWER_DOWN          "cam0_vcm_2p8"
/* Camera VDD 1.8V Switch */
#define GP_CAMERA_1P8			"camera_on_n"
/* Camera0 Standby pin control */
#define GP_CAMERA_0_STANDBY		"camera_0_power"
#define GP_CAMERA_1_POWER_DOWN          "camera_1_power"
#define GP_CAMERA_0_RESET               "camera_0_reset"
#define GP_CAMERA_1_RESET               "camera_1_reset"
#endif
//Peter--
//Patrick++
#define GP_CAMERA_SPI_CLK "SPI_CLK"
#define GP_CAMERA_SPI_SS3 "SPI_SS3"
#define GP_CAMERA_SPI_SDO "SPI_SDO"
#define GP_AON_019	19
#define GP_AON_021	21
#define GP_AON_023	23

//Patrick--

extern int camera_sensor_gpio(int gpio, char *name, int dir, int value);
extern int camera_sensor_csi(struct v4l2_subdev *sd, u32 port,
			u32 lanes, u32 format, u32 bayer_order, int flag);
//ASUS_BSP+++, RAW patch
extern int camera_sensor_csi_2(struct v4l2_subdev *sd, u32 port,
			u32 lanes, u32 format, u32 bayer_order,
			u32 raw_format, u32 raw_bayer_order, int flag);
//ASUS_BSP---, RAW patch
extern void intel_register_i2c_camera_device(
				struct sfi_device_table_entry *pentry,
				struct devs_id *dev);
#endif

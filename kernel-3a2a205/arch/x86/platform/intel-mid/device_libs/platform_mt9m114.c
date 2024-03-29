/*
 * platform_mt9m114.c: MT9M114 with iCatch 7002A ISP platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/atomisp_platform.h>
#include <asm/intel-mid.h>
#include <asm/intel_scu_ipcutil.h>
#include <media/v4l2-subdev.h>
#include <linux/regulator/consumer.h>
#include "platform_camera.h"
#include "platform_mt9m114.h"
//FW_BSP++
#include <linux/proc_fs.h>	//ASUS_BSP+++
#include <linux/lnw_gpio.h>

#define SPI_magic_number 49
#define EEPROM_magic_number 50

static int camera_SPI_1_CLK;
static int camera_SPI_1_SS3;
static int camera_SPI_1_SDO;
//FW_BSP--

#define VPROG1_VAL 1800000
#define VPROG2_VAL 2800000

static int camera_power_1p2_en;
static int camera_reset;
static int camera_suspend;
static int camera_vprog1_on;
static int camera_vprog2_on;

static struct regulator *vprog1_reg;
static struct regulator *vprog2_reg;

static char DEFAULT_SPI_FILE_WITH_PATH[] = "/system/etc/camera_spi_init.txt";
static char EXTERNAL_SPI_FILE_WITH_PATH[] = "/data/media/0/camera_spi_init.txt";
static char *SPI_FILE_WITH_PATH = EXTERNAL_SPI_FILE_WITH_PATH;




/*
 * ME302C Camera ISP - MT9M114 with iCatch 7002A ISP platform data
 */

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
				printk("Read SPI init parameter\n");
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


static int mt9m114_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	return 0;
}

static int mt9m114_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200; //Intel just can support 19.2MHz/9.6MHz/4.8MHz 
	int ret = 0;
	v4l2_err(sd, "%s: ++\n",__func__);
	ret = intel_scu_ipc_osc_clk(OSC_CLK_CAM0, flag ? clock_khz : 0);
	return ret;
}

static int mt9m114_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int reg_err;
	int ret, SPI_ret=0;
	printk("%s: ++\n",__func__);


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
	
	//printk("HW ID:%d\n", HW_ID);
	switch (HW_ID){
		case HW_ID_SR1:
		case HW_ID_SR2:
			if (camera_power_1p2_en < 0) {
				ret = camera_sensor_gpio(-1, GP_CAMERA_ISP_POWER_1P2_EN,
							 GPIOF_DIR_OUT, 0);
				if (ret < 0){
					printk("%s not available.\n", GP_CAMERA_ISP_POWER_1P2_EN);
					return ret;
				}
				camera_power_1p2_en = ret;
			}
			if (camera_reset < 0) {
				ret = camera_sensor_gpio(-1, GP_CAMERA_ISP_RESET,
							 GPIOF_DIR_OUT, 0);
				if (ret < 0){
					printk("%s not available.\n", GP_CAMERA_ISP_RESET);
					return ret;
				}
				camera_reset = ret;
			}
			if (camera_suspend < 0) {
				ret = camera_sensor_gpio(-1, GP_CAMERA_ISP_SUSPEND,
							GPIOF_DIR_OUT, 0);
				if (ret < 0){
					printk("%s not available.\n", GP_CAMERA_ISP_SUSPEND);
					return ret;
				}
				camera_suspend = ret;
			}
			break;
		case HW_ID_ER:
		case HW_ID_PR:
		case HW_ID_MP:
		default:
			printk("@@@@@HW_ID is unknow:%d, use SR2 setting\n", HW_ID);
			if (camera_power_1p2_en < 0) {
			ret = camera_sensor_gpio(111, GP_CAMERA_ISP_POWER_1P2_EN,
							 GPIOF_DIR_OUT, 0);
				if (ret < 0){
					printk("%s not available.\n", GP_CAMERA_ISP_POWER_1P2_EN);
					return ret;
				}
				camera_power_1p2_en = 111;
			}
			if (camera_reset < 0) {
				ret = camera_sensor_gpio(161, GP_CAMERA_ISP_RESET,
							 GPIOF_DIR_OUT, 0);
				if (ret < 0){
					printk("%s not available.\n", GP_CAMERA_ISP_RESET);
					return ret;
				}
				camera_reset = 161;
			}
			if (camera_suspend < 0) {
				ret = camera_sensor_gpio(162, GP_CAMERA_ISP_SUSPEND,
							GPIOF_DIR_OUT, 0);
				if (ret < 0){
					printk("%s not available.\n", GP_CAMERA_ISP_SUSPEND);
					return ret;
				}
				camera_suspend = 162;
			}
			break;
	}

	printk("<<1p2_en:%d, reset:%d, suspend:%d, flag:%d\n", camera_power_1p2_en, camera_reset, camera_suspend, flag);
	if (flag){
		//pull low reset first
		if (camera_reset >= 0){
			gpio_set_value(camera_reset, 0);
			printk("<<< camera_reset = 0\n");
			msleep(1);
		}

		//turn on DVDD power 1.2V
		if (camera_power_1p2_en >= 0){
			gpio_set_value(camera_power_1p2_en, 1);
			printk("<<< DVDD 1.2V = 1\n");
			msleep(1);
		}

		//turn on power VDD_SEN VDD_HOST 1.8V
		if (!camera_vprog1_on) {
			camera_vprog1_on = 1;
			reg_err = regulator_enable(vprog1_reg);
			if (reg_err) {
				printk(KERN_ALERT "Failed to enable regulator vprog1\n");
				return reg_err;
			}
			printk("<<< VDD_SEN VDD_HOST 1.8V = 1\n");
			msleep(10);
		}

		//turn on power AVDD 2.8V
		if (!camera_vprog2_on) {
			camera_vprog2_on = 1;
			reg_err = regulator_enable(vprog2_reg);
			if (reg_err) {
				printk(KERN_ALERT "Failed to enable regulator vprog2\n");
				return reg_err;
			}
			printk("<<< AVDD 2.8V = 1\n");
			msleep(10);
		}

		//turn on MCLK
		mt9m114_flisclk_ctrl(sd, 1);
		msleep(1); //need wait 16 clk cycle

//FW_BSP++		
		if (SPI_ENABLE==0) {
			//Pull high suspend to load fw from SPI
			if (camera_suspend >= 0){
				gpio_set_value(camera_suspend, 1);
				printk("<<< suspend = 1, load fw\n");
			}

			//Reset control
			if (camera_reset >= 0){
				gpio_set_value(camera_reset, 1);
				printk("<<< reset = 1\n");
				msleep(6); //wait 6ms
			}							
		} else {
			//Pull low suspend to load fw from host
			if (camera_suspend >= 0){
				gpio_set_value(camera_suspend, 0);
				printk("<<< suspend = 0, load fw\n");
			}

			lnw_gpio_set_alt(GP_AON_019, LNW_GPIO);
			lnw_gpio_set_alt(GP_AON_021, LNW_GPIO);
			lnw_gpio_set_alt(GP_AON_023, LNW_GPIO);

			if (camera_SPI_1_SS3 < 0) {
				ret = camera_sensor_gpio(GP_AON_019, GP_CAMERA_SPI_SS3,
						 GPIOF_DIR_OUT, 0);
				if (ret < 0){
					printk("%s not available.\n", GP_CAMERA_SPI_SS3);
					return ret;
				}
				camera_SPI_1_SS3= GP_AON_019;
			}		

			if (camera_SPI_1_SDO < 0) {
				ret = camera_sensor_gpio(GP_AON_021, GP_CAMERA_SPI_SDO,
						 GPIOF_DIR_OUT, 0);
				if (ret < 0){
					printk("%s not available.\n", GP_CAMERA_SPI_SDO);
					return ret;
				}
				camera_SPI_1_SDO= GP_AON_021;
			}							


			if (camera_SPI_1_CLK < 0) {
			ret = camera_sensor_gpio(GP_AON_023, GP_CAMERA_SPI_CLK,
							 GPIOF_DIR_OUT, 1);
				if (ret < 0){
					printk("%s not available.\n", GP_CAMERA_SPI_CLK);
					return ret;
				}
				camera_SPI_1_CLK = GP_AON_023;
			}


			if (camera_SPI_1_SS3 >= 0){
				gpio_set_value(camera_SPI_1_SS3, 0);
				printk("<<< SPI SS3 = 0\n");
			}					


			if (camera_SPI_1_SDO >= 0){
				gpio_set_value(camera_SPI_1_SDO, 0);
				printk("<<< SPI SDO = 0\n");
			}	


			if (camera_SPI_1_CLK >= 0){
				gpio_set_value(camera_SPI_1_CLK, 1);
				printk("<<< SPI CLK = 1\n");
				msleep(6);
			}				

			//Reset control
			if (camera_reset >= 0){
				gpio_set_value(camera_reset, 1);
				printk("<<< reset = 1\n");
				msleep(6); //wait 6ms
			}

			if (camera_SPI_1_SS3 >= 0){
				gpio_free(camera_SPI_1_SS3);
				camera_SPI_1_SS3 = -1;
			}				


			if (camera_SPI_1_SDO >= 0){
				gpio_free(camera_SPI_1_SDO);
				camera_SPI_1_SDO = -1;
			}	
				

			if (camera_SPI_1_CLK >= 0){
				gpio_set_value(camera_SPI_1_CLK, 0);
				printk("<<< SPI CLK = 0\n");
				gpio_free(camera_SPI_1_CLK);
				camera_SPI_1_CLK = -1;
			}

			lnw_gpio_set_alt(GP_AON_019, LNW_ALT_1);
			lnw_gpio_set_alt(GP_AON_021, LNW_ALT_1);
			lnw_gpio_set_alt(GP_AON_023, LNW_ALT_1);		
		}
//FW_BSP--

		//Pull low suspend
		if (camera_suspend >= 0){
			gpio_set_value(camera_suspend, 0);
			printk("<<< suspend = 0\n");
		}
		msleep(10); //delay time for first i2c command
	}else{
		//pull low reset
		if (camera_reset >= 0){
			gpio_set_value(camera_reset, 0);
			printk("<<< reset = 0\n");
			gpio_free(camera_reset);
			camera_reset = -1;
		}

		//turn off MCLK
		mt9m114_flisclk_ctrl(sd, 0);

		//turn off power AVDD 2.8V
		if (camera_vprog2_on) {
			camera_vprog2_on = 0;
			reg_err = regulator_disable(vprog2_reg);
			if (reg_err) {
				printk(KERN_ALERT "Failed to disable regulator vprog2\n");
				return reg_err;
			}
			printk("<<< AVDD 2.8V = 0\n");
			msleep(10);
		}

		lnw_gpio_set_alt(GP_AON_019, LNW_GPIO);
		lnw_gpio_set_alt(GP_AON_021, LNW_GPIO);
		
		if (camera_SPI_1_SS3 < 0) {
			ret = camera_sensor_gpio(GP_AON_019, GP_CAMERA_SPI_SS3,
					 GPIOF_DIR_OUT, 0);
			if (ret < 0){
				printk("%s not available.\n", GP_CAMERA_SPI_SS3);
				return ret;
			}
			camera_SPI_1_SS3= GP_AON_019;
		}			


		if (camera_SPI_1_SDO < 0) {
			ret = camera_sensor_gpio(GP_AON_021, GP_CAMERA_SPI_SDO,
					 GPIOF_DIR_OUT, 0);
			if (ret < 0){
				printk("%s not available.\n", GP_CAMERA_SPI_SDO);
				return ret;
			}
			camera_SPI_1_SDO= GP_AON_021;
		}			
		

		if (camera_SPI_1_SS3 >= 0){
			gpio_set_value(camera_SPI_1_SS3, 0);
			printk("<<< SPI SS3 = 0\n");
			gpio_free(camera_SPI_1_SS3);
			camera_SPI_1_SS3 = -1;
			mdelay(1);
		}		


		if (camera_SPI_1_SDO >= 0){
			gpio_set_value(camera_SPI_1_SDO, 0);
			printk("<<< SPI SDO = 0\n");
			gpio_free(camera_SPI_1_SDO);
			camera_SPI_1_SDO = -1;		
			mdelay(1);
		}					

		//turn off power VDD_SEN VDD_HOST 1.8V
		if (camera_vprog1_on) {
			camera_vprog1_on = 0;
			reg_err = regulator_disable(vprog1_reg);
			if (reg_err) {
				printk(KERN_ALERT "Failed to disable regulator vprog1\n");
				return reg_err;
			}
			printk("<<< VDD_SEN VDD_HOST 1.8V = 0\n");
			msleep(10);
		}

		//turn off DVDD power 1.2V
		if (camera_power_1p2_en >= 0){
			gpio_set_value(camera_power_1p2_en, 0);
			printk("<<< DVDD 1.2V = 0\n");
			gpio_free(camera_power_1p2_en);
			camera_power_1p2_en = -1;
		}
		msleep(1);

		//release suspend gpio
		if (camera_suspend >= 0){
			printk("<<< Release camera_suspend pin:%d\n", camera_suspend);
			gpio_free(camera_suspend);
			camera_suspend = -1;
		}
	}
	return 0;
}

static int mt9m114_csi_configure(struct v4l2_subdev *sd, int flag)
{
	/* soc sensor, there is no raw bayer order (set to -1) */
	//ASUS_BSP+++, RAW patch
	return camera_sensor_csi_2(sd, ATOMISP_CAMERA_PORT_PRIMARY, 4,
		ATOMISP_INPUT_FORMAT_YUV422_8, -1,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_bggr,
		flag);
	//ASUS_BSP---, RAW patch
}

static int mt9m114_platform_init(struct i2c_client *client)
{
	int ret;

	printk("%s: ++\n", __func__);
	//VPROG1 for VDD_HOST and VDD_SEN, 1.8V
	vprog1_reg = regulator_get(&client->dev, "vprog1");
	if (IS_ERR(vprog1_reg)) {
		dev_err(&client->dev, "vprog1 failed\n");
		return PTR_ERR(vprog1_reg);
	}
	ret = regulator_set_voltage(vprog1_reg, VPROG1_VAL, VPROG1_VAL);
	if (ret) {
		dev_err(&client->dev, "vprog1 set failed\n");
		regulator_put(vprog1_reg);
	}

	//VPROG2 for AVDD, 2.8V
	vprog2_reg = regulator_get(&client->dev, "vprog2");
	if (IS_ERR(vprog2_reg)) {
		dev_err(&client->dev, "vprog2 failed\n");
		return PTR_ERR(vprog2_reg);
	}
	ret = regulator_set_voltage(vprog2_reg, VPROG2_VAL, VPROG2_VAL);
	if (ret) {
		dev_err(&client->dev, "vprog2 set failed\n");
		regulator_put(vprog2_reg);
	}

	return ret;
}

static int mt9m114_platform_deinit(void)
{
	regulator_put(vprog1_reg);
	regulator_put(vprog2_reg);
}

static struct camera_sensor_platform_data mt9m114_sensor_platform_data = {
	.gpio_ctrl	 = mt9m114_gpio_ctrl,
	.flisclk_ctrl	 = mt9m114_flisclk_ctrl,
	.power_ctrl	 = mt9m114_power_ctrl,
	.csi_cfg	 = mt9m114_csi_configure,
	.platform_init   = mt9m114_platform_init,
	.platform_deinit = mt9m114_platform_deinit,
};

void *mt9m114_platform_data(void *info)
{
	camera_power_1p2_en = -1;
	camera_reset = -1;
	camera_suspend = -1;
	camera_SPI_1_CLK = -1;
	camera_SPI_1_SS3 = -1;
	camera_SPI_1_SDO = -1;
	return &mt9m114_sensor_platform_data;
}

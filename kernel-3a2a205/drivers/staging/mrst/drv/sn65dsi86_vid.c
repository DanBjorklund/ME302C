/*
 * Copyright (c)  2012 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicensen
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 */

#include "displays/sn65dsi86_vid.h"
#include "mdfld_dsi_dpi.h"
#include "mdfld_dsi_pkg_sender.h"
#include <linux/gpio.h>
#include "psb_drv.h"
#include <linux/lnw_gpio.h>
#include <asm/intel_scu_pmic.h>

#define SN65DSI86_PANEL_NAME	"OTC3108B-AUOB101"
#define TI_EDP_BRIDGE_I2C_ADAPTER	2
#define TI_EDP_BRIDGE_I2C_ADDR	0x2C

#define DP_BRDG_EN 63        //GP_AON_063
#define LCD_LOGIC_PWR_EN 108 //GP_CORE_012 = 12+96 (SOC gpio, panel logic power)
#define LCD_BL_EN 0x7F       //PANEL_EN(PMIC gpio, panel backlight)
#define EN_VDD_BL 0x7E       //BACKLIGHT_EN(PMIC, panel led driver)
#define PCB_ID3 112          //GP_CORE_016 = 16+96

#define PWM_ENABLE_GPIO 49
#define PWM_BASE_UNIT 0x80 //586 Hz

static struct i2c_client *sn65dsi86_client;
static int panel_id; // 0:Innolux(1920*1080), 1:BOE(1366*768)

union sst_pwmctrl_reg {
	struct {
		u32 pwmtd:8;
		u32 pwmbu:22;
		u32 pwmswupdate:1;
		u32 pwmenable:1;
	} part;
	u32 full;
};

static void __iomem *pwmctrl_mmio;

static int pwm_configure(int duty)
{
	union sst_pwmctrl_reg pwmctrl;

	/*Read the PWM register to make sure there is no pending
	*update.
	*/
	pwmctrl.full = readl(pwmctrl_mmio);

	/*check pwnswupdate bit */
	if (pwmctrl.part.pwmswupdate)
		return -EBUSY;
	pwmctrl.part.pwmswupdate = 0x1;
	pwmctrl.part.pwmbu = PWM_BASE_UNIT;
	pwmctrl.part.pwmtd = duty;
	writel(pwmctrl.full, pwmctrl_mmio);

	return 0;
}

static void pwm_enable() {
	union sst_pwmctrl_reg pwmctrl;

	lnw_gpio_set_alt(PWM_ENABLE_GPIO, LNW_ALT_2);
	/*Enable the PWM by setting PWM enable bit to 1 */
	pwmctrl.full = readl(pwmctrl_mmio);
	pwmctrl.part.pwmenable = 1;
	writel(pwmctrl.full, pwmctrl_mmio);
}

static void pwm_disable() {
	union sst_pwmctrl_reg pwmctrl;
	/*setting PWM enable bit to 0 */
	pwmctrl.full = readl(pwmctrl_mmio);
	pwmctrl.part.pwmenable = 0;
	writel(pwmctrl.full, pwmctrl_mmio);
}

struct work_struct irq_workq;

static int sn65dsi86_reg_write(struct i2c_client *client, u8 reg, u8 value)
{
	int r;
	u8 tx_data[] = {
		reg & 0xff,
		value & 0xff,
	};
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = tx_data,
			.len = ARRAY_SIZE(tx_data),
		},
	};

	r = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (r < 0) {
		dev_err(&client->dev, "%s: reg 0x%04x val 0x%08x error %d\n",
			__func__, reg, value, r);
		return r;
	}

	if (r < ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s: reg 0x%04x val 0x%08x msgs %d\n",
			__func__, reg, value, r);
		return -EAGAIN;
	}

	return 0;
}

/**
 * sn65dsi86_reg_read - Read DSI-eDP bridge register using I2C
 * @client: struct i2c_client to use
 * @reg: register address
 * @value: pointer for storing the value
 *
 * Returns 0 on success, or a negative error value.
 */
static int sn65dsi86_reg_read(struct i2c_client *client, u8 reg, u8 *value)
{
	int r;
	u8 tx_data[] = {
		reg & 0xff,
	};
	u8 rx_data[1];
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = tx_data,
			.len = ARRAY_SIZE(tx_data),
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = rx_data,
			.len = ARRAY_SIZE(rx_data),
		 },
	};

	r = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (r < 0) {
		dev_err(&client->dev, "%s: reg 0x%04x error %d\n", __func__,
			reg, r);
		return r;
	}

	if (r < ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s: reg 0x%04x msgs %d\n", __func__,
			reg, r);
		return -EAGAIN;
	}

	*value = rx_data[0];

	dev_dbg(&client->dev, "%s: reg 0x%04x value 0x%08x\n", __func__,
		reg, *value);

	return 0;
}

void sn65dsi86_csr_clear_all_status_bits()
{
	struct i2c_client *i2c = sn65dsi86_client;
	u32 status_value = 0;

	//Clear Status
	sn65dsi86_reg_write(i2c, 0xF0, 0xff);
	sn65dsi86_reg_write(i2c, 0xF1, 0xff);
	sn65dsi86_reg_write(i2c, 0xF2, 0xff);
	sn65dsi86_reg_write(i2c, 0xF3, 0xff);
	sn65dsi86_reg_write(i2c, 0xF4, 0xff);
	sn65dsi86_reg_write(i2c, 0xF5, 0xff);
	sn65dsi86_reg_write(i2c, 0xF6, 0xff);
	sn65dsi86_reg_write(i2c, 0xF7, 0xff);
	sn65dsi86_reg_write(i2c, 0xF8, 0xff);

}

void sn65dsi86_csr_read_irq()
{
	struct i2c_client *i2c = sn65dsi86_client;
	u32 irq_value_f0 = 0;
	u32 irq_value_f1 = 0;
//	u32 irq_value_f2 = 0;
//	u32 irq_value_f3 = 0;
	u32 irq_value_f4 = 0;
	u32 irq_value_f5 = 0;
	u32 irq_value_f6 = 0;
	u32 irq_value_f7 = 0;
	u32 irq_value_f8 = 0;

	//Read Status
	sn65dsi86_reg_read(i2c, 0xF0, &irq_value_f0);
	sn65dsi86_reg_read(i2c, 0xF1, &irq_value_f1);
//	sn65dsi86_reg_read(i2c, 0xF2, &irq_value_f2);
//	sn65dsi86_reg_read(i2c, 0xF3, &irq_value_f3);
	sn65dsi86_reg_read(i2c, 0xF4, &irq_value_f4);
	sn65dsi86_reg_read(i2c, 0xF5, &irq_value_f5);
	sn65dsi86_reg_read(i2c, 0xF6, &irq_value_f6);
	sn65dsi86_reg_read(i2c, 0xF7, &irq_value_f7);
	sn65dsi86_reg_read(i2c, 0xF8, &irq_value_f8);

	printk(KERN_INFO "IRQ [F0, F1, F4, F5, F6, F7, F8] = %x, %x, %x, %x, %x, %x, %x\n",
			irq_value_f0, irq_value_f1, irq_value_f4, irq_value_f5,
			irq_value_f6, irq_value_f7, irq_value_f8);

	//Clear Status
	sn65dsi86_reg_write(i2c, 0xF0, 0xff);
	sn65dsi86_reg_write(i2c, 0xF1, 0xff);
	sn65dsi86_reg_write(i2c, 0xF2, 0xff);
	sn65dsi86_reg_write(i2c, 0xF3, 0xff);
	sn65dsi86_reg_write(i2c, 0xF4, 0xff);
	sn65dsi86_reg_write(i2c, 0xF5, 0xff);
	sn65dsi86_reg_write(i2c, 0xF6, 0xff);
	sn65dsi86_reg_write(i2c, 0xF7, 0xff);
	sn65dsi86_reg_write(i2c, 0xF8, 0xff);
}

static void sn65dsi86_csr_read_irq_cycle(struct work_struct *work)
{
	struct i2c_client *i2c = sn65dsi86_client;
	u32 irq_value_f0 = 0;
	u32 irq_value_f1 = 0;
	u32 irq_value_f4 = 0;
	u32 irq_value_f5 = 0;
	u32 irq_value_f6 = 0;
	u32 irq_value_f7 = 0;
	u32 irq_value_f8 = 0;

	do {
		//Read Status
		sn65dsi86_reg_read(i2c, 0xF0, &irq_value_f0);
		sn65dsi86_reg_read(i2c, 0xF1, &irq_value_f1);
		sn65dsi86_reg_read(i2c, 0xF4, &irq_value_f4);
		sn65dsi86_reg_read(i2c, 0xF5, &irq_value_f5);
		sn65dsi86_reg_read(i2c, 0xF6, &irq_value_f6);
		sn65dsi86_reg_read(i2c, 0xF7, &irq_value_f7);
		sn65dsi86_reg_read(i2c, 0xF8, &irq_value_f8);

		if (irq_value_f0 || irq_value_f1 || irq_value_f4 || irq_value_f5 ||
			irq_value_f6 || irq_value_f7 || irq_value_f8)
		{
			printk(KERN_INFO "[DISPLAY] IRQ [F0, F1, F4, F5, F6, F7, F8] = %x, %x, %x, %x, %x, %x, %x\n",
					irq_value_f0, irq_value_f1, irq_value_f4, irq_value_f5,
					irq_value_f6, irq_value_f7, irq_value_f8);

			//Clear Status
			sn65dsi86_reg_write(i2c, 0xF0, 0xff);
			sn65dsi86_reg_write(i2c, 0xF1, 0xff);
			sn65dsi86_reg_write(i2c, 0xF2, 0xff);
			sn65dsi86_reg_write(i2c, 0xF3, 0xff);
			sn65dsi86_reg_write(i2c, 0xF4, 0xff);
			sn65dsi86_reg_write(i2c, 0xF5, 0xff);
			sn65dsi86_reg_write(i2c, 0xF6, 0xff);
			sn65dsi86_reg_write(i2c, 0xF7, 0xff);
			sn65dsi86_reg_write(i2c, 0xF8, 0xff);
		}
		msleep(1000);
	} while (1);
}

void sn65dsi86_csr_configure_boe(void)
{
	struct i2c_client *i2c = sn65dsi86_client;
	u32 pll_lock_value = 0;
	int timeout = 0;

	//printk(KERN_INFO "[DISPLAY] %s: Enter\n", __func__);

	sn65dsi86_reg_read(i2c, 0x08, &pll_lock_value);
	printk(KERN_INFO "[DISPLAY] TI BRIDGE DEVICE_REV = %x \n", pll_lock_value);

	//Soft Reset
	sn65dsi86_reg_write(i2c, 0x09, 0x01);
	//4 DSI Lanes - DSIA_Only
	//sn65dsi86_reg_write(i2c, 0x10, 0x26);
	sn65dsi86_reg_write(i2c, 0x10, 0x2e);  //3 lanes
	//DSIA Clock - 255MHz
	//sn65dsi86_reg_write(i2c, 0x12, 0x2b);

	//IRQ Enable
	sn65dsi86_reg_write(i2c, 0xE0, 0x01);
	//HPD Disable
	sn65dsi86_reg_write(i2c, 0x5C, 0x01);
	//Ench Frame, No Authen.
	sn65dsi86_reg_write(i2c, 0x5A, 0x04);

	//DP_18BPP_EN=1
	sn65dsi86_reg_write(i2c, 0x5B, 0x01);

	//1DP No SSC
	sn65dsi86_reg_write(i2c, 0x93, 0x14);
	//HBR 2.7Gbps
	sn65dsi86_reg_write(i2c, 0x94, 0x80);

//	------------------------------------------------------------------------------
	sn65dsi86_reg_write(i2c, 0x0D, 0x01);
	do {
		//read 0x0A until DP PLL locked
		sn65dsi86_reg_read(i2c, 0x0A, &pll_lock_value);
		printk(KERN_INFO "[DISPLAY] DP_PLL_LOCK = %x \n", pll_lock_value);
		usleep_range(1000, 1500);
		timeout++;

	} while (!(pll_lock_value & 0x80) && (timeout < 5)); //0x01
	if (timeout >= 5)
		printk(KERN_INFO "[DISPLAY] enable DisplayPort PLL faild\n");

	//ASSR

	//Post-Cursor2 0dB
	sn65dsi86_reg_write(i2c, 0x95, 0x00);
	//Start LInk Training.
	sn65dsi86_reg_write(i2c, 0x96, 0x0A);
	timeout = 0;
	do {
		//read 0x96 until ML_TX_MODE transition to Normal Mode (0x01)
		sn65dsi86_reg_read(i2c, 0x96, &pll_lock_value);
		printk(KERN_INFO "[DISPLAY] ML_TX_MODE = %x \n", pll_lock_value);
		usleep_range(1000, 10000); //1~10ms
		timeout++;

	} while ((pll_lock_value & 0xfe) && (timeout < 5)); //0x80
	if (timeout >= 5)
		printk(KERN_INFO "[DISPLAY] Train the DisplayPort Link faild\n");

	sn65dsi86_reg_read(i2c, 0x93, &pll_lock_value);
	printk(KERN_INFO "[DISPLAY] 0x93 = %x \n", pll_lock_value);
	sn65dsi86_reg_read(i2c, 0x94, &pll_lock_value);
	printk(KERN_INFO "[DISPLAY] 0x93 = %x \n", pll_lock_value);
	sn65dsi86_reg_read(i2c, 0x12, &pll_lock_value);
	printk(KERN_INFO "[DISPLAY] DSI-A Clock = %x \n", pll_lock_value);

	//Hact=1366
	sn65dsi86_reg_write(i2c, 0x20, 0x56);
	sn65dsi86_reg_write(i2c, 0x21, 0x05);
	sn65dsi86_reg_write(i2c, 0x22, 0x00);
	sn65dsi86_reg_write(i2c, 0x23, 0x00);
	//Vact=768
	sn65dsi86_reg_write(i2c, 0x24, 0x00);
	sn65dsi86_reg_write(i2c, 0x25, 0x03);
	//HSYNC=32 Negative
	sn65dsi86_reg_write(i2c, 0x2C, 0x20);
	sn65dsi86_reg_write(i2c, 0x2D, 0x80);
	//VSYNC=6 Negative
	sn65dsi86_reg_write(i2c, 0x30, 0x06);
	sn65dsi86_reg_write(i2c, 0x31, 0x80);
	//HBP=80
	sn65dsi86_reg_write(i2c, 0x34, 0x50);
	//VBP=13
	sn65dsi86_reg_write(i2c, 0x36, 0x0d);
	//HFP=48
	sn65dsi86_reg_write(i2c, 0x38, 0x30);
	//VFP=3
	sn65dsi86_reg_write(i2c, 0x3A, 0x03);
	//Color_Bar Disabled
	sn65dsi86_reg_write(i2c, 0x3C, 0x00);

	sn65dsi86_reg_write(i2c, 0x3D, 0x00);
	sn65dsi86_reg_write(i2c, 0x3E, 0x00);

	//printk(KERN_INFO "[DISPLAY] %s: Exit\n", __func__);
}

void sn65dsi86_csr_configure_innolux(void)
{
	struct i2c_client *i2c = sn65dsi86_client;
	u32 pll_lock_value = 0;
	int timeout = 0;

	//printk(KERN_INFO "[DISPLAY] %s: Enter\n", __func__);

	sn65dsi86_reg_read(i2c, 0x08, &pll_lock_value);
	printk(KERN_INFO "[DISPLAY] TI BRIDGE DEVICE_REV = %x \n", pll_lock_value);

	//Soft Reset
	sn65dsi86_reg_write(i2c, 0x09, 0x01);
	//4 DSI Lanes - DSIA_Only
	sn65dsi86_reg_write(i2c, 0x10, 0x26);
	//DSIA Clock - 445MHz
	sn65dsi86_reg_write(i2c, 0x12, 0x59);

	//IRQ Enable
	sn65dsi86_reg_write(i2c, 0xE0, 0x01);

	//HPD Disable
	sn65dsi86_reg_write(i2c, 0x5C, 0x01);
	//Ench Frame, No Authen.
	sn65dsi86_reg_write(i2c, 0x5A, 0x04);

	//2DP No SSC
	sn65dsi86_reg_write(i2c, 0x93, 0x24);
	//HBR 2.7Gbps
	sn65dsi86_reg_write(i2c, 0x94, 0x80);

//	------------------------------------------------------------------------------

	sn65dsi86_reg_write(i2c, 0x0D, 0x01);
	do {
		//read 0x0A until DP PLL locked
		sn65dsi86_reg_read(i2c, 0x0A, &pll_lock_value);
		printk(KERN_INFO "[DISPLAY] DP_PLL_LOCK = %x \n", pll_lock_value);
		usleep_range(1000, 1500);
		timeout++;

	} while (!(pll_lock_value & 0x80) && (timeout < 5)); //0x01
	if (timeout >= 5)
		printk(KERN_INFO "[DISPLAY] enable DisplayPort PLL faild\n");

	//ASSR

	//Post-Cursor2 0dB
	sn65dsi86_reg_write(i2c, 0x95, 0x00);
	//Start LInk Training.
	sn65dsi86_reg_write(i2c, 0x96, 0x0A);
	timeout = 0;
	do {
		//read 0x96 until ML_TX_MODE transition to Normal Mode (0x01)
		sn65dsi86_reg_read(i2c, 0x96, &pll_lock_value);
		printk(KERN_INFO "[DISPLAY] ML_TX_MODE = %x \n", pll_lock_value);
		usleep_range(1000, 10000); //1~10ms
		timeout++;

	} while ((pll_lock_value & 0xfe) && (timeout < 5)); //0x80
	if (timeout >= 5)
		printk(KERN_INFO "[DISPLAY] Train the DisplayPort Link faild\n");

	//Hact=1920
	sn65dsi86_reg_write(i2c, 0x20, 0x80);
	sn65dsi86_reg_write(i2c, 0x21, 0x07);
	//Vact=1080
	sn65dsi86_reg_write(i2c, 0x24, 0x38);
	sn65dsi86_reg_write(i2c, 0x25, 0x04);
	//HSYNC=32 Positive
	sn65dsi86_reg_write(i2c, 0x2C, 0x20);
	sn65dsi86_reg_write(i2c, 0x2D, 0x00);
	//VSYNC=5 Negative
	sn65dsi86_reg_write(i2c, 0x30, 0x05);
	sn65dsi86_reg_write(i2c, 0x31, 0x80);
	//HBP=80
	sn65dsi86_reg_write(i2c, 0x34, 0x50);
	//VBP=24
	sn65dsi86_reg_write(i2c, 0x36, 0x18);
	//HFP=48
	sn65dsi86_reg_write(i2c, 0x38, 0x30);
	//VFP=3
	sn65dsi86_reg_write(i2c, 0x3A, 0x03);

	//DP_18BPP_EN=0
	sn65dsi86_reg_write(i2c, 0x5B, 0x00);
	//Color_Bar Disabled
	sn65dsi86_reg_write(i2c, 0x3C, 0x00);

	//printk(KERN_INFO "[DISPLAY] %s: Exit\n", __func__);
}

void sn65dsi86_csr_video_stream_enable()
{
	struct i2c_client *i2c = sn65dsi86_client;

	sn65dsi86_reg_write(i2c, 0x5A, 0x0c);

}

static void sn65dsi86_vid_dsi_controller_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_context *hw_ctx = &dsi_config->dsi_hw_context;
	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);

	if (panel_id) {
		// BOE 1366*768
		dsi_config->lane_count = 3;
		dsi_config->bpp = 24;
		dsi_config->lane_config = MDFLD_DSI_DATA_LANE_3_1;

		hw_ctx->cck_div = 1;
		hw_ctx->pll_bypass_mode = 0;

		hw_ctx->mipi_control = 0x18;
		hw_ctx->intr_en = 0xffffffff;
		hw_ctx->hs_tx_timeout = 0xffffff;
		hw_ctx->lp_rx_timeout = 0xffffff;
		hw_ctx->turn_around_timeout = 0x1f;
		hw_ctx->device_reset_timer = 0xffff;
		hw_ctx->high_low_switch_count = 0x1d;
		hw_ctx->init_count = 0xf0;
		hw_ctx->eot_disable = 0x3;
		hw_ctx->lp_byteclk = 0x4;
		hw_ctx->clk_lane_switch_time_cnt = 0x1d000d;
		hw_ctx->dphy_param = 0x1d114715;

	} else {
		// Innolux 1920*1080
		dsi_config->lane_count = 4;
		dsi_config->bpp = 24;
		dsi_config->lane_config = MDFLD_DSI_DATA_LANE_4_0;

		hw_ctx->cck_div = 0;
		hw_ctx->pll_bypass_mode = 0;

		hw_ctx->mipi_control = 0x18;
		hw_ctx->intr_en = 0xffffffff;
		hw_ctx->hs_tx_timeout = 0xffffff;
		hw_ctx->lp_rx_timeout = 0xffffff;
		hw_ctx->turn_around_timeout = 0x1f;
		hw_ctx->device_reset_timer = 0xffff;
		hw_ctx->high_low_switch_count = 0x2c;
		hw_ctx->init_count = 0xf0;
		hw_ctx->eot_disable = 0x2;
		hw_ctx->lp_byteclk = 0x6;
		hw_ctx->clk_lane_switch_time_cnt = 0x2c0015; //0x270013
		hw_ctx->dphy_param = 0x301b741f;
	}

	/* Setup video mode format */
	hw_ctx->video_mode_format = 0xf;

	/* Set up func_prg, RGB888(0x200) MIPIA_DSI_FUNC_PRG_REG 0xb00c*/
	hw_ctx->dsi_func_prg = (0x200 | dsi_config->lane_count);

	/* Setup mipi port configuration */
	hw_ctx->mipi = PASS_FROM_SPHY_TO_AFE | dsi_config->lane_config;

}

static int sn65dsi86_vid_detect(struct mdfld_dsi_config *dsi_config)
{
	int status;
	struct drm_device *dev = dsi_config->dev;
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	u32 dpll_val, device_ready_val;
	int pipe = dsi_config->pipe;
	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);

	if (pipe == 0) {
		/*
		 * FIXME: WA to detect the panel connection status, and need to
		 * implement detection feature with get_power_mode DSI command.
		 */
		if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
			OSPM_UHB_FORCE_POWER_ON)) {
			DRM_ERROR("hw begin failed\n");
			return -EAGAIN;
		}

		dpll_val = REG_READ(regs->dpll_reg);
		device_ready_val = REG_READ(regs->device_ready_reg);
		if ((device_ready_val & DSI_DEVICE_READY) &&
		    (dpll_val & DPLL_VCO_ENABLE)) {
			dsi_config->dsi_hw_context.panel_on = true;
			psb_enable_vblank(dev, pipe);
		} else {
			dsi_config->dsi_hw_context.panel_on = false;
			DRM_INFO("%s: panel is not detected!\n", __func__);
		}

		status = MDFLD_DSI_PANEL_CONNECTED;
		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	} else {
		DRM_INFO("%s: do NOT support dual panel\n", __func__);
		status = MDFLD_DSI_PANEL_DISCONNECTED;
	}

	return status;
}

static int sn65dsi86_vid_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}
	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);

	/* LCD_BL_EN*/
	intel_scu_ipc_iowrite8(LCD_BL_EN, 0x1);
	usleep_range(200000, 250000);

	if (panel_id)
		sn65dsi86_csr_configure_boe();
	else
		sn65dsi86_csr_configure_innolux();

	/* Send TURN_ON packet */
	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,
					    MDFLD_DSI_DPI_SPK_TURN_ON);
	if (err) {
		DRM_ERROR("Failed to send turn on packet\n");
		return err;
	}

	sn65dsi86_csr_video_stream_enable();
	sn65dsi86_csr_clear_all_status_bits();

	// ti bridge interrupt enable
	//schedule_work(&irq_workq);

	/*PWM enable*/
	pwm_enable();
	usleep_range(10000, 15000);

	/*EN_VDD_BL*/
	intel_scu_ipc_iowrite8(EN_VDD_BL, 0x1);

	return 0;
}

static int sn65dsi86_vid_power_off(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}
	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);

	/*EN_VDD_BL*/
	intel_scu_ipc_iowrite8(EN_VDD_BL,0x0);

	/*PWM disable*/
	usleep_range(10000, 15000);
	pwm_disable();

	usleep_range(200000, 250000);

	/* Send SHUT_DOWN packet */
	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,
					    MDFLD_DSI_DPI_SPK_SHUT_DOWN);
	if (err) {
		DRM_ERROR("Failed to send turn off packet\n");
		return err;
	}
	usleep_range(10000, 15000);

	/* LCD_BL_EN*/
	intel_scu_ipc_iowrite8(LCD_BL_EN,0x0);
	usleep_range(1000, 3000);

	sn65dsi86_csr_clear_all_status_bits();

	/*LCD_LOGIC_PWR_EN*/
	if (gpio_direction_output(LCD_LOGIC_PWR_EN, 0))
		gpio_set_value_cansleep(LCD_LOGIC_PWR_EN, 0);
	usleep_range(10000, 15000);

	/*DP_BRDG_EN*/
	if (gpio_direction_output(DP_BRDG_EN, 0))
		gpio_set_value_cansleep(DP_BRDG_EN, 0);
	usleep_range(1000, 2000);

	return 0;
}

static int sn65dsi86_vid_reset(struct mdfld_dsi_config *dsi_config)
{
	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);

	/*DP_BRDG_EN*/
	if (gpio_direction_output(DP_BRDG_EN, 0))
		gpio_set_value_cansleep(DP_BRDG_EN, 0);  /*Pull MIPI Bridge EN pin to Low */
	usleep_range(1000, 1500);
	if (gpio_direction_output(DP_BRDG_EN, 1))
		gpio_set_value_cansleep(DP_BRDG_EN, 1);  /*Pull MIPI Bridge EN pin to High */

	/*LCD_LOGIC_PWR_EN*/
	if (gpio_direction_output(LCD_LOGIC_PWR_EN, 1))
		gpio_set_value_cansleep(LCD_LOGIC_PWR_EN, 1);
	usleep_range(1000, 10000);

	return 0;
}

#define PWM0DUTYCYCLE	0x67
#define DUTY_VALUE_MAX	0x63
#define BRIGHTNESS_LEVEL_MAX	100
#define BACKLIGHT_DUTY_FACTOR	0xff

static int sn65dsi86_vid_set_brightness(struct mdfld_dsi_config *dsi_config,
					 int level)
{
	int duty_val = 0;
	int ret = 0;

	duty_val = level*BACKLIGHT_DUTY_FACTOR/BRIGHTNESS_LEVEL_MAX;
	ret = pwm_configure(duty_val);

	if(ret)
		DRM_ERROR("[DISPLAY] pwm configure fail.\n");

	//DRM_INFO("brightness level is %d.( duty = 0x%x )\n", level, duty_val);

	return ret;
}

struct drm_display_mode *sn65dsi86_vid_get_config_mode(void)
{
	struct drm_display_mode *mode;

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;
	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);

	if (panel_id) {
		// BOE 1366*768
		mode->hdisplay = 1368;
		mode->hsync_start = mode->hdisplay + 48;	//sync offset
		mode->hsync_end = mode->hsync_start + 32;	//pulse width
		mode->htotal = mode->hdisplay + 160;		//blank

		mode->vdisplay = 768;
		mode->vsync_start = mode->vdisplay + 3;
		mode->vsync_end = mode->vsync_start + 6;
		mode->vtotal = mode->vdisplay + 22;

	} else {
		// Innolux 1920*1080
		mode->hdisplay = 1920;
		mode->hsync_start = mode->hdisplay + 64;	//sync offset
		mode->hsync_end = mode->hsync_start + 43;	//pulse width
		mode->htotal = mode->hdisplay + 214;		//blank

		mode->vdisplay = 1080;
		mode->vsync_start = mode->vdisplay + 4;
		mode->vsync_end = mode->vsync_start + 7;
		mode->vtotal = mode->vdisplay + 43;
	}

	mode->vrefresh = 60;
	mode->clock = mode->vrefresh * mode->vtotal * mode->htotal / 1000;
	mode->type |= DRM_MODE_TYPE_PREFERRED;
	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	return mode;
}

static void sn65dsi86_vid_get_panel_info(int pipe, struct panel_info *pi)
{
	pi->width_mm = 256;
	pi->height_mm = 144;
}

static int sn65dsi86_vid_gpio_init(void)
{
	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);

	gpio_request(LCD_LOGIC_PWR_EN, "LCD_LOGIC_PWR_EN");
	gpio_request(PWM_ENABLE_GPIO, "PWM_ENABLE_GPIO");
	gpio_request(PCB_ID3, "PCB_ID3");

	panel_id = gpio_get_value(PCB_ID3);
	DRM_INFO("[DISPLAY] Panel ID = %d\n", panel_id);

	return 0;
}

#define PWM0CLKDIV1 0x61
#define PWM0CLKDIV0 0x62

#define PWMCTRL_REG 0xffae9000
#define PWMCTRL_SIZE 0x80

static int sn65dsi86_vid_brightness_init(void)
{
	int ret = 0;
	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);

	pwmctrl_mmio = ioremap_nocache(PWMCTRL_REG, PWMCTRL_SIZE);

	lnw_gpio_set_alt(PWM_ENABLE_GPIO, LNW_ALT_2);

	return ret;
}

static int sn65dsi86_bridge_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	PSB_DEBUG_ENTRY("\n");
	printk(KERN_INFO "[DISPLAY] %s: Enter\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		DRM_ERROR("i2c_check_functionality() failed\n");
		return -ENODEV;
	}

	gpio_request(DP_BRDG_EN, "DP_BRDG_EN");

	sn65dsi86_client = client;

	INIT_WORK(&irq_workq, sn65dsi86_csr_read_irq_cycle);

	return 0;
}

static int sn65dsi86_bridge_remove(struct i2c_client *client)
{
	PSB_DEBUG_ENTRY("\n");
	printk(KERN_INFO "[DISPLAY] %s: Enter\n", __func__);

	gpio_free(DP_BRDG_EN);

	sn65dsi86_client = NULL;

	return 0;
}

static int sn65dsi86_hack_create_device(void)
{
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	struct i2c_board_info info = {
		.type = "i2c_disp_brig",
		.addr = TI_EDP_BRIDGE_I2C_ADDR,
	};

	printk(KERN_INFO "[DISPLAY] %s: Enter\n", __func__);

	adapter = i2c_get_adapter(TI_EDP_BRIDGE_I2C_ADAPTER);
	if (!adapter) {
		pr_err("%s: i2c_get_adapter(%d) failed\n", __func__,
			TI_EDP_BRIDGE_I2C_ADAPTER);
		return -EINVAL;
	}

	client = i2c_new_device(adapter, &info);
	if (!client) {
		pr_err("%s: i2c_new_device() failed\n", __func__);
		i2c_put_adapter(adapter);
		return -EINVAL;
	}

	return 0;
}

MODULE_DEVICE_TABLE(i2c, sn65dsi86_bridge_id);

static const struct i2c_device_id sn65dsi86_bridge_id[] = {
	{"i2c_disp_brig", 0},
	{}
};

static struct i2c_driver sn65dsi86_bridge_i2c_driver = {
	.driver = {
		.name = "i2c_disp_brig",
	},
	.id_table = sn65dsi86_bridge_id,
	.probe = sn65dsi86_bridge_probe,
	.remove = __devexit_p(sn65dsi86_bridge_remove),
};

void sn65dsi86_vid_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	int ret = 0;
	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);

	p_funcs->get_config_mode = sn65dsi86_vid_get_config_mode;
	p_funcs->get_panel_info = sn65dsi86_vid_get_panel_info;
	p_funcs->dsi_controller_init = sn65dsi86_vid_dsi_controller_init;
	p_funcs->detect = sn65dsi86_vid_detect;
	p_funcs->power_on = sn65dsi86_vid_power_on;
	p_funcs->power_off = sn65dsi86_vid_power_off;
	p_funcs->reset = sn65dsi86_vid_reset;
	p_funcs->set_brightness = sn65dsi86_vid_set_brightness;
	ret = sn65dsi86_vid_gpio_init();
	if (ret)
		DRM_ERROR("Faild to request GPIO for SN65DSI86 bridge\n");

	ret = sn65dsi86_vid_brightness_init();
	if (ret)
		DRM_ERROR("Faild to initilize PWM of MSCI\n");

	ret = sn65dsi86_hack_create_device();
	if (ret)
		DRM_ERROR("sn65dsi86_hack_create_device() faild\n");

	ret = i2c_add_driver(&sn65dsi86_bridge_i2c_driver);
	if (ret)
		DRM_ERROR("add bridge I2C driver faild\n");

}

static int sn65dsi86_vid_lcd_probe(struct platform_device *pdev)
{
	int ret = 0;

	DRM_INFO("%s: SN65DSI86 bridge detected\n", __func__);
	intel_mid_panel_register(sn65dsi86_vid_init);


	return 0;
}

static struct platform_driver sn65dsi86_vid_lcd_driver = {
	.probe	= sn65dsi86_vid_lcd_probe,
	.driver	= {
		.name	= SN65DSI86_PANEL_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init sn65dsi86_vid_lcd_init(void)
{
	DRM_INFO("%s\n", __func__);
	return platform_driver_register(&sn65dsi86_vid_lcd_driver);;
}
module_init(sn65dsi86_vid_lcd_init);

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

#include "displays/b101uan017_vid.h"
#include "mdfld_dsi_dpi.h"
#include "mdfld_dsi_pkg_sender.h"
#include <linux/gpio.h>
#include "psb_drv.h"
#include <linux/lnw_gpio.h>
#include <asm/intel_scu_pmic.h>
#include "asus_panel_id.h"

#define B101UAN017_PANEL_NAME	"OTC3108B-AUOB101"
#define B101UAN017_DEBUG 1
/*
 * GPIO pin definition
 */

#define LCD_LOGIC_PWR_EN 108  //GP_CORE_012 = 12+96 (SOC gpio, panel logic power)
#define LCD_BL_EN 0x7F       //PANEL_EN(PMIC gpio, panel backlight)
#define EN_VDD_BL 0x7E       //BACKLIGHT_EN(PMIC, panel led driver)
#define PCB_ID3 112          //GP_CORE_016 = 20+96

static struct mdfld_dsi_config *b101uan017_dsi_config;

#define PWM_ENABLE_GPIO 49
//#define PWM_BASE_UNIT 0x80 //586 Hz
//#define PWM_BASE_UNIT 0x32 //200Hz
#define PWM_BASE_UNIT 0x1111 //20,000Hz


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
	writel(pwmctrl.full,  pwmctrl_mmio);

	return 0;
}


static void pwm_enable(){
	union sst_pwmctrl_reg pwmctrl;

	lnw_gpio_set_alt(PWM_ENABLE_GPIO, LNW_ALT_2);
	/*Enable the PWM by setting PWM enable bit to 1 */
	pwmctrl.full = readl(pwmctrl_mmio);
	pwmctrl.part.pwmenable = 1;
	writel(pwmctrl.full, pwmctrl_mmio);
}

static void pwm_disable(){
	union sst_pwmctrl_reg pwmctrl;
	/*setting PWM enable bit to 0 */
	pwmctrl.full = readl(pwmctrl_mmio);
	pwmctrl.part.pwmenable = 0;
	writel(pwmctrl.full,  pwmctrl_mmio);
}


static void
b101uan017_vid_dsi_controller_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_context *hw_ctx = &dsi_config->dsi_hw_context;

	struct drm_device *dev = dsi_config->dev;
	struct gamma_setting auo_gamma = {	.pipe = 0,
									.type = GAMMA_REG_SETTING,
									.enable_state = true,
									.data_len = GAMMA_10_BIT_TABLE_COUNT,
									.gamma_tableX100 = {
										0x000000, 0x020202, 0x040404, 0x060606,
										0x080808, 0x0A0A0A, 0x0C0C0C, 0x0E0E0E,
										0x101010, 0x121212, 0x141414, 0x161616,
										0x181818, 0x1A1A1A, 0x1C1C1C, 0x1E1E1E,
										0x202020, 0x222222, 0x242424, 0x262626,
										0x282828, 0x2A2A2A, 0x2C2C2C, 0x2E2E2E,
										0x303030, 0x323232, 0x343434, 0x363636,
										0x383838, 0x3A3A3A, 0x3C3C3C, 0x3E3E3E,
										0x404040, 0x424242, 0x444444, 0x464646,
										0x484848, 0x4A4A4A, 0x4C4C4C, 0x4E4E4E,
										0x505050, 0x525252, 0x545454, 0x565656,
										0x585858, 0x5A5A5A, 0x5C5C5C, 0x5E5E5E,
										0x606060, 0x626262, 0x646464, 0x666666,
										0x686868, 0x6A6A6A, 0x6C6C6C, 0x6E6E6E,
										0x707070, 0x727272, 0x747474, 0x767676,
										0x787878, 0x7A7A7A, 0x7C7C7C, 0x7E7E7E,
										0x808080, 0x828282, 0x848484, 0x868686,
										0x888888, 0x8A8A8A, 0x8C8C8C, 0x8E8E8E,
										0x909090, 0x929292, 0x949494, 0x969696,
										0x989898, 0x9A9A9A, 0x9C9C9C, 0x9E9E9E,
										0xA0A0A0, 0xA2A2A2, 0xA4A4A4, 0xA6A6A6,
										0xA8A8A8, 0xAAAAAA, 0xACACAC, 0xAEAEAE,
										0xB0B0B0, 0xB2B2B2, 0xB4B4B4, 0xB6B6B6,
										0xB8B8B8, 0xBABABA, 0xBCBCBC, 0xBEBEBE,
										0xC0C0C0, 0xC2C2C2, 0xC4C4C4, 0xC6C6C6,
										0xC8C8C8, 0xCACACA, 0xCCCCCC, 0xCECECE,
										0xD0D0D0, 0xD2D2D2, 0xD4D4D4, 0xD6D6D6,
										0xD8D8D8, 0xDADADA, 0xDCDCDC, 0xDEDEDE,
										0xE0E0E0, 0xE2E2E2, 0xE4E4E4, 0xE6E6E6,
										0xE8E8E8, 0xEAEAEA, 0xECECEC, 0xEEEEEE,
										0xF0F0F0, 0xF2F2F2, 0xF4F4F4, 0xF6F6F6,
										0xF8F8F8, 0xFAFAFA, 0xFCFCFC, 0xFFFFFF}
								 };

	struct gamma_setting cpt_gamma = {	.pipe = 0,
									.type = GAMMA_REG_SETTING,
									.enable_state = true,
									.data_len = GAMMA_10_BIT_TABLE_COUNT,
									.gamma_tableX100 = {
										0x000000, 0x010000, 0x030202, 0x040403,
										0x060505, 0x080707, 0x0A0909, 0x0C0B0B,
										0x0E0E0D, 0x101010, 0x121212, 0x141515,
										0x171717, 0x191A1A, 0x1C1C1D, 0x1E1F1F,
										0x212122, 0x232324, 0x262627, 0x282829,
										0x2B2B2C, 0x2D2D2E, 0x2F2F31, 0x323133,
										0x343435, 0x363637, 0x38383A, 0x3A3A3C,
										0x3D3D3E, 0x3F3F41, 0x414143, 0x434445,
										0x454648, 0x48484A, 0x4A4A4C, 0x4C4C4E,
										0x4E4E51, 0x505153, 0x535355, 0x555558,
										0x57575A, 0x5A5A5C, 0x5C5C5F, 0x5E5E61,
										0x616063, 0x636366, 0x656568, 0x67676A,
										0x69696C, 0x6B6B6E, 0x6D6D70, 0x6F6F72,
										0x717174, 0x737376, 0x757578, 0x77767A,
										0x79787B, 0x7B7A7D, 0x7D7C7F, 0x7F7E81,
										0x818083, 0x838285, 0x858487, 0x878689,
										0x88888B, 0x8A8A8D, 0x8C8C8F, 0x8E8E91,
										0x908F93, 0x929194, 0x949396, 0x969598,
										0x97979A, 0x99999C, 0x9B9B9E, 0x9D9CA0,
										0x9F9EA2, 0xA0A0A4, 0xA2A2A5, 0xA4A4A7,
										0xA6A5A9, 0xA7A7AB, 0xA9A9AC, 0xABABAE,
										0xACACB0, 0xAEAEB2, 0xB0B0B4, 0xB2B2B6,
										0xB3B3B7, 0xB5B5B9, 0xB7B7BB, 0xB9B9BD,
										0xBBBBBF, 0xBCBCC1, 0xBEBEC3, 0xC0C0C4,
										0xC2C2C6, 0xC4C4C8, 0xC5C6CA, 0xC7C7CC,
										0xC9C9CE, 0xCBCBCF, 0xCDCDD1, 0xCFCFD3,
										0xD0D0D5, 0xD2D2D7, 0xD4D4D8, 0xD6D6DA,
										0xD8D8DC, 0xD9D9DE, 0xDBDBDF, 0xDDDDE1,
										0xDFDFE3, 0xE0E0E4, 0xE2E2E6, 0xE4E4E7,
										0xE6E6E9, 0xE7E7EB, 0xE9E9EC, 0xEBEBEE,
										0xEDEDEF, 0xEFEFF1, 0xF1F1F3, 0xF3F3F5,
										0xF6F5F7, 0xF8F8F9, 0xFBFBFB, 0xFFFFFF}
								 };
	/* Reconfig lane configuration */
	dsi_config->lane_count = 4;
	dsi_config->lane_config = MDFLD_DSI_DATA_LANE_4_0;

	/*
	 * Data Rate = 2040 * 1212 * 60 * 24 / 4 = 890.0928Mbps,
	 * DSI Clock Frequency = 890.0928 / 2 = 445.0464MHz,
	 * UI = 1 / 445.0464 / 2 = 1.1235ns
	 */
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
	hw_ctx->clk_lane_switch_time_cnt = 0x2c0015;

	if(asus_panel_id == 0x10)
		hw_ctx->dphy_param = 0x301b7416;
	else
		hw_ctx->dphy_param = 0x301b741f;

	/* Setup video mode format */
	hw_ctx->video_mode_format = 0xf;

	/* Set up func_prg, RGB888(0x200) */
	hw_ctx->dsi_func_prg = (0x200 | dsi_config->lane_count);

	/* Setup mipi port configuration */
	hw_ctx->mipi = PASS_FROM_SPHY_TO_AFE | dsi_config->lane_config;

	b101uan017_dsi_config = dsi_config;
	
	drm_psb_enable_gamma = 1;
	if(asus_panel_id == 0x11)
	    mdfld_intel_crtc_set_gamma(dev, &auo_gamma);
	else
	    mdfld_intel_crtc_set_gamma(dev, &cpt_gamma);

	DRM_INFO("[DISPLAY] %s: asus_panel_id = 0x%x\n", __func__, asus_panel_id);

	pr_debug("[DISPLAY] %s: Enter\n", __func__);
}

static int b101uan017_vid_detect(struct mdfld_dsi_config *dsi_config)
{
	int status;
	struct drm_device *dev = dsi_config->dev;
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	u32 dpll_val, device_ready_val;
	int pipe = dsi_config->pipe;

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

	pr_debug("[DISPLAY] %s: Enter\n", __func__);
	return status;
}

static int b101uan017_vid_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	/* LCD_BL_EN*/
	intel_scu_ipc_iowrite8(LCD_BL_EN,0x1);
	usleep_range(10000, 15000);

	/* Send TURN_ON packet */
	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,
					    MDFLD_DSI_DPI_SPK_TURN_ON);
	if (err) {
		DRM_ERROR("Failed to send turn on packet\n");
		return err;
	}
	usleep_range(200000, 200000);

	/*PWM enable*/
	pwm_enable();
	usleep_range(10000, 15000);


	if(asus_panel_id == 0x10){
		mdfld_dsi_send_gen_short_lp(sender, 0xf3, 0xa0, 2, 0);
		//BIST
		//mdfld_dsi_send_gen_short_lp(sender, 0x0e, 0x86, 2, 0);
		//fail rate count(default 0x44)
		mdfld_dsi_send_gen_short_lp(sender, 0x18, 0x47, 2, 0);
		mdfld_dsi_send_gen_short_lp(sender, 0x00, 0x00, 0, 0);
	}

	/*EN_VDD_BL*/
	intel_scu_ipc_iowrite8(EN_VDD_BL,0x1);

	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);
	return 0;
}

static int b101uan017_vid_power_off(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

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

	/*LCD_LOGIC_PWR_EN*/
	if (gpio_direction_output(LCD_LOGIC_PWR_EN, 0))
		gpio_set_value_cansleep(LCD_LOGIC_PWR_EN, 0);
	usleep_range(500000, 510000);

	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);

	return 0;
}

static int b101uan017_vid_reset(struct mdfld_dsi_config *dsi_config)
{

	/*LCD_LOGIC_PWR_EN*/
	if (gpio_direction_output(LCD_LOGIC_PWR_EN, 1))
		gpio_set_value_cansleep(LCD_LOGIC_PWR_EN, 1);
	usleep_range(1000, 3000);

	pr_debug("[DISPLAY] %s: Enter\n", __func__);

	return 0;
}

#define PWM0DUTYCYCLE 0x67
#define DUTY_VALUE_MAX 0x63
#define BRIGHTNESS_LEVEL_MAX 100
#define BACKLIGHT_DUTY_FACTOR	0xff

static int bl_prev_level = 0;
static int b101uan017_vid_set_brightness(struct mdfld_dsi_config *dsi_config,
					 int level)
{
	int duty_val = 0;

	duty_val = level;
	pwm_configure(duty_val);

	//add for debug ++
	if(!!bl_prev_level^!!level)
		DRM_INFO("[DISPLAY] brightness level : %d% > %d%\n", bl_prev_level, level);

	bl_prev_level = level;
	//add for debug --

	pr_debug("[DISPLAY] brightness level : %d% (duty = 0x%x)\n", level, duty_val);

	return 0;
}

struct drm_display_mode *b101uan017_vid_get_config_mode(void)
{
	struct drm_display_mode *mode;

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	mode->hdisplay = 1920;
	mode->vdisplay = 1200;
	mode->hsync_start = mode->hdisplay + 40;		/* HFP */
	mode->hsync_end = mode->hsync_start + 40;		/* HSW */
	mode->htotal = mode->hsync_end + 40;			/* HBP */
	mode->vsync_start = mode->vdisplay + 10;		/* VFP */
	mode->vsync_end = mode->vsync_start + 4;		/* VSW */
	mode->vtotal = mode->vsync_end + 10;			/* VBP */
	mode->vrefresh = 60;
	mode->clock = mode->vrefresh * mode->vtotal * mode->htotal / 1000;
	mode->type |= DRM_MODE_TYPE_PREFERRED;
	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	pr_debug("[DISPLAY] %s: Enter\n", __func__);
	return mode;
}

static void b101uan017_vid_get_panel_info(int pipe, struct panel_info *pi)
{
	pi->width_mm = 217;
	pi->height_mm = 136;
}

static int b101uan017_vid_gpio_init(void)
{
	gpio_request(LCD_LOGIC_PWR_EN, "LCD_LOGIC_PWR_EN");
	gpio_request(PWM_ENABLE_GPIO, "PWM_ENABLE_GPIO");
	gpio_request(PCB_ID3, "PCB_ID3");

	pr_debug("[DISPLAY] %s: Enter\n", __func__);
	return 0;
}

#define PWM0CLKDIV1 0x61
#define PWM0CLKDIV0 0x62

#define PWMCTRL_REG 0xffae9000
#define PWMCTRL_SIZE 0x80

static int b101uan017_vid_brightness_init(void)
{
	int ret = 0;

	pwmctrl_mmio = ioremap_nocache(PWMCTRL_REG,PWMCTRL_SIZE);

	lnw_gpio_set_alt(PWM_ENABLE_GPIO, LNW_ALT_2);

	pr_debug("[DISPLAY] %s: Enter\n", __func__);
	return ret;
}

#ifdef B101UAN017_DEBUG

static int send_mipi_ret = -1;
static int read_mipi_ret = -1;
static u8 read_mipi_data = 0;

static ssize_t send_mipi_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    int x0=0, x1=0;
    struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(b101uan017_dsi_config);

    sscanf(buf, "%x,%x", &x0, &x1);

    send_mipi_ret = mdfld_dsi_send_gen_short_lp(sender,x0,x1,1,0);

	DRM_INFO("[DISPLAY] send %x,%x : ret = %d\n",x0,x1,send_mipi_ret);

    return count;
}

static ssize_t send_mipi_show(struct device *dev,
	struct device_attribute *attr, const char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n",send_mipi_ret);
}


static ssize_t read_mipi_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    int x0=0;
    struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(b101uan017_dsi_config);

    sscanf(buf, "%x", &x0);

    read_mipi_ret = mdfld_dsi_read_mcs_lp(sender,x0,&read_mipi_data,1);
    if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
        read_mipi_ret = -EIO;

	DRM_INFO("[DISPLAY] read 0x%x :ret=%d data=0x%x\n", x0, read_mipi_ret, read_mipi_data);

    return count;
}

static ssize_t read_mipi_show(struct device *dev,
	struct device_attribute *attr, const char *buf)
{
	return snprintf(buf, PAGE_SIZE, "ret=%d data=0x%x\n",read_mipi_ret,read_mipi_data);
}

DEVICE_ATTR(send_mipi,S_IRUGO | S_IWUSR, send_mipi_show,send_mipi_store);
DEVICE_ATTR(read_mipi,S_IRUGO | S_IWUSR, read_mipi_show,read_mipi_store);


static struct attribute *b101uan017_attrs[] = {
        &dev_attr_send_mipi.attr,
        &dev_attr_read_mipi.attr,
        NULL
};

static struct attribute_group b101uan017_attr_group = {
        .attrs = b101uan017_attrs,
        .name = "b101uan017",
};

#endif
void b101uan017_vid_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	int ret = 0;

	p_funcs->get_config_mode = b101uan017_vid_get_config_mode;
	p_funcs->get_panel_info = b101uan017_vid_get_panel_info;
	p_funcs->dsi_controller_init = b101uan017_vid_dsi_controller_init;
	p_funcs->detect = b101uan017_vid_detect;
	p_funcs->power_on = b101uan017_vid_power_on;
	p_funcs->power_off = b101uan017_vid_power_off;
	p_funcs->reset = b101uan017_vid_reset;
	p_funcs->set_brightness = b101uan017_vid_set_brightness;
	ret = b101uan017_vid_gpio_init();
	if (ret)
		DRM_ERROR("Faild to request GPIO for B101UAN01.7 panel\n");

	ret = b101uan017_vid_brightness_init();
	if (ret)
		DRM_ERROR("Faild to initilize PWM of MSCI\n");

	asus_panel_id = ME302C_PANEL | (gpio_get_value(PCB_ID3)?0x1:0x0);

#ifdef B101UAN017_DEBUG

    sysfs_create_group(&dev->dev->kobj, &b101uan017_attr_group);

#endif

	DRM_INFO("[DISPLAY] %s: Enter\n", __func__);

}

static int b101uan017_vid_lcd_probe(struct platform_device *pdev)
{
	int ret = 0;

	DRM_INFO("%s: B101UAN017 panel detected\n", __func__);
	intel_mid_panel_register(b101uan017_vid_init);


	return 0;
}

static struct platform_driver b101uan017_vid_lcd_driver = {
	.probe	= b101uan017_vid_lcd_probe,
	.driver	= {
		.name	= B101UAN017_PANEL_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init b101uan017_vid_lcd_init(void)
{
	DRM_INFO("%s\n", __func__);
	return platform_driver_register(&b101uan017_vid_lcd_driver);;
}
module_init(b101uan017_vid_lcd_init);

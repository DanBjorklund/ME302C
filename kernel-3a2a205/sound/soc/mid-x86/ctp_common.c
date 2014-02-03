/*
 *  ctp_common.c - ASoc Machine driver for Intel Clovertrail MID platform
 *
 *  Copyright (C) 2011-13 Intel Corp
 *  Author: KP Jeeja<jeeja.kp@intel.com>
 *  Author: Vaibhav Agarwal <vaibhav.agarwal@intel.com>
 *  Author: Dharageswari.R<dharageswari.r@intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */


#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/gpio.h>
#include <linux/rpmsg.h>
#include <linux/input.h>
#include <linux/time.h>
#include <asm/intel_mid_gpadc.h>
#include <asm/intel_scu_pmic.h>
#include <asm/intel_scu_ipcutil.h>
#include <asm/intel_mid_rpmsg.h>
#include <asm/intel_mid_remoteproc.h>
#include <asm/platform_ctp_audio.h>
#include <sound/pcm.h>
#include <sound/jack.h>
#include <sound/soc.h>
#include "ctp_common.h"

// joe_cheng : Shorten delay from 1sec to 400ms for TT#273302
#define HPDETECT_POLL_INTERVAL	msecs_to_jiffies(400)	/* 1sec */
#define JACK_DEBOUNCE_REMOVE	50
#define JACK_DEBOUNCE_INSERT	100

// joe_cheng
#define HEADSET_BUTTON_KEYCODE 233
#define HEADSET_PLUG_KEYCODE 245

extern int rt5640_enable_micbias(int headset_devices);

enum {
	NO_DEVICE = 0, 
	HEADSET_WITH_MIC, 
	HEADSET_WITHOUT_MIC 
};

extern void mid_headset_report(int state);
static int hp_status = NO_DEVICE;
struct input_dev *input;
static int g_headset_detect_time_start = 0;
static int g_headset_detect_time_end = 0;
static int g_headset_detect_time_resume = 0;

struct snd_soc_card snd_soc_card_ctp = {
	.name = "cloverview_audio",
	.set_bias_level = ctp_set_bias_level,
	.set_bias_level_post = ctp_set_bias_level_post,
};

unsigned int rates_8000_16000[] = {
	8000,
	16000,
};

struct snd_pcm_hw_constraint_list constraints_8000_16000 = {
	.count = ARRAY_SIZE(rates_8000_16000),
	.list = rates_8000_16000,
};

unsigned int rates_48000[] = {
	48000,
};

struct snd_pcm_hw_constraint_list constraints_48000 = {
	.count  = ARRAY_SIZE(rates_48000),
	.list   = rates_48000,
};

unsigned int rates_16000[] = {
	16000,
};

struct snd_pcm_hw_constraint_list constraints_16000 = {
	.count  = ARRAY_SIZE(rates_16000),
	.list   = rates_16000,
};

static struct snd_soc_jack_gpio hs_gpio[] = {
	[CTP_HSDET_GPIO] = {
		.name = "cs-hsdet-gpio",
		.report = SND_JACK_HEADSET,
		.debounce_time = JACK_DEBOUNCE_INSERT,
		.jack_status_check = ctp_soc_jack_gpio_detect,
		.irq_flags = IRQF_TRIGGER_FALLING,
	},
	[CTP_BTN_GPIO] = {
		.name = "cs-hsbutton-gpio",
		.report = SND_JACK_HEADSET | SND_JACK_BTN_0,
		.debounce_time = 100,
		.jack_status_check = ctp_soc_jack_gpio_detect_bp,
		.irq_flags = IRQF_TRIGGER_FALLING,
	},
};

int ctp_startup_probe(struct snd_pcm_substream *substream)
{
	pr_debug("%s - applying rate constraint\n", __func__);
	snd_pcm_hw_constraint_list(substream->runtime, 0,
					SNDRV_PCM_HW_PARAM_RATE,
					&constraints_48000);
	return 0;
}

int ctp_startup_asp(struct snd_pcm_substream *substream)
{
	pr_debug("%s - applying rate constraint\n", __func__);
	snd_pcm_hw_constraint_list(substream->runtime, 0,
				SNDRV_PCM_HW_PARAM_RATE,
				&constraints_48000);
	return 0;
}

int ctp_startup_bt_xsp(struct snd_pcm_substream *substream)
{
	pr_debug("%s - applying rate constraint\n", __func__);
	snd_pcm_hw_constraint_list(substream->runtime, 0,
				SNDRV_PCM_HW_PARAM_RATE,
				&constraints_8000_16000);
	return 0;
}
int get_ssp_bt_sco_master_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card =  snd_kcontrol_chip(kcontrol);
	struct ctp_mc_private *ctx = snd_soc_card_get_drvdata(card);
	struct comms_mc_private *ctl = &(ctx->comms_ctl);

	ucontrol->value.integer.value[0] = ctl->ssp_bt_sco_master_mode;
	return 0;
}
EXPORT_SYMBOL_GPL(get_ssp_bt_sco_master_mode);

int set_ssp_bt_sco_master_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card =  snd_kcontrol_chip(kcontrol);
	struct ctp_mc_private *ctx = snd_soc_card_get_drvdata(card);
	struct comms_mc_private *ctl = &(ctx->comms_ctl);

	if (ucontrol->value.integer.value[0] == ctl->ssp_bt_sco_master_mode)
		return 0;

	ctl->ssp_bt_sco_master_mode = ucontrol->value.integer.value[0];

	return 0;
}
EXPORT_SYMBOL_GPL(set_ssp_bt_sco_master_mode);

int get_ssp_voip_master_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card =  snd_kcontrol_chip(kcontrol);
	struct ctp_mc_private *ctx = snd_soc_card_get_drvdata(card);
	struct comms_mc_private *ctl = &(ctx->comms_ctl);

	ucontrol->value.integer.value[0] = ctl->ssp_voip_master_mode;
	return 0;
}
EXPORT_SYMBOL_GPL(get_ssp_voip_master_mode);

int set_ssp_voip_master_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card =  snd_kcontrol_chip(kcontrol);
	struct ctp_mc_private *ctx = snd_soc_card_get_drvdata(card);
	struct comms_mc_private *ctl = &(ctx->comms_ctl);

	if (ucontrol->value.integer.value[0] == ctl->ssp_voip_master_mode)
		return 0;

	ctl->ssp_voip_master_mode = ucontrol->value.integer.value[0];

	return 0;
}
EXPORT_SYMBOL_GPL(set_ssp_voip_master_mode);

int get_ssp_modem_master_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card =  snd_kcontrol_chip(kcontrol);
	struct ctp_mc_private *ctx = snd_soc_card_get_drvdata(card);
	struct comms_mc_private *ctl = &(ctx->comms_ctl);

	ucontrol->value.integer.value[0] = ctl->ssp_modem_master_mode;
	return 0;
}
EXPORT_SYMBOL_GPL(get_ssp_modem_master_mode);

int set_ssp_modem_master_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card =  snd_kcontrol_chip(kcontrol);
	struct ctp_mc_private *ctx = snd_soc_card_get_drvdata(card);
	struct comms_mc_private *ctl = &(ctx->comms_ctl);

	if (ucontrol->value.integer.value[0] == ctl->ssp_modem_master_mode)
		return 0;

	ctl->ssp_modem_master_mode = ucontrol->value.integer.value[0];

	return 0;
}
EXPORT_SYMBOL_GPL(set_ssp_modem_master_mode);

static int mc_driver_ops(struct ctp_mc_private *ctx,
			 struct ctp_audio_platform_data *pdata)
{
#if 0
	switch (pdata->spid->product_line_id) {
	case INTEL_CLVTP_PHONE_RHB_ENG:
	case INTEL_CLVTP_PHONE_RHB_PRO:
		ctx->ops = ctp_get_rhb_ops();
		return 0;
	case INTEL_CLVTP_PHONE_VB_ENG:
	case INTEL_CLVTP_PHONE_VB_PRO:
		ctx->ops = ctp_get_vb_ops();
		return 0;
	default:
		pr_err("No data for prod line id: %x",
				pdata->spid->product_line_id);
		return -EINVAL;
	};
#else
	ctx->ops = ctp_get_rhb_ops();
	return 0;
#endif
}

/* Board specific codec bias level control */
int ctp_set_bias_level(struct snd_soc_card *card,
		struct snd_soc_dapm_context *dapm,
		enum snd_soc_bias_level level)
{
	struct snd_soc_codec *codec;

	/* Clock management is done only if there is an associated codec
	 * to dapm context and if this not the dummy codec
	 */
	if (dapm->codec) {
		codec = dapm->codec;
		if (!strcmp(codec->name, "snd-soc-dummy"))
			return 0;
	} else {
		pr_debug("In %s dapm context has no associated codec or it is dummy codec.", __func__);
		return 0;
	}

	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
		if (card->dapm.bias_level == SND_SOC_BIAS_OFF)
			intel_scu_ipc_set_osc_clk0(true, CLK0_MSIC);
		card->dapm.bias_level = level;
		break;
	case SND_SOC_BIAS_OFF:
		/* OSC clk will be turned OFF after processing
		 * codec->dapm.bias_level = SND_SOC_BIAS_OFF.
		 */
		break;
	default:
		pr_err("%s: Invalid bias level=%d\n", __func__, level);
		return -EINVAL;
		break;
	}
	pr_debug("card(%s)->bias_level %u\n", card->name,
			card->dapm.bias_level);

	return 0;
}

int ctp_set_bias_level_post(struct snd_soc_card *card,
		struct snd_soc_dapm_context *dapm,
		enum snd_soc_bias_level level)
{
	struct snd_soc_codec *codec;

	/* Clock management is done only if there is an associated codec
	 * to dapm context and if this not the dummy codec
	 */
	if (dapm->codec) {
		codec = dapm->codec;
		if (!strcmp(codec->name, "snd-soc-dummy"))
			return 0;
	} else {
		pr_debug("In %s dapm context has no associated codec or it is dummy codec.", __func__);
		return 0;
	}

	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
		/* Processing already done during set_bias_level()
		 * callback. No action required here.
		 */
		break;
	case SND_SOC_BIAS_OFF:
		if (codec->dapm.bias_level != SND_SOC_BIAS_OFF)
			break;
		intel_scu_ipc_set_osc_clk0(false, CLK0_MSIC);
		card->dapm.bias_level = level;
		break;
	default:
		pr_err("%s: Invalid bias level=%d\n", __func__, level);
		return -EINVAL;
		break;
	}
	pr_debug("%s:card(%s)->bias_level %u\n", __func__, card->name,
			card->dapm.bias_level);
	return 0;
}

static int set_mic_bias(struct snd_soc_jack *jack,
			const char *bias_widget, bool enable)
{
	struct snd_soc_codec *codec = jack->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	if (enable)
		snd_soc_dapm_force_enable_pin(dapm, bias_widget);
	else
		snd_soc_dapm_disable_pin(dapm, bias_widget);

	snd_soc_dapm_sync(&codec->dapm);

	return 0;
}

int ctp_amp_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *k, int event)
{
	int ret;

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		/*Enable  IHFAMP_SD_N  GPIO */
		ret = intel_scu_ipc_iowrite8(GPIOHVCTL, GPIO_AMP_ON);
		if (ret)
			pr_err("write of  failed, err %d\n", ret);
	} else {
		/*Disable  IHFAMP_SD_N  GPIO */
		ret = intel_scu_ipc_iowrite8(GPIOHVCTL, GPIO_AMP_OFF);
		if (ret)
			pr_err("write of  failed, err %d\n", ret);
	}
	return 0;
}

static inline void set_bp_interrupt(struct ctp_mc_private *ctx, bool enable)
{
	if (!enable) {
		if (!atomic_dec_return(&ctx->bpirq_flag)) {
			pr_debug("Disable %d interrupt line\n", ctx->bpirq);
			disable_irq_nosync(ctx->bpirq);
		} else
			atomic_inc(&ctx->bpirq_flag);
	} else {
		if (atomic_inc_return(&ctx->bpirq_flag) == 1) {
			/* If BP intr not enabled */
			pr_debug("Enable %d interrupt line\n", ctx->bpirq);
			enable_irq(ctx->bpirq);
		} else
			atomic_dec(&ctx->bpirq_flag);
	}
}

void cancel_all_work(struct ctp_mc_private *ctx)
{
	struct snd_soc_jack_gpio *gpio;
	cancel_delayed_work_sync(&ctx->jack_work);
	gpio = &hs_gpio[CTP_BTN_GPIO];
	cancel_delayed_work_sync(&gpio->work);
}

static int check_headset_type()
{
	struct snd_soc_jack_gpio *gpio = &hs_gpio[CTP_BTN_GPIO];
	int i, hp_state = NO_DEVICE; 
	for (i = 0; i < 5; i++) {
		printk(KERN_DEBUG "h2w : %s() %d, gpio %d\n", __func__, i, gpio->gpio);
		msleep(10); // 10ms
		if (gpio_get_value(gpio->gpio) == 0) {
			printk(KERN_DEBUG "h2w : %s() hook is 0, i = %d\n", __func__, i);
			hp_state = HEADSET_WITH_MIC;
			break;
		} else {
			printk(KERN_DEBUG "h2w : %s() hook is 1, i = %d\n", __func__, i);
			hp_state = HEADSET_WITHOUT_MIC;
		}
	}
	printk(KERN_DEBUG "h2w : hp_state = %d\n", hp_state);
	return hp_state;
}

static void button_pressed(void)
{
	input_report_key(input, HEADSET_BUTTON_KEYCODE, 1);
	input_sync(input);
	input_report_key(input, HEADSET_BUTTON_KEYCODE, 0);
	input_sync(input);
	printk(KERN_INFO "h2w : %s()\n", __func__);
}

static void report_headset_plug_keycode(void)
{
	input_report_key(input, HEADSET_PLUG_KEYCODE, 1);
	input_sync(input);
	input_report_key(input, HEADSET_PLUG_KEYCODE, 0);
	input_sync(input);
	printk(KERN_INFO "h2w : %s()\n", __func__);
}

int ctp_soc_jack_gpio_detect(void)
{
	struct snd_soc_jack_gpio *gpio = &hs_gpio[CTP_HSDET_GPIO];
	int enable, status;
	struct snd_soc_jack *jack = gpio->jack;
	struct snd_soc_codec *codec = jack->codec;
	struct ctp_mc_private *ctx =
		container_of(jack, struct ctp_mc_private, ctp_jack);

	/* During jack removal, spurious BP interrupt may occur.
	 * Better to disable interrupt until jack insert/removal stabilize.
	 * Also cancel the BP and jack_status_verify work if already sceduled */
	cancel_all_work(ctx);
	set_bp_interrupt(ctx, false);
	enable = gpio_get_value(gpio->gpio);
	if (gpio->invert)
		enable = !enable;
	pr_debug("%s:gpio->%d=0x%d\n", __func__, gpio->gpio, enable);
	pr_debug("Current jack status = 0x%x\n", jack->status);

	set_mic_bias(jack, "micbias1", true);
	status = ctx->ops->hp_detection(codec, jack, enable);
	set_mic_bias(jack, "micbias1", false);
//	if (!status) {
	if (enable) {
		ctx->headset_plug_flag = false;
		/* Jack removed, Disable BP interrupts if not done already */
		set_bp_interrupt(ctx, false);
		mid_headset_report(NO_DEVICE);
		hp_status = NO_DEVICE;
		//report_headset_plug_keycode();
		printk("h2w %s(): headset remove\n", __func__);
	} else if (enable == 0 && hp_status != 0) {
		mid_headset_report(NO_DEVICE);
		hp_status = NO_DEVICE;
		msleep(50);
		g_headset_detect_time_start = jiffies;
		schedule_delayed_work(&ctx->jack_work, HPDETECT_POLL_INTERVAL);
		printk("h2w %s() : wrong headset detection, disconnect and connect again\n", __func__);
	} else { /* If jack inserted, schedule delayed_wq */
		g_headset_detect_time_start = jiffies;
		schedule_delayed_work(&ctx->jack_work, HPDETECT_POLL_INTERVAL);
#ifdef CONFIG_HAS_WAKELOCK
		/*
		 * Take wakelock for one second to give time for the detection
		 * to finish. Jack detection is happening rarely so this doesn't
		 * have big impact to power consumption.
		 */
		wake_lock_timeout(ctx->jack_wake_lock,
				HPDETECT_POLL_INTERVAL + msecs_to_jiffies(50));
#endif
	}

	return status;
}

/* Func to verify Jack status after HPDETECT_POLL_INTERVAL */
void headset_status_verify(struct work_struct *work)
{
	struct snd_soc_jack_gpio *gpio = &hs_gpio[CTP_HSDET_GPIO];
	int enable, status;
	struct snd_soc_jack *jack = gpio->jack;
	struct snd_soc_codec *codec = jack->codec;
	unsigned int mask = SND_JACK_HEADSET;
	struct ctp_mc_private *ctx =
		container_of(jack, struct ctp_mc_private, ctp_jack);

	enable = gpio_get_value(gpio->gpio);
	if (gpio->invert)
		enable = !enable;
	pr_debug("%s:gpio->%d=0x%d\n", __func__, gpio->gpio, enable);
	pr_debug("Current jack status = 0x%x\n", jack->status);

	set_mic_bias(jack, "micbias1", true);
	status = ctx->ops->hp_detection(codec, jack, enable);
	hp_status = check_headset_type();
	printk("h2w %s() : hs_status %d\n", __func__, hp_status);

	/* Enable Button_press interrupt if HS is inserted
	 * and interrupts are not already enabled
	 */
	if(!enable) {
		if (hp_status == HEADSET_WITH_MIC) {
			set_bp_interrupt(ctx, true);
			/* Decrease the debounce time for HS removal detection */
			gpio->debounce_time = JACK_DEBOUNCE_REMOVE;
			mid_headset_report(HEADSET_WITH_MIC);
			//report_headset_plug_keycode();
		} else {
			set_mic_bias(jack, "micbias1", false);
			/* Disable Button_press interrupt if no Headset */
			set_bp_interrupt(ctx, false);
			/* Restore the debounce time for HS insertion detection */
			gpio->debounce_time = JACK_DEBOUNCE_INSERT;

			if (hp_status == HEADSET_WITHOUT_MIC) {
				mid_headset_report(HEADSET_WITHOUT_MIC);
				//report_headset_plug_keycode();
			} else {
				hp_status = 0;
				mid_headset_report(NO_DEVICE);
			}

		}
	}
	//ASUS_BSP++, Fix TT bug #276264, #277265
	else{
		hp_status = 0;
		printk("<<%s hp_status: %d, enable: %d\n", __func__,  hp_status, enable);
		mid_headset_report(NO_DEVICE);
		}
	//ASUS_BSP--, Fix TT bug #276264, #277265
	if (jack->status != hp_status)
		snd_soc_jack_report(jack, hp_status, mask);

	pr_debug("%s: status 0x%x\n", __func__, status);
}

int ctp_soc_jack_gpio_detect_bp(void)
{
	struct snd_soc_jack_gpio *gpio = &hs_gpio[CTP_BTN_GPIO];
	int enable, hs_status, status;
	struct snd_soc_jack *jack = gpio->jack;
	struct snd_soc_codec *codec = jack->codec;
	struct ctp_mc_private *ctx =
		container_of(jack, struct ctp_mc_private, ctp_jack);

	g_headset_detect_time_end = jiffies;
	if ((g_headset_detect_time_end - g_headset_detect_time_start) < 200 || 
	    (g_headset_detect_time_end - g_headset_detect_time_resume) < 200 ) {
		pr_err("h2w : Ignore spurious button interrupt(%lu).\n", 
				(g_headset_detect_time_end - g_headset_detect_time_start));
		return 0;
	}

	status = 0;
	enable = gpio_get_value(gpio->gpio);
	if (gpio->invert)
		enable = !enable;
	pr_debug("%s: gpio->%d=0x%x\n", __func__, gpio->gpio, enable);

	/* Check for headset status before processing interrupt */
	gpio = &hs_gpio[CTP_HSDET_GPIO];
	hs_status = gpio_get_value(gpio->gpio);
	if (gpio->invert)
		hs_status = !hs_status;
	pr_debug("%s: gpio->%d=0x%x\n", __func__, gpio->gpio, hs_status);
	pr_debug("Jack status = %x\n", jack->status);
	printk("h2w %s() : hp_status %d\n", __func__, hp_status);
	if (((hp_status & HEADSET_WITH_MIC) == HEADSET_WITH_MIC)
						&& (!hs_status)) {
		/* HS present, process the interrupt */
		if (!enable) {
			/* Jack removal might be in progress, check interrupt status
			 * before proceeding for button press detection */
			if (!atomic_dec_return(&ctx->bpirq_flag)) {
				status = ctx->ops->bp_detection(codec, jack, enable);
				atomic_inc(&ctx->bpirq_flag);
				printk("h2w %s() : get button interrupt\n");
				button_pressed();
			} else {
				atomic_inc(&ctx->bpirq_flag);
				button_pressed();
				printk("h2w %s() : get button interrupt 2\n");
			}
		} else {
			status = jack->status;
			pr_debug("%s:Invalid BP interrupt\n", __func__);
		}
	} else {
		pr_debug("%s:Spurious BP interrupt : jack_status 0x%x, HS_status 0x%x\n",
				__func__, jack->status, hs_status);
		set_mic_bias(jack, "micbias1", false);
		/* Disable Button_press interrupt if no Headset */
		set_bp_interrupt(ctx, false);
	}
	pr_debug("%s: status 0x%x\n", __func__, status);

	return status;
}


#ifdef CONFIG_PM

static int snd_ctp_prepare(struct device *dev)
{
	struct snd_soc_card *card = dev_get_drvdata(dev);
	struct ctp_mc_private *ctx = snd_soc_card_get_drvdata(card);
	printk("%s device name\n", __func__);
	
	// joe_cheng
	struct snd_soc_jack_gpio *gpio = &hs_gpio[CTP_HSDET_GPIO];
	struct snd_soc_jack *jack = gpio->jack;

	/* switch the mclk to the lowpower mode */
	if (ctx->headset_plug_flag && !ctx->voice_call_flag) {
		if (ctx->ops->mclk_switch) {
			ctx->ops->mclk_switch(dev, false);
			/* Decrease the OSC clk to 4.8Mhz when suspend */
			intel_scu_ipc_osc_clk(OSC_CLK_AUDIO, 4800);
		}
	}
	
	if (hp_status == 1)
		set_mic_bias(jack, "micbias1", false);

	return snd_soc_suspend(dev);
}
static int snd_ctp_complete(struct device *dev)
{
	struct snd_soc_card *card = dev_get_drvdata(dev);
	struct ctp_mc_private *ctx = snd_soc_card_get_drvdata(card);
	int ret;
	printk("%s\n", __func__);
	
	struct snd_soc_jack_gpio *gpio = &hs_gpio[CTP_HSDET_GPIO];
	struct snd_soc_jack *jack = gpio->jack;
	
	/* switch the mclk to the normal mode */
	if (ctx->headset_plug_flag && !ctx->voice_call_flag) {
		if (ctx->ops->mclk_switch) {
			/* recovery the OSC clk to 19.2Mhz when resume */
			intel_scu_ipc_osc_clk(OSC_CLK_AUDIO, 19200);
			ctx->ops->mclk_switch(dev, true);
		}
	}
	if (hp_status == 1 && !gpio_get_value_cansleep(gpio->gpio)) {
		printk("%s call rt5640_enable_micbias\n", __func__);
		rt5640_enable_micbias(hp_status);
	}
	ret = snd_soc_resume(dev);
	
	//printk("joe snd_ctp_complete snd_soc_resume\n");
	if (hp_status != 0 && !gpio_get_value_cansleep(gpio->gpio)) {
		pr_debug("snd_ctp_complete msleep 200\n");
		msleep(200);	
		if (hp_status == 1) {
			pr_debug("snd_ctp_complete micbias1 on\n");
			set_mic_bias(jack, "micbias1", true);
		}
	}

	g_headset_detect_time_resume = jiffies;
	return ret;
}

static void snd_ctp_poweroff(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	snd_soc_poweroff(dev);
	return 0;
}

#else
#define snd_ctp_suspend NULL
#define snd_ctp_resume NULL
#define snd_ctp_poweroff NULL
#endif


static int __devexit snd_ctp_mc_remove(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);
	struct ctp_mc_private *ctx = snd_soc_card_get_drvdata(soc_card);

	pr_debug("In %s\n", __func__);
	cancel_delayed_work_sync(&ctx->jack_work);
#ifdef CONFIG_HAS_WAKELOCK
	if (wake_lock_active(ctx->jack_wake_lock))
		wake_unlock(ctx->jack_wake_lock);
	wake_lock_destroy(ctx->jack_wake_lock);
#endif
	snd_soc_jack_free_gpios(&ctx->ctp_jack, 2, ctx->hs_gpio_ops);
	snd_soc_card_set_drvdata(soc_card, NULL);
	snd_soc_unregister_card(soc_card);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

int snd_ctp_init(struct snd_soc_pcm_runtime *runtime)
{
	int ret, irq;
	struct ctp_mc_private *ctx = snd_soc_card_get_drvdata(runtime->card);
	struct snd_soc_jack_gpio *gpio = &hs_gpio[CTP_BTN_GPIO];
	struct snd_soc_codec *codec = runtime->codec;

	ret = ctx->ops->ctp_init(runtime);
	if (ret) {
		pr_err("CTP init returned failure\n");
		return ret;
	}
	/* Setup the HPDET timer */
	INIT_DELAYED_WORK(&ctx->jack_work, headset_status_verify);

	/* Headset and button jack detection */
	ret = snd_soc_jack_new(codec, "Intel MID Audio Jack",
			SND_JACK_HEADSET | SND_JACK_BTN_0, &ctx->ctp_jack);
	if (ret) {
		pr_err("jack creation failed\n");
		return ret;
	}
	ret = snd_soc_jack_add_gpios(&ctx->ctp_jack, 2, ctx->hs_gpio_ops);
	if (ret) {
		pr_err("adding jack GPIO failed\n");
		return ret;
	}
	irq = gpio_to_irq(gpio->gpio);
	if (irq < 0) {
		pr_err("%d:Failed to map gpio_to_irq\n", irq);
		return irq;
	}

	/* Disable Button_press interrupt if no Headset */
	pr_err("Disable %d interrupt line\n", irq);
	disable_irq_nosync(irq);
	atomic_set(&ctx->bpirq_flag, 0);
	
	return ret;
}

/* SoC card */
static int snd_ctp_mc_probe(struct platform_device *pdev)
{
	int ret_val = 0;
	struct ctp_mc_private *ctx;
	struct ctp_audio_platform_data *pdata = pdev->dev.platform_data;

	pr_debug("In %s\n", __func__);
	ctx = devm_kzalloc(&pdev->dev, sizeof(*ctx), GFP_ATOMIC);
	if (!ctx) {
		pr_err("allocation failed\n");
		return -ENOMEM;
	}
#ifdef CONFIG_HAS_WAKELOCK
	ctx->jack_wake_lock =
		devm_kzalloc(&pdev->dev, sizeof(*(ctx->jack_wake_lock)), GFP_ATOMIC);
	if (!ctx->jack_wake_lock) {
		pr_err("allocation failed for wake_lock\n");
		return -ENOMEM;
	}
	wake_lock_init(ctx->jack_wake_lock, WAKE_LOCK_SUSPEND,
			"jack_detect");
#endif
	/* register the soc card */
	snd_soc_card_ctp.dev = &pdev->dev;

	if (0 != mc_driver_ops(ctx, pdata)) {
		ret_val = -EINVAL;
		goto unalloc;
	}
	ctx->ops->dai_link(&snd_soc_card_ctp);
	if (pdata->codec_gpio_hsdet >= 0 && pdata->codec_gpio_button >= 0) {
		hs_gpio[CTP_HSDET_GPIO].gpio = pdata->codec_gpio_hsdet;
		hs_gpio[CTP_BTN_GPIO].gpio = pdata->codec_gpio_button;
		ret_val = gpio_to_irq(hs_gpio[CTP_BTN_GPIO].gpio);
		if (ret_val < 0) {
			pr_err("%d:Failed to map button irq\n", ret_val);
			goto unalloc;
		}
		ctx->bpirq = ret_val;
	}
	ctx->voice_call_flag = false;
	ctx->hs_gpio_ops = hs_gpio;
	snd_soc_card_set_drvdata(&snd_soc_card_ctp, ctx);
	ret_val = snd_soc_register_card(&snd_soc_card_ctp);
	if (ret_val) {
		pr_err("snd_soc_register_card failed %d\n", ret_val);
		goto unalloc;
	}

	platform_set_drvdata(pdev, &snd_soc_card_ctp);

	// joe_cheng
	input = input_allocate_device();
	if (!input) {
		ret_val = -ENOMEM;
		printk("h2w : input device allocated failed\n");
		return 1;
	}

	input->name = "headset_dev";
	set_bit(EV_SYN, input->evbit);
	set_bit(EV_KEY, input->evbit);
	//set_bit(KEY_POWER, input->keybit);
	set_bit(HEADSET_BUTTON_KEYCODE, input->keybit);
	//set_bit(HEADSET_PLUG_KEYCODE, input->keybit);

	ret_val = input_register_device(input);
	if (ret_val < 0)
		printk("h2w : can't register input device\n");

	pr_info("%s : successfully exited probe\n", __func__);
	return ret_val;

unalloc:
#ifdef CONFIG_HAS_WAKELOCK
	if (wake_lock_active(ctx->jack_wake_lock))
		wake_unlock(ctx->jack_wake_lock);
	wake_lock_destroy(ctx->jack_wake_lock);
#endif
	return ret_val;
}

const struct dev_pm_ops snd_ctp_mc_pm_ops = {
	.prepare = snd_ctp_prepare,
	.complete = snd_ctp_complete,
	.poweroff = snd_ctp_poweroff,
};

static struct platform_driver snd_ctp_mc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "ctp_audio",
		.pm   = &snd_ctp_mc_pm_ops,
	},
	.probe = snd_ctp_mc_probe,
	.remove = __devexit_p(snd_ctp_mc_remove),
};

static int __init snd_ctp_driver_init(void)
{
	pr_info("In %s\n", __func__);
	return platform_driver_register(&snd_ctp_mc_driver);
}

static void snd_ctp_driver_exit(void)
{
	pr_debug("In %s\n", __func__);
	platform_driver_unregister(&snd_ctp_mc_driver);
}

static int snd_clv_rpmsg_probe(struct rpmsg_channel *rpdev)
{
	int ret = 0;

	if (rpdev == NULL) {
		pr_err("rpmsg channel not created\n");
		ret = -ENODEV;
		goto out;
	}

	dev_info(&rpdev->dev, "Probed snd_clv rpmsg device\n");

	ret = snd_ctp_driver_init();

out:
	return ret;
}

static void snd_clv_rpmsg_remove(struct rpmsg_channel *rpdev)
{
	snd_ctp_driver_exit();
	dev_info(&rpdev->dev, "Removed snd_clv rpmsg device\n");
}

static void snd_clv_rpmsg_cb(struct rpmsg_channel *rpdev, void *data,
				int len, void *priv, u32 src)
{
	dev_warn(&rpdev->dev, "unexpected, message\n");

	print_hex_dump(KERN_DEBUG, __func__, DUMP_PREFIX_NONE, 16, 1,
			data, len,  true);
}

static struct rpmsg_device_id snd_clv_rpmsg_id_table[] = {
	{ .name = "rpmsg_msic_clv_audio" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, snd_clv_rpmsg_id_table);

static struct rpmsg_driver snd_clv_rpmsg = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= snd_clv_rpmsg_id_table,
	.probe		= snd_clv_rpmsg_probe,
	.callback	= snd_clv_rpmsg_cb,
	.remove		= snd_clv_rpmsg_remove,
};

static int __init snd_clv_rpmsg_init(void)
{
	return register_rpmsg_driver(&snd_clv_rpmsg);
}

late_initcall(snd_clv_rpmsg_init);

static void __exit snd_clv_rpmsg_exit(void)
{
	return unregister_rpmsg_driver(&snd_clv_rpmsg);
}
module_exit(snd_clv_rpmsg_exit);


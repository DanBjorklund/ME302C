/*
 * Support for Medifield PNW Camera Imaging ISP subsystem.
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
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include <media/v4l2-event.h>
#include <media/v4l2-mediabus.h>
#include "atomisp_cmd.h"
#include "atomisp_common.h"
#include "atomisp_internal.h"

static const unsigned int isp_subdev_input_fmts[] = {
	V4L2_MBUS_FMT_SGRBG10_1X10,
	V4L2_MBUS_FMT_SRGGB10_1X10,
	V4L2_MBUS_FMT_SBGGR10_1X10,
	V4L2_MBUS_FMT_SGBRG10_1X10,
};

static const unsigned int isp_subdev_preview_output_fmts[] = {
	/* yuv420, nv12, yv12, nv21, rgb565 */
	V4L2_MBUS_FMT_UYVY8_1X16,
	V4L2_MBUS_FMT_YUYV8_1X16,
	V4L2_PIX_FMT_YUV420,
	V4L2_PIX_FMT_YVU420,
	V4L2_PIX_FMT_NV12,
	V4L2_PIX_FMT_RGB565,
	V4L2_PIX_FMT_NV21,
};

static const unsigned int isp_subdev_vf_output_fmts[] = {
	/* yuv420, nv12, yv12, nv21, rgb565 */
	V4L2_MBUS_FMT_UYVY8_1X16,
	V4L2_MBUS_FMT_YUYV8_1X16,
	V4L2_PIX_FMT_YUV420,
	V4L2_PIX_FMT_YVU420,
	V4L2_PIX_FMT_NV12,
	V4L2_PIX_FMT_RGB565,
	V4L2_PIX_FMT_NV21,
};

static const unsigned int isp_subdev_capture_output_fmts[] = {
	/* yuv420, nv12, yv12, nv21, rgb565, nv11, yuv422, nv16, yv16, yuy2 */
	/* rgb565, rgb888 */
	V4L2_MBUS_FMT_UYVY8_1X16,
	V4L2_MBUS_FMT_YUYV8_1X16,
	V4L2_PIX_FMT_YUV420,
	V4L2_PIX_FMT_YVU420,
	V4L2_PIX_FMT_YUV422P,
	V4L2_PIX_FMT_YUV444,
	V4L2_PIX_FMT_NV12,
	V4L2_PIX_FMT_NV21,
	V4L2_PIX_FMT_NV16,
	V4L2_PIX_FMT_NV61,
	V4L2_PIX_FMT_YUYV,
	V4L2_PIX_FMT_UYVY,
	V4L2_PIX_FMT_RGB565,
	V4L2_PIX_FMT_RGB32
};

const struct atomisp_in_fmt_conv atomisp_in_fmt_conv[] = {
	{ V4L2_MBUS_FMT_SBGGR8_1X8, SH_CSS_INPUT_FORMAT_RAW_8, sh_css_bayer_order_bggr },
	{ V4L2_MBUS_FMT_SGBRG8_1X8, SH_CSS_INPUT_FORMAT_RAW_8, sh_css_bayer_order_gbrg },
	{ V4L2_MBUS_FMT_SGRBG8_1X8, SH_CSS_INPUT_FORMAT_RAW_8, sh_css_bayer_order_grbg },
	{ V4L2_MBUS_FMT_SRGGB8_1X8, SH_CSS_INPUT_FORMAT_RAW_8, sh_css_bayer_order_rggb },
	{ V4L2_MBUS_FMT_SBGGR10_1X10, SH_CSS_INPUT_FORMAT_RAW_10, sh_css_bayer_order_bggr },
	{ V4L2_MBUS_FMT_SGBRG10_1X10, SH_CSS_INPUT_FORMAT_RAW_10, sh_css_bayer_order_gbrg },
	{ V4L2_MBUS_FMT_SGRBG10_1X10, SH_CSS_INPUT_FORMAT_RAW_10, sh_css_bayer_order_grbg },
	{ V4L2_MBUS_FMT_SRGGB10_1X10, SH_CSS_INPUT_FORMAT_RAW_10, sh_css_bayer_order_rggb },
	{ V4L2_MBUS_FMT_SBGGR12_1X12, SH_CSS_INPUT_FORMAT_RAW_12, sh_css_bayer_order_bggr },
	{ V4L2_MBUS_FMT_SGBRG12_1X12, SH_CSS_INPUT_FORMAT_RAW_12, sh_css_bayer_order_gbrg },
	{ V4L2_MBUS_FMT_SGRBG12_1X12, SH_CSS_INPUT_FORMAT_RAW_12, sh_css_bayer_order_grbg },
	{ V4L2_MBUS_FMT_SRGGB12_1X12, SH_CSS_INPUT_FORMAT_RAW_12, sh_css_bayer_order_rggb },
	{ V4L2_MBUS_FMT_UYVY8_1X16, ATOMISP_INPUT_FORMAT_YUV422_8, 0 },
};

const struct atomisp_in_fmt_conv *atomisp_find_in_fmt_conv(
	enum v4l2_mbus_pixelcode code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(atomisp_in_fmt_conv); i++)
		if (code == atomisp_in_fmt_conv[i].code)
			return &atomisp_in_fmt_conv[i];

	return NULL;
}

/*
 * V4L2 subdev operations
 */

/*
 * isp_subdev_ioctl - CCDC module private ioctl's
 * @sd: ISP V4L2 subdevice
 * @cmd: ioctl command
 * @arg: ioctl argument
 *
 * Return 0 on success or a negative error code otherwise.
 */
static long isp_subdev_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, void *arg)
{
	int ret = 0;

	return ret;
}

/*
 * isp_subdev_set_power - Power on/off the CCDC module
 * @sd: ISP V4L2 subdevice
 * @on: power on/off
 *
 * Return 0 on success or a negative error code otherwise.
 */
static int isp_subdev_set_power(struct v4l2_subdev *sd, int on)
{
	return 0;
}

static int isp_subdev_subscribe_event(struct v4l2_subdev *sd,
	struct v4l2_fh *fh,
	struct v4l2_event_subscription *sub)
{
	if (sub->type != V4L2_EVENT_FRAME_SYNC)
		return -EINVAL;

	return v4l2_event_subscribe(fh, sub, 16, NULL);
}

static int isp_subdev_unsubscribe_event(struct v4l2_subdev *sd,
	struct v4l2_fh *fh,
	struct v4l2_event_subscription *sub)
{
	return v4l2_event_unsubscribe(fh, sub);
}

/*
 * isp_subdev_enum_mbus_code - Handle pixel format enumeration
 * @sd: pointer to v4l2 subdev structure
 * @fh : V4L2 subdev file handle
 * @code: pointer to v4l2_subdev_pad_mbus_code_enum structure
 * return -EINVAL or zero on success
 */
static int isp_subdev_enum_mbus_code(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh,
	struct v4l2_subdev_mbus_code_enum *code)
{
	switch (code->pad) {
	case ATOMISP_SUBDEV_PAD_SINK:
		if (code->index >= ARRAY_SIZE(isp_subdev_input_fmts))
			return -EINVAL;

		code->code = isp_subdev_input_fmts[code->index];
		break;

	case ATOMISP_SUBDEV_PAD_SOURCE_PREVIEW:
		/* format conversion inside isp subdev */
		if (code->index >= ARRAY_SIZE(isp_subdev_preview_output_fmts))
			return -EINVAL;

		code->code = isp_subdev_preview_output_fmts[code->index];
		break;
	case ATOMISP_SUBDEV_PAD_SOURCE_VF:
		/* format conversion inside isp subdev */
		if (code->index >= ARRAY_SIZE(isp_subdev_vf_output_fmts))
			return -EINVAL;

		code->code = isp_subdev_vf_output_fmts[code->index];
		break;
	case ATOMISP_SUBDEV_PAD_SOURCE_CAPTURE:
		/* format conversion inside isp subdev */
		if (code->index >= ARRAY_SIZE(isp_subdev_capture_output_fmts))
			return -EINVAL;

		code->code = isp_subdev_capture_output_fmts[code->index];
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int isp_subdev_validate_rect(struct v4l2_subdev *sd, uint32_t pad,
				    uint32_t target)
{
	switch (pad) {
	case ATOMISP_SUBDEV_PAD_SINK:
		switch (target) {
		case V4L2_SEL_TGT_CROP:
			return 0;
		}
		break;
	default:
		switch (target) {
		case V4L2_SEL_TGT_COMPOSE:
			return 0;
		}
		break;
	}

	return -EINVAL;
}

struct v4l2_rect *atomisp_subdev_get_rect(struct v4l2_subdev *sd,
					  struct v4l2_subdev_fh *fh,
					  uint32_t which, uint32_t pad,
					  uint32_t target)
{
	struct atomisp_sub_device *isp_sd = v4l2_get_subdevdata(sd);

	if (which == V4L2_SUBDEV_FORMAT_TRY) {
		switch (target) {
		case V4L2_SEL_TGT_CROP:
			return v4l2_subdev_get_try_crop(fh, pad);
		case V4L2_SEL_TGT_COMPOSE:
			return v4l2_subdev_get_try_compose(fh, pad);
		}
	}

	switch (target) {
	case V4L2_SEL_TGT_CROP:
		return &isp_sd->fmt[pad].crop;
	case V4L2_SEL_TGT_COMPOSE:
		return &isp_sd->fmt[pad].compose;
	}

	return NULL;
}

struct v4l2_mbus_framefmt
*atomisp_subdev_get_ffmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			 uint32_t which, uint32_t pad)
{
	struct atomisp_sub_device *isp_sd = v4l2_get_subdevdata(sd);

	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_get_try_format(fh, pad);

	return &isp_sd->fmt[pad].fmt;
}

static void isp_get_fmt_rect(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			     uint32_t which, struct v4l2_mbus_framefmt **ffmt,
			     struct v4l2_rect *crop[ATOMISP_SUBDEV_PADS_NUM],
			     struct v4l2_rect *comp[ATOMISP_SUBDEV_PADS_NUM])
{
	unsigned int i;

	for (i = 0; i < ATOMISP_SUBDEV_PADS_NUM; i++) {
		ffmt[i] = atomisp_subdev_get_ffmt(sd, fh, which, i);
		crop[i] = atomisp_subdev_get_rect(sd, fh, which, i,
						  V4L2_SEL_TGT_CROP);
		comp[i] = atomisp_subdev_get_rect(sd, fh, which, i,
						  V4L2_SEL_TGT_COMPOSE);
	}
}

static void isp_subdev_propagate(struct v4l2_subdev *sd,
				 struct v4l2_subdev_fh *fh,
				 uint32_t which, uint32_t pad, uint32_t target,
				 uint32_t flags)
{
	struct v4l2_mbus_framefmt *ffmt[ATOMISP_SUBDEV_PADS_NUM];
	struct v4l2_rect *crop[ATOMISP_SUBDEV_PADS_NUM],
		*comp[ATOMISP_SUBDEV_PADS_NUM];
	struct v4l2_rect r;

	if (flags & V4L2_SEL_FLAG_KEEP_CONFIG)
		return;

	isp_get_fmt_rect(sd, fh, which, ffmt, crop, comp);

	memset(&r, 0, sizeof(r));

	switch (pad) {
	case ATOMISP_SUBDEV_PAD_SINK:
		/* Only crop target supported on sink pad. */
		r.width = ffmt[pad]->width;
		r.height = ffmt[pad]->height;

		atomisp_subdev_set_selection(
			sd, fh, which, pad, target, flags, &r);
		break;
	}
}

static int isp_subdev_get_selection(struct v4l2_subdev *sd,
				    struct v4l2_subdev_fh *fh,
				    struct v4l2_subdev_selection *sel)
{
	int rval = isp_subdev_validate_rect(sd, sel->pad, sel->target);
	if (rval)
		return rval;

	sel->r = *atomisp_subdev_get_rect(sd, fh, sel->which, sel->pad,
					  sel->target);

	return 0;
}

static char *atomisp_pad_str[] = { "ATOMISP_SUBDEV_PAD_SINK",
				   "ATOMISP_SUBDEV_PAD_SOURCE_CAPTURE",
				   "ATOMISP_SUBDEV_PAD_SOURCE_VF",
				   "ATOMISP_SUBDEV_PAD_SOURCE_PREVIEW" };

int atomisp_subdev_set_selection(struct v4l2_subdev *sd,
				 struct v4l2_subdev_fh *fh, uint32_t which,
				 uint32_t pad, uint32_t target, uint32_t flags,
				 struct v4l2_rect *r)
{
	struct atomisp_sub_device *isp_sd = v4l2_get_subdevdata(sd);
	struct atomisp_device *isp = isp_sd->isp;
	struct v4l2_mbus_framefmt *ffmt[ATOMISP_SUBDEV_PADS_NUM];
	struct v4l2_rect *crop[ATOMISP_SUBDEV_PADS_NUM],
		*comp[ATOMISP_SUBDEV_PADS_NUM];
	unsigned int i;

	isp_get_fmt_rect(sd, fh, which, ffmt, crop, comp);

	dev_dbg(isp->dev,
		"sel: pad %s tgt %s l %d t %d w %d h %d which %s f 0x%8.8x\n",
		atomisp_pad_str[pad], target == V4L2_SEL_TGT_CROP
		? "V4L2_SEL_TGT_CROP" : "V4L2_SEL_TGT_COMPOSE",
		r->left, r->top, r->width, r->height,
		which == V4L2_SUBDEV_FORMAT_TRY ? "V4L2_SUBDEV_FORMAT_TRY"
		: "V4L2_SUBDEV_FORMAT_ACTIVE", flags);

	r->width = rounddown(r->width, ATOM_ISP_STEP_WIDTH);
	r->height = rounddown(r->height, ATOM_ISP_STEP_HEIGHT);

	switch (pad) {
	case ATOMISP_SUBDEV_PAD_SINK: {
		/* Only crop target supported on sink pad. */
		unsigned int dvs_w, dvs_h;

		crop[pad]->width = ffmt[pad]->width;
		crop[pad]->height = ffmt[pad]->height;

		if (!isp->sw_contex.bypass && crop[pad]->width
		    && crop[pad]->height)
			crop[pad]->width -= pad_w, crop[pad]->height -= pad_h;

		/* if subdev type is SOC camera,we do not need to set DVS */
		if (isp->inputs[isp->input_curr].type == SOC_CAMERA)
			isp->params.video_dis_en = 0;

		if (isp->params.video_dis_en &&
		    isp->isp_subdev.run_mode->val == ATOMISP_RUN_MODE_VIDEO) {
			/* This resolution contains 20 % of DVS slack
			 * (of the desired captured image before
			 * scaling, or 1 / 6 of what we get from the
			 * sensor) in both width and height. Remove
			 * it. */
			crop[pad]->width = roundup(crop[pad]->width * 5 / 6,
						   ATOM_ISP_STEP_WIDTH);
			crop[pad]->height = roundup(crop[pad]->height * 5 / 6,
						    ATOM_ISP_STEP_HEIGHT);
		}

		crop[pad]->width = min(crop[pad]->width, r->width);
		crop[pad]->height = min(crop[pad]->height, r->height);

		if (!(flags & V4L2_SEL_FLAG_KEEP_CONFIG)) {
			for (i = ATOMISP_SUBDEV_PAD_SOURCE_CAPTURE;
			     i <= ATOMISP_SUBDEV_PAD_SOURCE_PREVIEW; i++) {
				struct v4l2_rect tmp = *crop[pad];

				atomisp_subdev_set_selection(
					sd, fh, which, i, V4L2_SEL_TGT_COMPOSE,
					flags, &tmp);
			}
		}

		if (which == V4L2_SUBDEV_FORMAT_TRY)
			break;

		if (isp->params.video_dis_en &&
		    isp->isp_subdev.run_mode->val == ATOMISP_RUN_MODE_VIDEO) {
			dvs_w = rounddown(crop[pad]->width / 5,
					  ATOM_ISP_STEP_WIDTH);
			dvs_h = rounddown(crop[pad]->height / 5,
					  ATOM_ISP_STEP_HEIGHT);
		} else {
			dvs_w = dvs_h = 0;
		}

		sh_css_video_set_dis_envelope(dvs_w, dvs_h);
		sh_css_input_set_effective_resolution(
			crop[pad]->width, crop[pad]->height);

		break;
	}
	case ATOMISP_SUBDEV_PAD_SOURCE_CAPTURE: {
		/* Only compose target is supported on source pads. */

		if (crop[ATOMISP_SUBDEV_PAD_SINK]->width == r->width
		    && crop[ATOMISP_SUBDEV_PAD_SINK]->height == r->height)
			isp->params.yuv_ds_en = false;
		else
			isp->params.yuv_ds_en = true;

		comp[pad]->width = r->width;
		comp[pad]->height = r->height;

		break;
	}
	case ATOMISP_SUBDEV_PAD_SOURCE_VF:
	case ATOMISP_SUBDEV_PAD_SOURCE_PREVIEW:
		comp[pad]->width = r->width;
		comp[pad]->height = r->height;
		break;
	}

	/* Set format dimensions on non-sink pads as well. */
	if (pad != ATOMISP_SUBDEV_PAD_SINK) {
		ffmt[pad]->width = comp[pad]->width;
		ffmt[pad]->height = comp[pad]->height;
	}

	*r = *atomisp_subdev_get_rect(sd, fh, which, pad, target);

	dev_dbg(isp->dev, "sel actual: l %d t %d w %d h %d\n",
		r->left, r->top, r->width, r->height);

	return 0;
}

static int isp_subdev_set_selection(struct v4l2_subdev *sd,
				    struct v4l2_subdev_fh *fh,
				    struct v4l2_subdev_selection *sel)
{
	int rval = isp_subdev_validate_rect(sd, sel->pad, sel->target);
	if (rval)
		return rval;

	return atomisp_subdev_set_selection(sd, fh, sel->which, sel->pad,
					    sel->target, sel->flags, &sel->r);
}

static int atomisp_get_sensor_bin_factor(struct atomisp_device *isp)
{
	struct v4l2_control ctrl;
	int hbin, vbin;
	int ret;

	memset(&ctrl, 0, sizeof(ctrl));

	ctrl.id = V4L2_CID_BIN_FACTOR_HORZ;
	ret = v4l2_subdev_call(isp->inputs[isp->input_curr].camera, core,
			       g_ctrl, &ctrl);
	hbin = ctrl.value;
	ctrl.id = V4L2_CID_BIN_FACTOR_VERT;
	ret |= v4l2_subdev_call(isp->inputs[isp->input_curr].camera, core,
				g_ctrl, &ctrl);
	vbin = ctrl.value;

	/*
	 * ISP needs to know binning factor from sensor.
	 * In case horizontal and vertical sensor's binning factors
	 * are different or sensor does not support binning factor CID,
	 * ISP will apply default 0 value.
	 */
	if (ret || hbin != vbin)
		hbin = 0;

	return hbin;
}

int atomisp_subdev_set_ffmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			    uint32_t which, uint32_t pad,
			    struct v4l2_mbus_framefmt *ffmt)
{
	struct atomisp_sub_device *isp_sd = v4l2_get_subdevdata(sd);
	struct atomisp_device *isp = isp_sd->isp;
	struct v4l2_mbus_framefmt *__ffmt =
		atomisp_subdev_get_ffmt(sd, fh, which, pad);

	dev_dbg(isp->dev, "ffmt: pad %s w %d h %d code 0x%8.8x which %s\n",
		atomisp_pad_str[pad], ffmt->width, ffmt->height, ffmt->code,
		which == V4L2_SUBDEV_FORMAT_TRY ? "V4L2_SUBDEV_FORMAT_TRY"
		: "V4L2_SUBDEV_FORMAT_ACTIVE");

	/* Set bypass mode. One must only set raw or non-raw formats
	 * on the source pads. */
	if (pad != ATOMISP_SUBDEV_PAD_SINK
	    && which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		struct v4l2_mbus_framefmt *f =
			atomisp_subdev_get_ffmt(sd, fh, which,
						ATOMISP_SUBDEV_PAD_SINK);

		isp->sw_contex.bypass = !atomisp_is_mbuscode_raw(f->code)
			|| atomisp_is_mbuscode_raw(ffmt->code);
	}

	switch (pad) {
	case ATOMISP_SUBDEV_PAD_SINK: {
		const struct atomisp_in_fmt_conv *fc =
			atomisp_find_in_fmt_conv(ffmt->code);

		if (!fc) {
			ffmt->code = atomisp_in_fmt_conv[0].code;
			dev_dbg(isp->dev, "using 0x%8.8x instead\n",
				ffmt->code);
			fc = atomisp_find_in_fmt_conv(ffmt->code);
			BUG_ON(!fc);
		}

		*__ffmt = *ffmt;

		isp_subdev_propagate(sd, fh, which, pad,
				     V4L2_SEL_TGT_CROP, 0);

		if (which == V4L2_SUBDEV_FORMAT_ACTIVE) {
			sh_css_input_set_resolution(ffmt->width, ffmt->height);
			sh_css_input_set_binning_factor(
				atomisp_get_sensor_bin_factor(isp));
			sh_css_input_set_bayer_order(fc->bayer_order);
			sh_css_input_set_format(fc->in_sh_fmt);
		}

		break;
	}
	case ATOMISP_SUBDEV_PAD_SOURCE_CAPTURE:
	case ATOMISP_SUBDEV_PAD_SOURCE_PREVIEW:
	case ATOMISP_SUBDEV_PAD_SOURCE_VF:
		__ffmt->code = ffmt->code;
		break;
	}

	return 0;
}

/*
 * isp_subdev_get_format - Retrieve the video format on a pad
 * @sd : ISP V4L2 subdevice
 * @fh : V4L2 subdev file handle
 * @pad: Pad number
 * @fmt: Format
 *
 * Return 0 on success or -EINVAL if the pad is invalid or doesn't correspond
 * to the format type.
 */
static int isp_subdev_get_format(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh, struct v4l2_subdev_format *fmt)
{
	fmt->format = *atomisp_subdev_get_ffmt(sd, fh, fmt->which, fmt->pad);

	return 0;
}

/*
 * isp_subdev_set_format - Set the video format on a pad
 * @sd : ISP subdev V4L2 subdevice
 * @fh : V4L2 subdev file handle
 * @pad: Pad number
 * @fmt: Format
 *
 * Return 0 on success or -EINVAL if the pad is invalid or doesn't correspond
 * to the format type.
 */
static int isp_subdev_set_format(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh, struct v4l2_subdev_format *fmt)
{
	return atomisp_subdev_set_ffmt(sd, fh, fmt->which, fmt->pad,
				       &fmt->format);
}

/* V4L2 subdev core operations */
static const struct v4l2_subdev_core_ops isp_subdev_v4l2_core_ops = {
	 .ioctl = isp_subdev_ioctl,
	 .s_power = isp_subdev_set_power,
	 .subscribe_event = isp_subdev_subscribe_event,
	 .unsubscribe_event = isp_subdev_unsubscribe_event,
};

/* V4L2 subdev pad operations */
static const struct v4l2_subdev_pad_ops isp_subdev_v4l2_pad_ops = {
	 .enum_mbus_code = isp_subdev_enum_mbus_code,
	 .get_fmt = isp_subdev_get_format,
	 .set_fmt = isp_subdev_set_format,
	 .get_selection = isp_subdev_get_selection,
	 .set_selection = isp_subdev_set_selection,
};

/* V4L2 subdev operations */
static const struct v4l2_subdev_ops isp_subdev_v4l2_ops = {
	 .core = &isp_subdev_v4l2_core_ops,
	 .pad = &isp_subdev_v4l2_pad_ops,
};

static void isp_subdev_init_params(struct atomisp_sub_device *isp_subdev)
{
	/* parameters initialization */
}

/*
* isp_subdev_link_setup - Setup isp subdev connections
* @entity: ispsubdev media entity
* @local: Pad at the local end of the link
* @remote: Pad at the remote end of the link
* @flags: Link flags
*
* return -EINVAL or zero on success
*/
static int isp_subdev_link_setup(struct media_entity *entity,
	const struct media_pad *local,
	const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct atomisp_sub_device *isp_sd = v4l2_get_subdevdata(sd);
	struct atomisp_device *isp = isp_sd->isp;
	unsigned int i;

	switch (local->index | media_entity_type(remote->entity)) {
	case ATOMISP_SUBDEV_PAD_SINK | MEDIA_ENT_T_V4L2_SUBDEV:
		/* Read from the sensor CSI2-ports. */
		if (!(flags & MEDIA_LNK_FL_ENABLED)) {
			isp_sd->input = ATOMISP_SUBDEV_INPUT_NONE;
			break;
		}

		if (isp_sd->input != ATOMISP_SUBDEV_INPUT_NONE)
			return -EBUSY;

		for (i = 0; i < ATOMISP_CAMERA_NR_PORTS; i++) {
			if (remote->entity != &isp->csi2_port[i].subdev.entity)
				continue;

			isp_sd->input = ATOMISP_SUBDEV_INPUT_CSI2_PORT1 + i;
			return 0;
		}

		return -EINVAL;

	case ATOMISP_SUBDEV_PAD_SINK | MEDIA_ENT_T_DEVNODE:
		/* read from memory */
		if (flags & MEDIA_LNK_FL_ENABLED) {
			if (isp_sd->input >= ATOMISP_SUBDEV_INPUT_CSI2_PORT1 &&
				isp_sd->input < (ATOMISP_SUBDEV_INPUT_CSI2_PORT1
						+ ATOMISP_CAMERA_NR_PORTS))
				return -EBUSY;
			isp_sd->input = ATOMISP_SUBDEV_INPUT_MEMORY;
		} else {
			if (isp_sd->input == ATOMISP_SUBDEV_INPUT_MEMORY)
				isp_sd->input = ATOMISP_SUBDEV_INPUT_NONE;
		}
		break;

	case ATOMISP_SUBDEV_PAD_SOURCE_PREVIEW | MEDIA_ENT_T_DEVNODE:
		/* always write to memory */
		break;

	case ATOMISP_SUBDEV_PAD_SOURCE_VF | MEDIA_ENT_T_DEVNODE:
		/* always write to memory */
		break;

	case ATOMISP_SUBDEV_PAD_SOURCE_CAPTURE | MEDIA_ENT_T_DEVNODE:
		/* always write to memory */
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

/* media operations */
static const struct media_entity_operations isp_subdev_media_ops = {
	 .link_setup = isp_subdev_link_setup,
/*	 .set_power = v4l2_subdev_set_power,	*/
};

static int __atomisp_update_run_mode(struct atomisp_device *isp)
{
	struct v4l2_ctrl *ctrl = isp->isp_subdev.run_mode;
	struct v4l2_ctrl *c;
	struct v4l2_streamparm p;
	int modes[] = { CI_MODE_NONE,
			CI_MODE_VIDEO,
			CI_MODE_STILL_CAPTURE,
			CI_MODE_CONTINUOUS,
			CI_MODE_PREVIEW };
	s32 mode;

	if (ctrl->val != ATOMISP_RUN_MODE_VIDEO &&
	    isp->params.continuous_vf)
		mode = ATOMISP_RUN_MODE_PREVIEW;
	else
		mode = ctrl->val;

	c = v4l2_ctrl_find(
		isp->inputs[isp->input_curr].camera->ctrl_handler,
		V4L2_CID_RUN_MODE);

	if (c)
		return v4l2_ctrl_s_ctrl(c, mode);

	/* Fall back to obsolete s_parm */
	memset(&p, 0, sizeof(p));

	p.parm.capture.capturemode = modes[mode];

	return v4l2_subdev_call(
		isp->inputs[isp->input_curr].camera, video, s_parm, &p);
}

int atomisp_update_run_mode(struct atomisp_device *isp)
{
	int rval;

	mutex_lock(isp->isp_subdev.ctrl_handler.lock);
	rval = __atomisp_update_run_mode(isp);
	mutex_unlock(isp->isp_subdev.ctrl_handler.lock);

	return rval;
}

static int s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct atomisp_device *isp = container_of(
		ctrl->handler, struct atomisp_sub_device, ctrl_handler)->isp;

	switch (ctrl->id) {
	case V4L2_CID_RUN_MODE:
		return __atomisp_update_run_mode(isp);
	}

	return 0;
}

static const struct v4l2_ctrl_ops ctrl_ops = {
	.s_ctrl = &s_ctrl,
};

static const struct v4l2_ctrl_config ctrl_fmt_auto = {
	.ops = &ctrl_ops,
	.id = V4L2_CID_FMT_AUTO,
	.name = "Automatic format guessing",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = 0,
	.max = 1,
	.def = 1,
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
	.name = "Atomisp run mode",
	.type = V4L2_CTRL_TYPE_MENU,
	.min = 1,
	.def = 1,
	.max = 4,
	.qmenu = ctrl_run_mode_menu,
};

static const struct v4l2_ctrl_config ctrl_enable_vfpp = {
	.id = V4L2_CID_ENABLE_VFPP,
	.name = "Atomisp vf postprocess",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = 0,
	.def = 1,
	.max = 1,
};

/*
 * isp_subdev_init_entities - Initialize V4L2 subdev and media entity
 * @isp_subdev: ISP CCDC module
 *
 * Return 0 on success and a negative error code on failure.
 */
static int isp_subdev_init_entities(struct atomisp_sub_device *isp_subdev)
{
	struct v4l2_subdev *sd = &isp_subdev->subdev;
	struct media_pad *pads = isp_subdev->pads;
	struct media_entity *me = &sd->entity;
	int ret;

	isp_subdev->input = ATOMISP_SUBDEV_INPUT_NONE;

	v4l2_subdev_init(sd, &isp_subdev_v4l2_ops);
	strlcpy(sd->name, "ATOM ISP SUBDEV", sizeof(sd->name));
	v4l2_set_subdevdata(sd, isp_subdev);
	sd->flags |= V4L2_SUBDEV_FL_HAS_EVENTS | V4L2_SUBDEV_FL_HAS_DEVNODE;

	pads[ATOMISP_SUBDEV_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	pads[ATOMISP_SUBDEV_PAD_SOURCE_PREVIEW].flags = MEDIA_PAD_FL_SOURCE;
	pads[ATOMISP_SUBDEV_PAD_SOURCE_VF].flags = MEDIA_PAD_FL_SOURCE;
	pads[ATOMISP_SUBDEV_PAD_SOURCE_CAPTURE].flags = MEDIA_PAD_FL_SOURCE;

	isp_subdev->fmt[ATOMISP_SUBDEV_PAD_SINK].fmt.code =
		V4L2_MBUS_FMT_SBGGR10_1X10;
	isp_subdev->fmt[ATOMISP_SUBDEV_PAD_SOURCE_PREVIEW].fmt.code =
		V4L2_MBUS_FMT_SBGGR10_1X10;
	isp_subdev->fmt[ATOMISP_SUBDEV_PAD_SOURCE_VF].fmt.code =
		V4L2_MBUS_FMT_SBGGR10_1X10;
	isp_subdev->fmt[ATOMISP_SUBDEV_PAD_SOURCE_CAPTURE].fmt.code =
		V4L2_MBUS_FMT_SBGGR10_1X10;

	me->ops = &isp_subdev_media_ops;
	me->type = MEDIA_ENT_T_V4L2_SUBDEV;
	ret = media_entity_init(me, ATOMISP_SUBDEV_PADS_NUM, pads, 0);
	if (ret < 0)
		return ret;

	isp_subdev->video_in.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	isp_subdev->video_in.isp = isp_subdev->isp;
	spin_lock_init(&isp_subdev->video_in.irq_lock);

	isp_subdev->video_out_preview.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	isp_subdev->video_out_preview.isp = isp_subdev->isp;
	isp_subdev->video_out_preview.pipe_type = ATOMISP_PIPE_PREVIEW;
	spin_lock_init(&isp_subdev->video_out_preview.irq_lock);

	isp_subdev->video_out_vf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	isp_subdev->video_out_vf.isp = isp_subdev->isp;
	isp_subdev->video_out_vf.pipe_type = ATOMISP_PIPE_VIEWFINDER;
	spin_lock_init(&isp_subdev->video_out_vf.irq_lock);

	isp_subdev->video_out_capture.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	isp_subdev->video_out_capture.isp = isp_subdev->isp;
	isp_subdev->video_out_capture.pipe_type = ATOMISP_PIPE_CAPTURE;
	spin_lock_init(&isp_subdev->video_out_capture.irq_lock);

	ret = atomisp_video_init(&isp_subdev->video_in, "MEMORY");
	if (ret < 0)
		return ret;

	ret = atomisp_video_init(&isp_subdev->video_out_capture, "CAPTURE");
	if (ret < 0)
		return ret;

	ret = atomisp_video_init(&isp_subdev->video_out_vf, "VIEWFINDER");
	if (ret < 0)
		return ret;

	ret = atomisp_video_init(&isp_subdev->video_out_preview, "PREVIEW");
	if (ret < 0)
		return ret;

	/* Connect the isp subdev to the video node. */
	ret = media_entity_create_link(&isp_subdev->video_in.vdev.entity,
		0, &isp_subdev->subdev.entity, ATOMISP_SUBDEV_PAD_SINK, 0);
	if (ret < 0)
		return ret;

	ret = media_entity_create_link(&isp_subdev->subdev.entity,
		ATOMISP_SUBDEV_PAD_SOURCE_PREVIEW,
		&isp_subdev->video_out_preview.vdev.entity, 0, 0);
	if (ret < 0)
		return ret;

	ret = media_entity_create_link(&isp_subdev->subdev.entity,
		ATOMISP_SUBDEV_PAD_SOURCE_VF,
		&isp_subdev->video_out_vf.vdev.entity, 0, 0);
	if (ret < 0)
		return ret;

	ret = media_entity_create_link(&isp_subdev->subdev.entity,
		ATOMISP_SUBDEV_PAD_SOURCE_CAPTURE,
		&isp_subdev->video_out_capture.vdev.entity, 0, 0);
	if (ret < 0)
		return ret;

	ret = v4l2_ctrl_handler_init(&isp_subdev->ctrl_handler, 1);
	if (ret)
		return ret;

	isp_subdev->fmt_auto = v4l2_ctrl_new_custom(&isp_subdev->ctrl_handler,
						    &ctrl_fmt_auto, NULL);
	isp_subdev->run_mode = v4l2_ctrl_new_custom(&isp_subdev->ctrl_handler,
						    &ctrl_run_mode, NULL);
	isp_subdev->enable_vfpp =
				v4l2_ctrl_new_custom(&isp_subdev->ctrl_handler,
						     &ctrl_enable_vfpp, NULL);

	/* Make controls visible on subdev as well. */
	isp_subdev->subdev.ctrl_handler = &isp_subdev->ctrl_handler;

	return isp_subdev->ctrl_handler.error;
}

void atomisp_subdev_unregister_entities(struct atomisp_sub_device *isp_subdev)
{
	v4l2_ctrl_handler_free(&isp_subdev->ctrl_handler);

	media_entity_cleanup(&isp_subdev->subdev.entity);

	v4l2_device_unregister_subdev(&isp_subdev->subdev);
	atomisp_video_unregister(&isp_subdev->video_in);
	atomisp_video_unregister(&isp_subdev->video_out_preview);
	atomisp_video_unregister(&isp_subdev->video_out_vf);
	atomisp_video_unregister(&isp_subdev->video_out_capture);
}

int atomisp_subdev_register_entities(struct atomisp_sub_device *isp_subdev,
	struct v4l2_device *vdev)
{
	int ret;

	/* Register the subdev and video node. */
	ret = v4l2_device_register_subdev(vdev, &isp_subdev->subdev);
	if (ret < 0)
		goto error;

	ret = atomisp_video_register(&isp_subdev->video_out_capture, vdev);
	if (ret < 0)
		goto error;

	ret = atomisp_video_register(&isp_subdev->video_out_vf, vdev);
	if (ret < 0)
		goto error;

	ret = atomisp_video_register(&isp_subdev->video_out_preview, vdev);
	if (ret < 0)
		goto error;

	ret = atomisp_video_register(&isp_subdev->video_in, vdev);
	if (ret < 0)
		goto error;

	return 0;

error:
	atomisp_subdev_unregister_entities(isp_subdev);
	return ret;
}


/*
 * atomisp_subdev_init - ISP Subdevice  initialization.
 * @dev: Device pointer specific to the ATOM ISP.
 *
 * TODO: Get the initialisation values from platform data.
 *
 * Return 0 on success or a negative error code otherwise.
 */
int atomisp_subdev_init(struct atomisp_device *isp)
{
	struct atomisp_sub_device *isp_subdev = &isp->isp_subdev;
	int ret;

	spin_lock_init(&isp_subdev->lock);
	isp_subdev->isp = isp;
	isp_subdev_init_params(isp_subdev);
	ret = isp_subdev_init_entities(isp_subdev);
	if (ret < 0)
		atomisp_subdev_cleanup(isp);

	return ret;
}

void atomisp_subdev_cleanup(struct atomisp_device *isp)
{

}


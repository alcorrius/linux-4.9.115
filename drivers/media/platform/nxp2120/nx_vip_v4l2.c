/* linux/drivers/media/video/nexell/nx_vip_v4l2.c
 *
 * V4L2 interface support file for Nexell NXP2120 camera(VIP) driver
 *
 * Copyright (c) 2017 I4VINE corp.
 * All right reserved by Seungwoo Kim <ksw@i4vine.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/videodev2.h>
#include <media/v4l2-ioctl.h>
#include <linux/delay.h>

/* NXP SOC's registers */
//#include <mach/platform.h>//
#include "nx_vip_core.h"

#define USE_SUBDEV_CALL	1

static struct v4l2_input nx_vip_input_types[] = {
	{
		.index		= 0,
		.name		= "External Camera Input",
		.type		= V4L2_INPUT_TYPE_CAMERA,
		.audioset	= 1,
		.tuner		= 0,
		.std		= V4L2_STD_PAL_BG | V4L2_STD_NTSC_M,
		.status		= 0,
	},
};

static struct v4l2_output nx_vip_output_types[] = {
	{
		.index		= 0,
		.name		= "Memory Output",
		.type		= V4L2_OUTPUT_TYPE_MEMORY,
		.audioset	= 0,
		.modulator	= 0,
		.std		= 0,
	},
};

const static struct v4l2_fmtdesc nx_vip_capture_formats[] = {
	{
		.index		= 0,
		.type		= V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.flags		= FORMAT_FLAGS_PLANAR,
		.description	= "4:2:0, planar, Y-Cb-Cr",
		.pixelformat	= V4L2_PIX_FMT_YUV420,
	},
	{
		.index		= 1,
		.type		= V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.flags		= FORMAT_FLAGS_PLANAR,
		.description	= "4:2:2, planar, Y-Cb-Cr",
		.pixelformat	= V4L2_PIX_FMT_YUV422P,

	},
	{
		.index		= 2,
		.type		= V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.flags		= FORMAT_FLAGS_PACKED,
		.description	= "4:2:2, packed, YCBYCR",
		.pixelformat	= V4L2_PIX_FMT_YUYV,
	},
	{
		.index		= 3,
		.type		= V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.flags		= FORMAT_FLAGS_PACKED,
		.description	= "4:2:2, packed, CBYCRY",
		.pixelformat	= V4L2_PIX_FMT_UYVY,
	}
};

const static struct v4l2_fmtdesc nx_vip_overlay_formats[] = {
	{
		.index		= 0,
		.type		= V4L2_BUF_TYPE_VIDEO_OVERLAY,
		.flags		= FORMAT_FLAGS_PACKED,
		.description	= "16 bpp RGB, le",
		.pixelformat	= V4L2_PIX_FMT_RGB565,
	},
	{
		.index		= 1,
		.type		= V4L2_BUF_TYPE_VIDEO_OVERLAY,
		.flags		= FORMAT_FLAGS_PACKED,
		.description	= "24 bpp RGB, le",
		.pixelformat	= V4L2_PIX_FMT_RGB24,
	},
};


static struct nx_video_fmt supported_formats[] = {
    {
        /* yuyv 422 continuous, 16bit per pixel */
        .name        = "YUV 4:2:2 packed, YCbYCr",
        .pixelformat = V4L2_PIX_FMT_YUYV,
        .mbus_code   = MEDIA_BUS_FMT_YUYV8_2X8,
        .num_planes  = 1,
        .num_sw_planes = 1,
        .is_separated = false,
    }, {
        /* yuv 420 1 plane, 12bit per pixel */
        .name        = "YUV 4:2:0 separated 1-planar, YCbYCr",
        .pixelformat = V4L2_PIX_FMT_YUV420,
        .mbus_code   = MEDIA_BUS_FMT_YUYV8_1_5X8,
        .num_planes  = 1,
        .num_sw_planes = 3,
        .is_separated = false,
    }, {
        /* yuv 422 1 plane, 16bit per pixel */
        .name        = "YUV 4:2:2 separated 1-planar, Y/CbCr",
        .pixelformat = V4L2_PIX_FMT_NV16,
        .mbus_code   = MEDIA_BUS_FMT_YUYV8_1_5X8,
        .num_planes  = 1,
        .num_sw_planes = 3,
        .is_separated = false,
    }, {
        /* yuv 420 1 plane, 12bit per pixel */
        .name        = "YUV 4:2:0 separated 1-planar, Y/CrCb",
        .pixelformat = V4L2_PIX_FMT_NV21,
        .mbus_code   = MEDIA_BUS_FMT_YUYV8_1_5X8,
        .num_planes  = 1,
        .num_sw_planes = 2,
        .is_separated = false,
    }, {
        /* yuv 420 separated 3 plane, 12bit per pixel */
        .name        = "YUV 4:2:0 separated 3-planar, YCbYCr",
        .pixelformat = V4L2_PIX_FMT_YUV420M,
        .mbus_code   = MEDIA_BUS_FMT_YUYV8_1_5X8,
        .num_planes  = 3,
        .num_sw_planes = 3,
        .is_separated = true,

    }, {
        /* yuv 422 separated 3 plane, 16bit per pixel */
        .name        = "YUV 4:2:2 separated 3-planar, YCbYCr",
        .pixelformat = V4L2_PIX_FMT_YUV422P,
        .mbus_code   = MEDIA_BUS_FMT_YUYV8_1X16,
        .num_planes  = 3,
        .num_sw_planes = 3,
        .depth = {8,4,4},
        .is_separated = true,
    }, {
        /* yuv 444 separated 3 plane, 24bit per pixel */
        .name        = "YUV 4:4:4 separated 3-planar, YCbYCr",
        .pixelformat = V4L2_PIX_FMT_YUV444,
        .mbus_code   = MEDIA_BUS_FMT_YUV8_1X24,
        .num_planes  = 3,
        .num_sw_planes = 3,
        .is_separated = true,
    }, {
        /* rgb 565 */
        .name        = "RGB 565",
        .pixelformat = V4L2_PIX_FMT_RGB565,
        .mbus_code   = MEDIA_BUS_FMT_RGB565_2X8_LE,
        .num_planes  = 1,
        .num_sw_planes = 1,
        .is_separated = false,
    }, {
        /* rgb 8888 */
        /*.name        = "RGB 32",
        .pixelformat = V4L2_PIX_FMT_RGB32,
        .mbus_code   = MEDIA_BUS_FMT_XRGB8888_4X8_LE,
        .num_planes  = 1,
        .num_sw_planes = 1,
        .is_separated = false,*/
    },
};


#define NX_VIP_MAX_INPUT_TYPES	ARRAY_SIZE(nx_vip_input_types)
#define NX_VIP_MAX_OUTPUT_TYPES	ARRAY_SIZE(nx_vip_output_types)
#define NX_VIP_MAX_CAPTURE_FORMATS	ARRAY_SIZE(nx_vip_capture_formats)
#define NX_VIP_MAX_OVERLAY_FORMATS	ARRAY_SIZE(nx_vip_overlay_formats)

void nxp_stream_on(struct nx_vip_control *ctrl);
void nxp_stream_off(struct nx_vip_control *ctrl);

static int nx_vip_v4l2_querycap(struct file *file, void *fh,
					struct v4l2_capability *cap)
{
	struct nx_vip_control *ctrl = video_drvdata(file);//(struct nx_vip_control *) fh;
	printk("############%s enter\n", __func__);
	strncpy(cap->driver, "Nexell VIP Driver",sizeof(cap->driver)-1);
	strncpy(cap->card, ctrl->pdev->name, sizeof(cap->card)-1);
	strlcpy(cap->bus_info, "Nxp2120 AHB-bus",sizeof(cap->bus_info));

	cap->version = 0;
//	cap->capabilities = (V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING | V4L2_CAP_READWRITE); /* ksw : READWRITE ? */
	cap->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_CAPTURE_MPLANE;
	cap->capabilities =  cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	printk("############%s exit\n", __func__);
	return 0;
}

static int nx_vip_v4l2_g_fbuf(struct file *file, void *fh,
					struct v4l2_framebuffer *fb)
{
#if 0
	struct nx_vip_control *ctrl = (struct nx_vip_control *) fh;

	*fb = ctrl->v4l2.frmbuf;

	fb->base = ctrl->v4l2.frmbuf.base;
	fb->capability = V4L2_FBUF_CAP_LIST_CLIPPING;

	fb->fmt.pixelformat  = ctrl->v4l2.frmbuf.fmt.pixelformat;
	fb->fmt.width = ctrl->v4l2.frmbuf.fmt.width;
	fb->fmt.height = ctrl->v4l2.frmbuf.fmt.height;
	fb->fmt.bytesperline = ctrl->v4l2.frmbuf.fmt.bytesperline;
#endif
	return 0;
}

static int nx_vip_v4l2_s_fbuf(struct file *filp, void *fh,
					struct v4l2_framebuffer *fb)
{
#if 0
	struct nx_vip_control *ctrl = (struct nx_vip_control *) fh;
	struct v4l2_framebuffer *frmbuf = &(ctrl->v4l2.frmbuf);
	int bpp;

	bpp = nx_vip_set_output_frame(ctrl, &fb->fmt);

	frmbuf->base  = fb->base;
	frmbuf->flags = fb->flags;
	frmbuf->capability = fb->capability;
	frmbuf->fmt.width = fb->fmt.width;
	frmbuf->fmt.height = fb->fmt.height;
	frmbuf->fmt.field = fb->fmt.field;
	frmbuf->fmt.pixelformat = fb->fmt.pixelformat;
	frmbuf->fmt.bytesperline = fb->fmt.width * bpp / 8;
	frmbuf->fmt.sizeimage = fb->fmt.width * frmbuf->fmt.bytesperline;
#endif
	return 0;
}

static int nx_vip_v4l2_enum_fmt_vid_cap(struct file *filp, void *fh,
					struct v4l2_fmtdesc *f)
{
	struct nx_vip_control *ctrl = (struct nx_vip_control *) fh;
	int index = f->index;

	if (index >= NX_VIP_MAX_CAPTURE_FORMATS)
		return -EINVAL;

	memset(f, 0, sizeof(*f));
	memcpy(f, ctrl->v4l2.fmtdesc + index, sizeof(*f));

	return 0;
}


int nx_fill_format(struct nx_video_frame *frame, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pixm = &f->fmt.pix_mp;
	//int i;
	pr_debug("%s enter\n",__func__);
	pr_debug("o_w %d o_h %d\n",  frame->o_width ,frame->o_height);
	pixm->width = frame->o_width;
	pixm->height = frame->o_height;
	pixm->pixelformat = frame->fmt->pixelformat;
	pixm->field = V4L2_FIELD_NONE;
	pixm->colorspace = V4L2_COLORSPACE_SRGB;
	pixm->num_planes = frame->fmt->num_planes;
	
/*	for (i=0; i<pixm->num_planes; ++i){
		int bpl = frame->f_width;
		if (frame->fmt->num_sw_planes == 1)
			bpl = (bpl + frame->fmt->depth[0]) / 8;
		pixm->plane_fmt[i].bytesperline = bpl;
		pixm->plane_fmt[i].sizeimage = (frame->o_width * frame->o_height * frame->fmt->depth[i]) / 8;
	}*/	
	pr_debug("%s exit\n", __func__);
	return 0;
}

static int nx_vip_v4l2_g_fmt_vid_cap_mplane(struct file *file, void *fh, struct v4l2_format *f)
{
	struct nx_vip_control *ctrl = video_drvdata(file);
	struct nx_video_frame *frame = &ctrl->cap_frame;
	int ret;
	
	printk("%s enter\n", __func__);
	if(f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;	
	
	ret = nx_fill_format(frame, f);
	printk("%s exit\n", __func__);	
	return ret;
}

static int nx_vip_v4l2_g_fmt_vid_cap(struct file *file, void *fh,
					struct v4l2_format *f)
{
	struct nx_vip_control *ctrl = video_drvdata(file);
	int size = sizeof(struct v4l2_pix_format);

	memset(&f->fmt.pix, 0, size);
	memcpy(&f->fmt.pix, &(ctrl->v4l2.frmbuf.fmt), size);

	return 0;
}

extern int nx_vip_v4l2_streamon(struct file *filp, void *fh, enum v4l2_buf_type i);
extern int nx_vip_v4l2_streamoff(struct file *filp, void *fh, enum v4l2_buf_type i);



/**
 * nx_find_format - lookup fimc color format by fourcc or media bus format
 * @pixelformat: fourcc to match, ignored if null
 * @mbus_code: media bus code to match, ignored if null
 * @mask: the color flags to match
 * @index: offset in the fimc_formats array, ignored if negative
 */
struct nx_video_fmt *nx_find_format(const u32 pixelformat, const u32 *mbus_code, unsigned int mask, int index)
{
	struct nx_video_fmt *fmt, *def_fmt = NULL;
	unsigned int i;
//	int id = 0;
	pr_debug("%s enter\n", __func__);
	if (index >= (int)ARRAY_SIZE(supported_formats))
		return NULL;
	
	for(i=0; i< ARRAY_SIZE(supported_formats); ++i) {
		fmt = &supported_formats[i];

		if(fmt->pixelformat == pixelformat)	{
			pr_debug("fmt 0x%X fmt->pixelformat %d pixelformat %d\n",(unsigned int)fmt, fmt->pixelformat, pixelformat);
			return fmt;
		}
		if(mbus_code && fmt->mbus_code == *mbus_code) 
			return fmt;
		if(index == i)
			def_fmt = fmt;
	}
	pr_debug("%s exit\n", __func__);
	return def_fmt;
}


static struct nx_video_fmt *nx_vip_try_format(void *ctx, u32 *width, u32 *height, u32 *code, u32 *pixelformat)
{
	u32 mask = 0;
	struct nx_video_fmt *fmt;
	
	pr_debug("%s enter\n",__func__);
	fmt = nx_find_format(*pixelformat, code, mask, 0);
	if (WARN_ON(!fmt))
		return NULL;
	if (code)
		*code = fmt->mbus_code;
	
	if (pixelformat)
		*pixelformat = fmt->pixelformat;
	
	pr_debug("%s exit\n",__func__);
	return fmt;
}


int _set_plane_size(struct nx_video_frame *frame, unsigned int sizes[])
{
    u32 y_stride = ALIGN(frame->width, 32);
    u32 width = ALIGN(frame->width, 32);
    u32 height = ALIGN(frame->height, 16);
    u32 y_size = y_stride * ALIGN(frame->height, 16);

    pr_debug("%s enter\n", __func__);

    pr_debug("y_stride %d y_size %d width %d height %d\n", y_stride, y_size, frame->width, frame->height);
    pr_debug("pixelformat %d\n",frame->fmt->pixelformat);

    switch (frame->fmt->pixelformat) {
    case V4L2_PIX_FMT_YUYV:
    	pr_debug("V4L2_PIX_FMT_YUYV\n");
        frame->size[0] = sizes[0] = y_size << 1;
        frame->stride[0] = y_stride << 1;
        frame->heights[0] = height;
        frame->heights[1] = height >> 1;
        frame->heights[2] = frame->heights[1];
        break;

    case V4L2_PIX_FMT_YUV420M:
    	pr_debug("V4L2_PIX_FMT_YUV420M\n");
        frame->size[0] = sizes[0] = width * height;
        frame->size[1] = sizes[1] =
        frame->size[2] = sizes[2] = frame->size[0] >> 2;

        frame->stride[0] = y_stride;
        frame->stride[1] = frame->stride[2] = ALIGN(y_stride >> 1, 16);
        frame->heights[0] = height;
        frame->heights[1] = height >> 1;
        frame->heights[2] = frame->heights[1];
        break;

    case V4L2_PIX_FMT_YUV420:
    	pr_debug("V4L2_PIX_FMT_YUV420\n");
        frame->size[0] = sizes[0] = y_size;
        frame->size[1] = sizes[1] =
        frame->size[2] = sizes[2] = ALIGN(y_stride >> 1, 16) * ALIGN(frame->height >> 1, 16);

        frame->stride[0] = y_stride;
        frame->stride[1] = frame->stride[2] = ALIGN(y_stride >> 1, 16);
        frame->heights[0] = height;
        frame->heights[1] = height >> 1;
        frame->heights[2] = frame->heights[1];
        break;

    case V4L2_PIX_FMT_NV16:
    	pr_debug("V4L2_PIX_FMT_NV16\n");
        frame->size[0] = sizes[0] =
        frame->size[1] = sizes[1] = y_size;

        frame->stride[0] = frame->stride[1] = y_stride;
        
        frame->heights[0] = height;
        frame->heights[1] = height;
        frame->heights[2] = frame->heights[1];
        break;

    case V4L2_PIX_FMT_NV21:
    	pr_debug("V4L2_PIX_FMT_NV21\n");
        frame->size[0] = sizes[0] = y_size;
        frame->size[1] = y_stride * ALIGN(frame->height >> 1, 16);

        frame->stride[0] = y_stride;
        frame->stride[1] = y_stride;
        
        frame->heights[0] = height;
        frame->heights[1] = height;
        frame->heights[2] = frame->heights[1];
        break;

    case V4L2_PIX_FMT_YUV422P:
    	pr_debug("V4L2_PIX_FMT_YUV422P\n");
        frame->size[0] = sizes[0] = y_size;
        frame->size[1] = frame->size[2] = sizes[1] = sizes[2] = y_size >> 1;

        frame->stride[0] = y_stride;
        frame->stride[1] = frame->stride[2] = ALIGN(y_stride >> 1, 16);
        frame->heights[0] = height;
        frame->heights[1] = height;
        frame->heights[2] = frame->heights[1];
        break;

    case V4L2_PIX_FMT_YUV444:
    	pr_debug("V4L2_PIX_FMT_YUV444\n");
        frame->size[0] = frame->size[1] = frame->size[2] = sizes[0] =
            sizes[1] = sizes[2] = y_size;
        frame->stride[0] = frame->stride[1] = frame->stride[2] =
            y_stride;
        frame->heights[0] = height;
        frame->heights[1] = height;
        frame->heights[2] = frame->heights[1];
        break;

    case V4L2_PIX_FMT_RGB565:
    	pr_debug("V4L2_PIX_FMT_RGB565\n");
        frame->size[0] = sizes[0] = (frame->width * frame->height) << 1;
        frame->stride[0] = width << 1;
        frame->heights[0] = height;
        frame->heights[1] = height;
        frame->heights[2] = frame->heights[1];
        break;

    case V4L2_PIX_FMT_RGB32:
    	pr_debug("V4L2_PIX_FMT_RGB32\n");
        frame->size[0] = sizes[0] = (frame->width * frame->height) << 2;
        frame->stride[0] = (frame->width) << 2;
        frame->heights[0] = height;
        frame->heights[1] = height;
        frame->heights[2] = frame->heights[1];
        break;

    default:
        pr_err("%s: unknown format(%d)\n", __func__, frame->fmt->pixelformat);
        return -EINVAL;
    }
    pr_debug("%s exit\n", __func__);                                       
    return 0;
}


/**
 * nx_video_adjust_mplane_format - adjust bytesperline/sizeimage for each plane
 * @fmt: nx video pixel format description (input)
 * @width: requested pixel width
 * @height: requested pixel height
 * @pix: multi-plane format to adjust
 */
void nx_video_adjust_mplane_format(struct nx_video_fmt *fmt, u32 width, u32 height, struct v4l2_pix_format_mplane *pix)
{
	u32 bytesperline = 0;
	int i;
	
	pr_debug("%s enter\n", __func__);
	pix->colorspace = V4L2_COLORSPACE_SRGB;
	pix->field = V4L2_FIELD_NONE;
	pix->num_planes = fmt->num_planes;
	pix->pixelformat =  fmt->pixelformat;
	pix->height = height;
	pix->width = width;
	
	for(i=0; i<pix->num_planes; ++i){
		u32 bpl = pix->plane_fmt[i].bytesperline;
		u32 *sizeimage = &pix->plane_fmt[i].sizeimage;
		 
		if(fmt->num_sw_planes > 1 && (bpl == 0 || bpl < pix->width))
			bpl = pix->width;	// planar
		
		if(fmt->num_sw_planes == 1 && (bpl == 0 || ((bpl*8)/fmt->depth[i]) < pix->width))
			bpl = (pix->width * fmt->depth[0]) /8;	//packed
		
		if(i==0)
			bytesperline = bpl;
		
		pix->plane_fmt[i].bytesperline = bytesperline;
		*sizeimage = (pix->width * pix->height * fmt->depth[i]) / 8;
	}
	pr_debug("%s exit\n", __func__);
}


int nx_capture_set_format(struct nx_vip_control *ctrl, struct v4l2_format *f)
{
//	struct nx_video_ctx *ctx = &vip_dev->vip_cap->nvctx;
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	struct nx_video_frame *frame = &ctrl->cap_frame;
	//struct nx_video_fmt *s_fmt = NULL;
	int ret = 0;
	int i;
    unsigned int sizes[3];
    int ctx;
	pr_debug("%s enter\n",__func__);
	if(f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE){
		pr_debug("type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE\n");
		return -EINVAL;
	}

/*	if(vb2_is_busy(&vip_dev->vp->vbq) > 0){
		pr_debug("vb2_is_busy\n");
		return 0;
	}*/
	
	pr_debug("pix->pixelformat 0x%X pix->width %d, pix->height %d\n",pix->pixelformat,pix->width,pix->height);
	printk("frame 0x%X\n", frame);
	frame->fmt = nx_vip_try_format(&ctx, &pix->width, &pix->height, NULL,&pix->pixelformat);
	
	if(!frame->fmt)
		return -EINVAL;

	nx_video_adjust_mplane_format(frame->fmt, pix->width, pix->height, pix);
	for(i=0; i<frame->fmt->num_sw_planes; i++)
		frame->size[i] = (pix->width *pix->height * frame->fmt->depth[i])/8;
		
	set_frame_bounds(frame, pix->width, pix->height);
	/* Reset the composition rectangle if not yet configured */
	set_frame_crop(frame, 0, 0, pix->width, pix->height);
	
	//ctx->cap_frame.fmt = s_fmt;
	//set_frame_bounds(&ctx->cap_frame, pix->width, pix->height);
	//set_frame_crop(&ctx->cap_frame, 0, 0, pix->width, pix->height);

    _set_plane_size(frame, sizes);

	pr_debug("%s exit\n",__func__);
	return ret;
}

static int nx_vip_v4l2_s_fmt_vid_cap_mplane(struct file *file, void *fh, struct v4l2_format *f)
{
	struct nx_vip_control *ctrl = video_drvdata(file);//(struct nx_vip_control *) fh;
    struct v4l2_subdev_format format;	
    int ret;
	printk("%s enter\n", __func__);
	format.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	format.format.width  = f->fmt.pix_mp.width;
	format.format.height = f->fmt.pix_mp.height;
	format.format.colorspace = f->fmt.pix_mp.colorspace;
	format.format.field  = f->fmt.pix_mp.field;

	
	ret = nx_capture_set_format(ctrl, f);
	if (ret != 0)
		return ret;	
	
	ret = v4l2_subdev_call(ctrl->subdev, pad, set_fmt, NULL, &format);
	
	
//	nx_vip_set_output_frame(ctrl, &f->fmt.pix_mp);
	printk("%s exit\n", __func__);
	return ret;
}

static int nx_vip_v4l2_s_fmt_vid_cap(struct file *file, void *fh,
					struct v4l2_format *f)
{
	struct nx_vip_control *ctrl = video_drvdata(file);//(struct nx_vip_control *) fh;
    struct v4l2_subdev_format format;	
	//struct v4l2_framebuffer *frmbuf = &(ctrl->v4l2.frmbuf);
	int depth;
	int ret;

	memset(&format, 0, sizeof(format));
	format.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	format.format.width  = f->fmt.pix.width;
	format.format.height = f->fmt.pix.height;
	format.format.colorspace = f->fmt.pix.colorspace;
	format.format.field  = f->fmt.pix.field;	
	
	pr_debug("%s: capture w:%d h:%d\n", __func__, format.format.width, format.format.height);
	ret = nx_capture_set_format(ctrl, f);
	if (ret != 0)
		return ret;	
	
	ret = v4l2_subdev_call(ctrl->subdev, pad, set_fmt, NULL, &format);

	pr_debug("%s exit\n",__func__);	
	return ret;
#if 0
	//frmbuf.fmt = f->fmt.pix;
	printk("%s: ww=%d hh=%d \n", __func__, f->fmt.pix.width, f->fmt.pix.height);
	f->fmt.pix.priv = 1; /* We don't use Input Frame Format... Yet. */
	if (f->fmt.pix.priv == V4L2_FMT_IN)
		depth = nx_vip_set_input_frame(ctrl, &f->fmt.pix);
	else {
		// if stream on then we should stream off first.
		if (ctrl->streamon) {
			nxp_stream_off(ctrl);
		}
		// Check if camera input is same as output frame
		if ((f->fmt.pix.width != ctrl->cam_ex->cur_width) ||
			(f->fmt.pix.height != ctrl->cam_ex->cur_height)) {
			int ww = f->fmt.pix.width;
			int hh = f->fmt.pix.height;
			struct nx_vip_camera_ex *cam = ctrl->cam_ex;
			int ret;
#if USE_SUBDEV_CALL
			//struct v4l2_format fmt;
			struct v4l2_subdev_format fmt;
			struct v4l2_subdev_fh *subdev_fh = to_v4l2_subdev_fh(fh);
	
			fmt.which               = 0;
			fmt.pad                 = 0;
			fmt.format.width     = ww;
			fmt.format.height    = hh;
			
			/* Now camera can handle this resolution */
			ret = v4l2_subdev_call(cam->subdev, pad, set_fmt, subdev_fh, &fmt);
			if (0 == ret) {
				// success then crop width should be adjustted.
				printk("Success! then crop width should be adjustted.\n");
				cam->cur_width = ww;
				cam->cur_height = hh;
				ctrl->v4l2.crop_current.left = 0;
				ctrl->v4l2.crop_current.top = 0;
				ctrl->v4l2.crop_current.width = cam->cur_width;
				ctrl->v4l2.crop_current.height = cam->cur_height;
				ctrl->in_frame.width = cam->cur_width;
				ctrl->in_frame.height = cam->cur_height;
			}
#else	
			int res;
	
			if ((ww > 1024) && (ww <= 1280)) {
				// try SXGA
				res = CAM_RES_SXGA;
			} else
			if ((ww > 640) && (ww <= 1024)) {
				// try XGA
				res = CAM_RES_XGA;
			} else
			if ((ww > 320) && (ww <= 640)) {
				// try VGA
				res = CAM_RES_VGA;
			} else
			if ((ww > 160) && (ww <= 320)) {
				// try QVGA
				res = CAM_RES_QVGA;
			} else {
				return -EINVAL;
			}
			ret = nx_vip_change_resolution(ctrl, res);
			if (0 == ret) {
				ctrl->v4l2.crop_current.left = 0;
				ctrl->v4l2.crop_current.top = 0;
				ctrl->v4l2.crop_current.width = cam->cur_width;
				ctrl->v4l2.crop_current.height = cam->cur_height;
			}
#endif
		}

		depth = nx_vip_set_output_frame(ctrl, &f->fmt.pix);
		if (depth > 0) {
		    nx_vip_v4l2_streamon(filp, (void *)ctrl, V4L2_BUF_TYPE_VIDEO_CAPTURE);
		}
	}

	return (depth > 0) ? 0 : -EINVAL;

	return 0;
#endif
}

static int nx_vip_v4l2_try_fmt_vid_cap(struct file *filp, void *fh,
					  struct v4l2_format *f)
{
	return 0;
}

static int nx_vip_v4l2_try_fmt_overlay(struct file *filp, void *fh,
					  struct v4l2_format *f)
{
	return 0;
}

static int nx_vip_v4l2_overlay(struct file *filp, void *fh, unsigned int i)
{
#if 0
	struct nx_vip_control *ctrl = (struct nx_vip_control *) fh;

	if (i) {
		//if (ctrl->in_type != PATH_IN_DMA)
		//	nx_vip_init_camera(ctrl);

		FSET_PREVIEW(ctrl);
		nx_vip_start_vip(ctrl);
	} else {
		nx_vip_stop_vip(ctrl);

		if (ctrl->out_type != PATH_OUT_LCDFIFO) {
			nx_vip_free_output_memory(&ctrl->out_frame);
			nx_vip_set_output_address(ctrl);
		}
	}
#endif
	return 0;
}

#define V4L2_CID_CAMERA_ID (V4L2_CID_PRIVATE_BASE + 0x10)

static int nx_vip_v4l2_g_ctrl(struct file *filp, void *fh,
					struct v4l2_control *c)
{
	struct nx_vip_control *ctrl = (struct nx_vip_control *) fh;
	struct nx_vip_out_frame *frame = &ctrl->out_frame;

	switch (c->id) {
	case V4L2_CID_OUTPUT_ADDR:
		c->value = frame->addr[c->value].phys_y;
		break;
	case V4L2_CID_CAMERA_ID:
		//c->value = ctrl->in_cam->cam_id;
		break;

	default:
		err("invalid control id: %d\n", c->id);
		return -EINVAL;
	}

	return 0;
}

static int nx_vip_v4l2_s_ctrl(struct file *filp, void *fh,
					struct v4l2_control *c)
{
	struct nx_vip_control *ctrl = (struct nx_vip_control *) fh;
	//struct nx_vip_out_frame *frame = &ctrl->out_frame;
	//struct nx_vip_window_offset *offset = &ctrl->in_cam->offset;

	switch (c->id) {
#if 0
	case V4L2_CID_EFFECT_ORIGINAL:
		frame->effect.type = EFFECT_ORIGINAL;
		nx_vip_change_effect(ctrl);
		break;

	case V4L2_CID_EFFECT_NEGATIVE:
		frame->effect.type = EFFECT_NEGATIVE;
		nx_vip_change_effect(ctrl);
		break;

	case V4L2_CID_EFFECT_EMBOSSING:
		frame->effect.type = EFFECT_EMBOSSING;
		nx_vip_change_effect(ctrl);
		break;

	case V4L2_CID_EFFECT_ARTFREEZE:
		frame->effect.type = EFFECT_ARTFREEZE;
		nx_vip_change_effect(ctrl);
		break;

	case V4L2_CID_EFFECT_SILHOUETTE:
		frame->effect.type = EFFECT_SILHOUETTE;
		nx_vip_change_effect(ctrl);
		break;

	case V4L2_CID_EFFECT_ARBITRARY:
		frame->effect.type = EFFECT_ARBITRARY;
		frame->effect.pat_cb = PAT_CB(c->value);
		frame->effect.pat_cr = PAT_CR(c->value);
		nx_vip_change_effect(ctrl);
		break;

	case V4L2_CID_ROTATE_ORIGINAL:
		frame->flip = FLIP_ORIGINAL;
		ctrl->rot90 = 0;
		nx_vip_change_rotate(ctrl);
		break;

	case V4L2_CID_HFLIP:
		frame->flip = FLIP_X_AXIS;
		ctrl->rot90 = 0;
		nx_vip_change_rotate(ctrl);
		break;

	case V4L2_CID_VFLIP:
		frame->flip = FLIP_Y_AXIS;
		ctrl->rot90 = 0;
		nx_vip_change_rotate(ctrl);
		break;

	case V4L2_CID_ROTATE_180:
		frame->flip = FLIP_XY_AXIS;
		ctrl->rot90 = 0;
		nx_vip_change_rotate(ctrl);
		break;

	case V4L2_CID_ROTATE_90:
		frame->flip = FLIP_ORIGINAL;
		ctrl->rot90 = 1;
		nx_vip_change_rotate(ctrl);
		break;

	case V4L2_CID_ROTATE_270:
		frame->flip = FLIP_XY_AXIS;
		ctrl->rot90 = 1;
		nx_vip_change_rotate(ctrl);
		break;

	case V4L2_CID_ROTATE_90_HFLIP:
		frame->flip = FLIP_X_AXIS;
		ctrl->rot90 = 1;
		nx_vip_change_rotate(ctrl);
		break;

	case V4L2_CID_ROTATE_90_VFLIP:
		frame->flip = FLIP_Y_AXIS;
		ctrl->rot90 = 1;
		nx_vip_change_rotate(ctrl);
		break;

	case V4L2_CID_ZOOM_IN:
		if (nx_vip_check_zoom(ctrl, c->id) == 0) {
			offset->h1 += NX_VIP_ZOOM_PIXELS;
			offset->h2 += NX_VIP_ZOOM_PIXELS;
			offset->v1 += NX_VIP_ZOOM_PIXELS;
			offset->v2 += NX_VIP_ZOOM_PIXELS;
			nx_vip_restart_dma(ctrl);
		}

		break;

	case V4L2_CID_ZOOM_OUT:
		if (nx_vip_check_zoom(ctrl, c->id) == 0) {
			offset->h1 -= NX_VIP_ZOOM_PIXELS;
			offset->h2 -= NX_VIP_ZOOM_PIXELS;
			offset->v1 -= NX_VIP_ZOOM_PIXELS;
			offset->v2 -= NX_VIP_ZOOM_PIXELS;
			nx_vip_restart_dma(ctrl);
		}

		break;

	case V4L2_CID_AUTO_WHITE_BALANCE:
		nx_vip_i2c_command(ctrl, I2C_CAM_WB, c->value);
		break;

	case V4L2_CID_ACTIVE_CAMERA:
		nx_vip_set_active_camera(ctrl, c->value);
		nx_vip_i2c_command(ctrl, I2C_CAM_WB, WB_AUTO);
		break;

	case V4L2_CID_TEST_PATTERN:
		nx_vip_set_active_camera(ctrl, NX_VIP_TPID);
		nx_vip_set_test_pattern(ctrl, c->value);
		break;

	case V4L2_CID_NR_FRAMES:
		nx_vip_set_nr_frames(ctrl, c->value);
		break;

	case V4L2_CID_INPUT_ADDR:
		nx_vip_alloc_input_memory(&ctrl->in_frame, \
						(dma_addr_t) c->value);
		nx_vip_set_input_address(ctrl);
		break;

	case V4L2_CID_INPUT_ADDR_Y:
	case V4L2_CID_INPUT_ADDR_RGB:
		nx_vip_alloc_y_memory(&ctrl->in_frame, \
						(dma_addr_t) c->value);
		nx_vip_set_input_address(ctrl);
		break;

	case V4L2_CID_INPUT_ADDR_CB:	/* fall through */
	case V4L2_CID_INPUT_ADDR_CBCR:
		nx_vip_alloc_cb_memory(&ctrl->in_frame, \
						(dma_addr_t) c->value);
		nx_vip_set_input_address(ctrl);
		break;

	case V4L2_CID_INPUT_ADDR_CR:
		nx_vip_alloc_cr_memory(&ctrl->in_frame, \
						(dma_addr_t) c->value);
		nx_vip_set_input_address(ctrl);
		break;

	case V4L2_CID_RESET:
		ctrl->rot90 = 0;
		ctrl->in_frame.flip = FLIP_ORIGINAL;
		ctrl->out_frame.flip = FLIP_ORIGINAL;
		ctrl->out_frame.effect.type = EFFECT_ORIGINAL;
		ctrl->scaler.bypass = 0;
		nx_vip_reset(ctrl);
		break;

	case V4L2_CID_JPEG_INPUT:	/* fall through */
	case V4L2_CID_SCALER_BYPASS:
		ctrl->scaler.bypass = 1;
		break;
#endif
	case V4L2_CID_BLACK_LEVEL:
	    // Do nothing but implemented..

	    break;
	case V4L2_CID_BRIGHTNESS:
	    // Do nothing but implemeted..
	    //nx_vip_i2c_command(ctrl, I2C_CAM_BRIGHTNESS, c->value);
		//  v4l2_subdev_call(ctrl->cam_ex->subdev, core, s_ctrl, c);
	    break;

	case V4L2_CID_POWER_SAVE:
	    // Do nothing but implemeted..
	    //nx_vip_i2c_command(ctrl, I2C_CAM_POWER_SAVE, c->value);
	    break;

	case V4L2_CID_EXPOSURE:
	    //nx_vip_i2c_command(ctrl, I2C_CAM_EXPOSURE, c->value);
		//  v4l2_subdev_call(ctrl->cam_ex->subdev, core, s_ctrl, c);
		break;

	case V4L2_CID_BASE|0xA00:
		//nxp_stream_off(ctrl);
	    //nx_vip_i2c_command(ctrl, I2C_CAM_FIXED_FRAME, c->value);
	    //nxp_stream_on(ctrl);
		break;

	case V4L2_CID_EXPOSURE_AUTO:
	    //nx_vip_i2c_command(ctrl, I2C_CAM_EXPOSURE_AUTO, c->value);
		//  v4l2_subdev_call(ctrl->cam_ex->subdev, core, s_ctrl, c);
		break;

	case V4L2_CID_GAIN:
		//nx_vip_i2c_command(ctrl, I2C_CAM_GAIN, c->value);
		//  v4l2_subdev_call(ctrl->cam_ex->subdev, core, s_ctrl, c);
		break;

	default:
		err("invalid control id: %d\n", c->id);
		return -EINVAL;
	}


	return 0;
}

void nxp_stream_on(struct nx_vip_control *ctrl)
{
	int i;
	printk("%s enter\n", __func__);
	for (i=0; i< NX_VIP_MAX_FRAMES; i++)
		ctrl->cap_frame.skip_frames[i] = 0;
//	FSET_STOP(ctrl);
//	FSET_IRQ_NORMAL(ctrl);
	nx_vip_start_vip(ctrl);
	ctrl->streamon = 1;
	printk("%s exit\n", __func__);	
}

void nxp_stream_off(struct nx_vip_control *ctrl)
{
//    FSET_STOP(ctrl);
//	UNMASK_USAGE(ctrl);
//	UNMASK_IRQ(ctrl);

	ctrl->streamon = 0;

	nx_vip_stop_vip(ctrl);
}

int nx_vip_v4l2_streamon(struct file *file, void *fh,
					enum v4l2_buf_type i)
{
	struct nx_vip_control *ctrl = video_drvdata(file);//(struct nx_vip_control *) fh;
	struct vb2_queue *vq =  &ctrl->vb2_q;
	struct v4l2_subdev *subdev = ctrl->subdev;	
	int ret = 0;
	printk("%s enter ctrl->id %d\n", __func__, ctrl->id);
	if (i != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;

	//if (ctrl->in_type != PATH_IN_DMA)
	//	nx_vip_init_camera(ctrl);
	/* we need power on device of subdev,
	   But we already set format first, so we
	   don't do power on this stage.
	   maybe open video device? */
    //ret = v4l2_subdev_call(ctrl->cam_ex->subdev, core, s_power, 1);
    ret = v4l2_subdev_call(subdev, core, s_power, 1);
    ret = vb2_streamon(vq, i);
   
	printk("capture start=%d\n", ret);    
    if (!ret) 
        ret = v4l2_subdev_call(subdev, video, s_stream, 1); 
    
    if (!ret)
		nxp_stream_on(ctrl);
	
	
	printk("%s exit\n", __func__);
	return 0;
}

int nx_vip_v4l2_streamoff(struct file *file, void *fh, enum v4l2_buf_type i)
{
	struct nx_vip_control *ctrl = video_drvdata(file);//(struct nx_vip_control *) fh;
	struct vb2_queue *vq =  &ctrl->vb2_q;
	struct v4l2_subdev *subdev = ctrl->subdev;	
	int ret = 0;

	if (i != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;

	ret = vb2_streamoff(vq, i);	
	
	ret = v4l2_subdev_call(subdev, video, s_stream, 0);
	if (!ret)
		ret = v4l2_subdev_call(ctrl->subdev, core, s_power, 0);
    if (!ret)
		nxp_stream_off(ctrl);


	/* Now deallocate memory */
	//nx_vip_free_output_memory(&ctrl->out_frame);
//	nx_vip_set_output_address(ctrl);

	return 0;
}

static int nx_vip_v4l2_g_input(struct file *filp, void *fh,
					unsigned int *i)
{
	struct nx_vip_control *ctrl = (struct nx_vip_control *) fh;

	*i = ctrl->v4l2.input->index;

	return 0;
}

static int nx_vip_v4l2_s_input(struct file *filp, void *fh,
					unsigned int i)
{
	struct nx_vip_control *ctrl = (struct nx_vip_control *) fh;

	if (i >= NX_VIP_MAX_INPUT_TYPES)
		return -EINVAL;

	ctrl->v4l2.input = &nx_vip_input_types[i];

	if (nx_vip_input_types[i].type == V4L2_INPUT_TYPE_CAMERA)
		ctrl->in_type = PATH_IN_ITU_CAMERA;
	else
		ctrl->in_type = PATH_IN_DMA;

	return 0;
}

static int nx_vip_v4l2_g_output(struct file *filp, void *fh,
					unsigned int *i)
{
	struct nx_vip_control *ctrl = (struct nx_vip_control *) fh;

	*i = ctrl->v4l2.output->index;

	return 0;
}

static int nx_vip_v4l2_s_output(struct file *filp, void *fh,
					unsigned int i)
{
	struct nx_vip_control *ctrl = (struct nx_vip_control *) fh;

	if (i >= NX_VIP_MAX_OUTPUT_TYPES)
		return -EINVAL;

	ctrl->v4l2.output = &nx_vip_output_types[i];

	if (nx_vip_output_types[i].type == V4L2_OUTPUT_TYPE_MEMORY)
		ctrl->out_type = PATH_OUT_DMA;
	else
		ctrl->out_type = PATH_OUT_LCDFIFO;

	return 0;
}

static int nx_vip_v4l2_enum_input(struct file *filp, void *fh,
					struct v4l2_input *i)
{
	if (i->index >= NX_VIP_MAX_INPUT_TYPES)
		return -EINVAL;

	memcpy(i, &nx_vip_input_types[i->index], sizeof(struct v4l2_input));

	return 0;
}

static int nx_vip_v4l2_enum_output(struct file *filp, void *fh,
					struct v4l2_output *o)
{
	if ((o->index) >= NX_VIP_MAX_OUTPUT_TYPES)
		return -EINVAL;

	memcpy(o, &nx_vip_output_types[o->index], sizeof(struct v4l2_output));

	return 0;
}

static int nx_vip_v4l2_reqbufs(struct file *file, void *fh, struct v4l2_requestbuffers *b)
{
    struct nx_vip_control *ctrl = video_drvdata(file);//(struct nx_vip_control *) fh;
	struct vb2_queue *vq;      
	int ret;
	
	printk("%s enter\n", __func__);
	if (b->memory != V4L2_MEMORY_MMAP) {
		err("V4L2_MEMORY_MMAP is only supported\n");
		return -EINVAL;
	}
	ctrl->cap_frame.nr_frames = b->count;
	vq = &ctrl->vb2_q;
	INIT_LIST_HEAD(&ctrl->bufs);	
	
	ret = vb2_reqbufs(vq, b);

	printk("%s exit\n", __func__);	
	
#if 0
	/* control user input */
	if (b->count > 4)
		b->count = 4;
	else if (b->count < 1)
		b->count = 1;
/* We should allocate framebuffer for read.? */
	if (NULL == ctrl->out_frame.addr[0].virt_y) {
		printk("alloc frame buffer for output, format=%x, width=%d, height=%d\n",
			ctrl->out_frame.format,
			ctrl->out_frame.width,
			ctrl->out_frame.height);
		//ctrl->out_frame.format = FORMAT_YCBCR422;
		//ctrl->out_frame.width = ctrl->in_cam->width;
		//ctrl->out_frame.height = ctrl->in_cam->height;
		ctrl->out_frame.nr_frames = b->count;
		nx_vip_alloc_output_memory(ctrl);
	}
#endif
	return ret;
}

static int nx_vip_v4l2_querybuf(struct file *filp, void *fh,
					struct v4l2_buffer *b)
{
	struct nx_vip_control *ctrl = (struct nx_vip_control *) fh;
	struct nx_vip_out_frame *info = &ctrl->out_frame;

	printk("b->type = %x %x\n", b->type, V4L2_BUF_TYPE_VIDEO_CAPTURE);
	if (b->type != V4L2_BUF_TYPE_VIDEO_OVERLAY &&
		b->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	printk("b->memory = %x %x\n", b->memory, V4L2_MEMORY_MMAP); 
	if (b->memory != V4L2_MEMORY_MMAP)
		return -EINVAL;

	printk("v4l2 querybuf : buf size = %d\n", ctrl->out_frame.buf_size);
	b->length = ctrl->out_frame.buf_size;

	/*
	 * NOTE: we use the m.offset as an index for multiple frames out.
	 * Because all frames are not contiguous, we cannot use it as
	 * original purpose.
	 * The index value used to find out which frame user wants to mmap.
	 */
	b->m.offset = info->addr[b->index].phys_y;
	printk("offset = %x, index = %d\n", b->m.offset, b->index);

	return 0;
}

static int nx_vip_v4l2_qbuf(struct file *file, void *fh,
				struct v4l2_buffer *b)
{
	struct nx_vip_control *ctrl = video_drvdata(file);//(struct nx_vip_control *) fh;
	//struct nx_vip_out_frame *frame = &ctrl->out_frame;
	struct vb2_queue *vq;	
	int index = b->index;	
	int ret;
	printk("%s enter\n", __func__);

	vq = &ctrl->vb2_q;
	if (vq->num_buffers <= 0)
		return -EINVAL;


	ret =  vb2_qbuf(vq, b);
#if 0	
//	if ((b->index >= 0) && (b->index < frame->nr_frames)) {
	if ((b->index >= 0) && (b->index < ctrl->cap_frame.nr_frames)) {
		frame->skip_frames[b->index] = 0;
		return 0;
	}
#endif	
	printk("%s exit\n", __func__);	
	return -EINVAL;
}

static int nx_vip_v4l2_dqbuf(struct file *file, void *fh,
				struct v4l2_buffer *b)
{
	struct nx_vip_control *ctrl = video_drvdata(file);//(struct nx_vip_control *) fh;
	struct nx_video_buffer *nb = &ctrl->cap_frame;
	struct vb2_queue *vq;	
	int index;
	int ret;
	printk("%s enter\n", __func__);
	vq = &ctrl->vb2_q;
//	if (wait_event_interruptible(ctrl->waitq, IS_IRQ_HANDLING(ctrl)))
//		return -ERESTARTSYS;

//	FSET_STOP(ctrl);
	ret = vb2_dqbuf(vq, b, file->f_flags & O_NONBLOCK);	

	printk("%s exit\n", __func__);
	return 0;
}

static int nx_vip_v4l2_cropcap(struct file *filp, void *fh,
					struct v4l2_cropcap *a)
{
	struct nx_vip_control *ctrl = (struct nx_vip_control *) fh;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE &&
		a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) //a->type != V4L2_BUF_TYPE_VIDEO_OVERLAY)
		return -EINVAL;

	/* crop limitations */
	ctrl->v4l2.crop_bounds.left = 0;
	ctrl->v4l2.crop_bounds.top = 0;
	ctrl->v4l2.crop_bounds.width = 640;//ctrl->cam_ex->max_width;
	ctrl->v4l2.crop_bounds.height = 480;//ctrl->cam_ex->max_height;

	/* 
	 * Crop default values,
	 * Always camera native resoultion
	 *
	 */
	ctrl->v4l2.crop_defrect.left = 0;
	ctrl->v4l2.crop_defrect.top = 0;
	ctrl->v4l2.crop_defrect.width = 640;//ctrl->cam_ex->def_width;
	ctrl->v4l2.crop_defrect.height = 480;//ctrl->cam_ex->def_height;

	printk("camera's def value w=%d h=%d\n", ctrl->v4l2.crop_defrect.width, ctrl->v4l2.crop_defrect.height);
	/* set current crop value to default value. */
	ctrl->v4l2.crop_current.left = ctrl->v4l2.crop_defrect.left;
	ctrl->v4l2.crop_current.top = ctrl->v4l2.crop_defrect.top;
	ctrl->v4l2.crop_current.width = ctrl->v4l2.crop_defrect.width;
	ctrl->v4l2.crop_current.height = ctrl->v4l2.crop_defrect.height;

	a->bounds = ctrl->v4l2.crop_bounds;
	a->defrect = ctrl->v4l2.crop_defrect;

	return 0;
}

static int nx_vip_v4l2_g_crop(struct file *filp, void *fh,
				struct v4l2_crop *a)
{
	struct nx_vip_control *ctrl = (struct nx_vip_control *) fh;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE &&
		a->type != V4L2_BUF_TYPE_VIDEO_OVERLAY)
		return -EINVAL;

	a->c = ctrl->v4l2.crop_current;

	return 0;
}

static int nx_vip_v4l2_s_crop(struct file *filp, void *fh,
				struct v4l2_crop *a)
{
	struct nx_vip_control *ctrl = (struct nx_vip_control *) fh;
#if 0
	struct nx_vip_camera_ex *cam = ctrl->cam_ex;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE &&
		a->type != V4L2_BUF_TYPE_VIDEO_OVERLAY)
		return -EINVAL;

	if (a->c.height < 0)
		return -EINVAL;

	if (a->c.width < 0)
		return -EINVAL;

	if ((a->c.left + a->c.width > cam->max_width) || \
		(a->c.top + a->c.height > cam->max_height))
		return -EINVAL;
	
	ctrl->v4l2.crop_current = a->c;

	/*
	if (a->c.left != 0 || a->c.top != 0) {
		printk("c.left=%d c.top=%d\n", a->c.left, a->c.top);
	}*/
#endif
//	if (IS_CAPTURE(ctrl))
//		nx_vip_restart_vip(ctrl);

	return 0;
}

static int nx_vip_v4l2_s_parm(struct file *filp, void *fh,
				struct v4l2_streamparm *a)
{
	struct nx_vip_control *ctrl = (struct nx_vip_control *) fh;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

#if 0 /* We don't have anything to do woth capturemode. */
	if (a->parm.capture.capturemode == V4L2_MODE_HIGHQUALITY) {
		info("changing to max resolution\n");
		nx_vip_change_resolution(ctrl, CAM_RES_MAX);
	} else {
		info("changing to default resolution\n");
		nx_vip_change_resolution(ctrl, CAM_RES_DEFAULT);
	}
#endif

	nx_vip_restart_vip(ctrl);

	return 0;
}

static int nx_vip_v4l2_g_parm(struct file *filp, void *fh,
				struct v4l2_streamparm *a)
{
//	struct nx_vip_control *ctrl = (struct nx_vip_control *) fh;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	a->parm.capture.capturemode =  !V4L2_MODE_HIGHQUALITY;

	return 0;
}


/* Custom I2C control IOCTL */
struct i2c_struc {
	short int subaddr;
	short int val;
};

#define VIDIOC_I2C_CTRL_R	_IOWR('V',100,struct i2c_struc)
#define VIDIOC_I2C_CTRL_W	_IOW('V',101,struct i2c_struc)
#define VIDIOC_READ_ARM		_IOWR('V',102, int)
#define VIDIOC_CAM_PWR_STATUS _IOR('V',103, int)
#define VIDIOC_SHUTDOWN_CAM _IOW('V',104, int)
//#define VIDIOC_CHANGE_CAM   _IOW('V',105, int)
//#define VIDIOC_REQUEST_OTHER _IOW('V',106, int)
#define VIDIOC_LASER_CTRL _IOW('V',107, int)

static long nx_vip_v4l2_default(struct file *file, void *fh,
					int cmd, void *arg)
{
#if 0
    struct nx_vip_control *ctrl = (struct nx_vip_control *) fh;

    //printk("V4L2 default ioctl cmd = %x %x\n", cmd, VIDIOC_I2C_CTRL_W);
    if (cmd == VIDIOC_I2C_CTRL_R) {
        struct i2c_struc *i2cs= arg;
        //printk("v4l2 ctrlr\n");
        //nx_vip_i2c_read(ctrl, (int)i2cs->subaddr, (int *)&i2cs->val);

    } else
    if (cmd == VIDIOC_I2C_CTRL_W) {
        struct i2c_struc *i2cs= arg;
        //printk("v4l2 ctrlw\n");
        //nx_vip_i2c_write(ctrl, (int)i2cs->subaddr, (int)i2cs->val);
    } else
    if (cmd == VIDIOC_READ_ARM) {
        // just arm the read file
        //int *ra = arg;
        //nxp_arm_read(cam, ra);
    } else
    if (cmd == VIDIOC_CAM_PWR_STATUS) {
        int *ra = arg;
        *ra = !ctrl->pwrdn;
    } else
    if (cmd == VIDIOC_SHUTDOWN_CAM) {
        int *ra = arg;
      	if (ctrl->gpio_reset) {
    //  	    ctrl->gpio_reset(*ra);
     // 	    ctrl->pwrdn = *ra;
      	}
    } else
    if (cmd == VIDIOC_LASER_CTRL) {
    	ctrl->laser_ctrl = *((int*)arg);
    } else
        return -EINVAL;
#endif
    return 0;
}

const struct v4l2_ioctl_ops nx_vip_v4l2_ops = {
	.vidioc_querycap		= nx_vip_v4l2_querycap,
	.vidioc_g_fbuf			= nx_vip_v4l2_g_fbuf,
	.vidioc_s_fbuf			= nx_vip_v4l2_s_fbuf,
	.vidioc_enum_fmt_vid_cap	= nx_vip_v4l2_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap		= nx_vip_v4l2_g_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap_mplane	= nx_vip_v4l2_g_fmt_vid_cap_mplane,		
	.vidioc_s_fmt_vid_cap		= nx_vip_v4l2_s_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap_mplane	= nx_vip_v4l2_s_fmt_vid_cap_mplane,		
	.vidioc_try_fmt_vid_cap		= nx_vip_v4l2_try_fmt_vid_cap,
/*	.vidioc_try_fmt_vid_overlay	= nx_vip_v4l2_try_fmt_overlay,
	.vidioc_overlay			= nx_vip_v4l2_overlay,*/
	.vidioc_g_ctrl			= nx_vip_v4l2_g_ctrl,
	.vidioc_s_ctrl			= nx_vip_v4l2_s_ctrl,
	.vidioc_streamon		= nx_vip_v4l2_streamon,
	.vidioc_streamoff		= nx_vip_v4l2_streamoff,
	.vidioc_g_input			= nx_vip_v4l2_g_input,
	.vidioc_s_input			= nx_vip_v4l2_s_input,
	.vidioc_g_output		= nx_vip_v4l2_g_output,
	.vidioc_s_output		= nx_vip_v4l2_s_output,
	.vidioc_enum_input		= nx_vip_v4l2_enum_input,
	.vidioc_enum_output		= nx_vip_v4l2_enum_output,	
	.vidioc_reqbufs			= nx_vip_v4l2_reqbufs,//vb2_ioctl_reqbufs,//nx_vip_v4l2_reqbufs,
	.vidioc_create_bufs		= vb2_ioctl_create_bufs,	
	.vidioc_prepare_buf		= vb2_ioctl_prepare_buf,	
	.vidioc_querybuf		= vb2_ioctl_querybuf,//nx_vip_v4l2_querybuf,
	.vidioc_qbuf			= vb2_ioctl_qbuf,//nx_vip_v4l2_qbuf
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,//nx_vip_v4l2_dqbuf,//vb2_ioctl_dqbuf,//
	.vidioc_expbuf			= vb2_ioctl_expbuf,	
	.vidioc_cropcap			= nx_vip_v4l2_cropcap,
	.vidioc_g_crop			= nx_vip_v4l2_g_crop,
	.vidioc_s_crop			= nx_vip_v4l2_s_crop,
	.vidioc_g_parm          = nx_vip_v4l2_g_parm,
	.vidioc_s_parm			= nx_vip_v4l2_s_parm,
	.vidioc_default         = nx_vip_v4l2_default,
};

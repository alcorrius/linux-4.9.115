/*! linux/drivers/media/video/poa030r.c
 *
 * PixelPlus POA030 CMOS Image Sensor driver
 * Copyright(c) 2014 STcube Inc.,
 * All right reserved by Seungwoo Kim <ksw@stcube.com>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#define DEBUG	1
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/i2c.h>                                          
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/gpio/consumer.h>
#include <linux/of_gpio.h>
#include <asm/io.h>

#include <media/v4l2-common.h>
#include <media/soc_camera.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <linux/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>
#if defined(CONFIG_ARCH_SUNXI)
#else
//#include <linux/regulator/nxe2000-regulator.h>
#endif

#include "poa030r.h"

#ifdef CONFIG_CPU_NXP4330
int cap_i2c_write(u8 chip, u32 addr, int alen, u8 *buffer, int len);
#endif
#define NUM_CTRLS				10

#define V4L2_CID_CAMERA_SCENE_MODE		(V4L2_CTRL_CLASS_CAMERA | 0x1001)
#define V4L2_CID_CAMERA_ANTI_SHAKE		(V4L2_CTRL_CLASS_CAMERA | 0x1002)
#define V4L2_CID_CAMERA_MODE_CHANGE		(V4L2_CTRL_CLASS_CAMERA | 0x1003)
#define V4L2_CID_CAMERA_SELECT			(V4L2_CTRL_CLASS_CAMERA | 0x1004)
#define V4L2_CID_CAMERA_LASER_CTRL		(V4L2_CTRL_CLASS_CAMERA | 0x1005)
#define V4L2_CID_CAMERA_POWER_SAVE		(V4L2_CTRL_CLASS_CAMERA | 0x1006)
#define V4L2_CID_CAMERA_POWER_DOWN		(V4L2_CTRL_CLASS_CAMERA | 0x1007)
#define V4L2_CID_CAMERA_GAIN_BRIGHT		(V4L2_CTRL_CLASS_CAMERA | 0x1008)
#define V4L2_CID_CAMERA_GAIN_DARK		(V4L2_CTRL_CLASS_CAMERA | 0x1009)
#define V4L2_CID_CAMERA_LASER_CTRL_MODE	(V4L2_CTRL_CLASS_CAMERA | 0x100A)
#define V4L2_CID_CAMERA_LASER_ON_DELAY	(V4L2_CTRL_CLASS_CAMERA | 0x100B)
#define V4L2_CID_CAMERA_LASER_OFF_DELAY	(V4L2_CTRL_CLASS_CAMERA | 0x100C)
#define V4L2_CID_CAMERA_EXPOSURE_BRIGHT	(V4L2_CTRL_CLASS_CAMERA | 0x100D)
#define V4L2_CID_CAMERA_EXPOSURE_DARK	(V4L2_CTRL_CLASS_CAMERA | 0x100E)
#define V4L2_CID_CAMERA_CS_WEIGHT       (V4L2_CTRL_CLASS_CAMERA | 0x100F)

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

#define VGA_WIDTH	640
#define VGA_HEIGHT	480
#define QVGA_WIDTH	320
#define QVGA_HEIGHT	240

struct regval_list {
	unsigned short int reg_num;
	unsigned short int value;
};


struct poa030r_state {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct i2c_client *client;
	//const struct poa030r_platform_data *pdata;
	struct v4l2_ctrl_handler handler;
	/* standard */
	struct v4l2_ctrl *focus;
	struct v4l2_ctrl *wb;
	struct v4l2_ctrl *color_effect;
	struct v4l2_ctrl *aemode;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *gain;
	/* custom */
	struct v4l2_ctrl *scene_mode;
	struct v4l2_ctrl *anti_shake;
	struct v4l2_ctrl *mode_change;
	struct v4l2_ctrl *camera_select;
	struct v4l2_ctrl *laser_ctrl;
	struct v4l2_ctrl *power_save;
	struct v4l2_ctrl *power_down;

	int aemode_val;
	int exposure_val;
	int gain_val;
	int laser_ctrl_val;
	int power_save_val;
	int power_down_val;

	bool inited;
	int priv_data;
	int def_width;
	int def_height;
	int width;
	int height;
	int monochrome;
	int def_ae;
	int mode; // PREVIEW or CAPTURE
	struct mutex power_lock;
	int power_count;
	

	/* for zoom */
	struct v4l2_rect crop;

	struct regulator *cam_core_18V;
	struct regulator *cam_io_28V;
	
//	struct gpio_desc *sda;
//	struct gpio_desc *scl;


#ifdef USE_INITIAL_WORKER_THREAD
	struct workqueue_struct *init_wq;
	struct work_struct init_work;
#endif
};

struct regval_list poa030_def_regs[] = {
	/* First, PAD control should be set */
	{POA030_PAD_CONTROL, 0x00}, /* Standby data hiz, normal not hiz */	
	{POA030_PAD_CONTROL2,	0x7F}, /* Clear Bit7 -> Disable OSC pad? */
	{POA030_BAYER_CONTROL_01, 0x07}, /* Mirror control : none */
	{POA030_FORMAT, 0x00}, /* Format control : Y only */ /* 0: CbYCrY, 1: CrYCbY, 2:YCbYCr, 3:YCrYCb */
	{POA030_SYNC_CONTROL_1, 0x00}, /* Polarity : none */
	{POA030_AUTO_CONTROL_1, 0x98}, /* Auto : AWB/AE enable */
	/* QVGA */
	{POA030_WINDOWX1_L, 0x03},
	{POA030_WINDOWY1_L, 0x03},
	{POA030_WINDOWX2_H, 0x01},
	{POA030_WINDOWX2_L, 0x42},
	{POA030_WINDOWY2_H, 0x00},
	{POA030_WINDOWY2_L, 0xF2},
	{POA030_SCALE_X,	 0x40},
	{POA030_SCALE_Y,	 0x40},
	{0x195,			 0x01},
	{0x196,			 0x40},
	{POA030_AE_FWX1_H,	 0x00},
	{POA030_AE_FWX1_L,	 0x03},
	{POA030_AE_FWX2_H,	 0x01},
	{POA030_AE_FWX2_L,	 0x42},
	{POA030_AE_FWY1_H,	 0x00},
	{POA030_AE_FWY1_L,	 0x03},
	{POA030_AE_FWY2_H,	 0x00},
	{POA030_AE_FWY2_L,	 0xF2},
	{POA030_AE_CWX1_H,	 0x00},
	{POA030_AE_CWX1_L,	 0x6D},
	{POA030_AE_CWX2_H,	 0x00},
	{POA030_AE_CWX2_L,	 0xD8},
	{POA030_AE_CWY1_H,	 0x00},
	{POA030_AE_CWY1_L,	 0x53},
	{POA030_AE_CWY2_H,	 0x00},
	{POA030_AE_CWY2_L,	 0xA2},
	/* Now Set CCIR 656 */
	{POA030_SYNC_BLANKSAV, 0xAB}, // 0xAB
	{POA030_SYNC_BLANKEAV, 0xB6}, // 0xB6
	{POA030_SYNC_ACTIVSAV, 0x80},
	{POA030_SYNC_ACTIVEAV, 0x9D},
};

struct regval_list poa030_fmt_yuv422[] = {
	{POA030_FORMAT, 0},
	{POA030_CS_MAX, 0x7F},
	{POA030_Y_CONTRAST, 0x40},
	{POA030_Y_BRIGHTNESS, 0x01},
	{POA030_Y_MAX, 0xFE},
	{POA030_SYNC_CONTROL_0, 0x00},
};

struct regval_list poa030_fmt_rgb444[] = {
	{POA030_FORMAT, 0x30},
	{POA030_CS_MAX, 0x7F},
	{POA030_Y_CONTRAST, 0x40},
	{POA030_Y_BRIGHTNESS, 0x00},
	{POA030_Y_MAX, 0xFE},
	{POA030_SYNC_CONTROL_0, 0x00},
};

struct regval_list poa030_fmt_rgb565[] = {
	{POA030_FORMAT, 0x33},
	{POA030_CS_MAX, 0x7F},
	{POA030_Y_CONTRAST, 0x40},
	{POA030_Y_BRIGHTNESS, 0x00},
	{POA030_Y_MAX, 0xFE},
	{POA030_SYNC_CONTROL_0, 0x00},
};


struct regval_list poa030_fmt_raw[] = {
	{POA030_FORMAT, 0x10},
	{POA030_CS_MAX, 0x7F},
	{POA030_Y_CONTRAST, 0x40},
	{POA030_Y_BRIGHTNESS, 0x00},
	{POA030_Y_MAX, 0xFE},
	{0x0058, 0x00},
	{POA030_SYNC_CONTROL_0, 0x01},
};

/*
 * Store information about the video data format.  The color matrix
 * is deeply tied into the format, so keep the relevant values here.
 * The magic matrix nubmers come from OmniVision.
 */
static struct poa030_format_struct {
	__u8 *desc;
	__u32 pixelformat;
	struct regval_list *regs;
	int nlist;
	int bpp;   /* Bytes per pixel */
} poa030_formats[] = {
	{
		.desc		= "YUYV 4:2:2",
		.pixelformat	= V4L2_PIX_FMT_YUYV,
		.regs 		= poa030_fmt_yuv422,
		.nlist		= ARRAY_SIZE(poa030_fmt_yuv422),
		.bpp		= 2,
	},
	{
		.desc		= "RGB 444",
		.pixelformat	= V4L2_PIX_FMT_RGB444,
		.regs		= poa030_fmt_rgb444,
		.nlist		= ARRAY_SIZE(poa030_fmt_rgb444),
		.bpp		= 2,
	},
	{
		.desc		= "RGB 565",
		.pixelformat	= V4L2_PIX_FMT_RGB565,
		.regs		= poa030_fmt_rgb565,
		.nlist		= ARRAY_SIZE(poa030_fmt_rgb565),
		.bpp		= 2,
	},
	{
		.desc		= "Raw RGB Bayer",
		.pixelformat	= V4L2_PIX_FMT_SBGGR8,
		.regs 		= poa030_fmt_raw,
		.nlist		= ARRAY_SIZE(poa030_fmt_raw),		
		.bpp		= 1
	},
};
#define N_POA030_FMTS ARRAY_SIZE(poa030_formats)

static struct regval_list vga_reg_vals[] = {
	{POA030_WINDOWX1_L, 0x05},
	{POA030_WINDOWY1_L, 0x05},
	{POA030_WINDOWX2_H, 0x02},
	{POA030_WINDOWX2_L, 0x84},
	{POA030_WINDOWY2_H, 0x01},
	{POA030_WINDOWY2_L, 0xE4},
	{POA030_SCALE_X,	 0x20},
	{POA030_SCALE_Y,	 0x20},
	{0x195,			 0x00},
	{0x196,			 0x0A},
	{POA030_AE_FWX1_H,	 0x00},
	{POA030_AE_FWX1_L,	 0x05},
	{POA030_AE_FWX2_H,	 0x02},
	{POA030_AE_FWX2_L,	 0x84},
	{POA030_AE_FWY1_H,	 0x00},
	{POA030_AE_FWY1_L,	 0x05},
	{POA030_AE_FWY2_H,	 0x01},
	{POA030_AE_FWY2_L,	 0xE4},
	{POA030_AE_CWX1_H,	 0x00},
	{POA030_AE_CWX1_L,	 0xDA},
	{POA030_AE_CWX2_H,	 0x01},
	{POA030_AE_CWX2_L,	 0xAF},
	{POA030_AE_CWY1_H,	 0x00},
	{POA030_AE_CWY1_L,	 0xA5},
	{POA030_AE_CWY2_H,	 0x01},
	{POA030_AE_CWY2_L,	 0x44},
#if !defined(CONFIG_CPU_NXP4330)
	{POA030_PAD_CONTROL2, 0},
	{POA030_PAD_CONTROL, 0x00}, /* Standby data hiz, normal not hiz */
	{POA030_PAD_CONTROL2,0}, /* Clear Bit7 -> Disable OSC pad? */
	{POA030_BAYER_CONTROL_01, 0x07}, /* Mirror control : none */
#if defined(CONFIG_ARCH_NXP2120)
	{POA030_FORMAT, 0x01}, /* 0: CbYCrY, 1: CrYCbY, 2:YCbYCr, 3:YCrYCb */
#else
	{POA030_FORMAT, 0x00}, /* 0: CbYCrY, 1: CrYCbY, 2:YCbYCr, 3:YCrYCb */
#endif
	{POA030_SYNC_CONTROL_1, 0x00}, /* Polarity : none */
	{POA030_AUTO_CONTROL_1, 0x98},
#endif
};

static struct regval_list qvga_reg_vals[] = {
	{POA030_WINDOWX1_L, 0x03},
	{POA030_WINDOWY1_L, 0x03},
	{POA030_WINDOWX2_H, 0x01},
	{POA030_WINDOWX2_L, 0x42},
	{POA030_WINDOWY2_H, 0x00},
	{POA030_WINDOWY2_L, 0xF2},
	{POA030_SCALE_X,	 0x40},
	{POA030_SCALE_Y,	 0x40},
	{0x195,			 0x01},
	{0x196,			 0x40},
	{POA030_AE_FWX1_H,	 0x00},
	{POA030_AE_FWX1_L,	 0x03},
	{POA030_AE_FWX2_H,	 0x01},
	{POA030_AE_FWX2_L,	 0x42},
	{POA030_AE_FWY1_H,	 0x00},
	{POA030_AE_FWY1_L,	 0x03},
	{POA030_AE_FWY2_H,	 0x00},
	{POA030_AE_FWY2_L,	 0xF2},
	{POA030_AE_CWX1_H,	 0x00},
	{POA030_AE_CWX1_L,	 0x6D},
	{POA030_AE_CWX2_H,	 0x00},
	{POA030_AE_CWX2_L,	 0xD8},
	{POA030_AE_CWY1_H,	 0x00},
	{POA030_AE_CWY1_L,	 0x53},
	{POA030_AE_CWY2_H,	 0x00},
	{POA030_AE_CWY2_L,	 0xA2},
#if !defined(CONFIG_CPU_NXP4330)
	{POA030_PAD_CONTROL2, 0},
	{POA030_PAD_CONTROL, 0x00}, /* Standby data hiz, normal not hiz */
	{POA030_PAD_CONTROL2,0}, /* Clear Bit7 -> Disable OSC pad? */
	{POA030_BAYER_CONTROL_01, 0x07}, /* Mirror control : none */
#if defined(CONFIG_ARCH_NXP2120)
	{POA030_FORMAT, 0x01},
#else
	{POA030_FORMAT, 0x00}, /* Format control : Y only */ /* 0: CbYCrY, 1: CrYCbY, 2:YCbYCr, 3:YCrYCb */
#endif
	{POA030_SYNC_CONTROL_1, 0x00}, /* Polarity : none */
	{POA030_AUTO_CONTROL_1, 0x98},
#endif
};

static struct poa030_win_size {
	int	width;
	int	height;
	struct regval_list *regs; /* Regs to tweak */
	int nlist;
/* h/vref stuff */
} poa030_win_sizes[] = {
	/* VGA */
	{
		.width		= VGA_WIDTH,
		.height		= VGA_HEIGHT,
		.regs 		= vga_reg_vals,
		.nlist		= ARRAY_SIZE(vga_reg_vals),
	},
	/* QVGA */
	{
		.width		= QVGA_WIDTH,
		.height		= QVGA_HEIGHT,
		.regs 		= qvga_reg_vals,
		.nlist		= ARRAY_SIZE(qvga_reg_vals),
	},
};

int poa030_s_brightness(struct v4l2_subdev *sd, int value);

#define N_WIN_SIZES (ARRAY_SIZE(poa030_win_sizes))

static inline struct poa030r_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct poa030r_state, sd);
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct poa030r_state, handler)->sd;
}

int (*vsync_i2c_write)(unsigned  char);

#define I2C_FLAG_READ	0x10
#if 0 //defined(CONFIG_NXP_CAPTURE_VSYNC_SEQUENCER)
int poa030r_i2c_write(int flag, int bright_gain, int dark_gain, int bright_exposure, int dark_exposure)
{
	int maxval;
	int value;
	unsigned char buf[8], bank;
	int ret;

	printk("%s enter\n", __func__);
	if ((flag % 2)==1){
		buf[0] = (bright_exposure >> 8) & 0xFF;
		buf[1] = (bright_exposure) & 0xFF;;
	}else{
		buf[0] = (dark_exposure >> 8) & 0xFF;
		buf[1] = (dark_exposure) & 0xFF;;
	}

	bank = 2;

	ret = cap_i2c_write(0x6E, POA030_BANK, 1, &bank, 1, sda, scl);
	if (!ret)
		ret = cap_i2c_write(0x6E, POA030_EXT_INTTIME_H, 1, &buf[0], 1, sda, scl);
	if (!ret)
		ret = cap_i2c_write(0x6E, POA030_EXT_INTTIME_M, 1, &buf[1], 1, sda, scl);

	if ((flag % 2)==1){
		value = bright_gain;
	}else{
		value = dark_gain;
	}

	/* Value would be 0 to 100 */
	/*	AE mode 00 01 10 11 */
	maxval = 0x1000 - 0x0100;//3,840
	maxval = maxval * value / 100 + 0x0100;

	buf[0] = (maxval >> 8) & 0xFF;
	buf[1] = maxval & 0xFF;

	if (!ret)
		cap_i2c_write(0x6E, POA030_EXT_GLBG_H, 1, &buf[0], 1, sda, scl);
	if (!ret)
		cap_i2c_write(0x6E, POA030_EXT_GLBG_L, 1, &buf[1], 1, sda, scl);

	return ret;
}
#endif

static int poa030_i2c_xfer(struct i2c_client *client, int reg, char *buf, int num, int tran_flag)
{
	struct i2c_msg msg[2];
	int ret;
	char xbuf[128];

	if (tran_flag & I2C_FLAG_READ) {
		msg[0].addr = client->addr;
		msg[0].len = 1;
		msg[0].buf = (char *)&reg;
		msg[0].flags = 0;

		msg[1].addr = client->addr;
		msg[1].len = num;
		msg[1].buf = buf;
		msg[1].flags = I2C_M_RD;

		ret = i2c_transfer(client->adapter, msg, 2);
	} else {
		xbuf[0] = reg;
		memcpy(&xbuf[1], buf, num);
		msg[0].addr = client->addr;
		msg[0].len = 1 + num;
		msg[0].buf = xbuf;
		msg[0].flags = 0;

		ret = i2c_transfer(client->adapter, msg, 1);
	}

//	printk("xfer flag=%X, ret=%d\n", tran_flag, ret);
	if (ret >= 0)
		return 0;

	return ret;
}

static int reg_page_map_set(struct i2c_client *client, const u16 reg)
{
#if 1
	int ret;
	u8 temp1;
	u8 page;
	u8 bank;
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct poa030r_state *cam = to_state(sd);
	//struct nx_vip_camera *cam = to_vip_cam(sd);
	/* cam->priv_data would be PageMap cache value */
	bank = (reg >> 8) & 0xFF;
	if (bank > 3)
		return -EINVAL;
	if (cam->priv_data != bank) {
		ret = poa030_i2c_xfer(client, POA030_BANK, (u8 *) & bank, 1, 0);
		if (ret >= 0)
			cam->priv_data = bank;
	}
	return 0;
#else
	int ret;
	u8 temp1;
	u8 page;
	u8 bank;
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct poa030r_state *cam = to_state(sd);
	//struct nx_vip_camera *cam = to_vip_cam(sd);
	/* cam->priv_data would be PageMap cache value */

	ret = poa030_i2c_xfer(client, POA030_BANK, (u8 *) &bank, 1, I2C_FLAG_READ);
	if (ret < 0)
		return ret;

	page = (reg >> 8);
	if (page == bank)
		return 0;
	if (page > 3)
		return -EINVAL;

	ret = poa030_i2c_xfer(client, POA030_BANK, (u8 *) & page, 1, 0);
	if (ret >= 0)
		cam->priv_data = page;
	return ret;
#endif
}

static int poa030_read_reg8(struct i2c_client *client, u16	reg, u8 *val)
{
	int ret;
	u8 rval;

	ret = reg_page_map_set(client, reg);
	if (ret < 0) return ret;

	reg &= 0xFF;

	ret = poa030_i2c_xfer(client, reg, (u8 *) &rval, 1, I2C_FLAG_READ);
	if (0 == ret) {
		*val = rval;
		return 0;
	}
	return ret;
}

static int poa030_write_reg8(struct i2c_client *client,u16 reg, u8 val)
{
	u8 temp1;
	int ret;

	//printk("poa030 write %X=%X\n", reg, val);
	temp1 = reg & 0xFF;
	ret = reg_page_map_set(client, reg);
	if (ret < 0) return ret;
	return poa030_i2c_xfer(client, temp1, (u8 *) & val, 1, 0);
}

static int poa030_write_reg8_array(struct i2c_client *client, struct regval_list *regs, int n)
{
	int i, ret;

	for (i=0; i<n; i++, regs++) {
		ret = poa030_write_reg8(client, regs->reg_num, regs->value);
		if (ret) break; // != 1
	}
	return ret;
}

static int poa030_start(struct i2c_client *client)
{
	u8 val;
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct poa030r_state *cam = to_state(sd);
	int i, ret = 0;

	if (!cam->monochrome)
	{
		/* VGA */
		ret = poa030_write_reg8(client, POA030_WINDOWX1_L, 0x05);
		if (ret != 0)
			return ret;
		ret = poa030_write_reg8(client, POA030_WINDOWY1_L, 0x05);
		if (ret != 0)
			return ret;
		poa030_write_reg8(client, POA030_WINDOWX2_H, 0x02);
		poa030_write_reg8(client, POA030_WINDOWX2_L, 0x84);
		poa030_write_reg8(client, POA030_WINDOWY2_H, 0x01);
		poa030_write_reg8(client, POA030_WINDOWY2_L, 0xE4);
		poa030_write_reg8(client, POA030_SCALE_X,	 0x20);
		poa030_write_reg8(client, POA030_SCALE_Y,	 0x20);
		poa030_write_reg8(client, 0x195,			 0x00);
		poa030_write_reg8(client, 0x196,			 0x0A);
		poa030_write_reg8(client, POA030_AE_FWX1_H,	 0x00);
		poa030_write_reg8(client, POA030_AE_FWX1_L,	 0x05);
		poa030_write_reg8(client, POA030_AE_FWX2_H,	 0x02);
		poa030_write_reg8(client, POA030_AE_FWX2_L,	 0x84);
		poa030_write_reg8(client, POA030_AE_FWY1_H,	 0x00);
		poa030_write_reg8(client, POA030_AE_FWY1_L,	 0x05);
		poa030_write_reg8(client, POA030_AE_FWY2_H,	 0x01);
		poa030_write_reg8(client, POA030_AE_FWY2_L,	 0xE4);
		poa030_write_reg8(client, POA030_AE_CWX1_H,	 0x00);
		poa030_write_reg8(client, POA030_AE_CWX1_L,	 0xDA);
		poa030_write_reg8(client, POA030_AE_CWX2_H,	 0x01);
		poa030_write_reg8(client, POA030_AE_CWX2_L,	 0xAF);
		poa030_write_reg8(client, POA030_AE_CWY1_H,	 0x00);
		poa030_write_reg8(client, POA030_AE_CWY1_L,	 0xA5);
		poa030_write_reg8(client, POA030_AE_CWY2_H,	 0x01);
		poa030_write_reg8(client, POA030_AE_CWY2_L,	 0x44);

//#if defined(CONFIG_PLAT_NXP4330_R7) // Top camera(HUV9500) Lens shading & Flicker free setting
		// Flicker free 26MHz
		poa030_write_reg8(client, POA030_FD_PERIOD_A_M,  0x7E);
		poa030_write_reg8(client, POA030_FD_PERIOD_A_L,  0x43);
		poa030_write_reg8(client, POA030_FD_PERIOD_B_M,  0xAB);
		poa030_write_reg8(client, POA030_FD_PERIOD_B_L,  0xF5);
		poa030_write_reg8(client, POA030_FD_PERIOD_C_H,  0x02);
		poa030_write_reg8(client, POA030_FD_PERIOD_C_M,  0xF5);
		poa030_write_reg8(client, 0x01E8,  0x07);
		poa030_write_reg8(client, 0x01E9,  0xD1);
		poa030_write_reg8(client, 0x01EA,  0x06);
		poa030_write_reg8(client, 0x01EB,  0x85);

		// Flicker free enable
		poa030_write_reg8(client, POA030_FLICKER_CONTROL, 0x08);

		// awb rg/bg ratio control
		poa030_write_reg8(client,  0x0206, 0x84);
#if 0
		// awb rg/bg ratio fitting
		poa030_write_reg8(client,  0x026C, 0x81);
		poa030_write_reg8(client,  0x026D, 0x7C);
		poa030_write_reg8(client,  0x026E, 0x7C);
		poa030_write_reg8(client,  0x026F, 0x80);
		poa030_write_reg8(client,  0x0270, 0x80);
		poa030_write_reg8(client,  0x0271, 0x80);
#endif
		// enable lens/cs fitting
		poa030_write_reg8(client,  0x0205, 0x7D);

		// lens gain fitting
		poa030_write_reg8(client,  0x0141, 0x23);
		poa030_write_reg8(client,  0x0142, 0x23);

		poa030_write_reg8(client,  0x0294, 0x2C);
		poa030_write_reg8(client,  0x0295, 0x28);
		poa030_write_reg8(client,  0x0296, 0x2E);
		poa030_write_reg8(client,  0x0297, 0x25);
		poa030_write_reg8(client,  0x0298, 0x2E);
		poa030_write_reg8(client,  0x0299, 0x25);

		// AWB gain control
		poa030_write_reg8(client,  0x0206, 0x84);

		poa030_write_reg8(client,  0x027B, 0x00);
		poa030_write_reg8(client,  0x027C, 0x00);
		poa030_write_reg8(client,  0x027D, 0xFF);
		poa030_write_reg8(client,  0x027E, 0xFF);
		poa030_write_reg8(client,  0x027F, 0x00);
		poa030_write_reg8(client,  0x0280, 0x00);
		poa030_write_reg8(client,  0x0281, 0xFF);
		poa030_write_reg8(client,  0x0282, 0xFF);
		poa030_write_reg8(client,  0x0283, 0x00);
		poa030_write_reg8(client,  0x0284, 0x08);
		poa030_write_reg8(client,  0x0285, 0x00);
		poa030_write_reg8(client,  0x0286, 0x40);

		// Set AWB sampling range
		poa030_write_reg8(client,  0x0256, 0x00);
		poa030_write_reg8(client,  0x0257, 0x80);
		poa030_write_reg8(client,  0x0258, 0x80);
		poa030_write_reg8(client,  0x0259, 0x00);
		poa030_write_reg8(client,  0x025A, 0x80);
		poa030_write_reg8(client,  0x025B, 0x80);
		poa030_write_reg8(client,  0x025C, 0x40);
		poa030_write_reg8(client,  0x025D, 0x00);
		poa030_write_reg8(client,  0x025E, 0x80);
		poa030_write_reg8(client,  0x025F, 0x80);
		poa030_write_reg8(client,  0x0260, 0x00);
		poa030_write_reg8(client,  0x0261, 0x80);
		poa030_write_reg8(client,  0x0262, 0x80);
		poa030_write_reg8(client,  0x0263, 0x40);

		if (cam->def_ae) {
			poa030_write_reg8(client, POA030_AE_UP_SPEED, 0);
			poa030_write_reg8(client, POA030_AE_DOWN_SPEED, 0);
	
			poa030_read_reg8(client, POA030_AUTO_CONTROL_1, &val);
			poa030_write_reg8(client, POA030_AUTO_CONTROL_1, val | 0x02);
	
			poa030_write_reg8(client, POA030_EXT_INTTIME_M, 0x80);
		}

	}
	if (cam->monochrome)
	{
		
		printk("%s : set monochrome mode\n", __func__);
		ret = poa030_write_reg8(client, POA030_FORMAT, 0x44);	/* Format control : Y only */ /* 0: CbYCrY, 1: CrYCbY, 2:YCbYCr, 3:YCrYCb */
		if (ret != 0)
			return ret;
		ret = poa030_write_reg8(client, 0x0058, 0x01);
		if (ret != 0)
			return ret;
		/* QVGA */
		ret = poa030_write_reg8(client, POA030_WINDOWX1_L, 0x03);
		if (ret != 0)
			return ret;
		ret = poa030_write_reg8(client, POA030_WINDOWY1_L, 0x03);
		if (ret != 0)
			return ret;
		poa030_write_reg8(client, POA030_WINDOWX2_H, 0x01);
		poa030_write_reg8(client, POA030_WINDOWX2_L, 0x42);
		poa030_write_reg8(client, POA030_WINDOWY2_H, 0x00);
		poa030_write_reg8(client, POA030_WINDOWY2_L, 0xF2);
		poa030_write_reg8(client, POA030_SCALE_X,	 0x40);
		poa030_write_reg8(client, POA030_SCALE_Y,	 0x40);
		poa030_write_reg8(client, 0x195,			 0x01);
		poa030_write_reg8(client, 0x196,			 0x40);
		poa030_write_reg8(client, POA030_AE_FWX1_H,	 0x00);
		poa030_write_reg8(client, POA030_AE_FWX1_L,	 0x03);
		poa030_write_reg8(client, POA030_AE_FWX2_H,	 0x01);
		poa030_write_reg8(client, POA030_AE_FWX2_L,	 0x42);
		poa030_write_reg8(client, POA030_AE_FWY1_H,	 0x00);
		poa030_write_reg8(client, POA030_AE_FWY1_L,	 0x03);
		poa030_write_reg8(client, POA030_AE_FWY2_H,	 0x00);
		poa030_write_reg8(client, POA030_AE_FWY2_L,	 0xF2);
		poa030_write_reg8(client, POA030_AE_CWX1_H,	 0x00);
		poa030_write_reg8(client, POA030_AE_CWX1_L,	 0x6D);
		poa030_write_reg8(client, POA030_AE_CWX2_H,	 0x00);
		poa030_write_reg8(client, POA030_AE_CWX2_L,	 0xD8);
		poa030_write_reg8(client, POA030_AE_CWY1_H,	 0x00);
		poa030_write_reg8(client, POA030_AE_CWY1_L,	 0x53);
		poa030_write_reg8(client, POA030_AE_CWY2_H,	 0x00);
		poa030_write_reg8(client, POA030_AE_CWY2_L,	 0xA2);

		if (cam->def_ae) {
			poa030_write_reg8(client, POA030_AE_UP_SPEED, 0);
			poa030_write_reg8(client, POA030_AE_DOWN_SPEED, 0);
	
			poa030_read_reg8(client, POA030_AUTO_CONTROL_1, &val);
			poa030_write_reg8(client, POA030_AUTO_CONTROL_1, val | 0x02);
	
			poa030_write_reg8(client, POA030_EXT_INTTIME_H, 0x01);
			poa030_write_reg8(client, POA030_EXT_INTTIME_M, 0x2C);
	
			poa030_s_brightness(sd, 10);
		}
	}

	/* Now Set CCIR 656 */
	poa030_write_reg8(client, POA030_SYNC_BLANKSAV, 0xAB); // 0xAB
	poa030_write_reg8(client, POA030_SYNC_BLANKEAV, 0xB6); // 0xB6
	poa030_write_reg8(client, POA030_SYNC_ACTIVSAV, 0x80);
	poa030_write_reg8(client, POA030_SYNC_ACTIVEAV, 0x9D);

	poa030_write_reg8(client, POA030_PAD_CONTROL2, 0xC0); /* Clear Bit7 -> Disable OSC pad? */
	poa030_write_reg8(client, POA030_PAD_CONTROL, 0x00); /* Standby data hiz, normal not hiz */

#if 0
	for (i=0; i<255; i++) {
		poa030_read_reg8(client, i + 0x0000, &val);
		dev_info(&client->dev, "page A: %x => %x\n", i, val);
	}
	for (i=0; i<255; i++) {
		poa030_read_reg8(client, i + 0x0100, &val);
		dev_info(&client->dev, "page B: %x => %x\n", i, val);
	}
	for (i=0; i<255; i++) {
		poa030_read_reg8(client, i + 0x0200, &val);
		dev_info(&client->dev, "page C: %x => %x\n", i, val);
	}
#endif
	return ret;
}

/*!
 * poa030 get_i2c function
 *
 * @return	none
 */
int poa030_get_i2c(struct i2c_client *client, int addr, int *val)
{
	printk("poa030 get i2c!, addr = 0x%x\n", addr);
	return poa030_read_reg8(client, addr, (u8 *)val);
}

/*!
 * poa030 set_i2c function
 *
 * @return	none
 */
int poa030_set_i2c(struct i2c_client *client, int addr, int val)
{
	printk("poa030 set i2c!, addr = 0x%x val = 0x%x\n", addr, val);
	return poa030_write_reg8(client, addr, val);
}
/*
 * Stuff that knows about the sensor.
 */
int poa030_reset(struct v4l2_subdev *sd, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	printk("%s : enter\n", __func__);
	ret = poa030_write_reg8(client, POA030_SOFTRESET, 0x01); /* Software reset */
	if (ret != 0)
		return ret;
	udelay(1000); /* wait some time */

	return poa030_write_reg8(client, POA030_SOFTRESET, 0x00); /* Clear software reset */
}

int poa030_init(struct v4l2_subdev *sd, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	printk("%s : enter\n", __func__);

	return poa030_start(client);
}

/*!
 * poa030 detect
 *
 * @return 0(OK) or -NODEV
 */
int poa030_detect(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned char val;

	int ret;

	/* read id register and compare to correct value.. */
	ret = poa030_read_reg8(client, POA030_DEVICEID_H, &val);

	if (ret)
		return ret;

	if (val != 0x00A0)
		return -ENODEV;
	
	ret = poa030_read_reg8(client, POA030_DEVICEID_L, &val);
	if (ret)
		return ret;

	if (val != 0x0030)
		return -ENODEV;
	
	return 0;
}

/*!
 * enum_fmt, try_fmt
 * We only support YUV422, 320X240 & 640X480 only.
 *
 * No other modes are tested. So return ERROR if non supported mode specified.
 *
 */
 
int poa030_enum_fmt(struct v4l2_subdev *sd, struct v4l2_fmtdesc *fmt)
{
	struct poa030_format_struct *ofmt;

	if (fmt->index >= N_POA030_FMTS)
		return -EINVAL;

	ofmt = poa030_formats + fmt->index;
	fmt->flags = 0;
	strcpy(fmt->description, ofmt->desc);
	fmt->pixelformat = ofmt->pixelformat;

	return 0;
}


int poa030_try_fmt_internal(struct v4l2_subdev *sd,
		struct v4l2_mbus_framefmt *fmt,
		struct poa030_format_struct **ret_fmt,
		struct poa030_win_size **ret_wsize)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct poa030r_state *cam = to_state(sd);
	int ret;

	printk("poa030 try fmt internal cam(w=%d h=%d), fmt(w=%d h=%d)\n ", cam->width, cam->height, fmt->width, fmt->height);
	//printk("win0 regs=%X,win1 regs=%x\n",poa030_win_sizes[0].regs,poa030_win_sizes[1].regs);
	//printk("win0 regs[0].reg_=%X val=%X\n", poa030_win_sizes[0].regs[0].reg_num, poa030_win_sizes[0].regs[0].value);
	//printk("win0 regs[1].reg_=%X val=%X\n", poa030_win_sizes[0].regs[1].reg_num, poa030_win_sizes[0].regs[1].value);
	//printk("win0 ptr=%X\n", &poa030_win_sizes[0]);
	if (!(fmt->width == 320 && fmt->height == 240) && !(fmt->width == 640 && fmt->height == 480))
		return -EINVAL;

	/*
	 * Round requested image size down to the nearest
	 * we support, but not below the smallest.
	 */
	if ((cam->width != fmt->width) ||
		(cam->height != fmt->height))
	{
		struct poa030_win_size *wsize;

		printk("poa030 winsize hack\n");
		for (wsize = poa030_win_sizes; wsize < poa030_win_sizes + N_WIN_SIZES;
			 wsize++)
			if (fmt->width >= wsize->width && fmt->height >= wsize->height)
				break;
		if (wsize >= poa030_win_sizes + N_WIN_SIZES)
			wsize--;   /* Take the smallest one */
		pr_debug("wsize = %x, poa030 winsize=%x\n",(unsigned int) wsize, (unsigned int)poa030_win_sizes);
		pr_debug("wsize nlist=%d %X\n", wsize->nlist,(unsigned int)wsize->regs);
		ret = poa030_write_reg8_array(client, wsize->regs, wsize->nlist);
		if (ret != 0)
			return ret;

		if (ret_wsize != NULL)
			*ret_wsize = wsize;

		/*
		 * Note the size we'll actually handle.
		 */
		pr_info("wsize w:h=%d:%d\n", wsize->width, wsize->height);
		cam->width = wsize->width;
		cam->height = wsize->height;
	}

	return 0;
}

static int poa030_try_fmt(struct v4l2_subdev *sd, 
             struct v4l2_mbus_framefmt *fmt)
{
  return poa030_try_fmt_internal(sd, fmt, NULL, NULL);
}

/*
 * Set a format.
 */
int poa030_s_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *fmt)
{
	struct poa030_format_struct *ovfmt;
	struct poa030_win_size *wsize;

	return poa030_try_fmt_internal(sd, fmt, &ovfmt, &wsize);
}

/*
 * Implement G/S_PARM.	There is a "high quality" mode we could try
 * to do someday; for now, we just do the frame rate tweak.
 */
int poa030_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	return 0;
}

int poa030_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	return 0;
}


/*
 * Code for dealing with controls.
 */
int poa030_s_sat(struct v4l2_subdev *sd, int value)
{
	return 0;
}

int poa030_g_sat(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}


int poa030_s_hue(struct v4l2_subdev *sd, int value)
{
	return 0;
}


int poa030_g_hue(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}

int poa030_s_brightness(struct v4l2_subdev *sd, int value)
{
	struct poa030r_state *info = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int maxval;
	u8 bank, vv;
	bool capi2c = false;
	int ret;

	printk("%s : value = %d\n", __func__, value);
	
	if (value & 0x40000000) {
		capi2c = true;
		value &= ~0x40000000;
	}
	//return 0;
	/* Value would be 0 to 100 */
	/*	AE mode 00 01 10 11 */
	maxval = 0x1000 - 0x0100;//3,840
	maxval = maxval * value / 100 + 0x0100;
	bank = 2;
	if (capi2c) {
#ifdef CONFIG_CPU_NXP4330
		cap_i2c_write(0x6E, POA030_BANK, 1, &bank, 1);
		vv = maxval;
		cap_i2c_write(0x6E, POA030_EXT_GLBG_L, 1, &vv, 1);
		vv = maxval >> 8;
		cap_i2c_write(0x6E, POA030_EXT_GLBG_L, 1, &vv, 1);
#endif
	} else {
		poa030_write_reg8(client, POA030_EXT_GLBG_L, maxval&0xFF);
		poa030_write_reg8(client, POA030_EXT_GLBG_H, (maxval>>8)&0xFF);
	}

	info->gain_val = value;

	return ret;
}

int poa030_g_brightness(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}

int poa030_s_contrast(struct v4l2_subdev *sd, int value)
{
	return 0;
}

int poa030_g_contrast(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}

int poa030_g_hflip(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}


int poa030_s_hflip(struct v4l2_subdev *sd, int value)
{
	return 0;
}


int poa030_g_vflip(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}


int poa030_s_vflip(struct v4l2_subdev *sd, int value)
{
	return 0;
}

int poa030_g_exposure(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}

int poa030_s_exposure(struct v4l2_subdev *sd, int value)
{
	struct poa030r_state *info = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 vv, bank;
	bool capi2c = false;

	printk("%s : value=%d\n", __func__, value);
	if (value & 0x40000000) {
		capi2c = true;
		value &= ~0x40000000;
	}

	bank = 2;
	if (capi2c) {
#ifdef CONFIG_CPU_NXP4330
		cap_i2c_write(0x6E, POA030_BANK, 1, &bank, 1);
		vv = value >> 8;
		cap_i2c_write(0x6E, POA030_EXT_INTTIME_H, 1, &vv, 1);
		vv = value;
		cap_i2c_write(0x6E, POA030_EXT_INTTIME_M, 1, &vv, 1);
#endif
	} else {
		poa030_write_reg8(client, POA030_EXT_INTTIME_H, (value >> 8) & 0xFF);
		poa030_write_reg8(client, POA030_EXT_INTTIME_M, (value >> 0) & 0xFF);
	}

	info->exposure_val = value;

	return 0;
}

int poa030_g_exposure_auto(struct v4l2_subdev *sd, __s32 *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int val;

	poa030_read_reg8(client, POA030_AUTO_CONTROL_1, (unsigned char *)&val);

	*value = (val | 0x02) == 0 ? 1 : 0;

	return 0;
}

int poa030_s_exposure_auto(struct v4l2_subdev *sd, int value)
{
	struct poa030r_state *info = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int val;

	poa030_read_reg8(client, POA030_AUTO_CONTROL_1, (unsigned char *)&val);

	if (value)
		val &= ~(0x02);	// Auto Exposure
	else
		val |= 0x02;	// Manual Exposure

	poa030_write_reg8(client, POA030_AUTO_CONTROL_1, val);

	info->aemode_val = value;

	return 0;
}

int poa030_g_camera(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}

int poa030_s_camera(struct v4l2_subdev *sd, int value)
{
	struct poa030r_state *info = to_state(sd);

	return 0;
}

int poa030_g_power_save(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}

int poa030_s_power_save(struct v4l2_subdev *sd, int value)
{
	struct poa030r_state *info = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 padval;

    poa030_read_reg8(client,POA030_PAD_CONTROL, &padval);

	if (value)
		poa030_write_reg8(client,POA030_PAD_CONTROL, padval|0x80);
	else
		poa030_write_reg8(client,POA030_PAD_CONTROL, padval&0x7F);

	info->power_save_val = value;

	return 0;
}
int poa030_g_cs_weight(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}

int poa030_s_cs_weight(struct v4l2_subdev *sd, int value)
{
	struct poa030r_state *info = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	poa030_write_reg8(client,POA030_USER_CS, value);

	return 0;
}


int poa030_queryctrl(struct v4l2_subdev *sd,
		struct v4l2_queryctrl *qc)
{
	/* Fill in min, max, step and default value for these controls. */
	switch (qc->id) {
	case V4L2_CID_GAIN:
		return v4l2_ctrl_query_fill(qc, 0, 1024, 1, 20);
	case V4L2_CID_CONTRAST:
		return v4l2_ctrl_query_fill(qc, 0, 127, 1, 64);
	case V4L2_CID_VFLIP:
	case V4L2_CID_HFLIP:
		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);
	case V4L2_CID_SATURATION:
		return v4l2_ctrl_query_fill(qc, 0, 256, 1, 128);
	case V4L2_CID_HUE:
		return v4l2_ctrl_query_fill(qc, -180, 180, 5, 0);
	case V4L2_CID_EXPOSURE_AUTO:
		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);
	case V4L2_CID_EXPOSURE:
		return v4l2_ctrl_query_fill(qc, 0, 0xFFFFFF, 1, 0x8000);
	case V4L2_CID_CAMERA_SELECT:
		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);
	case V4L2_CID_CAMERA_LASER_CTRL:
		return v4l2_ctrl_query_fill(qc, 0, 3, 1, 3);
	case V4L2_CID_CAMERA_POWER_SAVE:
		return  v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);
	case V4L2_CID_CAMERA_POWER_DOWN:
		return  v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);
	}
	return -EINVAL;
}

int poa030_subdev_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		return poa030_g_brightness(sd, &ctrl->value);
	case V4L2_CID_CONTRAST:
		return poa030_g_contrast(sd, &ctrl->value);
	case V4L2_CID_SATURATION:
		return poa030_g_sat(sd, &ctrl->value);
	case V4L2_CID_HUE:
		return poa030_g_hue(sd, &ctrl->value);
	case V4L2_CID_VFLIP:
		return poa030_g_vflip(sd, &ctrl->value);
	case V4L2_CID_HFLIP:
		return poa030_g_hflip(sd, &ctrl->value);
	case V4L2_CID_EXPOSURE_AUTO:
		return poa030_g_exposure_auto(sd, &ctrl->value);
	case V4L2_CID_EXPOSURE:
		return poa030_g_exposure(sd, &ctrl->value);
	case V4L2_CID_CAMERA_SELECT:
		return poa030_g_camera(sd, &ctrl->value);
/*	case V4L2_CID_CAMERA_LASER_CTRL:
		return poa030_g_laser_ctrl(sd, &ctrl->value);
	case V4L2_CID_CAMERA_POWER_SAVE:
		return  poa030_g_power_save(sd, &ctrl->value);
	case V4L2_CID_CAMERA_POWER_DOWN:
		return  poa030_g_power_down(sd, &ctrl->value); */
	case V4L2_CID_CAMERA_CS_WEIGHT:
		return poa030_g_cs_weight(sd, &ctrl->value);
	}
	return -EINVAL;
}

int poa030_subdev_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	//printk("%s : id=%d value=%d\n", __func__, ctrl->id, ctrl->value);
	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		return poa030_s_brightness(sd, ctrl->value);
	case V4L2_CID_CONTRAST:
		return poa030_s_contrast(sd, ctrl->value);
	case V4L2_CID_SATURATION:
		return poa030_s_sat(sd, ctrl->value);
	case V4L2_CID_HUE:
		return poa030_s_hue(sd, ctrl->value);
	case V4L2_CID_VFLIP:
		return poa030_s_vflip(sd, ctrl->value);
	case V4L2_CID_HFLIP:
		return poa030_s_hflip(sd, ctrl->value);
	case V4L2_CID_EXPOSURE_AUTO:
		return poa030_s_exposure_auto(sd, ctrl->value);
	case V4L2_CID_EXPOSURE:
		return poa030_s_exposure(sd, ctrl->value);
	case V4L2_CID_CAMERA_SELECT:
		return poa030_s_camera(sd, ctrl->value);
/*	case V4L2_CID_CAMERA_LASER_CTRL:
		return poa030_s_laser_ctrl(sd, ctrl->value);
	case V4L2_CID_CAMERA_POWER_SAVE:
		return  poa030_s_power_save(sd, ctrl->value);
	case V4L2_CID_CAMERA_POWER_DOWN:
		return  poa030_s_power_down(sd, ctrl->value); */
	case V4L2_CID_CAMERA_CS_WEIGHT:
		return poa030_s_cs_weight(sd, ctrl->value);
	}
	return -EINVAL;
}

int poa030_g_chip_ident(struct v4l2_subdev *sd,
		struct v4l2_dbg_chip_ident *chip)
{
	//struct i2c_client *client = v4l2_get_subdevdata(sd);

	/* POA030 is not registered for V4L2-chip-ident... */
	poa030_detect(sd);
	
	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int poa030_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned char val = 0;
	int ret;
#if 0
	if (!v4l2_chip_match_i2c_client(client, &reg->match))
		return -EINVAL;
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;
#endif
	ret = poa030_read_reg8(client, reg->reg & 0xffff, &val);
	reg->val = val;
	reg->size = 1;
	return ret;
}

static int poa030_s_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
#if 0
	if (!v4l2_chip_match_i2c_client(client, &reg->match))
		return -EINVAL;
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;
#endif
	poa030_write_reg8(client, reg->reg & 0xffff, reg->val & 0xff);
	return 0;
}
#endif

int poa030_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev	*sd = ctrl_to_sd(ctrl);

	//printk("%s : id=%d value=%d\n", __func__, ctrl->id, ctrl->val);

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		return poa030_s_brightness(sd, ctrl->val);
	case V4L2_CID_CONTRAST:
		return poa030_s_contrast(sd, ctrl->val);
	case V4L2_CID_SATURATION:
		return poa030_s_sat(sd, ctrl->val);
	case V4L2_CID_HUE:
		return poa030_s_hue(sd, ctrl->val);
	case V4L2_CID_VFLIP:
		return poa030_s_vflip(sd, ctrl->val);
	case V4L2_CID_HFLIP:
		return poa030_s_hflip(sd, ctrl->val);
	case V4L2_CID_EXPOSURE_AUTO:
		return poa030_s_exposure_auto(sd, ctrl->val);
	case V4L2_CID_EXPOSURE:
		return poa030_s_exposure(sd, ctrl->val);
	case V4L2_CID_CAMERA_SELECT:
		return poa030_s_camera(sd, ctrl->val);
/*	case V4L2_CID_CAMERA_LASER_CTRL:
		return poa030_s_laser_ctrl(sd, ctrl->val);
	case V4L2_CID_CAMERA_POWER_SAVE:
		return  poa030_s_power_save(sd, ctrl->val);
	case V4L2_CID_CAMERA_POWER_DOWN:
		return  poa030_s_power_down(sd, ctrl->val); */
	}
	return -EINVAL;
}

static const struct v4l2_ctrl_ops poa030r_ctrl_ops = {
	.s_ctrl = poa030_s_ctrl,
};

enum {
	WB_AUTO = 0,
	WB_DAYLIGHT,
	WB_CLOUDY,
	WB_FLUORESCENT,
	WB_INCANDESCENT,
	WB_MAX
};

enum {
	COLORFX_NONE = 0,
	COLORFX_SEPIA,
	COLORFX_AQUA,
	COLORFX_MONO,
	COLORFX_NEGATIVE,
	COLORFX_SKETCH,
	COLORFX_MAX
};

enum {
	SCENE_OFF = 0,
	SCENE_PORTRAIT,
	SCENE_LANDSCAPE,
	SCENE_SPORTS,
	SCENE_NIGHTSHOT,
	SCENE_MAX
};

enum {
	ANTI_SHAKE_OFF = 0,
	ANTI_SHAKE_50Hz,
	ANTI_SHAKE_60Hz,
	ANTI_SHAKE_MAX
};

#define MIN_EXPOSURE			-4
#define MAX_EXPOSURE			4

static const struct v4l2_ctrl_config poa030r_custom_ctrls[] = {
    {
        .ops    = &poa030r_ctrl_ops,
        .id     = V4L2_CID_CAMERA_SCENE_MODE,
        .type   = V4L2_CTRL_TYPE_INTEGER,
        .name   = "SceneMode",
        .min    = 0,
        .max    = SCENE_MAX - 1,
        .def    = 0,
        .step   = 1,
    }, {
        .ops    = &poa030r_ctrl_ops,
        .id     = V4L2_CID_CAMERA_ANTI_SHAKE,
        .type   = V4L2_CTRL_TYPE_INTEGER,
        .name   = "AntiShake",
        .min    = 0,
        .max    = ANTI_SHAKE_MAX - 1,
        .def    = 0,
        .step   = 1,
    }, {
        .ops    = &poa030r_ctrl_ops,
        .id     = V4L2_CID_CAMERA_MODE_CHANGE,
        .type   = V4L2_CTRL_TYPE_INTEGER,
        .name   = "ModeChange",
        .min    = 0,
        .max    = 1,
        .def    = 0,
        .step   = 1,
    }, {
        .ops    = &poa030r_ctrl_ops,
        .id     = V4L2_CID_CAMERA_SELECT,
        .type   = V4L2_CTRL_TYPE_INTEGER,
        .name   = "CameraSelect",
        .min    = 0,
        .max    = 1,
        .def    = 1,
        .step   = 1,
    }, {
        .ops    = &poa030r_ctrl_ops,
        .id     = V4L2_CID_CAMERA_LASER_CTRL,
        .type   = V4L2_CTRL_TYPE_INTEGER,
        .name   = "LaserCtrl",
        .min    = 0,
        .max    = 3,
        .def    = 3,
        .step   = 1,
    }, {
		.ops    = &poa030r_ctrl_ops,
		.id     = V4L2_CID_CAMERA_POWER_SAVE,
		.type   = V4L2_CTRL_TYPE_INTEGER,
		.name   = "PowerSave",
		.min    = 0,
		.max    = 1,
		.def    = 0,
		.step   = 1,
    }, {
		.ops    = &poa030r_ctrl_ops,
		.id     = V4L2_CID_CAMERA_POWER_DOWN,
		.type   = V4L2_CTRL_TYPE_INTEGER,
		.name   = "PowerDown",
		.min    = 0,
		.max    = 1,
		.def    = 0,
		.step   = 1,
	}, {
		.ops    = &poa030r_ctrl_ops,
		.id     = V4L2_CID_EXPOSURE,
		.type   = V4L2_CTRL_TYPE_INTEGER,
		.name   = "Exposure",
		.min    = 0,
		.max    = 0x40000000 | 0xFFFFFF,
		.def    = 0,
		.step   = 1,
    },
};

int poa030r_initialize_ctrls(struct poa030r_state *state)
{
    v4l2_ctrl_handler_init(&state->handler, sizeof(poa030r_custom_ctrls)/sizeof(struct v4l2_ctrl_config));

    /* standard */
    state->gain = v4l2_ctrl_new_std(&state->handler, &poa030r_ctrl_ops,
            V4L2_CID_GAIN, 0, 0x40000000 | 4096, 1, 0);
    if (!state->gain) {
        pr_err("%s: failed to create gain ctrl\n", __func__);
        return -1;
    }
    state->focus = v4l2_ctrl_new_std(&state->handler, &poa030r_ctrl_ops,
            V4L2_CID_FOCUS_AUTO, 0, 1, 1, 0);
    if (!state->focus) {
        pr_err("%s: failed to create focus ctrl\n", __func__);
        return -1;
    }
    state->wb = v4l2_ctrl_new_std(&state->handler, &poa030r_ctrl_ops,
            V4L2_CID_DO_WHITE_BALANCE, WB_AUTO, WB_MAX - 1, 1, WB_AUTO);
    if (!state->wb) {
        pr_err("%s: failed to create wb ctrl\n", __func__);
        return -1;
    }
    state->color_effect = v4l2_ctrl_new_std_menu(&state->handler, &poa030r_ctrl_ops,
            V4L2_CID_COLORFX, COLORFX_MAX - 1, 0, COLORFX_NONE);
    if (!state->color_effect) {
        pr_err("%s: failed to create color_effect ctrl\n", __func__);
        return -1;
    }
    state->aemode = v4l2_ctrl_new_std_menu(&state->handler, &poa030r_ctrl_ops,
            V4L2_CID_EXPOSURE_AUTO, MAX_EXPOSURE, 0, 0);
    if (!state->aemode) {
        pr_err("%s: failed to create exposure ctrl\n", __func__);
        return -1;
    }

    /* custom */
    state->scene_mode = v4l2_ctrl_new_custom(&state->handler, &poa030r_custom_ctrls[0], NULL);
    if (!state->scene_mode) {
        pr_err("%s: failed to create scene_mode ctrl\n", __func__);
        return -1;
    }
    state->anti_shake = v4l2_ctrl_new_custom(&state->handler, &poa030r_custom_ctrls[1], NULL);
    if (!state->anti_shake) {
        pr_err("%s: failed to create anti_shake ctrl\n", __func__);
        return -1;
    }
    state->mode_change = v4l2_ctrl_new_custom(&state->handler, &poa030r_custom_ctrls[2], NULL);
    if (!state->mode_change) {
        pr_err("%s: failed to create mode_change ctrl\n", __func__);
        return -1;
    }

    state->camera_select = v4l2_ctrl_new_custom(&state->handler, &poa030r_custom_ctrls[3], NULL);
    if (!state->camera_select) {
        pr_err("%s: failed to create camera_select ctrl\n", __func__);
        return -1;
    }

    state->laser_ctrl = v4l2_ctrl_new_custom(&state->handler, &poa030r_custom_ctrls[4], NULL);
    if (!state->laser_ctrl) {
        pr_err("%s: failed to create laser ctrl\n", __func__);
        return -1;
    }

    state->power_save = v4l2_ctrl_new_custom(&state->handler, &poa030r_custom_ctrls[5], NULL);
    if (!state->power_save) {
        pr_err("%s: failed to create power save ctrl\n", __func__);
        return -1;
    }

    state->power_down = v4l2_ctrl_new_custom(&state->handler, &poa030r_custom_ctrls[6], NULL);
    if (!state->power_down) {
        pr_err("%s: failed to create power down ctrl\n", __func__);
        return -1;
    }

    state->exposure = v4l2_ctrl_new_custom(&state->handler, &poa030r_custom_ctrls[7], NULL);
    if (!state->exposure) {
        pr_err("%s: failed to create exposure down ctrl\n", __func__);
        return -1;
    }

    state->sd.ctrl_handler = &state->handler;
    if (state->handler.error) {
        printk("%s: ctrl handler error(%d)\n", __func__, state->handler.error);
        v4l2_ctrl_handler_free(&state->handler);
        return -1;
    }
    return 0;
}
#if 0
void poa030_regulator_enable(int enable)
{
	struct regulator *cam_io_28V = NULL;
    struct regulator *cam_core_15V = NULL;

    cam_core_15V = regulator_get(NULL, "NXE2000_LDO1");
    if (IS_ERR(cam_core_15V)) {
        printk(KERN_ERR "%s: failed to regulator_get() for vcam_1.5V", __func__);
        return;
    }

    cam_io_28V = regulator_get(NULL, "NXE2000_LDO7");
    if (IS_ERR(cam_io_28V)) {
        printk(KERN_ERR "%s: failed to regulator_get() for vvid_2.8V", __func__);
        return;
    }

    printk("%s: %d\n", __func__, enable);
    if (enable) {
        regulator_enable(cam_core_15V);
        regulator_enable(cam_io_28V);
    } else {
        regulator_disable(cam_io_28V);
        regulator_disable(cam_core_15V);
    }

    regulator_put(cam_io_28V);
    regulator_put(cam_core_15V);
}
#endif
int poa030_suspend(struct v4l2_subdev *subdev)
{
	struct poa030r_state *state = to_state(subdev);

	pr_debug("+%s\n", __func__);

    pr_debug("-%s\n", __func__);

	return 0;
}

static int poa030_resume(struct v4l2_subdev *subdev)
{
	struct poa030r_state *state = to_state(subdev);

	pr_debug("+%s\n", __func__);

	poa030_reset(subdev, 0);
	poa030_init(subdev, 0);
	//poa030_s_brightness(subdev, state->gain->cur.val);
	//poa030_s_exposure_auto(subdev, state->exposure->cur.val);
	//poa030_s_laser_ctrl(subdev, state->laser_ctrl->cur.val);
	//poa030_s_exposure(subdev, state->exposure->cur.val);

    pr_debug("-%s\n", __func__);

    return 0;
}

static int poa030_set_power(struct v4l2_subdev *subdev, int on)
{
	struct poa030r_state *poa030 = to_state(subdev);
	struct i2c_client *client = v4l2_get_subdevdata(subdev);
	int ret = 0;

	pr_info("%s: on=%d, power_count=%d\n", __func__, on, poa030->power_count);

	mutex_lock(&poa030->power_lock);

	/* If the power count is modified from 0 to != 0 or from != 0 to 0,
	 * update the power state.
	 */
	if (poa030->power_count == !on) {
		if (on) {
			ret = poa030_resume(subdev);
			if (ret) {
				dev_err(&client->dev,
					"Failed to resume the sensor: %d\n", ret);
				goto done;
			}
		} else {
			poa030_suspend(subdev);
		}
	}

	/* Update the power count. */
	poa030->power_count = on ? 1 : 0;
	WARN_ON(poa030->power_count < 0);

done:
	mutex_unlock(&poa030->power_lock);

	pr_info("%s: on=%d, power_count=%d\n", __func__, on, poa030->power_count);
	return ret;
}

static int poa030_get_fmt(struct v4l2_subdev *sd,
								struct v4l2_subdev_pad_config *cfg, struct v4l2_subdev_format *fmt)
{
	return 0;
}

static int poa030_set_fmt(struct v4l2_subdev *sd,
								struct v4l2_subdev_pad_config *cfg, struct v4l2_subdev_format *fmt)
{
	int i,err = 0;
	struct v4l2_mbus_framefmt *_fmt = &fmt->format;
                                                            
	//printk("%s: %dx%d\n", __func__, _fmt->width, _fmt->height);

	return poa030_s_fmt(sd, _fmt);
}

static int poa030_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int i2c_id = i2c_adapter_id(client->adapter);

	// Do nothing now, but may have code to start/stop camera
	if (enable) {
		printk("poa030 start!!\n");
		//poa030_start(client);
	}

	return 0;
}


/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops poa030_core_ops = {
/*	.g_chip_ident = poa030_g_chip_ident,*/
/*	.g_ctrl = poa030_subdev_g_ctrl,*/
/*	.s_ctrl = poa030_subdev_s_ctrl,*/
/*	.queryctrl = poa030_queryctrl,*/
	.s_power = poa030_set_power,
	.reset = poa030_reset,
	.init = poa030_init,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = poa030_g_register,
	.s_register = poa030_s_register,
#endif
};

static const struct v4l2_subdev_video_ops poa030_video_ops = {
//	.enum_fmt = poa030_enum_fmt,
//	.try_fmt = poa030_try_fmt,
//	.s_fmt = poa030_s_fmt,
/*	.try_mbus_fmt = poa030_try_fmt,*/
/*	.s_mbus_fmt = poa030_s_fmt,	*/
	.s_parm = poa030_s_parm,
	.g_parm = poa030_g_parm,
	.s_stream = poa030_s_stream,
};

static const struct v4l2_subdev_pad_ops poa030_pad_ops = {
	.get_fmt = poa030_get_fmt,
	.set_fmt = poa030_set_fmt,
};

static const struct v4l2_subdev_ops poa030_ops = {
	.core = &poa030_core_ops,
	.video = &poa030_video_ops,
	.pad = &poa030_pad_ops,
};

#if defined(CONFIG_MEDIA_CONTROLLER)
/**
 * media_entity_operations
 */
static int _link_setup(struct media_entity *entity,
							const struct media_pad *local,
							const struct media_pad *remote, u32 flags)
{
	/* printk("%s: entered\n", __func__); */
	return 0;
}

static const struct media_entity_operations poa030_media_ops = {
	.link_setup = _link_setup,
};
#endif
/* ----------------------------------------------------------------------- */
//static struct poa030r_platform_data *poa030_get_pdata(struct i2c_client *client)

static int poa030_parse_dt(struct device_node *np, struct poa030r_state *info)
{
	int err;
	int val;
	
	err = of_property_read_u32(np, "monochrome", &val); 
	if (IS_ERR(err) || (0 != err)) {
		info->monochrome = 0;
	} else {
		info->monochrome = val;
	}
	err = of_property_read_u32(np, "def_exposure", &val); 
	if (IS_ERR(err) || (0 != err)) {
		info->exposure_val = 0;
	} else {
		info->exposure_val = val;
	}
	err = of_property_read_u32(np, "def_width", &val);
	if (IS_ERR(err) || (0 != err)) {
		info->def_width = 640;
	} else {
		info->def_width = val;
		info->width = val;
	}
	err = of_property_read_u32(np, "def_height", &val);
	if (IS_ERR(err) || (0 != err)) {
		info->def_height = 480;
	} else {
		info->def_height = val;
		info->height = val;
	}
	err = of_property_read_u32(np, "def_ae", &val);
	if (IS_ERR(err) || (0 != err)) {
		info->def_ae = 0;
	} else {
		info->def_ae = val;
	}
	
	printk("mono:%d, def_exp=%d, def_ae=%d\n", info->monochrome, info->exposure_val, info->def_ae);
	
	return 0;
}

static int poa030_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	//const struct poa030r_platform_data *pdata = poa030_get_pdata(client); 
	struct v4l2_subdev *sd;
	struct poa030r_state *info;
	struct clk *clk;
	int ret = 0;
	int detected = 0;
	struct device_node *np;

	//if(!pdata){
	//	dev_err(&client->dev, "platform data not specfied\n");
	//	return -EINVAL;
	//}
	
	info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	
	//info->pdata = pdata;
	info->client = client;
	
	//poa030_get_of_data(info);
	
	//clk = devm_clk_get(&client->dev, "clk");
	
	sd = &info->sd;
	v4l2_i2c_subdev_init(sd, client, &poa030_ops);
	
	printk("client=%X\n", client);

	/* Make sure it's an poa030 */
	//if (strstr(id->name, "_2"))
	ret = poa030_detect(sd);
	if (ret) {
		v4l_info(client, "chip not found @ 0x%02x (%s)\n",
				client->addr << 1, client->adapter->name);
		kfree(info);
		return ret;
	}
	else
	{
		v4l_info(client, "chip found @ 0x%02x (%s)\n",
				client->addr << 1, client->adapter->name);
		detected = 1;
	}

	mutex_init(&info->power_lock);

	info->power_count = 1;

	info->aemode_val = 0;
	//info->exposure_val = info->id == 1 ? 0x012C : 0x80;
	info->gain_val = 0;
	
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	info->pad.flags = MEDIA_PAD_FL_SOURCE;

#if defined(CONFIG_MEDIA_CONTROLLER)
//	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sd->entity.ops = &poa030_media_ops;
	if (media_entity_pads_init(&sd->entity, 1, &info->pad)) {
		printk("media_entity_init error!");
		dev_err(&client->dev, "%s: failed to media_entity_init()\n", __func__);
		kfree(info);
		return -ENOENT;
	}
#endif

	printk("POA030R name = %s, s_power=%p subdev=%p\n", id->name, sd->ops->core->s_power, sd);
	
	vsync_i2c_write = NULL;

	//nx_vip_register_subdev(sd);
	if (detected)
	{
		ret = poa030r_initialize_ctrls(info);
		if (ret < 0) {
			pr_err("%s: failed to initialize controls\n", __func__);
			kfree(info);
			return ret;
		}
		info->priv_data = -1; /* page map value */
		np = client->dev.of_node;
		poa030_parse_dt(np, info);
		//poa030_regulator_enable(1);
		poa030_reset(sd, 0);
		udelay(1000); /* wait some time */
		poa030_start(client);
		info->inited = 1;
#if 0 //defined(CONFIG_NXP_CAPTURE_VSYNC_SEQUENCER)
		vsync_i2c_write = poa030r_i2c_write;
#endif
	}
	ret = v4l2_async_register_subdev(&info->sd);
	if(ret)
		v4l2_ctrl_handler_free(&info->handler);
	
	dev_info(&client->dev, "%s sensor driver registered !!\n", sd->name);

	return 0;
}

static int poa030_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_device_unregister_subdev(sd);
	kfree(to_state(sd));
	return	0;
}

static const struct i2c_device_id poa030_id[] = {
	{ "poa030", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, poa030_id);

#if defined(CONFIG_OF)
static const struct of_device_id poa030_of_match[]={
	{.compatible = "pixelplus,poa030",},
	{},
};
MODULE_DEVICE_TABLE(of,poa030_of_match);
#endif

static struct i2c_driver poa030_driver = {
	.driver = {
		.name = "poa030",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(poa030_of_match),
	},
	.id_table	  = poa030_id,
	.probe		  = poa030_probe,
	.remove		  = poa030_remove,
	//.command	  = poa030_command,
};

module_i2c_driver(poa030_driver);


MODULE_AUTHOR("Seungwoo Kim<ksw@i4vine.com>");
MODULE_DESCRIPTION("POA030R Camera Driver");
MODULE_LICENSE("GPL");


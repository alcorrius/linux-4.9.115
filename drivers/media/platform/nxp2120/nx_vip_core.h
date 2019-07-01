/* linux/drivers/media/video/nexell/nx_vip.h
 *
 * Header file for Nexell's camera(VIP) driver.
 * Copyright (c) 2019 I4VINE
 *
 * Copyright (c) 2011 MOSTiTECH co., ltd.
 * All right reserved by Seungwoo Kim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __NX_VIP_CAMIF_H__
#define __NX_VIP_CAMIF_H__

#ifdef __KERNEL__
#include <linux/wait.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>

#endif
//#include <mach/devices.h>
//#include <mach/platform.h>

/*
 * PIXEL FORMAT GUIDE
 *
 * The 'x' means 'DO NOT CARE'
 * The '*' means 'FIMC SPECIFIC'
 * For some fimc formats, we couldn't find equivalent format in the V4L2 FOURCC.
 *
 * FIMC TYPE    PLANES  ORDER       V4L2_PIX_FMT
 * ---------------------------------------------------------
 * RGB565   x   x           V4L2_PIX_FMT_RGB565
 * RGB888   x   x           V4L2_PIX_FMT_RGB24
 * YUV420   2   LSB_CBCR    V4L2_PIX_FMT_NV12
 * YUV420   2   LSB_CRCB    V4L2_PIX_FMT_NV21
 * YUV420   2   MSB_CBCR    V4L2_PIX_FMT_NV21X*
 * YUV420   2   MSB_CRCB    V4L2_PIX_FMT_NV12X*
 * YUV420   3   x           V4L2_PIX_FMT_YUV420
 * YUV422   1   YCBYCR      V4L2_PIX_FMT_YUYV
 * YUV422   1   YCRYCB      V4L2_PIX_FMT_YVYU
 * YUV422   1   CBYCRY      V4L2_PIX_FMT_UYVY
 * YUV422   1   CRYCBY      V4L2_PIX_FMT_VYUY*
 * YUV422   2   LSB_CBCR    V4L2_PIX_FMT_NV16*
 * YUV422   2   LSB_CRCB    V4L2_PIX_FMT_NV61*
 * YUV422   2   MSB_CBCR    V4L2_PIX_FMT_NV16X*
 * YUV422   2   MSB_CRCB    V4L2_PIX_FMT_NV61X*
 * YUV422   3   x           V4L2_PIX_FMT_YUV422P
 *
*/

/*
 * COMMON DEFINITIONS
 *
*/
#define NX_VIP_NAME "nx-vip"
#define USE_VB2		0

#define info(args...)   do { printk(KERN_INFO NX_VIP_NAME ": " args); } while (0)
#define err(args...)    do { printk(KERN_ERR  NX_VIP_NAME ": " args); } while (0)

#define NX_VIP_FRAME_SKIP       0
#define NX_VIP_FRAME_TAKE       1

#define NX_VIP_MAX_CTRLS        2 //3
#define NX_VIP_MAX_FRAMES       4

/* including 1 more for test pattern */
#define NX_VIP_MAX_CAMS     4
#define NX_VIP_TPID         (NX_VIP_MAX_CAMS - 1)


#define NX_VIP_ZOOM_PIXELS      32

#define NX_VIP_CROP_DEF_WIDTH       352
#define NX_VIP_CROP_DEF_HEIGHT  272

/* S flag: global status flags */
#define NX_VIP_FLAG_RUNNING     0x0001
#define NX_VIP_FLAG_STOP        0x0002
#define NX_VIP_FLAG_HANDLE_IRQ  0x0004
#define NX_VIP_STA_MASK     0x000f

/* U flag: use case flags */
#define NX_VIP_FLAG_PREVIEW     0x0010
#define NX_VIP_FLAG_CAPTURE     0x0020
#define NX_VIP_USE_MASK     0x00f0

/* I flag: IRQ flags */
#define NX_VIP_FLAG_IRQ_NORMAL  0x0100
#define NX_VIP_FLAG_IRQ_X       0x0200
#define NX_VIP_FLAG_IRQ_Y       0x0400
#define NX_VIP_FLAG_IRQ_LAST        0x0800
#define NX_VIP_IRQ_MASK     0x0f00
/* C flag : scaler flags */
#define NX_VIP_FLAG_SCALER_IRQ	0x1000
#define NX_VIP_SCALER_MASK		0xf000

#if 0
#define UNMASK_STATUS(x)        (x->flag &= ~NX_VIP_STA_MASK)
#define UNMASK_USAGE(x)         (x->flag &= ~NX_VIP_USE_MASK)
#define UNMASK_IRQ(x)           (x->flag &= ~NX_VIP_IRQ_MASK)
#define UNMASK_SCALER(x)		(x->flag &= ~NX_VIP_SCALER_MASK)

#define FSET_RUNNING(x)         UNMASK_STATUS(x); (x->flag |= NX_VIP_FLAG_RUNNING)
#define FSET_STOP(x)            UNMASK_STATUS(x); (x->flag |= NX_VIP_FLAG_STOP)
#define FSET_HANDLE_IRQ(x)      UNMASK_STATUS(x); (x->flag |= NX_VIP_FLAG_HANDLE_IRQ)

#define FSET_SCALER_IRQ(x)      UNMASK_SCALER(x); (x->flag |= NX_VIP_FLAG_SCALER_IRQ)

#define FSET_PREVIEW(x)         UNMASK_USAGE(x); (x->flag |= NX_VIP_FLAG_PREVIEW)
#define FSET_CAPTURE(x)         UNMASK_USAGE(x); (x->flag |= NX_VIP_FLAG_CAPTURE)

#define FSET_IRQ_NORMAL(x)      UNMASK_IRQ(x); (x->flag |= NX_VIP_FLAG_IRQ_NORMAL)
#define FSET_IRQ_X(x)           UNMASK_IRQ(x); (x->flag |= NX_VIP_FLAG_IRQ_X)
#define FSET_IRQ_Y(x)           UNMASK_IRQ(x); (x->flag |= NX_VIP_FLAG_IRQ_Y)
#define FSET_IRQ_LAST(x)        UNMASK_IRQ(x); (x->flag |= NX_VIP_FLAG_IRQ_LAST)

#define IS_RUNNING(x)           (x->flag & NX_VIP_FLAG_RUNNING)
#define IS_IRQ_HANDLING(x)      (x->flag & NX_VIP_FLAG_HANDLE_IRQ)

#define IS_SCALER_HANDLING(x)   (x->flag & NX_VIP_FLAG_SCALER_IRQ)

#define IS_PREVIEW(x)           (x->flag & NX_VIP_FLAG_PREVIEW)
#define IS_CAPTURE(x)           (x->flag & NX_VIP_FLAG_CAPTURE)

#define IS_IRQ_NORMAL(x)        (x->flag & NX_VIP_FLAG_IRQ_NORMAL)
#define IS_IRQ_X(x)         (x->flag & NX_VIP_FLAG_IRQ_X)
#define IS_IRQ_Y(x)         (x->flag & NX_VIP_FLAG_IRQ_Y)
#define IS_IRQ_LAST(x)          (x->flag & NX_VIP_FLAG_IRQ_LAST)
#endif
#define PAT_CB(x)           ((x >> 8) & 0xff)
#define PAT_CR(x)           (x & 0xff)


/*
 * E N U M E R A T I O N S
 *
*/

enum nx_vip_order422_cam_t {
    CAM_ORDER422_8BIT_YCBYCR = (0 << 14),
    CAM_ORDER422_8BIT_YCRYCB = (1 << 14),
    CAM_ORDER422_8BIT_CBYCRY = (2 << 14),
    CAM_ORDER422_8BIT_CRYCBY = (3 << 14),
};

enum nx_vip_order422_in_t {
    IN_ORDER422_CRYCBY = (0 << 4),
    IN_ORDER422_YCRYCB = (1 << 4),
    IN_ORDER422_CBYCRY = (2 << 4),
    IN_ORDER422_YCBYCR = (3 << 4),
};

enum nx_vip_order422_out_t {
    OUT_ORDER422_YCBYCR = (0 << 0),
    OUT_ORDER422_YCRYCB = (1 << 0),
    OUT_ORDER422_CBYCRY = (2 << 0),
    OUT_ORDER422_CRYCBY = (3 << 0),
};

enum nx_vip_2plane_order_t {
    LSB_CBCR = 0,
    LSB_CRCB = 1,
    MSB_CRCB = 2,
    MSB_CBCR = 3,
};

enum nx_vip_itu_cam_ch_t {
    ITU_CAM_A = 1,
    ITU_CAM_B = 0,
};

enum nx_vip_scan_t {
    SCAN_TYPE_PROGRESSIVE   = 0,
    SCAN_TYPE_INTERLACE = 1,
};

enum nx_vip_format_t {
    FORMAT_RGB565,
    FORMAT_RGB666,
    FORMAT_RGB888,
    FORMAT_YCBCR420,
    FORMAT_YCBCR422,
    FORMAT_YCBCR444,
};

enum nx_vip_flip_t {
	NX_VIP_FLIP_FLAG_NONE = 0,
	NX_VIP_FLIP_FLAG_VFLIP = 1,
	NX_VIP_FLIP_FLAG_HFLIP = 2,
};

enum nx_vip_path_in_t {
    PATH_IN_ITU_CAMERA,
    PATH_IN_MIPI_CAMERA,
    PATH_IN_DMA,
};

enum nx_vip_path_out_t {
    PATH_OUT_DMA,
    PATH_OUT_LCDFIFO,
};

enum nx_vip_effect_t {
    EFFECT_ORIGINAL = (0 << 26),
    EFFECT_ARBITRARY = (1 << 26),
    EFFECT_NEGATIVE = (2 << 26),
    EFFECT_ARTFREEZE = (3 << 26),
    EFFECT_EMBOSSING = (4 << 26),
    EFFECT_SILHOUETTE = (5 << 26),
};

enum nx_vip_wb_t {
    WB_AUTO = 0,
    WB_INDOOR_3001 = 1,
    WB_OUTDOOR_5100 = 2,
    WB_INDOOR_2000 = 3,
    WB_HALT = 4,
    WB_CLOUDY = 5,
    WB_SUNNY = 6,
};

enum nx_vip_i2c_cmd_t {
    I2C_CAM_INIT,
    I2C_CAM_RESOLUTION,
    I2C_CAM_WB,
    I2C_CAM_BRIGHTNESS,
    I2C_CAM_POWER_SAVE,
    I2C_CAM_EXPOSURE,
    I2C_CAM_FIXED_FRAME,
    I2C_CAM_EXPOSURE_AUTO,
    I2C_CAM_GAIN,
};

enum nx_vip_cam_res_t {
    CAM_RES_DEFAULT,
    CAM_RES_QQVGA,
    CAM_RES_QVGA,
    CAM_RES_QSVGA,
    CAM_RES_VGA,
    CAM_RES_SVGA,
    CAM_RES_XGA,
    CAM_RES_SXGA,
    CAM_RES_UXGA,
    CAM_RES_MAX,
};

struct NX_VIP_RegisterSet
{
	volatile u16	VIP_CONFIG;			// 0x000 : VIP Configuration Register
	volatile u16	VIP_HVINT;			// 0x002 : VIP Interrupt Control Register
	volatile u16	VIP_SYNCCTRL;		// 0x004 : VIP Sync Control Register
	volatile u16	VIP_SYNCMON;		// 0x006 : VIP Sync Monitor Register
	volatile u16	VIP_VBEGIN;			// 0x008 : VIP Vertical Sync Start Register
	volatile u16	VIP_VEND;			// 0x00A : VIP Vertical Sync End Register
	volatile u16	VIP_HBEGIN;			// 0x00C : VIP Horizontal Sync Start Register
	volatile u16	VIP_HEND;			// 0x00E : VIP Horizontal Sync End Register
	volatile u16	VIP_FIFOCTRL;		// 0x010 : VIP FIFO Control Register
	volatile u16	VIP_HCOUNT;			// 0x012 : VIP Horizontal Counter Register
	volatile u16	VIP_VCOUNT;			// 0x014 : VIP Vertical Counter Register
	volatile u8		__Reserved00[0x200-0x16];	// 0x016 ~ 0x1FF
	volatile u16	VIP_CDENB;			// 0x200 : VIP Clipper & Decimator Enable Register
	volatile u16	VIP_ODINT;			// 0x202 : VIP Operation Done Interrupt Register
	volatile u16	VIP_IMGWIDTH;		// 0x204 : VIP Image Width Register
	volatile u16	VIP_IMGHEIGHT;		// 0x206 : VIP Image Height Register
	volatile u16	CLIP_LEFT;			// 0x208 : VIP Clipper Left Register
	volatile u16	CLIP_RIGHT;			// 0x20A : VIP Clipper Right Register
	volatile u16	CLIP_TOP;			// 0x20C : VIP Clipper Top Register
	volatile u16	CLIP_BOTTOM;		// 0x20E : VIP Clipper Bottom Register
	volatile u16	DECI_TARGETW;		// 0x210 : VIP Decimator Target Width Register
	volatile u16	DECI_TARGETH;		// 0x212 : VIP Decimator Target Height Register
	volatile u16	DECI_DELTAW;		// 0x214 : VIP Decimator Delta Width Register
	volatile u16	DECI_DELTAH;		// 0x216 : VIP Decimator Delta Height Register
	volatile s16	DECI_CLEARW;		// 0x218 : VIP Decimator Clear Width Register
	volatile s16	DECI_CLEARH;		// 0x21A : VIP Decimator Clear Height Register
	volatile u16	DECI_LUSEG;			// 0x21C : VIP Decimator Lu Segment Register
	volatile u16	DECI_CRSEG;			// 0x21E : VIP Decimator Cr Segment Register
	volatile u16	DECI_CBSEG;			// 0x220 : VIP Decimator Cb Segment Register
	volatile u16	DECI_FORMAT;		// 0x222 : VIP Decimator NX_VIP_FORMAT Register
	volatile u16	DECI_ROTFLIP;		// 0x224 : VIP Decimator Rotation & Flip Register
	volatile u16	DECI_LULEFT;		// 0x226 : VIP Decimator Lu Left Register
	volatile u16	DECI_CRLEFT;		// 0x228 : VIP Decimator Cr Left Register
	volatile u16	DECI_CBLEFT;		// 0x22A : VIP Decimator Cb Left Register
	volatile u16	DECI_LURIGHT;		// 0x22C : VIP Decimator Lu Right Register
	volatile u16	DECI_CRRIGHT;		// 0x22E : VIP Decimator Cr Right Register
	volatile u16	DECI_CBRIGHT;		// 0x230 : VIP Decimator Cb Right Register
	volatile u16	DECI_LUTOP;			// 0x232 : VIP Decimator Lu Top Register
	volatile u16	DECI_CRTOP;			// 0x234 : VIP Decimator Cr Top Register
	volatile u16	DECI_CBTOP;			// 0x236 : VIP Decimator Cb Top Register
	volatile u16	DECI_LUBOTTOM;		// 0x238 : VIP Decimator Lu Bottom Register
	volatile u16	DECI_CRBOTTOM;		// 0x23A : VIP Decimator Cr Bottom Register
	volatile u16	DECI_CBBOTTOM;		// 0x23C : VIP Decimator Cb Bottom Register
	volatile u16	CLIP_LUSEG;			// 0x23E : VIP Clipper Lu Segment Register
	volatile u16	CLIP_CRSEG;			// 0x240 : VIP Clipper Cr Segment Register
	volatile u16	CLIP_CBSEG;			// 0x242 : VIP Clipper Cb Segment Register
	volatile u16	CLIP_FORMAT;		// 0x244 : VIP Clipper Format Register
	volatile u16	CLIP_ROTFLIP;		// 0x246 : VIP Clipper Rotation & Flip Register
	volatile u16	CLIP_LULEFT;		// 0x248 : VIP Clipper Lu Left Register
	volatile u16	CLIP_CRLEFT;		// 0x24A : VIP Clipper Cr Left Register
	volatile u16	CLIP_CBLEFT;		// 0x24C : VIP Clipper Cb Left Register
	volatile u16	CLIP_LURIGHT;		// 0x24E : VIP Clipper Lu Right Register
	volatile u16	CLIP_CRRIGHT;		// 0x250 : VIP Clipper Cr Right Register
	volatile u16	CLIP_CBRIGHT;		// 0x252 : VIP Clipper Cb Right Register
	volatile u16	CLIP_LUTOP;			// 0x254 : VIP Clipper Lu Top Register
	volatile u16	CLIP_CRTOP;			// 0x256 : VIP Clipper Cr Top Register
	volatile u16	CLIP_CBTOP;			// 0x258 : VIP Clipper Cb Top Register
	volatile u16	CLIP_LUBOTTOM;		// 0x25A : VIP Clipper Lu Bottom Register
	volatile u16	CLIP_CRBOTTOM;		// 0x25C : VIP Clipper Cr Bottom Register
	volatile u16	CLIP_CBBOTTOM;		// 0x25E : VIP Clipper Cb Bottom Register
	volatile u16	VIP_SCANMODE;		// 0x260 : VIP Clipper & Decimator Scan Mode Register
	volatile u16	CLIP_YUYVENB;		// 0x262 : VIP Clipper Linear YUYV Enable Register
	volatile u16	CLIP_BASEADDRH;		// 0x264 : VIP Clipper Linear Base Address High Register
	volatile u16	CLIP_BASEADDRL;		// 0x266 : VIP Clipper Linear Base Address Low Register
	volatile u16	CLIP_STRIDEH;		// 0x268 : VIP Clipper Linear Stride High Register
	volatile u16	CLIP_STRIDEL;		// 0x26A : VIP Clipper Linear Stride Low Register
	volatile u16	VIP_VIP1;			// 0x26C : VIP SIP Enable Register
	volatile u8		__Reserved01[0x7C0-0x26E];	// 0x26E ~ 0xFBF :
	volatile u32	VIPCLKENB;			// 0xFC0 : VIP Clock Generation Enable Register
	volatile u32	VIPCLKGEN[2][2];	// 0xFC4 : VIP Clock Generation Control Register
};

struct NX_SCALER_RegisterSet
{
	volatile u32	SCRUNREG;				///< 0x00 : Scaler Configuration Register
	volatile u32	SCCFGREG;				///< 0x04 : Scaler Configuration Register1
	volatile u32	SCINTREG;				///< 0x08 : Scaler Interrupt Register
	volatile u32	SCSRCADDREG;			///< 0x0C : Scaler Source Address Register
	volatile u32	SCSRCSIZEREG;			///< 0x10 : Scaler Source Size Register
	volatile u32	SCDESTADDREG;			///< 0x14 : Scaler Destination Register
	volatile u32	SCDESTSIZEREG;			///< 0x18 : Scaler Destination Size Register
	volatile u32	DELTAXREG;				///< 0x1C : Scaler Delta X Register
	volatile u32	DELTAYREG;				///< 0x20 : Scaler Delta Y Register
	volatile u32	HVSOFTREG;				///< 0x24 : Horizontal and Vertical Filter Ratio Register
	volatile u32	__Reserved00[(0x7C0-0x28)/4];	///< 0x28 ~ 0x7BC : Reserved region
	volatile u32	CLKENB;					///< 0x7C0 : Clock Enable Register
};

/**
 * struct nx_video_fmt - the driver's internal color format data
 * @name: format description
 * @pixelformat:
 * @mbus_code: Media Bus pixel code, -1 if not applicable
 * @num_planes:
 * @num_sw_planes:
 * @depth: bpp
 * @is_separated:
 * @flags: flags indicating which operation mode format applies to
 */
struct nx_video_fmt {
	char *name;
	u32 pixelformat;
	u32 mbus_code;	 // ksw: this is deprecated...
	u32 num_planes;
	u32 num_sw_planes;
	u16	depth[VIDEO_MAX_PLANES];
	bool is_separated;
	u16 flags;
};




/*
 * NX VIP STRUCTURES
 *
*/

/*
 * struct nx_vip_frame_addr
 * @phys_rgb:   physical start address of rgb buffer
 * @phys_y: physical start address of y buffer
 * @phys_cb:    physical start address of u buffer
 * @phys_cr:    physical start address of v buffer
 * @virt_y: virtual start address of y buffer
 * @virt_rgb:   virtual start address of rgb buffer
 * @virt_cb:    virtual start address of u buffer
 * @virt_cr:    virtual start address of v buffer
*/
struct nx_vip_frame_addr {
    union {
        dma_addr_t  phys_rgb;
        dma_addr_t  phys_y;
    };

    dma_addr_t      phys_cb;
    dma_addr_t      phys_cr;

    union {
        u8      *virt_rgb;
        u8      *virt_y;
    };

    u8          *virt_cb;
    u8          *virt_cr;

    unsigned int lu_seg;
    unsigned int lu_left;
    unsigned int lu_top;
    unsigned int lu_right;
    unsigned int lu_bottom;
    unsigned int cb_seg;
    unsigned int cb_left;
    unsigned int cb_top;
    unsigned int cb_right;
    unsigned int cb_bottom;
    unsigned int cr_seg;
    unsigned int cr_left;
    unsigned int cr_right;
    unsigned int cr_top;
    unsigned int cr_bottom;
    // for scaler
    unsigned int src_addr_lu;
    unsigned int src_addr_cb;
    unsigned int src_addr_cr;
    unsigned int dst_addr_lu;
    unsigned int dst_addr_cb;
    unsigned int dst_addr_cr;
    unsigned char *vir_addr_lu;
    unsigned char *vir_addr_cb;
    unsigned char *vir_addr_cr;
};

/*
 * struct nx_vip_window_offset
 * @h1: left side offset of source
 * @h2: right side offset of source
 * @v1: upper side offset of source
 * @v2: lower side offset of source
*/
struct nx_vip_window_offset {
    int h1;
    int h2;
    int v1;
    int v2;
};

/*
 * struct nx_vip_dma_offset
 * @y_h:    y value horizontal offset
 * @y_v:    y value vertical offset
 * @cb_h:   cb value horizontal offset
 * @cb_v:   cb value vertical offset
 * @cr_h:   cr value horizontal offset
 * @cr_v:   cr value vertical offset
 *
*/
struct nx_vip_dma_offset {
    int y_h;
    int y_v;
    int cb_h;
    int cb_v;
    int cr_h;
    int cr_v;
};

/*
 * struct nx_vip_polarity
 * @pclk:   1 if PCLK polarity is inverse
 * @vsync:  1 if VSYNC polarity is inverse
 * @href:   1 if HREF polarity is inverse
 * @hsync:  1 if HSYNC polarity is inverse
*/
struct nx_vip_polarity {
    u32 pclk;
    u32 vsync;
    u32 href;
    u32 hsync;
};

/*
 * struct nx_vip_effect
 * @type:   effect type
 * @pat_cb: cr value when type == arbitrary
 * @pat_cR: cr value when type == arbitrary
 *
*/
struct nx_vip_effect {
    enum nx_vip_effect_t    type;
    u8          pat_cb;
    u8          pat_cr;
};

/*
 * struct nx_vip_camera: abstraction for input camera
 * @sd:			struct v4l2_subdev
 * @id:         cam id (0-2)
 * @type:       type of camera (ITU or MIPI)
 * @mode:       mode of input source
 * @order422:       YCBCR422 order
 * @clockrate:      camera clockrate
 * @width:      source width
 * @height:     source height
 * @offset:     offset information
 * @polarity        polarity information
 * @reset_type:     reset type (high or low)
 * @reset_delay:    delay time in microseconds (udelay)
 * @client:     i2c client
 * @initialized:    whether already initialized
*/
struct nx_vip_camera_ex {
	struct v4l2_subdev *subdev;
    int             id;
    int				cam_id; /* proprietary camera id for distinguish cameras. */
    enum nx_vip_order422_cam_t  order422;
//    u32             clockrate;
    int             cur_width;
    int             cur_height;
    int				def_width;
    int				def_height;
    int             max_width;
    int             max_height;
#if 0    
    unsigned int	cur_pixformat;
    int				bpp;
    int				flip_flag;
    int				enable;
    //struct nx_vip_window_offset offset;
    struct nx_vip_polarity  polarity;
    int             initialized;

    int				cur_brightness;
    int				cur_gain;
    int				cur_aemode;
    int				cur_exposure;

    struct i2c_client *client;
    int				priv_data;

    int             (*i2c_read)(struct i2c_client *client,int addr, int *val);
    int             (*i2c_write)(struct i2c_client *client,int addr, int val);
#endif
};

#if 0
/*
 * inline function to get v4l2_subdev to nx_vip_camera structure.
 *
*/
static inline struct nx_vip_camera *to_vip_cam(struct v4l2_subdev *sd)
{
	return container_of(sd, struct nx_vip_camera, sd);
}
#endif

/*
 * struct nx_vip_in_frame: abstraction for frame data
 * @addr:       address information of frame data
 * @width:      width
 * @height:     height
 * @offset:     dma offset
 * @format:     pixel format
 * @planes:     YCBCR planes (1, 2 or 3)
 * @order_1p        1plane YCBCR order
 * @order_2p:       2plane YCBCR order
 * @flip:       flip mode
*/
struct nx_vip_in_frame {
    u32             buf_size;
    struct nx_vip_frame_addr    addr;
    int             width;
    int             height;
    struct nx_vip_dma_offset    offset;
    enum nx_vip_format_t        format;
    int             planes;
    enum nx_vip_order422_in_t   order_1p;
    enum nx_vip_2plane_order_t  order_2p;
    enum nx_vip_flip_t      flip;
};

/*
 * struct nx_vip_out_frame: abstraction for frame data
 * @cfn:        current frame number
 * @buf_size:       1 buffer size
 * @addr[]:     address information of frames
 * @nr_frams:       how many output frames used
 * @skip_frames:    current streamed frames (for capture)
 * @width:      width
 * @height:     height
 * @offset:     offset for output dma
 * @format:     pixel format
 * @planes:     YCBCR planes (1, 2 or 3)
 * @order_1p        1plane YCBCR order
 * @order_2p:       2plane YCBCR order
 * @scan:       output scan method (progressive, interlace)
 * @flip:       flip mode
 * @effect:     output effect
*/
struct nx_vip_out_frame {
    int             cfn;
    u32             buf_size;
    struct nx_vip_frame_addr    addr[NX_VIP_MAX_FRAMES];
    int             nr_frames;
    int             skip_frames[NX_VIP_MAX_FRAMES];
    int				startx, starty;
    int             width;
    int             height;
    int				scw;
    int				sch;
    struct nx_vip_dma_offset    offset;
    enum nx_vip_format_t        format;
    int             planes;
    enum nx_vip_order422_out_t  order_1p;
    enum nx_vip_2plane_order_t  order_2p;
    enum nx_vip_scan_t      scan;
    enum nx_vip_flip_t      flip;
    struct nx_vip_effect        effect;
};

/*
 * struct nx_vip_v4l2
*/
struct nx_vip_v4l2 {
    struct v4l2_fmtdesc *fmtdesc;
    struct v4l2_framebuffer frmbuf;
    struct v4l2_input   *input;
    struct v4l2_output  *output;
    struct v4l2_rect    crop_bounds;
    struct v4l2_rect    crop_defrect;
    struct v4l2_rect    crop_current;
};


/**
 * struct nx_video_addr - the NEXELL physical address set for DMA
 * @y:	 luminance plane physical address
 * @cb:	 Cb plane physical address
 * @cr:	 Cr plane physical address
 */
struct nx_video_addr {
	u32 y;
	u32 cr;
	u32 cb;
};

/**
 * struct nx_video_dma_offset - pixel offset information for DMA
 * @y_h:	y value horizontal offset
 * @y_v:	y value vertical offset
 * @cb_h:	cb value horizontal offset
 * @cb_v:	cb value vertical offset
 * @cr_h:	cr value horizontal offset
 * @cr_v:	cr value vertical offset
 */
struct nx_video_dma_offset {
	int y_h;
	int y_v;
	int cb_h;
	int cb_v;
	int cr_h;
	int cr_v;
};

/**
 * struct nx_video_frame - source/target frame properties
 * @f_width:	image full width (virtual screen size)
 * @f_height:	image full height (virtual screen size)
 * @o_width:	original image width as set by S_FMT
 * @o_height:	original image height as set by S_FMT
 * @offs_h:	image horizontal pixel offset
 * @offs_v:	image vertical pixel offset
 * @width:	image pixel width
 * @height:	image pixel weight
 * @payload:	image size in bytes (w x h x bpp)
 * @paddr:	image frame buffer physical addresses
 * @dma_offset:	DMA offset in bytes
 * @fmt:	fimc color format pointer
 * @nr_frams:       how many output frames used
 * @skip_frames:    current streamed frames (for capture)
 * @addr[]:     address information of frames
 * @planes:     YCBCR planes (1, 2 or 3)
 */
struct nx_video_frame {
	u32	f_width;
	u32	f_height;
	u32	o_width;
	u32	o_height;
	u32	offs_h;
	u32	offs_v;
	u32	width;
	u32	height;
	u32 scw;
	u32 sch;
    int	nr_frames;
    int planes;
	unsigned long		size[VIDEO_MAX_PLANES];
	u16 stride[VIDEO_MAX_PLANES];
	u16 heights[VIDEO_MAX_PLANES];
    int             skip_frames[NX_VIP_MAX_FRAMES];
    struct nx_vip_frame_addr    addr[NX_VIP_MAX_FRAMES];
	struct nx_video_addr	paddr;
	struct nx_video_dma_offset	dma_offset;
	struct nx_video_fmt	 *fmt;
	u8			alpha;
};


static inline void set_frame_bounds(struct nx_video_frame *f, u32 width, u32 height)
{
	f->o_width  = width;
	f->o_height = height;
	f->f_width  = width;
	f->f_height = height;
}

static inline void set_frame_crop(struct nx_video_frame *f,
				  u32 left, u32 top, u32 width, u32 height)
{
	f->offs_h = left;
	f->offs_v = top;
	f->width  = width;
	f->height = height;
}


struct nx_vip_dt_data {
	int		id;
	u32		dma_mem;
	u32		dma_mem_size;
/*	int		dpc_device;	*/	/* display out module */
	int		hor_align;
	int		ver_align;
	int		buff_count;		/* alloc buffer count */
	int		skip_count;		/* first frame skip count */
	
	/* with MACH's configure, laser, and camera devices are selected. */
/*	int		laser_ctrl;	*/	/* laser control from MACH */
	int		time_stamp_on;  /* time stamp on from MACH */

//	struct nx_vip_camera_info *cam;
	u32		data_order;
    u32     external_sync;
    u32     h_frontporch;
    u32     h_syncwidth;
    u32     h_backporch;
    u32     v_frontporch;
    u32     v_backporch;
    u32     v_syncwidth;
    bool	clock_invert;    
    bool   interlace;    
    u32		power_enable;    
    int 	max_width;
    int 	max_height;   
    int 	def_width;
    int 	def_height;
    struct gpio_desc *gpio_reset;   
};

struct nx_vip_control;
/*
 * struct nx_scaler
 */
struct nx_scaler {
	struct NX_SCALER_RegisterSet   *regs;
	int							irq;
	unsigned					offset;
    int							done;
    int							started;
    int							current_bank;
    struct nx_vip_control		*current_ctrl;
    wait_queue_head_t			waitq;
};

struct nx_video_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
	union {
		struct nx_video_addr	paddr;
		u32 dma_addr[3];
	};
	union {
		struct nx_video_addr	vaddr;
		u32 vir_addr[3];
	};
	int stride[3];	
};

/*
 * struct nx_vip_control: abstraction for NXP2120 VIP controller
 * @id:     id number (= minor number)
 * @name:   name for video_device
 * @flag:   status, usage, irq flag (S, U, I flag)
 * @lock:   mutex lock
 * @waitq:  waitqueue
 * @pdata:  platform data
 * @clock:  fimc clock
 * @regs:   virtual address of SFR
 * @in_use: 1 when resource is occupied
 * @irq:    irq number
 * @vd:     video_device
 * @v4l2:   v4l2 info
 * @scaler: scaler related information
 * @in_type:    type of input
 * @in_cam: camera structure pointer if input is camera else null
 * @in_frame:   frame structure pointer if input is dma else null
 * @out_type:   type of output
 * @out_frame:  frame structure pointer if output is dma
 * @rot90:  1 if clockwise 90 degree for output
 *
 * @open_lcdfifo:   function pointer to open lcd fifo path (display driver)
 * @close_lcdfifo:  function pointer to close fifo path (display driver)
*/
struct nx_vip_control {
    /* general */
	void   *priv;
	struct platform_device *pdev;	
    struct device              *dev;    
    int                        id;
    char                       name[16];
    u32                        flag;
    struct mutex               lock;
	spinlock_t 					slock;  
	spinlock_t  				irqlock;	
    wait_queue_head_t           waitq;
    struct clk                 *clock;
    struct NX_VIP_RegisterSet  *regs;
	struct nx_vip_clk *clk_base;    

//   struct nx_platform_vip     *pdata;
    struct nx_vip_dt_data		vip_dt_data;
//    struct nx_vip_camera_ex    *cam_ex;
   	struct v4l2_async_subdev *asd[1]; 
	struct v4l2_async_notifier notifier;   	
    //struct NX_SCALER_RegisterSet   *sc_regs;
    //unsigned					scaler_offset;
    //int							sc_done;
    //int							sc_started;
    //int							scaler_bank;
    atomic_t                  in_use;
    int                       irq;
    int                       source_sel;
    struct v4l2_device 	   	  v4l2_dev;
	struct v4l2_subdev	*subdev;    
    struct video_device       vdev;
    struct nx_vip_v4l2        v4l2;
    struct nx_scaler      	  *scaler;
    int                       use_clipper;
    int                       use_scaler;
    int                       streamon;
#if USE_VB2
    struct nx_video_buffer	*cur_frm;
    //struct nx_video_buffer *next_frm;
	struct vb2_queue	vb2_q;
	struct list_head	bufs;
#endif
	struct v4l2_format fmt;	
    /* input */
    enum nx_vip_path_in_t     in_type;
    //struct nx_vip_camera      *in_cam;
    //struct v4l2_subdev        *cam_subdev;
    struct nx_vip_in_frame    in_frame;

    unsigned int             gpio_base; /* Not used as not DUAL CAMERA configuration */
    //struct nx_vip_camera      *in_cam2;

    /* output */
    enum nx_vip_path_out_t   out_type;
    struct nx_vip_out_frame  out_frame;
    struct nx_video_frame	cap_frame;
    int                      buf_index;
    int                      pwrdn;
//	int                      (*gpio_reset)(int reset);
	int						 gpio_reset;
	/* time stamp */
	int                      time_stamp_on;
	int                      time_stamp_offset;
	unsigned long			 time_stamp[NX_VIP_MAX_FRAMES];

	/* laser on/off */
    int                      laser_ctrl;
    int                      auto_laser_ctrl;
};

/*
 * struct nx_vip_config
*/
struct nx_vip_config {
    struct nx_vip_control   ctrl[NX_VIP_MAX_CTRLS];
//    struct nx_vip_camera_ex camera[NX_VIP_MAX_CAMS];
    struct nx_scaler        scaler;
};

/*
 * V 4 L 2   F I M C   E X T E N S I O N S
 *
*/
#define V4L2_INPUT_TYPE_MEMORY      10
#define V4L2_OUTPUT_TYPE_MEMORY     20
#define V4L2_OUTPUT_TYPE_LCDFIFO    21

#define FORMAT_FLAGS_PACKED     1
#define FORMAT_FLAGS_PLANAR     2

#define V4L2_FMT_IN         0
#define V4L2_FMT_OUT            1

#define TPATTERN_COLORBAR       1
#define TPATTERN_HORIZONTAL     2
#define TPATTERN_VERTICAL       3

/* FOURCC for FIMC specific */
#define V4L2_PIX_FMT_NV12X      v4l2_fourcc('N', '1', '2', 'X')
#define V4L2_PIX_FMT_NV21X      v4l2_fourcc('N', '2', '1', 'X')
#define V4L2_PIX_FMT_NV16X      v4l2_fourcc('N', '1', '6', 'X')
#define V4L2_PIX_FMT_NV61X      v4l2_fourcc('N', '6', '1', 'X')
/*
#define V4L2_PIX_FMT_VYUY       v4l2_fourcc('V', 'Y', 'U', 'Y')
#define V4L2_PIX_FMT_NV16       v4l2_fourcc('N', 'V', '1', '6')
#define V4L2_PIX_FMT_NV61       v4l2_fourcc('N', 'V', '6', '1')
*/

/* CID extensions */
#define V4L2_CID_ACTIVE_CAMERA      (V4L2_CID_PRIVATE_BASE + 0)
#define V4L2_CID_NR_FRAMES      (V4L2_CID_PRIVATE_BASE + 1)
#define V4L2_CID_RESET          (V4L2_CID_PRIVATE_BASE + 2)
//#define V4L2_CID_TEST_PATTERN       (V4L2_CID_PRIVATE_BASE + 3)
#define V4L2_CID_SCALER_BYPASS      (V4L2_CID_PRIVATE_BASE + 4)
#define V4L2_CID_JPEG_INPUT     (V4L2_CID_PRIVATE_BASE + 5)
#define V4L2_CID_OUTPUT_ADDR        (V4L2_CID_PRIVATE_BASE + 10)
#define V4L2_CID_INPUT_ADDR     (V4L2_CID_PRIVATE_BASE + 20)
#define V4L2_CID_INPUT_ADDR_RGB     (V4L2_CID_PRIVATE_BASE + 21)
#define V4L2_CID_INPUT_ADDR_Y       (V4L2_CID_PRIVATE_BASE + 22)
#define V4L2_CID_INPUT_ADDR_CB      (V4L2_CID_PRIVATE_BASE + 23)
#define V4L2_CID_INPUT_ADDR_CBCR    (V4L2_CID_PRIVATE_BASE + 24)
#define V4L2_CID_INPUT_ADDR_CR      (V4L2_CID_PRIVATE_BASE + 25)
#define V4L2_CID_EFFECT_ORIGINAL    (V4L2_CID_PRIVATE_BASE + 30)
#define V4L2_CID_EFFECT_ARBITRARY   (V4L2_CID_PRIVATE_BASE + 31)
#define V4L2_CID_EFFECT_NEGATIVE    (V4L2_CID_PRIVATE_BASE + 33)
#define V4L2_CID_EFFECT_ARTFREEZE   (V4L2_CID_PRIVATE_BASE + 34)
#define V4L2_CID_EFFECT_EMBOSSING   (V4L2_CID_PRIVATE_BASE + 35)
#define V4L2_CID_EFFECT_SILHOUETTE  (V4L2_CID_PRIVATE_BASE + 36)
#define V4L2_CID_ROTATE_ORIGINAL    (V4L2_CID_PRIVATE_BASE + 40)
#define V4L2_CID_ROTATE_90      (V4L2_CID_PRIVATE_BASE + 41)
#define V4L2_CID_ROTATE_180     (V4L2_CID_PRIVATE_BASE + 42)
#define V4L2_CID_ROTATE_270     (V4L2_CID_PRIVATE_BASE + 43)
#define V4L2_CID_ROTATE_90_HFLIP    (V4L2_CID_PRIVATE_BASE + 44)
#define V4L2_CID_ROTATE_90_VFLIP    (V4L2_CID_PRIVATE_BASE + 45)
#define V4L2_CID_ZOOM_IN        (V4L2_CID_PRIVATE_BASE + 51)
#define V4L2_CID_ZOOM_OUT       (V4L2_CID_PRIVATE_BASE + 52)
#define V4L2_CID_POWER_SAVE       (V4L2_CID_PRIVATE_BASE + 53)


/*
 * E X T E R N S
 *
*/
extern struct nx_vip_config nx_vip;
extern const struct v4l2_ioctl_ops nx_vip_v4l2_ops;
extern struct video_device nx_vip_video_device[];

//extern struct nx_platform_vip *to_vip_plat(struct device *dev);
//extern int nx_vip_i2c_read(struct nx_vip_control *ctrl,int subaddr, int *val);
//extern int nx_vip_i2c_write(struct nx_vip_control *ctrl,int subaddr, int val);
//extern void nx_vip_i2c_command(struct nx_vip_control *ctrl, u32 cmd, int arg);
//extern void nx_vip_register_camera(struct nx_vip_camera *cam);
//extern int  nx_vip_set_active_camera(struct nx_vip_control *ctrl, int id);
//extern void nx_vip_init_camera(struct nx_vip_control *ctrl);
extern int nx_vip_set_input_frame(struct nx_vip_control *ctrl, struct v4l2_pix_format *fmt);
extern int nx_vip_set_output_frame(struct nx_vip_control *ctrl, struct v4l2_pix_format *fmt);
extern void nx_vip_start_vip(struct nx_vip_control *ctrl);
extern void nx_vip_stop_vip(struct nx_vip_control *ctrl);
extern void nx_vip_restart_vip(struct nx_vip_control *ctrl);
extern int nx_vip_change_resolution(struct nx_vip_control *ctrl, enum nx_vip_cam_res_t res);
extern int nx_vip_alloc_output_memory(struct nx_vip_control *ctrl);
extern void nx_vip_free_output_memory(struct nx_vip_out_frame *info);
extern void nx_vip_free_frame_memory(struct nx_video_frame *frame);
extern void nx_vip_set_output_address(struct nx_vip_control *ctrl);
extern int nx_vip_alloc_frame_memory(struct nx_vip_control *ctrl);

extern int _set_plane_size(struct nx_video_frame *frame, unsigned int sizes[]);
//extern void nx_vip_register_subdev(struct v4l2_subdev *sd);
extern int nx_capture_set_format(struct nx_vip_control *ctrl, struct v4l2_format *f);
#endif /* __NX_VIP_CAMIF_H__ */

/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>
#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "ov50c40mipiraw_Sensor.h"

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define MULTI_WRITE 1
#else
#define MULTI_WRITE 0
#endif
#if MULTI_WRITE
#define I2C_BUFFER_LEN 255
#else
#define I2C_BUFFER_LEN 3
#endif
static kal_uint8 deviceInfo_register_value = 0x00;
#define PFX "ov50c40mipiraw_Sensor"
#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __func__, ##args)
extern void register_imgsensor_deviceinfo(char *name, char *version, u8 module_id);
static DEFINE_SPINLOCK(imgsensor_drv_lock);
#define DEVICE_VERSION  "ov50c40"
static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = OV50C40_SENSOR_ID_BARLEY,
	.module_id = 0x01,
	.checksum_value = 0x3acb7e3a,	/* checksum value for Camera Auto Test */

        .pre = {
            .pclk = 100000000,
            .linelength = 984,
            .framelength = 3387,
            .startx = 0,
            .starty = 0,
            .grabwindow_width = 4080,
            .grabwindow_height = 3072,
            .mipi_data_lp2hs_settle_dc = 85,
            .max_framerate = 300,
            .mipi_pixel_rate = 600000000,
        },
        .cap = {
            .pclk = 100000000,
            .linelength = 984,
            .framelength = 3387,
            .startx = 0,
            .starty = 0,
            .grabwindow_width = 4080,
            .grabwindow_height = 3072,
            .mipi_data_lp2hs_settle_dc = 85,
            .max_framerate = 300,
            .mipi_pixel_rate = 600000000,
        },
        .normal_video = {
            .pclk = 100000000,
            .linelength = 1000,
            .framelength = 3333,
            .startx = 0,
            .starty = 0,
            .grabwindow_width = 3840,
            .grabwindow_height = 2160,
            .mipi_data_lp2hs_settle_dc = 85,//unit(ns), 16/23/65/85 recommanded
            .max_framerate = 300,
            .mipi_pixel_rate = 600000000,
        },
	.hs_video = {
            .pclk = 100000000,
            .linelength = 432,
            .framelength = 1929,
            .startx = 0,
            .starty = 0,
            .grabwindow_width = 1920,
            .grabwindow_height = 1080,
            .mipi_data_lp2hs_settle_dc = 85,//unit(ns), 16/23/65/85 recommanded
            .max_framerate = 1200,
            .mipi_pixel_rate = 600000000,
        },
	.slim_video = {
            .pclk = 100000000,
            .linelength = 480,
            .framelength = 3472,
            .startx = 0,
            .starty = 0,
            .grabwindow_width = 2320,
            .grabwindow_height = 1736,
            .mipi_data_lp2hs_settle_dc = 85,//unit(ns), 16/23/65/85 recommanded
            .max_framerate = 600,
            .mipi_pixel_rate = 600000000,
	 },
	 .custom1 = {
            .pclk = 100000000,
            .linelength = 1050,
            .framelength = 3968,
            .startx = 0,
            .starty = 0,
            .grabwindow_width = 4064,
            .grabwindow_height = 3048,
            .mipi_data_lp2hs_settle_dc = 85,//unit(ns), 16/23/65/85 recommanded
            .max_framerate = 240,
            .mipi_pixel_rate = 600000000,
	 },

	.margin = 0x16,
	.min_shutter = 0x08,	//min shutter
	.max_frame_length = 0xffffe9,
	.min_gain = 64, /*1x gain*/
	.max_gain = 1024, /*16x gain*/
	.min_gain_iso = 100,
	.exp_step = 2,
	.gain_step = 1,
	.gain_type = 1,
	.ae_shut_delay_frame = 0,	//check
	.ae_sensor_gain_delay_frame = 0,	//check
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0,
	.ihdr_le_firstline = 0,
	.sensor_mode_num = 6,	//support sensor mode num
	.cap_delay_frame = 3,	//enter capture delay frame num
	.pre_delay_frame = 2,	//enter preview delay frame num
	.video_delay_frame = 2,	//enter video delay frame num
	.hs_video_delay_frame = 2,	//enter high speed video  delay frame num
	.slim_video_delay_frame = 2,	//enter slim video delay frame num
	.custom1_delay_frame = 2,
	.frame_time_delay_frame = 2, /* The delay frame of setting frame length  */
	.isp_driving_current = ISP_DRIVING_4MA,	//8MA
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_settle_delay_mode = 0,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gb,
	.mclk = 24,		//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
	.mipi_lane_num = SENSOR_MIPI_4_LANE,	//mipi lane num
	.i2c_addr_table = {0x6C, 0x21, 0xff},
	.i2c_speed = 1000,
	.current_ae_effective_frame = 2,
};

static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,
	.sensor_mode = IMGSENSOR_MODE_INIT,	/* IMGSENSOR_MODE enum value,
						 * record current sensor mode,such as:
						 * INIT, Preview, Capture, Video,High Speed Video, Slim Video
						 */
	.shutter = 0x3D0,	/* current shutter */
	.gain = 0x100,		/* current gain */
	.dummy_pixel = 0,	/* current dummypixel */
	.dummy_line = 0,	/* current dummyline */
	.current_fps = 300,	/* full size current fps : 24fps for PIP, 30fps for Normal or ZSD */
	.autoflicker_en = KAL_FALSE,	/* auto flicker enable:
					 * KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
					 */
	.test_pattern = KAL_FALSE,	/* test pattern mode or not.
					 * KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
					 */
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,	/* current scenario id */
	.ihdr_en = 0,		/* sensor need support LE, SE with HDR feature */
	.i2c_write_id = 0x6c,	/* record current sensor's i2c write id */
};

/* Sensor output window information*/
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[6] = {
    {8224, 6168,   0,  4, 8224, 6160, 4112, 3080,  0, 0, 4112, 3080, 16,  4,   4080, 3072},	// Preview
    {8224, 6168,   0,  4, 8224, 6160, 4112, 3080,  0, 0, 4112, 3080, 16,  4,   4080, 3072},	//capture
    {8224, 6168,   0,  4, 8224, 6160, 4112, 3080,  0, 0, 4112, 3080, 136, 460, 3840, 2160},	//normal-video
    {8224, 6168,   0,  4, 8224, 6160, 4112, 3080,  0, 0, 2056, 1540, 68,  230, 1920, 1080},	//hs-video 1920 1080
    {8224, 6168,   0,  4, 8224, 6160, 4112, 3080,  0, 0, 4112, 3080, 896, 672, 2320, 1736},	//slim-video
    {8224, 6168,   0,  4, 8224, 6160, 4112, 3080,  0, 0, 4112, 3080, 24,  16,  4064, 3048}	//custom1
};

static struct SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[6] = {
    /* preview mode setting */
    {
        0x02, 0x0a, 0x0000, 0x0008, 0x40, 0x00,
        0x00, 0x2b, 0x0ff0, 0x0C00, 0x00, 0x00, 0x0000, 0x0000,
        0x01, 0x2b, 0x03F8, 0x02F8, 0x03, 0x00, 0x0000, 0x0000
    },
    /* capture mode setting */
    {
        0x02, 0x0a, 0x0000, 0x0008, 0x40, 0x00,
        0x00, 0x2b, 0x0ff0, 0x0C00, 0x00, 0x00, 0x0000, 0x0000,
        0x01, 0x2b, 0x03F8, 0x02F8, 0x03, 0x00, 0x0000, 0x0000
    },
    /* normal_video mode setting */
    {
        0x02, 0x0a, 0x0000, 0x0008, 0x40, 0x00,
        0x00, 0x2b, 0x0f00, 0x0870, 0x00, 0x00, 0x0000, 0x0000,
        0x01, 0x2B, 0x03c0, 0x021c, 0x03, 0x00, 0x0000, 0x0000
    },
    /* high_speed_video mode setting */
    {
        0x02, 0x0a, 0x0000, 0x0008, 0x40, 0x00,
        0x00, 0x2b, 0x0500, 0x02D0, 0x00, 0x00, 0x0000, 0x0000,
        0x00, 0x00, 0x0000, 0x0000, 0x03, 0x00, 0x0000, 0x0000
    },
    /* slim_video mode setting */
    {
        0x02, 0x0a, 0x0000, 0x0008, 0x40, 0x00,
        0x00, 0x2b, 0x0800, 0x0600, 0x00, 0x00, 0x0000, 0x0000,
        0x00, 0x00, 0x0000, 0x0000, 0x03, 0x00, 0x0000, 0x0000
    },
    /* custom1 mode setting */
    {
        0x02, 0x0a, 0x0000, 0x0008, 0x40, 0x00,
        0x00, 0x2b, 0x0fe0, 0x0be8, 0x00, 0x00, 0x0000, 0x0000,
        0x01, 0x2b, 0x03F8, 0x02F4, 0x03, 0x00, 0x0000, 0x0000
    },
};
/*PDAF START*/
static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
	.i4OffsetX = 8,
	.i4OffsetY = 16,
	.i4PitchX = 16,
	.i4PitchY = 16,
	.i4PairNum = 8,
	.i4SubBlkW = 8,
	.i4SubBlkH = 4,
	.i4BlockNumX = 254,
	.i4BlockNumY = 190,
	.i4PosL = {
		{15, 18},
		{23, 18},
		{11, 22},
		{19, 22},
		{15, 26},
		{23, 26},
		{11, 30},
		{19, 30}
	},
	.i4PosR = {
		{14, 18},
		{22, 18},
		{10, 22},
		{18, 22},
		{14, 26},
		{22, 26},
		{10, 30},
		{18, 30}
	},
	.i4Crop = {
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}
	},
	.iMirrorFlip = 0,	/*0 IMAGE_NORMAL, 1 IMAGE_H_MIRROR, 2 IMAGE_V_MIRROR, 3 IMAGE_HV_MIRROR */
};
static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_16_9 = {
	.i4OffsetX = 16,
	.i4OffsetY = 16,
	.i4PitchX = 16,
	.i4PitchY = 16,
	.i4PairNum = 8,
	.i4SubBlkW = 8,
	.i4SubBlkH = 4,
	.i4BlockNumX = 240,
	.i4BlockNumY = 135,
	.i4PosL = {
		{23, 18},
		{31, 18},
		{19, 22},
		{27, 22},
		{23, 26},
		{31, 26},
		{19, 30},
		{27, 30}
	},
	.i4PosR = {
		{22, 18},
		{30, 18},
		{18, 22},
		{26, 22},
		{22, 26},
		{30, 26},
		{18, 30},
		{26, 30}
	},
	.i4Crop = {
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}
	},
	.iMirrorFlip = 0,	/*0 IMAGE_NORMAL, 1 IMAGE_H_MIRROR, 2 IMAGE_V_MIRROR, 3 IMAGE_HV_MIRROR */
};
static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_custom1 = {
	.i4OffsetX = 0,
	.i4OffsetY = 12,
	.i4PitchX = 16,
	.i4PitchY = 16,
	.i4PairNum = 8,
	.i4SubBlkW = 8,
	.i4SubBlkH = 4,
	.i4BlockNumX = 254,
	.i4BlockNumY = 189,
	.i4PosL = {
		{7, 14},
		{15, 14},
		{3, 18},
		{11, 18},
		{7, 22},
		{15, 22},
		{3, 26},
		{11, 26}
	},
	.i4PosR = {
		{6, 14},
		{14, 14},
		{2, 18},
		{10, 18},
		{6, 22},
		{14, 22},
		{2, 26},
		{10, 26}
	},
	.i4Crop = {
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}
	},
	.iMirrorFlip = 0,	/*0 IMAGE_NORMAL, 1 IMAGE_H_MIRROR, 2 IMAGE_V_MIRROR, 3 IMAGE_HV_MIRROR */
};

typedef struct SET_PD_BLOCK_INFO_T SET_PD_BLOCK_INFO_T;
/*PDAF END*/

static kal_uint16 ov50c40_table_write_cmos_sensor(kal_uint16 *para, kal_uint32 len)
{
	char puSendCmd[I2C_BUFFER_LEN];
	kal_uint32 tosend, IDX;
	kal_uint16 addr = 0, addr_last = 0, data;

	tosend = 0;
	IDX = 0;
	while (len > IDX) {
		addr = para[IDX];

		{
			puSendCmd[tosend++] = (char)(addr >> 8);
			puSendCmd[tosend++] = (char)(addr & 0xFF);
			data = para[IDX + 1];
			puSendCmd[tosend++] = (char)(data & 0xFF);
			IDX += 2;
			addr_last = addr;

		}
#if MULTI_WRITE
		/* Write when remain buffer size is less than 3 bytes or reach end of data */
		if ((I2C_BUFFER_LEN - tosend) < 3 || IDX == len || addr != addr_last) {
			iBurstWriteReg_multi(puSendCmd, tosend, imgsensor.i2c_write_id, 3,
				 imgsensor_info.i2c_speed);
			tosend = 0;
		}
#else
		iWriteRegI2C(puSendCmd, 3, imgsensor.i2c_write_id);
		tosend = 0;

#endif
	}
	return 0;
}

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pusendcmd, 2, (u8 *) &get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pusendcmd[4] = { (char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF) };

	iWriteRegI2C(pusendcmd, 3, imgsensor.i2c_write_id);
}

static void set_dummy(void)
{
	/* check */
	/* LOG_INF("dummyline = %d, dummypixels = %d\n", imgsensor.dummy_line, imgsensor.dummy_pixel); */
	kal_uint32 reg_frame_length;
	write_cmos_sensor(0x380c, imgsensor.line_length >> 8);
	write_cmos_sensor(0x380d, imgsensor.line_length & 0xFF);
	write_cmos_sensor(0x3840, imgsensor.frame_length >> 16);
	write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
	write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
	reg_frame_length = ((read_cmos_sensor(0x3840) << 16) |
		(read_cmos_sensor(0x380e) << 8) | read_cmos_sensor(0x380f));
	LOG_INF("suney reg framelength =%d\n", reg_frame_length);
}


static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;
	/* unsigned long flags; */

	LOG_INF("framerate = %d, min framelength should enable? %d\n", framerate,
		min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length)
		? frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	/* dummy_line = frame_length - imgsensor.min_frame_length; */
	/* if (dummy_line < 0) */
	/* imgsensor.dummy_line = 0; */
	/* else */
	/* imgsensor.dummy_line = dummy_line; */
	/* imgsensor.frame_length = frame_length + imgsensor.dummy_line; */
	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}				/*      set_max_framerate  */

/*************************************************************************
 * FUNCTION
 *	set_shutter
 *
 * DESCRIPTION
 *	This function set e-shutter of sensor to change exposure time.
 *
 * PARAMETERS
 *	iShutter : exposured lines
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void write_shutter(kal_uint32 shutter)
{
//	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	kal_uint32 reg_shutter, reg_frame_length;
	reg_shutter = ((read_cmos_sensor(0x3500) << 16) |
		(read_cmos_sensor(0x3501) << 8) | read_cmos_sensor(0x3502));
	reg_frame_length = ((read_cmos_sensor(0x3840) << 16) |
		(read_cmos_sensor(0x380e) << 8) | read_cmos_sensor(0x380f));

//	imgsensor.current_ae_effective_frame = 2;
	LOG_INF("reg shutter =%d, framelength =%d\n",reg_shutter, reg_frame_length);
	/* write_shutter(shutter); */
	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */

	/* OV Recommend Solution */
	/* if shutter bigger than frame_length, should extend frame length first */
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	}
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter >> 1) << 1;
	imgsensor.frame_length = (imgsensor.frame_length >> 1) << 1;
	if(((reg_shutter -1 )<= imgsensor.frame_length )&& (imgsensor.frame_length<=(reg_shutter + 7)))
		imgsensor.frame_length = reg_shutter + 8;

	if (imgsensor.autoflicker_en == KAL_TRUE) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 /
		    imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			realtime_fps = 296;
			set_max_framerate(realtime_fps, 0);
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			realtime_fps = 146;
			set_max_framerate(realtime_fps, 0);
		} else {
			imgsensor.frame_length =(imgsensor.frame_length >> 1) << 1;
			write_cmos_sensor(0x3208, 0x00);
			write_cmos_sensor(0x3840, imgsensor.frame_length >> 16);
			write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
			write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
			write_cmos_sensor(0x3208, 0x10);
			write_cmos_sensor(0x3208, 0xa0);
			reg_frame_length = ((read_cmos_sensor(0x3840) << 16) |
				(read_cmos_sensor(0x380e) << 8) | read_cmos_sensor(0x380f));
			LOG_INF("aaa reg framelength =%d, imgsensor.frame_length is %d\n", reg_frame_length, imgsensor.frame_length);
		}
	} else {
	        imgsensor.frame_length = (imgsensor.frame_length >> 1) << 1;
		write_cmos_sensor(0x3208, 0x00);
		write_cmos_sensor(0x3840, imgsensor.frame_length >> 16);
		write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
		write_cmos_sensor(0x3208, 0x10);
		write_cmos_sensor(0x3208, 0xa0);
		reg_frame_length = ((read_cmos_sensor(0x3840) << 16) |
			(read_cmos_sensor(0x380e) << 8) | read_cmos_sensor(0x380f));
		LOG_INF("bbb reg framelength =%d, imgsensor.frame_length is %d\n", reg_frame_length, imgsensor.frame_length);
	}

	/*Warning : shutter must be even. Odd might happen Unexpected Results */
	write_cmos_sensor(0x3208, 0x01);
	write_cmos_sensor(0x3500, (shutter >> 16) & 0xFF);
	write_cmos_sensor(0x3501, (shutter >> 8) & 0xFF);
	write_cmos_sensor(0x3502, (shutter)  & 0xFF);
	write_cmos_sensor(0x3208, 0x11);
	write_cmos_sensor(0x3208, 0xa1);
	LOG_INF("shutter =%d, framelength =%d, realtime_fps =%d\n",
		shutter, imgsensor.frame_length, realtime_fps);

}

static void set_shutter(kal_uint32 shutter)
{
	unsigned long flags;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	write_shutter(shutter);
}/*
static kal_uint32 streaming_control(kal_bool enable)
{
	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
	if (enable){
		mdelay(5);
		write_cmos_sensor(0x0100, 0X01);
	}else{
		mdelay(5);
		write_cmos_sensor(0x0100, 0x00);
	}
	mdelay(10);
	return ERROR_NONE;
}
*/
static void set_shutter_frame_length(kal_uint16 shutter, kal_uint16 frame_length,kal_bool auto_extend_en)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;
	kal_uint32 reg_shutter, reg_frame_length;
	reg_shutter = ((read_cmos_sensor(0x3500) << 16) |
		(read_cmos_sensor(0x3501) << 8) | read_cmos_sensor(0x3502));
	reg_frame_length = ((read_cmos_sensor(0x3840) << 16) |
		(read_cmos_sensor(0x380e) << 8) | read_cmos_sensor(0x380f));
	LOG_INF("reg shutter =%d, framelength =%d\n",reg_shutter, reg_frame_length);
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	spin_lock(&imgsensor_drv_lock);
	/*Change frame time*/
	if (frame_length > 1)
	dummy_line = frame_length - imgsensor.frame_length;
	imgsensor.frame_length = imgsensor.frame_length + dummy_line;
	//
	if (shutter > imgsensor.frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ?
		imgsensor_info.min_shutter : shutter;
	shutter =
		(shutter > (imgsensor_info.max_frame_length -
		imgsensor_info.margin)) ? (imgsensor_info.max_frame_length -
		imgsensor_info.margin) : shutter;

	if(reg_shutter -1 <= imgsensor.frame_length && imgsensor.frame_length<=reg_shutter + 7)
		imgsensor.frame_length = reg_shutter + 8;

	if (imgsensor.autoflicker_en == KAL_TRUE) {
		realtime_fps = imgsensor.pclk /
			imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			realtime_fps = 296;
			set_max_framerate(realtime_fps, 0);
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			realtime_fps = 146;
			set_max_framerate(realtime_fps, 0);
		} else {
		imgsensor.frame_length = (imgsensor.frame_length  >> 1) << 1;
		write_cmos_sensor(0x3208, 0x00);
		write_cmos_sensor(0x3840, imgsensor.frame_length >> 16);
		write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
		write_cmos_sensor(0x3208, 0x10);
		write_cmos_sensor(0x3208, 0xa0);
		reg_frame_length = ((read_cmos_sensor(0x3840) << 16) |
			(read_cmos_sensor(0x380e) << 8) | read_cmos_sensor(0x380f));
		LOG_INF("AAA reg framelength =%d, imgsensor.frame_length is %d\n", reg_frame_length, imgsensor.frame_length);
		}
	} else {
		imgsensor.frame_length = (imgsensor.frame_length  >> 1) << 1;
		write_cmos_sensor(0x3208, 0x00);
		write_cmos_sensor(0x3840, imgsensor.frame_length >> 16);
		write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
		write_cmos_sensor(0x3208, 0x10);
		write_cmos_sensor(0x3208, 0xa0);
		reg_frame_length = ((read_cmos_sensor(0x3840) << 16) |
			(read_cmos_sensor(0x380e) << 8) | read_cmos_sensor(0x380f));
		LOG_INF("BBB reg framelength =%d, imgsensor.frame_length is %d\n", reg_frame_length, imgsensor.frame_length);
	}
	/*Warning : shutter must be even. Odd might happen Unexpected Results */
	write_cmos_sensor(0x3208, 0x01);
	write_cmos_sensor(0x3500, (shutter >> 16) & 0xFF);
	write_cmos_sensor(0x3501, (shutter >> 8) & 0xFF);
	write_cmos_sensor(0x3502, (shutter)  & 0xFF);
	write_cmos_sensor(0x3208, 0x11);
	write_cmos_sensor(0x3208, 0xa1);
	LOG_INF("shutter =%d, framelength =%d, realtime_fps =%d\n",
		shutter, imgsensor.frame_length, realtime_fps);
}


static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 iReg = 0x0000;

	/* platform 1xgain = 64, sensor driver 1*gain = 0x80 */
	iReg = gain * 256 / BASEGAIN;

	// sensor 1xGain
	if (iReg < 0x100)
		iReg = 0X100;

	// sensor 15.5xGain
	if (iReg > 0xF80)
		iReg = 0XF80;

	return iReg;
}

/*************************************************************************
 * FUNCTION
 *	set_gain
 *
 * DESCRIPTION
 *	This function is to set global gain to sensor.
 *
 * PARAMETERS
 *	iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *	the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;
	unsigned long flags;

	reg_gain = gain2reg(gain);
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.gain = reg_gain;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);
	write_cmos_sensor(0x03508, (reg_gain >> 8));
	write_cmos_sensor(0x03509, (reg_gain & 0xff));
	return gain;
}				/*      set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n", le, se, gain);
}


#if 1
static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d\n", image_mirror);

	/********************************************************
	 *
	 *   0x3820[2] ISP Vertical flip
	 *   0x3820[1] Sensor Vertical flip
	 *
	 *   0x3821[2] ISP Horizontal mirror
	 *   0x3821[1] Sensor Horizontal mirror
	 *
	 *   ISP and Sensor flip or mirror register bit should be the same!!
	 *
	 ********************************************************/
	switch (image_mirror) {
	case IMAGE_NORMAL:
		break;
	case IMAGE_H_MIRROR:
		break;
	case IMAGE_V_MIRROR:
		break;
	case IMAGE_HV_MIRROR:
		break;
	default:
		LOG_INF("Error image_mirror setting\n");
	}
}
#endif
/*************************************************************************
 * FUNCTION
 *	night_mode
 *
 * DESCRIPTION
 *	This function night mode of sensor.
 *
 * PARAMETERS
 *	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void night_mode(kal_bool enable)
{
	/*No Need to implement this function */
}				/*      night_mode      */

/*************************************************************************
* FUNCTION
*	sensor_init
*
* DESCRIPTION
*	Sensor init
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/

kal_uint16 addr_data_pair_init_barley_ov50c40[] = {
	0x0103,0x01,
	0x0301,0xc8,
	0x0304,0x01,
	0x0305,0x77,
	0x0306,0x04,
	0x0324,0x02,
	0x0325,0x58,
	0x0326,0xcb,
	0x0327,0x05,
	0x0328,0x07,
	0x032a,0x0a,
	0x0344,0x01,
	0x0345,0x4a,
	0x0350,0x00,
	0x0360,0x09,
	0x3002,0x00,
	0x3009,0x04,
	0x3012,0x41,
	0x3019,0xc2,
	0x301c,0x81,
	0x3025,0x03,
	0x3026,0x10,
	0x3027,0x00,
	0x3107,0x48,
	0x3400,0x0c,
	0x3409,0x03,
	0x340c,0x10,
	0x340d,0x00,
	0x3420,0x11,
	0x3421,0x08,
	0x3423,0x15,
	0x3424,0x40,
	0x3425,0x10,
	0x3426,0x40,
	0x3500,0x00,
	0x3501,0x0c,
	0x3502,0x16,
	0x3504,0x28,
	0x3507,0x00,
	0x3508,0x01,
	0x3509,0x00,
	0x350a,0x01,
	0x350b,0x00,
	0x350c,0x00,
	0x350e,0x00,
	0x3540,0x00,
	0x3541,0x00,
	0x3542,0x00,
	0x3548,0x01,
	0x3549,0x00,
	0x354a,0x01,
	0x354b,0x00,
	0x354c,0x00,
	0x3607,0x01,
	0x3608,0xda,
	0x360a,0x73,
	0x360c,0x03,
	0x360e,0x1d,
	0x3618,0x80,
	0x361b,0x80,
	0x3622,0x88,
	0x3623,0x33,
	0x3624,0x95,
	0x3627,0xcc,
	0x3628,0xaa,
	0x362b,0x08,
	0x362d,0x08,
	0x363b,0x80,
	0x363d,0x0c,
	0x3680,0xc0,
	0x3684,0x03,
	0x368d,0x00,
	0x368e,0x01,
	0x3690,0x10,
	0x3697,0x80,
	0x3699,0x1f,
	0x369b,0x10,
	0x369c,0x2b,
	0x369d,0x40,
	0x369e,0x00,
	0x36a1,0x00,
	0x3700,0x28,
	0x3701,0x05,
	0x3702,0x4b,
	0x3703,0x27,
	0x3704,0x07,
	0x3706,0x2f,
	0x3707,0x08,
	0x3708,0x32,
	0x3709,0x69,
	0x370b,0x5d,
	0x370c,0x0f,
	0x3711,0x00,
	0x3712,0x50,
	0x3714,0x67,
	0x3715,0x00,
	0x3717,0x03,
	0x371c,0x04,
	0x371d,0x1e,
	0x371e,0x13,
	0x371f,0x0a,
	0x3720,0x08,
	0x3721,0x15,
	0x3725,0x32,
	0x3727,0x22,
	0x3729,0x01,
	0x3731,0x05,
	0x3736,0x02,
	0x3737,0x05,
	0x3738,0x02,
	0x3739,0x05,
	0x373b,0x2a,
	0x373d,0x2e,
	0x3741,0x41,
	0x3754,0xee,
	0x375f,0x80,
	0x3760,0x08,
	0x3761,0x10,
	0x3762,0x08,
	0x3763,0x08,
	0x3764,0x08,
	0x3765,0x10,
	0x3766,0x18,
	0x3767,0x28,
	0x3768,0x00,
	0x3769,0x08,
	0x376c,0x00,
	0x376f,0x02,
	0x3770,0x3b,
	0x379e,0x3b,
	0x379f,0x3b,
	0x37b0,0x37,
	0x37b1,0x37,
	0x37b2,0x37,
	0x37b3,0x3b,
	0x37b4,0x3a,
	0x37b5,0x3a,
	0x37ce,0x02,
	0x37cf,0x05,
	0x37d6,0x00,
	0x37d9,0x00,
	0x37dc,0x43,
	0x37ed,0x02,
	0x37ee,0x05,
	0x37f0,0x02,
	0x37f1,0x05,
	0x37f2,0x02,
	0x37f3,0x05,
	0x37f4,0x00,
	0x3800,0x00,
	0x3801,0x00,
	0x3802,0x00,
	0x3803,0x08,
	0x3804,0x20,
	0x3805,0x1f,
	0x3806,0x18,
	0x3807,0x17,
	0x3808,0x0f,
	0x3809,0xf0,
	0x380a,0x0c,
	0x380b,0x00,
	0x380c,0x04,
	0x380d,0x1a,
	0x380e,0x0c,
	0x380f,0x66,
	0x3810,0x00,
	0x3811,0x10,
	0x3812,0x00,
	0x3813,0x04,
	0x3814,0x11,
	0x3815,0x11,
	0x381a,0x00,
	0x381b,0x00,
	0x381e,0x00,
	0x381f,0x00,
	0x3820,0x02,
	0x3821,0x06,
	0x3822,0x10,
	0x3824,0x00,
	0x3825,0x00,
	0x3826,0x00,
	0x3827,0x00,
	0x3828,0x07,
	0x382a,0x81,
	0x382c,0x00,
	0x382d,0x00,
	0x3835,0x00,
	0x3836,0x00,
	0x383c,0x00,
	0x383d,0x10,
	0x3845,0x04,
	0x3846,0x1a,
	0x3847,0x00,
	0x3848,0x00,
	0x3849,0x00,
	0x384b,0x8e,
	0x384f,0x00,
	0x3856,0x10,
	0x3857,0x10,
	0x3858,0x20,
	0x3859,0x20,
	0x3865,0x00,
	0x3869,0x00,
	0x3902,0x01,
	0x3903,0x08,
	0x3905,0x05,
	0x3906,0x00,
	0x3909,0x00,
	0x390c,0x00,
	0x390f,0x00,
	0x3912,0x00,
	0x3915,0x04,
	0x3918,0x04,
	0x391b,0x00,
	0x391d,0x02,
	0x391e,0x00,
	0x3924,0x02,
	0x3927,0x02,
	0x392a,0x02,
	0x392d,0x02,
	0x3930,0x02,
	0x3933,0x02,
	0x3936,0x02,
	0x3939,0x02,
	0x393c,0x02,
	0x393f,0x02,
	0x3942,0x02,
	0x3945,0x02,
	0x3980,0x37,
	0x3981,0x3a,
	0x3982,0x0c,
	0x3983,0x3b,
	0x3990,0x01,
	0x399b,0x02,
	0x399c,0x03,
	0x39aa,0x05,
	0x39b1,0x05,
	0x39b3,0x78,
	0x39b6,0x40,
	0x39b7,0x10,
	0x39ba,0x2f,
	0x39bc,0x2f,
	0x39be,0x2f,
	0x39c0,0x2f,
	0x39c2,0x5d,
	0x39c4,0x5d,
	0x39c6,0x5d,
	0x39c8,0x5d,
	0x39c9,0x01,
	0x39cf,0x00,
	0x39d2,0x00,
	0x3a01,0x1e,
	0x3a12,0x00,
	0x3a13,0x00,
	0x3a14,0x00,
	0x3a18,0x04,
	0x3a36,0x20,
	0x3d85,0x0b,
	0x3d86,0x00,
	0x3d87,0x12,
	0x3d8c,0x73,
	0x3d8d,0xd8,
	0x3daa,0x00,
	0x3dab,0x00,
	0x3dac,0x00,
	0x3dad,0x00,
	0x3dae,0x00,
	0x3daf,0x00,
	0x3f01,0x13,
	0x3f9e,0x03,
	0x3f9f,0x04,
	0x4009,0x01,
	0x4010,0x38,
	0x4011,0x01,
	0x4012,0x0d,
	0x4015,0x00,
	0x4016,0x0f,
	0x4017,0x00,
	0x4018,0x03,
	0x401a,0x40,
	0x401b,0x04,
	0x401e,0x00,
	0x401f,0xd0,
	0x4020,0x04,
	0x4021,0x00,
	0x4022,0x04,
	0x4023,0x00,
	0x4024,0x04,
	0x4025,0x00,
	0x4026,0x04,
	0x4027,0x00,
	0x4030,0x00,
	0x4031,0x00,
	0x4032,0x00,
	0x4033,0x04,
	0x4034,0x00,
	0x4035,0x04,
	0x4036,0x00,
	0x4037,0x04,
	0x4040,0x00,
	0x4041,0x00,
	0x4042,0x00,
	0x4043,0x00,
	0x4044,0x00,
	0x4045,0x00,
	0x4046,0x00,
	0x4047,0x00,
	0x4056,0x25,
	0x4100,0x00,
	0x4103,0x00,
	0x4104,0x00,
	0x4300,0x00,
	0x4301,0x00,
	0x4302,0x00,
	0x4303,0x00,
	0x4304,0x00,
	0x4305,0x00,
	0x4306,0x00,
	0x4307,0x00,
	0x4308,0x00,
	0x430b,0xff,
	0x430c,0xff,
	0x430d,0x00,
	0x430e,0x00,
	0x4500,0x04,
	0x4503,0x0f,
	0x4507,0x00,
	0x4508,0x00,
	0x4510,0x07,
	0x4512,0x00,
	0x4513,0x00,
	0x4514,0x00,
	0x4515,0x00,
	0x4516,0x55,
	0x4517,0x55,
	0x4518,0x55,
	0x4519,0x55,
	0x451a,0x11,
	0x451b,0xbb,
	0x451c,0x11,
	0x451d,0xbb,
	0x451e,0x11,
	0x451f,0xbb,
	0x4520,0x11,
	0x4521,0xbb,
	0x460b,0x01,
	0x4640,0x00,
	0x4641,0x7f,
	0x4642,0x42,
	0x4643,0x0c,
	0x4648,0x0a,
	0x4649,0x05,
	0x4700,0x0a,
	0x4701,0x0c,
	0x4702,0x0a,
	0x4703,0x0a,
	0x4704,0x12,
	0x4705,0x0a,
	0x4706,0x0c,
	0x4707,0x0a,
	0x4708,0x0a,
	0x4709,0x12,
	0x470a,0x0a,
	0x470b,0x0c,
	0x470c,0x0a,
	0x470d,0x0a,
	0x470e,0x12,
	0x4720,0x00,
	0x4721,0x00,
	0x4723,0x00,
	0x4724,0x00,
	0x4725,0x01,
	0x4726,0x01,
	0x4748,0x00,
	0x4731,0x0d,
	0x4732,0x0d,
	0x4733,0x0f,
	0x4734,0x0f,
	0x4735,0x0c,
	0x4736,0x04,
	0x4737,0x00,
	0x4738,0x0f,
	0x4739,0x0f,
	0x473a,0x0c,
	0x473b,0x04,
	0x473c,0x00,
	0x473d,0x0d,
	0x473e,0x0f,
	0x473f,0x0f,
	0x4740,0x0c,
	0x4741,0x04,
	0x4742,0x00,
	0x4743,0x0f,
	0x4744,0x55,
	0x4745,0xff,
	0x4747,0x00,
	0x474e,0x80,
	0x4750,0x01,
	0x4753,0x80,
	0x4755,0x01,
	0x4757,0x0b,
	0x4759,0x0b,
	0x475b,0x06,
	0x475d,0x03,
	0x475f,0x02,
	0x4761,0x07,
	0x4763,0x07,
	0x4765,0x04,
	0x4767,0x02,
	0x4769,0x01,
	0x4800,0x64,
	0x4802,0x00,
	0x480b,0x10,
	0x480c,0x80,
	0x480e,0x04,
	0x480f,0x32,
	0x4815,0x19,
	0x481b,0x3c,
	0x481f,0x32,
	0x4837,0x0a,
	0x484b,0x27,
	0x4850,0x43,
	0x4851,0xaa,
	0x4853,0x4a,
	0x4854,0x06,
	0x4860,0x00,
	0x4861,0xec,
	0x4862,0x04,
	0x4883,0x00,
	0x4888,0x10,
	0x4d00,0x05,
	0x4d01,0x0e,
	0x4d02,0xb7,
	0x4d03,0x39,
	0x4d04,0xcd,
	0x4d05,0x4e,
	0x5000,0x8b,
	0x5001,0x23,
	0x5002,0x9d,
	0x5003,0xca,
	0x5005,0x00,
	0x5006,0x00,
	0x5016,0x00,
	0x5017,0x00,
	0x5035,0x18,
	0x5037,0x14,
	0x5038,0x0f,
	0x5039,0xe0,
	0x503a,0x0b,
	0x503b,0xe0,
	0x504c,0x12,
	0x504d,0x12,
	0x504e,0x12,
	0x504f,0x12,
	0x5050,0x12,
	0x5051,0x14,
	0x5055,0x12,
	0x5081,0x00,
	0x5180,0x70,
	0x5181,0x10,
	0x5182,0x00,
	0x5183,0x7f,
	0x5184,0x40,
	0x5185,0x2b,
	0x5187,0x88,
	0x518c,0x01,
	0x518d,0x01,
	0x518e,0x01,
	0x518f,0x01,
	0x5190,0x00,
	0x5191,0x00,
	0x5192,0x10,
	0x5193,0x0f,
	0x5194,0x00,
	0x5195,0x04,
	0x5880,0xc1,
	0x588a,0x04,
	0x5c00,0x63,
	0x5c01,0x04,
	0x5c02,0x00,
	0x5c03,0x3f,
	0x5c04,0x00,
	0x5c05,0x54,
	0x5c0e,0x01,
	0x5c0f,0x00,
	0x5c10,0x01,
	0x5c11,0x00,
	0x5c12,0x01,
	0x5c13,0x00,
	0x5c14,0x01,
	0x5c15,0x00,
	0x5c16,0x00,
	0x5c17,0x50,
	0x5c18,0x00,
	0x5c19,0x50,
	0x5c1a,0x00,
	0x5c1b,0x50,
	0x5c1c,0x00,
	0x5c1d,0x50,
	0x5c1e,0x00,
	0x5c1f,0x50,
	0x5c20,0x00,
	0x5c21,0x50,
	0x5c22,0x00,
	0x5c23,0x50,
	0x5c24,0x00,
	0x5c25,0x50,
	0x5c26,0x00,
	0x5c27,0x50,
	0x5c28,0x00,
	0x5c29,0x50,
	0x5c2a,0x00,
	0x5c2b,0x50,
	0x5c2c,0x00,
	0x5c2d,0x50,
	0x5c44,0x02,
	0x5c45,0x0a,
	0x5c46,0x00,
	0x5c47,0x07,
	0x5c48,0x0f,
	0x5c49,0x01,
	0x5c4a,0x00,
	0x5c4b,0x09,
	0x5c4c,0x06,
	0x5c4d,0x0e,
	0x5c4e,0x00,
	0x5c4f,0x00,
	0x5c50,0x00,
	0x5c51,0x00,
	0x5c52,0x00,
	0x5c53,0x00,
	0x5c54,0x10,
	0x5c55,0x08,
	0x5c56,0x04,
	0x5c57,0x04,
	0x5c58,0x0c,
	0x5c59,0x0c,
	0x5c5a,0x02,
	0x5c5b,0x02,
	0x5c5c,0x06,
	0x5c5d,0x06,
	0x5c5e,0x01,
	0x5c5f,0x00,
	0x5c60,0x00,
	0x5c61,0x30,
	0x5c62,0x00,
	0x5c63,0x30,
	0x5c64,0x1f,
	0x5c65,0xc0,
	0x5c66,0x17,
	0x5c67,0xc0,
	0x5c68,0x02,
	0x5c69,0x02,
	0x5c6a,0x02,
	0x5c6b,0x02,
	0x5c90,0x01,
	0x5d00,0xc2,
	0x5d01,0x68,
	0x5d02,0xff,
	0x5d03,0x1c,
	0x5d05,0x02,
	0x5d06,0x04,
	0x5d07,0x11,
	0x5d08,0x08,
	0x5d09,0x08,
	0x5d0a,0x02,
	0x5d0b,0x02,
	0x5d0c,0x06,
	0x5d0d,0x06,
	0x5d0e,0x02,
	0x5d0f,0x02,
	0x5d10,0x06,
	0x5d11,0x06,
	0x5d12,0x00,
	0x5d13,0x00,
	0x5d14,0xff,
	0x5d15,0x10,
	0x5d16,0x10,
	0x5d17,0x10,
	0x5d18,0x10,
	0x5d19,0xff,
	0x5d1a,0x10,
	0x5d1b,0x10,
	0x5d1c,0x10,
	0x5d1d,0x10,
	0x5d1e,0x01,
	0x5d1f,0x02,
	0x5d20,0x04,
	0x5d21,0xaa,
	0x5d34,0x00,
	0x5d35,0x30,
	0x5d36,0x00,
	0x5d37,0x30,
	0x5d38,0x1f,
	0x5d39,0xc0,
	0x5d3a,0x17,
	0x5d3b,0xc0,
	0x5d3d,0x08,
	0x5d40,0x00,
	0x5d41,0x00,
	0x5d45,0x05,
	0x5e42,0x0c,
	0x5e43,0x0c,
	0x5e44,0x0c,
	0x5e45,0x0c,
	0x5e46,0x0c,
	0x5e47,0x0c,
	0x5e48,0x0c,
	0x5e49,0x0c,
	0x5900,0x40,
};
static void sensor_init(void)
{
	write_cmos_sensor(0x0103, 0x01);	//SW Reset, need delay
	mdelay(10);
#if MULTI_WRITE
	LOG_INF("sensor_init MULTI_WRITE\n");
	ov50c40_table_write_cmos_sensor(addr_data_pair_init_barley_ov50c40,
			   sizeof(addr_data_pair_init_barley_ov50c40) / sizeof(kal_uint16));
#endif
}
/*************************************************************************
* FUNCTION
*	preview_setting
*
* DESCRIPTION
*	Sensor preview
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
kal_uint16 addr_data_pair_preview_barley_ov50c40[] = {
	0x3607,0x01,
	0x3608,0xda,
	0x360c,0x03,
	0x360e,0x1d,
	0x3684,0x03,
	0x369b,0x10,
	0x369c,0x2b,
	0x369d,0x40,
	0x3702,0x4b,
	0x3703,0x27,
	0x3706,0x2f,
	0x3709,0x69,
	0x370b,0x5d,
	0x3714,0x67,
	0x3717,0x03,
	0x373b,0x2a,
	0x373d,0x2e,
	0x3741,0x41,
	0x375f,0x80,
	0x37b0,0x37,
	0x37b3,0x3b,
	0x3800,0x00,
	0x3801,0x00,
	0x3802,0x00,
	0x3803,0x08,
	0x3804,0x20,
	0x3805,0x1f,
	0x3806,0x18,
	0x3807,0x17,
	0x3808,0x0f,
	0x3809,0xf0,
	0x380a,0x0c,
	0x380b,0x00,
	0x3811,0x10,
	0x3813,0x04,
	0x3814,0x11,
	0x3815,0x11,
	0x3820,0x02,
	0x3821,0x06,
	0x3822,0x10,
	0x383d,0x10,
	0x3845,0x04,
	0x3846,0x1a,
	0x3856,0x10,
	0x3857,0x10,
	0x3858,0x20,
	0x3859,0x20,
	0x3902,0x01,
	0x3903,0x08,
	0x3905,0x05,
	0x399b,0x02,
	0x399c,0x03,
	0x39aa,0x05,
	0x39ba,0x2f,
	0x39bc,0x2f,
	0x39be,0x2f,
	0x39c0,0x2f,
	0x39c2,0x5d,
	0x39c4,0x5d,
	0x39c6,0x5d,
	0x39c8,0x5d,
	0x4010,0x38,
	0x4016,0x0f,
	0x4018,0x03,
	0x401e,0x00,
	0x401f,0xd0,
	0x4031,0x00,
	0x4033,0x04,
	0x4034,0x00,
	0x4035,0x04,
	0x4036,0x00,
	0x4037,0x04,
	0x4040,0x00,
	0x4041,0x00,
	0x4042,0x00,
	0x4043,0x00,
	0x4045,0x00,
	0x4047,0x00,
	0x4510,0x07,
	0x4516,0x55,
	0x4517,0x55,
	0x4518,0x55,
	0x4519,0x55,
	0x451a,0x11,
	0x451b,0xbb,
	0x451c,0x11,
	0x451d,0xbb,
	0x451e,0x11,
	0x451f,0xbb,
	0x4520,0x11,
	0x4521,0xbb,
	0x4641,0x7f,
	0x4720,0x00,
	0x4721,0x00,
	0x4724,0x00,
	0x4733,0x0f,
	0x4734,0x0f,
	0x4735,0x0c,
	0x4736,0x04,
	0x4738,0x0f,
	0x4739,0x0f,
	0x473a,0x0c,
	0x473b,0x04,
	0x473e,0x0f,
	0x473f,0x0f,
	0x4740,0x0c,
	0x4741,0x04,
	0x4748,0x00,
	0x480e,0x04,
	0x5000,0x8b,
	0x5001,0x23,
	0x5002,0x9d,
	0x5003,0xca,
	0x5005,0x00,
	0x5017,0x00,
	0x5035,0x18,
	0x5037,0x14,
	0x5038,0x0f,
	0x5039,0xe0,
	0x503a,0x0b,
	0x503b,0xe0,
	0x5051,0x14,
	0x5187,0x88,
	0x518d,0x01,
	0x518f,0x01,
	0x5190,0x00,
	0x5191,0x00,
	0x5192,0x10,
	0x5193,0x0f,
	0x5194,0x00,
	0x5195,0x04,
	0x5c00,0x63,
	0x5c03,0x3f,
	0x5c05,0x54,
	0x5c44,0x02,
	0x5c45,0x0a,
	0x5c46,0x00,
	0x5c47,0x07,
	0x5c48,0x0f,
	0x5c49,0x01,
	0x5c4a,0x00,
	0x5c4b,0x09,
	0x5c4c,0x06,
	0x5c4d,0x0e,
	0x5c4e,0x00,
	0x5c4f,0x00,
	0x5c50,0x00,
	0x5c51,0x00,
	0x5c52,0x00,
	0x5c53,0x00,
	0x5c54,0x10,
	0x5c55,0x08,
	0x5c56,0x04,
	0x5c57,0x04,
	0x5c58,0x0c,
	0x5c59,0x0c,
	0x5c5a,0x02,
	0x5c5b,0x02,
	0x5c5c,0x06,
	0x5c5d,0x06,
	0x5c5e,0x01,
	0x5d08,0x08,
	0x5d09,0x08,
	0x5d0a,0x02,
	0x5d0b,0x02,
	0x5d0c,0x06,
	0x5d0d,0x06,
	0x5d0e,0x02,
	0x5d0f,0x02,
	0x5d10,0x06,
	0x5d11,0x06,
	0x380c,0x03,
	0x380d,0xd8,
	0x380e,0x0d,
	0x380f,0x3b,
	0x3501,0x01,
	0x3508,0x0f,
};
static void preview_setting(void)
{
	LOG_INF("preview_setting RES_4000x3000_30fps\n");
#if MULTI_WRITE
	LOG_INF("11preview_setting RES_4000x3000_30fps\n");
	ov50c40_table_write_cmos_sensor(addr_data_pair_preview_barley_ov50c40,
				   sizeof(addr_data_pair_preview_barley_ov50c40) / sizeof(kal_uint16));
#endif
}	/*	preview_setting  */
/*************************************************************************
* FUNCTION
*	Capture
*
* DESCRIPTION
*	Sensor capture
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
kal_uint16 addr_data_pair_capture_setting_barley_ov50c40[] = {
		0x3607,0x01,
	0x3608,0xda,
	0x360c,0x03,
	0x360e,0x1d,
	0x3684,0x03,
	0x369b,0x10,
	0x369c,0x2b,
	0x369d,0x40,
	0x3702,0x4b,
	0x3703,0x27,
	0x3706,0x2f,
	0x3709,0x69,
	0x370b,0x5d,
	0x3714,0x67,
	0x3717,0x03,
	0x373b,0x2a,
	0x373d,0x2e,
	0x3741,0x41,
	0x375f,0x80,
	0x37b0,0x37,
	0x37b3,0x3b,
	0x3800,0x00,
	0x3801,0x00,
	0x3802,0x00,
	0x3803,0x08,
	0x3804,0x20,
	0x3805,0x1f,
	0x3806,0x18,
	0x3807,0x17,
	0x3808,0x0f,
	0x3809,0xf0,
	0x380a,0x0c,
	0x380b,0x00,
	0x3811,0x10,
	0x3813,0x04,
	0x3814,0x11,
	0x3815,0x11,
	0x3820,0x02,
	0x3821,0x06,
	0x3822,0x10,
	0x383d,0x10,
	0x3845,0x04,
	0x3846,0x1a,
	0x3856,0x10,
	0x3857,0x10,
	0x3858,0x20,
	0x3859,0x20,
	0x3902,0x01,
	0x3903,0x08,
	0x3905,0x05,
	0x399b,0x02,
	0x399c,0x03,
	0x39aa,0x05,
	0x39ba,0x2f,
	0x39bc,0x2f,
	0x39be,0x2f,
	0x39c0,0x2f,
	0x39c2,0x5d,
	0x39c4,0x5d,
	0x39c6,0x5d,
	0x39c8,0x5d,
	0x4010,0x38,
	0x4016,0x0f,
	0x4018,0x03,
	0x401e,0x00,
	0x401f,0xd0,
	0x4031,0x00,
	0x4033,0x04,
	0x4034,0x00,
	0x4035,0x04,
	0x4036,0x00,
	0x4037,0x04,
	0x4040,0x00,
	0x4041,0x00,
	0x4042,0x00,
	0x4043,0x00,
	0x4045,0x00,
	0x4047,0x00,
	0x4510,0x07,
	0x4516,0x55,
	0x4517,0x55,
	0x4518,0x55,
	0x4519,0x55,
	0x451a,0x11,
	0x451b,0xbb,
	0x451c,0x11,
	0x451d,0xbb,
	0x451e,0x11,
	0x451f,0xbb,
	0x4520,0x11,
	0x4521,0xbb,
	0x4641,0x7f,
	0x4720,0x00,
	0x4721,0x00,
	0x4724,0x00,
	0x4733,0x0f,
	0x4734,0x0f,
	0x4735,0x0c,
	0x4736,0x04,
	0x4738,0x0f,
	0x4739,0x0f,
	0x473a,0x0c,
	0x473b,0x04,
	0x473e,0x0f,
	0x473f,0x0f,
	0x4740,0x0c,
	0x4741,0x04,
	0x4748,0x00,
	0x480e,0x04,
	0x5000,0x8b,
	0x5001,0x23,
	0x5002,0x9d,
	0x5003,0xca,
	0x5005,0x00,
	0x5017,0x00,
	0x5035,0x18,
	0x5037,0x14,
	0x5038,0x0f,
	0x5039,0xe0,
	0x503a,0x0b,
	0x503b,0xe0,
	0x5051,0x14,
	0x5187,0x88,
	0x518d,0x01,
	0x518f,0x01,
	0x5190,0x00,
	0x5191,0x00,
	0x5192,0x10,
	0x5193,0x0f,
	0x5194,0x00,
	0x5195,0x04,
	0x5c00,0x63,
	0x5c03,0x3f,
	0x5c05,0x54,
	0x5c44,0x02,
	0x5c45,0x0a,
	0x5c46,0x00,
	0x5c47,0x07,
	0x5c48,0x0f,
	0x5c49,0x01,
	0x5c4a,0x00,
	0x5c4b,0x09,
	0x5c4c,0x06,
	0x5c4d,0x0e,
	0x5c4e,0x00,
	0x5c4f,0x00,
	0x5c50,0x00,
	0x5c51,0x00,
	0x5c52,0x00,
	0x5c53,0x00,
	0x5c54,0x10,
	0x5c55,0x08,
	0x5c56,0x04,
	0x5c57,0x04,
	0x5c58,0x0c,
	0x5c59,0x0c,
	0x5c5a,0x02,
	0x5c5b,0x02,
	0x5c5c,0x06,
	0x5c5d,0x06,
	0x5c5e,0x01,
	0x5d08,0x08,
	0x5d09,0x08,
	0x5d0a,0x02,
	0x5d0b,0x02,
	0x5d0c,0x06,
	0x5d0d,0x06,
	0x5d0e,0x02,
	0x5d0f,0x02,
	0x5d10,0x06,
	0x5d11,0x06,
	0x380c,0x03,
	0x380d,0xd8,
	0x380e,0x0d,
	0x380f,0x3b,
	0x3501,0x01,
	0x3508,0x0f,
};
static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("capture 4224x3136_zsl_30fps currefps = %d\n", currefps);
#if MULTI_WRITE
		LOG_INF("11capture 4224x3136_zsl_30fps currefps = %d\n", currefps);
	ov50c40_table_write_cmos_sensor(
		addr_data_pair_capture_setting_barley_ov50c40,
		sizeof(addr_data_pair_capture_setting_barley_ov50c40) /
		sizeof(kal_uint16));
#endif
}

/*************************************************************************
* FUNCTION
*	Video
*
* DESCRIPTION
*	Sensor video
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/


kal_uint16 addr_data_pair_normal_video_setting_barley_ov50c40[] = {
	0x3607,0x01,
	0x3608,0xda,
	0x360c,0x03,
	0x360e,0x1d,
	0x3684,0x03,
	0x369b,0x10,
	0x369c,0x2b,
	0x369d,0x40,
	0x3702,0x4b,
	0x3703,0x27,
	0x3706,0x2f,
	0x3709,0x69,
	0x370b,0x5d,
	0x3714,0x67,
	0x3717,0x03,
	0x373b,0x2a,
	0x373d,0x2e,
	0x3741,0x41,
	0x375f,0x80,
	0x37b0,0x37,
	0x37b3,0x3b,
	0x3800,0x01,
	0x3801,0x00,
	0x3802,0x03,
	0x3803,0x98,
	0x3804,0x1f,
	0x3805,0x1f,
	0x3806,0x14,
	0x3807,0x87,
	0x3808,0x0f,
	0x3809,0x00,
	0x380a,0x08,
	0x380b,0x70,
	0x3811,0x08,
	0x3813,0x04,
	0x3814,0x11,
	0x3815,0x11,
	0x3820,0x02,
	0x3821,0x06,
	0x3822,0x10,
	0x383d,0x08,
	0x3845,0x03,
	0x3846,0xe8,
	0x3856,0x00,
	0x3857,0x00,
	0x3858,0x00,
	0x3859,0x00,
	0x3902,0x01,
	0x3903,0x08,
	0x3905,0x05,
	0x399b,0x02,
	0x399c,0x03,
	0x39aa,0x05,
	0x39ba,0x2f,
	0x39bc,0x2f,
	0x39be,0x2f,
	0x39c0,0x2f,
	0x39c2,0x5d,
	0x39c4,0x5d,
	0x39c6,0x5d,
	0x39c8,0x5d,
	0x4010,0x38,
	0x4016,0x0f,
	0x4018,0x03,
	0x401e,0x00,
	0x401f,0xd0,
	0x4031,0x00,
	0x4033,0x04,
	0x4034,0x00,
	0x4035,0x04,
	0x4036,0x00,
	0x4037,0x04,
	0x4040,0x00,
	0x4041,0x00,
	0x4042,0x00,
	0x4043,0x00,
	0x4045,0x00,
	0x4047,0x00,
	0x4510,0x07,
	0x4516,0x55,
	0x4517,0x55,
	0x4518,0x55,
	0x4519,0x55,
	0x451a,0x11,
	0x451b,0xbb,
	0x451c,0x11,
	0x451d,0xbb,
	0x451e,0x11,
	0x451f,0xbb,
	0x4520,0x11,
	0x4521,0xbb,
	0x4641,0x78,
	0x4720,0x00,
	0x4721,0x00,
	0x4724,0x00,
	0x4733,0x0f,
	0x4734,0x0f,
	0x4735,0x0c,
	0x4736,0x04,
	0x4738,0x0f,
	0x4739,0x0f,
	0x473a,0x0c,
	0x473b,0x04,
	0x473e,0x0f,
	0x473f,0x0f,
	0x4740,0x0c,
	0x4741,0x04,
	0x4748,0x00,
	0x480e,0x04,
	0x5000,0x8b,
	0x5001,0x23,
	0x5002,0x9d,
	0x5003,0xca,
	0x5005,0x00,
	0x5017,0x00,
	0x5035,0x18,
	0x5037,0x14,
	0x5038,0x0f,
	0x5039,0xe0,
	0x503a,0x0b,
	0x503b,0xe0,
	0x5051,0x14,
	0x5187,0x88,
	0x518d,0x01,
	0x518f,0x01,
	0x5190,0x00,
	0x5191,0x80,
	0x5192,0x0f,
	0x5193,0x8f,
	0x5194,0x01,
	0x5195,0xcc,
	0x5c00,0x63,
	0x5c03,0x3f,
	0x5c05,0x54,
	0x5c44,0x02,
	0x5c45,0x0a,
	0x5c46,0x00,
	0x5c47,0x07,
	0x5c48,0x0f,
	0x5c49,0x01,
	0x5c4a,0x00,
	0x5c4b,0x09,
	0x5c4c,0x06,
	0x5c4d,0x0e,
	0x5c4e,0x00,
	0x5c4f,0x00,
	0x5c50,0x00,
	0x5c51,0x00,
	0x5c52,0x00,
	0x5c53,0x00,
	0x5c54,0x10,
	0x5c55,0x08,
	0x5c56,0x04,
	0x5c57,0x04,
	0x5c58,0x0c,
	0x5c59,0x0c,
	0x5c5a,0x02,
	0x5c5b,0x02,
	0x5c5c,0x06,
	0x5c5d,0x06,
	0x5c5e,0x01,
	0x5d08,0x08,
	0x5d09,0x08,
	0x5d0a,0x02,
	0x5d0b,0x02,
	0x5d0c,0x06,
	0x5d0d,0x06,
	0x5d0e,0x02,
	0x5d0f,0x02,
	0x5d10,0x06,
	0x5d11,0x06,
	0x380c,0x03,
	0x380d,0xe8,
	0x380e,0x0d,
	0x380f,0x05,
	0x3501,0x01,
	0x3508,0x0f,
};

static void normal_video_setting(void)
{
	#if MULTI_WRITE
		LOG_INF("11capture 4224x3136_zsl_30fps normal_video_setting  \n");
	ov50c40_table_write_cmos_sensor(addr_data_pair_normal_video_setting_barley_ov50c40,
				   sizeof(addr_data_pair_normal_video_setting_barley_ov50c40) /
				   sizeof(kal_uint16));
	#endif

}
/*************************************************************************
* FUNCTION
*	Video
*
* DESCRIPTION
*	Sensor video
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/


kal_uint16 addr_data_pair_slim_video_setting_barley_ov50c40[] = {
	0x3607,0x00,
	0x3608,0xb2,
	0x360c,0x0b,
	0x360e,0x1e,
	0x3684,0x03,
	0x369b,0x80,
	0x369c,0x13,
	0x369d,0x00,
	0x3702,0x4b,
	0x3703,0x27,
	0x3706,0x26,
	0x3709,0x69,
	0x370b,0x56,
	0x3714,0x67,
	0x3717,0x01,
	0x373b,0x1e,
	0x373d,0x22,
	0x3741,0x3c,
	0x375f,0x00,
	0x37b0,0x32,
	0x37b3,0x36,
	0x3800,0x05,
	0x3801,0xe0,
	0x3802,0x04,
	0x3803,0x96,
	0x3804,0x1a,
	0x3805,0x3f,
	0x3806,0x13,
	0x3807,0x89,
	0x3808,0x09,
	0x3809,0x10,
	0x380a,0x06,
	0x380b,0xc8,
	0x3811,0x90,
	0x3813,0x59,
	0x3814,0x11,
	0x3815,0x11,
	0x3820,0x02,
	0x3821,0x14,
	0x3822,0x00,
	0x383d,0x90,
	0x3845,0x01,
	0x3846,0xf4,
	0x3856,0x20,
	0x3857,0x10,
	0x3858,0x40,
	0x3859,0x20,
	0x3902,0x02,
	0x3903,0x00,
	0x3905,0x28,
	0x399b,0x00,
	0x399c,0x00,
	0x39aa,0x55,
	0x39ba,0x26,
	0x39bc,0x26,
	0x39be,0x26,
	0x39c0,0x26,
	0x39c2,0x56,
	0x39c4,0x56,
	0x39c6,0x56,
	0x39c8,0x56,
	0x4010,0x38,
	0x4016,0x0f,
	0x4018,0x03,
	0x401e,0x01,
	0x401f,0x00,
	0x4031,0x02,
	0x4033,0x02,
	0x4034,0x00,
	0x4035,0x02,
	0x4036,0x00,
	0x4037,0x02,
	0x4040,0x00,
	0x4041,0x00,
	0x4042,0x00,
	0x4043,0x00,
	0x4045,0x00,
	0x4047,0x00,
	0x4510,0x00,
	0x4516,0x00,
	0x4517,0x00,
	0x4518,0x00,
	0x4519,0x00,
	0x451a,0x00,
	0x451b,0x00,
	0x451c,0x00,
	0x451d,0x00,
	0x451e,0x00,
	0x451f,0x00,
	0x4520,0x00,
	0x4521,0x00,
	0x4641,0x1f,
	0x4720,0x44,
	0x4721,0x42,
	0x4724,0x03,
	0x4733,0x0f,
	0x4734,0x0f,
	0x4735,0x0c,
	0x4736,0x04,
	0x4738,0x0f,
	0x4739,0x0f,
	0x473a,0x0c,
	0x473b,0x04,
	0x473e,0x0f,
	0x473f,0x0f,
	0x4740,0x0c,
	0x4741,0x04,
	0x4748,0x03,
	0x480e,0x00,
	0x5000,0x89,
	0x5001,0x03,
	0x5002,0x8d,
	0x5003,0xfa,
	0x5005,0x00,
	0x5017,0x00,
	0x5035,0x18,
	0x5037,0x14,
	0x5038,0x0f,
	0x5039,0xe0,
	0x503a,0x0b,
	0x503b,0xe0,
	0x5051,0x14,
	0x5187,0x88,
	0x518d,0x01,
	0x518f,0x01,
	0x5190,0x02,
	0x5191,0xf0,
	0x5192,0x0d,
	0x5193,0x1f,
	0x5194,0x02,
	0x5195,0x4b,
	0x5c00,0x61,
	0x5c03,0x3f,
	0x5c05,0x54,
	0x5c44,0x01,
	0x5c45,0x02,
	0x5c46,0x03,
	0x5c47,0x04,
	0x5c48,0x05,
	0x5c49,0x06,
	0x5c4a,0x07,
	0x5c4b,0x08,
	0x5c4c,0x09,
	0x5c4d,0x0a,
	0x5c4e,0x0b,
	0x5c4f,0x0c,
	0x5c50,0x0d,
	0x5c51,0x0e,
	0x5c52,0x0f,
	0x5c53,0x10,
	0x5c54,0x08,
	0x5c55,0x08,
	0x5c56,0x02,
	0x5c57,0x02,
	0x5c58,0x06,
	0x5c59,0x06,
	0x5c5a,0x02,
	0x5c5b,0x02,
	0x5c5c,0x06,
	0x5c5d,0x06,
	0x5c5e,0x00,
	0x5d08,0x08,
	0x5d09,0x08,
	0x5d0a,0x02,
	0x5d0b,0x02,
	0x5d0c,0x06,
	0x5d0d,0x06,
	0x5d0e,0x02,
	0x5d0f,0x02,
	0x5d10,0x06,
	0x5d11,0x06,
	0x380c,0x01,
	0x380d,0xe0,
	0x380e,0x0d,
	0x380f,0x90,
	0x3501,0x01,
	0x3508,0x0f,
};
static void slim_video_setting(void)
{
	#if MULTI_WRITE
	ov50c40_table_write_cmos_sensor(addr_data_pair_slim_video_setting_barley_ov50c40,
				   sizeof(addr_data_pair_slim_video_setting_barley_ov50c40) /
				   sizeof(kal_uint16));
	#endif

}
/*************************************************************************
* FUNCTION
*	Video
*
* DESCRIPTION
*	Sensor video
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/


kal_uint16 addr_data_pair_hs_video_setting_barley_ov50c40[] = {
	0x3607,0x00,
	0x3608,0xb2,
	0x360c,0x0b,
	0x360e,0x1e,
	0x3684,0x03,
	0x369b,0x80,
	0x369c,0x13,
	0x369d,0x00,
	0x3702,0x48,
	0x3703,0x20,
	0x3706,0x26,
	0x3709,0x75,
	0x370b,0x56,
	0x3714,0x63,
	0x3717,0x01,
	0x373b,0x1e,
	0x373d,0x22,
	0x3741,0x3c,
	0x375f,0x00,
	0x37b0,0x32,
	0x37b3,0x36,
	0x3800,0x00,
	0x3801,0xe0,
	0x3802,0x03,
	0x3803,0x90,
	0x3804,0x1f,
	0x3805,0x3f,
	0x3806,0x14,
	0x3807,0x8f,
	0x3808,0x07,
	0x3809,0x80,
	0x380a,0x04,
	0x380b,0x38,
	0x3811,0x0c,
	0x3813,0x04,
	0x3814,0x31,
	0x3815,0x31,
	0x3820,0x42,
	0x3821,0x1d,
	0x3822,0x08,
	0x383d,0x0c,
	0x3845,0x00,
	0x3846,0xf2,
	0x3856,0x00,
	0x3857,0x00,
	0x3858,0x00,
	0x3859,0x00,
	0x3902,0x02,
	0x3903,0x00,
	0x3905,0x28,
	0x399b,0x00,
	0x399c,0x00,
	0x39aa,0x55,
	0x39ba,0x26,
	0x39bc,0x26,
	0x39be,0x26,
	0x39c0,0x26,
	0x39c2,0x56,
	0x39c4,0x56,
	0x39c6,0x56,
	0x39c8,0x56,
	0x4010,0x28,
	0x4016,0x07,
	0x4018,0x01,
	0x401e,0x01,
	0x401f,0x00,
	0x4031,0x10,
	0x4033,0x10,
	0x4034,0x08,
	0x4035,0x10,
	0x4036,0x08,
	0x4037,0x10,
	0x4040,0x08,
	0x4041,0x10,
	0x4042,0x08,
	0x4043,0x10,
	0x4045,0x10,
	0x4047,0x10,
	0x4510,0x00,
	0x4516,0x00,
	0x4517,0x00,
	0x4518,0x00,
	0x4519,0x00,
	0x451a,0x00,
	0x451b,0x00,
	0x451c,0x00,
	0x451d,0x00,
	0x451e,0x00,
	0x451f,0x00,
	0x4520,0x00,
	0x4521,0x00,
	0x4641,0x1f,
	0x4720,0x44,
	0x4721,0x42,
	0x4724,0x03,
	0x4733,0x14,
	0x4734,0x14,
	0x4735,0x10,
	0x4736,0x09,
	0x4738,0x14,
	0x4739,0x14,
	0x473a,0x10,
	0x473b,0x09,
	0x473e,0x14,
	0x473f,0x14,
	0x4740,0x10,
	0x4741,0x09,
	0x4748,0x03,
	0x480e,0x00,
	0x5000,0x89,
	0x5001,0x02,
	0x5002,0x81,
	0x5003,0xfa,
	0x5005,0x04,
	0x5017,0x1b,
	0x5035,0x0c,
	0x5037,0x0a,
	0x5038,0x07,
	0x5039,0xc0,
	0x503a,0x05,
	0x503b,0xd0,
	0x5051,0x18,
	0x5187,0x99,
	0x518d,0x03,
	0x518f,0x03,
	0x5190,0x00,
	0x5191,0x70,
	0x5192,0x0f,
	0x5193,0x9f,
	0x5194,0x01,
	0x5195,0xc8,
	0x5c00,0x63,
	0x5c03,0x7e,
	0x5c05,0xa8,
	0x5c44,0x01,
	0x5c45,0x02,
	0x5c46,0x03,
	0x5c47,0x04,
	0x5c48,0x05,
	0x5c49,0x06,
	0x5c4a,0x07,
	0x5c4b,0x08,
	0x5c4c,0x09,
	0x5c4d,0x0a,
	0x5c4e,0x0b,
	0x5c4f,0x0c,
	0x5c50,0x0d,
	0x5c51,0x0e,
	0x5c52,0x0f,
	0x5c53,0x10,
	0x5c54,0x04,
	0x5c55,0x04,
	0x5c56,0x00,
	0x5c57,0x00,
	0x5c58,0x02,
	0x5c59,0x02,
	0x5c5a,0x00,
	0x5c5b,0x00,
	0x5c5c,0x02,
	0x5c5d,0x02,
	0x5c5e,0x00,
	0x5d08,0x04,
	0x5d09,0x04,
	0x5d0a,0x00,
	0x5d0b,0x00,
	0x5d0c,0x02,
	0x5d0d,0x02,
	0x5d0e,0x00,
	0x5d0f,0x00,
	0x5d10,0x02,
	0x5d11,0x02,
	0x380c,0x01,
	0x380d,0xb0,
	0x380e,0x07,
	0x380f,0x89,
	0x3501,0x01,
	0x3508,0x0f,
};
static void hs_video_setting(void)
{
	#if MULTI_WRITE
	ov50c40_table_write_cmos_sensor(addr_data_pair_hs_video_setting_barley_ov50c40,
				   sizeof(addr_data_pair_hs_video_setting_barley_ov50c40) /
				   sizeof(kal_uint16));
	#endif

}
static kal_uint32 streaming_control(kal_bool enable)
{
#if 1
	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
	if (enable) {
		write_cmos_sensor(0x0100, 0X01);
		mdelay(5);
	}
	else {
		write_cmos_sensor(0x0100, 0x00);
	}
#endif
	return ERROR_NONE;
}
kal_uint16 addr_data_pair_custom1_barley_ov50c40[] = {
	0x3607,0x01,
	0x3608,0xda,
	0x360c,0x03,
	0x360e,0x1d,
	0x3684,0x03,
	0x369b,0x10,
	0x369c,0x2b,
	0x369d,0x40,
	0x3702,0x4b,
	0x3703,0x27,
	0x3706,0x2f,
	0x3709,0x69,
	0x370b,0x5d,
	0x3714,0x67,
	0x3717,0x03,
	0x373b,0x2a,
	0x373d,0x2e,
	0x3741,0x41,
	0x375f,0x80,
	0x37b0,0x37,
	0x37b3,0x3b,
	0x3800,0x00,
	0x3801,0x00,
	0x3802,0x00,
	0x3803,0x08,
	0x3804,0x20,
	0x3805,0x1f,
	0x3806,0x18,
	0x3807,0x17,
	0x3808,0x0f,
	0x3809,0xf0,
	0x380a,0x0c,
	0x380b,0x00,
	0x3811,0x10,
	0x3813,0x04,
	0x3814,0x11,
	0x3815,0x11,
	0x3820,0x02,
	0x3821,0x06,
	0x3822,0x10,
	0x383d,0x10,
	0x3845,0x04,
	0x3846,0x1a,
	0x3856,0x10,
	0x3857,0x10,
	0x3858,0x20,
	0x3859,0x20,
	0x3902,0x01,
	0x3903,0x08,
	0x3905,0x05,
	0x399b,0x02,
	0x399c,0x03,
	0x39aa,0x05,
	0x39ba,0x2f,
	0x39bc,0x2f,
	0x39be,0x2f,
	0x39c0,0x2f,
	0x39c2,0x5d,
	0x39c4,0x5d,
	0x39c6,0x5d,
	0x39c8,0x5d,
	0x4010,0x38,
	0x4016,0x0f,
	0x4018,0x03,
	0x401e,0x00,
	0x401f,0xd0,
	0x4031,0x00,
	0x4033,0x04,
	0x4034,0x00,
	0x4035,0x04,
	0x4036,0x00,
	0x4037,0x04,
	0x4040,0x00,
	0x4041,0x00,
	0x4042,0x00,
	0x4043,0x00,
	0x4045,0x00,
	0x4047,0x00,
	0x4510,0x07,
	0x4516,0x55,
	0x4517,0x55,
	0x4518,0x55,
	0x4519,0x55,
	0x451a,0x11,
	0x451b,0xbb,
	0x451c,0x11,
	0x451d,0xbb,
	0x451e,0x11,
	0x451f,0xbb,
	0x4520,0x11,
	0x4521,0xbb,
	0x4641,0x7f,
	0x4720,0x00,
	0x4721,0x00,
	0x4724,0x00,
	0x4733,0x0f,
	0x4734,0x0f,
	0x4735,0x0c,
	0x4736,0x04,
	0x4738,0x0f,
	0x4739,0x0f,
	0x473a,0x0c,
	0x473b,0x04,
	0x473e,0x0f,
	0x473f,0x0f,
	0x4740,0x0c,
	0x4741,0x04,
	0x4748,0x00,
	0x480e,0x04,
	0x5000,0x8b,
	0x5001,0x23,
	0x5002,0x9d,
	0x5003,0xca,
	0x5005,0x00,
	0x5017,0x00,
	0x5035,0x18,
	0x5037,0x14,
	0x5038,0x0f,
	0x5039,0xe0,
	0x503a,0x0b,
	0x503b,0xe0,
	0x5051,0x14,
	0x5187,0x88,
	0x518d,0x01,
	0x518f,0x01,
	0x5190,0x00,
	0x5191,0x00,
	0x5192,0x10,
	0x5193,0x0f,
	0x5194,0x00,
	0x5195,0x04,
	0x5c00,0x63,
	0x5c03,0x3f,
	0x5c05,0x54,
	0x5c44,0x02,
	0x5c45,0x0a,
	0x5c46,0x00,
	0x5c47,0x07,
	0x5c48,0x0f,
	0x5c49,0x01,
	0x5c4a,0x00,
	0x5c4b,0x09,
	0x5c4c,0x06,
	0x5c4d,0x0e,
	0x5c4e,0x00,
	0x5c4f,0x00,
	0x5c50,0x00,
	0x5c51,0x00,
	0x5c52,0x00,
	0x5c53,0x00,
	0x5c54,0x10,
	0x5c55,0x08,
	0x5c56,0x04,
	0x5c57,0x04,
	0x5c58,0x0c,
	0x5c59,0x0c,
	0x5c5a,0x02,
	0x5c5b,0x02,
	0x5c5c,0x06,
	0x5c5d,0x06,
	0x5c5e,0x01,
	0x5d08,0x08,
	0x5d09,0x08,
	0x5d0a,0x02,
	0x5d0b,0x02,
	0x5d0c,0x06,
	0x5d0d,0x06,
	0x5d0e,0x02,
	0x5d0f,0x02,
	0x5d10,0x06,
	0x5d11,0x06,
	0x380c,0x03,
	0x380d,0xd8,
	0x380e,0x0d,
	0x380f,0x3b,
	0x3501,0x01,
	0x3508,0x0f,
};

static void custom1_setting(void)
{
	#if MULTI_WRITE
	LOG_INF("E\n");
	ov50c40_table_write_cmos_sensor(addr_data_pair_custom1_barley_ov50c40,
				   sizeof(addr_data_pair_custom1_barley_ov50c40) /
				   sizeof(kal_uint16));
	#endif
}

#define OV50C40_BARLEY_EEPROM_I2C_ADDR 0xA0
static BYTE ov50c40_barley_common_data[CAMERA_EEPPROM_COMDATA_LENGTH] = { 0 };
static kal_uint16 read_ov50c40_barley_eeprom_module(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pusendcmd, 2, (u8 *) &get_byte, 1, OV50C40_BARLEY_EEPROM_I2C_ADDR);

	return get_byte;
}

static void read_ov50c40_barley_module_data()
{
// ov50c40_barley_common_data
// length   : 64
// QR       : [8 ~ 26]
// vcm      : [40 ~ 41]
// lensid   : [44 ~ 45]
// sensorid : [30 ~ 33]

	kal_uint16 idx = 0;
    kal_uint16 sn_length = 23;
	kal_uint32 sn_starAddr = 0x5A;
	kal_uint32 vcmAddr = 0x0C;
	kal_uint32 lensAddr = 0x08;

	memset(ov50c40_barley_common_data, 0,sizeof(ov50c40_barley_common_data));
	// QR
	for(idx = 0; idx < sn_length; idx++)
	{
		ov50c40_barley_common_data[8 + idx] = read_ov50c40_barley_eeprom_module(sn_starAddr + idx);
	}
	//vcm
	ov50c40_barley_common_data[40] = read_ov50c40_barley_eeprom_module(vcmAddr);
	ov50c40_barley_common_data[41] = read_ov50c40_barley_eeprom_module(vcmAddr + 1);
	//lensid
	ov50c40_barley_common_data[44] = read_ov50c40_barley_eeprom_module(lensAddr);
	ov50c40_barley_common_data[45] = read_ov50c40_barley_eeprom_module(lensAddr + 1);

	for (idx = 0; idx < CAMERA_EEPPROM_COMDATA_LENGTH; idx = idx + 4)
		LOG_INF("cam data: %02x %02x %02x %02x\n",
		       ov50c40_barley_common_data[idx],
		       ov50c40_barley_common_data[idx + 1],
		       ov50c40_barley_common_data[idx + 2],
		       ov50c40_barley_common_data[idx + 3]);
}
/*#ifdef CONFIG_HQ_HARDWARE_INFO
#define DEBUG 0
static void get_eeprom_data(EEPROM_DATA *data)
{
	kal_uint8 i =0x0;
	u8 *otp_data = (u8 *)data;

	for (;i <= 0xE; i++, otp_data++)
		*otp_data = read_eeprom_module(i);

#if DEBUG
	otp_data = (u8 *)data;
	for (i=0;i<=0xE;i++)
		pr_err(" otpdata[0x%x]=0x%x    ", i, *(otp_data + i));
#endif
	return ;
}
#endif*/

/*************************************************************************
 * FUNCTION
 *	get_imgsensor_id
 *
 * DESCRIPTION
 *	This function get the sensor ID
 *
 * PARAMETERS
 *	*sensorID : return the sensor ID
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/

static kal_uint32 return_sensor_id(void)
{
return ((read_cmos_sensor(0x300a) << 16) |(read_cmos_sensor(0x300b) << 8) | read_cmos_sensor(0x300c));
}

// extern char back_cam_name[64];
// extern char back_cam_efuse_id[64];
// extern u32 dual_main_sensorid;

// static  void get_back_cam_efuse_id(void)
// {
	// int ret, i = 0;
	// kal_uint8 efuse_id;

	// ret = read_cmos_sensor(0x5000);
	// write_cmos_sensor(0x5000, (0x00 & 0x08) | (ret & (~0x08)));

	// write_cmos_sensor(0x0100, 0x01);
	// msleep(5);
	// write_cmos_sensor(0x3D84, 0x40);
	// write_cmos_sensor(0x3D88, 0x70);
	// write_cmos_sensor(0x3D89, 0x00);
	// write_cmos_sensor(0x3D8A, 0x70);
	// write_cmos_sensor(0x3D8B, 0x0f);
	// write_cmos_sensor(0x0100, 0x01);

	// for(i=0;i<16;i++)
	// {
		// efuse_id = read_cmos_sensor(0x7000+i);
		// sprintf(back_cam_efuse_id+2*i,"%02x",efuse_id);
		// msleep(1);
	// }
// }

static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	/* sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id();
			/* lx_revised */
			printk("[ov50c40]Read sensor id OK, write id:0x%x ,sensor Id:0x%x\n",
				imgsensor.i2c_write_id, *sensor_id);
			if (*sensor_id == imgsensor_info.sensor_id) {
				if(deviceInfo_register_value == 0x00) {
				    register_imgsensor_deviceinfo("Cam_r0", DEVICE_VERSION, imgsensor_info.module_id);
				    read_ov50c40_barley_module_data();
				    deviceInfo_register_value = 0x01;
			    }
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
									imgsensor.i2c_write_id, *sensor_id);
				/*Readout SN data*/
				return ERROR_NONE;
			}
			LOG_INF("Read sensor id OK:0x%x, id: 0x%x\n", imgsensor.i2c_write_id,
				*sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 1;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		/* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF */
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}

	LOG_INF("park Read sensor sensor_id fail, id: 0x%x\n", *sensor_id);
	return ERROR_NONE;
}


/*************************************************************************
 * FUNCTION
 *	open
 *
 * DESCRIPTION
 *	This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 open(void)
{
	/* const kal_uint8 i2c_addr[] = {IMGSENSOR_WRITE_ID_1, IMGSENSOR_WRITE_ID_2}; */
	kal_uint8 i = 0;
	kal_uint8 retry = 1;
	kal_uint32 sensor_id = 0;

	LOG_INF("PLATFORM:MIPI 4LANE ov50c40 open+++++ ++++\n");

	/* sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("[16885]i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, sensor_id);
				break;
			}
			LOG_INF("Read sensor id fail, id: 0x%x,sensor_id =0x%x\n",
				imgsensor.i2c_write_id, sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail sequence write in  */
	sensor_init();

//	write_cmos_sensor(0x0100, 0x00);
//	mdelay(10);
//	preview_setting();
//	write_cmos_sensor(0x0100, 0x01);

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x2D00;
	imgsensor.gain = 0x100;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	imgsensor.pdaf_mode = 0;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}				/*      open  */



/*************************************************************************
 * FUNCTION
 *	close
 *
 * DESCRIPTION
 *
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");

	/*No Need to implement this function */

	return ERROR_NONE;
}				/*      close  */


/*************************************************************************
 * FUNCTION
 * preview
 *
 * DESCRIPTION
 *	This function start the sensor preview.
 *
 * PARAMETERS
 *	*image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("preview E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	/* imgsensor.video_mode = KAL_FALSE; */
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.current_fps = imgsensor.current_fps;
/* imgsensor.autoflicker_en = KAL_FALSE; */
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();

	//mdelay(10);
	set_mirror_flip(imgsensor.mirror);
	return ERROR_NONE;
}				/*      preview   */

/*************************************************************************
 * FUNCTION
 *	capture
 *
 * DESCRIPTION
 *	This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("capture E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	imgsensor.pclk = imgsensor_info.cap.pclk;
	imgsensor.line_length = imgsensor_info.cap.linelength;
	imgsensor.frame_length = imgsensor_info.cap.framelength;
	imgsensor.min_frame_length = imgsensor_info.cap.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	capture_setting(imgsensor.current_fps);

	return ERROR_NONE;
}				/* capture() */

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			       MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("normal_video E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting();

	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}				/*      normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("hs_video E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();

	//mdelay(10);
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}				/*      hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			     MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("slim_video E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();

	//mdelay(10);
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}				/*      slim_video       */

static kal_uint32 Custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("custom1 E\n");
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
    imgsensor.pclk = imgsensor_info.custom1.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom1.linelength;
    imgsensor.frame_length = imgsensor_info.custom1.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom1_setting();
    return ERROR_NONE;
}   /*  Custom1   */
/*static kal_uint32 Custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
   LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
    imgsensor.pclk = imgsensor_info.custom2.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom2.linelength;
    imgsensor.frame_length = imgsensor_info.custom2.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
   // custom2_setting();
    return ERROR_NONE;
} */  /*  Custom2   */
/*static kal_uint32 Custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
   LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;
    imgsensor.pclk = imgsensor_info.custom3.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom3.linelength;
    imgsensor.frame_length = imgsensor_info.custom3.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom3.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    //custom3_setting();
    return ERROR_NONE;
} */  /*  Custom3   */
/*static kal_uint32 Custom4(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
   LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM4;
    imgsensor.pclk = imgsensor_info.custom4.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom4.linelength;
    imgsensor.frame_length = imgsensor_info.custom4.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom4.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom4_setting();
    return ERROR_NONE;
}  */ /*  Custom4   */


static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


	sensor_resolution->SensorHighSpeedVideoWidth = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight = imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight = imgsensor_info.slim_video.grabwindow_height;

	sensor_resolution->SensorCustom1Width = imgsensor_info.custom1.grabwindow_width;
	sensor_resolution->SensorCustom1Height = imgsensor_info.custom1.grabwindow_height;

	return ERROR_NONE;
}				/*      get_resolution  */

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			   MSDK_SENSOR_INFO_STRUCT *sensor_info,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d %d\n", scenario_id, sensor_info->SensorOutputDataFormat);

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;	/* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;	/* inverse with datasheet */
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4;	/* not use */
	sensor_info->SensorResetActiveHigh = FALSE;	/* not use */
	sensor_info->SensorResetDelayCount = 5;	/* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
	sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;


	sensor_info->SensorMasterClockSwitch = 0;	/* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;	/* The frame of setting
										 * shutter default 0 for TG int
										 */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting
												 * sensor gain
												 */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3;	/* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2;	/* not use */
	sensor_info->SensorPixelClockCount = 3;	/* not use */
	sensor_info->SensorDataLatchCount = 2;	/* not use */
	sensor_info->PDAF_Support = 2;	//0: NO PDAF, 1: PDAF Raw Data mode, 2:PDAF VC mode

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorHightSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;

	sensor_info->FrameTimeDelayFrame = imgsensor_info.frame_time_delay_frame;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

		sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

	break;
	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}

	return ERROR_NONE;
}				/*      get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			  MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("50C-scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		LOG_INF("preview_scenario_id = %d\n", scenario_id);
		preview(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		capture(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		normal_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		hs_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		slim_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		Custom1(image_window, sensor_config_data);
	break;

	default:
		LOG_INF("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}				/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	/* SetVideoMode Function should fix framerate */
	if (framerate == 0)	/* Dynamic frame rate */
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable)		/* enable auto flicker */
		imgsensor.autoflicker_en = KAL_TRUE;
	else			/* Cancel Auto flick */
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 set_max_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id,
						MUINT32 framerate)
{
	/* kal_int16 dummyLine; */
	kal_uint32 frameHeight;
	kal_uint32 frame_length;
	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	if (framerate == 0)
		return ERROR_NONE;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frameHeight =
		    imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		LOG_INF("frameHeight = %d\n", frameHeight);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frameHeight > imgsensor_info.pre.framelength)
		    ? (frameHeight - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
		frameHeight =
		    imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frameHeight >  imgsensor_info.normal_video.framelength)
			? (frameHeight - imgsensor_info.normal_video.framelength) : 0;
		imgsensor.frame_length =
		    imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);

		set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		frameHeight =
		    imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
		spin_lock(&imgsensor_drv_lock);

		imgsensor.dummy_line = (frameHeight > imgsensor_info.cap.framelength)
		    ? (frameHeight - imgsensor_info.cap.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frameHeight =
		    imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frameHeight >  imgsensor_info.hs_video.framelength)
		    ? (frameHeight - imgsensor_info.hs_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);

		set_dummy();
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frameHeight =
		    imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frameHeight > imgsensor_info.slim_video.framelength)
			? (frameHeight - imgsensor_info.slim_video.framelength) : 0;
		imgsensor.frame_length =
		    imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		frame_length = imgsensor_info.custom1.pclk / framerate * 10 / imgsensor_info.custom1.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom1.framelength) ? (frame_length - imgsensor_info.custom1.framelength) : 0;
		if (imgsensor.dummy_line < 0)
			imgsensor.dummy_line = 0;
		imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if(imgsensor.frame_length > imgsensor.shutter){
					set_dummy();
		}
		break;

	default:		/* coding with  preview scenario by default */
		frameHeight =
		    imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frameHeight > imgsensor_info.pre.framelength)
			? (frameHeight - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);

		set_dummy();
		LOG_INF("error scenario_id = %d, we use preview scenario\n", scenario_id);
		break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id,
						    MUINT32 *framerate)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		*framerate = imgsensor_info.pre.max_framerate;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		*framerate = imgsensor_info.normal_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*framerate = imgsensor_info.cap.max_framerate;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		*framerate = imgsensor_info.hs_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		*framerate = imgsensor_info.slim_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
        *framerate = imgsensor_info.custom1.max_framerate;
        break;

	default:
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable) {
		write_cmos_sensor(0x5000, 0x81);
		write_cmos_sensor(0x5081, 0x80);
	} else {
		write_cmos_sensor(0x5000, 0x8d);
		write_cmos_sensor(0x5081, 0x00);
	}

	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

/*
#define EEPROM_READ_ID  0xA0
static void read_eeprom(int offset, char *data, kal_uint32 size)
{
	int i = 0, addr = offset;
	char pu_send_cmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };

	for (i = 0; i < size; i++) {
		pu_send_cmd[0] = (char)(addr >> 8);
		pu_send_cmd[1] = (char)(addr & 0xFF);
		iReadRegI2C(pu_send_cmd, 2, &data[i], 1, EEPROM_READ_ID);

		addr++;
	}
}
*/

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
				  UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *)feature_para;

	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
	    (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;
	struct SET_PD_BLOCK_INFO_T *PDAFinfo;
    struct SENSOR_VC_INFO_STRUCT *pvcinfo;

	if (!((feature_id == 3040) || (feature_id == 3058)))
		LOG_INF("feature_id = %d\n", feature_id);

	LOG_INF("feature_id = %d, len=%d\n", feature_id, *feature_para_len);
	switch (feature_id) {
	case SENSOR_FEATURE_GET_GAIN_RANGE_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_gain;
		*(feature_data + 2) = imgsensor_info.max_gain;
		break;
	case SENSOR_FEATURE_GET_BASE_GAIN_ISO_AND_STEP:
		*(feature_data + 0) = imgsensor_info.min_gain_iso;
		*(feature_data + 1) = imgsensor_info.gain_step;
		*(feature_data + 2) = imgsensor_info.gain_type;
		break;
	case SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_shutter;
		*(feature_data + 2) = imgsensor_info.exp_step;
		break;
	case SENSOR_FEATURE_GET_BINNING_TYPE:
		switch (*(feature_data + 1)) {
		case MSDK_SCENARIO_ID_CUSTOM3:
			*feature_return_para_32 = 1; /*BINNING_NONE*/
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_CUSTOM1:
		default:
			*feature_return_para_32 = 1; /*BINNING_AVERAGED*/
			break;
		}
		pr_debug("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
			*feature_return_para_32);
		*feature_para_len = 4;

		break;
	case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
		/*
		 * 1, if driver support new sw frame sync
		 * set_shutter_frame_length() support third para auto_extend_en
		 */
		*(feature_data + 1) = 1;
		/* margin info by scenario */
		*(feature_data + 2) = imgsensor_info.margin;
		break;
	case SENSOR_FEATURE_GET_AWB_REQ_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 1;
			break;
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		night_mode((BOOL) * feature_data);
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) *feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor(sensor_reg_data->RegAddr,
				  sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData =
		    read_cmos_sensor(sensor_reg_data->RegAddr);
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/* get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE */
		/* if EEPROM does not exist in camera module. */
		*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode((BOOL) * feature_data_16,
				      *(feature_data_16 + 1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)
					      *feature_data,
					      *(feature_data + 1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)
						  *(feature_data),
						  (MUINT32 *) (uintptr_t) (*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_GET_PDAF_DATA:
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL) * feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:	/* for factory mode auto testing */
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE:
	   *feature_return_para_32 = imgsensor.current_ae_effective_frame;
	    break;
	case SENSOR_FEATURE_GET_AE_FRAME_MODE_FOR_LE:
	    memcpy(feature_return_para_32, &imgsensor.ae_frm_mode,
			 sizeof(struct IMGSENSOR_AE_FRM_MODE));
	    break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_INF("current fps :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = (UINT16)*feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		LOG_INF("current fps :%d\n", imgsensor.current_fps);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32) *feature_data);
		wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *) (uintptr_t) (*(feature_data + 1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[1],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[2],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[3],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[4],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[5],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[0],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n", (UINT16) *feature_data,
			(UINT16) *(feature_data + 1), (UINT16) *(feature_data + 2));
		ihdr_write_shutter_gain((UINT16) *feature_data, (UINT16) *(feature_data + 1),
					(UINT16) *(feature_data + 2));
		break;
	case SENSOR_FEATURE_GET_MODULE_INFO:
		break;
	case SENSOR_FEATURE_GET_MODULE_SN:
		break;
	case SENSOR_FEATURE_SET_SENSOR_OTP:
		break;
	case SENSOR_FEATURE_CHECK_MODULE_ID:
		break;
	case SENSOR_FEATURE_GET_EEPROM_COMDATA:
		memcpy(feature_return_para_32, ov50c40_barley_common_data,
				CAMERA_EEPPROM_COMDATA_LENGTH);
		*feature_para_len = CAMERA_EEPPROM_COMDATA_LENGTH;
		break;
	case SENSOR_FEATURE_GET_EEPROM_STEREODATA:
		break;
	case SENSOR_FEATURE_GET_DISTORTIONPARAMS:
		break;
/*	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n", *feature_data);
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
		break;*/
	case SENSOR_FEATURE_GET_4CELL_DATA:/*get 4 cell data from eeprom*/
	/*{
		int type = (kal_uint16)(*feature_data);
		char *data = (char *)((feature_data+1));

		if (type == FOUR_CELL_CAL_TYPE_XTALK_CAL) {
			read_eeprom(0x763, data, 600+2);
			LOG_INF("read Cross Talk calibration data size= %d %d\n", data[0], data[1]);
		} else if (type == FOUR_CELL_CAL_TYPE_DPC) {
			read_eeprom(0x9BE, data, 832+2);
			LOG_INF("read DPC calibration data size= %d %d\n", data[0], data[1]);
		}*/
		break;
	//
	// case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME://lzl
		// set_shutter_frame_length((UINT16)*feature_data,(UINT16)*(feature_data+1));
		// break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length((UINT16)(*feature_data), (UINT16)(*(feature_data + 1)), (BOOL) (*(feature_data + 2)));
		break;
    case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.cap.framelength << 16)
				+ imgsensor_info.cap.linelength;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.normal_video.framelength << 16)
				+ imgsensor_info.normal_video.linelength;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.hs_video.framelength << 16)
				+ imgsensor_info.hs_video.linelength;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.slim_video.framelength << 16)
				+ imgsensor_info.slim_video.linelength;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.pre.framelength << 16)
				+ imgsensor_info.pre.linelength;
				break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom1.framelength << 16)
				+ imgsensor_info.custom1.linelength;
			break;
		}
		break;
    case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.cap.pclk;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.normal_video.pclk;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.hs_video.pclk;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.slim_video.pclk;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom1.pclk;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.pclk;
		        break;
		}
		break;
	case SENSOR_FEATURE_GET_PDAF_INFO:
		PDAFinfo = (struct SET_PD_BLOCK_INFO_T *)
		    (uintptr_t) (*(feature_data + 1));

		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info,
				sizeof(struct SET_PD_BLOCK_INFO_T));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info_16_9,
				sizeof(struct SET_PD_BLOCK_INFO_T));
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info_custom1,
				sizeof(struct SET_PD_BLOCK_INFO_T));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		default:
			break;
		}
		break;

	case SENSOR_FEATURE_GET_VC_INFO:
		LOG_INF("SENSOR_FEATURE_GET_VC_INFO %d\n",(UINT16) *feature_data);
		pvcinfo = (struct SENSOR_VC_INFO_STRUCT *) (uintptr_t) (*(feature_data + 1));
		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[1],sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[2],sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[5],sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[0],sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		}
		break;

	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 1;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 1;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 1;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		default:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0;
			break;
		}
		break;
	case SENSOR_FEATURE_SET_PDAF:
			imgsensor.pdaf_mode = *feature_data_16;
		break;
#if 0
	case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
		*feature_return_para_i32 = get_sensor_temperature();
		*feature_para_len = 4;
		break;
#endif
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		streaming_control(KAL_FALSE);
		break;

	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
		break;

	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = imgsensor_info.cap.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = imgsensor_info.normal_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = imgsensor_info.hs_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = imgsensor_info.slim_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = imgsensor_info.custom1.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = imgsensor_info.pre.mipi_pixel_rate;
			break;
		}
		break;
	default:
		break;
	}
	LOG_INF("feature_control finish\n");
	return ERROR_NONE;
}				/*    feature_control()  */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 OV50C40_MIPI_RAW_BARLEY_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}				/*      OV5693_MIPI_RAW_SensorInit      */

/*****************************************************************************
 *
 * Copyright:
 * ---------
 * Copyright (C), 2023-2024, MetaSilicon Tech. Co., Ltd.
 * All rights reserved.
 *
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/types.h>
#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "mt510mipiraw_mumbaid_Sensor.h"

#define PFX "mt510_camera_sensor"
#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)
#define MAX_GAIN 240
#define MIN_GAIN 2
//#define MIPI_1LANE
#ifdef MT510_LONG_EXP
#define MAX_CIT_LSHIFT 7
#endif

#ifdef MT_SENSOR_CUSTOMIZED
static struct IMGSENSOR_LOAD_FROM_FILE *gp_load_file = NULL;
#endif

#define MULTI_WRITE 1
#if MULTI_WRITE
#define I2C_BUFFER_LEN 765
#else
#define I2C_BUFFER_LEN 3
#endif
static int GroupFlag = 0;
static DEFINE_SPINLOCK(imgsensor_drv_lock);
static BYTE mt510_mumbai_common_data[CAMERA_EEPPROM_COMDATA_LENGTH] = { 0 };
static struct imgsensor_info_struct imgsensor_info = {
    .sensor_id = MT510_SENSOR_ID_MUMBAID,
    .checksum_value = 0xbe0a4dd5,
    .pre = {
        .pclk = 96000000,
        .linelength =  1556,
        .framelength = 2056,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2592,
        .grabwindow_height = 1944,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 300,
        .mipi_pixel_rate = 192000000,
    },
    .cap = {
        .pclk = 96000000,
        .linelength =  1556,
        .framelength = 2056,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2592,
        .grabwindow_height = 1944,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 300,
        .mipi_pixel_rate = 192000000,
    },
    .normal_video = {
        .pclk = 96000000,
        .linelength =  1556,
        .framelength = 2056,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2592,
        .grabwindow_height = 1458,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 300,
        .mipi_pixel_rate = 192000000,
    },
    .hs_video = {
        .pclk = 96000000,
        .linelength =  1556,
        .framelength = 2056,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1296,
        .grabwindow_height = 972,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 300,
        .mipi_pixel_rate = 192000000,
    },
    .slim_video = {
        .pclk = 96000000,
        .linelength =  1556,
        .framelength = 2056,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2592,
        .grabwindow_height = 1458,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 300,
        .mipi_pixel_rate = 192000000,
     },

    .min_gain = 72,   // 1.125x, base 64
    .max_gain = 1024,  // 16x
    .min_gain_iso = 100,
    .gain_step = 4,
    .gain_type = 2,

    .margin = 6,
    .min_shutter = 2,
    .max_frame_length = 0xffff,
    .ae_shut_delay_frame = 0,
    .ae_sensor_gain_delay_frame = 0,
    .ae_ispGain_delay_frame = 2,
    .frame_time_delay_frame = 3,
    .ihdr_support = 0,                                       // 1:support; 0:not support
    .ihdr_le_firstline = 0,                                  // 1:le first ; 0:se first
    .sensor_mode_num = 5,                                    // support sensor mode num

    .cap_delay_frame = 3,
    .pre_delay_frame = 3,
    .video_delay_frame = 3,
    .hs_video_delay_frame = 3,
    .slim_video_delay_frame = 3,

    .isp_driving_current = ISP_DRIVING_4MA,
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
    .mipi_sensor_type = MIPI_OPHY_NCSI2,                      // 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
    .mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,          // 0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,
    .mclk = 24,

    .mipi_lane_num = SENSOR_MIPI_2_LANE,

    .i2c_addr_table = {0x50,0x52,0xff},
    .i2c_speed = 400,
};

static  imgsensor_struct imgsensor = {
    .mirror = IMAGE_NORMAL,                                     // mirrorflip information
    .sensor_mode = IMGSENSOR_MODE_INIT,                         // IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
    .shutter = 0x3D0,                                           // current shutter
    .gain = 0x100,                                              // current gain
    .dummy_pixel = 0,                                           // current dummypixel
    .dummy_line = 0,                                            // current dummyline
    .current_fps = 0,                                           // full size current fps : 24fps for PIP, 30fps for Normal or ZSD
    .autoflicker_en = KAL_FALSE,                                // auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
    .test_pattern = KAL_FALSE,                                  // test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
    .current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,     // current scenario id
    .ihdr_mode = 0,                                             // sensor need support LE, SE with HDR feature
    .i2c_write_id = 0x50,
#ifdef MT510_LONG_EXP
    .current_ae_effective_frame = 2,
#endif
};
/* Sensor output window information */
static  struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =
{
 { 2592, 1944,	  0,	0, 2592, 1944, 2592, 1944, 0000, 0000, 2592, 1944,    0, 0, 2592, 1944}, // Preview
 { 2592, 1944,	  0,	0, 2592, 1944, 2592, 1944, 0000, 0000, 2592, 1944,    0, 0, 2592, 1944}, // capture
 { 2592, 1944,	  0,  243, 2592, 1458, 2592, 1458, 0000, 0000, 2592, 1458,    0, 0, 2592, 1458}, // video
 { 2592, 1944,	  0,	0, 2592, 1944, 1296,  972, 0000, 0000, 1296,  972,    0, 0, 1296,  972}, //hight speed video
 { 2592, 1944,	  0,  243, 2592, 1458, 2592, 1458, 0000, 0000, 2592, 1458,    0, 0, 2592, 1458},// slim video
};
static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
    char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
    iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);
    LOG_INF("mt510 read_cmos_sensor addr is 0x%x, val is 0x%x", addr, get_byte);
    return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
    char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
    LOG_INF("mt510 write_cmos_sensor addr is 0x%x, val is 0x%x", addr, para);
    iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

static void set_dummy(void)
{
    LOG_INF("dummyline = %d, dummypixels = %d\n", imgsensor.dummy_line, imgsensor.dummy_pixel);
    write_cmos_sensor(0x0507, 1);
    write_cmos_sensor(0x0508, imgsensor.frame_length & 0xFF);
    write_cmos_sensor(0x0509, imgsensor.frame_length >> 8);
    write_cmos_sensor(0x050a, imgsensor.line_length & 0xFF);
    write_cmos_sensor(0x050b, imgsensor.line_length >> 8);
    write_cmos_sensor(0x0507, 0);
}

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
    kal_uint32 frame_length = imgsensor.frame_length;
    LOG_INF("framerate = %d, min framelength should enable = %d\n", framerate,min_framelength_en);

    frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
    spin_lock(&imgsensor_drv_lock);
    if(frame_length >= imgsensor.min_frame_length)
        imgsensor.frame_length = frame_length;
    else
        imgsensor.frame_length = imgsensor.min_frame_length;
    imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
    // imgsensor.dummy_line = dummy_line;
    // imgsensor.frame_length = frame_length + imgsensor.dummy_line;
    if(imgsensor.frame_length > imgsensor_info.max_frame_length)
    {
        imgsensor.frame_length = imgsensor_info.max_frame_length;
        imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
    }
    if(min_framelength_en)
        imgsensor.min_frame_length = imgsensor.frame_length;
    spin_unlock(&imgsensor_drv_lock);
    set_dummy();
}

static void update_shutter_frame_reg(kal_uint8 exp_mode, kal_uint8 long_exp_time, kal_uint32 shutter, kal_uint32 frame_length)
{
    write_cmos_sensor(0x0507, 0x01);
    write_cmos_sensor(0x050f, exp_mode & 0xFF);
    write_cmos_sensor(0x050e, long_exp_time & 0xFF);
    write_cmos_sensor(0x0508, frame_length & 0xFF);
    write_cmos_sensor(0x0509, (frame_length >> 8) & 0xFF);
    write_cmos_sensor(0x050c, shutter  & 0xFF);
    write_cmos_sensor(0x050d, (shutter >> 8) & 0xFF);
    write_cmos_sensor(0x0507, 0x00);
}

static void write_shutter(kal_uint32 shutter)
{
#ifdef MT510_LONG_EXP
    kal_uint8 exp_mode = 0;
    kal_uint8 l_shift = 0;
    kal_uint8 long_exp_time = 0;
#endif
    kal_uint16 realtime_fps = 0;

    spin_lock(&imgsensor_drv_lock);
    if(shutter > imgsensor.min_frame_length - imgsensor_info.margin)
        imgsensor.frame_length = shutter + imgsensor_info.margin;
    else
        imgsensor.frame_length = imgsensor.min_frame_length;
    if(imgsensor.frame_length > imgsensor_info.max_frame_length)
        imgsensor.frame_length = imgsensor_info.max_frame_length;
    spin_unlock(&imgsensor_drv_lock);
    if(shutter < imgsensor_info.min_shutter)
        shutter = imgsensor_info.min_shutter;

    realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
    if(imgsensor.autoflicker_en)
    {
        if(realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(296, 0);
        else if(realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(146, 0);
        else
            LOG_INF("autoflicker enable do not need change framerate\n");
    }
#ifdef MT510_LONG_EXP
    /* long expsoure */
    if (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
    {
        for (l_shift = 1; l_shift < MAX_CIT_LSHIFT; l_shift++) {
            if ((shutter >> l_shift)  < (imgsensor_info.max_frame_length - imgsensor_info.margin))
                break;
        }
        if (l_shift > MAX_CIT_LSHIFT) {
            LOG_INF("unable to set such a long exposure %d\n", shutter);
            l_shift = MAX_CIT_LSHIFT;
        }
        exp_mode = 0x01;
        long_exp_time = 0x08 | l_shift;
        shutter = shutter >> l_shift;
        imgsensor.frame_length = shutter + imgsensor_info.margin;
        LOG_INF("enter long exposure mode, time is %d", l_shift);

        // Update frame length and shutter of long exposure mode
        update_shutter_frame_reg(exp_mode, long_exp_time, shutter, imgsensor.frame_length);

        /* Frame exposure mode customization for LE*/
        imgsensor.ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
        imgsensor.ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
    }
    else
#endif
    {
        // Update frame length and shutter of normal mode
        update_shutter_frame_reg(0, 0, shutter, imgsensor.frame_length);
    }
#ifdef MT510_LONG_EXP
    imgsensor.current_ae_effective_frame = 2;
#endif
    LOG_INF("shutter =%d, framelength =%d, real_fps =%d\n", shutter, imgsensor.frame_length, realtime_fps);
}

/*************************************************************************
* FUNCTION
*       set_shutter
*
* DESCRIPTION
*       This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*       iShutter : exposured lines
*
* RETURNS
*       None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint32 shutter)
{
    unsigned long flags;
    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

    write_shutter(shutter);
}

static kal_uint16 gain2reg(const kal_uint16 gain)
{
    kal_uint16 val = MIN_GAIN;

    if (gain < imgsensor_info.min_gain)
        val = MIN_GAIN;
    else if (gain > imgsensor_info.max_gain)
        val = MAX_GAIN;
    else
        val = (kal_uint16)(gain * (MAX_GAIN - MIN_GAIN) / (imgsensor_info.max_gain - imgsensor_info.min_gain)) - 16;

    LOG_INF("plantform gain=%d, sensor gain= 0x%x\n", gain, val);
    return val;
}

/*************************************************************************
* FUNCTION
*       set_gain
*
* DESCRIPTION
*       This function is to set global gain to sensor.
*
* PARAMETERS
*       iGain : sensor global gain(base: 0x40)
*
* RETURNS
*       the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
    kal_uint16 reg_gain;

    if (gain < BASEGAIN)
    {
        LOG_INF("error gain setting = %d, base gain = %d", gain, BASEGAIN);
        gain = BASEGAIN;
    }
    if (gain > 16 * BASEGAIN)
    {
        LOG_INF("error gain setting = %d, max gain = %d", gain, 16 * BASEGAIN);
        gain = 16 * BASEGAIN;
    }

    reg_gain = gain2reg(gain);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.gain = 4 * reg_gain + 64;
    spin_unlock(&imgsensor_drv_lock);
    LOG_INF("gain = %d, reg_gain = 0x%x\n", gain, reg_gain);

    write_cmos_sensor(0x0507, 1);
    write_cmos_sensor(0x030b, reg_gain);
    write_cmos_sensor(0x0507, 0);

    return gain;
}

/*************************************************************************
* FUNCTION
*       night_mode
*
* DESCRIPTION
*       This function night mode of sensor.
*
* PARAMETERS
*       bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*       None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
}

static kal_uint16 mt510_table_write_cmos_sensor(
                    kal_uint16 *para, kal_uint32 len)
{
    char puSendCmd[I2C_BUFFER_LEN];
    kal_uint32 tosend = 0, IDX = 0;
    kal_uint16 addr = 0, addr_last = 0, data = 0;

    while (len > IDX) {
        addr = para[IDX];
        {
            if (addr == 0xFFFF)
            {
                data = para[IDX + 1];
                msleep(data);
                IDX += 2;
                LOG_INF("sleep time = %d, IDX = %d", data, IDX);
            }
            else
            {
                puSendCmd[tosend++] = (char)(addr >> 8);
                puSendCmd[tosend++] = (char)(addr & 0xFF);
                data = para[IDX + 1];
                puSendCmd[tosend++] = (char)(data & 0xFF);
                IDX += 2;
                addr_last = addr;
            }
        }

    #if MULTI_WRITE
        /* Write when remain buffer size is less than
        *3 bytes or reach end of data*/
        if ((I2C_BUFFER_LEN - tosend) < 3
            || IDX == len
            || addr != addr_last) {
            iBurstWriteReg_multi(puSendCmd,
                tosend,
                imgsensor.i2c_write_id,
                3,
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

kal_uint16 addr_data_pair_init_mt510d[] = {
    0x0d00, 0x00,
    0x0d01, 0x00,
    0x0d02, 0x82,
    0x0d03, 0x32,
    0x0d04, 0x18,
    0x0d05, 0x4c,
    0x0d06, 0x02,
    0x0d07, 0x05,
    0x0d08, 0x04,
    0x0d09, 0x00,
    0x0d10, 0x22,
    0x0d11, 0x2e,
    0x0d12, 0x28,
    0x0d13, 0x01,
    0x0d14, 0x11,
    0x0d15, 0x11,
    0x0008, 0x11,
    0x0009, 0x11,
    0x0014, 0x4b,
    0x0502, 0x02,
    0x0523, 0x02,
    0x0525, 0x40,
    0x0528, 0x00,
    0x0529, 0x00,
    0x052a, 0xe4,
    0x0544, 0x00,
    0x0545, 0x00,
    0x0550, 0x01,
    0x0301, 0x00,
    0x0302, 0x01,
    0x0303, 0xe8,
    0x0304, 0x03,
    0x0306, 0x01,
    0x0307, 0x30,
    0x0308, 0x01,
    0x030a, 0x00,
    0x030c, 0x70,
    0x0524, 0x07,
    0x051c, 0x05,
    0x051d, 0x01,
    0x051e, 0x05,
    0x051f, 0x02,
    0x0520, 0x05,
    0x0521, 0x02,
    0x0522, 0x05,
    0x052b, 0x00,
    0x052c, 0x40,
    0x052d, 0x01,
    0x052e, 0x92,
    0x052f, 0x01,
    0x0530, 0x12,
    0x0531, 0x00,
    0x0532, 0x88,
    0x0533, 0x00,
    0x0330, 0x00,
    0x0331, 0xff,
    0x0332, 0x3f,
    0x0333, 0xff,
    0x0334, 0x3f,
    0x0335, 0x00,
    0x0336, 0xff,
    0x0337, 0x3f,
    0x0338, 0xff,
    0x0339, 0x3f,
    0x033a, 0x00,
    0x033b, 0xff,
    0x033c, 0x3f,
    0x033d, 0xff,
    0x033e, 0x3f,
    0x033f, 0x00,
    0x0340, 0xff,
    0x0341, 0x3f,
    0x0342, 0xff,
    0x0343, 0x3f,
    0x0344, 0x01,
    0x0345, 0x6c,
    0x0346, 0x00,
    0x0347, 0x91,
    0x0348, 0x02,
    0x0349, 0x00,
    0x034a, 0xaa,
    0x034b, 0x00,
    0x034c, 0x4a,
    0x034d, 0x01,
    0x034e, 0x8a,
    0x034f, 0x01,
    0x0350, 0x91,
    0x0351, 0x02,
    0x0352, 0x01,
    0x0353, 0xe4,
    0x0354, 0x00,
    0x0355, 0x4a,
    0x0356, 0x01,
    0x0357, 0xc4,
    0x0358, 0x01,
    0x0359, 0x91,
    0x035a, 0x02,
    0x035b, 0x01,
    0x035c, 0xe4,
    0x035d, 0x00,
    0x035e, 0x4a,
    0x035f, 0x01,
    0x0360, 0xc4,
    0x0361, 0x01,
    0x0362, 0x91,
    0x0363, 0x02,
    0x0364, 0x01,
    0x0365, 0x6c,
    0x0366, 0x00,
    0x0367, 0x91,
    0x0368, 0x02,
    0x0369, 0x00,
    0x036a, 0xaa,
    0x036b, 0x00,
    0x036c, 0xad,
    0x036d, 0x00,
    0x036e, 0x4a,
    0x036f, 0x01,
    0x0370, 0x4d,
    0x0371, 0x01,
    0x0372, 0x8a,
    0x0373, 0x01,
    0x0374, 0x8d,
    0x0375, 0x01,
    0x0376, 0x91,
    0x0377, 0x02,
    0x0378, 0x96,
    0x0379, 0x02,
    0x037a, 0x01,
    0x037b, 0x6c,
    0x037c, 0x00,
    0x037d, 0x91,
    0x037e, 0x02,
    0x037f, 0x01,
    0x0380, 0xaa,
    0x0381, 0x00,
    0x0382, 0x91,
    0x0383, 0x02,
    0x0384, 0x01,
    0x0385, 0xa5,
    0x0386, 0x00,
    0x0387, 0x91,
    0x0388, 0x02,
    0x0389, 0x00,
    0x038a, 0xaa,
    0x038b, 0x00,
    0x038c, 0xaf,
    0x038d, 0x00,
    0x038e, 0x8a,
    0x038f, 0x01,
    0x0390, 0x8f,
    0x0391, 0x01,
    0x0392, 0x01,
    0x0393, 0x6c,
    0x0394, 0x00,
    0x0395, 0x91,
    0x0396, 0x02,
    0x0397, 0x01,
    0x0398, 0xda,
    0x0399, 0x00,
    0x039a, 0x48,
    0x039b, 0x01,
    0x039c, 0xba,
    0x039d, 0x01,
    0x039e, 0x8f,
    0x039f, 0x02,
    0x03a0, 0x00,
    0x03a1, 0xc6,
    0x03a2, 0x00,
    0x03a3, 0xd0,
    0x03a4, 0x00,
    0x03a5, 0xa6,
    0x03a6, 0x01,
    0x03a7, 0xb0,
    0x03a8, 0x01,
    0x03a9, 0x00,
    0x03aa, 0x4b,
    0x03ab, 0x01,
    0x03ac, 0x4c,
    0x03ad, 0x01,
    0x03ae, 0x92,
    0x03af, 0x02,
    0x03b0, 0x93,
    0x03b1, 0x02,
    0x03b2, 0x01,
    0x03b3, 0x50,
    0x03b4, 0x01,
    0x03b5, 0x52,
    0x03b6, 0x01,
    0x03b7, 0xa4,
    0x03b8, 0x02,
    0x03b9, 0xa6,
    0x03ba, 0x02,
    0x03bb, 0x00,
    0x03bc, 0x51,
    0x03bd, 0x01,
    0x03be, 0x52,
    0x03bf, 0x01,
    0x03c0, 0xa5,
    0x03c1, 0x02,
    0x03c2, 0xa6,
    0x03c3, 0x02,
    0x03cd, 0x00,
    0x03ce, 0xff,
    0x03cf, 0x3f,
    0x03d0, 0xff,
    0x03d1, 0x3f,
    0x03d2, 0xff,
    0x03d3, 0x3f,
    0x03d4, 0xff,
    0x03d5, 0x3f,
    0x03d6, 0x00,
    0x03d7, 0xff,
    0x03d8, 0x3f,
    0x03d9, 0xff,
    0x03da, 0x3f,
    0x03db, 0xff,
    0x03dc, 0x3f,
    0x03dd, 0xff,
    0x03de, 0x3f,
    0x03df, 0x00,
    0x03e0, 0xff,
    0x03e1, 0x3f,
    0x03e2, 0xff,
    0x03e3, 0x3f,
    0x03e4, 0xff,
    0x03e5, 0x3f,
    0x03e6, 0xff,
    0x03e7, 0x3f,
    0x03e8, 0x00,
    0x03e9, 0xff,
    0x03ea, 0x3f,
    0x03eb, 0xff,
    0x03ec, 0x3f,
    0x03ed, 0xff,
    0x03ee, 0x3f,
    0x03ef, 0xff,
    0x03f0, 0x3f,
    0x03f1, 0xff,
    0x03f2, 0x3f,
    0x03f3, 0xff,
    0x03f4, 0x3f,
    0x03f5, 0xff,
    0x03f6, 0x3f,
    0x03f7, 0xff,
    0x03f8, 0x3f,
    0x03f9, 0xff,
    0x03fa, 0x3f,
    0x03fb, 0xff,
    0x03fc, 0x3f,
    0x03fd, 0xff,
    0x03fe, 0x3f,
    0x03ff, 0xff,
    0x0400, 0x3f,
    0x0401, 0x00,
    0x0402, 0xff,
    0x0403, 0x3f,
    0x0404, 0xff,
    0x0405, 0x3f,
    0x0406, 0xff,
    0x0407, 0x3f,
    0x0408, 0xff,
    0x0409, 0x3f,
    0x040a, 0x00,
    0x040b, 0xff,
    0x040c, 0x3f,
    0x040d, 0xff,
    0x040e, 0x3f,
    0x040f, 0xff,
    0x0410, 0x3f,
    0x0411, 0xff,
    0x0412, 0x3f,
    0x0413, 0xff,
    0x0414, 0x3f,
    0x0415, 0xff,
    0x0416, 0x3f,
    0x0417, 0xff,
    0x0418, 0x3f,
    0x0419, 0xff,
    0x041a, 0x3f,
    0x0c00, 0x01,
    0x0804, 0x01,
    0x0805, 0x00,
    0x0821, 0x01,
    0x0828, 0xff,
    0x0829, 0x00,
    0x082a, 0x30,
    0x082b, 0x10,
    0x082c, 0x01,
    0x082d, 0x01,
    0x082e, 0x64,
    0x082f, 0x00,
    0x0830, 0x00,
    0x0831, 0x00,
    0x0832, 0x01,
    0x0833, 0x00,
    0x0834, 0x00,
    0x0835, 0x00,
    0x0836, 0x01,
    0x0838, 0x00,
    0x0839, 0x80,
    0x083a, 0x01,
    0x083b, 0x01,
    0x083c, 0x01,
    0x0885, 0x01,
    0x0886, 0x20,
    0x0887, 0x00,
    0x0884, 0x01,
    0x088b, 0x03,
    0x0c00, 0x00,
    0x0888, 0xff,
    0x0889, 0x01,
    0x088a, 0x3f,
    0x062d, 0x17,
    0x0601, 0x00,
    0x0602, 0x0c,
    0x0603, 0x04,
    0x0604, 0x03,
    0x060a, 0x01,
    0x060b, 0x2b,
    0x0610, 0x44,
    0x0611, 0x2f,
    0x0612, 0x24,
    0x0613, 0xd5,
    0x0614, 0x01,
    0x0615, 0x0d,
    0x0616, 0x07,
    0x0617, 0x0f,
    0x0618, 0x06,
    0x0619, 0x21,
    0x061a, 0x03,
    0x061b, 0x07,
    0x061c, 0x0f,
    0x061d, 0x09,
    0x061e, 0x0f,
    0x061f, 0x02,
    0x0620, 0x02,
    0x0621, 0x02,
    0x0632, 0xbb,
    0x0633, 0x0b,
};

static void sensor_init(void)
{
#ifdef MT_SENSOR_CUSTOMIZED
    int i = 0;
    if(gp_load_file && gp_load_file->init.size > 0)
    {
        LOG_INF("sensor_init set from file load size=%d\n", gp_load_file->init.size);
        for(i = 0; i < gp_load_file->init.size; i++)
        {
            if(gp_load_file->init.reg[i].RegAddr == 0xffff)
            {
                msleep(gp_load_file->init.reg[i].RegData);
                continue;
            }
            write_cmos_sensor(gp_load_file->init.reg[i].RegAddr, gp_load_file->init.reg[i].RegData);
        }
        return;
    }
#endif
    LOG_INF("sensor_init E\n");
    mt510_table_write_cmos_sensor(addr_data_pair_init_mt510d,
        sizeof(addr_data_pair_init_mt510d)/sizeof(kal_uint16));
}

kal_uint16 addr_data_pair_preview_mt510d[] = {
    0x0534, 0x4a,
    0x0535, 0x01,
    0x0536, 0x88,
    0x0537, 0x01,
    0x0538, 0x54,
    0x0539, 0x04,
    0x053a, 0x92,
    0x053b, 0x04,
    0x053c, 0x5e,
    0x053d, 0x07,
    0x053e, 0x9c,
    0x053f, 0x07,
    0x0540, 0x68,
    0x0541, 0x0a,
    0x0542, 0xa6,
    0x0543, 0x0a,
    0x030e, 0x01,
    0x03c4, 0x00,
    0x03c5, 0x05,
    0x03c6, 0x00,
    0x03c7, 0x4b,
    0x03c8, 0x01,
    0x03c9, 0x59,
    0x03ca, 0x01,
    0x03cb, 0x9f,
    0x03cc, 0x02,
    0x0507, 0x01,
    0x0501, 0x00,
    0x0801, 0x03,
    0x0609, 0x03,
    0x0503, 0x00,
    0x0504, 0x01,
    0x0505, 0x01,
    0x0506, 0x00,
    0x0508, 0x08,
    0x0509, 0x08,
    0x050a, 0x14,
    0x050b, 0x06,
    0x050c, 0x00,
    0x050d, 0x01,
    0x050e, 0x00,
    0x050f, 0x00,
    0x0510, 0x01,
    0x0511, 0x01,
    0x0512, 0x00,
    0x0513, 0x18,
    0x0514, 0x00,
    0x0515, 0x08,
    0x0516, 0x00,
    0x0517, 0x00,
    0x0518, 0x98,
    0x0519, 0x07,
    0x051a, 0x00,
    0x051b, 0x08,
    0x030b, 0x02,
    0x0526, 0x01,
    0x0527, 0x01,
    0x0507, 0x00,
    0x0822, 0x02,
    0x0823, 0x10,
    0x0824, 0x01,
    0x0825, 0x10,
    0x0826, 0x09,
    0x0827, 0x01,
    0x08dc, 0x00,
    0x08dd, 0x00,
    0x08de, 0x00,
    0x08df, 0x00,
    0x08e0, 0x20,
    0x08e1, 0x0a,
    0x08e2, 0x98,
    0x08e3, 0x07,
    0x060c, 0x20,
    0x060d, 0x0a,
    0x060e, 0x98,
    0x060f, 0x07,
    0x08da, 0x01,
    0x0800, 0x01,
};

static void preview_setting(void)
{
    // Sensor Information////////////////////////////
    // Sensor            : mt510
    // Date              : 2023-10-30
    // Image size        : 1632x1224(binning)
    // Frame Length      : 642@2lane/641@4lane
    // Line Length       : 4280@2lane/4368@4lane
    // line Time         : 25939@2lane/26000@4lane
    // Max Fps           : 60.00fps
    // Pixel order       : Green 1st (=GB)
    // X-mirror/Y-flip   : x-mirror/y-flip
    // BLC offset        : 64code
    ////////////////////////////////////////////////
#ifdef MT_SENSOR_CUSTOMIZED
    int i = 0;
    if(gp_load_file && gp_load_file->preview.size > 0)
    {
        LOG_INF("preview_setting set from file load size=%d\n", gp_load_file->preview.size);
        for(i = 0; i < gp_load_file->preview.size; i++)
        {
            if(gp_load_file->preview.reg[i].RegAddr == 0xffff)
            {
                msleep(gp_load_file->preview.reg[i].RegData);
                continue;
            }
            write_cmos_sensor(gp_load_file->preview.reg[i].RegAddr, gp_load_file->preview.reg[i].RegData);
        }
        return;
    }
#endif
    LOG_INF("preview_setting E\n");
    mt510_table_write_cmos_sensor(addr_data_pair_preview_mt510d,
        sizeof(addr_data_pair_preview_mt510d)/sizeof(kal_uint16));
}

kal_uint16 addr_data_pair_capture_mt510d[] = {
    0x0534, 0x4a,
    0x0535, 0x01,
    0x0536, 0x88,
    0x0537, 0x01,
    0x0538, 0x54,
    0x0539, 0x04,
    0x053a, 0x92,
    0x053b, 0x04,
    0x053c, 0x5e,
    0x053d, 0x07,
    0x053e, 0x9c,
    0x053f, 0x07,
    0x0540, 0x68,
    0x0541, 0x0a,
    0x0542, 0xa6,
    0x0543, 0x0a,
    0x030e, 0x01,
    0x03c4, 0x00,
    0x03c5, 0x05,
    0x03c6, 0x00,
    0x03c7, 0x4b,
    0x03c8, 0x01,
    0x03c9, 0x59,
    0x03ca, 0x01,
    0x03cb, 0x9f,
    0x03cc, 0x02,
    0x0507, 0x01,
    0x0501, 0x00,
    0x0801, 0x03,
    0x0609, 0x03,
    0x0503, 0x00,
    0x0504, 0x01,
    0x0505, 0x01,
    0x0506, 0x00,
    0x0508, 0x08,
    0x0509, 0x08,
    0x050a, 0x14,
    0x050b, 0x06,
    0x050c, 0x00,
    0x050d, 0x01,
    0x050e, 0x00,
    0x050f, 0x00,
    0x0510, 0x01,
    0x0511, 0x01,
    0x0512, 0x00,
    0x0513, 0x18,
    0x0514, 0x00,
    0x0515, 0x08,
    0x0516, 0x00,
    0x0517, 0x00,
    0x0518, 0x98,
    0x0519, 0x07,
    0x051a, 0x00,
    0x051b, 0x08,
    0x030b, 0x02,
    0x0526, 0x01,
    0x0527, 0x01,
    0x0507, 0x00,
    0x0822, 0x02,
    0x0823, 0x10,
    0x0824, 0x01,
    0x0825, 0x10,
    0x0826, 0x09,
    0x0827, 0x01,
    0x08dc, 0x00,
    0x08dd, 0x00,
    0x08de, 0x00,
    0x08df, 0x00,
    0x08e0, 0x20,
    0x08e1, 0x0a,
    0x08e2, 0x98,
    0x08e3, 0x07,
    0x060c, 0x20,
    0x060d, 0x0a,
    0x060e, 0x98,
    0x060f, 0x07,
    0x08da, 0x01,
    0x0800, 0x01,
};

static void capture_setting(kal_uint16 currefps)
{
    // Sensor Information////////////////////////////
    // Sensor           : mt510
    // Date             : 2023-10-30
    // Image size       : 3264x2448(full_size)
    // Frame Length     : 1285@2lane/1282@4lane
    // Line Length      : 4280@2lane/4368@4lane
    // line Time        : 25939@2lane/26000@4lane
    // Pixel order      : Green 1st (=GB)
    // Max Fps          : 30.00fps
    // Pixel order      : Green 1st (=GB)
    // X-mirror/Y-flip  : x-mirror/y-flip
    // BLC offset       : 64code
    ////////////////////////////////////////////////
#ifdef MT_SENSOR_CUSTOMIZED
    int i = 0;
    if(gp_load_file && gp_load_file->capture.size > 0)
    {
        LOG_INF("capture_setting set from file load size=%d\n", gp_load_file->capture.size);
        for(i = 0; i < gp_load_file->capture.size; i++)
        {
            if(gp_load_file->capture.reg[i].RegAddr == 0xffff)
            {
                msleep(gp_load_file->capture.reg[i].RegData);
                continue;
            }
            write_cmos_sensor(gp_load_file->capture.reg[i].RegAddr, gp_load_file->capture.reg[i].RegData);
        }
        return;
    }
#endif
    LOG_INF("capture_setting E\n");
    mt510_table_write_cmos_sensor(addr_data_pair_capture_mt510d,
        sizeof(addr_data_pair_capture_mt510d)/sizeof(kal_uint16));
}

kal_uint16 addr_data_pair_normal_video_mt510d[] = {
    0x0534, 0x4a,
    0x0535, 0x01,
    0x0536, 0x88,
    0x0537, 0x01,
    0x0538, 0x54,
    0x0539, 0x04,
    0x053a, 0x92,
    0x053b, 0x04,
    0x053c, 0x5e,
    0x053d, 0x07,
    0x053e, 0x9c,
    0x053f, 0x07,
    0x0540, 0x68,
    0x0541, 0x0a,
    0x0542, 0xa6,
    0x0543, 0x0a,
    0x030e, 0x01,
    0x03c4, 0x00,
    0x03c5, 0x05,
    0x03c6, 0x00,
    0x03c7, 0x4b,
    0x03c8, 0x01,
    0x03c9, 0x59,
    0x03ca, 0x01,
    0x03cb, 0x9f,
    0x03cc, 0x02,
    0x0507, 0x01,
    0x0501, 0x00,
    0x0801, 0x03,
    0x0609, 0x03,
    0x0503, 0x00,
    0x0504, 0x01,
    0x0505, 0x01,
    0x0506, 0x00,
    0x0508, 0x08,
    0x0509, 0x08,
    0x050a, 0x14,
    0x050b, 0x06,
    0x050c, 0x00,
    0x050d, 0x01,
    0x050e, 0x00,
    0x050f, 0x00,
    0x0510, 0x01,
    0x0511, 0x01,
    0x0512, 0x00,
    0x0513, 0x18,
    0x0514, 0x00,
    0x0515, 0x08,
    0x0516, 0xf4,
    0x0517, 0x00,
    0x0518, 0xb2,
    0x0519, 0x05,
    0x051a, 0x00,
    0x051b, 0x08,
    0x030b, 0x02,
    0x0526, 0x01,
    0x0527, 0x01,
    0x0507, 0x00,
    0x0822, 0x02,
    0x0823, 0x10,
    0x0824, 0x01,
    0x0825, 0x10,
    0x0826, 0x09,
    0x0827, 0x01,
    0x08dc, 0x00,
    0x08dd, 0x00,
    0x08de, 0x00,
    0x08df, 0x00,
    0x08e0, 0x20,
    0x08e1, 0x0a,
    0x08e2, 0xb2,
    0x08e3, 0x05,
    0x060c, 0x20,
    0x060d, 0x0a,
    0x060e, 0xb2,
    0x060f, 0x05,
    0x08da, 0x01,
    0x0800, 0x01,
};

static void normal_video_setting(kal_uint16 currefps)
{
#ifdef MT_SENSOR_CUSTOMIZED
    int i = 0;
    if(gp_load_file && gp_load_file->normal_video.size > 0)
    {
        LOG_INF("normal_video_setting set from file load size=%d\n", gp_load_file->normal_video.size);
        for(i = 0; i < gp_load_file->normal_video.size; i++)
        {
            if(gp_load_file->normal_video.reg[i].RegAddr == 0xffff)
            {
                msleep(gp_load_file->normal_video.reg[i].RegData);
                continue;
            }
            write_cmos_sensor(gp_load_file->normal_video.reg[i].RegAddr, gp_load_file->normal_video.reg[i].RegData);
        }
        return;
    }
#endif
    LOG_INF("normal_video_setting E\n");
    mt510_table_write_cmos_sensor(addr_data_pair_normal_video_mt510d,
        sizeof(addr_data_pair_normal_video_mt510d)/sizeof(kal_uint16));
}

kal_uint16 addr_data_pair_hs_video_mt510d[] = {
    0x0534, 0x4a,
    0x0535, 0x01,
    0x0536, 0x88,
    0x0537, 0x01,
    0x0538, 0x54,
    0x0539, 0x04,
    0x053a, 0x92,
    0x053b, 0x04,
    0x053c, 0x5e,
    0x053d, 0x07,
    0x053e, 0x9c,
    0x053f, 0x07,
    0x0540, 0x68,
    0x0541, 0x0a,
    0x0542, 0xa6,
    0x0543, 0x0a,
    0x030e, 0x01,
    0x03c4, 0x00,
    0x03c5, 0x05,
    0x03c6, 0x00,
    0x03c7, 0x4b,
    0x03c8, 0x01,
    0x03c9, 0x59,
    0x03ca, 0x01,
    0x03cb, 0x9f,
    0x03cc, 0x02,
    0x0507, 0x01,
    0x0501, 0x01,
    0x0801, 0x02,
    0x0609, 0x03,
    0x0503, 0x00,
    0x0504, 0x01,
    0x0505, 0x01,
    0x0506, 0x00,
    0x0508, 0x08,
    0x0509, 0x08,
    0x050a, 0x14,
    0x050b, 0x06,
    0x050c, 0x00,
    0x050d, 0x01,
    0x050e, 0x00,
    0x050f, 0x00,
    0x0510, 0x01,
    0x0511, 0x01,
    0x0512, 0x00,
    0x0513, 0x18,
    0x0514, 0x00,
    0x0515, 0x08,
    0x0516, 0x00,
    0x0517, 0x00,
    0x0518, 0x98,
    0x0519, 0x07,
    0x051a, 0x00,
    0x051b, 0x08,
    0x030b, 0x02,
    0x0526, 0x01,
    0x0527, 0x01,
    0x0507, 0x00,
    0x0822, 0x02,
    0x0823, 0x10,
    0x0824, 0x01,
    0x0825, 0x10,
    0x0826, 0x09,
    0x0827, 0x01,
    0x08dc, 0x00,
    0x08dd, 0x00,
    0x08de, 0x00,
    0x08df, 0x00,
    0x08e0, 0x10,
    0x08e1, 0x05,
    0x08e2, 0xcc,
    0x08e3, 0x03,
    0x060c, 0x10,
    0x060d, 0x05,
    0x060e, 0xcc,
    0x060f, 0x03,
    0x08da, 0x01,
    0x0800, 0x01,
};

static void hs_video_setting(void)
{
    // Sensor Information////////////////////////////
    // Sensor            : mt510
    // Date              : 2023-10-30
    // Customer          : SPRD_validation
    // Image size        : 640x480(bining+crop)
    // Frame Length     : 321@2lane/320@4lane
    // Line Length      : 4280@2lane/4368@4lane
    // line Time        : 25939@2lane/26000@4lane
    // Max Fps           : 120.00fps
    // Pixel order       : Green 1st (=GB)
    // X-mirror/Y-flip   : x-mirror/y-flip
    // BLC offset        : 64code
    ////////////////////////////////////////////////
#ifdef MT_SENSOR_CUSTOMIZED
    int i = 0;
    if(gp_load_file && gp_load_file->hs_video.size > 0)
    {
        LOG_INF("hs_video_setting set from file load size=%d\n", gp_load_file->hs_video.size);
        for(i = 0; i < gp_load_file->hs_video.size; i++)
        {
            if(gp_load_file->hs_video.reg[i].RegAddr == 0xffff)
            {
                msleep(gp_load_file->hs_video.reg[i].RegData);
                continue;
            }
            write_cmos_sensor(gp_load_file->hs_video.reg[i].RegAddr, gp_load_file->hs_video.reg[i].RegData);
        }
        return;
    }
#endif
    LOG_INF("hs_video_setting E\n");
    mt510_table_write_cmos_sensor(addr_data_pair_hs_video_mt510d,
        sizeof(addr_data_pair_hs_video_mt510d)/sizeof(kal_uint16));
}

kal_uint16 addr_data_pair_silm_video_mt510d[] = {
    0x0534, 0x4a,
    0x0535, 0x01,
    0x0536, 0x88,
    0x0537, 0x01,
    0x0538, 0x54,
    0x0539, 0x04,
    0x053a, 0x92,
    0x053b, 0x04,
    0x053c, 0x5e,
    0x053d, 0x07,
    0x053e, 0x9c,
    0x053f, 0x07,
    0x0540, 0x68,
    0x0541, 0x0a,
    0x0542, 0xa6,
    0x0543, 0x0a,
    0x030e, 0x01,
    0x03c4, 0x00,
    0x03c5, 0x05,
    0x03c6, 0x00,
    0x03c7, 0x4b,
    0x03c8, 0x01,
    0x03c9, 0x59,
    0x03ca, 0x01,
    0x03cb, 0x9f,
    0x03cc, 0x02,
    0x0507, 0x01,
    0x0501, 0x00,
    0x0801, 0x03,
    0x0609, 0x03,
    0x0503, 0x00,
    0x0504, 0x01,
    0x0505, 0x01,
    0x0506, 0x00,
    0x0508, 0x08,
    0x0509, 0x08,
    0x050a, 0x14,
    0x050b, 0x06,
    0x050c, 0x00,
    0x050d, 0x01,
    0x050e, 0x00,
    0x050f, 0x00,
    0x0510, 0x01,
    0x0511, 0x01,
    0x0512, 0x00,
    0x0513, 0x18,
    0x0514, 0x00,
    0x0515, 0x08,
    0x0516, 0xf4,
    0x0517, 0x00,
    0x0518, 0xb2,
    0x0519, 0x05,
    0x051a, 0x00,
    0x051b, 0x08,
    0x030b, 0x02,
    0x0526, 0x01,
    0x0527, 0x01,
    0x0507, 0x00,
    0x0822, 0x02,
    0x0823, 0x10,
    0x0824, 0x01,
    0x0825, 0x10,
    0x0826, 0x09,
    0x0827, 0x01,
    0x08dc, 0x00,
    0x08dd, 0x00,
    0x08de, 0x00,
    0x08df, 0x00,
    0x08e0, 0x20,
    0x08e1, 0x0a,
    0x08e2, 0xb2,
    0x08e3, 0x05,
    0x060c, 0x20,
    0x060d, 0x0a,
    0x060e, 0xb2,
    0x060f, 0x05,
    0x08da, 0x01,
    0x0800, 0x01,
};

static void slim_video_setting(void)
{
    // Sensor Information////////////////////////////
    // Sensor            : mt510
    // Date              : 2023-10-30
    // Customer          : SPRD_validation
    // Image size        : 1280x720(bining+crop)
    // Frame Length     : 428@2lane/427@4lane
    // Line Length      : 4280@2lane/4368@4lane
    // line Time        : 25939@2lane/26000@4lane
    // Max Fps           : 90.00fps
    // Pixel order       : Green 1st (=GB)
    // X-mirror/Y-flip   : x-mirror/y-flip
    // BLC offset        : 64code
    ////////////////////////////////////////////////
#ifdef MT_SENSOR_CUSTOMIZED
    int i = 0;
    if(gp_load_file && gp_load_file->slim_video.size > 0)
    {
        LOG_INF("slim_video_setting set from file load size=%d\n", gp_load_file->slim_video.size);
        for(i = 0; i < gp_load_file->slim_video.size; i++)
        {
            if(gp_load_file->slim_video.reg[i].RegAddr == 0xffff)
            {
                msleep(gp_load_file->slim_video.reg[i].RegData);
                continue;
            }
            write_cmos_sensor(gp_load_file->slim_video.reg[i].RegAddr, gp_load_file->slim_video.reg[i].RegData);
        }
        return;
    }
#endif
    LOG_INF("slim_video_setting E\n");
    mt510_table_write_cmos_sensor(addr_data_pair_silm_video_mt510d,
        sizeof(addr_data_pair_silm_video_mt510d)/sizeof(kal_uint16));
}

static void mt510_otp_init()
{
    /* 1. Enable OTP work mode */
	write_cmos_sensor(0x0D01, 0x00);
	write_cmos_sensor(0x0C00, 0x01);
    /* 2. Configure read mode */
	write_cmos_sensor(0x0C19, 0x00);

}

static void mt510_set_start_addr(u32 addr)
{

	write_cmos_sensor(0x0c13, (addr >> 8) & 0xff);
	write_cmos_sensor(0x0c14, addr & 0xff);

}

static void mt510_otp_trigger()
{

    write_cmos_sensor(0x0c16, 0x50);
	write_cmos_sensor(0x0c16, 0x70);
}

static int mt510_otp_wait()
{
    u8 cnt = 0;
    /* 5. Wait for read completion */
    for (cnt = 0; cnt < 10; cnt++) {
        if (read_cmos_sensor(0x0C1A) == 0x01)
            break;
    }
    if (cnt >= 10) {
        LOG_INF("OTP read timeout");
        return -1;
    }

    return 0;
}

static void mt510_otp_close()
{
    /* 7. Exit OTP working status */
    write_cmos_sensor(0x0C17, 0x01);
    /* 8. Disable OTP work */
    write_cmos_sensor(0x0C00, 0x00);
}

static int read_ready(u32 start_addr)
{
    mt510_otp_init();

	mt510_set_start_addr(start_addr);

    mt510_otp_trigger();

	if (mt510_otp_wait() != 0) {
		LOG_INF("OTP read timeout");
        return -1;
	}
    return 0;
}

static u32 mt510_read(u32 addr)
{
    int ret = -1;
    u32 addr_offset = 0x1000;
	u32 map_addr = addr + addr_offset;
	ret = read_ready(addr);
    if (ret < 0) {
        LOG_INF("ready failed");
        return -1;
    }

    return map_addr;
}

static int read_mt510_otp_group_flag()
{
	u32 read_addr = 0;
    read_addr = mt510_read(MT510_OTP_GROUP_FLAGADDR);
    GroupFlag = read_cmos_sensor(read_addr);
    mt510_otp_close();

    return 0;
}

static int read_mt510_module_data()
{
	int idx;
    u16 sn_start = 0;
    u16 sum = 0;
    u16 check_number = 0;
    u16 checksumAddr = 0;
    u32 read_addr = 0;
    u16 moduleAddr = 0;
    u16 sn_length = 23;
    u16 module_length = 17;
    BYTE temp_buff[32];

    read_mt510_otp_group_flag();

    if (GroupFlag == MT510_GROUP1_FLAG) {
        sn_start = MT510_OTP_SN_GROUP1_STARTADDR;
        moduleAddr = MT510_OTP_MODULE_GROUP1_STARTADDR;
        checksumAddr = MT510_OTP_SN_GROUP1_CHECK;
    } else if(GroupFlag == MT510_GROUP2_FLAG) {
        sn_start = MT510_OTP_SN_GROUP2_STARTADDR;
        moduleAddr = MT510_OTP_MODULE_GROUP2_STARTADDR;
        checksumAddr = MT510_OTP_SN_GROUP2_CHECK;
    } else {
        LOG_INF("Invalid GroupFlag: %d, skip module data reading", GroupFlag);
        return -1;
    }
    LOG_INF("sn_start = %d,GroupFlag = %d",sn_start,GroupFlag);

	memset(mt510_mumbai_common_data, 0, sizeof(mt510_mumbai_common_data));
    memset(temp_buff, 0, sizeof(temp_buff));

    read_addr = mt510_read(sn_start - 2);

    for(idx = 0; idx < sn_length+2; idx++)
    {
        temp_buff[idx] = read_cmos_sensor(read_addr++);
    }

    for(idx = 0; idx < sn_length; idx++)
    {
        mt510_mumbai_common_data[8+idx] = temp_buff[1+idx];
        sum += mt510_mumbai_common_data[8 + idx];
    }
    mt510_otp_close();

    read_addr = mt510_read(checksumAddr);
    check_number = read_cmos_sensor(checksumAddr);

    if ( check_number == sum%255) {
            LOG_INF("data check success sum :%d\n,checksumAddr:%x,check_number:%d", sum, checksumAddr,check_number);
        } else {
            LOG_INF("data check success sum :%d\n,checksumAddr:%x,check_number:%d", sum, checksumAddr,check_number);
        }

    memset(temp_buff, 0, sizeof(temp_buff));
    read_addr = mt510_read(moduleAddr-1);
    for(idx = 0; idx < module_length; idx++)
    {
        temp_buff[idx] = read_cmos_sensor(read_addr++);
    }

    // Lens ID
    mt510_mumbai_common_data[44] = temp_buff[8];
    mt510_mumbai_common_data[45] = temp_buff[9];
    // Vcm ID
    mt510_mumbai_common_data[40] = temp_buff[10];
    mt510_mumbai_common_data[41] = temp_buff[11];

    mt510_otp_close();
    for (idx = 8; idx < sn_length; idx = idx + 4) {
		pr_debug("mt510 common data sn is : %02x %02x %02x %02x\n",
		mt510_mumbai_common_data[idx],
		mt510_mumbai_common_data[idx + 1],
	    mt510_mumbai_common_data[idx + 2],
		mt510_mumbai_common_data[idx + 3]);
	}
    return 0;
}
/*************************************************************************
* FUNCTION
*       get_imgsensor_id
*
* DESCRIPTION
*       This function get the sensor ID
*
* PARAMETERS
*       *sensorID : return the sensor ID
*
* RETURNS
*       None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    LOG_INF("mt510 get_imgsensor_id imgsensor_info.sensor_id=0x%x\n", imgsensor_info.sensor_id);
    while(imgsensor_info.i2c_addr_table[i] != 0xff)
    {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do
        {
            msleep(10);
            *sensor_id = (((read_cmos_sensor(0x0000) << 8) | read_cmos_sensor(0x0001)) + 1);
            printk("mt510 Read sensor id start, write id:0x%x ,sensor Id:0x%x/0x%x\n",
				imgsensor.i2c_write_id, *sensor_id, imgsensor_info.sensor_id);
            if (*sensor_id == imgsensor_info.sensor_id)
            {
                LOG_INF("mt510 i2c config id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, *sensor_id);
                 if (read_mt510_module_data() == 0) {
                    LOG_INF("read_mt510_module_data success\n");
                } else {
                    LOG_INF("read_mt510_module_data failed\n");
                }
                return ERROR_NONE;
            }
            LOG_INF("mt510 Read sensor id fail, config id:0x%x id: 0x%x\n", imgsensor.i2c_write_id, *sensor_id);
            retry--;
        } while(retry > 0);
        i++;
        retry = 3;
    }
    if (*sensor_id != imgsensor_info.sensor_id) {
        *sensor_id = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*       open
*
* DESCRIPTION
*       This function initialize the registers of CMOS sensor
*
* PARAMETERS
*       None
*
* RETURNS
*       None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    kal_uint16 sensor_id = 0;
#ifdef MIPI_1LANE
    LOG_INF("PLATFORM:MT6877, MIPI 1LANE\n");
#else
    LOG_INF("PLATFORM:MT6877, MIPI 2LANE\n");
#endif

    while (imgsensor_info.i2c_addr_table[i] != 0xff)
    {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do
        {
            msleep(10);
            sensor_id = (((read_cmos_sensor(0x0000) << 8) | read_cmos_sensor(0x0001)) + 1);
            if (sensor_id == imgsensor_info.sensor_id)
            {
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, sensor_id);
                break;
            }
            LOG_INF("Read sensor id fail, write id:0x%x id: 0x%x\n", imgsensor.i2c_write_id, sensor_id);
            retry--;
        } while(retry > 0);
        i++;
        if (sensor_id == imgsensor_info.sensor_id)
            break;
        retry = 3;
    }
    if (imgsensor_info.sensor_id != sensor_id)
            return ERROR_SENSOR_CONNECT_FAIL;

    /* initail sequence write in  */
    sensor_init();
    spin_lock(&imgsensor_drv_lock);
    imgsensor.autoflicker_en= KAL_FALSE;
    imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
    imgsensor.shutter = 0x3D0;
    imgsensor.gain = 0x100;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.dummy_pixel = 0;
    imgsensor.dummy_line = 0;
    imgsensor.ihdr_mode = 0;
    imgsensor.test_pattern = KAL_FALSE;
    imgsensor.current_fps = imgsensor_info.pre.max_framerate;
    spin_unlock(&imgsensor_drv_lock);

    return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*       close
*
* DESCRIPTION
*
*
* PARAMETERS
*       None
*
* RETURNS
*       None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
    LOG_INF("E\n");

    return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*       This function start the sensor preview.
*
* PARAMETERS
*       *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*       None
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
    // imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();

    return ERROR_NONE;
}       /*      preview   */

/*************************************************************************
* FUNCTION
*       capture
*
* DESCRIPTION
*       This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*       None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
    if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate)
    {
        imgsensor.pclk = imgsensor_info.cap1.pclk;
        imgsensor.line_length = imgsensor_info.cap1.linelength;
        imgsensor.frame_length = imgsensor_info.cap1.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    }
    else if(imgsensor.current_fps == imgsensor_info.cap2.max_framerate)
    {
        imgsensor.pclk = imgsensor_info.cap2.pclk;
        imgsensor.line_length = imgsensor_info.cap2.linelength;
        imgsensor.frame_length = imgsensor_info.cap2.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap2.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    }
    else
    {
        imgsensor.pclk = imgsensor_info.cap.pclk;
        imgsensor.line_length = imgsensor_info.cap.linelength;
        imgsensor.frame_length = imgsensor_info.cap.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    }
    spin_unlock(&imgsensor_drv_lock);

    capture_setting(imgsensor.current_fps);

    return ERROR_NONE;
}

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
    imgsensor.pclk = imgsensor_info.normal_video.pclk;
    imgsensor.line_length = imgsensor_info.normal_video.linelength;
    imgsensor.frame_length = imgsensor_info.normal_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
    // imgsensor.current_fps = 300;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);

    normal_video_setting(imgsensor.current_fps);

    return ERROR_NONE;
}

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
    imgsensor.pclk = imgsensor_info.hs_video.pclk;
    // imgsensor.video_mode = KAL_TRUE;
    imgsensor.line_length = imgsensor_info.hs_video.linelength;
    imgsensor.frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    // imgsensor.current_fps = 300;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    hs_video_setting();

    return ERROR_NONE;
}

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
    imgsensor.pclk = imgsensor_info.slim_video.pclk;
    // imgsensor.video_mode = KAL_TRUE;
    imgsensor.line_length = imgsensor_info.slim_video.linelength;
    imgsensor.frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    // imgsensor.current_fps = 300;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    slim_video_setting();

    return ERROR_NONE;
}

static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
    LOG_INF("E\n");
    sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
    sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

    sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
    sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

    sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
    sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


    sensor_resolution->SensorHighSpeedVideoWidth     = imgsensor_info.hs_video.grabwindow_width;
    sensor_resolution->SensorHighSpeedVideoHeight    = imgsensor_info.hs_video.grabwindow_height;

    sensor_resolution->SensorSlimVideoWidth  = imgsensor_info.slim_video.grabwindow_width;
    sensor_resolution->SensorSlimVideoHeight     = imgsensor_info.slim_video.grabwindow_height;
    return ERROR_NONE;
}

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
                              MSDK_SENSOR_INFO_STRUCT *sensor_info,
                              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("scenario_id = %d\n", scenario_id);

    sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
    sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
    sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorInterruptDelayLines = 4; /* not use */
    sensor_info->SensorResetActiveHigh = FALSE; /* not use */
    sensor_info->SensorResetDelayCount = 5; /* not use */

    sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
    // sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
    // sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
    sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

    sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
    sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
    sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
    sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
    sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

    sensor_info->SensorMasterClockSwitch = 0; /* not use */
    sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

    sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;          /* The frame of setting shutter default 0 for TG int */
    sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;    /* The frame of setting sensor gain */
    sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
    sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
    sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
    sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

    sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
    sensor_info->SensorClockFreq = imgsensor_info.mclk;
    sensor_info->SensorClockDividCount = 3; /* not use */
    sensor_info->SensorClockRisingCount = 0;
    sensor_info->SensorClockFallingCount = 2; /* not use */
    sensor_info->SensorPixelClockCount = 3; /* not use */
    sensor_info->SensorDataLatchCount = 2; /* not use */

    sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
    sensor_info->SensorHightSampling = 0;   // 0 is default 1x
    sensor_info->SensorPacketECCOrder = 1;

    switch (scenario_id)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

            sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

            break;
        default:
            sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
            break;
    }

    return ERROR_NONE;
}


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("control scenario_id = %d\n", scenario_id);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.current_scenario_id = scenario_id;
    spin_unlock(&imgsensor_drv_lock);
    switch (scenario_id)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
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
        default:
            LOG_INF("Error ScenarioId setting");
            preview(image_window, sensor_config_data);
            return ERROR_INVALID_SCENARIO_ID;
    }
    return ERROR_NONE;
}

static kal_uint32 set_video_mode(UINT16 framerate)
{
    LOG_INF("framerate = %d\n ", framerate);
    if (framerate == 0)
        return ERROR_NONE;
    spin_lock(&imgsensor_drv_lock);
    if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
        imgsensor.current_fps = 296;
    else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
        imgsensor.current_fps = 146;
    else
        imgsensor.current_fps = framerate;
    spin_unlock(&imgsensor_drv_lock);
    set_max_framerate(imgsensor.current_fps,1);

    return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
    LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
    spin_lock(&imgsensor_drv_lock);
    if (enable)
        imgsensor.autoflicker_en = KAL_TRUE;
    else
        // Cancel Auto flick
        imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
    kal_uint32 frame_length;

    LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

    switch (scenario_id)
    {
       case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            // set_dummy();
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            if(framerate == 0)
                return ERROR_NONE;
            frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            // set_dummy();
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate)
            {
                frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
                spin_lock(&imgsensor_drv_lock);
                imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
                imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
                imgsensor.min_frame_length = imgsensor.frame_length;
                spin_unlock(&imgsensor_drv_lock);
            }
            else if (imgsensor.current_fps == imgsensor_info.cap2.max_framerate)
            {
                frame_length = imgsensor_info.cap2.pclk / framerate * 10 / imgsensor_info.cap2.linelength;
                spin_lock(&imgsensor_drv_lock);
                imgsensor.dummy_line = (frame_length > imgsensor_info.cap2.framelength) ? (frame_length - imgsensor_info.cap2.framelength) : 0;
                imgsensor.frame_length = imgsensor_info.cap2.framelength + imgsensor.dummy_line;
                imgsensor.min_frame_length = imgsensor.frame_length;
                spin_unlock(&imgsensor_drv_lock);
            }
            else
            {
                if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
                {
                    LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
                        framerate,imgsensor_info.cap.max_framerate/10);
                }
                frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
                spin_lock(&imgsensor_drv_lock);
                imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
                imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
                imgsensor.min_frame_length = imgsensor.frame_length;
                spin_unlock(&imgsensor_drv_lock);
            }
            // set_dummy();
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            // set_dummy();
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;
            imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            // set_dummy();
            break;
        default:  // coding with  preview scenario by default
            frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            // set_dummy();
            LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
            break;
    }
    return ERROR_NONE;
}

static kal_uint32 get_default_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
    LOG_INF("scenario_id = %d\n", scenario_id);

    switch (scenario_id)
    {
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
        default:
            break;
    }

    return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
    LOG_INF("enable: %d\n", enable);

    if (enable)
    {
        //color bar enable
        write_cmos_sensor(0x0804, 0x02);
        write_cmos_sensor(0x09ec, 0x00);
        write_cmos_sensor(0x09ed, 0x20);
        write_cmos_sensor(0x09ee, 0x0a);
        write_cmos_sensor(0x09ef, 0x98);
        write_cmos_sensor(0x09f0, 0x07);
        write_cmos_sensor(0x09f1, 0xf6);
        write_cmos_sensor(0x09f2, 0x00);
        write_cmos_sensor(0x09f3, 0xcc);
        write_cmos_sensor(0x09f4, 0xe9);
        write_cmos_sensor(0x09f5, 0x01);
        write_cmos_sensor(0x09f6, 0x02);
        write_cmos_sensor(0x09f7, 0x00);
        write_cmos_sensor(0x09f8, 0x00);
        write_cmos_sensor(0x09f9, 0x02);
        write_cmos_sensor(0x09fa, 0x00);
        write_cmos_sensor(0x09fb, 0x02);
        write_cmos_sensor(0x09fc, 0x00);
        write_cmos_sensor(0x09fd, 0x02);
        write_cmos_sensor(0x09fe, 0x00);
        write_cmos_sensor(0x09ff, 0x02);
        write_cmos_sensor(0x0a00, 0x44);
        write_cmos_sensor(0x0a01, 0x01);
        write_cmos_sensor(0x0a03, 0x01);
        write_cmos_sensor(0x0604, 0x03);
        write_cmos_sensor(0x0606, 0x00);
        write_cmos_sensor(0x0800, 0x01);
        write_cmos_sensor(0x060c, 0x20);
        write_cmos_sensor(0x060d, 0x0a);
        write_cmos_sensor(0x060e, 0x98);
        write_cmos_sensor(0x060f, 0x07);
        write_cmos_sensor(0x08da, 0x01);
        write_cmos_sensor(0x0800, 0x01);
        write_cmos_sensor(0x09ec, 0x01);
    }
    else
    {
        write_cmos_sensor(0x09ec, 0x00);
        write_cmos_sensor(0x0804, 0x01);
        write_cmos_sensor(0x0606, 0x01);
    }
    spin_lock(&imgsensor_drv_lock);
    imgsensor.test_pattern = enable;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}

static kal_uint32 streaming_control(kal_bool enable)
{
    kal_uint8 mipi_status;
    kal_uint8 cnt = 105;
    pr_info("streaming_enable(0=Sw tandby,1=streaming): %d\n", enable);
    if (enable)
    {
        write_cmos_sensor(0x062d, 0x17);
        msleep(1);
        write_cmos_sensor(0x0500, 0x01);
    } else {
        write_cmos_sensor(0x0500, 0x00);
        do {
            mipi_status = read_cmos_sensor(0x0624);
            if ((mipi_status & 0x01) == 0x01) {
                pr_info("mipi_status: 0x%02x, cnt = %d\n", mipi_status, cnt);
                break;
            } else {
                msleep(1);
                cnt--;
            }
        } while(cnt);
        write_cmos_sensor(0x062d, 0x00);
    }
    return ERROR_NONE;
}

static void set_shutter_frame_length(
                    kal_uint16 shutter, kal_uint16 frame_length,
                    kal_bool auto_extend_en)
{
    unsigned long flags;
    kal_uint16 realtime_fps = 0;
    kal_int32 dummy_line = 0;

    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

    spin_lock(&imgsensor_drv_lock);
    /* Change frame time */
    dummy_line = frame_length - imgsensor.frame_length;
    imgsensor.frame_length = imgsensor.frame_length + dummy_line;
    imgsensor.min_frame_length = imgsensor.frame_length;

    if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
        imgsensor.frame_length = shutter + imgsensor_info.margin;
    else
        imgsensor.frame_length = imgsensor.min_frame_length;

    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
        imgsensor.frame_length = imgsensor_info.max_frame_length;
    spin_unlock(&imgsensor_drv_lock);

    shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
    shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
              ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

    if(imgsensor.autoflicker_en)
    {
        realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
        if(realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(296, 0);
        else if(realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(146, 0);
        else
            LOG_INF("autoflicker enable\n");
    }

     // Update frame length and shutter of normal mode
     update_shutter_frame_reg(0, 0, shutter, imgsensor.frame_length);

    LOG_INF("Exit! shutter =%d, imgsensor.framelength =%d, frame_length=%d, dummy_line=%d\n",
            shutter, imgsensor.frame_length, frame_length, dummy_line);
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
                                     UINT8 *feature_para,UINT32 *feature_para_len)
{
    UINT16 *feature_return_para_16=(UINT16 *) feature_para;
    UINT16 *feature_data_16=(UINT16 *) feature_para;
    UINT32 *feature_return_para_32=(UINT32 *) feature_para;
    UINT32 *feature_data_32=(UINT32 *) feature_para;
    unsigned long long *feature_data=(unsigned long long *) feature_para;

    struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;
#ifdef MT_SENSOR_CUSTOMIZED
    struct IMGSENSOR_LOAD_FROM_FILE *load_val;
#endif
    LOG_INF("mt510  feature_control feature_id = %d\n", feature_id);
    switch (feature_id)
    {
        case SENSOR_FEATURE_GET_PERIOD:
            *feature_return_para_16++ = imgsensor.line_length;
            *feature_return_para_16 = imgsensor.frame_length;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            LOG_INF("feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n", imgsensor.pclk,imgsensor.current_fps);
            *feature_return_para_32 = imgsensor.pclk;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            set_shutter(*feature_data);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            night_mode((kal_bool) *feature_data);
            break;
        case SENSOR_FEATURE_SET_GAIN:
            set_gain((UINT16) *feature_data);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            set_video_mode(*feature_data);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            get_imgsensor_id(feature_return_para_32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            set_auto_flicker_mode((kal_bool)*feature_data_16,*(feature_data_16+1));
            break;
        case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            set_max_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
            break;
        case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            get_default_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
            break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((kal_bool)*feature_data);
            break;
        case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
            // for factory mode auto testing
            *feature_return_para_32 = imgsensor_info.checksum_value;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_FRAMERATE:
            LOG_INF("current fps :%d\n", (UINT32)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_SET_HDR:
            LOG_INF("ihdr enable :%d\n", (kal_bool)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            //imgsensor.ihdr_mode = *feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_GET_CROP_INFO:
            LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);
            wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
            switch(*feature_data_32)
            {
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_SLIM_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                default:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
            }
            break;
        case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
            LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            // ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            break;
        case SENSOR_FEATURE_GET_VC_INFO:
            LOG_INF("SENSOR_FEATURE_GET_VC_INFO not implement%d\n", (UINT16)*feature_data);
            break;
        case SENSOR_FEATURE_SET_AWB_GAIN:
            break;
        case SENSOR_FEATURE_SET_HDR_SHUTTER:
            LOG_INF("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1));
            // ihdr_write_shutter((UINT16)*feature_data,(UINT16)*(feature_data+1));
            break;
        case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
            break;
        case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
            set_shutter_frame_length((UINT16) (*feature_data),
                        (UINT16) (*(feature_data + 1)),
                        (BOOL) (*(feature_data + 2)));
            break;
        case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
            pr_debug("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
            streaming_control(KAL_FALSE);
            break;
        case SENSOR_FEATURE_SET_STREAMING_RESUME:
            pr_info("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n",
                    *feature_data);
            if (*feature_data != 0)
                set_shutter(*feature_data);
            streaming_control(KAL_TRUE);
            break;
        case SENSOR_FEATURE_GET_BINNING_TYPE:
            switch (*(feature_data + 1)) {
            case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            case MSDK_SCENARIO_ID_SLIM_VIDEO:
                *feature_return_para_32 = 1; /*BINNING_NONE*/
                break;
            default:
                *feature_return_para_32 = 2; /*BINNING_AVERAGED*/
                break;
            }
            pr_debug("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
                *feature_return_para_32);
            *feature_para_len = 4;
            break;
#ifdef MT510_LONG_EXP
        case SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE:
            *feature_return_para_32 = imgsensor.current_ae_effective_frame;
            break;
        case SENSOR_FEATURE_GET_AE_FRAME_MODE_FOR_LE:
            memcpy(feature_return_para_32,
            &imgsensor.ae_frm_mode, sizeof(struct IMGSENSOR_AE_FRM_MODE));
            break;
#endif
        case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
        {
            switch(*feature_data)
            {
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                    *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.cap.mipi_pixel_rate;
                    break;
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                    *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.normal_video.mipi_pixel_rate;
                    break;
                case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                    *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.hs_video.mipi_pixel_rate;
                    break;
                case MSDK_SCENARIO_ID_SLIM_VIDEO:
                    *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
                        imgsensor_info.slim_video.mipi_pixel_rate;
                    break;
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                default:
                    *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.pre.mipi_pixel_rate;
                    break;
            }
            break;
        }
        case SENSOR_FEATURE_GET_EEPROM_COMDATA:
            memcpy(feature_return_para_32, mt510_mumbai_common_data, CAMERA_EEPPROM_COMDATA_LENGTH);
            *feature_para_len = CAMERA_EEPPROM_COMDATA_LENGTH;
            break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO:
            switch(*feature_data)
            {
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
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                default:
                    *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.pre.pclk;
                    break;
            }
            break;
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
            break;
        case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
            switch (*feature_data)
            {
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
                default:
                    *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                    = (imgsensor_info.pre.framelength << 16)
                        + imgsensor_info.pre.linelength;
                    break;
            }
            break;
        case SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 3000000;
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
#ifdef MT_SENSOR_CUSTOMIZED
        case SENSOR_FEATURE_LOAD_PARA_FROM_FILE:
            load_val = (struct IMGSENSOR_LOAD_FROM_FILE *)feature_para;
            if(load_val && load_val->enable)
            {
                LOG_INF("SENSOR_FEATURE_LOAD_PARA_FROM_FILE\n");
                gp_load_file = load_val;
            }
            else
            {
                LOG_INF("clear SENSOR_FEATURE_LOAD_PARA_FROM_FILE\n");
                gp_load_file = NULL;
            }
            break;
#endif
        default:
            break;
    }

    return ERROR_NONE;
}/*      feature_control()  */

static struct  SENSOR_FUNCTION_STRUCT sensor_func = {
    open,
    get_info,
    get_resolution,
    feature_control,
    control,
    close
};

UINT32 MT510_MIPI_RAW_MUMBAID_SensorInit(struct SENSOR_FUNCTION_STRUCT  **pfFunc)
{
    LOG_INF("MT510_MIPI_RAW_SensorInit\n");
    if (pfFunc!=NULL)
            *pfFunc=&sensor_func;
    return ERROR_NONE;
}

/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

/*****************************************************************************
 *
 * Filename:
 * ---------
 *     IMX852mipi_Sensor.c
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "imx852mipiraw_mumbai_Sensor.h"
#include "imx852_eeprom.h"
#include "imx852_seamless_switch.h"

#ifdef MAX_VC_INFO_CNT
#undef MAX_VC_INFO_CNT
#endif
#define PD_PIX_2_EN 0
#define MAX_VC_INFO_CNT 13
#define SENSOR_GET_LINELENGTH_FOR_READOUT (0x1 << 0)
#define LEFT_SIZE (0x4000)
#define READ_EEPROM_1K (1024)
#define EEPROM_I2C_ADDR 0xA0
#define BASEGAIN 0x40
/***************Modify Following Strings for Debug**********************/
#define PFX "imx852_camera_sensor"
#define LOG_1 LOG_INF("IMX852,MIPI 4LANE\n")
/****************************Modify end**************************/
// extern enum IMGSENSOR_RETURN Eeprom_DataInit(
//             enum IMGSENSOR_SENSOR_IDX sensor_idx,
//             kal_uint32 sensorID);
extern struct CAMERA_DEVICE_INFO gImgEepromInfo;
//static kal_uint8 deviceInfo_register_value = 0x00;
static kal_uint8 otp_data[LEFT_SIZE] = {0};

#define LOG_INF(format, args...) pr_info(PFX "[%s] " format, __func__, ##args)

static kal_uint32 streaming_control(kal_bool enable);
#define MODULE_ID_OFFSET 0x0000
static kal_uint16 imx852_table_write_cmos_sensor(kal_uint16 *para, kal_uint32 len);
static kal_uint16 imx852_burst_write_cmos_sensor(kal_uint16 *para, kal_uint32 len);
static DEFINE_SPINLOCK(imgsensor_drv_lock);

static kal_uint16 imx852_SPC_setting[SPC_SIZE*2];
static kal_uint16 imx852_QSC_setting[QSC_SIZE*2];
static kal_uint8 qsc_flag = 0, spc_flag = 0;
static BYTE imx852_common_data[CAMERA_EEPPROM_COMDATA_LENGTH] = { 0 };
static uint8_t deviceInfo_register_value = 0;
static kal_uint8 imx852_hw_version = 0;

static struct imgsensor_info_struct imgsensor_info = {
    .sensor_id = IMX852_SENSOR_ID_MUMBAI,

    .checksum_value = 0xe5ffccac,
    .pre = { //reg_B1-S1  QBIN(VBIN)_4096x3072_30FPS with PDAF VB_max
        .pclk = 732000000,
        .linelength = 6396,
        .framelength = 3814,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4096,
        .grabwindow_height = 3072,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 759600000,  //1899 Msps/lane *4/10 = 759.6
        .max_framerate = 300,
    },

    .cap = { //reg_B1-S1  QBIN(VBIN)_4096x3072_30FPS with PDAF VB_max
        .pclk = 732000000,
        .linelength = 6396,
        .framelength = 3814,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4096,
        .grabwindow_height = 3072,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 759600000,  //1899 Msps/lane *4/10 = 759.6
        .max_framerate = 300,
    },

    .normal_video = { //reg_B3-S1  QBIN(VBIN)_4096x2304 @30FPS with PDAF VB_max
        .pclk = 732000000,
        .linelength = 6396,
        .framelength = 3814,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4096,
        .grabwindow_height = 2304,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 759600000,//1899 Msps/lane *4/10 = 759.6
        .max_framerate = 300,
    },

    .hs_video = { //reg_C-4 QBIN(VBIN)-V2H2 2048x1152_120FPS with PDAF VB_max
        .pclk = 732000000,
        .linelength = 2276,
        .framelength = 2680,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1280,
        .grabwindow_height = 720,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 759600000,
        .max_framerate = 1200, /* 120fps */
    },

    .slim_video = { //reg_A  QBIN(VBIN)_4096x3072_30FPS with PDAF VB_max
        .pclk = 732000000,
        .linelength = 6396,
        .framelength = 3814,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4096,
        .grabwindow_height = 2304,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 759600000,//1899 Msps/lane *4/10 = 759.6
        .max_framerate = 300,
    },

    .custom1 = { //reg_B4 4096x3072 @24FPS QBIN(VBIN) RST= 31.4ms
        .pclk = 732000000,
        .linelength = 6396,
        .framelength = 3814,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4096,
        .grabwindow_height = 3072,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 759600000,  //1899 Msps/lane *4/10 = 759.6
        .max_framerate = 300,
    },

    .custom2 = { //reg_B-1 QBIN_4096x2304 @60FPS with PDAF VB_max
        .pclk = 732000000,
        .linelength = 6396,
        .framelength = 4768,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4096,
        .grabwindow_height = 3072,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 759600000,  //1899 Msps/lane *4/10 = 759.6
        .max_framerate = 240,
    },

    .custom3 = {  //reg_D Full RMSC 15fps with PDAF VB_max
        .pclk = 732000000,
        .linelength = 4152,
        .framelength = 1958,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2048,
        .grabwindow_height = 1536,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 541600000, ////1624.00 Msps/Trio *3*2.28/10 = 1110.816
        .max_framerate = 900,
        },

    .custom4 = { //reg_C-5 QBIN-V2H2 2048x1152_240FPS w/o PDAF VB_max
        .pclk = 732000000,
        .linelength = 6396,
        .framelength = 3814,// 7804/2
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 3264,
        .grabwindow_height = 2448,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 759600000,//1428.00 Msps/Trio *3*2.28/10
        .max_framerate = 300,
    },

    .custom5 = {//reg_A-3-S1 Full-RMSC-Crop_4096x3072_30FPS with PDAF VB_max seamless A-2-S1
        .pclk = 878400000,
        .linelength = 8960,
        .framelength = 3252,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4096,
        .grabwindow_height = 3072,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 699048000, //1022 Msps/Trio *3*2.28/10 = 699.048
        .max_framerate = 300, /* 30fps */
    },

    .custom6 = {// reg_C-6 QBIN-V2H2 1280x674_480FPS w/o PDAF VB_max
        .pclk = 878400000,
        .linelength = 2468,
        .framelength = 734,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1280,
        .grabwindow_height = 674,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 692208000, //1012 Msps/Trio *3*2.28/10
        .max_framerate = 4800, //480fps
    },

    .custom7 = {
        .pclk = 878400000,
        .linelength = 7500,
        .framelength = 3900,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4096,
        .grabwindow_height = 3072,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 699048000,  //1022.00 Msps/Trio *3*2.28/10 = 699.048
        .max_framerate = 300,
    },

    .custom8 = {
        .pclk = 2246400000,
        .linelength = 15616,
        .framelength = 4792,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4096,
        .grabwindow_height = 2304,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 858510000,
        .max_framerate = 300,
    },

    .custom9 = {  //reg_V2 2048x1536 @24FPS QBIN(VBIN)-V2H2 RST= 33.3ms (Tline Max.)
        .pclk = 210000000,
        .linelength = 4024,
        .framelength = 2168,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2048,
        .grabwindow_height = 1536,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 342820000,
        .max_framerate = 240,
    },

    .custom10 = {
        .pclk = 2246400000,
        .linelength = 15616,
        .framelength = 4792,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4096,
        .grabwindow_height = 2304,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 858510000,
        .max_framerate = 300,
    },


    .min_gain = BASEGAIN * 1, //1.429
    .max_gain = BASEGAIN * 32,
    .min_gain_iso = 100,
    .margin = 48,
    .min_shutter = 16,
    .gain_step = 1,
    .gain_type = 0,
    .max_frame_length = 0xffff,
    .ae_shut_delay_frame = 0,
    .ae_sensor_gain_delay_frame = 0,
    .ae_ispGain_delay_frame = 2,    /* isp gain delay frame for AE cycle */
    .ihdr_support = 0,    /* 1: support; 0: not support */
    .ihdr_le_firstline = 0,    /* 1:le first; 0: se first */
    .temperature_support = 1, /* 1, support; 0,not support */
    .sensor_mode_num = 15,    /* support sensor mode num */
    .frame_time_delay_frame = 3,
    .pre_delay_frame = 2,    /* enter preview delay frame num */
    .cap_delay_frame = 2,    /* enter capture delay frame num */
    .video_delay_frame = 2,    /* enter video delay frame num */
    .hs_video_delay_frame = 2,    /* enter hs video delay frame num */
    .slim_video_delay_frame = 2,/* enter slim video delay frame num */
    .custom1_delay_frame = 2,    /* enter custom1 delay frame num */
    .custom2_delay_frame = 2,    /* enter custom2 delay frame num */
    .custom3_delay_frame = 2,    /* enter custom3 delay frame num */
    .custom4_delay_frame = 2,    /* enter custom4 delay frame num */
    .custom5_delay_frame = 2,    /* enter custom5 delay frame num */
    .custom6_delay_frame = 2,    /* enter custom6 delay frame num */
    .custom7_delay_frame = 2,    /* enter custom7 delay frame num */
    .custom8_delay_frame = 2,    /* enter custom8 delay frame num */
    .custom9_delay_frame = 2,    /* enter custom9 delay frame num */
    .custom10_delay_frame = 2,    /* enter custom10 delay frame num */

    .isp_driving_current = ISP_DRIVING_6MA,
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
    .mipi_sensor_type = MIPI_OPHY_NCSI2,
    .mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_B,
    .mclk = 24, /* suggest 24 or 26 for 24Mhz or 26Mhz */
    .mipi_lane_num = SENSOR_MIPI_4_LANE,
    .i2c_addr_table = {0x20, 0x52, 0xff},
    /* record sensor support all write id addr,
     * only supprt 4 must end with 0xff
     */
    .i2c_speed = 1000, /* kbps */
};

static struct imgsensor_struct imgsensor = {
    .mirror = IMAGE_HV_MIRROR,    /* mirror and flip on information */
    .sensor_mode = IMGSENSOR_MODE_INIT,
    /* IMGSENSOR_MODE enum value,record current sensor mode,such as:
     * INIT, Preview, Capture, Video,High Speed Video, Slim Video
     */
    .shutter = 0x3D0,    /* current shutter */
    .gain = 0x100,        /* current gain */
    .dummy_pixel = 0,    /* current dummypixel */
    .dummy_line = 0,    /* current dummyline */
    .current_fps = 450,
    .autoflicker_en = KAL_FALSE,
    .test_pattern = KAL_FALSE,
    .current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
    .ihdr_mode = 0, /* sensor need support LE, SE with HDR feature */
    .i2c_write_id = 0x20, /* record current sensor's i2c write id */
    .extend_frame_length_en = KAL_FALSE,
    .fast_mode_on = KAL_FALSE,
};

static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[] = {
    // Preview
    {8192, 6144,    0,    0, 8192, 6144, 4096, 3072,    0,    0, 4096, 3072,    0,    0, 4096, 3072},
    // Capture
    {8192, 6144,    0,    0, 8192, 6144, 4096, 3072,    0,    0, 4096, 3072,    0,    0, 4096, 3072},
    // Video
    {8192, 6144,    0,  768, 8192, 4608, 4096, 2304,    0,    0, 4096, 2304,    0,    0, 4096, 2304},
    // hs_video
    {8192, 6144,  256,  912, 7680, 4320, 1280,  720,    0,    0, 1280,  720,    0,    0, 1280,  720},
    // slim_video
    {8192, 6144,    0,  768, 8192, 4608, 4096, 2304,    0,    0, 4096, 2304,    0,    0, 4096, 2304},
    // custom1 4096*3072@24FPS
    {8192, 6144,    0,    0, 8192, 6144, 4096, 3072,    0,    0, 4096, 3072,    0,    0, 4096, 3072},
    // custom2 4096*2304@60FPS
    {8192, 6144,    0,    0, 8192, 6144, 4096, 3072,    0,    0, 4096, 3072,    0,    0, 4096, 3072},
    // custom3 remosaic
    {8192, 6144,    0,    0, 8192, 6144, 2048, 1536,    0,    0, 2048, 1536,    0,    0, 2048, 1536},//
    // custom4 2048x1152_240FPS
    {8192, 6144,  832,  624, 6528, 4896, 3264, 2448,    0,    0, 3264, 2448,    0,    0, 3264, 2448},
    /* custom5 full_rmsc_corp 4096*3072@30fps*/
    {8192, 6144,    0, 1536, 8192, 3072, 8192, 3072, 2048,    0, 4096, 3072,    0,    0, 4096, 3072},
    /* custom6 1280x674_480FPS*/
    {8192, 6144,    0, 1600, 8192, 2944, 2048,  736,  384,   31, 1280,  674,    0,    0, 1280,  674},
    /* custom7 preISP Reg S*/
    {8192, 6144,    0,    0, 8192, 6144, 4096, 3072,    0,    0, 4096, 3072,    0,    0, 4096, 3072},
    /* custom8 preISP Reg K 2DOL*/
    {8192, 6144,    0,  768, 8192, 4608, 4096, 2304,    0,    0, 4096, 2304,    0,    0, 4096, 2304},
    /* custom9 2048x1536 @24fps*/
    {8192, 6144,    0,    0, 8192, 6144, 2048, 1536,    0,    0, 2048, 1536,    0,    0, 2048, 1536},
    //custom10
    {8192, 6144,    0,  768, 8192, 4608, 4096, 2304,    0,    0, 4096, 2304,    0,    0, 4096, 2304},
};

static struct SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[] = {
    {
        0x02, 0x0a, 0x0000, 0x0008, 0x40, 0x00,
        0x00, 0x2b, 0x1000, 0x0C00, 0x00, 0x00, 0x0000, 0x0000,//4096*3072
        0x01, 0x2b, 0x03F8, 0x05F0, 0x03, 0x00, 0x0000, 0x0000 //1016*1520
    },
    /* capture mode setting */
    {
        0x02, 0x0a, 0x0000, 0x0008, 0x40, 0x00,
        0x00, 0x2b, 0x1000, 0x0C00, 0x00, 0x00, 0x0000, 0x0000,//4096*3072
        0x01, 0x2b, 0x03F8, 0x05F0, 0x03, 0x00, 0x0000, 0x0000 //1016*1520
    },
    /* normal/slim video mode setting */
    {
        0x02, 0x0a, 0x0000, 0x0008, 0x40, 0x00,
        0x00, 0x2b, 0x1000, 0x0900, 0x00, 0x00, 0x0000, 0x0000, //4096*2304
        0x01, 0x2b, 0x03F8, 0x0480, 0x03, 0x00, 0x0000, 0x0000 //1016*1152
    },
    /* custom3 video 60fps mode setting */
    {
        0x02, 0x0a, 0x0000, 0x0008, 0x40, 0x00,
        0x00, 0x2b, 0x0800, 0x0600, 0x00, 0x00, 0x0000, 0x0000, //2048*1536
        0x01, 0x2b, 0x03F8, 0x017C, 0x03, 0x00, 0x0000, 0x0000 //1016*380
    },
    /* custom4 8M portrait mode setting */
    {
        0x02, 0x0a, 0x0000, 0x0008, 0x40, 0x00,
        0x00, 0x2b, 0x0CC0, 0x0990, 0x00, 0x00, 0x0000, 0x0000, //3264*2448
        0x01, 0x2b, 0x0330, 0x04C0, 0x03, 0x00, 0x0000, 0x0000  //816*1216
    },
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
    .i4OffsetX = 17,
    .i4OffsetY = 16,
    .i4PitchX = 8,
    .i4PitchY = 8,
    .i4PairNum = 4,
    .i4SubBlkW = 8,
    .i4SubBlkH = 2,
    .i4PosL = { {20, 17},
                {18, 19},
                {22, 21},
                {24, 23}, },
    .i4PosR = { {19, 17},
                {17, 19},
                {21, 21},
                {23, 23}, },
    .i4BlockNumX = 508,
    .i4BlockNumY = 380,
    .i4LeFirst = 0,
    .i4Crop = { {0, 0}, {0, 0}, {0, 384}, {0, 384}, {0, 384},
        {0, 0}, {0, 0} ,{0, 0}, {416, 312},
    },
    .iMirrorFlip = 3,
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_video = {
    .i4OffsetX = 17,
    .i4OffsetY = 16,
    .i4PitchX = 8,
    .i4PitchY = 8,
    .i4PairNum = 4,
    .i4SubBlkW = 8,
    .i4SubBlkH = 2,
    .i4PosL = { {20, 17},
                {18, 19},
                {22, 21},
                {24, 23}, },
    .i4PosR = { {19, 17},
                {17, 19},
                {21, 21},
                {23, 23}, },
    .i4BlockNumX = 508,
    .i4BlockNumY = 288,
    .i4LeFirst = 0,
    .i4Crop = { {0, 0}, {0, 0}, {0, 384}, {0, 384}, {0, 384},
        {0, 0}, {0, 0} ,{0, 0}, {416, 312},
    },
    .iMirrorFlip = 3,
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_custom3 = {
    .i4OffsetX = 17,
    .i4OffsetY = 16,
    .i4PitchX = 8,
    .i4PitchY = 8,
    .i4PairNum = 4,
    .i4SubBlkW = 8,
    .i4SubBlkH = 2,
    .i4PosL = { {20, 17},
                {18, 19},
                {22, 21},
                {24, 23}, },
    .i4PosR = { {19, 17},
                {17, 19},
                {21, 21},
                {23, 23}, },
    .i4BlockNumX = 508,
    .i4BlockNumY = 95,
    .i4LeFirst = 0,
    .i4Crop = { {0, 0}, {0, 0}, {0, 384}, {0, 384}, {0, 384},
        {0, 0}, {0, 0} ,{0, 0}, {416, 312},
    },
    .iMirrorFlip = 3,
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_custom4 = {
    .i4OffsetX = 17,
    .i4OffsetY = 16,
    .i4PitchX = 8,
    .i4PitchY = 8,
    .i4PairNum = 4,
    .i4SubBlkW = 8,
    .i4SubBlkH = 2,
    .i4PosL = { {20, 17},
                {18, 19},
                {22, 21},
                {24, 23}, },
    .i4PosR = { {19, 17},
                {17, 19},
                {21, 21},
                {23, 23}, },
    .i4BlockNumX = 408,
    .i4BlockNumY = 304,
    .i4LeFirst = 0,
    .i4Crop = { {0, 0}, {0, 0}, {0, 384}, {0, 384}, {0, 384},
        {0, 0}, {0, 0} ,{0, 0}, {416, 312},
    },
    .iMirrorFlip = 3,
};

static struct SEAMLESS_SYS_DELAY seamless_sys_delays[] = {
    { MSDK_SCENARIO_ID_VIDEO_PREVIEW, MSDK_SCENARIO_ID_CUSTOM5, 1 },
    { MSDK_SCENARIO_ID_CUSTOM5, MSDK_SCENARIO_ID_VIDEO_PREVIEW, 1 },
};

static bool oplus_is_system_camera(unsigned int val)
{
    static bool is_system_cam = true;
    if(val != 0xFFFF) {
        is_system_cam = (val != 0);
    }
    pr_debug("oplus_is_system_camera:in %d out %d\n", val, is_system_cam);

    return is_system_cam;
}

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte = 0;
    char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF)};

    iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 2, imgsensor.i2c_write_id);
    return ((get_byte<<8)&0xff00) | ((get_byte>>8)&0x00ff);
}

static kal_uint16 read_cmos_sensor_8(kal_uint16 addr)
{
    kal_uint16 get_byte = 0;
    char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

    iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);
    return get_byte;
}

static enum IMGSENSOR_RETURN write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
    char pusendcmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF),
            (char)(para & 0xFF)};

    return iWriteRegI2C(pusendcmd, 3, imgsensor.i2c_write_id);
}

static kal_uint32 get_cur_exp_cnt(void)
{
    kal_uint32 exp_cnt = 1;

    if (0x1 == (read_cmos_sensor_8(0x3170) & 0x1)) { // DOL_EN
        exp_cnt = 2;
    }

    return exp_cnt;
}

static void read_sensor_verison(void)
{
    kal_uint8 chip_value = 0;

    chip_value = read_cmos_sensor_8(chip_revision_reg);
    if (chip_value > 2){
        imx852_hw_version = HW_MP;
    } else if (chip_value == 2) {
        imx852_hw_version = HW_CUT1_0;
    } else if (chip_value == 1) {
        imx852_hw_version = HW_CUT0_91;
    } else {
        imx852_hw_version = HW_CUT0_90;
    }

    LOG_INF("%s chip_vale(0x%x) hw_version(%d) \n",
        __func__, chip_value, imx852_hw_version);
}

static enum IMGSENSOR_RETURN write_frame_len(kal_uint32 fll)
{
    // //write_frame_len should be called inside GRP_PARAM_HOLD (0x0104)
    // FRM_LENGTH_LINES must be multiple of 4
    enum IMGSENSOR_RETURN ret = IMGSENSOR_RETURN_SUCCESS;
    kal_uint32 exp_cnt = get_cur_exp_cnt();
    imgsensor.frame_length = round_up(fll / exp_cnt, 4) * exp_cnt;

    LOG_INF("extend_frame_length_en %d\n", imgsensor.extend_frame_length_en);
    if (imgsensor.extend_frame_length_en == KAL_FALSE) {
        LOG_INF("fll %d exp_cnt %d\n", imgsensor.frame_length, exp_cnt);
        ret += write_cmos_sensor_8(0x0340, imgsensor.frame_length / exp_cnt >> 8);
        ret += write_cmos_sensor_8(0x0341, imgsensor.frame_length / exp_cnt & 0xFF);
    }

    LOG_INF("fast_mode_on %d\n", imgsensor.fast_mode_on);
    if (imgsensor.fast_mode_on == KAL_TRUE) {
        imgsensor.fast_mode_on = KAL_FALSE;
        ret += write_cmos_sensor_8(0x3010, 0x00);
    }
    return ret;
}

static kal_uint16 Eeprom_1ByteDataRead(kal_uint16 addr, kal_uint16 slaveaddr)
{
    kal_uint16 get_byte=0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    iReadRegI2C(pusendcmd , 2, (u8*)&get_byte, 1, slaveaddr);
    return get_byte;
}

static void set_dummy(void)
{
    enum IMGSENSOR_RETURN ret = IMGSENSOR_RETURN_SUCCESS;
    kal_uint8 write_retry = 2;
    LOG_INF("dummyline = %d, dummypixels = %d, frame_length:%d, line_length:%d\n",
        imgsensor.dummy_line, imgsensor.dummy_pixel, imgsensor.frame_length, imgsensor.line_length);
    /* return;*/ /* for test */
    if (imgsensor.shutter > imgsensor.min_frame_length - imgsensor_info.margin)
        imgsensor.frame_length = imgsensor.shutter + imgsensor_info.margin;
    while(write_retry) {
        ret = write_frame_len(imgsensor.frame_length);
        if (ret == IMGSENSOR_RETURN_SUCCESS)
            break;
        write_retry--;
        mdelay(2);
    }
}    /*    set_dummy  */

static void set_mirror_flip(kal_uint8 image_mirror)
{
    kal_uint8 itemp;

    LOG_INF("image_mirror = %d\n", image_mirror);
    itemp = read_cmos_sensor_8(0x0101);
    itemp &= ~0x03;

    switch (image_mirror) {

    case IMAGE_NORMAL:
    write_cmos_sensor_8(0x0101, itemp);
    break;

    case IMAGE_V_MIRROR:
    write_cmos_sensor_8(0x0101, itemp | 0x02);
    break;

    case IMAGE_H_MIRROR:
    write_cmos_sensor_8(0x0101, itemp | 0x01);
    break;

    case IMAGE_HV_MIRROR:
    write_cmos_sensor_8(0x0101, itemp | 0x03);
    break;
    }
}

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
    /*kal_int16 dummy_line;*/
    kal_uint32 frame_length = imgsensor.frame_length;

    LOG_INF("framerate = %d, min framelength should enable %d\n", framerate,
        min_framelength_en);

    frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
    spin_lock(&imgsensor_drv_lock);
    if (frame_length >= imgsensor.min_frame_length)
        imgsensor.frame_length = frame_length;
    else
        imgsensor.frame_length = imgsensor.min_frame_length;

    imgsensor.dummy_line =
            imgsensor.frame_length - imgsensor.min_frame_length;

    if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
        imgsensor.frame_length = imgsensor_info.max_frame_length;
        imgsensor.dummy_line =
            imgsensor.frame_length - imgsensor.min_frame_length;
    }
    if (min_framelength_en)
        imgsensor.min_frame_length = imgsensor.frame_length;
    spin_unlock(&imgsensor_drv_lock);
    set_dummy();
}    /*    set_max_framerate  */

#define MAX_CIT_LSHIFT 9
static void write_shutter(kal_uint32 shutter, kal_bool gph)
{
    kal_uint16 realtime_fps = 0;
    kal_uint16 l_shift = 1;
    //shutter = round_up(shutter, 4);

    spin_lock(&imgsensor_drv_lock);
    if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
        imgsensor.frame_length = shutter + imgsensor_info.margin;
    else
        imgsensor.frame_length = imgsensor.min_frame_length;
    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
        imgsensor.frame_length = imgsensor_info.max_frame_length;
    spin_unlock(&imgsensor_drv_lock);
    if (shutter < imgsensor_info.min_shutter) {
        shutter = imgsensor_info.min_shutter;
    }

    if (gph) {
        write_cmos_sensor_8(0x0104, 0x01);
    }

    shutter = (shutter >> 1) << 1;
    imgsensor.frame_length = (imgsensor.frame_length >> 1) << 1;
    //frame_length and shutter should be an even number

    if (imgsensor.autoflicker_en) {
        realtime_fps = imgsensor.pclk / imgsensor.line_length * 10
                / imgsensor.frame_length;
        LOG_INF("autoflicker enable, realtime_fps = %d\n", realtime_fps);
        if (realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(296, 0);
        else if (realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(146, 0);
    }

    if (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) {
         for (l_shift = 1; l_shift < MAX_CIT_LSHIFT; l_shift++) {
             if ((shutter >> l_shift) < (imgsensor_info.max_frame_length - imgsensor_info.margin))
                 break;
         }
         if (l_shift > MAX_CIT_LSHIFT) {
             LOG_INF("Unable to set such a long exposure %d, set to max\n", shutter);
             l_shift = MAX_CIT_LSHIFT;
         }
         shutter = shutter >> l_shift;
         imgsensor.frame_length = shutter + imgsensor_info.margin;
         LOG_INF("enter long exposure mode, time is %d", l_shift);
         write_cmos_sensor_8(0x3100, read_cmos_sensor(0x3100) | (l_shift & 0x0f));
    } else {
         write_cmos_sensor_8(0x3100, read_cmos_sensor(0x3100) & 0xf0);
    }

    //write_frame_len(imgsensor.frame_length);

    /* Update Shutter */
    write_cmos_sensor_8(0x0350, 0x01); /* Enable auto extend */
    write_cmos_sensor_8(0x0202, (shutter >> 8) & 0xFF);
    write_cmos_sensor_8(0x0203, shutter  & 0xFF);
    if (gph) {
        write_cmos_sensor_8(0x0104, 0x00);
    }

    LOG_INF("shutter =%d, framelength =%d read 0x0202: 0x%x 0x0203: 0x%x\n",
        shutter, imgsensor.frame_length, read_cmos_sensor_8(0x0202), read_cmos_sensor_8(0x0203));
}    /*    write_shutter  */

/*************************************************************************
 * FUNCTION
 *    set_shutter
 *
 * DESCRIPTION
 *    This function set e-shutter of sensor to change exposure time.
 *
 * PARAMETERS
 *    iShutter : exposured lines
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void set_shutter_w_gph(kal_uint32 shutter, kal_bool gph)
{
    unsigned long flags;

    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

    write_shutter(shutter, gph);
}

static void set_shutter(kal_uint32 shutter)
{
    LOG_INF("Fine Integ Time = %d", (read_cmos_sensor_8(0x0200) << 8) | read_cmos_sensor_8(0x0201));
    set_shutter_w_gph(shutter, KAL_TRUE);
} /* set_shutter */

/*************************************************************************
 * FUNCTION
 *    set_shutter_frame_length
 *
 * DESCRIPTION
 *    for frame & 3A sync
 *
 *************************************************************************/
static void set_shutter_frame_length(kal_uint16 shutter, kal_uint16 frame_length,
                kal_bool auto_extend_en)
{
    unsigned long flags;
    kal_uint16 realtime_fps = 0;
    kal_int32 dummy_line = 0;

    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

    /*0x3500, 0x3501, 0x3502 will increase VBLANK to
     *get exposure larger than frame exposure
     *AE doesn't update sensor gain at capture mode,
     *thus extra exposure lines must be updated here.
     */

    /* OV Recommend Solution */
    /*if shutter bigger than frame_length,
     *should extend frame length first
     */
    spin_lock(&imgsensor_drv_lock);
    /* Change frame time */
    if (frame_length > 1)
        dummy_line = frame_length - imgsensor.frame_length;

    imgsensor.frame_length = imgsensor.frame_length + dummy_line;

    /*  */
    //if (shutter > imgsensor.frame_length - imgsensor_info.margin)
    //    imgsensor.frame_length = shutter + imgsensor_info.margin;

    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
        imgsensor.frame_length = imgsensor_info.max_frame_length;
    spin_unlock(&imgsensor_drv_lock);
    shutter = (shutter < imgsensor_info.min_shutter)
            ? imgsensor_info.min_shutter : shutter;
    shutter =
    (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
        ? (imgsensor_info.max_frame_length - imgsensor_info.margin)
        : shutter;
    write_cmos_sensor_8(0x0104, 0x01);

    shutter = (shutter >> 1) << 1;
    imgsensor.frame_length = (imgsensor.frame_length >> 1) << 1;
    //frame_length and shutter should be an even number

    if (imgsensor.autoflicker_en) {
        realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 /
                imgsensor.frame_length;
        if (realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(296, 0);
        else if (realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(146, 0);
        else {
            /* Extend frame length */
            write_frame_len(imgsensor.frame_length);
        }
    } else {
        /* Extend frame length */
        write_frame_len(imgsensor.frame_length);
    }

    /* Update Shutter */
    if (auto_extend_en)
        write_cmos_sensor_8(0x0350, 0x01); /* Enable auto extend */
    else
        write_cmos_sensor_8(0x0350, 0x00); /* Disable auto extend */
    write_cmos_sensor_8(0x0202, (shutter >> 8) & 0xFF);
    write_cmos_sensor_8(0x0203, shutter  & 0xFF);
    write_cmos_sensor_8(0x0104, 0x00);
    LOG_INF(
        "Exit! shutter =%d, framelength =%d/%d, dummy_line=%d, auto_extend=%d\n",
        shutter, imgsensor.frame_length, frame_length,
        dummy_line, read_cmos_sensor(0x0350));

}    /* set_shutter_frame_length */

static kal_uint16 gain2reg(const kal_uint16 gain)
{
     kal_uint16 reg_gain = 0x0;

    reg_gain = 16384 - (16384*64)/gain;
    return (kal_uint16) reg_gain;
}

/*************************************************************************
 * FUNCTION
 *    set_gain
 *
 * DESCRIPTION
 *    This function is to set global gain to sensor.
 *
 * PARAMETERS
 *    iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *    the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint16 set_gain_w_gph(kal_uint16 gain, kal_bool gph)
{
    kal_uint16 reg_gain, max_gain = imgsensor_info.max_gain;

    // if (imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM3) {
    //     max_gain = 16 * BASEGAIN;//set 16x gain for full size mode.
    // }

    if (gain < imgsensor_info.min_gain || gain > imgsensor_info.max_gain) {
        LOG_INF("Error gain setting");

        if (gain < imgsensor_info.min_gain)
            gain = imgsensor_info.min_gain;
        else if (gain > max_gain)
            gain = max_gain;
    }

    reg_gain = gain2reg(gain);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.gain = reg_gain;
    spin_unlock(&imgsensor_drv_lock);
    reg_gain = (reg_gain >> 2) << 2;
    LOG_INF(" gph:%d,gain = %d, reg_gain = 0x%x\n", gph, gain, reg_gain);

    if (gph) {
        write_cmos_sensor_8(0x0104, 0x01);
    }
    write_cmos_sensor_8(0x0204, (reg_gain>>8) & 0xFF);
    write_cmos_sensor_8(0x0205, reg_gain & 0xFF);
    if (gph) {
        write_cmos_sensor_8(0x0104, 0x00);
    }
    LOG_INF(" 0x0204:0x%x,0x0205 = 0x%x\n", read_cmos_sensor_8(0x0204), read_cmos_sensor_8(0x0205));
    return gain;
} /* set_gain_w_gph */

static kal_uint16 set_gain(kal_uint16 gain)
{
    return set_gain_w_gph(gain, KAL_TRUE);
} /* set_gain */

/*************************************************************************
 * FUNCTION
 *    night_mode
 *
 * DESCRIPTION
 *    This function night mode of sensor.
 *
 * PARAMETERS
 *    bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 streaming_control(kal_bool enable)
{
    LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n",
        enable);
    if (enable)
        write_cmos_sensor_8(0x0100, 0X01);
    else
        write_cmos_sensor_8(0x0100, 0x00);
    return ERROR_NONE;
}

#define I2C_BUFFER_LEN 255 /* trans# max is 255, each 3 bytes */
static kal_uint16 imx852_table_write_cmos_sensor(kal_uint16 *para,
                         kal_uint32 len)
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
        /* Write when remain buffer size is less than 3 bytes
         * or reach end of data
         */
        if ((I2C_BUFFER_LEN - tosend) < 3
            || IDX == len || addr != addr_last) {
            iBurstWriteReg_multi(puSendCmd,
                        tosend,
                        imgsensor.i2c_write_id,
                        3,
                        imgsensor_info.i2c_speed);
            tosend = 0;
        }
    }
    return 0;
}
#define I2C_BURST_BUFFER_LEN 255 /* trans# max is 255, each 3 bytes */
static kal_uint16 imx852_burst_write_cmos_sensor(
                    kal_uint16 *para, kal_uint32 len)
{
    char puSendCmd[I2C_BURST_BUFFER_LEN];
    kal_uint32 tosend, IDX;
    kal_uint16 addr = 0, addr_last = 0, data;

    tosend = 0;
    IDX = 0;
    while (len > IDX) {
        addr = para[IDX];
        if (tosend == 0)
        {
            puSendCmd[tosend++] = (char)(addr >> 8);
            puSendCmd[tosend++] = (char)(addr & 0xFF);
            data = para[IDX + 1];
            puSendCmd[tosend++] = (char)(data & 0xFF);
            IDX += 2;
            addr_last = addr;
        }
        else if (addr == addr_last + 1)
        {
            data = para[IDX + 1];
            puSendCmd[tosend++] = (char)(data & 0xFF);
            addr_last = addr;
            IDX += 2;
        }

        if ((tosend >= I2C_BURST_BUFFER_LEN)||
            len == IDX ||
            addr != addr_last) {
            iBurstWriteReg_multi(puSendCmd, tosend,
                imgsensor.i2c_write_id,
                tosend, imgsensor_info.i2c_speed);
            tosend = 0;
        }
    }
    return 0;
}

static kal_uint16 imx852_init_setting[] = {
	0x0136, 0x18,
	0x0137, 0x00,
	0xB60C, 0x33,
	0xB63F, 0x66,
	0xB640, 0x0A,
	0x4339, 0x01,
	0x3204, 0x00,
	0x33F0, 0x01,
	0x33F1, 0x05,
	0x0111, 0x02,
	0x4065, 0x01,
	0x5AFD, 0x4E,
	0x5B05, 0x4E,
	0x5B0D, 0x4E,
	0x5B15, 0x4E,
	0x5B1D, 0x4E,
	0x5B25, 0x4E,
	0x81F3, 0x67,
	0x824D, 0x01,
	0x88D2, 0x01,
	0x9002, 0x05,
	0x9003, 0x03,
	0x9004, 0x16,
	0x9210, 0x5A,
	0x9211, 0xEE,
	0x9212, 0x5A,
	0x9213, 0xEF,
	0x9214, 0x5E,
	0x9215, 0xB0,
	0x9216, 0x5E,
	0x9217, 0xB5,
	0x9218, 0x5E,
	0x9219, 0x88,
	0x921A, 0x5A,
	0x921B, 0x7B,
	0x921C, 0x5A,
	0x921D, 0x7C,
	0x921E, 0x5A,
	0x921F, 0x86,
	0x9220, 0x5A,
	0x9221, 0x87,
	0x9222, 0x5E,
	0x9223, 0xA2,
	0x9224, 0x5E,
	0x9225, 0xA0,
	0x9226, 0x99,
	0x9227, 0x2B,
	0x9228, 0x99,
	0x9229, 0x8C,
	0x922A, 0xAF,
	0x922B, 0x25,
	0x922C, 0xAF,
	0x922D, 0x27,
	0x922E, 0xAF,
	0x922F, 0x29,
	0x9230, 0xAE,
	0x9231, 0x12,
	0x9232, 0xAE,
	0x9233, 0x13,
	0x9234, 0xAE,
	0x9235, 0x14,
	0x9236, 0x99,
	0x9237, 0xE5,
	0x9238, 0x99,
	0x9239, 0xE3,
	0x923A, 0x99,
	0x923B, 0x08,
	0x9901, 0x10,
	0x9906, 0x10,
	0xA300, 0x01,
	0xA301, 0x01,
	0xA313, 0x0C,
	0xA319, 0xC0,
	0xA31B, 0x00,
	0xA31C, 0x00,
	0xA31D, 0x00,
	0xA324, 0x00,
	0xA325, 0x00,
	0xA326, 0x00,
	0xA40B, 0x40,
	0xA413, 0x3C,
	0xA415, 0x3C,
	0xA417, 0x3C,
	0xA437, 0x23,
	0xA439, 0x23,
	0xA43B, 0x23,
	0xA43D, 0x0A,
	0xA43F, 0x0A,
	0xA441, 0x0A,
	0xA473, 0x00,
	0xA475, 0x00,
	0xA477, 0x00,
	0xA485, 0x23,
	0xA487, 0x23,
	0xA489, 0x23,
	0xA48B, 0x0A,
	0xA48D, 0x0A,
	0xA48F, 0x0A,
	0xA4A9, 0x1E,
	0xA4AB, 0x1E,
	0xA4AD, 0x1E,
	0xA4BB, 0x28,
	0xA4BD, 0x28,
	0xA4BF, 0x00,
	0xA4C1, 0x03,
	0xA4C3, 0x03,
	0xA4C5, 0x00,
	0xA4CD, 0xC8,
	0xA4CF, 0x10,
	0xA4D2, 0x01,
	0xA4D3, 0x1E,
	0xA4D5, 0x60,
	0xA4DB, 0x1D,
	0xA4ED, 0x08,
	0xA4EF, 0x08,
	0xA4F5, 0x20,
	0xA511, 0x01,
	0xA513, 0x08,
	0xA515, 0x08,
	0xA517, 0x08,
	0xA65A, 0x04,
	0xA65B, 0x04,
	0xA65C, 0x02,
	0xA67F, 0x3F,
	0xA680, 0x3F,
	0xA68E, 0x3F,
	0xA68F, 0x3F,
	0xA7F0, 0x00,
	0xA7F1, 0xD8,
	0xA7F2, 0x00,
	0xA7F3, 0x00,
	0xA7F5, 0x00,
	0xA930, 0x00,
	0xA931, 0xD8,
	0xA932, 0x00,
	0xA933, 0x00,
	0xA935, 0x00,
	0xB07B, 0x20,
	0xB608, 0x11,
	0xB609, 0x91,
	0xB60A, 0x54,
	0xE80C, 0x00,
	0xE80D, 0x00,
	0xE810, 0x01,
	0xE813, 0x01,
	0xE820, 0x0A,
	0xE830, 0x00,
	0xE831, 0x00,
	0xE833, 0x00,
	0xE834, 0x00,
	0xE84C, 0x00,
	0xE84D, 0x00,
	0xE84F, 0x00,
	0xE850, 0x00,
	0x0D06, 0x08,
	0x0D07, 0x52,
	0x0D12, 0xA8,
	0x0D13, 0x11,
	0x0D08, 0x01,
	0x9200, 0x06,
	0x9201, 0x01,
	0x4000, 0x02,
	0x0D06, 0x08,
	0x0D07, 0x56,
	0x0D10, 0xA8,
	0x0D11, 0x01,
	0x0D16, 0xA8,
	0x0D17, 0x21,
	0x9200, 0x06,
	0x9201, 0x01,
	0x4000, 0x02,
	0x0D08, 0x00,
};

static kal_uint16 imx852_preview_setting[] = {
    0x0114, 0x03,
    0x0342, 0x18,
    0x0343, 0xFC,
    0x0340, 0x0E,
    0x0341, 0xE6,
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x00,
    0x0347, 0x00,
    0x0348, 0x1F,
    0x0349, 0xFF,
    0x034A, 0x17,
    0x034B, 0xFF,
    0x0900, 0x01,
    0x0901, 0x22,
    0x0902, 0x00,
    0x30D8, 0x04,
    0x31D0, 0x41,
    0x31D1, 0x41,
    0x0408, 0x00,
    0x0409, 0x00,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x10,
    0x040D, 0x00,
    0x040E, 0x0C,
    0x040F, 0x00,
    0x034C, 0x10,
    0x034D, 0x00,
    0x034E, 0x0C,
    0x034F, 0x00,
    0x0301, 0x06,
    0x0303, 0x02,
    0x0305, 0x04,
    0x0306, 0x01,
    0x0307, 0x6E,
    0x030B, 0x02,
    0x030D, 0x0C,
    0x030E, 0x07,
    0x030F, 0x6B,
    0x32D5, 0x00,
    0x32D6, 0x00,
    0x4008, 0x00,
    0x4009, 0xFF,
    0x400A, 0x00,
    0x400B, 0x00,
    0x400C, 0x00,
    0x400D, 0x82,
    0x400E, 0xE8,
    0x400F, 0xFF,
    0x4010, 0x00,
    0x4011, 0x4D,
    0x4012, 0x36,
    0x4013, 0x04,
    0x4014, 0x04,
    0x4015, 0x40,
    0x4016, 0x40,
    0x4017, 0x40,
    0x4018, 0xFF,
    0x4019, 0xFF,
    0x401A, 0xFF,
    0x401B, 0x07,
    0x401C, 0x07,
    0x401D, 0x06,
    0x40B2, 0x01,
    0x40B3, 0x5E,
    0x40B8, 0x01,
    0x40B9, 0x5E,
    0x0202, 0x0E,
    0x0203, 0xA6,
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x3093, 0x01,
    0x3478, 0x01,
    0x3479, 0xFC,
    0x3072, 0x01,
    0x3073, 0x2b,
};

static kal_uint16 imx852_capture_setting[] = {
    0x0114, 0x03,
    0x0342, 0x18,
    0x0343, 0xFC,
    0x0340, 0x0E,
    0x0341, 0xE6,
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x00,
    0x0347, 0x00,
    0x0348, 0x1F,
    0x0349, 0xFF,
    0x034A, 0x17,
    0x034B, 0xFF,
    0x0900, 0x01,
    0x0901, 0x22,
    0x0902, 0x00,
    0x30D8, 0x04,
    0x31D0, 0x41,
    0x31D1, 0x41,
    0x0408, 0x00,
    0x0409, 0x00,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x10,
    0x040D, 0x00,
    0x040E, 0x0C,
    0x040F, 0x00,
    0x034C, 0x10,
    0x034D, 0x00,
    0x034E, 0x0C,
    0x034F, 0x00,
    0x0301, 0x06,
    0x0303, 0x02,
    0x0305, 0x04,
    0x0306, 0x01,
    0x0307, 0x6E,
    0x030B, 0x02,
    0x030D, 0x0C,
    0x030E, 0x07,
    0x030F, 0x6B,
    0x32D5, 0x00,
    0x32D6, 0x00,
    0x4008, 0x00,
    0x4009, 0xFF,
    0x400A, 0x00,
    0x400B, 0x00,
    0x400C, 0x00,
    0x400D, 0x82,
    0x400E, 0xE8,
    0x400F, 0xFF,
    0x4010, 0x00,
    0x4011, 0x4D,
    0x4012, 0x36,
    0x4013, 0x04,
    0x4014, 0x04,
    0x4015, 0x40,
    0x4016, 0x40,
    0x4017, 0x40,
    0x4018, 0xFF,
    0x4019, 0xFF,
    0x401A, 0xFF,
    0x401B, 0x07,
    0x401C, 0x07,
    0x401D, 0x06,
    0x40B2, 0x01,
    0x40B3, 0x5E,
    0x40B8, 0x01,
    0x40B9, 0x5E,
    0x0202, 0x0E,
    0x0203, 0xA6,
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x3093, 0x01,
    0x3478, 0x01,
    0x3479, 0xFC,
    0x3072, 0x01,
    0x3073, 0x2b,
};

static kal_uint16 imx852_normal_video_setting[] = {
    //reg_B3-S14096x2304 @30FPS Binning w/ PD VB Max.
    0x0114, 0x03,
    0x0342, 0x18,
    0x0343, 0xFC,
    0x0340, 0x0E,
    0x0341, 0xE6,
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x03,
    0x0347, 0x00,
    0x0348, 0x1F,
    0x0349, 0xFF,
    0x034A, 0x14,
    0x034B, 0xFF,
    0x0900, 0x01,
    0x0901, 0x22,
    0x0902, 0x00,
    0x30D8, 0x04,
    0x31D0, 0x41,
    0x31D1, 0x41,
    0x0408, 0x00,
    0x0409, 0x00,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x10,
    0x040D, 0x00,
    0x040E, 0x09,
    0x040F, 0x00,
    0x034C, 0x10,
    0x034D, 0x00,
    0x034E, 0x09,
    0x034F, 0x00,
    0x0301, 0x06,
    0x0303, 0x02,
    0x0305, 0x04,
    0x0306, 0x01,
    0x0307, 0x6E,
    0x030B, 0x02,
    0x030D, 0x0C,
    0x030E, 0x07,
    0x030F, 0x6B,
    0x32D5, 0x00,
    0x32D6, 0x00,
    0x4008, 0x00,
    0x4009, 0xFF,
    0x400A, 0x00,
    0x400B, 0x00,
    0x400C, 0x00,
    0x400D, 0x82,
    0x400E, 0xE8,
    0x400F, 0xFF,
    0x4010, 0x00,
    0x4011, 0x4D,
    0x4012, 0x36,
    0x4013, 0x04,
    0x4014, 0x04,
    0x4015, 0x40,
    0x4016, 0x40,
    0x4017, 0x40,
    0x4018, 0xFF,
    0x4019, 0xFF,
    0x401A, 0xFF,
    0x401B, 0x07,
    0x401C, 0x07,
    0x401D, 0x06,
    0x40B2, 0x01,
    0x40B3, 0x5E,
    0x40B8, 0x01,
    0x40B9, 0x5E,
    0x0202, 0x0E,
    0x0203, 0xA6,
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x3093, 0x01,
    0x3478, 0x01,
    0x3479, 0xFC,
    0x3072, 0x01,
    0x3073, 0x2b,
};

static kal_uint16 imx852_hs_video_setting[] = {
    0x0114, 0x03,
    0x0342, 0x08,
    0x0343, 0xE4,
    0x0340, 0x0A,
    0x0341, 0x78,
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x06,
    0x0347, 0x60,
    0x0348, 0x1F,
    0x0349, 0xFF,
    0x034A, 0x11,
    0x034B, 0x9F,
    0x0900, 0x01,
    0x0901, 0x44,
    0x0902, 0x00,
    0x30D8, 0x00,
    0x31D0, 0x43,
    0x31D1, 0x43,
    0x0408, 0x01,
    0x0409, 0x80,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x05,
    0x040D, 0x00,
    0x040E, 0x02,
    0x040F, 0xD0,
    0x034C, 0x05,
    0x034D, 0x00,
    0x034E, 0x02,
    0x034F, 0xD0,
    0x0301, 0x06,
    0x0303, 0x02,
    0x0305, 0x04,
    0x0306, 0x01,
    0x0307, 0x6E,
    0x030B, 0x02,
    0x030D, 0x0C,
    0x030E, 0x07,
    0x030F, 0x6B,
    0x32D5, 0x00,
    0x32D6, 0x00,
    0x4008, 0x00,
    0x4009, 0xFF,
    0x400A, 0x00,
    0x400B, 0x00,
    0x400C, 0x00,
    0x400D, 0x82,
    0x400E, 0xE8,
    0x400F, 0xFF,
    0x4010, 0x00,
    0x4011, 0x4D,
    0x4012, 0x36,
    0x4013, 0x0D,
    0x4014, 0x0D,
    0x4015, 0x63,
    0x4016, 0x42,
    0x4017, 0x42,
    0x4018, 0x12,
    0x4019, 0x20,
    0x401A, 0x80,
    0x401B, 0x1C,
    0x401C, 0x1C,
    0x401D, 0x12,
    0x40B2, 0x03,
    0x40B3, 0x84,
    0x40B8, 0x03,
    0x40B9, 0x84,
    0x0202, 0x0A,
    0x0203, 0x38,
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x3093, 0x00,
    0x3478, 0x01,
    0x3479, 0xFC,
    0x3072, 0x00,
    0x3073, 0x00,
};

static kal_uint16 imx852_slim_video_setting[] = {
    //reg_B3-S14096x2304 @30FPS Binning w/ PD VB Max.
    0x0114, 0x03,
    0x0342, 0x18,
    0x0343, 0xFC,
    0x0340, 0x0E,
    0x0341, 0xE6,
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x03,
    0x0347, 0x00,
    0x0348, 0x1F,
    0x0349, 0xFF,
    0x034A, 0x14,
    0x034B, 0xFF,
    0x0900, 0x01,
    0x0901, 0x22,
    0x0902, 0x00,
    0x30D8, 0x04,
    0x31D0, 0x41,
    0x31D1, 0x41,
    0x0408, 0x00,
    0x0409, 0x00,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x10,
    0x040D, 0x00,
    0x040E, 0x09,
    0x040F, 0x00,
    0x034C, 0x10,
    0x034D, 0x00,
    0x034E, 0x09,
    0x034F, 0x00,
    0x0301, 0x06,
    0x0303, 0x02,
    0x0305, 0x04,
    0x0306, 0x01,
    0x0307, 0x6E,
    0x030B, 0x02,
    0x030D, 0x0C,
    0x030E, 0x07,
    0x030F, 0x6B,
    0x32D5, 0x00,
    0x32D6, 0x00,
    0x4008, 0x00,
    0x4009, 0xFF,
    0x400A, 0x00,
    0x400B, 0x00,
    0x400C, 0x00,
    0x400D, 0x82,
    0x400E, 0xE8,
    0x400F, 0xFF,
    0x4010, 0x00,
    0x4011, 0x4D,
    0x4012, 0x36,
    0x4013, 0x04,
    0x4014, 0x04,
    0x4015, 0x40,
    0x4016, 0x40,
    0x4017, 0x40,
    0x4018, 0xFF,
    0x4019, 0xFF,
    0x401A, 0xFF,
    0x401B, 0x07,
    0x401C, 0x07,
    0x401D, 0x06,
    0x40B2, 0x01,
    0x40B3, 0x5E,
    0x40B8, 0x01,
    0x40B9, 0x5E,
    0x0202, 0x0E,
    0x0203, 0xA6,
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x3093, 0x01,
    0x3478, 0x01,
    0x3479, 0xFC,
    0x3072, 0x01,
    0x3073, 0x2b,
};

static kal_uint16 imx852_custom1_setting[] = {
    0x0114, 0x03,
    0x0342, 0x18,
    0x0343, 0xFC,
    0x0340, 0x0E,
    0x0341, 0xE6,
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x00,
    0x0347, 0x00,
    0x0348, 0x1F,
    0x0349, 0xFF,
    0x034A, 0x17,
    0x034B, 0xFF,
    0x0900, 0x01,
    0x0901, 0x22,
    0x0902, 0x00,
    0x30D8, 0x04,
    0x31D0, 0x41,
    0x31D1, 0x41,
    0x0408, 0x00,
    0x0409, 0x00,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x10,
    0x040D, 0x00,
    0x040E, 0x0C,
    0x040F, 0x00,
    0x034C, 0x10,
    0x034D, 0x00,
    0x034E, 0x0C,
    0x034F, 0x00,
    0x0301, 0x06,
    0x0303, 0x02,
    0x0305, 0x04,
    0x0306, 0x01,
    0x0307, 0x6E,
    0x030B, 0x02,
    0x030D, 0x0C,
    0x030E, 0x07,
    0x030F, 0x6B,
    0x32D5, 0x00,
    0x32D6, 0x00,
    0x4008, 0x00,
    0x4009, 0xFF,
    0x400A, 0x00,
    0x400B, 0x00,
    0x400C, 0x00,
    0x400D, 0x82,
    0x400E, 0xE8,
    0x400F, 0xFF,
    0x4010, 0x00,
    0x4011, 0x4D,
    0x4012, 0x36,
    0x4013, 0x04,
    0x4014, 0x04,
    0x4015, 0x40,
    0x4016, 0x40,
    0x4017, 0x40,
    0x4018, 0xFF,
    0x4019, 0xFF,
    0x401A, 0xFF,
    0x401B, 0x07,
    0x401C, 0x07,
    0x401D, 0x06,
    0x40B2, 0x01,
    0x40B3, 0x5E,
    0x40B8, 0x01,
    0x40B9, 0x5E,
    0x0202, 0x0E,
    0x0203, 0xA6,
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x3093, 0x01,
    0x3478, 0x01,
    0x3479, 0xFC,
    0x3072, 0x01,
    0x3073, 0x2b,
};

static kal_uint16 imx852_custom2_setting[] = {
    0x0114, 0x03,
    0x0342, 0x18,
    0x0343, 0xFC,
    0x0340, 0x12,
    0x0341, 0xA0,
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x00,
    0x0347, 0x00,
    0x0348, 0x1F,
    0x0349, 0xFF,
    0x034A, 0x17,
    0x034B, 0xFF,
    0x0900, 0x01,
    0x0901, 0x22,
    0x0902, 0x00,
    0x30D8, 0x04,
    0x31D0, 0x41,
    0x31D1, 0x41,
    0x0408, 0x00,
    0x0409, 0x00,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x10,
    0x040D, 0x00,
    0x040E, 0x0C,
    0x040F, 0x00,
    0x034C, 0x10,
    0x034D, 0x00,
    0x034E, 0x0C,
    0x034F, 0x00,
    0x0301, 0x06,
    0x0303, 0x02,
    0x0305, 0x04,
    0x0306, 0x01,
    0x0307, 0x6E,
    0x030B, 0x02,
    0x030D, 0x0C,
    0x030E, 0x07,
    0x030F, 0x6B,
    0x32D5, 0x00,
    0x32D6, 0x00,
    0x4008, 0x00,
    0x4009, 0xFF,
    0x400A, 0x00,
    0x400B, 0x00,
    0x400C, 0x00,
    0x400D, 0x82,
    0x400E, 0xE8,
    0x400F, 0xFF,
    0x4010, 0x00,
    0x4011, 0x4D,
    0x4012, 0x36,
    0x4013, 0x04,
    0x4014, 0x04,
    0x4015, 0x40,
    0x4016, 0x40,
    0x4017, 0x40,
    0x4018, 0xFF,
    0x4019, 0xFF,
    0x401A, 0xFF,
    0x401B, 0x07,
    0x401C, 0x07,
    0x401D, 0x06,
    0x40B2, 0x01,
    0x40B3, 0x5E,
    0x40B8, 0x01,
    0x40B9, 0x5E,
    0x0202, 0x12,
    0x0203, 0x60,
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x3093, 0x01,
    0x3478, 0x01,
    0x3479, 0xFC,
    0x3072, 0x01,
    0x3073, 0x2b,
};

static kal_uint16 imx852_custom3_setting[] = {
    0x0114, 0x03,
    0x0342, 0x10,
    0x0343, 0x38,
    0x0340, 0x07,
    0x0341, 0xA6,
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x00,
    0x0347, 0x00,
    0x0348, 0x1F,
    0x0349, 0xFF,
    0x034A, 0x17,
    0x034B, 0xFF,
    0x0900, 0x01,
    0x0901, 0x44,
    0x0902, 0x02,
    0x30D8, 0x04,
    0x31D0, 0x43,
    0x31D1, 0x43,
    0x0408, 0x00,
    0x0409, 0x00,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x08,
    0x040D, 0x00,
    0x040E, 0x06,
    0x040F, 0x00,
    0x034C, 0x08,
    0x034D, 0x00,
    0x034E, 0x06,
    0x034F, 0x00,
    0x0301, 0x06,
    0x0303, 0x02,
    0x0305, 0x04,
    0x0306, 0x01,
    0x0307, 0x6E,
    0x030B, 0x02,
    0x030D, 0x0C,
    0x030E, 0x05,
    0x030F, 0x4A,
    0x32D5, 0x00,
    0x32D6, 0x00,
    0x4008, 0x00,
    0x4009, 0xFF,
    0x400A, 0x00,
    0x400B, 0x00,
    0x400C, 0x00,
    0x400D, 0x82,
    0x400E, 0xE8,
    0x400F, 0xFF,
    0x4010, 0x00,
    0x4011, 0x4D,
    0x4012, 0x36,
    0x4013, 0x0D,
    0x4014, 0x0D,
    0x4015, 0x63,
    0x4016, 0x42,
    0x4017, 0x42,
    0x4018, 0x12,
    0x4019, 0x20,
    0x401A, 0x80,
    0x401B, 0x1C,
    0x401C, 0x1C,
    0x401D, 0x12,
    0x40B2, 0x06,
    0x40B3, 0xA4,
    0x40B8, 0x06,
    0x40B9, 0xA4,
    0x0202, 0x07,
    0x0203, 0x66,
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x3093, 0x01,
    0x3478, 0x01,
    0x3479, 0xFC,
    0x3072, 0x01,
    0x3073, 0x2b,
};

static kal_uint16 imx852_custom4_setting[] = {
    0x0114, 0x03,
    0x0342, 0x18,
    0x0343, 0xFC,
    0x0340, 0x0E,
    0x0341, 0xE6,
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x02,
    0x0347, 0x70,
    0x0348, 0x1F,
    0x0349, 0xFF,
    0x034A, 0x15,
    0x034B, 0x8F,
    0x0900, 0x01,
    0x0901, 0x22,
    0x0902, 0x00,
    0x30D8, 0x04,
    0x31D0, 0x41,
    0x31D1, 0x41,
    0x0408, 0x01,
    0x0409, 0xA0,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x0C,
    0x040D, 0xC0,
    0x040E, 0x09,
    0x040F, 0x90,
    0x034C, 0x0C,
    0x034D, 0xC0,
    0x034E, 0x09,
    0x034F, 0x90,
    0x0301, 0x06,
    0x0303, 0x02,
    0x0305, 0x04,
    0x0306, 0x01,
    0x0307, 0x6E,
    0x030B, 0x02,
    0x030D, 0x0C,
    0x030E, 0x07,
    0x030F, 0x6B,
    0x32D5, 0x00,
    0x32D6, 0x00,
    0x4008, 0x00,
    0x4009, 0xFF,
    0x400A, 0x00,
    0x400B, 0x00,
    0x400C, 0x00,
    0x400D, 0x82,
    0x400E, 0xE8,
    0x400F, 0xFF,
    0x4010, 0x00,
    0x4011, 0x4D,
    0x4012, 0x36,
    0x4013, 0x04,
    0x4014, 0x04,
    0x4015, 0x40,
    0x4016, 0x40,
    0x4017, 0x40,
    0x4018, 0xFF,
    0x4019, 0xFF,
    0x401A, 0xFF,
    0x401B, 0x07,
    0x401C, 0x07,
    0x401D, 0x06,
    0x40B2, 0x01,
    0x40B3, 0x5E,
    0x40B8, 0x01,
    0x40B9, 0x5E,
    0x0202, 0x0E,
    0x0203, 0xA6,
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x3093, 0x01,
    0x3478, 0x01,
    0x3479, 0x98,
    0x3072, 0x01,
    0x3073, 0x2b,
};

static kal_uint16 imx852_custom5_setting[] = {
    //3Lane
    //reg_A-3-S1
    //Full-RMSC-Crop_4096x3072_30FPS with PDAF VB_max seamless A-2-S1
    //H: 4096
    //V: 3072
    //min shutter 9
    //step 1
    //margin 48
    //MIPI output setting
    //Address    value
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x02,
    //Line Length PCK Setting
    //Address    value
    0x0342, 0x23,
    0x0343, 0x00,
    0x3850, 0x00,
    0x3851, 0xF6,
    //Frame Length Lines Setting
    //Address    value
    0x0340, 0x0C,
    0x0341, 0xB4,
    //ROI Setting
    //Address    value
    0x0344, 0x08,
    0x0345, 0x00,
    0x0346, 0x06,
    0x0347, 0x00,
    0x0348, 0x17,
    0x0349, 0xFF,
    0x034A, 0x11,
    0x034B, 0xFF,
    //Mode Setting
    //Address    value
    0x0900, 0x00,
    0x0901, 0x11,
    0x0902, 0x00,
    0x3005, 0x00,
    0x3006, 0x00,
    0x3140, 0x0A,
    0x3144, 0x00,
    0x3148, 0x00,
    0x31C0, 0x01,
    0x31C1, 0x01,
    0x3205, 0x01,
    0x323C, 0x01,
    0x39AC, 0x01,
    //Digital Crop & Scaling
    //Address    value
    0x0408, 0x00,
    0x0409, 0x00,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x10,
    0x040D, 0x00,
    0x040E, 0x0C,
    0x040F, 0x00,
    //Output Size Setting
    //Address    value
    0x034C, 0x10,
    0x034D, 0x00,
    0x034E, 0x0C,
    0x034F, 0x00,
    //Clock Setting
    //Address    value
    0x0301, 0x05,
    0x0303, 0x02,
    0x0305, 0x04,
    0x0306, 0x01,
    0x0307, 0x6E,
    0x030B, 0x02,
    0x030D, 0x06,
    0x030E, 0x01,
    0x030F, 0xFF,
    //Other Setting
    //Address    value
    0x3104, 0x01,
    0x38A0, 0x00,
    0x38A1, 0x00,
    0x38A2, 0x00,
    0x38A3, 0x00,
    0x38A8, 0x00,
    0x38A9, 0x00,
    0x38AA, 0x00,
    0x38AB, 0x00,
    0x38B0, 0x03,
    0x38B1, 0xFF,
    0x38B4, 0x03,
    0x38B5, 0xFF,
    0x38B8, 0x03,
    0x38B9, 0xFF,
    0x38BC, 0x03,
    0x38BD, 0xFF,
    0x38D0, 0x03,
    0x38D1, 0x52,
    0x38D2, 0x03,
    0x38D3, 0x52,
    0x38D8, 0x14,
    0x38E0, 0x00,
    0x38E1, 0x00,
    0x38E2, 0x00,
    0x38E3, 0x00,
    0x38E4, 0x00,
    0x38E5, 0x00,
    0x38E6, 0x00,
    0x38E7, 0x00,
    0x3B00, 0x00,
    0x3B01, 0x00,
    0x3B04, 0x00,
    0x3B05, 0x00,
    //Integration Setting
    //Address    value
    0x0202, 0x03,
    0x0203, 0xE8,
    //Gain Setting
    //Address    value
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    //EAE-Bracketing Setting
    //0x0e00, 0x00,
    //PDAF TYPE2 Setting
    //Address    value
    0x3103, 0x00,
    0x3422, 0x01,
    0x3423, 0xFC,
    //EAE-Bracketing Setting
    //Address    value
    0x0E00,    0x00,
    //PHASE PIX VCID Setting
    //Address    value
    0x30A4, 0x00,
    0x30A6, 0x00,
    0x30F2, 0x01,
    0x30F3, 0x01,
    //PHASE PIX data type Setting
    //Address    value
    0x30A5, 0x30,
    0x30A7, 0x30,
    //PDAF TYPE2 VCID Setting
    //Address    value
    0x30A2, 0x00,
    0x30F1, 0x01,
    //PDAF TYPE2 data type Setting
    //Address    value
    0x30A3, 0x30,
    //MIPI Global Timing Setting
    //Address    value
    0x084E, 0x00,
    0x084F, 0x09,
    0x0850, 0x00,
    0x0851, 0x09,
    0x0852, 0x00,
    0x0853, 0x0F,
    0x0854, 0x00,
    0x0855, 0x29,
    0x0858, 0x00,
    0x0859, 0x1F,
};

static kal_uint16 imx852_custom6_setting[] = {
    //3Lane
    //reg_C-6
    //QBIN-V2H2 1280x674_480FPS w/o PDAF VB_max
    //H: 1280
    //V: 674
    //min shutter 9
    //step 1
    //margin 48
    //MIPI output setting
    //Address    value
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x02,
    //Line Length PCK Setting
    //Address    value
    0x0342, 0x09,
    0x0343, 0xA4,
    0x3850, 0x00,
    0x3851, 0x44,
    //Frame Length Lines Setting
    //Address    value
    0x0340, 0x02,
    0x0341, 0xDE,
    //ROI Setting
    //Address    value
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x06,
    0x0347, 0xA0,
    0x0348, 0x1F,
    0x0349, 0xFF,
    0x034A, 0x11,
    0x034B, 0x3F,
    //Mode Setting
    //Address    value
    0x0900, 0x01,
    0x0901, 0x44,
    0x0902, 0x02,
    0x3005, 0x00,
    0x3006, 0x00,
    0x3140, 0x0A,
    0x3144, 0x00,
    0x3148, 0x00,
    0x31C0, 0x43,
    0x31C1, 0x43,
    0x3205, 0x00,
    0x323C, 0x01,
    0x39AC, 0x01,
    //Digital Crop & Scaling
    //Address    value
    0x0408, 0x01,
    0x0409, 0x80,
    0x040A, 0x00,
    0x040B, 0x06,
    0x040C, 0x05,
    0x040D, 0x00,
    0x040E, 0x02,
    0x040F, 0xA2,
    //Output Size Setting
    //Address    value
    0x034C, 0x05,
    0x034D, 0x00,
    0x034E, 0x02,
    0x034F, 0xA2,
    //Clock Setting
    //Address    value
    0x0301, 0x05,
    0x0303, 0x02,
    0x0305, 0x04,
    0x0306, 0x01,
    0x0307, 0x6E,
    0x030B, 0x02,
    0x030D, 0x06,
    0x030E, 0x01,
    0x030F, 0xFA,
    //Other Setting
    //Address    value
    0x3104, 0x00,
    0x38A0, 0x00,
    0x38A1, 0x00,
    0x38A2, 0x00,
    0x38A3, 0x00,
    0x38A8, 0x00,
    0x38A9, 0x00,
    0x38AA, 0x00,
    0x38AB, 0x00,
    0x38B0, 0x03,
    0x38B1, 0xFF,
    0x38B4, 0x03,
    0x38B5, 0xFF,
    0x38B8, 0x03,
    0x38B9, 0xFF,
    0x38BC, 0x03,
    0x38BD, 0xFF,
    0x38D0, 0x00,
    0x38D1, 0x32,
    0x38D2, 0x00,
    0x38D3, 0x32,
    0x38D8, 0x14,
    0x38E0, 0x00,
    0x38E1, 0x00,
    0x38E2, 0x00,
    0x38E3, 0x00,
    0x38E4, 0x00,
    0x38E5, 0x00,
    0x38E6, 0x00,
    0x38E7, 0x00,
    0x3B00, 0x00,
    0x3B01, 0x00,
    0x3B04, 0x00,
    0x3B05, 0x00,
    //Integration Setting
    //Address    value
    0x0202, 0x02,
    0x0203, 0xA6,
    //Gain Setting
    //Address    value
    0x0204, 0x13,
    0x0205, 0x34,
    0x020E, 0x01,
    0x020F, 0x00,
    //EAE-Bracketing Setting
    //0x0e00, 0x00,
    //PDAF TYPE2 Setting
    //Address    value
    0x3103, 0x00,
    0x3422, 0x01,
    0x3423, 0xFC,
    //EAE-Bracketing Setting
    //Address    value
    0x0E00,    0x00,
    //PHASE PIX VCID Setting
    //Address    value
    0x30A4, 0x00,
    0x30A6, 0x00,
    0x30F2, 0x01,
    0x30F3, 0x01,
    //PHASE PIX data type Setting
    //Address    value
    0x30A5, 0x30,
    0x30A7, 0x30,
    //PDAF TYPE2 VCID Setting
    //Address    value
    0x30A2, 0x00,
    0x30F1, 0x01,
    //PDAF TYPE2 data type Setting
    //Address    value
    0x30A3, 0x30,
    //MIPI Global Timing Setting
    //Address    value
    0x084E, 0x00,
    0x084F, 0x09,
    0x0850, 0x00,
    0x0851, 0x09,
    0x0852, 0x00,
    0x0853, 0x0F,
    0x0854, 0x00,
    0x0855, 0x29,
    0x0858, 0x00,
    0x0859, 0x1F,
};
static kal_uint16 imx852_custom7_setting[] = {
//3Lane
//reg_A
//QBIN(VBIN)_4096x3072_30FPS with PDAF VB_max
//H: 4096
//V: 3072
//MIPI output setting
//Address   value
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x02,
//Line Length PCK Setting
//Address   value
    0x0342, 0x1D,
    0x0343, 0x4C,
    0x3850, 0x00,
    0x3851, 0xCD,
//Frame Length Lines Setting
//Address   value
    0x0340, 0x0F,
    0x0341, 0x3C,
//ROI Setting
//Address   value
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x00,
    0x0347, 0x00,
    0x0348, 0x1F,
    0x0349, 0xFF,
    0x034A, 0x17,
    0x034B, 0xFF,
//Mode Setting
//Address   value
    0x0900, 0x01,
    0x0901, 0x22,
    0x0902, 0x00,
    0x3005, 0x02,
    0x3006, 0x02,
    0x3140, 0x0A,
    0x3144, 0x00,
    0x3148, 0x04,
    0x31C0, 0x41,
    0x31C1, 0x41,
    0x3205, 0x00,
    0x323C, 0x01,
    0x39AC, 0x01,
//Digital Crop & Scaling
//Address   value
    0x0408, 0x00,
    0x0409, 0x00,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x10,
    0x040D, 0x00,
    0x040E, 0x0C,
    0x040F, 0x00,
//Output Size Setting
//Address   value
    0x034C, 0x10,
    0x034D, 0x00,
    0x034E, 0x0C,
    0x034F, 0x00,
//Clock Setting
//Address   value
    0x0301, 0x05,
    0x0303, 0x02,
    0x0305, 0x04,
    0x0306, 0x01,
    0x0307, 0x6E,
    0x030B, 0x02,
    0x030D, 0x06,
    0x030E, 0x01,
    0x030F, 0xFF,
//Other Setting
//Address   value
    0x3104, 0x01,
    0x38A0, 0x00,
    0x38A1, 0x00,
    0x38A2, 0x00,
    0x38A3, 0x00,
    0x38A8, 0x00,
    0x38A9, 0x00,
    0x38AA, 0x00,
    0x38AB, 0x00,
    0x38B0, 0x03,
    0x38B1, 0xFF,
    0x38B4, 0x03,
    0x38B5, 0xFF,
    0x38B8, 0x03,
    0x38B9, 0xFF,
    0x38BC, 0x03,
    0x38BD, 0xFF,
    0x38D0, 0x07,
    0x38D1, 0x08,
    0x38D2, 0x07,
    0x38D3, 0x08,
    0x38D8, 0x14,
    0x38E0, 0x00,
    0x38E1, 0x00,
    0x38E2, 0x00,
    0x38E3, 0x00,
    0x38E4, 0x00,
    0x38E5, 0x00,
    0x38E6, 0x00,
    0x38E7, 0x00,
    0x3B00, 0x00,
    0x3B01, 0x00,
    0x3B04, 0x00,
    0x3B05, 0x00,
//Integration Setting
//Address   value
    0x0202, 0x03,
    0x0203, 0xE8,
//Gain Setting
//Address   value
    0x0204, 0x13,
    0x0205, 0x34,
    0x020E, 0x01,
    0x020F, 0x00,
//PDAF TYPE2 Setting
//Address   value
    0x3103, 0x00,
    0x3422, 0x01,
    0x3423, 0xFC,
//EAE-Bracketing Setting
//Address    value
    0x0E00, 0x00,
//PHASE PIX VCID Setting
//Address   value
    0x30A4, 0x00,
    0x30A6, 0x00,
    0x30F2, 0x01,
    0x30F3, 0x01,
//PHASE PIX data type Setting
//Address   value
    0x30A5, 0x30,
    0x30A7, 0x30,
//PDAF TYPE2 VCID Setting
//Address   value
    0x30A2, 0x00,
    0x30F1, 0x01,
//PDAF TYPE2 data type Setting
//Address   value
    0x30A3, 0x30,
//MIPI Global Timing Setting
//Address   value
    0x084E, 0x00,
    0x084F, 0x09,
    0x0850, 0x00,
    0x0851, 0x09,
    0x0852, 0x00,
    0x0853, 0x0F,
    0x0854, 0x00,
    0x0855, 0x29,
    0x0858, 0x00,
    0x0859, 0x1F,
};

static kal_uint16 imx852_custom8_setting[] = {
//3Lane
//reg_B
//QBIN(VBIN)_4096x2304 @30FPS with PDAF VB_max
//H: 4096
//V: 2304
//min shutter 8
//step 2
//margin 48
//MIPI output setting
    //Address    value
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x02,
    //Line Length PCK Setting
    //Address    value
    0x0342, 0x1D,
    0x0343, 0x4C,
    0x3850, 0x00,
    0x3851, 0xCD,
    //Frame Length Lines Setting
    //Address    value
    0x0340, 0x0F,
    0x0341, 0x3C,
    //ROI Setting
    //Address    value
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x03,
    0x0347, 0x00,
    0x0348, 0x1F,
    0x0349, 0xFF,
    0x034A, 0x14,
    0x034B, 0xFF,
    //Mode Setting
    //Address    value
    0x0900, 0x01,
    0x0901, 0x22,
    0x0902, 0x00,
    0x3005, 0x02,
    0x3006, 0x02,
    0x3140, 0x0A,
    0x3144, 0x00,
    0x3148, 0x04,
    0x31C0, 0x41,
    0x31C1, 0x41,
    0x3205, 0x00,
    0x323C, 0x01,
    0x39AC, 0x01,
    //Digital Crop & Scaling
    //Address    value
    0x0408, 0x00,
    0x0409, 0x00,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x10,
    0x040D, 0x00,
    0x040E, 0x09,
    0x040F, 0x00,
    //Output Size Setting
    //Address    value
    0x034C, 0x10,
    0x034D, 0x00,
    0x034E, 0x09,
    0x034F, 0x00,
    //Clock Setting
    //Address    value
    0x0301, 0x05,
    0x0303, 0x02,
    0x0305, 0x04,
    0x0306, 0x01,
    0x0307, 0x6E,
    0x030B, 0x02,
    0x030D, 0x06,
    0x030E, 0x01,
    0x030F, 0xFF,
    //Other Setting
    //Address    value
    0x3104, 0x01,
    0x38A0, 0x00,
    0x38A1, 0x00,
    0x38A2, 0x00,
    0x38A3, 0x00,
    0x38A8, 0x00,
    0x38A9, 0x00,
    0x38AA, 0x00,
    0x38AB, 0x00,
    0x38B0, 0x03,
    0x38B1, 0xFF,
    0x38B4, 0x03,
    0x38B5, 0xFF,
    0x38B8, 0x03,
    0x38B9, 0xFF,
    0x38BC, 0x03,
    0x38BD, 0xFF,
    0x38D0, 0x07,
    0x38D1, 0x08,
    0x38D2, 0x07,
    0x38D3, 0x08,
    0x38D8, 0x14,
    0x38E0, 0x00,
    0x38E1, 0x00,
    0x38E2, 0x00,
    0x38E3, 0x00,
    0x38E4, 0x00,
    0x38E5, 0x00,
    0x38E6, 0x00,
    0x38E7, 0x00,
    0x3B00, 0x00,
    0x3B01, 0x00,
    0x3B04, 0x00,
    0x3B05, 0x00,
    //Integration Setting
    //Address    value
    0x0202, 0x03,
    0x0203, 0xE8,
    //Gain Setting
    //Address    value
    0x0204, 0x13,
    0x0205, 0x34,
    0x020E, 0x01,
    0x020F, 0x00,
    //PDAF TYPE2 Setting
    //Address    value
    0x3103, 0x00,
    0x3422, 0x01,
    0x3423, 0xFC,
    //EAE-Bracketing Setting
    //Address    value
    0x0E00,    0x00,
    //PHASE PIX VCID Setting
    //Address    value
    0x30A4, 0x00,
    0x30A6, 0x00,
    0x30F2, 0x01,
    0x30F3, 0x01,
    //PHASE PIX data type Setting
    //Address    value
    0x30A5, 0x30,
    0x30A7, 0x30,
    //PDAF TYPE2 VCID Setting
    //Address    value
    0x30A2, 0x00,
    0x30F1, 0x01,
    //PDAF TYPE2 data type Setting
    //Address    value
    0x30A3, 0x30,
    //MIPI Global Timing Setting
    //Address    value
    0x084E, 0x00,
    0x084F, 0x09,
    0x0850, 0x00,
    0x0851, 0x09,
    0x0852, 0x00,
    0x0853, 0x0F,
    0x0854, 0x00,
    0x0855, 0x29,
    0x0858, 0x00,
    0x0859, 0x1F,
};
static kal_uint16 imx852_custom9_setting[] = {
    //3Lane
    //reg_V2
    //H: 2048
    //V: 1536
    //MIPI output setting
    //Address    value
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x02,
    //Line Length PCK Setting
    //Address    value
    0x0342, 0x0F,
    0x0343, 0xB8,
    0x3850, 0x01,
    0x3851, 0xCC,
    //Frame Length Lines Setting
    //Address    value
    0x0340, 0x08,
    0x0341, 0x78,
    //ROI Setting
    //Address    value
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x00,
    0x0347, 0x00,
    0x0348, 0x1F,
    0x0349, 0xFF,
    0x034A, 0x17,
    0x034B, 0xFF,
    //Mode Setting
    //Address    value
    0x0900, 0x01,
    0x0901, 0x44,
    0x0902, 0x02,
    0x3005, 0x02,
    0x3006, 0x02,
    0x3140, 0x0A,
    0x3144, 0x00,
    0x3148, 0x04,
    0x31C0, 0x43,
    0x31C1, 0x43,
    0x3205, 0x00,
    0x323C, 0x01,
    0x39AC, 0x01,
    //Digital Crop & Scaling
    //Address    value
    0x0408, 0x00,
    0x0409, 0x00,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x08,
    0x040D, 0x00,
    0x040E, 0x06,
    0x040F, 0x00,
    //Output Size Setting
    //Address    value
    0x034C, 0x08,
    0x034D, 0x00,
    0x034E, 0x06,
    0x034F, 0x00,
    //Clock Setting
    //Address    value
    0x0301, 0x05,
    0x0303, 0x04,
    0x0305, 0x04,
    0x0306, 0x00,
    0x0307, 0xAF,
    0x030B, 0x04,
    0x030D, 0x06,
    0x030E, 0x01,
    0x030F, 0xF4,
    //Other Setting
    //Address    value
    0x3104, 0x01,
    0x3104, 0x01,
    0x38A0, 0x00,
    0x38A1, 0x00,
    0x38A2, 0x00,
    0x38A3, 0x00,
    0x38A8, 0x00,
    0x38A9, 0x00,
    0x38AA, 0x00,
    0x38AB, 0x00,
    0x38B0, 0x03,
    0x38B1, 0xFF,
    0x38B4, 0x03,
    0x38B5, 0xFF,
    0x38B8, 0x03,
    0x38B9, 0xFF,
    0x38BC, 0x03,
    0x38BD, 0xFF,
    0x38D0, 0x05,
    0x38D1, 0x14,
    0x38D2, 0x05,
    0x38D3, 0x14,
    0x38D8, 0x14,
    0x38E0, 0x00,
    0x38E1, 0x00,
    0x38E2, 0x00,
    0x38E3, 0x00,
    0x38E4, 0x00,
    0x38E5, 0x00,
    0x38E6, 0x00,
    0x38E7, 0x00,
    0x3B00, 0x00,
    0x3B01, 0x00,
    0x3B04, 0x00,
    0x3B05, 0x00,
    //Integration Setting
    //Address    value
    0x0202, 0x03,
    0x0203, 0xE8,
    //Gain Setting
    //Address    value
    0x0204, 0x13,
    0x0205, 0x34,
    0x020E, 0x01,
    0x020F, 0x00,
    //EAE-Bracketing Setting
    //0x0e00, 0x00,
    //PDAF TYPE2 Setting
    //Address    value
    0x3103, 0x00,
    0x3422, 0x01,
    0x3423, 0xFC,
    //EAE-Bracketing Setting
    //Address    value
    0x0E00, 0x00,
    //PHASE PIX VCID Setting
    //Address    value
    0x30A4, 0x00,
    0x30A6, 0x00,
    0x30F2, 0x01,
    0x30F3, 0x01,
    //PHASE PIX data type Setting
    //Address    value
    0x30A5, 0x30,
    0x30A7, 0x30,
    //PDAF TYPE2 VCID Setting
    //Address    value
    0x30A2, 0x00,
    0x30F1, 0x01,
    //PDAF TYPE2 data type Setting
    //Address    value
    0x30A3, 0x30,
    //MIPI Global Timing Setting
    //Address    value
    0x084E, 0x00,
    0x084F, 0x05,
    0x0850, 0x00,
    0x0851, 0x05,
    0x0852, 0x00,
    0x0853, 0x09,
    0x0854, 0x00,
    0x0855, 0x29,
    0x0858, 0x00,
    0x0859, 0x1F,
};

static kal_uint16 imx852_custom10_setting[] = {
//3Lane
//reg_B
//QBIN(VBIN)_4096x2304 @30FPS with PDAF VB_max
//H: 4096
//V: 2304
//min shutter 8
//step 2
//margin 48
//MIPI output setting
    //Address    value
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x02,
    //Line Length PCK Setting
    //Address    value
    0x0342, 0x1D,
    0x0343, 0x4C,
    0x3850, 0x00,
    0x3851, 0xCD,
    //Frame Length Lines Setting
    //Address    value
    0x0340, 0x0F,
    0x0341, 0x3C,
    //ROI Setting
    //Address    value
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x03,
    0x0347, 0x00,
    0x0348, 0x1F,
    0x0349, 0xFF,
    0x034A, 0x14,
    0x034B, 0xFF,
    //Mode Setting
    //Address    value
    0x0900, 0x01,
    0x0901, 0x22,
    0x0902, 0x00,
    0x3005, 0x02,
    0x3006, 0x02,
    0x3140, 0x0A,
    0x3144, 0x00,
    0x3148, 0x04,
    0x31C0, 0x41,
    0x31C1, 0x41,
    0x3205, 0x00,
    0x323C, 0x01,
    0x39AC, 0x01,
    //Digital Crop & Scaling
    //Address    value
    0x0408, 0x00,
    0x0409, 0x00,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x10,
    0x040D, 0x00,
    0x040E, 0x09,
    0x040F, 0x00,
    //Output Size Setting
    //Address    value
    0x034C, 0x10,
    0x034D, 0x00,
    0x034E, 0x09,
    0x034F, 0x00,
    //Clock Setting
    //Address    value
    0x0301, 0x05,
    0x0303, 0x02,
    0x0305, 0x04,
    0x0306, 0x01,
    0x0307, 0x6E,
    0x030B, 0x02,
    0x030D, 0x06,
    0x030E, 0x01,
    0x030F, 0xFF,
    //Other Setting
    //Address    value
    0x3104, 0x01,
    0x38A0, 0x00,
    0x38A1, 0x00,
    0x38A2, 0x00,
    0x38A3, 0x00,
    0x38A8, 0x00,
    0x38A9, 0x00,
    0x38AA, 0x00,
    0x38AB, 0x00,
    0x38B0, 0x03,
    0x38B1, 0xFF,
    0x38B4, 0x03,
    0x38B5, 0xFF,
    0x38B8, 0x03,
    0x38B9, 0xFF,
    0x38BC, 0x03,
    0x38BD, 0xFF,
    0x38D0, 0x07,
    0x38D1, 0x08,
    0x38D2, 0x07,
    0x38D3, 0x08,
    0x38D8, 0x14,
    0x38E0, 0x00,
    0x38E1, 0x00,
    0x38E2, 0x00,
    0x38E3, 0x00,
    0x38E4, 0x00,
    0x38E5, 0x00,
    0x38E6, 0x00,
    0x38E7, 0x00,
    0x3B00, 0x00,
    0x3B01, 0x00,
    0x3B04, 0x00,
    0x3B05, 0x00,
    //Integration Setting
    //Address    value
    0x0202, 0x03,
    0x0203, 0xE8,
    //Gain Setting
    //Address    value
    0x0204, 0x13,
    0x0205, 0x34,
    0x020E, 0x01,
    0x020F, 0x00,
    //PDAF TYPE2 Setting
    //Address    value
    0x3103, 0x00,
    0x3422, 0x01,
    0x3423, 0xFC,
    //EAE-Bracketing Setting
    //Address    value
    0x0E00,    0x00,
    //PHASE PIX VCID Setting
    //Address    value
    0x30A4, 0x00,
    0x30A6, 0x00,
    0x30F2, 0x01,
    0x30F3, 0x01,
    //PHASE PIX data type Setting
    //Address    value
    0x30A5, 0x30,
    0x30A7, 0x30,
    //PDAF TYPE2 VCID Setting
    //Address    value
    0x30A2, 0x00,
    0x30F1, 0x01,
    //PDAF TYPE2 data type Setting
    //Address    value
    0x30A3, 0x30,
    //MIPI Global Timing Setting
    //Address    value
    0x084E, 0x00,
    0x084F, 0x09,
    0x0850, 0x00,
    0x0851, 0x09,
    0x0852, 0x00,
    0x0853, 0x0F,
    0x0854, 0x00,
    0x0855, 0x29,
    0x0858, 0x00,
    0x0859, 0x1F,
};

static void sensor_init(void)
{
    LOG_INF("sensor_init start\n");
    LOG_INF("E imx852_hw_version(%d)\n", imx852_hw_version);
    imx852_table_write_cmos_sensor(imx852_init_setting,
        sizeof(imx852_init_setting)/sizeof(kal_uint16));
    // if(imx852_hw_version != HW_CUT0_90) {
    //     imx852_table_write_cmos_sensor(imx852_init_additional_setting,
    //     sizeof(imx852_init_additional_setting)/sizeof(kal_uint16));
    // }
    set_mirror_flip(imgsensor.mirror);
    LOG_INF("sensor_init End\n");
}    /*sensor_init  */

static void preview_setting(void)
{
    LOG_INF("%s start\n", __func__);
    imx852_table_write_cmos_sensor(imx852_preview_setting,
        sizeof(imx852_preview_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
} /* preview_setting */


/*full size 30fps*/
static void capture_setting(kal_uint16 currefps)
{
    LOG_INF("%s start\n", __func__);
    imx852_table_write_cmos_sensor(imx852_capture_setting,
        sizeof(imx852_capture_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void normal_video_setting(kal_uint16 currefps)
{
    LOG_INF("%s start\n", __func__);
    imx852_table_write_cmos_sensor(imx852_normal_video_setting,
        sizeof(imx852_normal_video_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void hs_video_setting(void)
{
    LOG_INF("%s start\n", __func__);
    imx852_table_write_cmos_sensor(imx852_hs_video_setting,
        sizeof(imx852_hs_video_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void slim_video_setting(void)
{
    LOG_INF("%s start\n", __func__);
    imx852_table_write_cmos_sensor(imx852_slim_video_setting,
        sizeof(imx852_slim_video_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void custom1_setting(void)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx852_table_write_cmos_sensor(imx852_custom1_setting,
        sizeof(imx852_custom1_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void custom2_setting(void)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx852_table_write_cmos_sensor(imx852_custom2_setting,
        sizeof(imx852_custom2_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void custom3_setting(void)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx852_table_write_cmos_sensor(imx852_custom3_setting,
        sizeof(imx852_custom3_setting)/sizeof(kal_uint16));
    if (Eeprom_1ByteDataRead(0x1418, 0xA8) == 1) {
        pr_info("OTP QSC Data Valid, enable qsc register");
    } else {
        pr_info("OTP no QSC Data, close qsc register");
        write_cmos_sensor_8(0x3621, 0x00);
    }
    LOG_INF("%s end\n", __func__);
}

static void custom4_setting(void)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx852_table_write_cmos_sensor(imx852_custom4_setting,
        sizeof(imx852_custom4_setting)/sizeof(kal_uint16));
     LOG_INF("%s end\n", __func__);
}

static void custom5_setting(void)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx852_table_write_cmos_sensor(imx852_custom5_setting,
        sizeof(imx852_custom5_setting)/sizeof(kal_uint16));
     LOG_INF("%s end\n", __func__);
}

static void custom6_setting(void)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx852_table_write_cmos_sensor(imx852_custom6_setting,
        sizeof(imx852_custom6_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void custom7_setting(void)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx852_table_write_cmos_sensor(imx852_custom7_setting,
        sizeof(imx852_custom7_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void custom8_setting(void)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx852_table_write_cmos_sensor(imx852_custom8_setting,
        sizeof(imx852_custom8_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}
static void custom9_setting(void)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx852_table_write_cmos_sensor(imx852_custom9_setting,
        sizeof(imx852_custom9_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}
static void custom10_setting(void)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx852_table_write_cmos_sensor(imx852_custom10_setting,
        sizeof(imx852_custom10_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void hdr_write_double_shutter_w_gph(kal_uint16 le, kal_uint16 se, kal_bool gph)
{
    kal_uint16 realtime_fps = 0;
    kal_uint16 exposure_cnt = 0;

    if (le)
        exposure_cnt++;
    if (se) {
        exposure_cnt++;
    } else {
        se = imgsensor_info.min_shutter;
        exposure_cnt++;
    }

    le = (kal_uint16)max(imgsensor_info.min_shutter * exposure_cnt, (kal_uint32)le);
    se = (kal_uint16)max(imgsensor_info.min_shutter * exposure_cnt, (kal_uint32)se);
    if(le < se) {
        le = se;
    }

    if (le) {
        le = round_up(le/exposure_cnt, 4);
    }
    if (se) {
        se = round_up(se/exposure_cnt, 4);
    }

    spin_lock(&imgsensor_drv_lock);
    imgsensor.frame_length = max((kal_uint32)((le + se + (exposure_cnt * imgsensor_info.margin)) * exposure_cnt),
                                imgsensor.min_frame_length);
    imgsensor.frame_length = min(imgsensor.frame_length, imgsensor_info.max_frame_length);
    spin_unlock(&imgsensor_drv_lock);

    LOG_INF("E! le:0x%x, se:0x%x autoflicker_en %d frame_length %d\n",
        le, se, imgsensor.autoflicker_en, imgsensor.frame_length);

    if (gph) {
        write_cmos_sensor_8(0x0104, 0x01);
    }

    if (imgsensor.autoflicker_en) {
        realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
        if (realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(296, 0);
        else if (realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(146, 0);
        else {
            write_frame_len(imgsensor.frame_length);
        }
    } else {
        write_frame_len(imgsensor.frame_length);
    }

    // Long exposure
    write_cmos_sensor_8(0x0202, (le >> 8) & 0xFF);
    write_cmos_sensor_8(0x0203, le & 0xFF);

    // Short exposure
    write_cmos_sensor_8(0x0224, (se >> 8) & 0xFF);
    write_cmos_sensor_8(0x0225, se & 0xFF);
    if (gph) {
        write_cmos_sensor_8(0x0104, 0x00);
    }
    LOG_INF("L! le:0x%x, se:0x%x\n", le, se);
#if 0
    //read back
    le_b = ((read_cmos_sensor_8(0x0202) & 0xff) << 8) |  (read_cmos_sensor_8(0x0203) & 0xff);
    se_b = ((read_cmos_sensor_8(0x0224) & 0xff) << 8) |  (read_cmos_sensor_8(0x0225) & 0xff);
    LOG_INF("read from sensor le: 0x%x(%d), se: 0x%x(%d)", le_b, le_b, se_b, se_b);
#endif
}

static void hdr_write_double_shutter(kal_uint16 le, kal_uint16 se)
{
    LOG_INF("Fine Integ Time = %d", (read_cmos_sensor_8(0x0200) << 8) | read_cmos_sensor_8(0x0201));
    hdr_write_double_shutter_w_gph(le, se, KAL_TRUE);
}

static void hdr_write_double_gain_w_gph(kal_uint16 lgain, kal_uint16 sgain, kal_bool gph)
{
    kal_uint16 reg_lg, reg_sg;
    //kal_uint16 lg_b = 0, sg_b = 0;

    if (lgain < BASEGAIN || lgain > 32 * BASEGAIN) {
        LOG_INF("Error lgain setting");
        if (lgain < BASEGAIN)
            lgain = BASEGAIN;
        else if (lgain > 32 * BASEGAIN)
            lgain = 32 * BASEGAIN;
    }
    if (sgain < BASEGAIN || sgain > 32 * BASEGAIN) {
        LOG_INF("Error sgain setting");
        if (sgain < BASEGAIN)
            sgain = BASEGAIN;
        else if (sgain > 32 * BASEGAIN)
            sgain = 32 * BASEGAIN;
    }

    reg_lg = gain2reg(lgain);
    reg_sg = gain2reg(sgain);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.gain = reg_lg;
    spin_unlock(&imgsensor_drv_lock);
    if (gph) {
        write_cmos_sensor_8(0x0104, 0x01);
    }
    // Long Gian
    write_cmos_sensor_8(0x0204, (reg_lg>>8) & 0xFF);
    write_cmos_sensor_8(0x0205, reg_lg & 0xFF);

    // Short Gian
    write_cmos_sensor_8(0x0216, (reg_sg>>8) & 0xFF);
    write_cmos_sensor_8(0x0217, reg_sg & 0xFF);
    if (gph) {
        write_cmos_sensor_8(0x0104, 0x00);
    }
    LOG_INF("lgain:%d, reg_lg:0x%x, sgain:%d, reg_sg:0x%x\n",
        lgain, reg_lg, sgain, reg_sg);
#if 0
    //read back
    lg_b = ((read_cmos_sensor_8(0x0204) & 0xff) << 8) |  (read_cmos_sensor_8(0x0205) & 0xff);
    sg_b = ((read_cmos_sensor_8(0x0216) & 0xff) << 8) |  (read_cmos_sensor_8(0x0217) & 0xff);
    LOG_INF("read from sensor lg: 0x%x(%d), sg: 0x%x(%d)", lg_b, lg_b, sg_b, sg_b);
#endif
}
static void hdr_write_double_gain(kal_uint16 lgain, kal_uint16 sgain)
{
    hdr_write_double_gain_w_gph(lgain, sgain, KAL_TRUE);
}

static void write_sensor_QSC(void)
{
    kal_uint8 qsc_is_valid = Eeprom_1ByteDataRead(OTP_QSC_OFFSET + QSC_SIZE, IMX852_EEPROM_SLAVE_ADDRESS);
    LOG_INF("%s start write_sensor_QSC, qsc_is_valid:%d, qsc_flag:%d\n", __func__, qsc_is_valid, qsc_flag);
    if (qsc_is_valid == 1 && !qsc_flag) {
        imx852_burst_write_cmos_sensor(imx852_QSC_setting,
            sizeof(imx852_QSC_setting) / sizeof(kal_uint16));
        qsc_flag = 1;
    }
    LOG_INF("%s end\n", __func__);

}

static void write_sensor_SPC(void)
{
    kal_uint8 spc_is_valid = Eeprom_1ByteDataRead(OTP_SPC_OFFSET + SPC_SIZE, IMX852_EEPROM_SLAVE_ADDRESS);
    LOG_INF("%s start write_sensor_spc, spc_is_valid:%d, spc_flag:%d", __func__, spc_is_valid, spc_flag);
    if (spc_is_valid == 1 && !spc_flag) {
        imx852_table_write_cmos_sensor(imx852_SPC_setting,
            sizeof(imx852_SPC_setting) / sizeof(kal_uint16));
        spc_flag = 1;
    }
    LOG_INF("%s end\n", __func__);
}

static kal_uint16 read_imx852_eeprom_module(kal_uint32 addr)
{
    kal_uint16 get_byte = 0;
    char pusendcmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };

    iReadRegI2C(pusendcmd, 2, (u8 *) &get_byte, 1, EEPROM_I2C_ADDR);
    return get_byte;
}

static void read_imx852_module_data()
{
    // length   : 64
    // QR       : [8 ~ 26]
    // vcm      : [40 ~ 41]
    // lensid   : [44 ~ 45]
    // marcro   : [48 ~ 49]
    // inf      : [52 ~ 53]

    kal_uint16 idx = 0;
    kal_uint16 sn_length = 23;
    kal_uint32 sn_starAddr = 0xB0;
    kal_uint32 vcmAddr = 0x0A;
    kal_uint32 lensAddr = 0x08;
    kal_uint32 macAddr = 0x92;
    kal_uint32 infAddr = 0x94;

    memset(imx852_common_data, 0, sizeof(imx852_common_data));
    // QR
    for(idx = 0; idx < sn_length; idx++)
    {
       imx852_common_data[8 + idx] = read_imx852_eeprom_module(sn_starAddr + idx);
    }
    // Vcm ID
    imx852_common_data[40] = read_imx852_eeprom_module(vcmAddr);
    imx852_common_data[41] = read_imx852_eeprom_module(vcmAddr + 1);
    // Lens ID
    imx852_common_data[44] = read_imx852_eeprom_module(lensAddr);
    imx852_common_data[45] = read_imx852_eeprom_module(lensAddr + 1);
    // Macro
    imx852_common_data[48] = read_imx852_eeprom_module(macAddr);
    imx852_common_data[49] = read_imx852_eeprom_module(macAddr + 1);
    // Inf
    imx852_common_data[52] = read_imx852_eeprom_module(infAddr);
    imx852_common_data[53] = read_imx852_eeprom_module(infAddr + 1);

    for (idx = 0; idx < CAMERA_EEPPROM_COMDATA_LENGTH; idx = idx + 4)
    {
        pr_info("imx852 common cam data : %02x %02x %02x %02x\n",
           imx852_common_data[idx],
           imx852_common_data[idx + 1],
           imx852_common_data[idx + 2],
           imx852_common_data[idx + 3]);
    }
}

static void read_cmos_eeprom_table(kal_uint16 addr, kal_uint8 *table, kal_uint32 size)
{
    char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
    iReadRegI2C(pusendcmd, 2, (u8 *)table, size, EEPROM_I2C_ADDR);
}

static void read_cmos_eeprom_checkPD()
{
    char pusendcmd[2] = {(char)(0x15f0 >> 8), (char)(0x15f0 & 0xFF) };
    char pusendcmd1[2] = {(char)(0x19EC >> 8), (char)(0x19EC & 0xFF) };
    uint8_t proc1_flag, proc2_flag;

    iReadRegI2C(pusendcmd, 2, &proc1_flag, 1, EEPROM_I2C_ADDR);

    iReadRegI2C(pusendcmd1, 2, &proc2_flag, 1, EEPROM_I2C_ADDR);

    LOG_INF("======= [%s] ======= proc1_flag = %x, proc2_flag = %x", __func__, proc1_flag, proc2_flag);
}

static kal_uint16 read_otp_info(kal_uint8 *data)
{
    kal_uint16 addr = 0x00;
    kal_uint16 left_size = 0;

    left_size = LEFT_SIZE;
    while (left_size > 0) {
        if (left_size >= READ_EEPROM_1K) {
            read_cmos_eeprom_table(addr,data, READ_EEPROM_1K);
            left_size = left_size - READ_EEPROM_1K;
        } else {
            read_cmos_eeprom_table(addr,data, left_size);
            break;
        }
        addr = addr + READ_EEPROM_1K;
        data = data + READ_EEPROM_1K;
    }
    return 0;
}
/*************************************************************************
 * FUNCTION
 *    get_imgsensor_id
 *
 * DESCRIPTION
 *    This function get the sensor ID
 *
 * PARAMETERS
 *    *sensorID : return the sensor ID
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    static bool first_read = KAL_TRUE;
    /*sensor have two i2c address 0x34 & 0x20,
     *we should detect the module used i2c address
     */
    LOG_INF("imx852 Enter %s.", __func__);
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        // gImgEepromInfo.i4CurSensorIdx = 0;
        // gImgEepromInfo.i4CurSensorId = imgsensor_info.sensor_id;
        spin_unlock(&imgsensor_drv_lock);
        do {
            *sensor_id = ((read_cmos_sensor_8(0x0016) << 8)
                    | read_cmos_sensor_8(0x0017));
            read_cmos_eeprom_checkPD();
            if (*sensor_id == IMX852_SENSOR_ID_MUMBAI) {
                *sensor_id = imgsensor_info.sensor_id;
                LOG_INF("Read sensor id Success, write id:0x%x, id: 0x%x\n", imgsensor.i2c_write_id, *sensor_id);
                read_sensor_verison();
                if(first_read){
                    read_imx852_mumbai_SPC(imx852_SPC_setting);
                    read_imx852_mumbai_QSC(imx852_QSC_setting);
                    first_read = KAL_FALSE;
                }
                if(deviceInfo_register_value == 0x00){
                    read_imx852_module_data();
                    deviceInfo_register_value = 0x01;
                }
                if (otp_data[0] == 0) {
                    read_otp_info(otp_data);
                } else {
                    pr_err("otp data has already read");
                }
                return ERROR_NONE;
            }
            LOG_INF("Read sensor id fail, write id:0x%x, id: 0x%x\n", imgsensor.i2c_write_id, *sensor_id);
            retry--;
        } while (retry > 0);
        i++;
        retry = 2;
    }
    if (*sensor_id != imgsensor_info.sensor_id) {
        // if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF
        *sensor_id = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;
}


/*************************************************************************
 * FUNCTION
 *    open
 *
 * DESCRIPTION
 *    This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *    None
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 open(void)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    kal_uint32 sensor_id = 0;

    LOG_INF("IMX852 open start\n");
    /*sensor have two i2c address 0x6c 0x6d & 0x21 0x20,
     *we should detect the module used i2c address
     */
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            sensor_id = ((read_cmos_sensor_8(0x0016) << 8) | read_cmos_sensor_8(0x0017));
            if (sensor_id == IMX852_SENSOR_ID_MUMBAI) {
                sensor_id = imgsensor_info.sensor_id;
                LOG_INF("Read sensor id Success, write id:0x%x, id: 0x%x\n", imgsensor.i2c_write_id, sensor_id);
                break;
            }
            LOG_INF("Read sensor id fail, write id:0x%x, id: 0x%x\n", imgsensor.i2c_write_id, sensor_id);
            retry--;
        } while (retry > 0);
        i++;
        if (sensor_id == imgsensor_info.sensor_id)
            break;
        retry = 2;
    }
    if (imgsensor_info.sensor_id != sensor_id) {
        return -1;
    }
    /* initail sequence write in  */
    sensor_init();
    if(oplus_is_system_camera(OPLUS_CHECK_IS_SYSTEM_CAM)){
        write_sensor_QSC();
    }

    write_sensor_SPC();
    spin_lock(&imgsensor_drv_lock);

    imgsensor.autoflicker_en = KAL_FALSE;
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
    imgsensor.extend_frame_length_en = KAL_FALSE;
    imgsensor.fast_mode_on = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    LOG_INF("IMX852 open End\n");
    return ERROR_NONE;
} /* open */

/*************************************************************************
 * FUNCTION
 *    close
 *
 * DESCRIPTION
 *
 *
 * PARAMETERS
 *    None
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 close(void)
{
    LOG_INF("E\n");
    /* No Need to implement this function */
    streaming_control(KAL_FALSE);
    qsc_flag = 0;
    spc_flag = 0;
    return ERROR_NONE;
} /* close */

/*write AWB gain to sensor*/
// static kal_uint16 imx852_feedback_awbgain[] = {
//     0x0b90, 0x00,
//     0x0b91, 0x01,
//     0x0b92, 0x00,
//     0x0b93, 0x01,
// };
// static void feedback_awbgain(kal_uint32 r_gain, kal_uint32 b_gain)
// {
//     UINT32 r_gain_int = 0;
//     UINT32 b_gain_int = 0;

//     r_gain_int = r_gain / 522;
//     b_gain_int = b_gain / 512;

//     imx852_feedback_awbgain[1] = r_gain_int;
//     imx852_feedback_awbgain[3] = (
//         ((r_gain*100) / 512) - (r_gain_int * 100)) * 2;
//     imx852_feedback_awbgain[5] = b_gain_int;
//     imx852_feedback_awbgain[7] = (
//         ((b_gain * 100) / 512) - (b_gain_int * 100)) * 2;
//     imx852_table_write_cmos_sensor(imx852_feedback_awbgain,
//         sizeof(imx852_feedback_awbgain)/sizeof(kal_uint16));
// }

/*************************************************************************
 * FUNCTION
 * preview
 *
 * DESCRIPTION
 *    This function start the sensor preview.
 *
 * PARAMETERS
 *    *image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("imx852:Enter %s.\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);

    preview_setting();
    LOG_INF("imx852:Leave %s., w*h:%d x %d.\n", __func__, imgsensor.line_length, imgsensor.frame_length);
    mdelay(8);

    return ERROR_NONE;
} /* preview */

/*************************************************************************
 * FUNCTION
 *    capture
 *
 * DESCRIPTION
 *    This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *    None
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

    if (imgsensor.current_fps != imgsensor_info.cap.max_framerate){
        LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
                imgsensor.current_fps,
                imgsensor_info.cap.max_framerate / 10);
    }
    imgsensor.pclk = imgsensor_info.cap.pclk;
    imgsensor.line_length = imgsensor_info.cap.linelength;
    imgsensor.frame_length = imgsensor_info.cap.framelength;
    imgsensor.min_frame_length = imgsensor_info.cap.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;


    spin_unlock(&imgsensor_drv_lock);
    capture_setting(imgsensor.current_fps);
    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}    /* capture() */
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
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    normal_video_setting(imgsensor.current_fps);
    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}    /*    normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
    imgsensor.pclk = imgsensor_info.hs_video.pclk;
    /*imgsensor.video_mode = KAL_TRUE;*/
    imgsensor.line_length = imgsensor_info.hs_video.linelength;
    imgsensor.frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    /*imgsensor.current_fps = 300;*/
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    hs_video_setting();
    //set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}    /*    hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s. 720P@240FPS\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
    imgsensor.pclk = imgsensor_info.slim_video.pclk;
    /*imgsensor.video_mode = KAL_TRUE;*/
    imgsensor.line_length = imgsensor_info.slim_video.linelength;
    imgsensor.frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    /*imgsensor.current_fps = 300;*/
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    slim_video_setting();
    //set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}    /* slim_video */


static kal_uint32 custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
    imgsensor.pclk = imgsensor_info.custom1.pclk;
    imgsensor.line_length = imgsensor_info.custom1.linelength;
    imgsensor.frame_length = imgsensor_info.custom1.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom1_setting();
    set_mirror_flip(imgsensor.mirror);
    return ERROR_NONE;
}    /* custom1 */

static kal_uint32 custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
    imgsensor.pclk = imgsensor_info.custom2.pclk;
    imgsensor.line_length = imgsensor_info.custom2.linelength;
    imgsensor.frame_length = imgsensor_info.custom2.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom2_setting();

    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}    /* custom2 */

static kal_uint32 custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;
    imgsensor.pclk = imgsensor_info.custom3.pclk;
    imgsensor.line_length = imgsensor_info.custom3.linelength;
    imgsensor.frame_length = imgsensor_info.custom3.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom3.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom3_setting();
    //write_sensor_QSC();
    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}    /* custom2 */

static kal_uint32 custom4(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM4;
    imgsensor.pclk = imgsensor_info.custom4.pclk;
    imgsensor.line_length = imgsensor_info.custom4.linelength;
    imgsensor.frame_length = imgsensor_info.custom4.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom4.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom4_setting();

    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}    /* custom4 */

static kal_uint32 custom5(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM5;
    imgsensor.pclk = imgsensor_info.custom5.pclk;
    imgsensor.line_length = imgsensor_info.custom5.linelength;
    imgsensor.frame_length = imgsensor_info.custom5.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom5.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom5_setting();

    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}

static kal_uint32 custom6(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM6;
    imgsensor.pclk = imgsensor_info.custom6.pclk;
    imgsensor.line_length = imgsensor_info.custom6.linelength;
    imgsensor.frame_length = imgsensor_info.custom6.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom6.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom6_setting();

    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}    /* custom6 */

static kal_uint32 custom7(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM7;
    imgsensor.pclk = imgsensor_info.custom7.pclk;
    imgsensor.line_length = imgsensor_info.custom7.linelength;
    imgsensor.frame_length = imgsensor_info.custom7.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom7.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom7_setting();

    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}/* custom7 */

static kal_uint32 custom8(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM8;
    imgsensor.pclk = imgsensor_info.custom8.pclk;
    imgsensor.line_length = imgsensor_info.custom8.linelength;
    imgsensor.frame_length = imgsensor_info.custom8.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom8.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom8_setting();

    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}/* custom8 */

static kal_uint32 custom9(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM9;
    imgsensor.pclk = imgsensor_info.custom9.pclk;
    imgsensor.line_length = imgsensor_info.custom9.linelength;
    imgsensor.frame_length = imgsensor_info.custom9.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom9.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom9_setting();

    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}/* custom9 */

static kal_uint32 custom10(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM10;
    imgsensor.pclk = imgsensor_info.custom10.pclk;
    imgsensor.line_length = imgsensor_info.custom10.linelength;
    imgsensor.frame_length = imgsensor_info.custom10.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom10.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom10_setting();

    set_mirror_flip(imgsensor.mirror);

    return ERROR_NONE;
}/* custom8 */

static kal_uint32
get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
    LOG_INF("E\n");
    sensor_resolution->SensorFullWidth =
        imgsensor_info.cap.grabwindow_width;
    sensor_resolution->SensorFullHeight =
        imgsensor_info.cap.grabwindow_height;

    sensor_resolution->SensorPreviewWidth =
        imgsensor_info.pre.grabwindow_width;
    sensor_resolution->SensorPreviewHeight =
        imgsensor_info.pre.grabwindow_height;

    sensor_resolution->SensorVideoWidth =
        imgsensor_info.normal_video.grabwindow_width;
    sensor_resolution->SensorVideoHeight =
        imgsensor_info.normal_video.grabwindow_height;

    sensor_resolution->SensorHighSpeedVideoWidth =
        imgsensor_info.hs_video.grabwindow_width;
    sensor_resolution->SensorHighSpeedVideoHeight =
        imgsensor_info.hs_video.grabwindow_height;

    sensor_resolution->SensorSlimVideoWidth =
        imgsensor_info.slim_video.grabwindow_width;
    sensor_resolution->SensorSlimVideoHeight =
        imgsensor_info.slim_video.grabwindow_height;

    sensor_resolution->SensorCustom1Width =
        imgsensor_info.custom1.grabwindow_width;
    sensor_resolution->SensorCustom1Height =
        imgsensor_info.custom1.grabwindow_height;

    sensor_resolution->SensorCustom2Width =
        imgsensor_info.custom2.grabwindow_width;
    sensor_resolution->SensorCustom2Height =
        imgsensor_info.custom2.grabwindow_height;

    sensor_resolution->SensorCustom3Width =
        imgsensor_info.custom3.grabwindow_width;
    sensor_resolution->SensorCustom3Height =
        imgsensor_info.custom3.grabwindow_height;

    sensor_resolution->SensorCustom4Width =
        imgsensor_info.custom4.grabwindow_width;
    sensor_resolution->SensorCustom4Height =
        imgsensor_info.custom4.grabwindow_height;

    sensor_resolution->SensorCustom5Width =
        imgsensor_info.custom5.grabwindow_width;
    sensor_resolution->SensorCustom5Height =
        imgsensor_info.custom5.grabwindow_height;

    sensor_resolution->SensorCustom6Width =
        imgsensor_info.custom6.grabwindow_width;
    sensor_resolution->SensorCustom6Height =
        imgsensor_info.custom6.grabwindow_height;

    sensor_resolution->SensorCustom7Width =
        imgsensor_info.custom7.grabwindow_width;
    sensor_resolution->SensorCustom7Height =
        imgsensor_info.custom7.grabwindow_height;

    sensor_resolution->SensorCustom8Width =
        imgsensor_info.custom8.grabwindow_width;
    sensor_resolution->SensorCustom8Height =
        imgsensor_info.custom8.grabwindow_height;

    sensor_resolution->SensorCustom9Width =
        imgsensor_info.custom9.grabwindow_width;
    sensor_resolution->SensorCustom9Height =
        imgsensor_info.custom9.grabwindow_height;

    sensor_resolution->SensorCustom10Width =
        imgsensor_info.custom10.grabwindow_width;
    sensor_resolution->SensorCustom10Height =
        imgsensor_info.custom10.grabwindow_height;
        return ERROR_NONE;
} /* get_resolution */

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
               MSDK_SENSOR_INFO_STRUCT *sensor_info,
               MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("scenario_id = %d\n", scenario_id);

    sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorInterruptDelayLines = 4; /* not use */
    sensor_info->SensorResetActiveHigh = FALSE; /* not use */
    sensor_info->SensorResetDelayCount = 5; /* not use */

    sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
    sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
    sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
    sensor_info->SensorOutputDataFormat =
        imgsensor_info.sensor_output_dataformat;

    sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
    sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
    sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
    sensor_info->HighSpeedVideoDelayFrame =
        imgsensor_info.hs_video_delay_frame;
    sensor_info->SlimVideoDelayFrame =
        imgsensor_info.slim_video_delay_frame;
    sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;
    sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame;
    sensor_info->Custom3DelayFrame = imgsensor_info.custom3_delay_frame;
    sensor_info->Custom4DelayFrame = imgsensor_info.custom4_delay_frame;
    sensor_info->Custom5DelayFrame = imgsensor_info.custom5_delay_frame;
    sensor_info->Custom6DelayFrame = imgsensor_info.custom6_delay_frame;
    sensor_info->Custom7DelayFrame = imgsensor_info.custom7_delay_frame;
    sensor_info->Custom8DelayFrame = imgsensor_info.custom8_delay_frame;
    sensor_info->Custom9DelayFrame = imgsensor_info.custom9_delay_frame;
    sensor_info->Custom10DelayFrame = imgsensor_info.custom10_delay_frame;

    sensor_info->SensorMasterClockSwitch = 0; /* not use */
    sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

    sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
    sensor_info->AESensorGainDelayFrame =
        imgsensor_info.ae_sensor_gain_delay_frame;
    sensor_info->AEISPGainDelayFrame =
        imgsensor_info.ae_ispGain_delay_frame;
    sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
    sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
    sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
    sensor_info->PDAF_Support = 2;
    sensor_info->HDR_Support = HDR_SUPPORT_STAGGER_FDOL;
    sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
    sensor_info->SensorClockFreq = imgsensor_info.mclk;
    sensor_info->SensorClockDividCount = 3; /* not use */
    sensor_info->SensorClockRisingCount = 0;
    sensor_info->SensorClockFallingCount = 2; /* not use */
    sensor_info->SensorPixelClockCount = 3; /* not use */
    sensor_info->SensorDataLatchCount = 2; /* not use */

    sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->SensorWidthSampling = 0; /* 0 is default 1x */
    sensor_info->SensorHightSampling = 0; /* 0 is default 1x */
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

        sensor_info->SensorGrabStartX =
            imgsensor_info.normal_video.startx;
        sensor_info->SensorGrabStartY =
            imgsensor_info.normal_video.starty;

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
        sensor_info->SensorGrabStartX =
            imgsensor_info.slim_video.startx;
        sensor_info->SensorGrabStartY =
            imgsensor_info.slim_video.starty;

        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

        break;

    case MSDK_SCENARIO_ID_CUSTOM1:
        sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
        sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;

        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
        break;

    case MSDK_SCENARIO_ID_CUSTOM2:
        sensor_info->SensorGrabStartX = imgsensor_info.custom2.startx;
        sensor_info->SensorGrabStartY = imgsensor_info.custom2.starty;

        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.custom2.mipi_data_lp2hs_settle_dc;
        break;

    case MSDK_SCENARIO_ID_CUSTOM3:
        sensor_info->SensorGrabStartX = imgsensor_info.custom3.startx;
        sensor_info->SensorGrabStartY = imgsensor_info.custom3.starty;

        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.custom3.mipi_data_lp2hs_settle_dc;
        break;

    case MSDK_SCENARIO_ID_CUSTOM4:
        sensor_info->SensorGrabStartX = imgsensor_info.custom4.startx;
        sensor_info->SensorGrabStartY = imgsensor_info.custom4.starty;

        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.custom4.mipi_data_lp2hs_settle_dc;
        break;
    case MSDK_SCENARIO_ID_CUSTOM5:
        sensor_info->SensorGrabStartX = imgsensor_info.custom5.startx;
        sensor_info->SensorGrabStartY = imgsensor_info.custom5.starty;

        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.custom5.mipi_data_lp2hs_settle_dc;
        break;
    case MSDK_SCENARIO_ID_CUSTOM6:
        sensor_info->SensorGrabStartX = imgsensor_info.custom6.startx;
        sensor_info->SensorGrabStartY = imgsensor_info.custom6.starty;

        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.custom6.mipi_data_lp2hs_settle_dc;
        break;
    case MSDK_SCENARIO_ID_CUSTOM7:
        sensor_info->SensorGrabStartX = imgsensor_info.custom7.startx;
        sensor_info->SensorGrabStartY = imgsensor_info.custom7.starty;

        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.custom7.mipi_data_lp2hs_settle_dc;
        break;
    case MSDK_SCENARIO_ID_CUSTOM8:
        sensor_info->SensorGrabStartX = imgsensor_info.custom8.startx;
        sensor_info->SensorGrabStartY = imgsensor_info.custom8.starty;

        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.custom8.mipi_data_lp2hs_settle_dc;
        break;
    case MSDK_SCENARIO_ID_CUSTOM9:
        sensor_info->SensorGrabStartX = imgsensor_info.custom9.startx;
        sensor_info->SensorGrabStartY = imgsensor_info.custom9.starty;

        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.custom9.mipi_data_lp2hs_settle_dc;
        break;
    case MSDK_SCENARIO_ID_CUSTOM10:
        sensor_info->SensorGrabStartX = imgsensor_info.custom10.startx;
        sensor_info->SensorGrabStartY = imgsensor_info.custom10.starty;

        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.custom10.mipi_data_lp2hs_settle_dc;
        break;
    default:
        sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
        sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
        break;
    }

    return ERROR_NONE;
}    /*    get_info  */

#define FMC_GPH_START \
do { \
        write_cmos_sensor_8(0x0104, 0x01); \
        write_cmos_sensor_8(0x3010, 0x02); \
} while (0)

#define FMC_GPH_END \
do { \
        write_cmos_sensor_8(0x0104, 0x00); \
} while (0)

enum {
    SHUTTER_NE_FRM_1 = 0,
    GAIN_NE_FRM_1,
    FRAME_LEN_NE_FRM_1,
    HDR_TYPE_FRM_1,
    SHUTTER_NE_FRM_2,
    GAIN_NE_FRM_2,
    FRAME_LEN_NE_FRM_2,
    HDR_TYPE_FRM_2,
    SHUTTER_SE_FRM_1,
    GAIN_SE_FRM_1,
    SHUTTER_SE_FRM_2,
    GAIN_SE_FRM_2,
    SHUTTER_ME_FRM_1,
    GAIN_ME_FRM_1,
    SHUTTER_ME_FRM_2,
    GAIN_ME_FRM_2,
};

static kal_uint32 seamless_switch(enum MSDK_SCENARIO_ID_ENUM scenario_id, uint32_t *ae_ctrl)
{
    imgsensor.extend_frame_length_en = KAL_FALSE;
    LOG_INF("scenario_id: %d SHUTTER_NE_FRM_1: %d GAIN_NE_FRM_1: %d SHUTTER_ME_FRM_1: %d GAIN_ME_FRM_1: %d SHUTTER_SE_FRM_1: %d GAIN_SE_FRM_1: %d",
            scenario_id, ae_ctrl[SHUTTER_NE_FRM_1], ae_ctrl[GAIN_NE_FRM_1], ae_ctrl[SHUTTER_ME_FRM_1], ae_ctrl[GAIN_ME_FRM_1], ae_ctrl[SHUTTER_SE_FRM_1], ae_ctrl[GAIN_SE_FRM_1]);

    switch (scenario_id) {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
    {
        spin_lock(&imgsensor_drv_lock);
        //imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
        imgsensor.autoflicker_en = KAL_FALSE;
        imgsensor.pclk = imgsensor_info.pre.pclk;
        imgsensor.line_length = imgsensor_info.pre.linelength;
        imgsensor.frame_length = imgsensor_info.pre.framelength;
        imgsensor.min_frame_length = imgsensor_info.pre.framelength;
        spin_unlock(&imgsensor_drv_lock);

        FMC_GPH_START;
        imx852_table_write_cmos_sensor(imx852_mumbai_seamless_preview, sizeof(imx852_mumbai_seamless_preview) / sizeof(kal_uint16));
        if (ae_ctrl) {
            LOG_INF("call MSDK_SCENARIO_ID_CAMERA_PREVIEW %d %d",
                    ae_ctrl[SHUTTER_NE_FRM_1] ,ae_ctrl[GAIN_NE_FRM_1]);
            set_shutter_w_gph(ae_ctrl[SHUTTER_NE_FRM_1], KAL_FALSE);
            set_gain_w_gph(ae_ctrl[GAIN_NE_FRM_1], KAL_FALSE);
        }
        FMC_GPH_END;
    }
    break;
    case MSDK_SCENARIO_ID_CUSTOM5:
    {
        spin_lock(&imgsensor_drv_lock);
        //imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM5;
        imgsensor.autoflicker_en = KAL_FALSE;
        imgsensor.pclk = imgsensor_info.custom5.pclk;
        imgsensor.line_length = imgsensor_info.custom5.linelength;
        imgsensor.frame_length = imgsensor_info.custom5.framelength;
        imgsensor.min_frame_length = imgsensor_info.custom5.framelength;
        spin_unlock(&imgsensor_drv_lock);

        FMC_GPH_START;
        imx852_table_write_cmos_sensor(imx852_mumbai_seamless_custom5, sizeof(imx852_mumbai_seamless_custom5) / sizeof(kal_uint16));
        if (ae_ctrl) {
            LOG_INF("call MSDK_SCENARIO_ID_CUSTOM5 %d %d",
                    ae_ctrl[SHUTTER_NE_FRM_1], ae_ctrl[GAIN_NE_FRM_1]);

            set_shutter_w_gph(ae_ctrl[SHUTTER_NE_FRM_1], KAL_FALSE);
            set_gain_w_gph(ae_ctrl[GAIN_NE_FRM_1], KAL_FALSE);
        }
        FMC_GPH_END;
    }
    break;
    default:
    {
        pr_info( "error! wrong setting in set_seamless_switch = %d",scenario_id);
        return 0xff;
    }
    }
    imgsensor.fast_mode_on = KAL_TRUE;
    LOG_INF("%s success, scenario is switched to %d", __func__, scenario_id);
    return 0;
}

static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
            MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
            MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("scenario_id = %d\n", scenario_id);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.current_scenario_id = scenario_id;
    spin_unlock(&imgsensor_drv_lock);
    switch (scenario_id) {
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
    case MSDK_SCENARIO_ID_CUSTOM1:
        custom1(image_window, sensor_config_data);
        break;
    case MSDK_SCENARIO_ID_CUSTOM2:
        custom2(image_window, sensor_config_data);
        break;
    case MSDK_SCENARIO_ID_CUSTOM3:
        custom3(image_window, sensor_config_data);
        break;
    case MSDK_SCENARIO_ID_CUSTOM4:
        custom4(image_window, sensor_config_data);
        break;
    case MSDK_SCENARIO_ID_CUSTOM5:
        custom5(image_window, sensor_config_data);
        break;
    case MSDK_SCENARIO_ID_CUSTOM6:
        custom6(image_window, sensor_config_data);
        break;
    case MSDK_SCENARIO_ID_CUSTOM7:
        custom7(image_window, sensor_config_data);
        break;
    case MSDK_SCENARIO_ID_CUSTOM8:
        custom8(image_window, sensor_config_data);
        break;
    case MSDK_SCENARIO_ID_CUSTOM9:
        custom9(image_window, sensor_config_data);
        break;
    case MSDK_SCENARIO_ID_CUSTOM10:
        custom10(image_window, sensor_config_data);
        break;
    default:
        LOG_INF("Error ScenarioId setting");
        preview(image_window, sensor_config_data);
        return ERROR_INVALID_SCENARIO_ID;
    }

    return ERROR_NONE;
}    /* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
    LOG_INF("framerate = %d\n ", framerate);
    /* SetVideoMode Function should fix framerate */
    if (framerate == 0)
        /* Dynamic frame rate */
        return ERROR_NONE;
    spin_lock(&imgsensor_drv_lock);
    if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
        imgsensor.current_fps = 296;
    else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
        imgsensor.current_fps = 146;
    else
        imgsensor.current_fps = framerate;
    spin_unlock(&imgsensor_drv_lock);
    write_cmos_sensor_8(0x0104, 0x01);
    set_max_framerate(imgsensor.current_fps, 1);
    write_cmos_sensor_8(0x0104, 0x00);

    return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
    LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
    spin_lock(&imgsensor_drv_lock);
    if (enable) /*enable auto flicker*/
        imgsensor.autoflicker_en = KAL_TRUE;
    else /*Cancel Auto flick*/
        imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(
        enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
    kal_uint32 frame_length;

    LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

    switch (scenario_id) {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        frame_length = imgsensor_info.pre.pclk / framerate * 10
                / imgsensor_info.pre.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.pre.framelength)
        ? (frame_length - imgsensor_info.pre.framelength) : 0;
        imgsensor.frame_length =
            imgsensor_info.pre.framelength
            + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        if (framerate == 0)
            return ERROR_NONE;
        frame_length = imgsensor_info.normal_video.pclk /
                framerate * 10 /
                imgsensor_info.normal_video.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.normal_video.framelength)
        ? (frame_length - imgsensor_info.normal_video.framelength)
        : 0;
        imgsensor.frame_length =
            imgsensor_info.normal_video.framelength
            + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
            LOG_INF(
                "Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n"
                , framerate, imgsensor_info.cap.max_framerate/10);
            frame_length = imgsensor_info.cap.pclk / framerate * 10
                    / imgsensor_info.cap.linelength;
            spin_lock(&imgsensor_drv_lock);
                imgsensor.dummy_line =
                (frame_length > imgsensor_info.cap.framelength)
                  ? (frame_length - imgsensor_info.cap.framelength) : 0;
                imgsensor.frame_length =
                    imgsensor_info.cap.framelength
                    + imgsensor.dummy_line;
                imgsensor.min_frame_length = imgsensor.frame_length;
                spin_unlock(&imgsensor_drv_lock);
            if (imgsensor.frame_length > imgsensor.shutter) {
                write_cmos_sensor_8(0x0104, 0x01);
                set_dummy();
                write_cmos_sensor_8(0x0104, 0x00);
            }
        break;
    case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
        frame_length = imgsensor_info.hs_video.pclk / framerate * 10
                / imgsensor_info.hs_video.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.hs_video.framelength)
              ? (frame_length - imgsensor_info.hs_video.framelength)
              : 0;
        imgsensor.frame_length =
            imgsensor_info.hs_video.framelength
                + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
    case MSDK_SCENARIO_ID_SLIM_VIDEO:
        frame_length = imgsensor_info.slim_video.pclk / framerate * 10
            / imgsensor_info.slim_video.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.slim_video.framelength)
            ? (frame_length - imgsensor_info.slim_video.framelength)
            : 0;
        imgsensor.frame_length =
            imgsensor_info.slim_video.framelength
            + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
    case MSDK_SCENARIO_ID_CUSTOM1:
        frame_length = imgsensor_info.custom1.pclk / framerate * 10
                / imgsensor_info.custom1.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.custom1.framelength)
            ? (frame_length - imgsensor_info.custom1.framelength)
            : 0;
        imgsensor.frame_length =
            imgsensor_info.custom1.framelength
            + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
    case MSDK_SCENARIO_ID_CUSTOM2:
        frame_length = imgsensor_info.custom2.pclk / framerate * 10
                / imgsensor_info.custom2.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.custom2.framelength)
            ? (frame_length - imgsensor_info.custom2.framelength)
            : 0;
        imgsensor.frame_length =
            imgsensor_info.custom2.framelength
            + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
    case MSDK_SCENARIO_ID_CUSTOM3:
        //framerate = 600;
        frame_length = imgsensor_info.custom3.pclk / framerate * 10
                / imgsensor_info.custom3.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.custom3.framelength)
            ? (frame_length - imgsensor_info.custom3.framelength)
            : 0;
        imgsensor.frame_length =
            imgsensor_info.custom3.framelength
            + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
    case MSDK_SCENARIO_ID_CUSTOM4:
        frame_length = imgsensor_info.custom4.pclk / framerate * 10
                / imgsensor_info.custom4.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.custom4.framelength)
            ? (frame_length - imgsensor_info.custom4.framelength)
            : 0;
        imgsensor.frame_length =
            imgsensor_info.custom4.framelength
            + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
    case MSDK_SCENARIO_ID_CUSTOM5:
        frame_length = imgsensor_info.custom5.pclk / framerate * 10
                / imgsensor_info.custom5.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.custom5.framelength)
            ? (frame_length - imgsensor_info.custom5.framelength)
            : 0;
        imgsensor.frame_length =
            imgsensor_info.custom5.framelength
            + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
    case MSDK_SCENARIO_ID_CUSTOM6:
        frame_length = imgsensor_info.custom6.pclk / framerate * 10
                / imgsensor_info.custom6.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.custom6.framelength)
            ? (frame_length - imgsensor_info.custom6.framelength)
            : 0;
        imgsensor.frame_length =
            imgsensor_info.custom6.framelength
            + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
    case MSDK_SCENARIO_ID_CUSTOM7:
        frame_length = imgsensor_info.custom7.pclk / framerate * 10
                / imgsensor_info.custom7.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.custom7.framelength)
            ? (frame_length - imgsensor_info.custom7.framelength)
            : 0;
        imgsensor.frame_length =
            imgsensor_info.custom7.framelength
            + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
    case MSDK_SCENARIO_ID_CUSTOM8:
        frame_length = imgsensor_info.custom8.pclk / framerate * 10
                / imgsensor_info.custom8.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.custom8.framelength)
            ? (frame_length - imgsensor_info.custom8.framelength)
            : 0;
        imgsensor.frame_length =
            imgsensor_info.custom8.framelength
            + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
    case MSDK_SCENARIO_ID_CUSTOM9:
        frame_length = imgsensor_info.custom9.pclk / framerate * 10
                / imgsensor_info.custom9.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.custom9.framelength)
            ? (frame_length - imgsensor_info.custom9.framelength)
            : 0;
        imgsensor.frame_length =
            imgsensor_info.custom9.framelength
            + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
    case MSDK_SCENARIO_ID_CUSTOM10:
        frame_length = imgsensor_info.custom10.pclk / framerate * 10
                / imgsensor_info.custom10.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.custom10.framelength)
            ? (frame_length - imgsensor_info.custom10.framelength)
            : 0;
        imgsensor.frame_length =
            imgsensor_info.custom10.framelength
            + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        break;
    default:  /*coding with  preview scenario by default*/
        frame_length = imgsensor_info.pre.pclk / framerate * 10
            / imgsensor_info.pre.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line =
            (frame_length > imgsensor_info.pre.framelength)
            ? (frame_length - imgsensor_info.pre.framelength) : 0;
        imgsensor.frame_length =
            imgsensor_info.pre.framelength + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter) {
            write_cmos_sensor_8(0x0104, 0x01);
            set_dummy();
            write_cmos_sensor_8(0x0104, 0x00);
        }
        LOG_INF("error scenario_id = %d, we use preview scenario\n",
            scenario_id);
        break;
    }
    return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(
        enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
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
    case MSDK_SCENARIO_ID_CUSTOM2:
        *framerate = imgsensor_info.custom2.max_framerate;
        break;
    case MSDK_SCENARIO_ID_CUSTOM3:
        *framerate = imgsensor_info.custom3.max_framerate;
        break;
    case MSDK_SCENARIO_ID_CUSTOM4:
        *framerate = imgsensor_info.custom4.max_framerate;
        break;
    case MSDK_SCENARIO_ID_CUSTOM5:
        *framerate = imgsensor_info.custom5.max_framerate;
        break;
    case MSDK_SCENARIO_ID_CUSTOM6:
        *framerate = imgsensor_info.custom6.max_framerate;
        break;
    case MSDK_SCENARIO_ID_CUSTOM7:
        *framerate = imgsensor_info.custom7.max_framerate;
        break;
    case MSDK_SCENARIO_ID_CUSTOM8:
        *framerate = imgsensor_info.custom8.max_framerate;
        break;
    case MSDK_SCENARIO_ID_CUSTOM9:
        *framerate = imgsensor_info.custom9.max_framerate;
        break;
    case MSDK_SCENARIO_ID_CUSTOM10:
        *framerate = imgsensor_info.custom10.max_framerate;
        break;
    default:
        break;
    }

    return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_uint8 modes, struct SET_SENSOR_PATTERN_SOLID_COLOR *pTestpatterndata)
{
    pr_debug("set_test_pattern enum: %d\n", modes);
    if (modes) {
           switch(modes) {
           case 5:
               write_cmos_sensor_8(0x020E, 0x00);
               write_cmos_sensor_8(0x0218, 0x00);
               write_cmos_sensor_8(0x3015, 0x00);
               break;
           default:
               write_cmos_sensor_8(0x0601, modes);
               break;
           }
       } else if (imgsensor.test_pattern) {
           write_cmos_sensor_8(0x0601, 0x00); /*No pattern*/
           write_cmos_sensor_8(0x020E, 0x01);
           write_cmos_sensor_8(0x0218, 0x01);
           write_cmos_sensor_8(0x3015, 0x40);
       }

    spin_lock(&imgsensor_drv_lock);
    imgsensor.test_pattern = modes;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
                 UINT8 *feature_para, UINT32 *feature_para_len)
{
    UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
    UINT16 *feature_data_16 = (UINT16 *) feature_para;
    UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
    UINT32 *feature_data_32 = (UINT32 *) feature_para;
    unsigned long long *feature_data = (unsigned long long *) feature_para;
    /* unsigned long long *feature_return_para
     *  = (unsigned long long *) feature_para;
     */
    uint32_t *pAeCtrls;
    uint32_t *pScenarios;
    struct SET_PD_BLOCK_INFO_T *PDAFinfo;
    struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    //struct SENSOR_VC_INFO_STRUCT *pvcinfo;
    /* SET_SENSOR_AWB_GAIN *pSetSensorAWB
     *  = (SET_SENSOR_AWB_GAIN *)feature_para;
     */
    struct SENSOR_VC_INFO_STRUCT *pvcinfo;
    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data
        = (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

    LOG_INF("feature_id = %d\n", feature_id);
    switch (feature_id) {
    case SENSOR_FEATURE_SET_SENSOR_OTP:
        {
            enum IMGSENSOR_RETURN ret = IMGSENSOR_RETURN_SUCCESS;
            LOG_INF("SENSOR_FEATURE_SET_SENSOR_OTP\n");
            // ret = Eeprom_CallWriteService((ACDK_SENSOR_ENGMODE_STEREO_STRUCT *)(feature_para));
            if (ret == IMGSENSOR_RETURN_SUCCESS)
                return ERROR_NONE;
            else
                return ERROR_MSDK_IS_ACTIVATED;
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
    case SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE:
        *(MINT32 *)(signed long)(*(feature_data + 1)) = -1345000;
        LOG_INF("exporsure");
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
        case MSDK_SCENARIO_ID_CUSTOM2:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.custom2.pclk;
                break;
        case MSDK_SCENARIO_ID_CUSTOM3:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.custom3.pclk;
                break;
        case MSDK_SCENARIO_ID_CUSTOM4:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.custom4.pclk;
                break;
        case MSDK_SCENARIO_ID_CUSTOM5:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.custom5.pclk;
                break;
        case MSDK_SCENARIO_ID_CUSTOM6:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.custom6.pclk;
                break;
        case MSDK_SCENARIO_ID_CUSTOM7:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.custom7.pclk;
                break;
        case MSDK_SCENARIO_ID_CUSTOM8:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.custom8.pclk;
                break;
        case MSDK_SCENARIO_ID_CUSTOM9:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.custom9.pclk;
                break;
        case MSDK_SCENARIO_ID_CUSTOM10:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.custom10.pclk;
                break;
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        default:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.pre.pclk;
                break;
        }
        break;
    case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO: {
        MUINT32 ratio = 1;

        if (*(feature_data + 2) & SENSOR_GET_LINELENGTH_FOR_READOUT) {
            //ratio = get_exp_cnt_by_scenario((*feature_data));
        }
        switch (*feature_data) {
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.cap.framelength << 16)
                                 + (ratio * imgsensor_info.cap.linelength);
                break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.normal_video.framelength << 16)
                                + (ratio * imgsensor_info.normal_video.linelength);
                break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.hs_video.framelength << 16)
                                 + (ratio * imgsensor_info.hs_video.linelength);
                break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.slim_video.framelength << 16)
                                 + (ratio * imgsensor_info.slim_video.linelength);
                break;
        case MSDK_SCENARIO_ID_CUSTOM1:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.custom1.framelength << 16)
                                 + (ratio * imgsensor_info.custom1.linelength);
                break;
        case MSDK_SCENARIO_ID_CUSTOM2:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.custom2.framelength << 16)
                                 + (ratio * imgsensor_info.custom2.linelength);
                break;
        case MSDK_SCENARIO_ID_CUSTOM3:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.custom3.framelength << 16)
                                 + (ratio * imgsensor_info.custom3.linelength);
                break;
        case MSDK_SCENARIO_ID_CUSTOM4:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.custom4.framelength << 16)
                                 + (ratio * imgsensor_info.custom4.linelength);
                break;
        case MSDK_SCENARIO_ID_CUSTOM5:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.custom5.framelength << 16)
                                 + (ratio * imgsensor_info.custom5.linelength);
                break;
        case MSDK_SCENARIO_ID_CUSTOM6:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.custom6.framelength << 16)
                                 + (ratio * imgsensor_info.custom6.linelength);
                break;
        case MSDK_SCENARIO_ID_CUSTOM7:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.custom7.framelength << 16)
                                 + (ratio * imgsensor_info.custom7.linelength);
                break;
        case MSDK_SCENARIO_ID_CUSTOM8:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.custom8.framelength << 16)
                                 + (ratio * imgsensor_info.custom8.linelength);
                break;
        case MSDK_SCENARIO_ID_CUSTOM9:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.custom9.framelength << 16)
                                 + (ratio * imgsensor_info.custom9.linelength);
                break;
        case MSDK_SCENARIO_ID_CUSTOM10:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.custom10.framelength << 16)
                                 + (ratio * imgsensor_info.custom10.linelength);
                break;
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        default:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.pre.framelength << 16)
                                 + (ratio * imgsensor_info.pre.linelength);
                break;
        }
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
         /* night_mode((BOOL) *feature_data); */
        break;
    case SENSOR_FEATURE_CHECK_MODULE_ID:
        *feature_return_para_32 = imgsensor_info.module_id;
        break;
    case SENSOR_FEATURE_SET_GAIN:
        set_gain((UINT16) *feature_data);
        break;
    case SENSOR_FEATURE_SET_FLASHLIGHT:
        break;
    case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
        break;
    case SENSOR_FEATURE_SET_REGISTER:
        write_cmos_sensor_8(sensor_reg_data->RegAddr,
                    sensor_reg_data->RegData);
        break;
    case SENSOR_FEATURE_GET_REGISTER:
        sensor_reg_data->RegData =
            read_cmos_sensor_8(sensor_reg_data->RegAddr);
        break;
    case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
        /*get the lens driver ID from EEPROM
         * or just return LENS_DRIVER_ID_DO_NOT_CARE
         * if EEPROM does not exist in camera module.
         */
        *feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
        *feature_para_len = 4;
        break;
    case SENSOR_FEATURE_SET_VIDEO_MODE:
        set_video_mode(*feature_data);
        break;
    case SENSOR_FEATURE_CHECK_SENSOR_ID:
        get_imgsensor_id(feature_return_para_32);
        break;
    case SENSOR_FEATURE_GET_SENSOR_OTP_ALL:
    {
        memcpy(feature_return_para_32, (UINT32 *)otp_data, sizeof(otp_data));
        break;
    }
    case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
        set_auto_flicker_mode((BOOL)*feature_data_16,
                      *(feature_data_16+1));
        break;
    case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
         set_max_framerate_by_scenario(
                (enum MSDK_SCENARIO_ID_ENUM)*feature_data,
                *(feature_data+1));
        break;
    case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
         get_default_framerate_by_scenario(
                (enum MSDK_SCENARIO_ID_ENUM)*(feature_data),
                (MUINT32 *)(uintptr_t)(*(feature_data+1)));
        break;
    case SENSOR_FEATURE_GET_PDAF_DATA:
        LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
        break;
    case SENSOR_FEATURE_SET_TEST_PATTERN:
        set_test_pattern_mode((UINT8)*feature_data, (struct SET_SENSOR_PATTERN_SOLID_COLOR *)(feature_data+1));
        break;
    case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
        /* for factory mode auto testing */
        *feature_return_para_32 = imgsensor_info.checksum_value;
        *feature_para_len = 4;
        break;
    case SENSOR_FEATURE_SET_FRAMERATE:
        LOG_INF("current fps :%d\n", (UINT32)*feature_data_32);
        spin_lock(&imgsensor_drv_lock);
        imgsensor.current_fps = *feature_data_32;
        spin_unlock(&imgsensor_drv_lock);
        break;
    case SENSOR_FEATURE_SET_HDR:
        LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data_32);
        spin_lock(&imgsensor_drv_lock);
        imgsensor.ihdr_mode = *feature_data_32;
        spin_unlock(&imgsensor_drv_lock);
        break;
    case SENSOR_FEATURE_GET_CROP_INFO:
        LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
            (UINT32)*feature_data);
        wininfo =
    (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

        switch (*feature_data_32) {
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            memcpy((void *)wininfo,
                (void *)&imgsensor_winsize_info[1],
                sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            memcpy((void *)wininfo,
                (void *)&imgsensor_winsize_info[2],
                sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            memcpy((void *)wininfo,
            (void *)&imgsensor_winsize_info[3],
            sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            memcpy((void *)wininfo,
            (void *)&imgsensor_winsize_info[4],
            sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case MSDK_SCENARIO_ID_CUSTOM1:
            memcpy((void *)wininfo,
            (void *)&imgsensor_winsize_info[5],
            sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            memcpy((void *)wininfo,
            (void *)&imgsensor_winsize_info[6],
            sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            memcpy((void *)wininfo,
            (void *)&imgsensor_winsize_info[7],
            sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            memcpy((void *)wininfo,
            (void *)&imgsensor_winsize_info[8],
            sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case MSDK_SCENARIO_ID_CUSTOM5:
            memcpy((void *)wininfo,
            (void *)&imgsensor_winsize_info[9],
            sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case MSDK_SCENARIO_ID_CUSTOM6:
            memcpy((void *)wininfo,
            (void *)&imgsensor_winsize_info[10],
            sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case MSDK_SCENARIO_ID_CUSTOM7:
            memcpy((void *)wininfo,
            (void *)&imgsensor_winsize_info[11],
            sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case MSDK_SCENARIO_ID_CUSTOM8:
            memcpy((void *)wininfo,
            (void *)&imgsensor_winsize_info[12],
            sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case MSDK_SCENARIO_ID_CUSTOM9:
            memcpy((void *)wininfo,
            (void *)&imgsensor_winsize_info[13],
            sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case MSDK_SCENARIO_ID_CUSTOM10:
            memcpy((void *)wininfo,
            (void *)&imgsensor_winsize_info[14],
            sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        default:
            memcpy((void *)wininfo,
            (void *)&imgsensor_winsize_info[0],
            sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        }
        break;
    case SENSOR_FEATURE_GET_PDAF_INFO:
        LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n",
            (UINT16) *feature_data);
       PDAFinfo = (struct SET_PD_BLOCK_INFO_T *)
            (uintptr_t)(*(feature_data+1));

        switch (*feature_data) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CUSTOM1:
        case MSDK_SCENARIO_ID_CUSTOM2:
            memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info,
                sizeof(struct SET_PD_BLOCK_INFO_T));
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info_video,
                sizeof(struct SET_PD_BLOCK_INFO_T));
            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info_custom4,
                sizeof(struct SET_PD_BLOCK_INFO_T));
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info_custom3,
                sizeof(struct SET_PD_BLOCK_INFO_T));
            break;
        default:
            break;
        }
        break;
    case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
        LOG_INF(
        "SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%d\n",
            (UINT16) *feature_data);
        /*PDAF capacity enable or not, 2p8 only full size support PDAF*/
        switch (*feature_data) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
        case MSDK_SCENARIO_ID_CUSTOM1:
        case MSDK_SCENARIO_ID_CUSTOM2:
        case MSDK_SCENARIO_ID_CUSTOM3:
        case MSDK_SCENARIO_ID_CUSTOM4:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
        case MSDK_SCENARIO_ID_CUSTOM5:
        case MSDK_SCENARIO_ID_CUSTOM6:
        case MSDK_SCENARIO_ID_CUSTOM7:
        case MSDK_SCENARIO_ID_CUSTOM8:
        case MSDK_SCENARIO_ID_CUSTOM9:
        case MSDK_SCENARIO_ID_CUSTOM10:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
            break;
        default:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
            break;
        }
        break;
    case SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY:
        /*
            HDR_NONE = 0,
            HDR_RAW = 1,
            HDR_CAMSV = 2,
            HDR_RAW_ZHDR = 9,
            HDR_MultiCAMSV = 10,
            HDR_RAW_STAGGER_2EXP = 0xB,
            HDR_RAW_STAGGER_MIN = HDR_RAW_STAGGER_2EXP,
            HDR_RAW_STAGGER_3EXP = 0xC,
            HDR_RAW_STAGGER_MAX = HDR_RAW_STAGGER_3EXP,
         */
        switch (*feature_data) {
        case MSDK_SCENARIO_ID_CUSTOM4:
            *(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0xb;
            break;
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
        default:
            *(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0x0;
            break;
        }
        LOG_INF("SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY scenarioId:%llu, HDR:%llu\n",
            *feature_data, *(feature_data+1));
        break;
    case SENSOR_FEATURE_GET_VC_INFO:
         LOG_INF("SENSOR_FEATURE_GET_VC_INFO %d\n",
            (UINT16) *feature_data);
        pvcinfo =
        (struct SENSOR_VC_INFO_STRUCT *) (uintptr_t) (*(feature_data + 1));

        switch (*feature_data) {
            case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            case MSDK_SCENARIO_ID_CUSTOM1:
            case MSDK_SCENARIO_ID_CUSTOM2:
                memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[0],
                       sizeof(struct SENSOR_VC_INFO_STRUCT));
                break;
            case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            case MSDK_SCENARIO_ID_SLIM_VIDEO:
                memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[2],
                       sizeof(struct SENSOR_VC_INFO_STRUCT));
                break;
            case MSDK_SCENARIO_ID_CUSTOM4:
                memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[4],
                       sizeof(struct SENSOR_VC_INFO_STRUCT));
                break;
            case MSDK_SCENARIO_ID_CUSTOM3:
                memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[3],
                       sizeof(struct SENSOR_VC_INFO_STRUCT));
                break;
            default:
                break;
            }
        break;
        // pvcinfo2 = (struct SENSOR_VC_INFO2_STRUCT *) (uintptr_t) (*(feature_data + 1));
        //  get_vc_info_2(pvcinfo2, *feature_data_32);
    case SENSOR_FEATURE_GET_STAGGER_TARGET_SCENARIO:
        if (*feature_data == MSDK_SCENARIO_ID_VIDEO_PREVIEW) {
            switch (*(feature_data + 1)) {
            case HDR_RAW_STAGGER_2EXP:
                *(feature_data + 2) = MSDK_SCENARIO_ID_CUSTOM4;//custom4 was the 2 exp mode for preview mode
                break;
            default:
                break;
            }
        }else if (*feature_data == MSDK_SCENARIO_ID_CUSTOM4) {
            switch (*(feature_data + 1)) {
            case HDR_NONE:
                *(feature_data + 2) = MSDK_SCENARIO_ID_VIDEO_PREVIEW;//normal_video mode for video preview mode
                break;
            default:
                break;
            }
        }
        break;
    case SENSOR_FEATURE_GET_STAGGER_MAX_EXP_TIME:
        if (*feature_data == MSDK_SCENARIO_ID_CUSTOM4) {
            switch (*(feature_data + 1)) {
            case VC_STAGGER_NE:
                *(feature_data + 2) = 32757;   //need repare
                break;
            case VC_STAGGER_ME:
                *(feature_data + 2) = 32757;
                break;
            case VC_STAGGER_SE:
                *(feature_data + 2) = 32757;
                break;
            default:
                *(feature_data + 2) = 32757;
                break;
            }
        } else {
            *(feature_data + 2) = 0;
        }
        break;
    case SENSOR_FEATURE_SET_HDR_SHUTTER://for 2EXP
        LOG_INF("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",
            (UINT16) *feature_data, (UINT16) *(feature_data + 1));
        // implement write shutter for NE/SE
        hdr_write_double_shutter((UINT16)*feature_data,
            (UINT16)*(feature_data+1));
        break;
    case SENSOR_FEATURE_SET_DUAL_GAIN://for 2EXP
        LOG_INF("SENSOR_FEATURE_SET_DUAL_GAIN LE=%d, SE=%d\n",
            (UINT16) *feature_data, (UINT16) *(feature_data + 1));
        // implement write gain for NE/SE
        hdr_write_double_gain((UINT16)*feature_data,
            (UINT16)*(feature_data+1));
        break;
    case SENSOR_FEATURE_SET_PDAF:
        LOG_INF("PDAF mode :%d\n", *feature_data_16);
        imgsensor.pdaf_mode = *feature_data_16;
        break;
    case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
        set_shutter_frame_length((UINT16) (*feature_data),
                    (UINT16) (*(feature_data + 1)), (BOOL) (*(feature_data + 2)));
        break;
    case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
        LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
            (UINT16)*feature_data,
            (UINT16)*(feature_data+1),
            (UINT16)*(feature_data+2));
        break;
    case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
        LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
        streaming_control(KAL_FALSE);
        break;
    case SENSOR_FEATURE_SET_STREAMING_RESUME:
        LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n",
            *feature_data);
        if (*feature_data != 0)
            set_shutter(*feature_data);
        streaming_control(KAL_TRUE);
        break;
    case SENSOR_FEATURE_GET_BINNING_TYPE:
        switch (*(feature_data + 1)) {
        //case MSDK_SCENARIO_ID_CUSTOM3:
        case MSDK_SCENARIO_ID_CUSTOM5:
            *feature_return_para_32 = 1; /*full size*/
            break;
        default:
            *feature_return_para_32 = 1; /*BINNING_AVERAGED*/
            break;
        }
        pr_debug("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
            *feature_return_para_32);
        *feature_para_len = 4;

        break;
    case SENSOR_FEATURE_GET_EEPROM_COMDATA:
        memcpy(feature_return_para_32, imx852_common_data, CAMERA_EEPPROM_COMDATA_LENGTH);
        *feature_para_len = CAMERA_EEPPROM_COMDATA_LENGTH;
        break;
    case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
    {
        switch (*feature_data) {
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
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.slim_video.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM1:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom1.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom2.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom3.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom4.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM5:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom5.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM6:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom6.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM7:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom7.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM8:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom8.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM9:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom9.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM10:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom10.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        default:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.pre.mipi_pixel_rate;
            break;
        }
    }
    break;

    case SENSOR_FEATURE_SET_AWB_GAIN:
        /* modify to separate 3hdr and remosaic */
        // if (imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM3) {
        //     /*write AWB gain to sensor*/
        //     feedback_awbgain((UINT32)*(feature_data_32 + 1),
        //             (UINT32)*(feature_data_32 + 2));
        // }
		break;
    case SENSOR_FEATURE_SET_LSC_TBL:
        break;
    case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
        *(feature_data + 1) = 1; /* margin info by scenario */
        *(feature_data + 2) = imgsensor_info.margin;
        break;
    /*case SENSOR_FEATURE_SET_SEAMLESS_EXTEND_FRAME_LENGTH:
        pr_info("extend_frame_len %d\n", *feature_data);
        extend_frame_length((MUINT32) *feature_data);
        pr_info("extend_frame_len done %d\n", *feature_data);
        break;*/
    case SENSOR_FEATURE_SEAMLESS_SWITCH:
    {
        if ((feature_data + 1) != NULL) {
            pAeCtrls = (MUINT32 *)((uintptr_t)(*(feature_data + 1)));
        } else {
            pr_debug("warning! no ae_ctrl input");
        }
        if (feature_data == NULL) {
            pr_info("error! input scenario is null!");
            return ERROR_INVALID_SCENARIO_ID;
        }
        LOG_INF("call seamless_switch");
        seamless_switch((*feature_data), pAeCtrls);
    }
    break;
    case SENSOR_FEATURE_GET_SEAMLESS_SCENARIOS:
        if ((feature_data + 1) != NULL) {
            pScenarios = (MUINT32 *)((uintptr_t)(*(feature_data + 1)));
        } else {
            pr_info("input pScenarios vector is NULL!\n");
            return ERROR_INVALID_SCENARIO_ID;
        }
        switch (*feature_data) {
        case MSDK_SCENARIO_ID_CUSTOM5:
            *pScenarios = MSDK_SCENARIO_ID_CAMERA_PREVIEW;
            break;
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            *pScenarios = MSDK_SCENARIO_ID_CUSTOM5;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
        case MSDK_SCENARIO_ID_CUSTOM1:
        case MSDK_SCENARIO_ID_CUSTOM2:
        case MSDK_SCENARIO_ID_CUSTOM3:
        case MSDK_SCENARIO_ID_CUSTOM4:
        case MSDK_SCENARIO_ID_CUSTOM6:
        case MSDK_SCENARIO_ID_CUSTOM7:
        case MSDK_SCENARIO_ID_CUSTOM8:
        case MSDK_SCENARIO_ID_CUSTOM9:
        case MSDK_SCENARIO_ID_CUSTOM10:
        default:
            *pScenarios = 0xff;
            break;
        }
        pr_debug("SENSOR_FEATURE_GET_SEAMLESS_SCENARIOS %llu %d\n",
        *feature_data, *pScenarios);
        break;
    case SENSOR_FEATURE_GET_SEAMLESS_SYSTEM_DELAY:
        {
            int i;
            *(feature_data + 2) = 0;
            for (i = 0; i < sizeof(seamless_sys_delays) / sizeof(struct SEAMLESS_SYS_DELAY); i++) {
                if (*feature_data == seamless_sys_delays[i].source_scenario &&
                    *(feature_data + 1) == seamless_sys_delays[i].target_scenario) {
                    *(feature_data + 2) = seamless_sys_delays[i].sys_delay;
                    break;
                }
            }
        }
		break;

    default:
        break;
    }

    return ERROR_NONE;
} /* feature_control() */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
    open,
    get_info,
    get_resolution,
    feature_control,
    control,
    close
};

UINT32 IMX852_MIPI_RAW_MUMBAI_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc != NULL)
        *pfFunc = &sensor_func;
    return ERROR_NONE;
} /* IMX852_MIPI_RAW_SensorInit */

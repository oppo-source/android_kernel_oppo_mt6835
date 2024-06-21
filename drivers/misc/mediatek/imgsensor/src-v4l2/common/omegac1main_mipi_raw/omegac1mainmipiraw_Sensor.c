// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 omegac1mainmipiraw_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include "omegac1mainmipiraw_Sensor.h"

#define PFX "omegac1main_camera_sensor"
#define LOG_INF(format, args...) pr_err(PFX "[%s] " format, __func__, ##args)

#define OMEGAC1MAIN_EEPROM_READ_ID	(0xA1)
#define OMEGAC1MAIN_EEPROM_WRITE_ID	(0xA0)

#ifdef  EEPROM_WRITE_DATA_MAX_LENGTH
#undef  EEPROM_WRITE_DATA_MAX_LENGTH
#endif
#define EEPROM_WRITE_DATA_MAX_LENGTH      (64)
#define OMEGAC1MAIN_STEREO_MT_START_ADDR  (0x358B)
#define OMEGAC1MAIN_STEREO_MW_START_ADDR  (0x2EF0)
#define OMEGAC1MAIN_AESYNC_START_ADDR     (0x3C26)
#define OMEGAC1MAIN_EEPROM_LOCK_REGISTER  (0xE000)

#define OTP_SIZE              (0x8000)
#define OTP_QSC_VALID_ADDR    (0x1D51)

#define OMEGAC1MAIN_UNIQUE_SENSOR_ID         (0x0A17)
#define OMEGAC1MAIN_UNIQUE_SENSOR_ID_LENGHT  (11)

static kal_uint8 otp_data_checksum[OTP_SIZE] = {0};
static void set_sensor_cali(void *arg);
static int get_sensor_temperature(void *arg);
static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
static void omegac1main_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void omegac1main_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void omegac1main_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void omegac1main_get_eeprom_comdata(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void omegac1main_set_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void omegac1main_get_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void omegac1main_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
//static void omegac1main_get_unique_sensorid(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void omegac1main_get_min_shutter_by_scenario_adapter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void omegac1main_set_awb_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static int open(struct subdrv_ctx *ctx);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt);
static void get_sensor_cali(void* arg);
static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size);

static struct oplus_get_eeprom_common_data omegac1main_eeprom_common_data = {0};

//static BYTE omegac1main_unique_id[OMEGAC1MAIN_UNIQUE_SENSOR_ID_LENGHT] = { 0 };

/* Normal(Qbin) to Normal(Qbin) */
/* Normal(Qbin) to 2DOL(Qbin) */
static void comp_mode_tran_time_cal1(struct subdrv_ctx *ctx, u32 pre_scenario_id, u32* prsh);
typedef void (*cal_comp_mode_tran_time)(struct subdrv_ctx *ctx, u32 pre_scenario_id, u32* prsh);

struct comp_mode_tran_time_params {
	u8 enable;
	u32 clock_vtpxck;
	cal_comp_mode_tran_time cal_fn;
};
static struct comp_mode_tran_time_params omegac1main_comp_params[SENSOR_SCENARIO_ID_MAX] = {
	{ .enable = 0, }, /*pre*/
	{ .enable = 1, .clock_vtpxck = 1884, .cal_fn = comp_mode_tran_time_cal1, }, /*cap*/
	{ .enable = 0, }, /*vid*/
	{ .enable = 0, }, /*hvid*/
	{ .enable = 0, }, /*svid*/
	{ .enable = 0, }, /*cus1*/
	{ .enable = 0, }, /*cus2*/
	{ .enable = 0, }, /*csu3*/
	{ .enable = 0, }, /*cus4*/
	{ .enable = 0, }, /*cus5*/
	{ .enable = 0, }, /*cus6*/
	{ .enable = 1, .clock_vtpxck = 1404, .cal_fn = comp_mode_tran_time_cal1, }, /*cus7*/
	{ .enable = 0, }, /*cus8*/
	{ .enable = 0, }, /*cus9*/
	{ .enable = 0, }, /*cus10*/
	{ .enable = 0, }, /*cus11*/
	{ .enable = 0, }, /*cus12*/
	{ .enable = 0, }, /*cus13*/
};

static struct eeprom_map_info omegac1main_eeprom_info[] = {
	{ EEPROM_META_MODULE_ID, 0x0000, 0x0010, 0x0011, 2, true },
	{ EEPROM_META_SENSOR_ID, 0x0006, 0x0010, 0x0011, 2, true },
	{ EEPROM_META_LENS_ID, 0x0008,0x0010, 0x0011, 2, true },
	{ EEPROM_META_VCM_ID, 0x000A, 0x0010, 0x0011, 2, true },
	{ EEPROM_META_MIRROR_FLIP, 0x000E, 0x0010, 0x0011, 1, true },
	{ EEPROM_META_MODULE_SN, 0x00B0, 0x00C7, 0x00C8, 23, true },
	{ EEPROM_META_AF_CODE, 0x0092, 0x009A, 0x009B, 6, true },
	{ EEPROM_META_AF_FLAG, 0x009A, 0x009A, 0x009B, 1, true },
	{ EEPROM_META_STEREO_DATA, 0, 0, 0, 0, false },
	{ EEPROM_META_STEREO_MW_MAIN_DATA, OMEGAC1MAIN_STEREO_MW_START_ADDR, 0x3589, 0x358A, CALI_DATA_MASTER_LENGTH, false },
	{ EEPROM_META_STEREO_MT_MAIN_DATA, OMEGAC1MAIN_STEREO_MT_START_ADDR, 0x3c24, 0x3c25, CALI_DATA_MASTER_LENGTH, false },
	{ EEPROM_META_STEREO_MT_MAIN_DATA_105CM, 0, 0, 0, 0, false },
};

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, omegac1main_set_test_pattern},
	{SENSOR_FEATURE_SEAMLESS_SWITCH, omegac1main_seamless_switch},
	{SENSOR_FEATURE_CHECK_SENSOR_ID, omegac1main_check_sensor_id},
	{SENSOR_FEATURE_GET_EEPROM_COMDATA, omegac1main_get_eeprom_comdata},
	{SENSOR_FEATURE_SET_SENSOR_OTP, omegac1main_set_eeprom_calibration},
	{SENSOR_FEATURE_GET_EEPROM_STEREODATA, omegac1main_get_eeprom_calibration},
	{SENSOR_FEATURE_GET_SENSOR_OTP_ALL, omegac1main_get_otp_checksum_data},
//	{SENSOR_FEATURE_GET_UNIQUE_SENSORID, omegac1main_get_unique_sensorid},
	{SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO, omegac1main_get_min_shutter_by_scenario_adapter},
	{SENSOR_FEATURE_SET_AWB_GAIN, omegac1main_set_awb_gain},
};

static struct eeprom_info_struct eeprom_info[] = {
	{
		.header_id = 0x01480126,//cal_layout_table
		.addr_header_id = 0x00000006,//cc
		.i2c_write_id = 0xA0,//cc

		.qsc_support = TRUE,
		.qsc_size = 3072,
		.addr_qsc = 0x1150,//QSC_EEPROM_ADDR
		.sensor_reg_addr_qsc = 0xC800,//QSC_OTP_ADDR
	},
};
static int g_need_addi_setting = 0;

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX = 0,
	.i4PitchY = 0,
	.i4PairNum = 0,
	.i4SubBlkW = 0,
	.i4SubBlkH = 0,
	.i4PosL = {{0, 0} },
	.i4PosR = {{0, 0} },
	.i4BlockNumX = 0,
	.i4BlockNumY = 0,
	.i4LeFirst = 0,
	.i4Crop = {
		{0, 0}, {0, 0}, {0, 384}, {0, 384}, {0, 384},
		{0, 384}, {0, 0}, {0, 0}, {0, 384}, {0, 0}, {0, 384},
		{0, 384}, {0, 384}, {0, 0}, {0, 0}, {1064, 798},
		{0, 384}, {0, 384}, {0, 384}
	},
	.iMirrorFlip = 0,
	.i4FullRawW = 4096,
	.i4FullRawH = 3072,
	.i4VCPackNum = 1,
	.i4ModeIndex = 0x2,
	.sPDMapInfo[0] = {
		.i4PDPattern = 1,//all-pd
		.i4BinFacX = 2,
		.i4BinFacY = 4,
		.i4PDRepetition = 0,
		.i4PDOrder = {1}, //R=1, L=0
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_v2h2 = {
	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX = 0,
	.i4PitchY = 0,
	.i4PairNum = 0,
	.i4SubBlkW = 0,
	.i4SubBlkH = 0,
	.i4PosL = {{0, 0} },
	.i4PosR = {{0, 0} },
	.i4BlockNumX = 0,
	.i4BlockNumY = 0,
	.i4LeFirst = 0,
	.i4Crop = {
		{0, 0}, {0, 0}, {0, 384}, {0, 384}, {0, 192},
		{0, 192}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 384},
		{320, 240}, {0, 384}, {0, 0},{0, 0},{0, 0},{0, 0},{0, 0},{0, 192}
	},
	.iMirrorFlip = 3,
	.i4FullRawW = 2048,
	.i4FullRawH = 1536,
	.i4VCPackNum = 1,
	.i4ModeIndex = 0x2,
	.sPDMapInfo[0] = {
		.i4PDPattern = 1,//all-pd
		.i4BinFacX = 2,
		.i4BinFacY = 4,
		.i4PDRepetition = 0,
		.i4PDOrder = {1}, //R=1, L=0
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_full = {
	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX = 0,
	.i4PitchY = 0,
	.i4PairNum = 0,
	.i4SubBlkW = 0,
	.i4SubBlkH = 0,
	.i4PosL = {{0, 0} },
	.i4PosR = {{0, 0} },
	.i4BlockNumX = 0,
	.i4BlockNumY = 0,
	.i4LeFirst = 0,
	.i4Crop = {
		{0, 0}, {0, 0}, {0, 384}, {0, 384}, {0, 192},
		{0, 384}, {0, 0}, {2048, 1536}, {0, 0}, {2048, 1536}, {0, 384},
		{0, 384}, {0, 384}, {0, 0}
	},
	.iMirrorFlip = 3,
	.i4FullRawW = 8192,
	.i4FullRawH = 6144,
	.i4VCPackNum = 1,
	.i4ModeIndex = 0x4,
	.sPDMapInfo[0] = {
		.i4PDPattern = 1,//all-pd
		.i4BinFacX = 4,
		.i4BinFacY = 2,
		.i4PDRepetition = 0,
		.i4PDOrder = {1}, //R=1, L=0
	},
};


static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0c00,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x1000,
			.vsize = 0x0300,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 4096,
			.vsize = 768,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
    {
        .bus.csi2 = {
            .channel = 0,  // vc-id
            .data_type = 0x2b,  // dt
            .hsize = 0x1000,
            .vsize = 0x0900,
            .user_data_desc = VC_STAGGER_NE,
        },
    },
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x30,
            .hsize = 0x1000,
            .vsize = 0x0240,
            .user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
        },
    },
};
static struct mtk_mbus_frame_desc_entry frame_desc_hs[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0900,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x1000,
			.vsize = 0x0240,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_slim[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 2048,
			.vsize = 1152,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 2048,
			.vsize = 288,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus1[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 2048,
			.vsize = 1152,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 2048,
			.vsize = 288,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus2[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0c00,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x1000,
			.vsize = 0x0300,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus3[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x2000,
			.vsize = 0x1800,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus4[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 2304,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 2304,
			.user_data_desc = VC_STAGGER_ME,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 4096,
			.vsize = 576,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus5[] = {
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2b,
            .hsize = 4096,
            .vsize = 3072,
            .user_data_desc = VC_STAGGER_NE,
        },
    },
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x30,
            .hsize = 2048,
            .vsize = 1536,
            .user_data_desc = VC_PDAF_STATS_NE_PIX_1,
            .dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
        },
    },
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus6[] = {
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2b,
            .hsize = 1296,
            .vsize = 736,
            .user_data_desc = VC_STAGGER_NE,
        },
    },
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus7[] = {
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2b,
            .hsize = 1408,
            .vsize = 1056,
            .user_data_desc = VC_STAGGER_NE,
        },
    },
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x30,
            .hsize = 1408,
            .vsize = 264,
            .user_data_desc = VC_PDAF_STATS_NE_PIX_1,
            .dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
        },
    },
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus8[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_ME,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x1000,
			.vsize = 0x0300,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
//	{
//		.bus.csi2 = {
//			.channel = 1,
//			.data_type = 0x30,
//			.hsize = 0x1000,
//			.vsize = 0x0300,
//			.user_data_desc = VC_PDAF_STATS_ME_PIX_1,
//			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
//		},
//	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus9[] = {
    {
        .bus.csi2 = {
            .channel = 0,       // vc-id
            .data_type = 0x2c,  // dt
            .hsize = 0x1000,    // 4096 (data width, raw data is by pixel)
            .vsize = 0x0900,    // 2304 (data height)
            .user_data_desc = VC_RAW_PROCESSED_DATA,  // processed raw
        },
    },
    {
        .bus.csi2 = {
            .channel = 1,       // vc-id
            .data_type = 0x2b,  // dt
            .hsize = 0x1008,    // 4104 (data width, raw data is by pixel)
            .vsize = 0x1200,    // 4608 (data height)
            .user_data_desc = VC_RAW_DATA,  // sensor raw data
        },
    },
    {
        .bus.csi2 = {
            .channel = 1,
            .data_type = 0x12,
            .hsize = 1024,   // 1024 pixels data width (embedded data by byte)
            .vsize = 568,    // 568 data height
            .user_data_desc = VC_GENERAL_EMBEDDED,  // 3a_meta

        },
    },
    {
        .bus.csi2 = {
            .channel = 2,
            .data_type = 0x2e,
            .hsize = 4096,
            .vsize = 576,
            .dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
            .user_data_desc = VC_PDAF_STATS_NE_PIX_1,
        },
    },
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus10[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0c00,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
#if 0
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x2b,
			.hsize = 0x0800,
			.vsize = 0x0300,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
		},
	},
#endif
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus11[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1968,
			.vsize = 1476,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 1968,
			.vsize = 369,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus12[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0900,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus13[] = {
    {
        .bus.csi2 = {
            .channel = 0,       // vc-id
            .data_type = 0x2c,  // dt
            .hsize = 0x1000,    // 4096 (data width, raw data is by pixel)
            .vsize = 0x0900,    // 2304 (data height)
            .user_data_desc = VC_RAW_PROCESSED_DATA, //  processed raw
        },
    },
    {
        .bus.csi2 = {
            .channel = 1,       // vc-id
            .data_type = 0x2b,  // dt
            .hsize = 0x1000,    // 4096 (data width, raw data is by pixel)
            .vsize = 0x0900,    // 2304 (data height)
            .user_data_desc = VC_RAW_DATA, // sensor raw data
        },
    },
    {
        .bus.csi2 = {
            .channel = 1,
            .data_type = 0x12,
            .hsize = 1024,   // 1024 pixels data width (embedded data by byte)
            .vsize = 568,   // 568 data height
            .user_data_desc = VC_GENERAL_EMBEDDED,  // 3a_meta

        },
    },
    {
        .bus.csi2 = {
            .channel = 2,
            .data_type = 0x2e,
            .hsize = 4096,
            .vsize = 576,
            .dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
            .user_data_desc = VC_PDAF_STATS_NE_PIX_1,
        },
    },
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus14[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0900,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0900,
			.user_data_desc = VC_STAGGER_ME,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 4096,
			.vsize = 576,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus15[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 2048,
			.vsize = 1152,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 2048,
			.vsize = 288,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct subdrv_mode_struct mode_struct[] = {
	{/* Reg_K8-1_4096x3072_30FPS */
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = omegac1main_preview_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1main_preview_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 3014400000,
		.linelength = 15616,
		.framelength = 6434,
		.max_framerate = 300,
		.mipi_pixel_rate = 1201370000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,//cc
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1465,//cc
		.fine_integ_line = 827,
		.delay_frame = 2,
		.csi_param = {},
		.ana_gain_max = BASEGAIN * 64,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 30,
		},
	},
	{/* Reg_K5-1_4096x3072_30FPS */
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = omegac1main_capture_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1main_capture_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = omegac1main_seamless_capture,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(omegac1main_seamless_capture),
		.hdr_group = 1,
		.hdr_mode = HDR_NONE,
		.pclk = 3014400000,
		.linelength = 15616,
		.framelength = 6434,
		.max_framerate = 300,
		.mipi_pixel_rate = 1201370000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1465,
		.fine_integ_line = 827,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 0x36,
		},
		.ana_gain_max = BASEGAIN * 64,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},
	{/* Reg_M3_S3_4096x3072_30FPS */
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = omegac1main_normal_video_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1main_normal_video_setting),
		.seamless_switch_group = 3,
		.seamless_switch_mode_setting_table = omegac1main_seamless_normal_video,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(omegac1main_seamless_normal_video),
		.hdr_group = 3,
		.hdr_mode = HDR_NONE,
		.pclk = 2246400000,
		.linelength = 15616,
		.framelength = 4794,
		.max_framerate = 300,
		.mipi_pixel_rate = 938060000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
			.scale_w = 4096,
			.scale_h = 2304,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 2304,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 2304,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1465,
		.fine_integ_line = 827,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 0x3C,
		},
		.ana_gain_max = BASEGAIN * 64,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 30,
		},
	},
	{/* Reg_S_1_4096x2304_60FPS */
		.frame_desc = frame_desc_hs,
		.num_entries = ARRAY_SIZE(frame_desc_hs),
		.mode_setting_table = omegac1main_hs_video_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1main_hs_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 3052800000,
		.linelength = 15616,
		.framelength = 3258,
		.max_framerate = 600,
		.mipi_pixel_rate = 1357710000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
			.scale_w = 4096,
			.scale_h = 2304,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 2304,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 2304,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1465,
		.fine_integ_line = 827,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 0x38,
		},
		.ana_gain_max = BASEGAIN * 64,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 60,
		},
	},
	{/* Reg_M-1 QBIN(HVBIN) - V2H2_FHD_2048x1152_120FPS */
		.frame_desc = frame_desc_slim,
		.num_entries = ARRAY_SIZE(frame_desc_slim),
		.mode_setting_table = omegac1main_slim_video_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1main_slim_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 3283200000,
		.linelength = 8816,
		.framelength = 3100,
		.max_framerate = 1200,
		.mipi_pixel_rate = 1780800000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
			.scale_w = 2048,
			.scale_h = 1152,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2048,
			.h1_size = 1152,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2048,
			.h2_tg_size = 1152,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_v2h2,
		.ae_binning_ratio = 1465,
		.fine_integ_line = 2555,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 0x3D,
		},
		.ana_gain_max = BASEGAIN * 64,
		.coarse_integ_step = 4,
		.min_exposure_line = 8,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 120,
		},
	},
	{/* Reg_M_2048x1152_240FPS */
		.frame_desc = frame_desc_cus1,
		.num_entries = ARRAY_SIZE(frame_desc_cus1),
		.mode_setting_table = omegac1main_custom1_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1main_custom1_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 3283200000,
		.linelength = 8816,
		.framelength = 1548,
		.max_framerate = 2400,
		.mipi_pixel_rate = 1667660000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
			.scale_w = 2048,
			.scale_h = 1152,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2048,
			.h1_size = 1152,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2048,
			.h2_tg_size = 1152,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_v2h2,
		.ae_binning_ratio = 1465,
		.fine_integ_line = 2555,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 0x3D,
		},
		.ana_gain_max = BASEGAIN * 64,
		.coarse_integ_step = 4,
		.min_exposure_line = 8,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 240,
		},
	},
	{/* reg_B2 QBIN(VBIN)_4096x3072 @24FPS*/
		.frame_desc = frame_desc_cus2,
		.num_entries = ARRAY_SIZE(frame_desc_cus2),
		.mode_setting_table = omegac1main_custom2_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1main_custom2_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 1523200000,
		.linelength = 15616,
		.framelength = 4064,
		.max_framerate = 240,
		.mipi_pixel_rate = 578740000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1465,
		.fine_integ_line = 413,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 0x38,
		},
		.ana_gain_max = BASEGAIN * 64,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 24,
		},
	},
	{/* Reg_A 8192x6144_11FPS remosaic*/
		.frame_desc = frame_desc_cus3,
		.num_entries = ARRAY_SIZE(frame_desc_cus3),
		.mode_setting_table = omegac1main_custom3_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1main_custom3_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 1603200000,
		.linelength = 23104,
		.framelength = 6308,
		.max_framerate = 110,
		.mipi_pixel_rate = 784460000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 8192,
			.scale_h = 6144,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 8192,
			.h1_size = 6144,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 8192,
			.h2_tg_size = 6144,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = &imgsensor_pd_info_full,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 751,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 0x42,
		},
		.ana_gain_max = BASEGAIN * 16,
		.coarse_integ_step = 1,
		.min_exposure_line = 9,
		.sensor_setting_info = {
			.sensor_scenario_usage = RMSC_MASK,
			.equivalent_fps = 11,
		},
	},
	{/* Reg_M4-S3 4096x2304_30FPS, 2DOL Video  seamless M3_S3 (normal_video)*/
    	.frame_desc = frame_desc_cus4,
		.num_entries = ARRAY_SIZE(frame_desc_cus4),
		.mode_setting_table = omegac1main_custom4_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1main_custom4_setting),
		.seamless_switch_group = 3,
		.seamless_switch_mode_setting_table = omegac1main_seamless_custom4,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(omegac1main_seamless_custom4),
		.hdr_group = 3,
		.hdr_mode = HDR_RAW_STAGGER_2EXP,
		.pclk = 2246400000,
		.linelength = 15616,
		.framelength = 4792,
		.max_framerate = 300,
		.mipi_pixel_rate = 938060000,
		.readout_length = 2374*2,//cc
		.read_margin = 10*2,
		.framelength_step = 4*2,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
			.scale_w = 4096,
			.scale_h = 2304,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 2304,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 2304,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1465,
		.fine_integ_line = 2826,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 0x41,
		},
		.ana_gain_max = BASEGAIN * 64,
		.sensor_setting_info = {
			.sensor_scenario_usage = HDR_RAW_STAGGER_2EXP_MASK,
			.equivalent_fps = 30,
		},
	},
	{/* Reg U QRMSC_4096x3072_30FPS for insensor zoom*/
		.frame_desc = frame_desc_cus5,
		.num_entries = ARRAY_SIZE(frame_desc_cus5),
		.mode_setting_table = omegac1main_custom5_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1main_custom5_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 1507200000,
		.linelength = 11552,
		.framelength = 4348,
		.max_framerate = 300,
		.mipi_pixel_rate = 798170000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 1536,
			.w0_size = 8192,
			.h0_size = 3072,
			.scale_w = 8192,
			.scale_h = 3072,
			.x1_offset = 2048,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_full,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 503,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 0x43,
		},
		.ana_gain_max = BASEGAIN * 16,
		.coarse_integ_step = 1,
		.min_exposure_line = 9,
		.sensor_setting_info = {
			.sensor_scenario_usage = INSENSORZOOM_MASK,
			.equivalent_fps = 30,
		},
	},
	{/* Reg_L_1296x736_480FPS */
		.frame_desc = frame_desc_cus6,
		.num_entries = ARRAY_SIZE(frame_desc_cus6),
		.mode_setting_table = omegac1main_custom6_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1main_custom6_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 3513600000,
		.linelength = 5568,
		.framelength = 1312,
		.max_framerate = 4800,
		.mipi_pixel_rate = 1305600000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 1600,
			.w0_size = 8192,
			.h0_size = 2944,
			.scale_w = 2048,
			.scale_h = 736,
			.x1_offset = 376,
			.y1_offset = 0,
			.w1_size = 1296,
			.h1_size = 736,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1296,
			.h2_tg_size = 736,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = &imgsensor_pd_info_v2h2,
		.ae_binning_ratio = 1465,
		.fine_integ_line = 1710,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 0x3C,
		},
		.ana_gain_max = BASEGAIN * 64,
		.coarse_integ_step = 4,
		.min_exposure_line = 8,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 480,
		},
	},
	{/* reg-V1 1408x1056 @24FPS QBIN(VBIN) V2H2 Crop w/ PD Tline 15.13us */
		.frame_desc = frame_desc_cus7,
		.num_entries = ARRAY_SIZE(frame_desc_cus7),
		.mode_setting_table = omegac1main_custom7_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1main_custom7_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 1747200000,
		.linelength = 26448,
		.framelength = 2752,
		.max_framerate = 240,
		.mipi_pixel_rate = 205710000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 1280,
			.y0_offset = 960,
			.w0_size = 5632,
			.h0_size = 4224,
			.scale_w = 1408,
			.scale_h = 1056,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 1408,
			.h1_size = 1056,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1408,
			.h2_tg_size = 1056,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_v2h2,
		.ae_binning_ratio = 1465,
		.fine_integ_line = 1709,
		.delay_frame = 2,
//		.csi_param = {
//			.cphy_settle = 0x78,
//		},
		.ana_gain_max = BASEGAIN * 64,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 24,
		},
	},
	{ /* Reg_K5 4096x3072_30FPS, 2DOL preview seamless with capture */
		.frame_desc = frame_desc_cus8,
		.num_entries = ARRAY_SIZE(frame_desc_cus8),
		.mode_setting_table = omegac1main_custom8_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1main_custom8_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = omegac1main_seamless_custom8,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(omegac1main_seamless_custom8),
		.hdr_group = 1,
		.hdr_mode = HDR_RAW_STAGGER_2EXP,
		.pclk = 3014400000,
		.linelength = 15616,
		.framelength = 6432,
		.max_framerate = 300,
		.mipi_pixel_rate = 1201370000,
		.readout_length = 3107*2,//cc
		.read_margin = 10*2,
		.framelength_step = 4*2,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1465,
		.fine_integ_line = 2826,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 0x39,
		},
		.ana_gain_max = BASEGAIN * 64,
		.sensor_setting_info = {
			.sensor_scenario_usage = HDR_RAW_STAGGER_2EXP_MASK,
			.equivalent_fps = 30,
		},
	},
	{/* Reg_B_4096x3072_30FPS, for preisp dump */    /*unused*/
		.frame_desc = frame_desc_cus9,
		.num_entries = ARRAY_SIZE(frame_desc_cus9),
		.mode_setting_table = omegac1main_custom9_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1main_custom9_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = 2,
		.hdr_mode = HDR_RAW_STAGGER_2EXP,
		.pclk = 2246400000,
		.linelength = 15616,
		.framelength = 4792,
		.max_framerate = 300,
		.mipi_pixel_rate = (2800*3*16/7/10*1000000),
		.readout_length = 2339 *2,
		.read_margin = 10 * 2,
		.framelength_step = 4 * 2,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
			.scale_w = 4096,
			.scale_h = 2304,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 2304,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 2304,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1465,
		.fine_integ_line = 826,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 0x78,
		},
		.ana_gain_max = BASEGAIN * 64,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},
	{
		.frame_desc = frame_desc_cus10,
		.num_entries = ARRAY_SIZE(frame_desc_cus10),
		.mode_setting_table = omegac1main_custom10_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1main_custom10_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 1785600000,
		.linelength = 11552,
		.framelength = 5152,
		.max_framerate = 300,
		.mipi_pixel_rate = 1782860000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 1536,
			.w0_size = 8192,
			.h0_size = 3072,
			.scale_w = 8192,
			.scale_h = 3072,
			.x1_offset = 2048,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1465,
		.fine_integ_line = 502,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 0x3A,
		},
		.ana_gain_max = BASEGAIN * 64,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},
	{/*regD1
		QBIN(VBIN) V2H2 1968x1476 @24FPS Tline Max. (20.66us) */
		.frame_desc = frame_desc_cus11,
		.num_entries = ARRAY_SIZE(frame_desc_cus11),
		.mode_setting_table = omegac1main_custom11_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1main_custom11_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 1280000000,
		.linelength = 26448,
		.framelength = 2016,
		.max_framerate = 240,
		// .mipi_pixel_rate = 342860000,
		.mipi_pixel_rate = 344230000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 96,
			.w0_size = 8192,
			.h0_size = 5952,
			.scale_w = 2048,
			.scale_h = 1488,
			.x1_offset = 40,
			.y1_offset = 6,
			.w1_size = 1968,
			.h1_size = 1476,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1968,
			.h2_tg_size = 1476,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1465,
		.fine_integ_line = 502,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 0x4C,
		},
		.ana_gain_max = BASEGAIN * 64,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},
	{/* Reg_Q-2-1_4096x2304_30FPS; Just for detect the brokenImg in film scenario. */
		.mode_setting_table = omegac1main_custom12_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1main_custom12_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 2246400000,
		.linelength = 15616,
		.framelength = 4794,
		.max_framerate = 300,
		.mipi_pixel_rate = 880457143,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
			.scale_w = 4096,
			.scale_h = 2304,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 2304,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 2304,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1465,
		.fine_integ_line = 827,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 0x3B,
		},
		.frame_desc = frame_desc_cus12,
		.num_entries = ARRAY_SIZE(frame_desc_cus12),
		.ana_gain_max = BASEGAIN * 64,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},
	{/* Reg_Q-2-1_4096x2304_30FPS, normal_video for preISP*/    /*unused*/
		.frame_desc = frame_desc_cus13,
		.num_entries = ARRAY_SIZE(frame_desc_cus13),
		.mode_setting_table = omegac1main_custom13_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1main_custom13_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = 2,
		.hdr_mode = HDR_NONE,
		.pclk = 2246400000,
		.linelength = 15616,
		.framelength = 4794,
		.max_framerate = 300,
		.mipi_pixel_rate = (2800*3*16/7/10*1000000),
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
			.scale_w = 4096,
			.scale_h = 2304,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 2304,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 2304,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1465,
		.fine_integ_line = 1709,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 0x78,
		},
		.ana_gain_max = BASEGAIN * 64,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},
	{/* Reg_S_4096x2304_30FPS seamless K*/
		.frame_desc = frame_desc_cus14,
		.num_entries = ARRAY_SIZE(frame_desc_cus14),
		.mode_setting_table = omegac1main_custom14_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1main_custom14_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = omegac1main_seamless_custom14,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(omegac1main_seamless_custom14),
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 2246400000,
		.linelength = 15616,
		.framelength = 4792,
		.max_framerate = 300,
		.mipi_pixel_rate = 938060000,
		.readout_length = 2374 * 2,
		.read_margin = 10 * 2,
		.framelength_step = 4 * 2,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
			.scale_w = 4096,
			.scale_h = 2304,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 2304,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 2304,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1465,
		.fine_integ_line = 827,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 0x3C,
		},
		.ana_gain_max = BASEGAIN * 64,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},
	{ /* reg_H1 QBin(VBin) V2H2 2048x1152 @30fps w/ PD Tline 3.90us */
		.frame_desc = frame_desc_cus15,
		.num_entries = ARRAY_SIZE(frame_desc_cus15),
		.mode_setting_table = omegac1main_custom15_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1main_custom15_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 2256000000,
		.linelength = 8816,
		.framelength = 8528,
		.max_framerate = 300,
		.mipi_pixel_rate = 880460000,
		.readout_length = 0,
		.read_margin = 10,
		.framelength_step = 4,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
			.scale_w = 2048,
			.scale_h = 1152,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2048,
			.h1_size = 1152,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2048,
			.h2_tg_size = 1152,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_v2h2,
		.ae_binning_ratio = 1465,
		.fine_integ_line = 2555,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 0x39,
		},
		.ana_gain_max = BASEGAIN * 64,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = OMEGAC1MAIN_SENSOR_ID,
	.reg_addr_sensor_id = {0x0016, 0x0017},
	.i2c_addr_table = {0x34, 0xFF},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8,
	.eeprom_info = eeprom_info,
	.eeprom_num = ARRAY_SIZE(eeprom_info),
	.resolution = {8192, 6144},
	.mirror = IMAGE_HV_MIRROR,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_CPHY,
	.mipi_lane_num = SENSOR_MIPI_3_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_B,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 64,
	.ana_gain_type = 0,
	.ana_gain_step = 1,
	.ana_gain_table = omegac1main_ana_gain_table,
	.ana_gain_table_size = sizeof(omegac1main_ana_gain_table),
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 8,
	.exposure_max = 128*(65532 - 48), /* exposure reg is limited to 4x. max = max - margin */
	.exposure_step = 2,
	.exposure_margin = 48,

	.frame_length_max = 0xffff,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 3,
	.start_exposure_offset = 3000000,//cc

	.pdaf_type = PDAF_SUPPORT_CAMSV_QPD,
	.hdr_type = HDR_SUPPORT_STAGGER_FDOL,
	.seamless_switch_support = TRUE,
	.temperature_support = TRUE,

	.g_temp = get_sensor_temperature,
	.g_gain2reg = get_gain2reg,
	.g_cali = get_sensor_cali,
	.s_gph = set_group_hold,
	.s_cali = set_sensor_cali,

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = 0x0101,
	.reg_addr_exposure = {
			{0x0202, 0x0203},//Long exposure
			{0x313A, 0x313B},//Middle exposure
			{0x0224, 0x0225},//Short exposure
	},
	.long_exposure_support = TRUE,
	.reg_addr_exposure_lshift = 0x3128,
	.reg_addr_ana_gain = {
			{0x0204, 0x0205},//Long Gian
			{0x313C, 0x313D},//Middle Gian
			{0x0216, 0x0217},//Short Gian
	},
	.reg_addr_frame_length = {0x0340, 0x0341},
	.reg_addr_temp_en = 0x0138,
	.reg_addr_temp_read = 0x013A,
	.reg_addr_auto_extend = 0x0350,
	.reg_addr_frame_count = PARAM_UNDEFINED,
	.reg_addr_fast_mode = 0x3010,

	.init_setting_table = omegac1main_init_setting,
	.init_setting_len = ARRAY_SIZE(omegac1main_init_setting),
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),

	.chk_s_off_before_s_on = 1,
	.chk_s_off_before_control = 1,
	.chk_s_off_after_s_off = 0,

	//.checksum_value = 0xAF3E324F,
	.checksum_value = 0xf10e5980,
};

static struct subdrv_ops ops = {
	.get_id = get_imgsensor_id,
	.init_ctx = init_ctx,
	.open = open,
	.get_info = common_get_info,
	.get_resolution = common_get_resolution,
	.control = common_control,
	.feature_control = common_feature_control,
	.close = common_close,
	.get_frame_desc = common_get_frame_desc,
	.get_temp = common_get_temp,
	.get_csi_param = common_get_csi_param,
	.vsync_notify = vsync_notify,
	.update_sof_cnt = common_update_sof_cnt,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_MCLK, 24, 0},
	{HW_ID_RST, 0, 1},
	{HW_ID_DOVDD, 1804000, 1},
	{HW_ID_DOVDD, 0, 3},
	{HW_ID_DOVDD, 1804000, 1},
	{HW_ID_DOVDD, 0, 3},
	{HW_ID_DOVDD, 1804000, 1},
	{HW_ID_AVDD, 2804000, 3},
	{HW_ID_AVDD1, 1804000, 3},
	{HW_ID_AFVDD, 2804000, 3},
	{HW_ID_DVDD, 1112000, 4},
	{HW_ID_MCLK_DRIVING_CURRENT, 4, 6},
	{HW_ID_RST, 1, 5}
};

static struct subdrv_pw_seq_entry pw_off_seq[] = {
	{HW_ID_MCLK, 24, 0},
	{HW_ID_RST, 0, 1},
	{HW_ID_DOVDD, 1804000, 1},
	{HW_ID_AVDD, 2804000, 3},
	{HW_ID_AVDD1, 1804000, 3},
	{HW_ID_AFVDD, 2804000, 3},
	{HW_ID_DVDD, 1112000, 4},
	{HW_ID_MCLK_DRIVING_CURRENT, 4, 6},
	{HW_ID_RST, 1, 5}
};

const struct subdrv_entry omegac1main_mipi_raw_entry = {
	.name = "omegac1main_mipi_raw",
	.id = OMEGAC1MAIN_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.pw_off_seq = pw_off_seq,
	.pw_off_seq_cnt = ARRAY_SIZE(pw_off_seq),
	.ops = &ops,
};

/* FUNCTION */

static unsigned int read_omegac1main_eeprom_info(struct subdrv_ctx *ctx, kal_uint16 meta_id,
	BYTE *data, int size)
{
	kal_uint16 addr;
	int readsize;

	if (meta_id != omegac1main_eeprom_info[meta_id].meta)
		return -1;

	if (size != omegac1main_eeprom_info[meta_id].size)
		return -1;

	addr = omegac1main_eeprom_info[meta_id].start;
	readsize = omegac1main_eeprom_info[meta_id].size;

	if(!read_cmos_eeprom_p8(ctx, addr, data, readsize)) {
		DRV_LOGE(ctx, "read meta_id(%d) failed", meta_id);
	}

	return 0;
}

static void read_eeprom_common_data(struct subdrv_ctx *ctx)
{
	kal_uint16 idx = 0;
	kal_uint16 AF_CODE_MACRO_idx = 0;
	kal_uint16 AF_CODE_INFINITY_idx = 0;
	kal_uint16 AF_CODE_MIDDLE_idx = 0;
	u8 ois_version = 0;

	memset(&omegac1main_eeprom_common_data, 0x00, sizeof(omegac1main_eeprom_common_data));

	omegac1main_eeprom_common_data.header[EEPROM_MODULE_ID] = 2;
	read_omegac1main_eeprom_info(ctx, EEPROM_META_MODULE_ID,
				&(omegac1main_eeprom_common_data.data[idx]), 2);
	idx += omegac1main_eeprom_common_data.header[EEPROM_MODULE_ID];

	omegac1main_eeprom_common_data.header[EEPROM_SENSOR_ID] = 2;
	read_omegac1main_eeprom_info(ctx, EEPROM_META_SENSOR_ID,
				&(omegac1main_eeprom_common_data.data[idx]), 2);
	idx += omegac1main_eeprom_common_data.header[EEPROM_SENSOR_ID];

	omegac1main_eeprom_common_data.header[EEPROM_LENS_ID] = 2;
	read_omegac1main_eeprom_info(ctx, EEPROM_META_LENS_ID,
				&(omegac1main_eeprom_common_data.data[idx]), 2);
	idx += omegac1main_eeprom_common_data.header[EEPROM_LENS_ID];

	omegac1main_eeprom_common_data.header[EEPROM_VCM_ID] = 2;
	read_omegac1main_eeprom_info(ctx, EEPROM_META_VCM_ID,
				&(omegac1main_eeprom_common_data.data[idx]), 2);
	if(omegac1main_eeprom_common_data.data[idx] == 0x04) {
		ois_version = 1;
	}
	idx += omegac1main_eeprom_common_data.header[EEPROM_VCM_ID];

	omegac1main_eeprom_common_data.header[EEPROM_MODULE_SN] = 23;
	read_omegac1main_eeprom_info(ctx, EEPROM_META_MODULE_SN,
				&(omegac1main_eeprom_common_data.data[idx]), 23);
	idx += omegac1main_eeprom_common_data.header[EEPROM_MODULE_SN];

	omegac1main_eeprom_common_data.header[EEPROM_AF_CODE_MACRO] = 2;
	omegac1main_eeprom_common_data.header[EEPROM_AF_CODE_INFINITY] = 2;
	omegac1main_eeprom_common_data.header[EEPROM_AF_CODE_MIDDLE] = 2;
	read_omegac1main_eeprom_info(ctx, EEPROM_META_AF_CODE,
				&(omegac1main_eeprom_common_data.data[idx]), 6);
	idx += 6;

	omegac1main_eeprom_common_data.header[SENSOR_OIS_VERSION] = 1;
	omegac1main_eeprom_common_data.data[idx] = ois_version;

	AF_CODE_MACRO_idx = idx;
	AF_CODE_INFINITY_idx = idx + 2;
	AF_CODE_MIDDLE_idx = idx + 4;

	for (idx = 0; idx < 64; idx = idx + 4)
		LOG_INF("In %s:common data: %02x %02x %02x %02x\n", __func__,
			omegac1main_eeprom_common_data.data[idx], omegac1main_eeprom_common_data.data[idx + 1],
			omegac1main_eeprom_common_data.data[idx + 2],
			omegac1main_eeprom_common_data.data[idx + 3]);
//    //4096 -> 1024
//    omegac1main_eeprom_common_data.data[AF_CODE_MACRO_idx] =
//		(kal_uint8)((omegac1main_eeprom_common_data.data[AF_CODE_MACRO_idx + 1] << 6) | (omegac1main_eeprom_common_data.data[AF_CODE_MACRO_idx] >> 2));
//    omegac1main_eeprom_common_data.data[AF_CODE_MACRO_idx + 1] = (kal_uint8)(omegac1main_eeprom_common_data.data[AF_CODE_MACRO_idx +1] >> 2);
//
//    omegac1main_eeprom_common_data.data[AF_CODE_INFINITY_idx] =
//		(kal_uint8)((omegac1main_eeprom_common_data.data[AF_CODE_INFINITY_idx + 1] << 6) | (omegac1main_eeprom_common_data.data[AF_CODE_INFINITY_idx] >> 2));
//    omegac1main_eeprom_common_data.data[AF_CODE_INFINITY_idx + 1] = (kal_uint8)(omegac1main_eeprom_common_data.data[AF_CODE_INFINITY_idx +1] >> 2);
//
//    omegac1main_eeprom_common_data.data[AF_CODE_MIDDLE_idx] =
//		(kal_uint8)((omegac1main_eeprom_common_data.data[AF_CODE_MIDDLE_idx + 1] << 6) | (omegac1main_eeprom_common_data.data[AF_CODE_MIDDLE_idx] >> 2));
//    omegac1main_eeprom_common_data.data[AF_CODE_MIDDLE_idx + 1] = (kal_uint8)(omegac1main_eeprom_common_data.data[AF_CODE_MIDDLE_idx +1] >> 2);
}

static void omegac1main_get_eeprom_comdata(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 *feature_return_para_32 = (u32 *)para;
	LOG_INF("+");

	if(*len == sizeof(omegac1main_eeprom_common_data)) {
		memcpy(feature_return_para_32, &omegac1main_eeprom_common_data,
		sizeof(omegac1main_eeprom_common_data));
	}
}

//static void read_unique_sensorid(struct subdrv_ctx *ctx)
//{
//	u8 i = 0;
//	LOG_INF("read sensor unique sensorid");
//	while (ctx->s_ctx.i2c_addr_table[i] != 0xFF) {
//		ctx->i2c_write_id = ctx->s_ctx.i2c_addr_table[i];
//		subdrv_i2c_wr_u8(ctx, 0x0A02, 0x9F);
//		subdrv_i2c_wr_u8(ctx, 0x0A00, 0x01);
//		adaptor_i2c_rd_p8(ctx->i2c_client, ctx->i2c_write_id >> 1,
//		OMEGAC1MAIN_UNIQUE_SENSOR_ID, &(omegac1main_unique_id[0]), OMEGAC1MAIN_UNIQUE_SENSOR_ID_LENGHT);
//		i++;
//	}
//}

//static void omegac1main_get_unique_sensorid(struct subdrv_ctx *ctx, u8 *para, u32 *len)
//{
//	u32 *feature_return_para_32 = (u32 *)para;
//	LOG_INF("get unique sensorid");
//	memcpy(feature_return_para_32, omegac1main_unique_id,
//		OMEGAC1MAIN_UNIQUE_SENSOR_ID_LENGHT);
//	LOG_INF("para :%x, get unique sensorid", *para);
//}

static kal_uint16 read_cmos_eeprom_8(struct subdrv_ctx *ctx, kal_uint16 addr)
{
	kal_uint16 get_byte = 0;

	adaptor_i2c_rd_u8(ctx->i2c_client, OMEGAC1MAIN_EEPROM_READ_ID >> 1, addr, (u8 *)&get_byte);
	return get_byte;
}

static kal_int32 table_write_eeprom_one_packet(struct subdrv_ctx *ctx,
        kal_uint16 addr, kal_uint8 *para, kal_uint32 len)
{
    kal_int32 ret = ERROR_NONE;
    ret = adaptor_i2c_wr_p8(ctx->i2c_client, OMEGAC1MAIN_EEPROM_WRITE_ID >> 1,
            addr, para, len);

    return ret;
}

static kal_int32 write_eeprom_protect(struct subdrv_ctx *ctx, kal_uint16 enable)
{
    kal_int32 ret = ERROR_NONE;
    kal_uint16 reg = OMEGAC1MAIN_EEPROM_LOCK_REGISTER;
    if (enable) {
        adaptor_i2c_wr_u8(ctx->i2c_client, OMEGAC1MAIN_EEPROM_WRITE_ID >> 1,
            reg, (OMEGAC1MAIN_EEPROM_WRITE_ID & 0xFE) | 0x01);
    }
    else {
        adaptor_i2c_wr_u8(ctx->i2c_client, OMEGAC1MAIN_EEPROM_WRITE_ID >> 1,
            reg, OMEGAC1MAIN_EEPROM_WRITE_ID & 0xFE);
    }

    return ret;
}


static kal_uint16 get_64align_addr(kal_uint16 data_base) {

	kal_uint16 multiple = 0;
	kal_uint16 surplus = 0;
	kal_uint16 addr_64align = 0;

	multiple = data_base / 64;
	surplus = data_base % 64;
	if(surplus) {
		addr_64align = (multiple + 1) * 64;
	} else {
		addr_64align = multiple * 64;
	}
	//LOG_INF("data_base(0x%x), multiple(%d), surplus(%d), addr_64align(0x%x)", data_base, multiple, surplus, addr_64align);
	return addr_64align;
}

static kal_int32 eeprom_table_write(struct subdrv_ctx *ctx, kal_uint16 data_base, kal_uint8 *pData, kal_uint16 data_length) {

	kal_uint16 idx;
	kal_uint16 idy;
	kal_int32 ret = ERROR_NONE;
	UINT32 i = 0;

	idx = data_length/EEPROM_WRITE_DATA_MAX_LENGTH;
	idy = data_length%EEPROM_WRITE_DATA_MAX_LENGTH;

    LOG_INF("data_base(0x%x) data_length(%d) idx(%d) idy(%d)\n", data_base, data_length, idx, idy);

	for (i = 0; i < idx; i++ ) {
		ret = table_write_eeprom_one_packet(ctx, (data_base + EEPROM_WRITE_DATA_MAX_LENGTH * i),
				&pData[EEPROM_WRITE_DATA_MAX_LENGTH*i], EEPROM_WRITE_DATA_MAX_LENGTH);
		if (ret != ERROR_NONE) {
			LOG_INF("write_eeprom error: i=%d\n", i);
			return -1;
		}
		msleep(6);
	}

	msleep(6);
	if(idy) {
		ret = table_write_eeprom_one_packet(ctx, (data_base + EEPROM_WRITE_DATA_MAX_LENGTH*idx),
				&pData[EEPROM_WRITE_DATA_MAX_LENGTH*idx], idy);
		if (ret != ERROR_NONE) {
			LOG_INF("write_eeprom error: idx= %d idy= %d\n", idx, idy);
			return -1;
		}
	}
	return 0;
}

static kal_int32 eeprom_64align_write(struct subdrv_ctx *ctx, kal_uint16 data_base, kal_uint8 *pData, kal_uint16 data_length) {

	kal_uint16 addr_64align = 0;
	kal_uint16 part1_length = 0;
	kal_uint16 part2_length = 0;
	kal_int32 ret = ERROR_NONE;

    addr_64align = get_64align_addr(data_base);

	part1_length = addr_64align - data_base;
	if(part1_length > data_length) {
		part1_length = data_length;
	}
	part2_length = data_length - part1_length;

	write_eeprom_protect(ctx, 0);
	msleep(6);

	if (part1_length) {
		ret = eeprom_table_write(ctx, data_base, pData, part1_length);
		if (ret == -1) {
			/* open write protect */
			write_eeprom_protect(ctx, 1);
			LOG_INF("write_eeprom error part1\n");
			msleep(6);
			return -1;
		}
	}

	msleep(6);
	if (part2_length) {
		ret = eeprom_table_write(ctx, addr_64align, pData + part1_length, part2_length);
		if (ret == -1) {
			/* open write protect */
			write_eeprom_protect(ctx, 1);
			LOG_INF("write_eeprom error part2\n");
			msleep(6);
			return -1;
		}
	}
	msleep(6);
	write_eeprom_protect(ctx, 1);
	msleep(6);

	return 0;
}

static kal_int32 write_Module_data(struct subdrv_ctx *ctx,
    ACDK_SENSOR_ENGMODE_STEREO_STRUCT * pStereodata)
{
    kal_int32  ret = ERROR_NONE;
    kal_uint16 data_base, data_length;
    kal_uint8 *pData;

    if(pStereodata != NULL) {
        LOG_INF("SET_SENSOR_OTP: 0x%x %d 0x%x %d\n",
                       pStereodata->uSensorId,
                       pStereodata->uDeviceId,
                       pStereodata->baseAddr,
                       pStereodata->dataLength);

        data_base = pStereodata->baseAddr;
        data_length = pStereodata->dataLength;
        pData = pStereodata->uData;
        if ((pStereodata->uSensorId == OMEGAC1MAIN_SENSOR_ID) && (data_length == CALI_DATA_MASTER_LENGTH)
            && (data_base == OMEGAC1MAIN_STEREO_MT_START_ADDR || data_base == OMEGAC1MAIN_STEREO_MW_START_ADDR)) {
            LOG_INF("Write: %x %x %x %x\n", pData[0], pData[39], pData[40], pData[1556]);

            eeprom_64align_write(ctx, data_base, pData, data_length);

            LOG_INF("com_0:0x%x\n", read_cmos_eeprom_8(ctx, data_base));
            LOG_INF("com_39:0x%x\n", read_cmos_eeprom_8(ctx, data_base+39));
            LOG_INF("innal_40:0x%x\n", read_cmos_eeprom_8(ctx, data_base+40));
            LOG_INF("innal_1556:0x%x\n", read_cmos_eeprom_8(ctx, data_base+1556));
            LOG_INF("write_Module_data Write end\n");

        } else if ((pStereodata->uSensorId == OMEGAC1MAIN_SENSOR_ID) && (data_length < AESYNC_DATA_LENGTH_TOTAL)
            && (data_base == OMEGAC1MAIN_AESYNC_START_ADDR)) {
            LOG_INF("write main aesync: %x %x %x %x %x %x %x %x\n", pData[0], pData[1],
                pData[2], pData[3], pData[4], pData[5], pData[6], pData[7]);

            eeprom_64align_write(ctx, data_base, pData, data_length);

            LOG_INF("readback main aesync: %x %x %x %x %x %x %x %x\n",
                    read_cmos_eeprom_8(ctx, OMEGAC1MAIN_AESYNC_START_ADDR),
                    read_cmos_eeprom_8(ctx, OMEGAC1MAIN_AESYNC_START_ADDR+1),
                    read_cmos_eeprom_8(ctx, OMEGAC1MAIN_AESYNC_START_ADDR+2),
                    read_cmos_eeprom_8(ctx, OMEGAC1MAIN_AESYNC_START_ADDR+3),
                    read_cmos_eeprom_8(ctx, OMEGAC1MAIN_AESYNC_START_ADDR+4),
                    read_cmos_eeprom_8(ctx, OMEGAC1MAIN_AESYNC_START_ADDR+5),
                    read_cmos_eeprom_8(ctx, OMEGAC1MAIN_AESYNC_START_ADDR+6),
                    read_cmos_eeprom_8(ctx, OMEGAC1MAIN_AESYNC_START_ADDR+7));
            LOG_INF("AESync write_Module_data Write end\n");
        } else {
            LOG_INF("Invalid Sensor id:0x%x write eeprom\n", pStereodata->uSensorId);
            return -1;
        }
    } else {
        LOG_INF("imx890 write_Module_data pStereodata is null\n");
        return -1;
    }
    return ret;
}

static void omegac1main_set_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
    int ret = ERROR_NONE;
    ret = write_Module_data(ctx, (ACDK_SENSOR_ENGMODE_STEREO_STRUCT *)(para));
    if (ret != ERROR_NONE) {
        LOG_INF("ret=%d\n", ret);
    }
}

static void omegac1main_get_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	UINT16 *feature_data_16 = (UINT16 *) para;
	UINT32 *feature_return_para_32 = (UINT32 *) para;
	if(*len > CALI_DATA_MASTER_LENGTH)
		*len = CALI_DATA_MASTER_LENGTH;
	LOG_INF("feature_data mode: %d", *feature_data_16);
	switch (*feature_data_16) {
	case EEPROM_STEREODATA_MT_MAIN:
		read_omegac1main_eeprom_info(ctx, EEPROM_META_STEREO_MT_MAIN_DATA,
				(BYTE *)feature_return_para_32, *len);
		break;
	case EEPROM_STEREODATA_MW_MAIN:
	default:
		read_omegac1main_eeprom_info(ctx, EEPROM_META_STEREO_MW_MAIN_DATA,
				(BYTE *)feature_return_para_32, *len);
		break;
	}
}

static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size)
{
	if (adaptor_i2c_rd_p8(ctx->i2c_client, OMEGAC1MAIN_EEPROM_READ_ID >> 1,
			addr, data, size) < 0) {
		return false;
	}
	return true;
}

static void read_otp_info(struct subdrv_ctx *ctx)
{
	DRV_LOGE(ctx, "omegac1main read_otp_info begin\n");
	read_cmos_eeprom_p8(ctx, 0, otp_data_checksum, OTP_SIZE);
	DRV_LOGE(ctx, "omegac1main read_otp_info end\n");
}

static void omegac1main_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 *feature_return_para_32 = (u32 *)para;
	u32 length = sizeof(otp_data_checksum);

	if(*len < sizeof(otp_data_checksum)) {
		length = *len;
	}
	DRV_LOGE(ctx, "get otp data length:0x%x", length);

	if (otp_data_checksum[0] == 0) {
		read_otp_info(ctx);
	} else {
		DRV_LOG(ctx, "otp data has already read");
	}

	memcpy(feature_return_para_32, (UINT32 *)otp_data_checksum, length);
}

static void omegac1main_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	get_imgsensor_id(ctx, (u32 *)para);
}

static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id)
{
	u8 i = 0;
	u8 retry = 2;
	static bool first_read = KAL_TRUE;
	u32 addr_h = ctx->s_ctx.reg_addr_sensor_id.addr[0];
	u32 addr_l = ctx->s_ctx.reg_addr_sensor_id.addr[1];
	u32 addr_ll = ctx->s_ctx.reg_addr_sensor_id.addr[2];

	while (ctx->s_ctx.i2c_addr_table[i] != 0xFF) {
		ctx->i2c_write_id = ctx->s_ctx.i2c_addr_table[i];
		do {
			*sensor_id = (subdrv_i2c_rd_u8(ctx, addr_h) << 8) |
				subdrv_i2c_rd_u8(ctx, addr_l);
			if (addr_ll)
				*sensor_id = ((*sensor_id) << 8) | subdrv_i2c_rd_u8(ctx, addr_ll);
			DRV_LOG(ctx, "i2c_write_id(0x%x) sensor_id(0x%x/0x%x)\n",
				ctx->i2c_write_id, *sensor_id, ctx->s_ctx.sensor_id);
			if ( (*sensor_id == IMX766_SENSOR_ID) || (*sensor_id == 0x890) ) {
				if (*sensor_id == 0x890) {
					g_need_addi_setting = 1;
				} else {
					g_need_addi_setting = 0;
				}
				*sensor_id = ctx->s_ctx.sensor_id;
				if (first_read) {
					read_eeprom_common_data(ctx);
//					read_unique_sensorid(ctx);
					first_read = KAL_FALSE;
				}
				return ERROR_NONE;
			}
			DRV_LOG(ctx, "Read sensor id fail. i2c_write_id: 0x%x\n", ctx->i2c_write_id);
			DRV_LOG(ctx, "sensor_id = 0x%x, ctx->s_ctx.sensor_id = 0x%x\n",
				*sensor_id, ctx->s_ctx.sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != ctx->s_ctx.sensor_id) {
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}

static u16 omegac1_feedback_awbgain[] = {
	0x0B8E, 0x01,
	0x0B8F, 0x00,
	0x0B90, 0x02,
	0x0B91, 0x28,
	0x0B92, 0x01,
	0x0B93, 0x77,
	0x0B94, 0x01,
	0x0B95, 0x00,
};

/*write AWB gain to sensor*/
static void feedback_awbgain(struct subdrv_ctx *ctx, kal_uint32 r_gain, kal_uint32 b_gain)
{
	UINT32 r_gain_reg = 0;
	UINT32 b_gain_reg = 0;

	if (ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM3 || // RMSC
			ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM5) { // QRMSC
		DRV_LOG(ctx, "feedback_awbgain r_gain: %d, b_gain: %d\n", r_gain, b_gain);
		r_gain_reg = r_gain / 2;
		b_gain_reg = b_gain / 2;

		omegac1_feedback_awbgain[5] = r_gain_reg >> 8;
		omegac1_feedback_awbgain[7] = r_gain_reg & 0xFF;
		omegac1_feedback_awbgain[9] = b_gain_reg >> 8;
		omegac1_feedback_awbgain[11] = b_gain_reg & 0xFF;
		subdrv_i2c_wr_regs_u8(ctx, omegac1_feedback_awbgain,
			ARRAY_SIZE(omegac1_feedback_awbgain));
	}
}

static void omegac1main_set_awb_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len) {
	struct SET_SENSOR_AWB_GAIN *awb_gain = (struct SET_SENSOR_AWB_GAIN *)para;
	feedback_awbgain(ctx, awb_gain->ABS_GAIN_R, awb_gain->ABS_GAIN_B);
}

static int open(struct subdrv_ctx *ctx)
{
	u32 sensor_id = 0;
	u32 scenario_id = 0;

	/* get sensor id */
	if (get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail setting */
	sensor_init(ctx);
	printk("main need addition setting[%d]", g_need_addi_setting);
	if (g_need_addi_setting == 1) {
		i2c_table_write(ctx, omegac1main_addition_setting, ARRAY_SIZE(omegac1main_addition_setting));
	}


	/*QSC setting*/
	if (ctx->s_ctx.s_cali != NULL) {
		ctx->s_ctx.s_cali((void*)ctx);
	} else {
		write_sensor_Cali(ctx);
	}

	memset(ctx->exposure, 0, sizeof(ctx->exposure));
	memset(ctx->ana_gain, 0, sizeof(ctx->gain));
	ctx->exposure[0] = ctx->s_ctx.exposure_def;
	ctx->ana_gain[0] = ctx->s_ctx.ana_gain_def;
	ctx->current_scenario_id = scenario_id;
	ctx->pclk = ctx->s_ctx.mode[scenario_id].pclk;
	ctx->line_length = ctx->s_ctx.mode[scenario_id].linelength;
	ctx->frame_length = ctx->s_ctx.mode[scenario_id].framelength;
	ctx->current_fps = 10 * ctx->pclk / ctx->line_length / ctx->frame_length;
	ctx->readout_length = ctx->s_ctx.mode[scenario_id].readout_length;
	ctx->read_margin = ctx->s_ctx.mode[scenario_id].read_margin;
	ctx->min_frame_length = ctx->frame_length;
	ctx->autoflicker_en = FALSE;
	ctx->test_pattern = 0;
	ctx->ihdr_mode = 0;
	ctx->pdaf_mode = 0;
	ctx->hdr_mode = 0;
	ctx->extend_frame_length_en = 0;
	ctx->is_seamless = 0;
	ctx->fast_mode_on = 0;
	ctx->sof_cnt = 0;
	ctx->ref_sof_cnt = 0;
	ctx->is_streaming = 0;

	return ERROR_NONE;
}

static void set_sensor_cali(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;

	u16 idx = 0;
	u8 support = FALSE;
	u8 *pbuf = NULL;
	u16 size = 0;
	u16 addr = 0;
	struct eeprom_info_struct *info = ctx->s_ctx.eeprom_info;

	if (!probe_eeprom(ctx))
		return;

	idx = ctx->eeprom_index;

	/* QSC data */
	support = info[idx].qsc_support;
	pbuf = info[idx].preload_qsc_table;
	size = info[idx].qsc_size;
	addr = info[idx].sensor_reg_addr_qsc;
	if (support) {
		if (pbuf != NULL && addr > 0 && size > 0) {
			subdrv_i2c_wr_u8(ctx, 0x86A9, 0x4E);
			subdrv_i2c_wr_seq_p8(ctx, addr, pbuf, size);
			subdrv_i2c_wr_u8(ctx, 0x32D2, 0x01);
			DRV_LOG(ctx, "set QSC calibration data done.");
		} else {
			subdrv_i2c_wr_u8(ctx, 0x32D2, 0x00);
		}
	}
}

static int get_sensor_temperature(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	u8 temperature = 0;
	int temperature_convert = 0;

	temperature = subdrv_i2c_rd_u8(ctx, ctx->s_ctx.reg_addr_temp_read);

	if (temperature <= 0x60)
		temperature_convert = temperature;
	else if (temperature >= 0x61 && temperature <= 0x7F)
		temperature_convert = 97;
	else if (temperature >= 0x80 && temperature <= 0xE2)
		temperature_convert = -30;
	else
		temperature_convert = (char)temperature | 0xFFFFFF0;

	DRV_LOG(ctx, "temperature: %d degrees\n", temperature_convert);
	return temperature_convert;
}

static void set_group_hold(void *arg, u8 en)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;

	if (en)
		set_i2c_buffer(ctx, 0x0104, 0x01);
	else
		set_i2c_buffer(ctx, 0x0104, 0x00);
}

static u16 get_gain2reg(u32 gain)
{
	return (16384 - (16384 * BASEGAIN) / gain);
}

void omegac1main_get_min_shutter_by_scenario(struct subdrv_ctx *ctx,
		enum SENSOR_SCENARIO_ID_ENUM scenario_id,
		u64 *min_shutter, u64 *exposure_step)
{
	check_current_scenario_id_bound(ctx);
	DRV_LOG(ctx, "sensor_mode_num[%d]", ctx->s_ctx.sensor_mode_num);
	if (scenario_id < ctx->s_ctx.sensor_mode_num) {
		switch (ctx->s_ctx.mode[scenario_id].hdr_mode) {
			case HDR_RAW_STAGGER_2EXP:
				*exposure_step = ctx->s_ctx.exposure_step * 2;
				*min_shutter = ctx->s_ctx.exposure_min * 2;
				break;
			case HDR_RAW_STAGGER_3EXP:
				*exposure_step = ctx->s_ctx.exposure_step*3;
				*min_shutter = ctx->s_ctx.exposure_min * 3;
				break;
			case HDR_NONE:
				if (ctx->s_ctx.mode[scenario_id].coarse_integ_step &&
					ctx->s_ctx.mode[scenario_id].min_exposure_line) {
					*exposure_step = ctx->s_ctx.mode[scenario_id].coarse_integ_step;
					*min_shutter = ctx->s_ctx.mode[scenario_id].min_exposure_line;
				} else {
					*exposure_step = ctx->s_ctx.exposure_step;
					*min_shutter = ctx->s_ctx.exposure_min;
				}
				break;
			default:
				*exposure_step = ctx->s_ctx.exposure_step;
				*min_shutter = ctx->s_ctx.exposure_min;
				break;
		}
	} else {
		DRV_LOG(ctx, "over sensor_mode_num[%d], use default", ctx->s_ctx.sensor_mode_num);
		*exposure_step = ctx->s_ctx.exposure_step;
		*min_shutter = ctx->s_ctx.exposure_min;
	}
	DRV_LOG(ctx, "scenario_id[%d] exposure_step[%llu] min_shutter[%llu]\n", scenario_id, *exposure_step, *min_shutter);
}

void omegac1main_get_min_shutter_by_scenario_adapter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *) para;
	omegac1main_get_min_shutter_by_scenario(ctx,
		(enum SENSOR_SCENARIO_ID_ENUM)*(feature_data),
		feature_data + 1, feature_data + 2);
}

static void omegac1main_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	enum SENSOR_SCENARIO_ID_ENUM scenario_id;
	struct mtk_hdr_ae *ae_ctrl = NULL;
	u64 *feature_data = (u64 *) para;
	u32 fine_integ_line = 0;
	u32 cit_step = 0;
	u32 rg_shutter = 0;
	u32 prsh_length_lines = 0;

	if (feature_data == NULL) {
		DRV_LOGE(ctx, "input scenario is null!");
		return;
	}
	scenario_id = *feature_data;
	if ((feature_data + 1) != NULL)
		ae_ctrl = (struct mtk_hdr_ae *)((uintptr_t)(*(feature_data + 1)));
	else
		DRV_LOGE(ctx, "no ae_ctrl input");

	check_current_scenario_id_bound(ctx);
	DRV_LOG(ctx, "E: set seamless switch %u %u\n", ctx->current_scenario_id, scenario_id);
	if (!ctx->extend_frame_length_en)
		DRV_LOGE(ctx, "please extend_frame_length before seamless_switch!\n");
	ctx->extend_frame_length_en = FALSE;

	if (scenario_id >= ctx->s_ctx.sensor_mode_num) {
		DRV_LOGE(ctx, "invalid sid:%u, mode_num:%u\n",
			scenario_id, ctx->s_ctx.sensor_mode_num);
		return;
	}
	if (ctx->s_ctx.mode[scenario_id].seamless_switch_group == 0 ||
		ctx->s_ctx.mode[scenario_id].seamless_switch_group !=
			ctx->s_ctx.mode[ctx->current_scenario_id].seamless_switch_group) {
		DRV_LOGE(ctx, "seamless_switch not supported\n");
		return;
	}
	if (ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_table == NULL) {
		DRV_LOGE(ctx, "Please implement seamless_switch setting\n");
		return;
	}

	if (omegac1main_comp_params[ctx->current_scenario_id].enable &&
		omegac1main_comp_params[ctx->current_scenario_id].clock_vtpxck > 0 &&
		omegac1main_comp_params[ctx->current_scenario_id].cal_fn) {
		omegac1main_comp_params[ctx->current_scenario_id].cal_fn(ctx,
			(u32)scenario_id, &prsh_length_lines);
	}

	ctx->is_seamless = TRUE;
	update_mode_info(ctx, scenario_id);

	subdrv_i2c_wr_u8(ctx, 0x0104, 0x01);
	subdrv_i2c_wr_u8(ctx, 0x3010, 0x02);
	i2c_table_write(ctx,
		ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_table,
		ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_len);
	if (ae_ctrl) {
		switch (ctx->s_ctx.mode[scenario_id].hdr_mode) {
		case HDR_RAW_STAGGER_2EXP:
			set_multi_shutter_frame_length(ctx, (u64*)&ae_ctrl->exposure, 2, 0);
			set_multi_gain(ctx, (u32*)&ae_ctrl->gain, 2);
			break;
		default:
			set_shutter(ctx, ae_ctrl->exposure.le_exposure);
			set_gain(ctx, ae_ctrl->gain.le_gain);
			break;
		}
	}
	DRV_LOG(ctx, "write seamless switch para done\n");

	if (ae_ctrl && prsh_length_lines > 0) {
		rg_shutter = ae_ctrl->exposure.le_exposure;
		switch (ctx->s_ctx.mode[scenario_id].hdr_mode) {
		case HDR_RAW_STAGGER_2EXP:
			rg_shutter /= 2;
			break;
		case HDR_RAW_STAGGER_3EXP:
			rg_shutter /= 3;
			break;
		default:
			fine_integ_line = ctx->s_ctx.mode[ctx->current_scenario_id].fine_integ_line;
			cit_step = ctx->s_ctx.mode[ctx->current_scenario_id].coarse_integ_step;
			rg_shutter = FINE_INTEG_CONVERT(rg_shutter, fine_integ_line);
			rg_shutter = max(rg_shutter, ctx->s_ctx.exposure_min);
			rg_shutter = min(rg_shutter, ctx->s_ctx.exposure_max);
			if (cit_step)
				rg_shutter = round_up(rg_shutter, cit_step);
			break;
		}

		if (prsh_length_lines > rg_shutter + 1) {
			prsh_length_lines -= rg_shutter;
			if (ctx->s_ctx.mode[scenario_id].hdr_mode == HDR_NONE)
				prsh_length_lines = round_up(prsh_length_lines, 2);
			DRV_LOG(ctx, "rg_shutter(%u) prsh_length_lines(%u)\n", rg_shutter, prsh_length_lines);
			subdrv_i2c_wr_u8(ctx, 0x3036, 0x01);
			subdrv_i2c_wr_u8(ctx, 0x3039, prsh_length_lines >> 16 & 0xFF);
			subdrv_i2c_wr_u8(ctx, 0x303a, prsh_length_lines >> 8  & 0xFF);
			subdrv_i2c_wr_u8(ctx, 0x303b, prsh_length_lines & 0xFF);
		}
	}

	subdrv_i2c_wr_u8(ctx, 0x0104, 0x00);

	ctx->fast_mode_on = TRUE;
	ctx->ref_sof_cnt = ctx->sof_cnt;
	ctx->is_seamless = FALSE;
	DRV_LOG(ctx, "X: set seamless switch done\n");

}

static void omegac1main_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);

	if (mode != ctx->test_pattern) {
		LOG_INF("mode(%u->%u)\n", ctx->test_pattern, mode);

		if (mode) {
		/* 1:Solid Color 2:Color Bar 5:Black */
			switch (mode) {
			case 5:
				subdrv_i2c_wr_u8(ctx, 0x020E, 0x00); /* dig_gain = 0 */
				subdrv_i2c_wr_u8(ctx, 0x0218, 0x00);
				subdrv_i2c_wr_u8(ctx, 0x3015, 0x00);
				break;
			default:
				subdrv_i2c_wr_u8(ctx, 0x0601, mode);
				break;
			}
		} else {
			if (ctx->test_pattern) {
				subdrv_i2c_wr_u8(ctx, 0x0601, 0x00); /*No pattern*/
				subdrv_i2c_wr_u8(ctx, 0x020E, 0x01);
				subdrv_i2c_wr_u8(ctx, 0x0218, 0x01);
				subdrv_i2c_wr_u8(ctx, 0x3015, 0x40);
			}
		}

		ctx->test_pattern = mode;
	}
}

static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id)
{
	memcpy(&(ctx->s_ctx), &static_ctx, sizeof(struct subdrv_static_ctx));
	subdrv_ctx_init(ctx);
	ctx->i2c_client = i2c_client;
	ctx->i2c_write_id = i2c_write_id;
	return 0;
}

static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt)
{
	DRV_LOG(ctx, "sof_cnt(%u) ctx->ref_sof_cnt(%u) ctx->fast_mode_on(%d)",
		sof_cnt, ctx->ref_sof_cnt, ctx->fast_mode_on);
	ctx->sof_cnt = sof_cnt;
	if (ctx->fast_mode_on && (sof_cnt > ctx->ref_sof_cnt)) {
		ctx->fast_mode_on = FALSE;
		ctx->ref_sof_cnt = 0;
		DRV_LOG(ctx, "seamless_switch disabled.");
		set_i2c_buffer(ctx, 0x3010, 0x00);
		set_i2c_buffer(ctx, 0x3036, 0x00);
		commit_i2c_buffer(ctx);
	}
	return 0;
}

void get_sensor_cali(void* arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;

	u16 idx = 0;
	u8 support = FALSE;
	u8 *buf = NULL;
	u16 size = 0;
	u16 addr = 0;
	u8 qsc_is_valid = 0;
	struct eeprom_info_struct *info = ctx->s_ctx.eeprom_info;

	/* Probe EEPROM device */
	if (!probe_eeprom(ctx))
		return;

	idx = ctx->eeprom_index;

	/* QSC data */
	support = info[idx].qsc_support;
	size = info[idx].qsc_size;
	addr = info[idx].addr_qsc;
	buf = info[idx].qsc_table;
	if (support && size > 0) {
		// Check QSC validation
		qsc_is_valid = i2c_read_eeprom(ctx, OTP_QSC_VALID_ADDR);
		if (qsc_is_valid != 0x01) {
			DRV_LOGE(ctx, "QSC data is invalid, flag(%02x)", qsc_is_valid);
		} else if (info[idx].preload_qsc_table == NULL) {
			info[idx].preload_qsc_table = kmalloc(size, GFP_KERNEL);
			if (buf == NULL) {
				if (!read_cmos_eeprom_p8(ctx, addr, info[idx].preload_qsc_table, size)) {
					DRV_LOGE(ctx, "preload QSC data failed");
				}
			} else {
				memcpy(info[idx].preload_qsc_table, buf, size);
			}
			DRV_LOG(ctx, "preload QSC data %u bytes", size);
		} else {
			DRV_LOG(ctx, "QSC data is already preloaded %u bytes", size);
		}
	}

	ctx->is_read_preload_eeprom = 1;
}

static void comp_mode_tran_time_cal1(struct subdrv_ctx *ctx, u32 scenario_id, u32* prsh) {
	#define SYSTEM_USED_LINES1 (96UL)
	#define SYSTEM_DELAY1      (189UL)
	u64 frame_duration = 0;
	u64 data_delay = 0;
	u64 system_delay = 0;
	u64 current_tline = 0;
	u64 tline = 0;

	if (!prsh) {
		DRV_LOGE(ctx, "prsh param is NULL");
		return;
	}

	*prsh = 0U;
	if (omegac1main_comp_params[ctx->current_scenario_id].clock_vtpxck == 0) {
		DRV_LOG(ctx, "invalid params");
		return;
	}

	frame_duration = 1000000000UL / ctx->current_fps * 10;
	current_tline = 1000000000UL * ctx->s_ctx.mode[ctx->current_scenario_id].linelength /
		ctx->s_ctx.mode[ctx->current_scenario_id].pclk;
	tline = 1000000000UL * ctx->s_ctx.mode[scenario_id].linelength /
		ctx->s_ctx.mode[scenario_id].pclk;
	data_delay = (ctx->s_ctx.mode[ctx->current_scenario_id].imgsensor_winsize_info.h2_tg_size +
		SYSTEM_USED_LINES1) * current_tline;
	system_delay = SYSTEM_DELAY1 * 1000 * 1000 * 10 /
		omegac1main_comp_params[ctx->current_scenario_id].clock_vtpxck;
	if (frame_duration <= data_delay + system_delay) {
		DRV_LOGE(ctx, "invalid parameter");
		return;
	}

	*prsh = (frame_duration - data_delay - system_delay) / tline;
	if (ctx->s_ctx.mode[scenario_id].hdr_mode == HDR_RAW_STAGGER_2EXP) {
		*prsh = *prsh / 2;
	}

	DRV_LOG(ctx, "frame_duration(%llu), current_tline(%llu), tline(%llu), "
		"data_delay(%llu) system_delay(%llu) prsh(%u)\n", frame_duration,
		current_tline, tline, data_delay, system_delay, *prsh);
}

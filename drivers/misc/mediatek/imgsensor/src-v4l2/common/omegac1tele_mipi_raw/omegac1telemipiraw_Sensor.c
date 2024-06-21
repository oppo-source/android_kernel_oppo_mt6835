// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 omegac1telemipiraw_Sensor.c
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
#include "omegac1telemipiraw_Sensor.h"

#define PFX "omegac1tele_camera_sensor"
#define LOG_INF(format, args...) pr_err(PFX "[%s] " format, __func__, ##args)
#define OMEGAC1TELE_EEPROM_READ_ID     (0xA1)
#define OMEGAC1TELE_EEPROM_WRITE_ID    (0xA0)
#define OMEGAC1TELE_MAX_OFFSET         (0x8000)

#define OTP_SIZE    (0x8000)
#ifdef  EEPROM_WRITE_DATA_MAX_LENGTH
#undef  EEPROM_WRITE_DATA_MAX_LENGTH
#endif
#define EEPROM_WRITE_DATA_MAX_LENGTH      (64)
#define OMEGAC1TELE_STEREO_MT_START_ADDR  (0x3150)
#define OMEGAC1TELE_AESYNC_START_ADDR     (0x3800)
#define OMEGAC1TELE_EEPROM_LOCK_REGISTER  (0xE000)
#define FRAME_FRAMEDURATION_MS             (33)
static u16 module_flag = 0;
static bool bNeedSetNormalMode = FALSE;
static u8 bRmscMode = 0;
static kal_uint8 otp_data_checksum[OTP_SIZE] = {0};

#define MAX_BURST_LEN  2048
static u8 * msg_buf = NULL;

static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
static void omegac1tele_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void omegac1tele_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void omegac1tele_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void omegac1tele_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void omegac1tele_get_eeprom_comdata(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void omegac1tele_set_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void omegac1tele_get_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void omegac1tele_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void omegac1tele_streaming_suspend(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void omegac1tele_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len);

static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static int open(struct subdrv_ctx *ctx);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int get_sensor_temperature(void *arg);

static void omegac1tele_set_gain_convert(struct subdrv_ctx *ctx, u32 gain);
static void omegac1tele_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void omegac1tele_set_multi_gain(struct subdrv_ctx *ctx, u32 *gains, u16 exp_cnt);
static void omegac1tele_set_hdr_tri_gain(struct subdrv_ctx *ctx, u64 *gains, u16 exp_cnt);
static void omegac1tele_set_hdr_tri_gain2(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void omegac1tele_set_hdr_tri_gain3(struct subdrv_ctx *ctx, u8 *para, u32 *len);

static void omegac1tele_set_shutter_convert(struct subdrv_ctx *ctx, u64 shutter);
static void omegac1tele_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void omegac1tele_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void omegac1tele_set_shutter_frame_length_convert(struct subdrv_ctx *ctx, u64 shutter, u32 frame_length);
static void omegac1tele_set_multi_shutter_frame_length(struct subdrv_ctx *ctx, u64 *shutters, u16 exp_cnt, u16 frame_length);
static void omegac1tele_set_multi_shutter_frame_length_ctrl(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void omegac1tele_set_hdr_tri_shutter2(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void omegac1tele_set_hdr_tri_shutter3(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void omegac1tele_set_max_framerate_by_scenario(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void omegac1tele_extend_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void omegac1tele_set_video_mode(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void omegac1tele_set_awb_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size);
static void omegac1tele_set_sensor_rmsc_mode(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int omegac1tele_i2c_burst_wr_regs_u16(struct subdrv_ctx *ctx, u16 * list, u32 len);
static int adapter_i2c_burst_wr_regs_u16(struct subdrv_ctx * ctx,
		u16 addr, u16 *list, u32 len);
static void omegac1tele_lens_pos_writeback(struct subdrv_ctx *ctx);
static void omegac1tele_set_register(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void omegac1tele_get_register(struct subdrv_ctx *ctx, u8 *para, u32 *len);
/* STRUCT */

static struct oplus_get_eeprom_common_data omegac1tele_eeprom_common_data = {0};
static kal_uint16 g_af_code_macro    = 0;
static kal_uint16 g_af_code_infinity = 0;
static kal_uint16 g_af_code_middle   = 0;

static struct eeprom_map_info omegac1tele_eeprom_info[] = {
	{ EEPROM_META_MODULE_ID, 0x0000, 0x0010, 0x0011, 2, true },
	{ EEPROM_META_SENSOR_ID, 0x0006, 0x0010, 0x0011, 2, true },
	{ EEPROM_META_LENS_ID, 0x0008, 0x0010, 0x0011, 2, true },
	{ EEPROM_META_VCM_ID, 0x000A, 0x0010, 0x0011, 2, true },
	{ EEPROM_META_MIRROR_FLIP, 0x000E, 0x0010, 0x0011, 1, true },
	{ EEPROM_META_MODULE_SN, 0x00B0, 0x00C7, 0x00C8, 23, true },
	{ EEPROM_META_AF_CODE, 0x0092, 0x0098, 0x0099, 6, true },
	{ EEPROM_META_AF_FLAG, 0x0098, 0x0098, 0x0099, 1, true },
	{ EEPROM_META_STEREO_DATA, 0, 0, 0, 0, false },
	{ EEPROM_META_STEREO_MW_MAIN_DATA, 0, 0, 0, 0, false },
	{ EEPROM_META_STEREO_MT_MAIN_DATA, OMEGAC1TELE_STEREO_MT_START_ADDR, 0x37E9, 0x37EA, CALI_DATA_SLAVE_LENGTH, false },
};

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, omegac1tele_set_test_pattern},
	{SENSOR_FEATURE_SET_TEST_PATTERN_DATA, omegac1tele_set_test_pattern_data},
	{SENSOR_FEATURE_SEAMLESS_SWITCH, omegac1tele_seamless_switch},
	{SENSOR_FEATURE_CHECK_SENSOR_ID, omegac1tele_check_sensor_id},
	{SENSOR_FEATURE_GET_EEPROM_COMDATA, omegac1tele_get_eeprom_comdata},
	{SENSOR_FEATURE_SET_SENSOR_OTP, omegac1tele_set_eeprom_calibration},
	{SENSOR_FEATURE_GET_EEPROM_STEREODATA, omegac1tele_get_eeprom_calibration},
	{SENSOR_FEATURE_GET_SENSOR_OTP_ALL, omegac1tele_get_otp_checksum_data},
	{SENSOR_FEATURE_SET_STREAMING_SUSPEND, omegac1tele_streaming_suspend},
	{SENSOR_FEATURE_SET_STREAMING_RESUME, omegac1tele_streaming_resume},
	{SENSOR_FEATURE_SET_GAIN, omegac1tele_set_gain},
	{SENSOR_FEATURE_SET_DUAL_GAIN, omegac1tele_set_hdr_tri_gain2},
	{SENSOR_FEATURE_SET_HDR_TRI_GAIN, omegac1tele_set_hdr_tri_gain3},
	{SENSOR_FEATURE_SET_ESHUTTER, omegac1tele_set_shutter},
	{SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME, omegac1tele_set_shutter_frame_length},
	{SENSOR_FEATURE_SET_HDR_SHUTTER, omegac1tele_set_hdr_tri_shutter2},
	{SENSOR_FEATURE_SET_HDR_TRI_SHUTTER, omegac1tele_set_hdr_tri_shutter3},
	{SENSOR_FEATURE_SET_MULTI_SHUTTER_FRAME_TIME, omegac1tele_set_multi_shutter_frame_length_ctrl},
	{SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO, omegac1tele_set_max_framerate_by_scenario},
	{SENSOR_FEATURE_SET_SEAMLESS_EXTEND_FRAME_LENGTH, omegac1tele_extend_frame_length},
	{SENSOR_FEATURE_SET_VIDEO_MODE, omegac1tele_set_video_mode},
	{SENSOR_FEATURE_SET_SENSOR_RMSC_MODE, omegac1tele_set_sensor_rmsc_mode},
	{SENSOR_FEATURE_SET_AWB_GAIN, omegac1tele_set_awb_gain},
	{SENSOR_FEATURE_SET_REGISTER, omegac1tele_set_register},
	{SENSOR_FEATURE_GET_REGISTER, omegac1tele_get_register},
};

static struct eeprom_info_struct eeprom_info[] = {
	{
		.header_id = 0x01A9010B,
		.addr_header_id = 0x00000006,
		.i2c_write_id = 0xA0,
	},
};

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
		/* <pre> <cap> <normal_video> <hs_video> <slim_video> */
		{0, 0}, {0, 0}, {0, 384}, {0, 384}, {0, 0},
		/* <cus1> <cus2> <cus3> <cus4> <cus5> */
		{0, 0}, {0, 0}, {0, 0}, {0, 384}, {0, 0},
		/* <cus6> <cus7> <cus8> <cus9> <cus10> */
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		/* <cus11> <cus12> <cus13> <cus14> <cus15> */
		{0, 0}, {416, 618}, {416, 618}, {0, 0}, {0, 0},
		/* <cus16> <cus17> */
		{0, 0}, {416, 312}
	},
	.iMirrorFlip = 0,
	.i4FullRawW = 4096,
	.i4FullRawH = 3072,
	.i4VCPackNum = 1,
	.i4ModeIndex = 0x3,
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
		/* <pre> <cap> <normal_video> <hs_video> <slim_video> */
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 192},
		/* <cus1> <cus2> <cus3> <cus4> <cus5> */
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		/* <cus6> <cus7> <cus8> <cus9> <cus10> */
		{0, 192}, {0, 192}, {0, 192}, {0, 0}, {0, 0},
		/* <cus11> <cus12> <cus13> <cus14> <cus15> */
		{0, 192}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		/* <cus16> <cus17> */
		{0, 0}, {0, 0}
	},
	.iMirrorFlip = 0,
	.i4FullRawW = 2048,
	.i4FullRawH = 1536,
	.i4VCPackNum = 1,
	.i4ModeIndex = 0x3,
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
		/* <pre> <cap> <normal_video> <hs_video> <slim_video> */
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		/* <cus1> <cus2> <cus3> <cus4> <cus5> */
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {2048, 1536},
		/* <cus6> <cus7> <cus8> <cus9> <cus10> */
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		/* <cus11> <cus12> <cus13> <cus14> <cus15> */
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {2048, 1536},
		/* <cus16> <cus17> */
		{0, 0}, {0, 0}
	},
	.iMirrorFlip = 0,
	.i4FullRawW = 8192,
	.i4FullRawH = 6144,
	.i4VCPackNum = 1,
	.i4ModeIndex = 0x3,
	.sPDMapInfo[0] = {
		.i4PDPattern = 1,//all-pd
		.i4BinFacX = 4,
		.i4BinFacY = 8,
		.i4PDRepetition = 0,
		.i4PDOrder = {1}, //R=1, L=0
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_prev_cap[] = {
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
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 2304,
			.user_data_desc = VC_STAGGER_NE,
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

static struct mtk_mbus_frame_desc_entry frame_desc_hs[] = {
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
			.channel = 0,
			.data_type = 0x30,
			.hsize = 4096,
			.vsize = 576,
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

static struct mtk_mbus_frame_desc_entry frame_desc_cus2[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 2048,
			.vsize = 1536,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 2048,
			.vsize = 384,
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
			.hsize = 8192,
			.vsize = 6144,
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
			.vsize = 384,
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

static struct mtk_mbus_frame_desc_entry frame_desc_cus7[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 2048,
			.vsize = 1152,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus8[] = {
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
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 2048,
			.vsize = 1152,
			.user_data_desc = VC_STAGGER_ME,
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

static struct mtk_mbus_frame_desc_entry frame_desc_cus9[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 2048,
			.vsize = 1536,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 2048,
			.vsize = 384,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus10[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 2048,
			.vsize = 1536,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 2048,
			.vsize = 384,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus11[] = {
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

static struct mtk_mbus_frame_desc_entry frame_desc_cus12[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 3264,
			.vsize = 1836,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 3264,
			.vsize = 458,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus13[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 3264,
			.vsize = 1836,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 3264,
			.vsize = 458,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus14[] = {
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
			.hsize = 4096,
			.vsize = 768,
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
			.vsize = 384,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus16[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 680,
			.vsize = 512,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus17[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 3264,
			.vsize = 2448,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 3264,
			.vsize = 612,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
		},
	},
};

static struct subdrv_mode_struct mode_struct[] = {
	{/*4sum12.5Mp_30FPS_4096x3072*/
		.frame_desc = frame_desc_prev_cap,
		.num_entries = ARRAY_SIZE(frame_desc_prev_cap),
		.mode_setting_table = omegac1tele_preview_capture_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1tele_preview_capture_setting),
		.seamless_switch_group = 2,
		.seamless_switch_mode_setting_table = omegac1tele_bining_12_5M_seamless_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(omegac1tele_bining_12_5M_seamless_setting),
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 920000000,
		.linelength = 4784,
		.framelength = 6408,
		.max_framerate = 300,
		.mipi_pixel_rate = 1324800000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.exposure_margin = 12,
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
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 80,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 30,
		},
	},
	{ /*4sum12.5Mp_30FPS_4096x3072*/
		.frame_desc = frame_desc_prev_cap,
		.num_entries = ARRAY_SIZE(frame_desc_prev_cap),
		.mode_setting_table = omegac1tele_preview_capture_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1tele_preview_capture_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 920000000,
		.linelength = 4784,
		.framelength = 6408,
		.max_framerate = 300,
		.mipi_pixel_rate = 1324800000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.exposure_margin = 12,
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
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 80,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},
	{/*4sum4k_30FPS_4096x2304*/
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = omegac1tele_normal_video_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1tele_normal_video_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = omegac1tele_bining_4k_30fps_seamless_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(omegac1tele_bining_4k_30fps_seamless_setting),
		.hdr_group = 1,
		.hdr_mode = HDR_NONE,
		.pclk = 920000000,
		.linelength = 4784,
		.framelength = 6408,
		.max_framerate = 300,
		.mipi_pixel_rate = 1324800000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.exposure_margin = 12,
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
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 80,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 30,
		},
	},
	{/*4sum4k_60FPS_4096x2304*/
		.frame_desc = frame_desc_hs,
		.num_entries = ARRAY_SIZE(frame_desc_hs),
		.mode_setting_table = omegac1tele_hs_video_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1tele_hs_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 920000000,
		.linelength = 4784,
		.framelength = 3204,
		.max_framerate = 600,
		.mipi_pixel_rate = 1280000000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.exposure_margin = 12,
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
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 80,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 60,
		},
	},
	{/*4sum2bin_120FPS_2048x1152*/
		.frame_desc = frame_desc_slim,
		.num_entries = ARRAY_SIZE(frame_desc_slim),
		.mode_setting_table = omegac1tele_slim_video_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1tele_slim_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 920000000,
		.linelength = 4784,
		.framelength = 1602,
		.max_framerate = 1200,
		.mipi_pixel_rate = 668800000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.exposure_margin = 8,
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
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 80,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 120,
		},
	},
	{/*4sum12.5Mp_24FPS_4096x3072*/
		.frame_desc = frame_desc_cus1,
		.num_entries = ARRAY_SIZE(frame_desc_cus1),
		.mode_setting_table = omegac1tele_custom1_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1tele_custom1_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 920000000,
		.linelength = 4784,
		.framelength = 8008,
		.max_framerate = 240,
		.mipi_pixel_rate = 1280000000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.exposure_margin = 12,
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
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 80,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 24,
		},
	},
	{/*4sum2bin_24FPS_2048x1536*/
		.frame_desc = frame_desc_cus2,
		.num_entries = ARRAY_SIZE(frame_desc_cus2),
		.mode_setting_table = omegac1tele_custom2_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1tele_custom2_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 920000000,
		.linelength = 4852,
		.framelength = 7896,
		.max_framerate = 240,
		.mipi_pixel_rate = 668800000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.exposure_margin = 8,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 2048,
			.scale_h = 1536,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2048,
			.h1_size = 1536,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2048,
			.h2_tg_size = 1536,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_v2h2,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 80,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},
	{/*full50Mp_15FPS_8192x6144_remosaicON*/
		.frame_desc = frame_desc_cus3,
		.num_entries = ARRAY_SIZE(frame_desc_cus3),
		.mode_setting_table = omegac1tele_custom3_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1tele_custom3_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 920000000,
		.linelength = 9600,
		.framelength = 6346,
		.max_framerate = 150,
		.mipi_pixel_rate = 902400000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 16,
		.exposure_margin = 48,
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
		.ae_binning_ratio = 1240,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 16,
		.sensor_setting_info = {
			.sensor_scenario_usage = RMSC_MASK,
			.equivalent_fps = 15,
		},
	},
	{/*4sum4k_30FPS_4096x2304_2expSHDR*/
		.frame_desc = frame_desc_cus4,
		.num_entries = ARRAY_SIZE(frame_desc_cus4),
		.mode_setting_table = omegac1tele_custom4_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1tele_custom4_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = omegac1tele_bining_4k_30fps_shdr_seamless_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(omegac1tele_bining_4k_30fps_shdr_seamless_setting),
		.hdr_group = 1,
		.hdr_mode = HDR_RAW_STAGGER_2EXP,
		.pclk = 920000000,
		.linelength = 6000,
		.framelength = 5110,
		.max_framerate = 300,
		.mipi_pixel_rate = 1324800000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 2,
		.min_exposure_line = 8,
		.exposure_margin = 24,
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
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 80,
		.sensor_setting_info = {
			.sensor_scenario_usage = HDR_RAW_STAGGER_2EXP_MASK,
			.equivalent_fps = 30,
		},
	},
	{/*fullcrop12.5Mp_30FPS_4096x3072_remosaicON*/
		.frame_desc = frame_desc_cus5,
		.num_entries = ARRAY_SIZE(frame_desc_cus5),
		.mode_setting_table = omegac1tele_custom5_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1tele_custom5_setting),
		.seamless_switch_group = 2,
		.seamless_switch_mode_setting_table = omegac1tele_Fullcrop_12_5M_remosaic_on_seamless_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(omegac1tele_Fullcrop_12_5M_remosaic_on_seamless_setting),
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 920000000,
		.linelength = 9200,
		.framelength = 3332,
		.max_framerate = 300,
		.mipi_pixel_rate = 1324800000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 16,
		.exposure_margin = 48,
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
		.ae_binning_ratio = 1240,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 16,
		.sensor_setting_info = {
			.sensor_scenario_usage = INSENSORZOOM_MASK,
			.equivalent_fps = 30,
		},
	},
	{/*4sum2bin_30FPS_2048x1152*/
		.frame_desc = frame_desc_cus6,
		.num_entries = ARRAY_SIZE(frame_desc_cus6),
		.mode_setting_table = omegac1tele_custom6_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1tele_custom6_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 920000000,
		.linelength = 4784,
		.framelength = 6408,
		.max_framerate = 300,
		.mipi_pixel_rate = 1324800000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.exposure_margin = 8,
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
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 80,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},
	{/*4sum2bin_240FPS_2048x1152*/
		.frame_desc = frame_desc_cus7,
		.num_entries = ARRAY_SIZE(frame_desc_cus7),
		.mode_setting_table = omegac1tele_custom7_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1tele_custom7_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 920000000,
		.linelength = 2944,
		.framelength = 1302,
		.max_framerate = 2400,
		.mipi_pixel_rate = 796800000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.exposure_margin = 8,
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
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = &imgsensor_pd_info_v2h2,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 80,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 240,
		},
	},
	{/*4sumFHD_30FPS_2048x1152_2expSHDR*/
		.frame_desc = frame_desc_cus8,
		.num_entries = ARRAY_SIZE(frame_desc_cus8),
		.mode_setting_table = omegac1tele_custom8_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1tele_custom8_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = 2,
		.hdr_mode = HDR_RAW_STAGGER_2EXP,
		.pclk = 920000000,
		.linelength = 6000,
		.framelength = 5110,
		.max_framerate = 300,
		.mipi_pixel_rate = 1324800000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 2,
		.min_exposure_line = 16,
		.exposure_margin = 24,
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
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 80,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},
	{/*4sum2bin_30FPS_2048x1536*/
		.frame_desc = frame_desc_cus9,
		.num_entries = ARRAY_SIZE(frame_desc_cus9),
		.mode_setting_table = omegac1tele_custom9_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1tele_custom9_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 920000000,
		.linelength = 4784,
		.framelength = 6408,
		.max_framerate = 300,
		.mipi_pixel_rate = 668800000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.exposure_margin = 8,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 2048,
			.scale_h = 1536,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2048,
			.h1_size = 1536,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2048,
			.h2_tg_size = 1536,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_v2h2,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 80,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},
	{/*4sum2bin_15FPS_2048x1536*/
		.frame_desc = frame_desc_cus10,
		.num_entries = ARRAY_SIZE(frame_desc_cus10),
		.mode_setting_table = omegac1tele_custom10_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1tele_custom10_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 920000000,
		.linelength = 4784,
		.framelength = 12816,
		.max_framerate = 150,
		.mipi_pixel_rate = 668800000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.exposure_margin = 8,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 2048,
			.scale_h = 1536,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2048,
			.h1_size = 1536,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2048,
			.h2_tg_size = 1536,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_v2h2,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 80,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},
	{/*4sum2bin_15FPS_2048x1152*/
		.frame_desc = frame_desc_cus11,
		.num_entries = ARRAY_SIZE(frame_desc_cus11),
		.mode_setting_table = omegac1tele_custom11_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1tele_custom11_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 920000000,
		.linelength = 4852,
		.framelength = 12640,
		.max_framerate = 150,
		.mipi_pixel_rate = 668800000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.exposure_margin = 8,
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
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 80,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},
	{/* 4sum4k_30FPS_3264x1836 */
		.frame_desc = frame_desc_cus12,
		.num_entries = ARRAY_SIZE(frame_desc_cus12),
		.mode_setting_table = omegac1tele_custom12_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1tele_custom12_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 920000000,
		.linelength = 4848,
		.framelength = 6309,
		.max_framerate = 300,
		.mipi_pixel_rate = 1280000000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.exposure_margin = 12,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
			.scale_w = 4096,
			.scale_h = 2304,
			.x1_offset = 416,
			.y1_offset = 234,
			.w1_size = 3264,
			.h1_size = 1836,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 1836,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 80,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},
	{/* 4sum4k_15FPS_3264x1836 */
		.frame_desc = frame_desc_cus13,
		.num_entries = ARRAY_SIZE(frame_desc_cus13),
		.mode_setting_table = omegac1tele_custom13_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1tele_custom13_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 920000000,
		.linelength = 4848,
		.framelength = 12640,
		.max_framerate = 150,
		.mipi_pixel_rate = 1280000000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.exposure_margin = 12,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
			.scale_w = 4096,
			.scale_h = 2304,
			.x1_offset = 416,
			.y1_offset = 234,
			.w1_size = 3264,
			.h1_size = 1836,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 1836,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 80,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},
	{/*4sum12.5M_30FPS_4096x3072_2expSHDR*/
		.frame_desc = frame_desc_cus14,
		.num_entries = ARRAY_SIZE(frame_desc_cus14),
		.mode_setting_table = omegac1tele_custom14_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1tele_custom14_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = 3,
		.hdr_mode = HDR_RAW_STAGGER_2EXP,
		.pclk = 920000000,
		.linelength = 4784,
		.framelength = 6408,
		.max_framerate = 300,
		.mipi_pixel_rate = 1324800000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 2,
		.min_exposure_line = 8,
		.exposure_margin = 24,
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
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 80,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},
	{/*fullcrop12.5Mp_30FPS_4096x3072_remosaic0ff*/
		.frame_desc = frame_desc_cus15,
		.num_entries = ARRAY_SIZE(frame_desc_cus15),
		.mode_setting_table = omegac1tele_custom15_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1tele_custom15_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 920000000,
		.linelength = 9200,
		.framelength = 3332,
		.max_framerate = 300,
		.mipi_pixel_rate = 1324800000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 16,
		.exposure_margin = 48,
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
		.ae_binning_ratio = 1240,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 16,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},
	{/*4sum2bin_30FPS_680x512*/
		.frame_desc = frame_desc_cus16,
		.num_entries = ARRAY_SIZE(frame_desc_cus16),
		.mode_setting_table = omegac1tele_custom16_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1tele_custom16_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 920000000,
		.linelength = 2944,
		.framelength = 10412,
		.max_framerate = 300,
		.mipi_pixel_rate = 312000000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.exposure_margin = 8,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
			.scale_w = 2048,
			.scale_h = 1152,
			.x1_offset = 684,
			.y1_offset = 320,
			.w1_size = 680,
			.h1_size = 512,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 680,
			.h2_tg_size = 512,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 120,
		},
		.ana_gain_max = BASEGAIN * 80,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},
	{/* 4sum4k_30FPS_3264x2448 */
		.frame_desc = frame_desc_cus17,
		.num_entries = ARRAY_SIZE(frame_desc_cus17),
		.mode_setting_table = omegac1tele_custom17_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1tele_custom17_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_group = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.pclk = 920000000,
		.linelength = 4848,
		.framelength = 6312,
		.max_framerate = 300,
		.mipi_pixel_rate = 1280000000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 4,
		.exposure_margin = 12,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 416,
			.y1_offset = 312,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.ana_gain_max = BASEGAIN * 80,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = OMEGAC1TELE_SENSOR_ID,
	.reg_addr_sensor_id = {0x0000, 0x0001},
	.i2c_addr_table = {0x5a, 0xff},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_16,
	.eeprom_info = eeprom_info,
	.eeprom_num = ARRAY_SIZE(eeprom_info),
	.resolution = {8192, 6144},
	.mirror = IMAGE_NORMAL,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_Gr,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 80,
	.ana_gain_type = 2, //0-SONY; 1-OV; 2 - SUMSUN; 3 -HYNIX; 4 -GC
	.ana_gain_step = 2,
	.ana_gain_table = omegac1tele_ana_gain_table,
	.ana_gain_table_size = sizeof(omegac1tele_ana_gain_table),
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 4,
	.exposure_max = (0xffff * 128) - 4,
	.exposure_step = 1,
	.exposure_margin = 24, //tentative
	.frame_length_max = 0xffff,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 3000000,

	.pdaf_type = PDAF_SUPPORT_CAMSV_QPD,
	.hdr_type = HDR_SUPPORT_STAGGER_FDOL,
	.seamless_switch_support = FALSE,
	.temperature_support = TRUE,
	.g_temp = get_sensor_temperature,
	.g_gain2reg = get_gain2reg,
//	.g_cali = get_sensor_cali,
	.s_gph = set_group_hold,

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = 0x0101,
	.reg_addr_exposure ={
			{0x0202, 0x0203}, //Short exposure
			{0x0202, 0x0203},
			{0x0226, 0x0227}, //Long exposure
	},
	.long_exposure_support = TRUE,
	.reg_addr_exposure_lshift = 0x0704,
	.reg_addr_ana_gain = {
			{0x0204, 0x0205}, //Short Gain
			{0x0204, 0x0205},
			{0x0206, 0x0207}, //Long Gain
	},
	.reg_addr_frame_length = {0x0340, 0x0341},
	.reg_addr_temp_en = PARAM_UNDEFINED,
	.reg_addr_temp_read = 0x0020,
	.reg_addr_auto_extend = PARAM_UNDEFINED, //0x0335,
	.reg_addr_frame_count = 0x0005,
//	.init_setting_table = omegac1tele_sensor_init_setting,
//	.init_setting_len =  ARRAY_SIZE(omegac1tele_sensor_init_setting),
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),

	.chk_s_off_before_s_on = 1,
	.chk_s_off_before_control = 1,
	.chk_s_off_after_s_off = 0,

	.checksum_value = 0x350174bc,
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
	.update_sof_cnt = common_update_sof_cnt,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_MCLK, 24, 2},
	{HW_ID_RST, 0, 1},
	{HW_ID_DOVDD, 1804000, 1},
	{HW_ID_DOVDD, 0, 3},
	{HW_ID_DOVDD, 1804000, 1},
	{HW_ID_DOVDD, 0, 3},
	{HW_ID_DOVDD, 1804000, 1},
	{HW_ID_AVDD, 2204000, 1},
	{HW_ID_DVDD, 1008000, 0},
	{HW_ID_AFVDD, 2804000, 0},
	{HW_ID_RST, 1, 2},
	{HW_ID_MCLK_DRIVING_CURRENT, 4, 10},
};

static struct subdrv_pw_seq_entry pw_off_seq[] = {
	{HW_ID_MCLK, 24, 2},
	{HW_ID_RST, 0, 1},
	{HW_ID_DOVDD, 1804000, 1},
	{HW_ID_AVDD, 2204000, 1},
	{HW_ID_DVDD, 1008000, 0},
	{HW_ID_AFVDD, 2804000, 0},
	{HW_ID_RST, 1, 2},
	{HW_ID_MCLK_DRIVING_CURRENT, 4, 10},
};

const struct subdrv_entry omegac1tele_mipi_raw_entry = {
	.name = "omegac1tele_mipi_raw",
	.id = OMEGAC1TELE_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.pw_off_seq = pw_off_seq,
	.pw_off_seq_cnt = ARRAY_SIZE(pw_off_seq),
	.ops = &ops,
};

/* FUNCTION */

static int get_sensor_temperature(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	short temperature = 0;
	int temperature_convert = 0;

	temperature = subdrv_i2c_rd_u16(ctx, ctx->s_ctx.reg_addr_temp_read);
	temperature_convert = temperature / 256;

	DRV_LOG(ctx, "reg_val:0x%x, temperature: %d degrees\n", temperature, temperature_convert);
	return temperature_convert;
}

static void streaming_ctrl(struct subdrv_ctx *ctx, bool enable)
{
	check_current_scenario_id_bound(ctx);
	if (ctx->s_ctx.mode[ctx->current_scenario_id].aov_mode) {
		DRV_LOG(ctx, "AOV mode set stream in SCP side! (sid:%u)\n",
			ctx->current_scenario_id);
		return;
	}

	if (enable) {
		if (ctx->s_ctx.chk_s_off_before_s_on) {
			DRV_LOG(ctx, "check_stream_off before stream on");
			check_stream_off(ctx);
		}
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_stream, 0x01);
	} else {
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_stream, 0x00);
	}
	ctx->is_streaming = enable;
	DRV_LOG(ctx, "X! enable:%u\n", enable);
}

static void omegac1tele_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
		DRV_LOG(ctx, "SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%u\n", *(u32 *)para);
		if (*(u32 *)para)
			omegac1tele_set_shutter_convert(ctx, *(u32 *)para);
		streaming_ctrl(ctx, true);
}

static void omegac1tele_streaming_suspend(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
		DRV_LOG(ctx, "streaming control para:%d\n", *para);
		streaming_ctrl(ctx, false);
}

static unsigned int read_omegac1tele_eeprom_info(struct subdrv_ctx *ctx, kal_uint16 meta_id,
	BYTE *data, int size)
{
	kal_uint16 addr;
	int readsize;

	if (meta_id != omegac1tele_eeprom_info[meta_id].meta)
		return -1;

	if (size != omegac1tele_eeprom_info[meta_id].size)
		return -1;

	addr = omegac1tele_eeprom_info[meta_id].start;
	readsize = omegac1tele_eeprom_info[meta_id].size;

	if (!read_cmos_eeprom_p8(ctx, addr, data, readsize)) {
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

	memset(&omegac1tele_eeprom_common_data, 0x00, sizeof(omegac1tele_eeprom_common_data));

	omegac1tele_eeprom_common_data.header[EEPROM_MODULE_ID] = 2;
	read_omegac1tele_eeprom_info(ctx, EEPROM_META_MODULE_ID,
				&(omegac1tele_eeprom_common_data.data[idx]), 2);
	idx += omegac1tele_eeprom_common_data.header[EEPROM_MODULE_ID];

	omegac1tele_eeprom_common_data.header[EEPROM_SENSOR_ID] = 2;
	read_omegac1tele_eeprom_info(ctx, EEPROM_META_SENSOR_ID,
				&(omegac1tele_eeprom_common_data.data[idx]), 2);
	idx += omegac1tele_eeprom_common_data.header[EEPROM_SENSOR_ID];

	omegac1tele_eeprom_common_data.header[EEPROM_LENS_ID] = 2;
	read_omegac1tele_eeprom_info(ctx, EEPROM_META_LENS_ID,
				&(omegac1tele_eeprom_common_data.data[idx]), 2);
	idx += omegac1tele_eeprom_common_data.header[EEPROM_LENS_ID];

	omegac1tele_eeprom_common_data.header[EEPROM_VCM_ID] = 2;
	read_omegac1tele_eeprom_info(ctx, EEPROM_META_VCM_ID,
				&(omegac1tele_eeprom_common_data.data[idx]), 2);
	idx += omegac1tele_eeprom_common_data.header[EEPROM_VCM_ID];

	omegac1tele_eeprom_common_data.header[EEPROM_MODULE_SN] = 23;
	read_omegac1tele_eeprom_info(ctx, EEPROM_META_MODULE_SN,
				&(omegac1tele_eeprom_common_data.data[idx]), 23);
	idx += omegac1tele_eeprom_common_data.header[EEPROM_MODULE_SN];

	omegac1tele_eeprom_common_data.header[EEPROM_AF_CODE_MACRO] = 2;
	omegac1tele_eeprom_common_data.header[EEPROM_AF_CODE_INFINITY] = 2;
	omegac1tele_eeprom_common_data.header[EEPROM_AF_CODE_MIDDLE] = 2;
	read_omegac1tele_eeprom_info(ctx, EEPROM_META_AF_CODE,
				&(omegac1tele_eeprom_common_data.data[idx]), 6);

	AF_CODE_MACRO_idx = idx;
	AF_CODE_INFINITY_idx = idx + 2;
	AF_CODE_MIDDLE_idx = idx + 4;

	g_af_code_macro = omegac1tele_eeprom_common_data.data[AF_CODE_MACRO_idx + 1] << 8 |
		omegac1tele_eeprom_common_data.data[AF_CODE_MACRO_idx];
	g_af_code_infinity = omegac1tele_eeprom_common_data.data[AF_CODE_INFINITY_idx + 1] << 8 |
		omegac1tele_eeprom_common_data.data[AF_CODE_INFINITY_idx];;
	g_af_code_middle = omegac1tele_eeprom_common_data.data[AF_CODE_MIDDLE_idx + 1] << 8 |
		omegac1tele_eeprom_common_data.data[AF_CODE_MIDDLE_idx];;

	for (idx = 0; idx < 64; idx = idx + 4)
		LOG_INF("In %s:common data: %02x %02x %02x %02x\n", __func__,
			omegac1tele_eeprom_common_data.data[idx], omegac1tele_eeprom_common_data.data[idx + 1],
			omegac1tele_eeprom_common_data.data[idx + 2],
			omegac1tele_eeprom_common_data.data[idx + 3]);
}

static void omegac1tele_get_eeprom_comdata(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 *feature_return_para_32 = (u32 *)para;
	LOG_INF("+");

	if(*len == sizeof(omegac1tele_eeprom_common_data)) {
		memcpy(feature_return_para_32, &omegac1tele_eeprom_common_data,
		sizeof(omegac1tele_eeprom_common_data));
	}
}

static kal_uint16 read_cmos_eeprom_8(struct subdrv_ctx *ctx, kal_uint16 addr)
{
	kal_uint16 get_byte = 0;

	adaptor_i2c_rd_u8(ctx->i2c_client, OMEGAC1TELE_EEPROM_READ_ID >> 1, addr, (u8 *)&get_byte);
	return get_byte;
}

static kal_int32 table_write_eeprom_one_packet(struct subdrv_ctx *ctx,
        kal_uint16 addr, kal_uint8 *para, kal_uint32 len)
{
    kal_int32 ret = ERROR_NONE;
    ret = adaptor_i2c_wr_p8(ctx->i2c_client, OMEGAC1TELE_EEPROM_WRITE_ID >> 1,
            addr, para, len);

    return ret;
}

static kal_int32 write_eeprom_protect(struct subdrv_ctx *ctx, kal_uint16 enable)
{
    kal_int32 ret = ERROR_NONE;
    kal_uint16 reg = OMEGAC1TELE_EEPROM_LOCK_REGISTER;
    if (enable) {
        adaptor_i2c_wr_u8(ctx->i2c_client, OMEGAC1TELE_EEPROM_WRITE_ID >> 1,
            reg, 0xA1);
    }
    else {
        adaptor_i2c_wr_u8(ctx->i2c_client, OMEGAC1TELE_EEPROM_WRITE_ID >> 1,
            reg, 0xA0);
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
        if ((pStereodata->uSensorId == OMEGAC1TELE_SENSOR_ID) && (data_length == CALI_DATA_SLAVE_LENGTH)
            && (data_base == OMEGAC1TELE_STEREO_MT_START_ADDR)) {
            LOG_INF("Write: %x %x %x %x\n", pData[0], pData[39], pData[40], pData[1556]);

            eeprom_64align_write(ctx, data_base, pData, data_length);

            LOG_INF("com_0:0x%x\n", read_cmos_eeprom_8(ctx, data_base));
            LOG_INF("com_39:0x%x\n", read_cmos_eeprom_8(ctx, data_base+39));
            LOG_INF("innal_40:0x%x\n", read_cmos_eeprom_8(ctx, data_base+40));
            LOG_INF("innal_1556:0x%x\n", read_cmos_eeprom_8(ctx, data_base+1556));
            LOG_INF("write_Module_data Write end\n");

        } else if ((pStereodata->uSensorId == OMEGAC1TELE_SENSOR_ID) && (data_length < AESYNC_DATA_LENGTH_TOTAL)
            && (data_base == OMEGAC1TELE_AESYNC_START_ADDR)) {
            LOG_INF("write main aesync: %x %x %x %x %x %x %x %x\n", pData[0], pData[1],
                pData[2], pData[3], pData[4], pData[5], pData[6], pData[7]);

            eeprom_64align_write(ctx, data_base, pData, data_length);

            LOG_INF("readback main aesync: %x %x %x %x %x %x %x %x\n",
                    read_cmos_eeprom_8(ctx, OMEGAC1TELE_AESYNC_START_ADDR),
                    read_cmos_eeprom_8(ctx, OMEGAC1TELE_AESYNC_START_ADDR+1),
                    read_cmos_eeprom_8(ctx, OMEGAC1TELE_AESYNC_START_ADDR+2),
                    read_cmos_eeprom_8(ctx, OMEGAC1TELE_AESYNC_START_ADDR+3),
                    read_cmos_eeprom_8(ctx, OMEGAC1TELE_AESYNC_START_ADDR+4),
                    read_cmos_eeprom_8(ctx, OMEGAC1TELE_AESYNC_START_ADDR+5),
                    read_cmos_eeprom_8(ctx, OMEGAC1TELE_AESYNC_START_ADDR+6),
                    read_cmos_eeprom_8(ctx, OMEGAC1TELE_AESYNC_START_ADDR+7));
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

static void omegac1tele_set_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
    int ret = ERROR_NONE;
    ret = write_Module_data(ctx, (ACDK_SENSOR_ENGMODE_STEREO_STRUCT *)(para));
    if (ret != ERROR_NONE) {
        *len = (u32)-1; /*write eeprom failed*/
        LOG_INF("ret=%d\n", ret);
    }
}

static void omegac1tele_get_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	if(*len > CALI_DATA_SLAVE_LENGTH) {
		*len = CALI_DATA_SLAVE_LENGTH;
	}
	read_omegac1tele_eeprom_info(ctx, EEPROM_META_STEREO_MT_MAIN_DATA,
			(BYTE *)para, *len);
}

static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size)
{
	if (adaptor_i2c_rd_p8(ctx->i2c_client, OMEGAC1TELE_EEPROM_READ_ID >> 1,
			addr, data, size) < 0) {
		return false;
	}
	return true;
}

static void read_otp_info(struct subdrv_ctx *ctx)
{
	DRV_LOGE(ctx, "jn5 read_otp_info begin\n");
	read_cmos_eeprom_p8(ctx, 0, otp_data_checksum, OTP_SIZE);
	DRV_LOGE(ctx, "jn5 read_otp_info end\n");
}

static void omegac1tele_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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

static void omegac1tele_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	get_imgsensor_id(ctx, (u32 *)para);
}

static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id)
{
	u8 i = 0;
	u8 retry = 2;
	static bool first_read = TRUE;
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
			DRV_LOGE(ctx, "i2c_write_id(0x%x) sensor_id(0x%x/0x%x)\n",
				ctx->i2c_write_id, *sensor_id, ctx->s_ctx.sensor_id);
			if (*sensor_id == 0x38E5) {
				*sensor_id = ctx->s_ctx.sensor_id;
				if (first_read) {
					read_eeprom_common_data(ctx);
					first_read = FALSE;
					subdrv_i2c_wr_u16(ctx, 0xFCFC, 0x4000);
					module_flag = subdrv_i2c_rd_u16(ctx, 0x0010);

					msg_buf = kmalloc(MAX_BURST_LEN, GFP_KERNEL);
					if(!msg_buf) {
						LOG_INF("boot stage, malloc msg_buf error");
					}
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

static int open(struct subdrv_ctx *ctx)
{
	u32 sensor_id = 0;
	u32 scenario_id = 0;

	/* get sensor id */
	if (get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;

	if (module_flag == 0x010F || module_flag == 0x011F || (module_flag & 0xFF00) == 0x0200){
		DRV_LOGE(ctx, "module_flag = 0x%x, modules with OTP data\n", module_flag);
		subdrv_i2c_wr_regs_u16(ctx, omegac1tele_sensor_init_pre_setting1, ARRAY_SIZE(omegac1tele_sensor_init_pre_setting1));
		mdelay(5);
		subdrv_i2c_wr_regs_u16(ctx, omegac1tele_sensor_init_pre_setting2, ARRAY_SIZE(omegac1tele_sensor_init_pre_setting2));
		mdelay(5);
		omegac1tele_i2c_burst_wr_regs_u16(ctx, omegac1tele_sensor_simple_init_setting, ARRAY_SIZE(omegac1tele_sensor_simple_init_setting));
		DRV_LOGE(ctx, "setting end\n");
	}else{
		DRV_LOGE(ctx, "module_flag = 0x%x, modules without OTP data\n", module_flag);
		subdrv_i2c_wr_regs_u16(ctx, omegac1tele_sensor_init_pre_setting1, ARRAY_SIZE(omegac1tele_sensor_init_pre_setting1));
		mdelay(5);
		subdrv_i2c_wr_regs_u16(ctx, omegac1tele_sensor_init_pre_setting2, ARRAY_SIZE(omegac1tele_sensor_init_pre_setting2));
		mdelay(5);
		subdrv_i2c_wr_regs_u16(ctx, omegac1tele_sensor_init_setting, ARRAY_SIZE(omegac1tele_sensor_init_setting));
		DRV_LOGE(ctx, "setting end\n");
	};

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
	bRmscMode = 0;
	return ERROR_NONE;
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
	return gain * 32 / BASEGAIN;
}

static bool test_pattern_change = false;
static void omegac1tele_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);

	if (mode != ctx->test_pattern) {

		LOG_INF("mode(%u->%u)\n", ctx->test_pattern, mode);
		test_pattern_change = true;

		/* 1:Solid Color 2:Color Bar 5:Black */
		if (mode) {
			if (mode == 5) {
				//subdrv_i2c_wr_u16(ctx, 0x0600, 0x0001); /*black*/
				subdrv_i2c_wr_u16(ctx, 0xFCFC, 0x4000);
				subdrv_i2c_wr_u16(ctx, 0x020C, 0x0000);
				subdrv_i2c_wr_u16(ctx, 0x020E, 0x0000);
				subdrv_i2c_wr_u16(ctx, 0x0210, 0x0000);
				subdrv_i2c_wr_u16(ctx, 0x0212, 0x0000);
				subdrv_i2c_wr_u16(ctx, 0x0214, 0x0000);
				subdrv_i2c_wr_u16(ctx, 0x0230, 0x0000);
				subdrv_i2c_wr_u16(ctx, 0x0232, 0x0000);
				subdrv_i2c_wr_u16(ctx, 0x0234, 0x0000);
				subdrv_i2c_wr_u16(ctx, 0x0236, 0x0000);
				subdrv_i2c_wr_u16(ctx, 0x0240, 0x0000);
				subdrv_i2c_wr_u16(ctx, 0x0242, 0x0000);
				subdrv_i2c_wr_u16(ctx, 0x0244, 0x0000);
				subdrv_i2c_wr_u16(ctx, 0x0246, 0x0000);
			} else {
				subdrv_i2c_wr_u16(ctx, 0x0600, mode); /*100% Color bar*/
			}
		} else {
			if (ctx->test_pattern) {
				subdrv_i2c_wr_u16(ctx, 0x0600, 0x0000); /*No pattern*/
				subdrv_i2c_wr_u16(ctx, 0xFCFC, 0x4000);
				subdrv_i2c_wr_u16(ctx, 0x020C, 0x0000);
				subdrv_i2c_wr_u16(ctx, 0x020E, 0x0100);
				subdrv_i2c_wr_u16(ctx, 0x0210, 0x0100);
				subdrv_i2c_wr_u16(ctx, 0x0212, 0x0100);
				subdrv_i2c_wr_u16(ctx, 0x0214, 0x0100);
				subdrv_i2c_wr_u16(ctx, 0x0230, 0x0100);
				subdrv_i2c_wr_u16(ctx, 0x0232, 0x0100);
				subdrv_i2c_wr_u16(ctx, 0x0234, 0x0100);
				subdrv_i2c_wr_u16(ctx, 0x0236, 0x0100);
				subdrv_i2c_wr_u16(ctx, 0x0240, 0x0100);
				subdrv_i2c_wr_u16(ctx, 0x0242, 0x0100);
				subdrv_i2c_wr_u16(ctx, 0x0244, 0x0100);
				subdrv_i2c_wr_u16(ctx, 0x0246, 0x0100);
			}
		}
		ctx->test_pattern = mode;
	}
}

static void omegac1tele_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	struct mtk_test_pattern_data *data = (struct mtk_test_pattern_data *)para;

	if(test_pattern_change && ctx->test_pattern) {
		u16 R = (data->Channel_R >> 22) & 0x3ff;
		u16 Gr = (data->Channel_R >> 22) & 0x3ff;
		u16 Gb = (data->Channel_R >> 22) & 0x3ff;
		u16 B = (data->Channel_R >> 22) & 0x3ff;

		subdrv_i2c_wr_u16(ctx, 0x0602, Gr);
		subdrv_i2c_wr_u16(ctx, 0x0604, R);
		subdrv_i2c_wr_u16(ctx, 0x0606, B);
		subdrv_i2c_wr_u16(ctx, 0x0608, Gb);

		test_pattern_change = false;

		LOG_INF("mode(%u) R/Gr/Gb/B = 0x%04x/0x%04x/0x%04x/0x%04x\n",
			ctx->test_pattern, R, Gr, Gb, B);
	}
}

static void omegac1tele_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	enum SENSOR_SCENARIO_ID_ENUM scenario_id;
	struct mtk_hdr_ae *ae_ctrl = NULL;
	u64 *feature_data = (u64 *) para;
//	u32 fine_integ_line = 0;
//	u32 cit_step = 0;
//	u32 rg_shutter = 0;
//	u32 prsh_length_lines = 0;

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
	LOG_INF("E: set seamless switch %u %u\n", ctx->current_scenario_id, scenario_id);

//	if (!ctx->extend_frame_length_en)
//		DRV_LOGE(ctx, "please extend_frame_length before seamless_switch!\n");
//	ctx->extend_frame_length_en = FALSE;

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

	ctx->is_seamless = TRUE;
	update_mode_info(ctx, scenario_id);

	set_group_hold(ctx, 1);
	commit_i2c_buffer(ctx);

	i2c_table_write(ctx,
		ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_table,
		ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_len);
	if (ae_ctrl) {
		switch (ctx->s_ctx.mode[scenario_id].hdr_mode) {
		case HDR_RAW_STAGGER_2EXP:
			omegac1tele_set_multi_shutter_frame_length(ctx, (u64*)&ae_ctrl->exposure, 2, 0);
			omegac1tele_set_multi_gain(ctx, (u32*)&ae_ctrl->gain, 2);
			break;
		case HDR_RAW_STAGGER_3EXP:
			omegac1tele_set_multi_shutter_frame_length(ctx, (u64*)&ae_ctrl->exposure, 3, 0);
			omegac1tele_set_multi_gain(ctx, (u32*)&ae_ctrl->gain, 3);
			break;
		default:
			omegac1tele_set_shutter_convert(ctx, ae_ctrl->exposure.le_exposure);
			omegac1tele_set_gain_convert(ctx, ae_ctrl->gain.le_gain);
			break;
		}
	}
	LOG_INF("write seamless switch para done\n");

	set_group_hold(ctx, 0);
	commit_i2c_buffer(ctx);

	ctx->ref_sof_cnt = ctx->sof_cnt;
	ctx->is_seamless = FALSE;
	LOG_INF("X: set seamless switch done\n");
}

static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id)
{
	memcpy(&(ctx->s_ctx), &static_ctx, sizeof(struct subdrv_static_ctx));
	subdrv_ctx_init(ctx);
	ctx->i2c_client = i2c_client;
	ctx->i2c_write_id = i2c_write_id;
	return 0;
}

void omegac1tele_write_frame_length(struct subdrv_ctx *ctx, u32 fll)
{
	u32 addr_h = ctx->s_ctx.reg_addr_frame_length.addr[0];
	u32 addr_l = ctx->s_ctx.reg_addr_frame_length.addr[1];
	u32 addr_ll = ctx->s_ctx.reg_addr_frame_length.addr[2];
	u32 fll_step = 0;
	u32 dol_cnt = 1;

	check_current_scenario_id_bound(ctx);

	switch (ctx->s_ctx.mode[ctx->current_scenario_id].hdr_mode) {
	case HDR_RAW_STAGGER_2EXP:
		dol_cnt = 2;
		break;
	case HDR_RAW_STAGGER_3EXP:
		dol_cnt = 3;
		break;
	default:
		break;
	}

	fll_step = ctx->s_ctx.mode[ctx->current_scenario_id].framelength_step;

	ctx->frame_length = fll;

	if (fll_step)
		fll = round_up(fll, fll_step);

	if (ctx->extend_frame_length_en == FALSE) {
		if (addr_ll) {
			set_i2c_buffer(ctx,	addr_h,	(fll >> 16) & 0xFF);
			set_i2c_buffer(ctx,	addr_l, (fll >> 8) & 0xFF);
			set_i2c_buffer(ctx,	addr_ll, fll & 0xFF);
		} else {
			set_i2c_buffer(ctx,	addr_h, (fll >> 8) & 0xFF);
			set_i2c_buffer(ctx,	addr_l, fll & 0xFF);
		}
	}
	LOG_INF("fll[0x%x] multiply %u, fll_step:%u ctx->extend_frame_length_en:%d\n",
		fll, dol_cnt, fll_step, ctx->extend_frame_length_en);
}

static void omegac1tele_set_multi_gain(struct subdrv_ctx *ctx, u32 *gains, u16 exp_cnt)
{
	int i = 0;
	u16 rg_gains[3] = {0};
	u8 has_gains[3] = {0};
	bool gph = !ctx->is_seamless && (ctx->s_ctx.s_gph != NULL);
	u32 ana_gain_min = ctx->s_ctx.mode[ctx->current_scenario_id].ana_gain_max ?
		ctx->s_ctx.mode[ctx->current_scenario_id].ana_gain_min : ctx->ana_gain_min;
	u32 ana_gain_max = ctx->s_ctx.mode[ctx->current_scenario_id].ana_gain_max ?
		ctx->s_ctx.mode[ctx->current_scenario_id].ana_gain_max : ctx->ana_gain_max;

	if(exp_cnt == 2) {
		LOG_INF("gains[0]:%u, gains[1]:%u, exp_cnt:%d ana_gain_min:%d ana_gain_max:%d\n",
			gains[0], gains[1], exp_cnt, ana_gain_min, ana_gain_max);
	} else {
		LOG_INF("gains[0]:%u, exp_cnt:%d ana_gain_min:%d ana_gain_max:%d\n",
			gains[0], exp_cnt, ana_gain_min, ana_gain_max);
	}
	if (exp_cnt > ARRAY_SIZE(ctx->ana_gain)) {
		DRV_LOGE(ctx, "invalid exp_cnt:%u>%u\n", exp_cnt, ARRAY_SIZE(ctx->ana_gain));
		exp_cnt = ARRAY_SIZE(ctx->ana_gain);
	}
	for (i = 0; i < exp_cnt; i++) {
		/* check boundary of gain */
		gains[i] = max(gains[i], ana_gain_min);
		gains[i] = min(gains[i], ana_gain_max);
		/* mapping of gain to register value */
		if (ctx->s_ctx.g_gain2reg != NULL)
			gains[i] = ctx->s_ctx.g_gain2reg(gains[i]);
		else
			gains[i] = gain2reg(gains[i]);
	}
	/* restore gain */
	memset(ctx->ana_gain, 0, sizeof(ctx->ana_gain));
	for (i = 0; i < exp_cnt; i++)
		ctx->ana_gain[i] = gains[i];
	/* group hold start */
	if (gph && !ctx->ae_ctrl_gph_en)
		ctx->s_ctx.s_gph((void *)ctx, 1);
	/* write gain */
	memset(has_gains, 1, sizeof(has_gains));
	switch (exp_cnt) {
	case 2:
		has_gains[1] = 0;
//		rg_gains[0] = gains[0];
//		rg_gains[2] = gains[1];
		rg_gains[0] = gains[1];
		rg_gains[2] = gains[0];
		break;
	case 3:
//		rg_gains[0] = gains[0];
		rg_gains[0] = gains[2];
		rg_gains[1] = gains[1];
//		rg_gains[2] = gains[2];
		rg_gains[2] = gains[0];
		break;
	default:
		has_gains[0] = 0;
		has_gains[1] = 0;
		has_gains[2] = 0;
		break;
	}
	for (i = 0; i < 3; i++) {
		if (has_gains[i]) {
			set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_ana_gain[i].addr[0],
				(rg_gains[i] >> 8) & 0xFF);
			set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_ana_gain[i].addr[1],
				rg_gains[i] & 0xFF);
		}
	}
	LOG_INF("reg[sg/mg/lg]: 0x%x 0x%x 0x%x\n", rg_gains[0], rg_gains[1], rg_gains[2]);
	if (gph)
		ctx->s_ctx.s_gph((void *)ctx, 0);
	commit_i2c_buffer(ctx);
	/* group hold end */
}

static void omegac1tele_set_hdr_tri_gain(struct subdrv_ctx *ctx, u64 *gains, u16 exp_cnt)
{
	int i = 0;
	u32 values[3] = {0};

	if (gains != NULL) {
		for (i = 0; i < 3; i++)
			values[i] = (u32) *(gains + i);
	}
	omegac1tele_set_multi_gain(ctx,	values, exp_cnt);
}

static void omegac1tele_set_hdr_tri_gain2(struct subdrv_ctx *ctx, u8 *para, u32 *len) {

	u64 *feature_data = (u64 *) para;
	omegac1tele_set_hdr_tri_gain(ctx, feature_data, 2);
}

static void omegac1tele_set_hdr_tri_gain3(struct subdrv_ctx *ctx, u8 *para, u32 *len) {
	u64 *feature_data = (u64 *) para;
	omegac1tele_set_hdr_tri_gain(ctx, feature_data, 3);
}

static void omegac1tele_set_multi_shutter_frame_length(struct subdrv_ctx *ctx, u64 *shutters, u16 exp_cnt, u16 frame_length)
{
	int i = 0;
	u32 fine_integ_line = 0;
	u16 last_exp_cnt = 1;
	u32 calc_fl[3] = {0};
//	int readout_diff = 0;
	bool gph = !ctx->is_seamless && (ctx->s_ctx.s_gph != NULL);
	u32 rg_shutters[3] = {0};
	u32 cit_step = 0;
	u8 exposure_margin = ctx->s_ctx.mode[ctx->current_scenario_id].exposure_margin ?
		ctx->s_ctx.mode[ctx->current_scenario_id].exposure_margin : ctx->s_ctx.exposure_margin;

	if(exp_cnt == 2) {
		LOG_INF("shutter[0]:%u, shutter[1]:%u, exp_cnt:%d, frame_length:%u exposure_margin:%d cit_step:%d\n",
			shutters[0], shutters[1], exp_cnt, frame_length, exposure_margin, cit_step);
	} else {
		LOG_INF("shutter[0]:%u, exp_cnt:%d, frame_length:%u, exposure_margin:%d cit_step:%d\n",
			shutters[0], exp_cnt, frame_length, exposure_margin, cit_step);
	}

	ctx->frame_length = frame_length ? frame_length : ctx->frame_length;
	if (exp_cnt > ARRAY_SIZE(ctx->exposure)) {
		DRV_LOGE(ctx, "invalid exp_cnt:%u>%lu\n", exp_cnt, ARRAY_SIZE(ctx->exposure));
		exp_cnt = ARRAY_SIZE(ctx->exposure);
	}
	check_current_scenario_id_bound(ctx);

	/* check boundary of shutter */
	for (i = 1; i < ARRAY_SIZE(ctx->exposure); i++)
		last_exp_cnt += ctx->exposure[i] ? 1 : 0;
	fine_integ_line = ctx->s_ctx.mode[ctx->current_scenario_id].fine_integ_line;
	cit_step = ctx->s_ctx.mode[ctx->current_scenario_id].coarse_integ_step;
	for (i = 0; i < exp_cnt; i++) {
		shutters[i] = FINE_INTEG_CONVERT(shutters[i], fine_integ_line);
		shutters[i] = max(shutters[i], (u64)ctx->s_ctx.exposure_min);
		shutters[i] = min(shutters[i], (u64)ctx->s_ctx.exposure_max);
		if (cit_step)
			shutters[i] = round_up(shutters[i], cit_step);
	}

	/* check boundary of framelength */
	/* - (1) previous se + previous me + current le */
//	calc_fl[0] = shutters[0];
//	for (i = 1; i < last_exp_cnt; i++)
//		calc_fl[0] += ctx->exposure[i];
//	calc_fl[0] += ctx->s_ctx.exposure_margin * exp_cnt * exp_cnt;

	/* - (2) current se + current me + current le */
	calc_fl[1] = shutters[0];
	for (i = 1; i < exp_cnt; i++)
		calc_fl[1] += shutters[i];
	calc_fl[1] += exposure_margin * exp_cnt;

//	/* - (3) readout time cannot be overlapped */
//	calc_fl[2] =
//		(ctx->s_ctx.mode[ctx->current_scenario_id].readout_length +
//		ctx->s_ctx.mode[ctx->current_scenario_id].read_margin);

//	if (last_exp_cnt == exp_cnt)
//		for (i = 1; i < exp_cnt; i++) {
//			readout_diff = ctx->exposure[i] - shutters[i];
//			calc_fl[2] += readout_diff > 0 ? readout_diff : 0;
//		}
	for (i = 0; i < ARRAY_SIZE(calc_fl); i++)
		ctx->frame_length = max(ctx->frame_length, calc_fl[i]);

	ctx->frame_length = max(ctx->frame_length, ctx->min_frame_length);
	ctx->frame_length = min(ctx->frame_length, ctx->s_ctx.frame_length_max);
	/* restore shutter */
	memset(ctx->exposure, 0, sizeof(ctx->exposure));
	for (i = 0; i < exp_cnt; i++)
		ctx->exposure[i] = shutters[i];
	/* group hold start */
	if (gph)
		ctx->s_ctx.s_gph((void *)ctx, 1);
	/* enable auto extend */
	if (ctx->s_ctx.reg_addr_auto_extend)
		set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_auto_extend, 0x01);
	/* write framelength */
	if (set_auto_flicker(ctx, 0) || frame_length || !ctx->s_ctx.reg_addr_auto_extend)
		omegac1tele_write_frame_length(ctx, ctx->frame_length);
	/* write shutter */
	switch (exp_cnt) {
	case 1:
		rg_shutters[0] = shutters[0];
		break;
	case 2:
		rg_shutters[0] = shutters[1];
		rg_shutters[2] = shutters[0];
		break;
	case 3:
		rg_shutters[0] = shutters[2];
		rg_shutters[1] = shutters[1];
		rg_shutters[2] = shutters[0];
		break;
	default:
		break;
	}
//	if (ctx->s_ctx.reg_addr_exposure_lshift != PARAM_UNDEFINED)
//		set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_exposure_lshift, 0);

	if (bNeedSetNormalMode) {
		LOG_INF("exit long exposure\n");
		set_i2c_buffer(ctx, 0x0702, 0x00);
		set_i2c_buffer(ctx, 0x0704, 0x00);
		bNeedSetNormalMode = FALSE;
	}

	for (i = 0; i < 3; i++) {
		if (rg_shutters[i]) {
			if (ctx->s_ctx.reg_addr_exposure[i].addr[2]) {
				set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[i].addr[0],
					(rg_shutters[i] >> 16) & 0xFF);
				set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[i].addr[1],
					(rg_shutters[i] >> 8) & 0xFF);
				set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[i].addr[2],
					rg_shutters[i] & 0xFF);
			} else {
				set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[i].addr[0],
					(rg_shutters[i] >> 8) & 0xFF);
				set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[i].addr[1],
					rg_shutters[i] & 0xFF);
			}
		}
	}
	LOG_INF("exp[0x%x/0x%x/0x%x], fll(input/output):%u/%u, flick_en:%u\n",
		rg_shutters[0], rg_shutters[1], rg_shutters[2],
		frame_length, ctx->frame_length, ctx->autoflicker_en);
	if (!ctx->ae_ctrl_gph_en) {
		if (gph)
			ctx->s_ctx.s_gph((void *)ctx, 0);
		commit_i2c_buffer(ctx);
	}
	/* group hold end */

	omegac1tele_lens_pos_writeback(ctx);

}

static void omegac1tele_set_multi_shutter_frame_length_ctrl(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64* feature_data = (u64*)para;

	omegac1tele_set_multi_shutter_frame_length(ctx, (u64 *)(*feature_data),
		(u64) (*(feature_data + 1)), (u64) (*(feature_data + 2)));
}

static void omegac1tele_set_hdr_tri_shutter(struct subdrv_ctx *ctx, u64 *shutters, u16 exp_cnt)
{
	int i = 0;
	u64 values[3] = {0};

	if (shutters != NULL) {
		for (i = 0; i < 3; i++)
			values[i] = (u64) *(shutters + i);
	}
	omegac1tele_set_multi_shutter_frame_length(ctx, values, exp_cnt, 0);
}

static void omegac1tele_set_hdr_tri_shutter2(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64* feature_data = (u64*)para;

	omegac1tele_set_hdr_tri_shutter(ctx, feature_data, 2);
}

static void omegac1tele_set_hdr_tri_shutter3(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64* feature_data = (u64*)para;

	omegac1tele_set_hdr_tri_shutter(ctx, feature_data, 3);
}

bool omegac1tele_set_long_exposure(struct subdrv_ctx *ctx,  u64 shutter)
{
	u16 l_shift = 0;
	u8 exposure_margin = ctx->s_ctx.mode[ctx->current_scenario_id].exposure_margin ?
		ctx->s_ctx.mode[ctx->current_scenario_id].exposure_margin : ctx->s_ctx.exposure_margin;

	if (shutter > (ctx->s_ctx.frame_length_max - exposure_margin)) {
		if (ctx->s_ctx.long_exposure_support == FALSE) {
			DRV_LOGE(ctx, "sensor no support of exposure lshift!\n");
			return FALSE;
		}
		if (ctx->s_ctx.reg_addr_exposure_lshift == PARAM_UNDEFINED) {
			DRV_LOGE(ctx, "please implement lshift register address\n");
			return FALSE;
		}
		for (l_shift = 1; l_shift < 7; l_shift++) {
			if ((shutter >> l_shift)
				< (ctx->s_ctx.frame_length_max - exposure_margin))
				break;
		}
		if (l_shift > 7) {
			DRV_LOGE(ctx, "unable to set exposure:%u, set to max\n", shutter);
			l_shift = 7;
		}
		shutter = shutter >> l_shift;
//		if (!ctx->s_ctx.reg_addr_auto_extend)
//			ctx->frame_length = shutter + exposure_margin;
		LOG_INF("long exposure mode: lshift %u times   shutter:%d", l_shift, shutter);

		set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_frame_length.addr[0], (shutter >> 8) & 0xFF);
		set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_frame_length.addr[1],  shutter & 0xFF);

		set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[0].addr[0], (shutter >> 8) & 0xFF);
		set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[0].addr[1],  shutter & 0xFF);

		set_i2c_buffer(ctx, 0x0702, l_shift);
		set_i2c_buffer(ctx, 0x0704, l_shift);

		commit_i2c_buffer(ctx);

		bNeedSetNormalMode = TRUE;
		/* Frame exposure mode customization for LE*/
		ctx->ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
		ctx->ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
//		ctx->current_ae_effective_frame = 2;
		return TRUE;

	} else {
		if (bNeedSetNormalMode) {
			LOG_INF("exit long exposure\n");
			set_i2c_buffer(ctx, 0x0702, 0x00);
			set_i2c_buffer(ctx, 0x0704, 0x00);
			bNeedSetNormalMode = FALSE;
		}
		return FALSE;
//		ctx->current_ae_effective_frame = 2;
	}

//	ctx->exposure[IMGSENSOR_STAGGER_EXPOSURE_LE] = shutter;
	return FALSE;
}
static void omegac1tele_set_shutter_frame_length_convert(struct subdrv_ctx *ctx, u64 shutter, u32 frame_length)
{
	u32 fine_integ_line = 0;
	u32 cit_step = 0;

	bool gph = !ctx->is_seamless && (ctx->s_ctx.s_gph != NULL);
	u8 exposure_margin = ctx->s_ctx.mode[ctx->current_scenario_id].exposure_margin ?
		ctx->s_ctx.mode[ctx->current_scenario_id].exposure_margin : ctx->s_ctx.exposure_margin;
	LOG_INF("shutter:%u, frame_length:%u  exposure_margin:%d\n", shutter, frame_length, exposure_margin);

	ctx->frame_length = frame_length ? frame_length : ctx->frame_length;
	check_current_scenario_id_bound(ctx);
	/* check boundary of shutter */
	fine_integ_line = ctx->s_ctx.mode[ctx->current_scenario_id].fine_integ_line;
	shutter = FINE_INTEG_CONVERT(shutter, fine_integ_line);
	shutter = max(shutter, (u64)ctx->s_ctx.exposure_min);
	shutter = min(shutter, (u64)ctx->s_ctx.exposure_max);
	/* check boundary of framelength */

	cit_step = ctx->s_ctx.mode[ctx->current_scenario_id].coarse_integ_step;
	if (cit_step)
		shutter = round_up(shutter, cit_step);

	ctx->frame_length =	max(shutter + exposure_margin, (u64)ctx->min_frame_length);
	ctx->frame_length =	min(ctx->frame_length, ctx->s_ctx.frame_length_max);
	/* restore shutter */
	memset(ctx->exposure, 0, sizeof(ctx->exposure));
	ctx->exposure[0] = shutter;
	/* group hold start */
	if (gph)
		ctx->s_ctx.s_gph((void *)ctx, 1);

	if(omegac1tele_set_long_exposure(ctx, shutter)) {
		if (gph)
			ctx->s_ctx.s_gph((void *)ctx, 0);
		return;
	}

	/* enable auto extend */
	if (ctx->s_ctx.reg_addr_auto_extend)
		set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_auto_extend, 0x01);
	/* write framelength */
	if (set_auto_flicker(ctx, 0) || frame_length || !ctx->s_ctx.reg_addr_auto_extend)
		omegac1tele_write_frame_length(ctx, ctx->frame_length);
	/* write shutter */
	//set_long_exposure(ctx);
	if (ctx->s_ctx.reg_addr_exposure[0].addr[2]) {
		set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[0].addr[0],
			(ctx->exposure[0] >> 16) & 0xFF);
		set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[0].addr[1],
			(ctx->exposure[0] >> 8) & 0xFF);
		set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[0].addr[2],
			ctx->exposure[0] & 0xFF);
	} else {
		set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[0].addr[0],
			(ctx->exposure[0] >> 8) & 0xFF);
		set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[0].addr[1],
			ctx->exposure[0] & 0xFF);
	}
	LOG_INF("exp[0x%x], fll(input/output):%u/%u, flick_en:%u\n",
		ctx->exposure[0], frame_length, ctx->frame_length, ctx->autoflicker_en);
	if (!ctx->ae_ctrl_gph_en) {
		if (gph)
			ctx->s_ctx.s_gph((void *)ctx, 0);
		commit_i2c_buffer(ctx);
	}
	/* group hold end */

	omegac1tele_lens_pos_writeback(ctx);

}

static void omegac1tele_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
 	omegac1tele_set_shutter_frame_length_convert(ctx, ((u64*)para)[0], ((u64*)para)[1]);
}

static void omegac1tele_set_shutter_convert(struct subdrv_ctx *ctx, u64 shutter)
{
    omegac1tele_set_shutter_frame_length_convert(ctx, shutter, 0);
}

static void omegac1tele_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	omegac1tele_set_shutter_frame_length_convert(ctx, ((u64*)para)[0], 0);
}

static void omegac1tele_set_gain_convert(struct subdrv_ctx *ctx, u32 gain)
{
	u16 rg_gain;
	bool gph = !ctx->is_seamless && (ctx->s_ctx.s_gph != NULL);
	u32 ana_gain_min = ctx->s_ctx.mode[ctx->current_scenario_id].ana_gain_max ?
		ctx->s_ctx.mode[ctx->current_scenario_id].ana_gain_min : ctx->ana_gain_min;
	u32 ana_gain_max = ctx->s_ctx.mode[ctx->current_scenario_id].ana_gain_max ?
		ctx->s_ctx.mode[ctx->current_scenario_id].ana_gain_max : ctx->ana_gain_max;

	LOG_INF("gain(%d) ana_gain_min(%d), ana_gain_max(%d)\n", gain, ana_gain_min, ana_gain_max);
	/* check boundary of gain */
	gain = max(gain, ana_gain_min);
	gain = min(gain, ana_gain_max);

	/* mapping of gain to register value */
	if (ctx->s_ctx.g_gain2reg != NULL)
		rg_gain = ctx->s_ctx.g_gain2reg(gain);
	else
		rg_gain = gain2reg(gain);

	/* restore gain */
	memset(ctx->ana_gain, 0, sizeof(ctx->ana_gain));
	ctx->ana_gain[0] = gain;
	/* group hold start */
	if (gph && !ctx->ae_ctrl_gph_en)
		ctx->s_ctx.s_gph((void *)ctx, 1);
	/* write gain */
	set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_ana_gain[0].addr[0],
		(rg_gain >> 8) & 0xFF);
	set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_ana_gain[0].addr[1],
		rg_gain & 0xFF);
	DRV_LOG(ctx, "gain[0x%x]\n", rg_gain);
	if (gph)
		ctx->s_ctx.s_gph((void *)ctx, 0);
	commit_i2c_buffer(ctx);
	/* group hold end */
}

static void omegac1tele_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64* feature_data = (u64*)para;
	u32 gain = *feature_data;
	omegac1tele_set_gain_convert(ctx, gain);
}

void omegac1tele_set_dummy(struct subdrv_ctx *ctx)
{
//	bool gph = !ctx->is_seamless && (ctx->s_ctx.s_gph != NULL);
//
//	if (gph)
//		ctx->s_ctx.s_gph((void *)ctx, 1);
//	omegac1tele_write_frame_length(ctx, ctx->frame_length);
//	if (gph)
//		ctx->s_ctx.s_gph((void *)ctx, 0);
//
//	commit_i2c_buffer(ctx);
}

static void omegac1tele_set_max_framerate_by_scenario(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	enum SENSOR_SCENARIO_ID_ENUM scenario_id = (u32)((u64*)para)[0];
	u32 framerate = (u32)((u64*)para)[1];
	u32 frame_length;
	u32 frame_length_step;

	LOG_INF("scenario_id(%d), framerate(%d)", scenario_id, framerate);

	if (scenario_id >= ctx->s_ctx.sensor_mode_num) {
		DRV_LOG(ctx, "invalid sid:%u, mode_num:%u\n",
			scenario_id, ctx->s_ctx.sensor_mode_num);
		scenario_id = SENSOR_SCENARIO_ID_NORMAL_PREVIEW;
	}

	if (framerate == 0) {
		DRV_LOG(ctx, "framerate (%u) is invalid\n", framerate);
		return;
	}

	if (ctx->s_ctx.mode[scenario_id].linelength == 0) {
		DRV_LOG(ctx, "linelength (%u) is invalid\n",
			ctx->s_ctx.mode[scenario_id].linelength);
		return;
	}

	frame_length = ctx->s_ctx.mode[scenario_id].pclk / framerate * 10
		/ ctx->s_ctx.mode[scenario_id].linelength;
	frame_length_step = ctx->s_ctx.mode[scenario_id].framelength_step;
	frame_length = frame_length_step ?
		(frame_length - (frame_length % frame_length_step)) : frame_length;
	ctx->frame_length =
		max(frame_length, ctx->s_ctx.mode[scenario_id].framelength);
	ctx->frame_length = min(ctx->frame_length, ctx->s_ctx.frame_length_max);
	ctx->current_fps = ctx->pclk / ctx->frame_length * 10 / ctx->line_length;
	ctx->min_frame_length = ctx->frame_length;
	LOG_INF("max_fps(input/output):%u/%u(sid:%u), min_fl_en:1\n",
		framerate, ctx->current_fps, scenario_id);
	if (ctx->s_ctx.reg_addr_auto_extend ||
			(ctx->frame_length > (ctx->exposure[0] + ctx->s_ctx.exposure_margin))){
		omegac1tele_set_dummy(ctx);
	}
}

void omegac1tele_set_max_framerate(struct subdrv_ctx *ctx, u16 framerate, bool min_framelength_en)
{
	u32 frame_length = 0;

	if (framerate && ctx->line_length)
		frame_length = ctx->pclk / framerate * 10 / ctx->line_length;
	ctx->frame_length = max(frame_length, ctx->min_frame_length);
	ctx->frame_length = min(ctx->frame_length, ctx->s_ctx.frame_length_max);
	if (ctx->frame_length && ctx->line_length)
		ctx->current_fps = ctx->pclk / ctx->frame_length * 10 / ctx->line_length;
	if (min_framelength_en)
		ctx->min_frame_length = ctx->frame_length;
	DRV_LOG(ctx, "max_fps(input/output):%u/%u, min_fl_en:%u\n",
		framerate, ctx->current_fps, min_framelength_en);
}	/*	set_max_framerate  */

void omegac1tele_extend_frame_length_convert(struct subdrv_ctx *ctx, u32 ns)
{
	return ;
//	int i;
//	u32 last_exp_cnt = 1;
//	u32 old_fl = ctx->frame_length;
//	u32 calc_fl = 0;
//	u32 readoutLength = 0;
//	u32 readMargin = 0;
//	u32 per_frame_ns = (u32)(((u64)ctx->frame_length *
//		(u64)ctx->line_length * 1000000000) / (u64)ctx->pclk);
//
//	check_current_scenario_id_bound(ctx);
//	readoutLength = ctx->s_ctx.mode[ctx->current_scenario_id].readout_length;
//	readMargin = ctx->s_ctx.mode[ctx->current_scenario_id].read_margin;
//
//	for (i = 1; i < ARRAY_SIZE(ctx->exposure); i++)
//		last_exp_cnt += ctx->exposure[i] ? 1 : 0;
//	if (ns)
//		ctx->frame_length = (u32)(((u64)(per_frame_ns + ns)) *
//			ctx->frame_length / per_frame_ns);
//	if (last_exp_cnt > 1) {
//		calc_fl = (readoutLength + readMargin);
//		for (i = 1; i < last_exp_cnt; i++)
//			calc_fl += (ctx->exposure[i] + ctx->s_ctx.exposure_margin * last_exp_cnt);
//		ctx->frame_length = max(calc_fl, ctx->frame_length);
//	}
//	omegac1tele_set_dummy(ctx);
//	ctx->extend_frame_length_en = TRUE;
//
//	ns = (u32)(((u64)(ctx->frame_length - old_fl) *
//		(u64)ctx->line_length * 1000000000) / (u64)ctx->pclk);
//		LOG_INF("fll(old/new):%u/%u, add %u ns", old_fl, ctx->frame_length, ns);
}

static void omegac1tele_extend_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 ns = (u32)((u64*)para)[0];

	omegac1tele_extend_frame_length_convert(ctx, ns);
}

bool omegac1tele_set_auto_flicker(struct subdrv_ctx *ctx, bool min_framelength_en)
{
	u16 framerate = 0;

	if (!ctx->line_length) {
		DRV_LOGE(ctx, "line_length(%u) is invalid\n", ctx->line_length);
		return FALSE;
	}

	if (!ctx->frame_length) {
		DRV_LOGE(ctx, "frame_length(%u) is invalid\n", ctx->frame_length);
		return FALSE;
	}

	framerate = ctx->pclk / ctx->line_length * 10 / ctx->frame_length;

	DRV_LOG(ctx, "cur_fps:%u, flick_en:%u, min_fl_en:%u\n",
		framerate, ctx->autoflicker_en, min_framelength_en);
	if (!ctx->autoflicker_en)
		return FALSE;

	if (framerate > 592 && framerate <= 607)
		omegac1tele_set_max_framerate(ctx, 592, min_framelength_en);
	else if (framerate > 296 && framerate <= 305)
		omegac1tele_set_max_framerate(ctx, 296, min_framelength_en);
	else if (framerate > 246 && framerate <= 253)
		omegac1tele_set_max_framerate(ctx, 246, min_framelength_en);
	else if (framerate > 236 && framerate <= 243)
		omegac1tele_set_max_framerate(ctx, 236, min_framelength_en);
	else if (framerate > 146 && framerate <= 153)
		omegac1tele_set_max_framerate(ctx, 146, min_framelength_en);
	else
		return FALSE;
	return TRUE;
}

static void omegac1tele_set_video_mode(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u16 framerate = (u32)((u64*)para)[0];

	if (!framerate)
		return;
	omegac1tele_set_max_framerate(ctx, framerate, 0);
	omegac1tele_set_auto_flicker(ctx, 1);
	omegac1tele_set_dummy(ctx);
	LOG_INF("fps(input/max):%u/%u\n", framerate, ctx->current_fps);
}

static void omegac1tele_set_sensor_rmsc_mode(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *)para;
	u32 TimeQbc = FRAME_FRAMEDURATION_MS;
	struct mtk_sensor_rmsc_mode *rmsc_mode = (struct mtk_sensor_rmsc_mode *)feature_data;

	if (bRmscMode == rmsc_mode->qbc_rmsc_mode) {
		return;
	}
	TimeQbc = 1000/(ctx->pclk/ctx->frame_length/ctx->line_length);
	TimeQbc = TimeQbc < (FRAME_FRAMEDURATION_MS) ? (FRAME_FRAMEDURATION_MS) : TimeQbc;
	msleep(TimeQbc);
	LOG_INF("rmsc_mode.qbc_rmsc_mode = %d, TimeQbc %d\n", rmsc_mode->qbc_rmsc_mode, TimeQbc);
	switch (rmsc_mode->qbc_rmsc_mode) {
	case IMGSENSOR_QBC_RMSC_DISABLED:
		subdrv_i2c_wr_u16(ctx, 0xFCFC, 0x2001);
		subdrv_i2c_wr_u16(ctx, 0x31D0, 0x0000);
		subdrv_i2c_wr_u16(ctx, 0xFCFC, 0x4000);
		subdrv_i2c_wr_u16(ctx, 0x0B08, 0x0100);
		LOG_INF("Disable sensor QBC_RMSC_EN. Switch to qual bayer.\n");
		break;
	case IMGSENSOR_QBC_RMSC_ENABLED:
		subdrv_i2c_wr_u16(ctx, 0xFCFC, 0x2001);
		subdrv_i2c_wr_u16(ctx, 0x31D0, 0x0200);
		subdrv_i2c_wr_u16(ctx, 0xFCFC, 0x4000);
		subdrv_i2c_wr_u16(ctx, 0x0B08, 0x0101);
		LOG_INF("Enable sensor QBC_RMSC_EN. Switch to bayer.\n");
		break;
        }
    bRmscMode = rmsc_mode->qbc_rmsc_mode;
}

static void omegac1tele_set_awb_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len) {

	struct SET_SENSOR_AWB_GAIN *awb_gain = (struct SET_SENSOR_AWB_GAIN *)para;

	adaptor_i2c_wr_u16(ctx->i2c_client, ctx->i2c_write_id >> 1, 0x0D82, awb_gain->ABS_GAIN_R * 2); //red 1024(1x)
	adaptor_i2c_wr_u16(ctx->i2c_client, ctx->i2c_write_id >> 1, 0x0D86, awb_gain->ABS_GAIN_B * 2); //blue

	DRV_LOG(ctx, "ABS_GAIN_GR(%d) ABS_GAIN_R(%d) ABS_GAIN_B(%d) ABS_GAIN_GB(%d)",
		awb_gain->ABS_GAIN_GR, awb_gain->ABS_GAIN_R, awb_gain->ABS_GAIN_B, awb_gain->ABS_GAIN_GB);

	return;
}

static bool dump_i2c_enable = false;

static void dump_i2c_buf(struct subdrv_ctx *ctx, u8 * buf, u32 length)
{
	int i;
	char *out_str = NULL;
	char *strptr = NULL;
	size_t buf_size = SUBDRV_I2C_BUF_SIZE * sizeof(char);
	size_t remind = buf_size;
	int num = 0;

	out_str = kzalloc(buf_size + 1, GFP_KERNEL);
	if (!out_str)
		return;

	strptr = out_str;
	memset(out_str, 0, buf_size + 1);

	num = snprintf(strptr, remind,"[ ");
	remind -= num;
	strptr += num;

	for (i = 0 ; i < length; i ++) {
		num = snprintf(strptr, remind,"0x%02x, ", buf[i]);

		if (num <= 0) {
			DRV_LOG(ctx, "snprintf return negative at line %d\n", __LINE__);
			kfree(out_str);
			return;
		}

		remind -= num;
		strptr += num;

		if (remind <= 20) {
			DRV_LOG(ctx, " write %s\n", out_str);
			memset(out_str, 0, buf_size + 1);
			strptr = out_str;
			remind = buf_size;
		}
	}

	num = snprintf(strptr, remind," ]");
	remind -= num;
	strptr += num;

	DRV_LOG(ctx, " write %s\n", out_str);
	strptr = out_str;
	remind = buf_size;

	kfree(out_str);
}

static int omegac1tele_i2c_burst_wr_regs_u16(struct subdrv_ctx * ctx, u16 * list, u32 len)
{
	adapter_i2c_burst_wr_regs_u16(ctx, ctx->i2c_write_id >> 1, list, len);
	return 	0;
}

static int adapter_i2c_burst_wr_regs_u16(struct subdrv_ctx * ctx ,
		u16 addr, u16 *list, u32 len)
{
	struct i2c_client *i2c_client = ctx->i2c_client;
	struct i2c_msg  msg;
	struct i2c_msg *pmsg = &msg;

	u8 *pbuf = NULL;
	u16 *plist = NULL;
	u16 *plist_end = NULL;

	u32 sent = 0;
	u32 total = 0;
	u32 per_sent = 0;
	int ret, i;

	if(!msg_buf) {
		LOG_INF("malloc msg_buf retry");
		msg_buf = kmalloc(MAX_BURST_LEN, GFP_KERNEL);
		if(!msg_buf) {
			LOG_INF("malloc error");
			return -ENOMEM;
		}
	}

	/* each msg contains addr(u16) + val(u16 *) */
	sent = 0;
	total = len / 2;
	plist = list;
	plist_end = list + len - 2;

	DRV_LOG(ctx, "len(%u)  total(%u)", len, total);

	while (sent < total) {

		per_sent = 0;
		pmsg = &msg;
		pbuf = msg_buf;

		pmsg->addr = addr;
		pmsg->flags = i2c_client->flags;
		pmsg->buf = pbuf;

		pbuf[0] = plist[0] >> 8;    //address
		pbuf[1] = plist[0] & 0xff;

		pbuf[2] = plist[1] >> 8;  //data 1
		pbuf[3] = plist[1] & 0xff;

		pbuf += 4;
		pmsg->len = 4;
		per_sent += 1;

		for (i = 0; i < total - sent - 1; i++) {  //Maximum number of remaining cycles - 1
			if(plist[0] + 2 == plist[2] ) {  //Addresses are consecutive
				pbuf[0] = plist[3] >> 8;
				pbuf[1] = plist[3] & 0xff;

				pbuf += 2;
				pmsg->len += 2;
				per_sent += 1;
				plist += 2;

				if(pmsg->len >= MAX_BURST_LEN) {
					break;
				}
			}
		}
		plist += 2;

		if(dump_i2c_enable) {
			DRV_LOG(ctx, "pmsg->len(%d) buff: ", pmsg->len);
			dump_i2c_buf(ctx, msg_buf, pmsg->len);
		}

		ret = i2c_transfer(i2c_client->adapter, pmsg, 1);

		if (ret < 0) {
			dev_info(&i2c_client->dev,
				"i2c transfer failed (%d)\n", ret);
			return -EIO;
		}

		sent += per_sent;

		DRV_LOG(ctx, "sent(%u)  total(%u)  per_sent(%u)", sent, total, per_sent);
	}

	return 0;
}

#define OMEGAC1TELE_AF_READ_ID  (0x19)  //dw9800s
#define OMEGAC1TELE_AF_POSITON_ADD  (0x03)

static bool read_af_pos(struct subdrv_ctx *ctx, u16 *positon)
{
	int ret;
	u8 buf[2];
	struct i2c_msg msg[2];
	struct i2c_client *i2c_client = ctx->i2c_client;

	buf[0] = OMEGAC1TELE_AF_POSITON_ADD;

	msg[0].addr = OMEGAC1TELE_AF_READ_ID >> 1;
	msg[0].flags = i2c_client->flags;
	msg[0].buf = buf;
	msg[0].len = 1;

	msg[1].addr  = OMEGAC1TELE_AF_READ_ID >> 1;
	msg[1].flags = i2c_client->flags | I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = 2;

	ret = i2c_transfer(i2c_client->adapter, msg, 2);
	if (ret < 0) {
		dev_info(&i2c_client->dev, "i2c transfer failed (%d)\n", ret);
		return false;
	}

	*positon = ((u16)buf[0] << 8) | buf[1];

	return true;
}

static u16 lens_position_setting[] = {
	0xFCFC, 0x2001,
	0x2566, 0x0000,
	0xFCFC, 0x4000,
};

static void omegac1tele_lens_pos_writeback(struct subdrv_ctx *ctx)
{
	kal_uint16 af_pos = 0;
	kal_uint16 write_pos = 0;
	bool ret;

	if (ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM3 ||  // RMSC
		ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM5) { // izoom

		ret = read_af_pos(ctx, &af_pos);
		if(ret == false || g_af_code_macro == 0 || g_af_code_infinity == 0 || g_af_code_macro == g_af_code_infinity) {
			pr_err("%s ret(%d) ",__func__, ret);
			return ;
		}
		if(af_pos < g_af_code_infinity) {
			af_pos = g_af_code_infinity;
		}
		if(af_pos > g_af_code_macro) {
			af_pos = g_af_code_macro;
		}

		write_pos = (u32)(af_pos - g_af_code_infinity) * 1023 / (g_af_code_macro - g_af_code_infinity);
		lens_position_setting[3] = write_pos;

		DRV_LOG(ctx,"%s af_pos(%d), write_pos(0x%x)", __func__, af_pos, write_pos);

		subdrv_i2c_wr_regs_u16(ctx, lens_position_setting, ARRAY_SIZE(lens_position_setting));
	}
}

static void omegac1tele_set_register(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	subdrv_i2c_wr_u16(ctx, 0x6028, ((MSDK_SENSOR_REG_INFO_STRUCT *)para)->RegAddr>>16);
	subdrv_i2c_wr_u16(ctx, 0x602A, ((MSDK_SENSOR_REG_INFO_STRUCT *)para)->RegAddr);
	subdrv_i2c_wr_u16(ctx, 0x6F12, ((MSDK_SENSOR_REG_INFO_STRUCT *)para)->RegData);
	pr_err("Indirect write RegAddr: 0x%08x, RegData: 0x%04x \n",
		((MSDK_SENSOR_REG_INFO_STRUCT *)para)->RegAddr, ((MSDK_SENSOR_REG_INFO_STRUCT *)para)->RegData);
}

static void omegac1tele_get_register(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	subdrv_i2c_wr_u16(ctx, 0x602C, ((MSDK_SENSOR_REG_INFO_STRUCT *)para)->RegAddr>>16);
	subdrv_i2c_wr_u16(ctx, 0x602E, ((MSDK_SENSOR_REG_INFO_STRUCT *)para)->RegAddr);
	((MSDK_SENSOR_REG_INFO_STRUCT *)para)->RegData = subdrv_i2c_rd_u16(ctx, 0x6F12);
	pr_err("Indirect read  RegAddr: 0x%08x, RegData: 0x%04x \n",
		((MSDK_SENSOR_REG_INFO_STRUCT *)para)->RegAddr, ((MSDK_SENSOR_REG_INFO_STRUCT *)para)->RegData);
}

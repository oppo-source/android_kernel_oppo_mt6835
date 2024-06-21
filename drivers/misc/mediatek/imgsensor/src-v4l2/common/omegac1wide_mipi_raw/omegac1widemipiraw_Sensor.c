// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 OPLUS. All rights reserved.
 */
/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 omegac1widemipiraw_Sensor.c
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
#include "omegac1widemipiraw_Sensor.h"

#define PFX "omegac1wide_camera_sensor"
#define LOG_INF(format, args...) pr_err(PFX "[%s] " format, __func__, ##args)

#define OMEGAC1WIDE_EEPROM_READ_ID	(0xA3)
#define OMEGAC1WIDE_EEPROM_WRITE_ID	(0xA2)
#define OMEGAC1WIDE_MAX_OFFSET		(0x8000)
#define OPLUS_CAMERA_COMMON_DATA_LENGTH 40

#define OTP_SIZE    (0x8000)
#define SENSOR_ID   (0x355)

#ifdef  EEPROM_WRITE_DATA_MAX_LENGTH
#undef  EEPROM_WRITE_DATA_MAX_LENGTH
#endif
#define EEPROM_WRITE_DATA_MAX_LENGTH        (64)
#define OMEGAC1WIDE_STEREO_START_ADDR       (0x2E00)
#define OMEGAC1WIDE_AESYNC_START_ADDR       (0x348D)

static kal_uint8 otp_data_checksum[OTP_SIZE] = {0};
static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
static void omegac1wide_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void omegac1wide_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void omegac1wide_set_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void omegac1wide_get_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void omegac1wide_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void omegac1wide_get_min_shutter_by_scenario_adapter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static int open(struct subdrv_ctx *ctx);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt);
static void omegac1wide_get_eeprom_comdata(struct subdrv_ctx *ctx, u8 *para, u32 *len);
/* static void omegac1wide_get_unique_sensorid(struct subdrv_ctx *ctx, u8 *para, u32 *len); */
static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size);
static void omegac1wide_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void omegac1wide_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void omegac1wide_set_shutter_frame_length_convert(struct subdrv_ctx *ctx, u32 shutter, u32 frame_length);
static bool omegac1wide_set_long_exposure(struct subdrv_ctx *ctx, u32 shutter);

/* STRUCT */
static struct oplus_get_eeprom_common_data omegac1wide_eeprom_common_data = {0};
/* static BYTE omegac1wide_unique_id[OMEGAC1WIDE_UNIQUE_SENSOR_ID_LENGHT] = { 0 }; */

static struct eeprom_map_info omegac1wide_eeprom_info[] = {
	{ EEPROM_META_MODULE_ID, 0x0000, 0x000f, 0x0010, 2, true },
	{ EEPROM_META_SENSOR_ID, 0x0006, 0x000f, 0x0010, 2, true },
	{ EEPROM_META_LENS_ID, 0x0008, 0x000f, 0x0010, 2, true },
	{ EEPROM_META_VCM_ID, 0x000A, 0x000f, 0x0010, 2, true },
	{ EEPROM_META_MIRROR_FLIP, 0x000E, 0x000f, 0x0010, 1, true },
	{ EEPROM_META_MODULE_SN, 0x00B0, 0x00C1, 0x00C2, 17, true },
	{ EEPROM_META_AF_CODE, 0x0000, 0x0000, 0x0000, 0, false },  /*fix focus*/
	{ EEPROM_META_AF_FLAG, 0x0000, 0x0000, 0x0000, 0, false },
	{ EEPROM_META_STEREO_DATA, 0x0000, 0x0000, 0x0000, 0, false },
	{ EEPROM_META_STEREO_MW_MAIN_DATA, OMEGAC1WIDE_STEREO_START_ADDR, 0x0000, 0x0000, CALI_DATA_SLAVE_LENGTH, true },
	{ EEPROM_META_STEREO_MT_MAIN_DATA, 0, 0, 0, 0, false },
	{ EEPROM_META_STEREO_MT_MAIN_DATA_105CM, 0, 0, 0, 0, false },
	{ EEPROM_META_DISTORTION_DATA, 0, 0, 0, 0, false },
};

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, omegac1wide_set_test_pattern},
	{SENSOR_FEATURE_CHECK_SENSOR_ID, omegac1wide_check_sensor_id},
	{SENSOR_FEATURE_GET_EEPROM_COMDATA, omegac1wide_get_eeprom_comdata},
	{SENSOR_FEATURE_SET_SENSOR_OTP, omegac1wide_set_eeprom_calibration},
	{SENSOR_FEATURE_GET_EEPROM_STEREODATA, omegac1wide_get_eeprom_calibration},
	{SENSOR_FEATURE_GET_SENSOR_OTP_ALL, omegac1wide_get_otp_checksum_data},
	{SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO, omegac1wide_get_min_shutter_by_scenario_adapter},
	{SENSOR_FEATURE_SET_ESHUTTER, omegac1wide_set_shutter},
	{SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME, omegac1wide_set_shutter_frame_length},
};

static struct eeprom_info_struct eeprom_info[] = {
	{
		.header_id = 0x00650006,
		.addr_header_id = 0x00000006,
		.i2c_write_id = 0xA2,
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_prev_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 3264,
			.vsize = 2448,
			.user_data_desc = VC_STAGGER_NE,
		},
	}
};

static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 3264,
			.vsize = 1840,
			.user_data_desc = VC_STAGGER_NE,
		},
	}
};

static struct mtk_mbus_frame_desc_entry frame_desc_hs[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1408,
			.vsize = 792,
			.user_data_desc = VC_STAGGER_NE,
		},
	}
};

static struct mtk_mbus_frame_desc_entry frame_desc_slim[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 3264,
			.vsize = 2448,
			.user_data_desc = VC_STAGGER_NE,
		},
	}
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus1[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 2304,
			.vsize = 1728,
			.user_data_desc = VC_STAGGER_NE,
		},
	}
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus2[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1632,
			.vsize = 1224,
			.user_data_desc = VC_STAGGER_NE,
		},
	}
};

static struct subdrv_mode_struct mode_struct[] = {
	{	/* reg-A1 3264x2448 @30FPS  VB as large as possible */
		.frame_desc = frame_desc_prev_cap,
		.num_entries = ARRAY_SIZE(frame_desc_prev_cap),
		.mode_setting_table = omegac1wide_preview_capture_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1wide_preview_capture_setting),
		.hdr_mode = HDR_NONE,
		.pclk = 288000000,
		.linelength = 3672,
		.framelength = 2612,
		.max_framerate = 300,
		.mipi_pixel_rate = 288000000,
		.readout_length = 0,
		.read_margin = 38,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 1,
		.imgsensor_winsize_info = {
			.full_w = 3280,
			.full_h = 2464,
			.x0_offset = 8,
			.y0_offset = 8,
			.w0_size = 3264,
			.h0_size = 2448,
			.scale_w = 3264,
			.scale_h = 2448,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
		},
		.ae_binning_ratio = 1,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 76,
		},
		.ana_gain_max = BASEGAIN * 16,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 30,
		},
	},

	{	/* reg-A1 3264x2448 @30FPS  VB as large as possible */
		.frame_desc = frame_desc_prev_cap,
		.num_entries = ARRAY_SIZE(frame_desc_prev_cap),
		.mode_setting_table = omegac1wide_preview_capture_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1wide_preview_capture_setting),
		.hdr_mode = HDR_NONE,
		.pclk = 288000000,
		.linelength = 3672,
		.framelength = 2612,
		.max_framerate = 300,
		.mipi_pixel_rate = 288000000,
		.readout_length = 0,
		.read_margin = 38,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 1,
		.imgsensor_winsize_info = {
			.full_w = 3280,
			.full_h = 2464,
			.x0_offset = 8,
			.y0_offset = 8,
			.w0_size = 3264,
			.h0_size = 2448,
			.scale_w = 3264,
			.scale_h = 2448,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
		},
		.ae_binning_ratio = 1,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 76,
		},
		.ana_gain_max = BASEGAIN * 16,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},

	{ /* reg-B1 3264x1840 @30FPS DataRate 720M VB as large as possible */
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = omegac1wide_normal_video_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1wide_normal_video_setting),
		.hdr_mode = HDR_NONE,
		.pclk = 288000000,
		.linelength = 3672,
		.framelength = 2612,
		.max_framerate = 300,
		.mipi_pixel_rate = 288000000,
		.readout_length = 0,
		.read_margin = 38,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 1,
		.imgsensor_winsize_info = {
			.full_w = 3280,
			.full_h = 2464,
			.x0_offset = 0,
			.y0_offset = 312,
			.w0_size = 3280,
			.h0_size = 1840,
			.scale_w = 3280,
			.scale_h = 1840,
			.x1_offset = 8,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 1840,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 1840,
		},
		.ae_binning_ratio = 1,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 76,
		},
		.ana_gain_max = BASEGAIN * 16,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 30,
		},
	},

	{
		.frame_desc = frame_desc_hs,
		.num_entries = ARRAY_SIZE(frame_desc_hs),
		.mode_setting_table = omegac1wide_hs_video_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1wide_hs_video_setting),
		.hdr_mode = HDR_NONE,
		.pclk = 288000000,
		.linelength = 3672,
		.framelength = 2612,
		.max_framerate = 300,
		.mipi_pixel_rate = 288000000,
		.readout_length = 0,
		.read_margin = 38,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 1,
		.imgsensor_winsize_info = {
			.full_w = 3280,
			.full_h = 2464,
			.x0_offset = 0,
			.y0_offset = 312,
			.w0_size = 3280,
			.h0_size = 1840,
			.scale_w = 3280,
			.scale_h = 1840,
			.x1_offset = 8,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 1840,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 1840,
		},
		.ae_binning_ratio = 1,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 76,
		},
		.ana_gain_max = BASEGAIN * 16,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},

	{
		.frame_desc = frame_desc_slim,
		.num_entries = ARRAY_SIZE(frame_desc_slim),
		.mode_setting_table = omegac1wide_slim_video_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1wide_slim_video_setting),
		.hdr_mode = HDR_NONE,
		.pclk = 288000000,
		.linelength = 3672,
		.framelength = 2612,
		.max_framerate = 300,
		.mipi_pixel_rate = 288000000,
		.readout_length = 0,
		.read_margin = 38,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 1,
		.imgsensor_winsize_info = {
			.full_w = 3280,
			.full_h = 2464,
			.x0_offset = 0,
			.y0_offset = 312,
			.w0_size = 3280,
			.h0_size = 1840,
			.scale_w = 3280,
			.scale_h = 1840,
			.x1_offset = 8,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 1840,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 1840,
		},
		.ae_binning_ratio = 1,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 76,
		},
		.ana_gain_max = BASEGAIN * 16,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},

	{	/* reg-I1 2304x1728 @24FPS  VB as large as possible */
		.frame_desc = frame_desc_cus1,
		.num_entries = ARRAY_SIZE(frame_desc_cus1),
		.mode_setting_table = omegac1wide_custom1_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1wide_custom1_setting),
		.hdr_mode = HDR_NONE,
		.pclk = 288000000,
		.linelength = 3672,
		.framelength = 3266,
		.max_framerate = 240,
		.mipi_pixel_rate = 288000000,
		.readout_length = 0,
		.read_margin = 38,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 1,
		.imgsensor_winsize_info = {
			.full_w = 3280,
			.full_h = 2464,
			.x0_offset = 0,
			.y0_offset = 368,
			.w0_size = 3280,
			.h0_size = 1728,
			.scale_w = 3280,
			.scale_h = 1728,
			.x1_offset = 488,
			.y1_offset = 0,
			.w1_size = 2304,
			.h1_size = 1728,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2304,
			.h2_tg_size = 1728,
		},
		.ae_binning_ratio = 1,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 76,
		},
		.ana_gain_max = BASEGAIN * 16,
		.sensor_setting_info = {
			.sensor_scenario_usage = UNUSE_MASK,
			.equivalent_fps = 0,
		},
	},
	{	/* reg-K 1632x1224 @24FPS VB Max */
		.frame_desc = frame_desc_cus2,
		.num_entries = ARRAY_SIZE(frame_desc_cus2),
		.mode_setting_table = omegac1wide_custom2_setting,
		.mode_setting_len = ARRAY_SIZE(omegac1wide_custom2_setting),
		.hdr_mode = HDR_NONE,
		.pclk = 144000000,
		.linelength = 1836,
		.framelength = 3266,
		.max_framerate = 240,
		.mipi_pixel_rate = 288000000,
		.readout_length = 0,
		.read_margin = 38,
		.framelength_step = 1,
		.coarse_integ_step = 1,
		.min_exposure_line = 1,
		.imgsensor_winsize_info = {
			.full_w = 3280,
			.full_h = 2464,
			.x0_offset = 8,
			.y0_offset = 8,
			.w0_size = 3264,
			.h0_size = 2448,
			.scale_w = 1632,
			.scale_h = 1224,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 1632,
			.h1_size = 1224,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1632,
			.h2_tg_size = 1224,
		},
		.ae_binning_ratio = 1,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 76,
		},
		.ana_gain_max = BASEGAIN * 16,
		.sensor_setting_info = {
			.sensor_scenario_usage = NORMAL_MASK,
			.equivalent_fps = 24,
		},
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = OMEGAC1WIDE_SENSOR_ID,
	.reg_addr_sensor_id = {0x0016, 0x0017},
	.i2c_addr_table = {0x34, 0xFF},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8,
	.eeprom_info = eeprom_info,
	.eeprom_num = ARRAY_SIZE(eeprom_info),
	.resolution = {3264, 2448},
	.mirror = IMAGE_HV_MIRROR,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 16,
	.ana_gain_type = 0,
	.ana_gain_step = 1,
	.ana_gain_table = omegac1wide_ana_gain_table,
	.ana_gain_table_size = sizeof(omegac1wide_ana_gain_table),
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 8,
	.exposure_max = (0xffff * 128) - 18,
	.exposure_step = 1,
	.exposure_margin = 18,

	.frame_length_max = 0xffff,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 3,
	.start_exposure_offset = 934000, /*xts used*/
	.pdaf_type = PDAF_SUPPORT_NA,

	.g_gain2reg = get_gain2reg,
	.s_gph = set_group_hold,

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = 0x0101,
	.reg_addr_exposure = {
			{0x0202, 0x0203}, /*COARSE_INTEG_TIME*/
	},
	.long_exposure_support = FALSE,
	.reg_addr_exposure_lshift = PARAM_UNDEFINED,
	.reg_addr_ana_gain = {
			{0x0204, 0x0205}, /*ANA_GAIN_GLOBAL*/
	},
	.reg_addr_frame_length = {0x0340, 0x0341},

	.reg_addr_auto_extend = 0x0350, /*FRM_LENGTH_CTL*/
	.reg_addr_frame_count = PARAM_UNDEFINED, /*FRM_CNT*/
	.reg_addr_fast_mode = 0x3010, /*FAST_MODETRANSIT_CTL*/

	.init_setting_table = omegac1wide_init_setting,
	.init_setting_len = ARRAY_SIZE(omegac1wide_init_setting),
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),

	.checksum_value = 0xD1EFF68B,
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
	{HW_ID_AVDD, 2804000, 0},
	{HW_ID_DVDD, 1200000, 0},
	{HW_ID_DOVDD, 1804000, 3},
	{HW_ID_MCLK_DRIVING_CURRENT, 4, 6},
	{HW_ID_RST, 1, 2},
};

const struct subdrv_entry omegac1wide_mipi_raw_entry = {
	.name = "omegac1wide_mipi_raw",
	.id = OMEGAC1WIDE_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

/* FUNCTION */

static unsigned int read_omegac1wide_eeprom_info(struct subdrv_ctx *ctx, kal_uint16 meta_id,
	BYTE *data, int size)
{
	kal_uint16 addr;
	int readsize;

	if (meta_id != omegac1wide_eeprom_info[meta_id].meta)
		return -1;

	if (size != omegac1wide_eeprom_info[meta_id].size)
		return -1;

	addr = omegac1wide_eeprom_info[meta_id].start;
	readsize = omegac1wide_eeprom_info[meta_id].size;

	if(!read_cmos_eeprom_p8(ctx, addr, data, readsize)) {
		DRV_LOGE(ctx, "read meta_id(%d) failed", meta_id);
	}

	return 0;
}

/*
static void read_unique_sensorid(struct subdrv_ctx *ctx)
{
	u8 i = 0;
	LOG_INF("read sensor unique sensorid");
	while (ctx->s_ctx.i2c_addr_table[i] != 0xFF) {
		ctx->i2c_write_id = ctx->s_ctx.i2c_addr_table[i];
		subdrv_i2c_wr_u8(ctx, 0x0A02, 0x7F);
		subdrv_i2c_wr_u8(ctx, 0x0A00, 0x01);
		adaptor_i2c_rd_p8(ctx->i2c_client, ctx->i2c_write_id >> 1,
		OMEGAC1WIDE_UNIQUE_SENSOR_ID, &(omegac1wide_unique_id[0]), OMEGAC1WIDE_UNIQUE_SENSOR_ID_LENGHT);
		i++;
	}
}

static void omegac1wide_get_unique_sensorid(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 *feature_return_para_32 = (u32 *)para;
	LOG_INF("get unique sensorid");
	memcpy(feature_return_para_32, omegac1wide_unique_id,
		OMEGAC1WIDE_UNIQUE_SENSOR_ID_LENGHT);
	LOG_INF("para :%x, get unique sensorid", *para);
}
*/

static void read_eeprom_common_data(struct subdrv_ctx *ctx)
{
	kal_uint16 idx = 0;

	memset(&omegac1wide_eeprom_common_data, 0x00, sizeof(omegac1wide_eeprom_common_data));

	omegac1wide_eeprom_common_data.header[EEPROM_MODULE_ID] = 2;
	read_omegac1wide_eeprom_info(ctx, EEPROM_META_MODULE_ID,
				&(omegac1wide_eeprom_common_data.data[idx]), 2);
	idx += omegac1wide_eeprom_common_data.header[EEPROM_MODULE_ID];

	omegac1wide_eeprom_common_data.header[EEPROM_SENSOR_ID] = 2;
	read_omegac1wide_eeprom_info(ctx, EEPROM_META_SENSOR_ID,
				&(omegac1wide_eeprom_common_data.data[idx]), 2);
	idx += omegac1wide_eeprom_common_data.header[EEPROM_SENSOR_ID];

	omegac1wide_eeprom_common_data.header[EEPROM_LENS_ID] = 2;
	read_omegac1wide_eeprom_info(ctx, EEPROM_META_LENS_ID,
				&(omegac1wide_eeprom_common_data.data[idx]), 2);
	idx += omegac1wide_eeprom_common_data.header[EEPROM_LENS_ID];

	omegac1wide_eeprom_common_data.header[EEPROM_VCM_ID] = 2;
	read_omegac1wide_eeprom_info(ctx, EEPROM_META_VCM_ID,
				&(omegac1wide_eeprom_common_data.data[idx]), 2);
	idx += omegac1wide_eeprom_common_data.header[EEPROM_VCM_ID];

	omegac1wide_eeprom_common_data.header[EEPROM_MODULE_SN] = 17;
	read_omegac1wide_eeprom_info(ctx, EEPROM_META_MODULE_SN,
				&(omegac1wide_eeprom_common_data.data[idx]), 17);
	idx += omegac1wide_eeprom_common_data.header[EEPROM_MODULE_SN];

	for (idx = 0; idx < 64; idx = idx + 4)
		LOG_INF("In %s:common data: %02x %02x %02x %02x\n", __func__,
			omegac1wide_eeprom_common_data.data[idx], omegac1wide_eeprom_common_data.data[idx + 1],
			omegac1wide_eeprom_common_data.data[idx + 2],
			omegac1wide_eeprom_common_data.data[idx + 3]);
}

static void omegac1wide_get_eeprom_comdata(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 *feature_return_para_32 = (u32 *)para;
	LOG_INF("+");

	if(*len == sizeof(omegac1wide_eeprom_common_data)) {
		memcpy(feature_return_para_32, &omegac1wide_eeprom_common_data,
		sizeof(omegac1wide_eeprom_common_data));
	}
}

static kal_uint16 read_cmos_eeprom_8(struct subdrv_ctx *ctx, kal_uint16 addr)
{
	kal_uint16 get_byte = 0;

	adaptor_i2c_rd_u8(ctx->i2c_client, OMEGAC1WIDE_EEPROM_READ_ID >> 1, addr, (u8 *)&get_byte);
	return get_byte;
}

static kal_int32 table_write_eeprom_one_packet(struct subdrv_ctx *ctx,
        kal_uint16 addr, kal_uint8 *para, kal_uint32 len)
{
	kal_int32 ret = ERROR_NONE;
	ret = adaptor_i2c_wr_p8(ctx->i2c_client, OMEGAC1WIDE_EEPROM_WRITE_ID >> 1,
			addr, para, len);

	return ret;
}

static kal_int32 write_eeprom_protect(struct subdrv_ctx *ctx, kal_uint16 enable)
{
	kal_int32 ret = ERROR_NONE;
	kal_uint16 reg = 0xE000;
	if (enable) {
		adaptor_i2c_wr_u8(ctx->i2c_client, OMEGAC1WIDE_EEPROM_WRITE_ID >> 1, reg, 0xA3);
	}
	else {
		adaptor_i2c_wr_u8(ctx->i2c_client, OMEGAC1WIDE_EEPROM_WRITE_ID >> 1, reg, 0xA2);
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

	for (i = 0; i < idx; i++) {
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

static kal_int32 write_Module_data(struct subdrv_ctx *ctx, ACDK_SENSOR_ENGMODE_STEREO_STRUCT * pStereodata)
{
	kal_int32  ret = ERROR_NONE;
	kal_uint16 data_base, data_length;
	kal_uint8 *pData;

	if (pStereodata != NULL) {
		LOG_INF("SET_SENSOR_OTP: 0x%x %d 0x%x %d\n",
						pStereodata->uSensorId,
						pStereodata->uDeviceId,
						pStereodata->baseAddr,
						pStereodata->dataLength);

		data_base = pStereodata->baseAddr;
		data_length = pStereodata->dataLength;
		pData = pStereodata->uData;
		if ((pStereodata->uSensorId == OMEGAC1WIDE_SENSOR_ID) && (data_length == CALI_DATA_SLAVE_LENGTH)
			&& (data_base == OMEGAC1WIDE_STEREO_START_ADDR)) {
			LOG_INF("Write: %x %x %x %x\n", pData[0], pData[39], pData[40], pData[1556]);

			eeprom_64align_write(ctx, data_base, pData, data_length);
			LOG_INF("com_0:0x%x\n", read_cmos_eeprom_8(ctx, data_base));
			LOG_INF("com_39:0x%x\n", read_cmos_eeprom_8(ctx, data_base+39));
			LOG_INF("innal_40:0x%x\n", read_cmos_eeprom_8(ctx, data_base+40));
			LOG_INF("innal_1556:0x%x\n", read_cmos_eeprom_8(ctx, data_base+1556));
			LOG_INF("write_Module_data Write end\n");

		} else if ((pStereodata->uSensorId == OMEGAC1WIDE_SENSOR_ID) && (data_length < AESYNC_DATA_LENGTH_TOTAL)
			&& (data_base == OMEGAC1WIDE_AESYNC_START_ADDR)) {
			LOG_INF("write main aesync: %x %x %x %x %x %x %x %x\n", pData[0], pData[1],
				pData[2], pData[3], pData[4], pData[5], pData[6], pData[7]);

			eeprom_64align_write(ctx, data_base, pData, data_length);

			LOG_INF("readback main aesync: %x %x %x %x %x %x %x %x\n",
					read_cmos_eeprom_8(ctx, OMEGAC1WIDE_AESYNC_START_ADDR),
					read_cmos_eeprom_8(ctx, OMEGAC1WIDE_AESYNC_START_ADDR+1),
					read_cmos_eeprom_8(ctx, OMEGAC1WIDE_AESYNC_START_ADDR+2),
					read_cmos_eeprom_8(ctx, OMEGAC1WIDE_AESYNC_START_ADDR+3),
					read_cmos_eeprom_8(ctx, OMEGAC1WIDE_AESYNC_START_ADDR+4),
					read_cmos_eeprom_8(ctx, OMEGAC1WIDE_AESYNC_START_ADDR+5),
					read_cmos_eeprom_8(ctx, OMEGAC1WIDE_AESYNC_START_ADDR+6),
					read_cmos_eeprom_8(ctx, OMEGAC1WIDE_AESYNC_START_ADDR+7));
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

static void omegac1wide_set_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	int ret = ERROR_NONE;
	ret = write_Module_data(ctx, (ACDK_SENSOR_ENGMODE_STEREO_STRUCT *)(para));
	if (ret != ERROR_NONE) {
		*len = (u32)-1; /*write eeprom failed*/
		LOG_INF("ret=%d\n", ret);
	}
}

static void omegac1wide_get_eeprom_calibration(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	UINT16 *feature_data_16 = (UINT16 *) para;
	UINT32 *feature_return_para_32 = (UINT32 *) para;
	if(*len > CALI_DATA_SLAVE_LENGTH)
		*len = CALI_DATA_SLAVE_LENGTH;
	LOG_INF("feature_data mode:%d  lens:%d", *feature_data_16, *len);
	read_omegac1wide_eeprom_info(ctx, EEPROM_META_STEREO_MW_MAIN_DATA,
			(BYTE *)feature_return_para_32, *len);
}

static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
                    BYTE *data, int size)
{
	if (adaptor_i2c_rd_p8(ctx->i2c_client, OMEGAC1WIDE_EEPROM_READ_ID >> 1,
			addr, data, size) < 0) {
		return false;
	}
	return true;
}

static void read_otp_info(struct subdrv_ctx *ctx)
{
	DRV_LOGE(ctx, "omegac1wide read_otp_info begin\n");
	read_cmos_eeprom_p8(ctx, 0, otp_data_checksum, OTP_SIZE);
	DRV_LOGE(ctx, "omegac1wide read_otp_info end\n");
}

static void omegac1wide_get_otp_checksum_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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

static void omegac1wide_check_sensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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
			if (*sensor_id == SENSOR_ID) {
				*sensor_id = ctx->s_ctx.sensor_id;
				if (first_read) {
					read_eeprom_common_data(ctx);
/*					read_unique_sensorid(ctx); */
					first_read = KAL_FALSE;
				}
				return ERROR_NONE;
			}
			DRV_LOGE(ctx, "Read sensor id fail. i2c_write_id: 0x%x\n", ctx->i2c_write_id);
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

	/* initail setting */
	sensor_init(ctx);

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
	return (1024 - (1024 * BASEGAIN) / gain);
}

void omegac1wide_get_min_shutter_by_scenario(struct subdrv_ctx *ctx,
		enum SENSOR_SCENARIO_ID_ENUM scenario_id,
		u64 *min_shutter, u64 *exposure_step)
{
	check_current_scenario_id_bound(ctx);
	DRV_LOG(ctx, "sensor_mode_num[%d]", ctx->s_ctx.sensor_mode_num);
	if (scenario_id < ctx->s_ctx.sensor_mode_num) {
		switch (ctx->s_ctx.mode[scenario_id].hdr_mode) {
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

void omegac1wide_get_min_shutter_by_scenario_adapter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *) para;
	omegac1wide_get_min_shutter_by_scenario(ctx,
		(enum SENSOR_SCENARIO_ID_ENUM)*(feature_data),
		feature_data + 1, feature_data + 2);
}

static void omegac1wide_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);

	if (mode != ctx->test_pattern) {
		LOG_INF("mode(%u->%u)\n", ctx->test_pattern, mode);

		/*1:Solid Color 2:Color bar 5:black*/
		if (mode) {
			switch(mode) {
			case 5:
				subdrv_i2c_wr_u8(ctx, 0x020E, 0x00);
				subdrv_i2c_wr_u8(ctx, 0x0218, 0x00);
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


bool omegac1wide_set_long_exposure(struct subdrv_ctx *ctx, u32 shutter)
{
	u16 l_shift = 0;

	if (shutter > (ctx->s_ctx.frame_length_max - ctx->s_ctx.exposure_margin)) {
		for (l_shift = 1; l_shift < 7; l_shift++) {
			if ((shutter >> l_shift)
				< (ctx->s_ctx.frame_length_max - ctx->s_ctx.exposure_margin))
				break;
		}
		if (l_shift > 7) {
			DRV_LOGE(ctx, "unable to set exposure:%u, set to max\n", shutter);
			l_shift = 7;
		}
		shutter = shutter >> l_shift;
		ctx->frame_length = shutter +  ctx->s_ctx.exposure_margin;

		DRV_LOG(ctx, "long exposure mode: lshift %u times", l_shift);
		set_i2c_buffer(ctx, 0x3060, subdrv_i2c_rd_u8(ctx, 0x3060) | (l_shift & 0x7));

		set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_frame_length.addr[0], (ctx->frame_length >> 8) & 0xFF);
		set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_frame_length.addr[1],  ctx->frame_length & 0xFF);

		set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[0].addr[0], (shutter >> 8) & 0xFF);
		set_i2c_buffer(ctx,	ctx->s_ctx.reg_addr_exposure[0].addr[1],  shutter & 0xFF);

		commit_i2c_buffer(ctx);
		/* Frame exposure mode customization for LE*/
		ctx->ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
		ctx->ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
		ctx->current_ae_effective_frame = 2;

		return TRUE;
	} else {
		set_i2c_buffer(ctx, 0x3060, subdrv_i2c_rd_u8(ctx, 0x3060) & 0xf8);
		ctx->current_ae_effective_frame = 2;

		return FALSE;
	}
}

static void omegac1wide_set_shutter_frame_length_convert(struct subdrv_ctx *ctx, u32 shutter, u32 frame_length)
{
	u32 fine_integ_line = 0;
	bool gph = !ctx->is_seamless && (ctx->s_ctx.s_gph != NULL);

	ctx->frame_length = frame_length ? frame_length : ctx->frame_length;
	check_current_scenario_id_bound(ctx);
	/* check boundary of shutter */
	fine_integ_line = ctx->s_ctx.mode[ctx->current_scenario_id].fine_integ_line;
	shutter = FINE_INTEG_CONVERT(shutter, fine_integ_line);
	shutter = max(shutter, ctx->s_ctx.exposure_min);
	shutter = min(shutter, ctx->s_ctx.exposure_max);
	/* check boundary of framelength */
	ctx->frame_length =	max(shutter + ctx->s_ctx.exposure_margin, ctx->min_frame_length);
	ctx->frame_length =	min(ctx->frame_length, ctx->s_ctx.frame_length_max);
	/* restore shutter */
	memset(ctx->exposure, 0, sizeof(ctx->exposure));
	ctx->exposure[0] = shutter;
	/* group hold start */
	if (gph)
		ctx->s_ctx.s_gph((void *)ctx, 1);

	if(omegac1wide_set_long_exposure(ctx, shutter)) {
		if (gph)
			ctx->s_ctx.s_gph((void *)ctx, 0);
		return;
	}

	/* enable auto extend */
	if (ctx->s_ctx.reg_addr_auto_extend)
		set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_auto_extend, 0x01);
	/* write framelength */
	if (set_auto_flicker(ctx, 0) || frame_length || !ctx->s_ctx.reg_addr_auto_extend)
		write_frame_length(ctx, ctx->frame_length);
	/* write shutter */

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
	DRV_LOG(ctx, "exp[0x%x], fll(input/output):%u/%u, flick_en:%u\n",
		ctx->exposure[0], frame_length, ctx->frame_length, ctx->autoflicker_en);
	if (!ctx->ae_ctrl_gph_en) {
		if (gph)
			ctx->s_ctx.s_gph((void *)ctx, 0);
		commit_i2c_buffer(ctx);
	}
	/* group hold end */
}

static void omegac1wide_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	omegac1wide_set_shutter_frame_length_convert(ctx, (u32)((u64*)para)[0], (u32)((u64*)para)[1]);
}

static void omegac1wide_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	omegac1wide_set_shutter_frame_length_convert(ctx, (u32)((u64*)para)[0], 0);
}

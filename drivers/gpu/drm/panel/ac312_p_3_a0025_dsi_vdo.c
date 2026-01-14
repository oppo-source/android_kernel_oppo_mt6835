// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/backlight.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_modes.h>
#include <linux/delay.h>
#include <drm/drm_connector.h>
#include <drm/drm_device.h>
#include <linux/of_graph.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <soc/oplus/device_info.h>
#include <linux/regulator/consumer.h>
#include <mt-plat/mtk_boot_common.h>

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#endif

/*#ifdef OPLUS_BUG_STABILITY*/
#include "../oplus/oplus_display_panel_power.h"
#include "../mediatek/mediatek_v2/mtk_log.h"

#include "ac312_p_3_a0025_dsi_vdo.h"
#include "oplus_bl.h"
#ifdef OPLUS_FEATURE_DISPLAY_ADFR
#include "../oplus/oplus_adfr_ext.h"
#endif /* OPLUS_FEATURE_DISPLAY_ADFR  */
/* #ifdef OPLUS_FEATURE_ONSCREENFINGERPRINT */
/* add for ofp */
#include "../oplus/oplus_display_onscreenfingerprint.h"
/* add for cmdq_pkt_sleep */
#include "../mediatek/mediatek_v2/mtk-cmdq-ext.h"
/* #endif */ /* OPLUS_FEATURE_ONSCREENFINGERPRINT */
/* add for dips_drv log  */
#include "../oplus/oplus_display_mtk_debug.h"
#include "../oplus/oplus_drm_disp_panel.h"
#define BRIGHTNESS_MAX    4094
#define BRIGHTNESS_HALF   2047
#define MAX_NORMAL_BRIGHTNESS   3083
#define LCM_BRIGHTNESS_TYPE 2
#define FHD_LCM_WIDTH  1080
#define FHD_LCM_HEIGHT 2372
extern unsigned int oplus_display_brightness;
extern unsigned int oplus_max_normal_brightness;
static int mode_id = -1;
static unsigned int lhbm_last_backlight = 0;

static unsigned int nt37703a_vdo_dphy_buf_thresh[14] ={896, 1792, 2688, 3584, 4480,
	5376, 6272, 6720, 7168, 7616, 7744, 7872, 8000, 8064};
static unsigned int nt37703a_vdo_dphy_range_min_qp[15] ={0, 4, 5, 5, 7, 7, 7, 7, 7,
	7, 9, 9, 9, 11, 17};
static unsigned int nt37703a_vdo_dphy_range_max_qp[15] ={8, 8, 9, 10, 11, 11, 11,
	12, 13, 14, 15, 16, 17, 17, 19};
static int nt37703a_vdo_dphy_range_bpg_ofs[15] ={2, 0, 0, -2, -4, -6, -8, -8, -8,
	-10, -10, -12, -12, -12, -12};

/* whether enter hbm brightness level or not */

extern void lcdinfo_notify(unsigned long val, void *v);
struct lcm_pmic_info {
	struct regulator *reg_vio1p8;
	struct regulator *reg_vio3P0;
};

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *vddr_enable_gpio;
	struct drm_display_mode *m;
	bool prepared;
	bool enabled;

	int error;
};

static inline struct lcm *panel_to_lcm(struct drm_panel *panel)
{
	return container_of(panel, struct lcm, panel);
}

#define lcm_dcs_write_seq(ctx, seq...)				\
	({												\
		const u8 d[] = { seq };						\
		BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 128,		\
				 "DCS sequence too big for stack");	\
		lcm_dcs_write(ctx, d, ARRAY_SIZE(d));		\
	})

#define lcm_dcs_write_seq_static(ctx, seq...)		\
	({												\
		static const u8 d[] = { seq };				\
		lcm_dcs_write(ctx, d, ARRAY_SIZE(d));		\
	})

static void lcm_dcs_write(struct lcm *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;
	char *addr;

	if (ctx->error < 0)
		return;

	addr = (char *)data;
	if ((int)*addr < 0xB0)
		ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	else
		ret = mipi_dsi_generic_write(dsi, data, len);
	if (ret < 0) {
		dev_info(ctx->dev, "error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

static void push_table(struct lcm *ctx, struct LCM_setting_table *table,
		unsigned int count)
{
	unsigned int i;
	unsigned int cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;
		switch (cmd) {
		case REGFLAG_DELAY:
			usleep_range(table[i].count*1000, table[i].count*1000 + 100);
			break;
		case REGFLAG_UDELAY:
			usleep_range(table[i].count, table[i].count + 100);
			break;
		case REGFLAG_END_OF_TABLE:
			break;
		default:
			lcm_dcs_write(ctx, table[i].para_list,
				table[i].count);
			break;
		}
	}
}

#ifdef PANEL_SUPPORT_READBACK
static int lcm_dcs_read(struct lcm *ctx, u8 cmd, void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error < 0)
		return 0;

	ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
	if (ret < 0) {
		DISP_ERR("error %d reading dcs seq:(%#x)\n", ret,
			 cmd);
		ctx->error = ret;
	}

	return ret;
}

static void lcm_panel_get_data(struct lcm *ctx)
{
	u8 buffer[3] = { 0 };
	static int ret;

	DISP_INFO("%s+\n");

	if (ret == 0) {
		ret = lcm_dcs_read(ctx, 0x0A, buffer, 1);
		DISP_DEBUG("return %d data(0x%08x) to dsi engine\n",ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif

#if 1
static struct lcm_pmic_info *g_pmic;
static unsigned int lcm_get_reg_vio1p8(void)
{
	unsigned int volt = 0;

	if (regulator_is_enabled(g_pmic->reg_vio1p8))
		/* regulator_get_voltage return volt with uV */
		volt = regulator_get_voltage(g_pmic->reg_vio1p8);

	return volt;
}

static unsigned int lcm_enable_reg_vio1p8(int en)
{
	unsigned int ret=0,volt = 0;
	if(en) {
		if (!IS_ERR_OR_NULL(g_pmic->reg_vio1p8)) {
			ret = regulator_enable(g_pmic->reg_vio1p8);
			DISP_INFO("Enable the Regulator vio1p8 ret=%d.\n",ret);
			volt = lcm_get_reg_vio1p8();
			DISP_DEBUG("get the Regulator vio1p8 =%d.\n",volt);
		}
	}else {
		if (!IS_ERR_OR_NULL(g_pmic->reg_vio1p8)) {
			ret = regulator_disable(g_pmic->reg_vio1p8);
			volt = lcm_get_reg_vio1p8();
			DISP_INFO("disable the Regulator vio1p8 ret=%d,volt=%d.\n",ret,volt);
		}
	}
	return ret;

}

static unsigned int lcm_get_reg_vio3P0(void)
{
	unsigned int volt = 0;

	if (regulator_is_enabled(g_pmic->reg_vio3P0))
		/* regulator_get_voltage return volt with uV */
		volt = regulator_get_voltage(g_pmic->reg_vio3P0);

	return volt;
}

static unsigned int lcm_enable_reg_vio3P0(int en)
{
	unsigned int ret=0,volt = 0;
	if(en) {
		if (!IS_ERR_OR_NULL(g_pmic->reg_vio3P0)) {
			ret = regulator_enable(g_pmic->reg_vio3P0);
			DISP_INFO("Enable the Regulator vio3p0 ret=%d.\n",ret);
			volt = lcm_get_reg_vio3P0();
			DISP_DEBUG("get the Regulator vio3p0 =%d.\n",volt);
		}
	}else {
		if (!IS_ERR_OR_NULL(g_pmic->reg_vio3P0)) {
			ret = regulator_disable(g_pmic->reg_vio3P0);
			volt = lcm_get_reg_vio3P0();
			DISP_INFO("disable the Regulator vio3p0 ret=%d,volt=%d.\n",ret,volt);
		}
	}
	return ret;

}
#endif

static struct drm_display_mode *get_mode_by_id(struct drm_connector *connector,
	unsigned int mode)
{
	struct drm_display_mode *m;
	unsigned int i = 0;

	list_for_each_entry(m, &connector->modes, head) {
		if (i == mode)
			return m;
		i++;
	}

	return NULL;
}

static int get_mode_enum(struct drm_display_mode *m)
{
	int ret = 0;
	int m_vrefresh = 0;

	if (m == NULL) {
		return -EINVAL;
	}

	m_vrefresh = drm_mode_vrefresh(m);

	if (m_vrefresh == 60) {
		ret = FHD_SDC60;
	} else if (m_vrefresh == 90) {
		ret = FHD_SDC90;
	} else if (m_vrefresh == 120) {
		ret = FHD_SDC120;
	} else if (m_vrefresh == 30) {
		ret = FHD_SDC30;
	} else {
		ret = FHD_SDC60;
	}
	return ret;
}

static void lcm_panel_init(struct lcm *ctx)
{
	pr_info("%s, mode_id = %d\n", __func__, mode_id);
	switch (mode_id) {
	case FHD_SDC60:
		pr_info("%s, fhd_dsi_on_cmd_sdc60\n", __func__);
		push_table(ctx, dsi_on_cmd_sdc60, sizeof(dsi_on_cmd_sdc60)/sizeof(struct LCM_setting_table));
		break;
	case FHD_SDC90:
		pr_info("%s, fhd_dsi_on_cmd_sdc90\n", __func__);
		push_table(ctx, dsi_on_cmd_sdc90, sizeof(dsi_on_cmd_sdc90)/sizeof(struct LCM_setting_table));
		break;
	case FHD_SDC120:
		pr_info("%s, fhd_dsi_on_cmd_sdc120\n", __func__);
		push_table(ctx, dsi_on_cmd_sdc120, sizeof(dsi_on_cmd_sdc120)/sizeof(struct LCM_setting_table));
		break;
	default:
		pr_info("%s, fhd_dsi_on_cmd_sdc120\n", __func__);
		push_table(ctx, dsi_on_cmd_sdc120, sizeof(dsi_on_cmd_sdc120)/sizeof(struct LCM_setting_table));
		break;
	}
	pr_info("%s,successful\n", __func__);

}

static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (!ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = false;

	return 0;
}

static int lcm_unprepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	DISP_INFO("prepared=%d\n",ctx->prepared);

	if (!ctx->prepared) {
		return 0;
	}

	lcm_dcs_write_seq_static(ctx, 0x28);
	usleep_range(25000, 25100);
	lcm_dcs_write_seq_static(ctx, 0x10);
	usleep_range(125000, 125100);
	ctx->error = 0;
	ctx->prepared = false;
	DISP_INFO("lcm_unprepare Successful\n");

	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared) {
		return 0;
	}

	lcm_panel_init(ctx);
	usleep_range(2000, 2100);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;
#ifdef PANEL_SUPPORT_READBACK
	lcm_panel_get_data(ctx);
#endif

	DISP_INFO("lcm_prepare Successful\n");
	return ret;
}

static int lcm_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = true;

	return 0;
}

#define VAC (2376)
#define HAC (1080)
static u32 fake_heigh = 2372;
static u32 fake_width = 1080;
static bool need_fake_resolution;

#define FRAME_WIDTH             (1080)
#define FRAME_HEIGHT            (2376)
#define HFP                     (180)
#define HBP                     (8)
#define HSA                     (6)
#define VFP_60HZ                (2492)
#define VFP_90HZ                (860)
#define VFP_120HZ               (44)
#define VBP                     (26)
#define VSA                     (2)
#define MIPI_CLK                (565)
#define DATA_RATE               (1130)
#define HFP_30HZ                (2100)
#define HBP_30HZ                (8)
#define HSA_30HZ                (6)
#define VFP_30HZ                (44)
#define VBP_30HZ                (26)
#define VSA_30HZ                (2)

static struct drm_display_mode display_mode[MODE_NUM] = {
	//sdc_120_mode
	{
		.clock = ((HAC + HFP + HBP + HSA) * (VAC + VFP_120HZ + VBP + VSA) * 120) / 1000,
		.hdisplay = HAC,
		.hsync_start = HAC + HFP,
		.hsync_end = HAC + HFP + HSA,
		.htotal = HAC + HFP + HSA + HBP,
		.vdisplay = VAC,
		.vsync_start = VAC + VFP_120HZ,
		.vsync_end = VAC + VFP_120HZ + VSA,
		.vtotal = VAC + VFP_120HZ + VSA + VBP,
		.hskew = SDC_ADFR,
	},
	//sdc_90_mode
	{
		.clock = ((HAC + HFP + HBP + HSA) * (VAC + VFP_90HZ + VBP + VSA) * 90) / 1000,
		.hdisplay = HAC,
		.hsync_start = HAC + HFP,
		.hsync_end = HAC + HFP + HSA,
		.htotal = HAC + HFP + HSA + HBP,
		.vdisplay = VAC,
		.vsync_start = VAC + VFP_90HZ,
		.vsync_end = VAC + VFP_90HZ + VSA,
		.vtotal = VAC + VFP_90HZ + VSA + VBP,
		.hskew = SDC_ADFR,
	},
	//sdc_60_mode
	{
		.clock = ((HAC + HFP + HBP + HSA) * (VAC + VFP_60HZ + VBP + VSA) * 60) / 1000,
		.hdisplay = HAC,
		.hsync_start = HAC + HFP,
		.hsync_end = HAC + HFP + HSA,
		.htotal = HAC + HFP + HSA + HBP,
		.vdisplay = VAC,
		.vsync_start = VAC + VFP_60HZ,
		.vsync_end = VAC + VFP_60HZ + VSA,
		.vtotal = VAC + VFP_60HZ + VSA + VBP,
		.hskew = SDC_MFR,
	},
	//sdc_30_mode
	{
		.clock = ((HAC + HFP_30HZ + HBP_30HZ + HSA_30HZ) * (VAC + VFP_30HZ + VBP_30HZ + VSA_30HZ) * 30) / 1000,
		.hdisplay = HAC,
		.hsync_start = HAC + HFP_30HZ,
		.hsync_end = HAC + HFP_30HZ + HSA_30HZ,
		.htotal = HAC + HFP_30HZ + HSA_30HZ + HBP_30HZ,
		.vdisplay = VAC,
		.vsync_start = VAC + VFP_30HZ,
		.vsync_end = VAC + VFP_30HZ + VSA_30HZ,
		.vtotal = VAC + VFP_30HZ + VSA_30HZ + VBP_30HZ,
		.hskew = SDC_MFR,
	},
};

#if defined(CONFIG_MTK_PANEL_EXT)
static struct mtk_panel_params ext_params[MODE_NUM] = {
	//fhd_sdc_120_mode
	{
	.pll_clk = MIPI_CLK,
	// .phy_timcon = {
	// 	.hs_trail = 14,
	// 	.clk_trail = 15,
	// },
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.esd_check_multi = 0,
	.lcm_esd_check_table[0] = {
                .cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
        },
	.lcm_esd_check_table[1] = {
		.cmd = 0xAB, .count = 2, .para_list[0] = 0x00, .para_list[1] = 0x00, .mask_list[0] = 0x0F, .mask_list[1] = 0x07,
	},
	.lcm_esd_check_table[2] = {
		.cmd = 0x91, .count = 2, .para_list[0] = 0xAB, .para_list[1] = 0x28,
	},
	.color_dual_panel_status = false,
	.color_dual_brightness_status = true,
	.vdo_per_frame_lp_enable = 0,
	.change_fps_by_vfp_send_cmd = 1,
	.color_vivid_status = true,
	.vendor = "AC312_P_3_A0025",
	.manufacture = "P_3",
	.lane_swap_en = 0,
	.lane_swap[0][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[0][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[0][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[0][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[0][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[0][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[1][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[1][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[1][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[1][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.oplus_display_color_mode_suppor = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 40,
		.rct_on = 1,
		.bit_per_channel = 10,
		.dsc_line_buf_depth = 11,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2376,
		.pic_width = 1080,
		.slice_height = 8,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 512,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 187,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 3511,
		.slice_bpg_offset = 3255,
		.initial_offset = 6144,
		.final_offset = 4336,
		.flatness_minqp = 7,
		.flatness_maxqp = 16,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 15,
		.rc_quant_incr_limit1 = 15,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
		.ext_pps_cfg = {
			.enable = 1,
			.rc_buf_thresh = nt37703a_vdo_dphy_buf_thresh,
			.range_min_qp = nt37703a_vdo_dphy_range_min_qp,
			.range_max_qp = nt37703a_vdo_dphy_range_max_qp,
			.range_bpg_ofs = nt37703a_vdo_dphy_range_bpg_ofs,
			},
		},
	.data_rate = DATA_RATE,
	.oplus_serial_para0 = 0xA3,
	.dyn_fps = {
		.switch_en = 1, .vact_timing_fps = 120,
		.dfps_cmd_table[0] = {0, 2 , {0x2F, 0x01}},
		.dfps_cmd_table[1] = {0, 6 , {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00}},
		.dfps_cmd_table[2] = {0, 2 , {0x6F, 0x31}},
		.dfps_cmd_table[3] = {0, 2 , {0xDF, 0x00}},
		},
	.panel_bpp = 10,
	},
	//fhd_sdc_90_mode
	{
	.pll_clk = MIPI_CLK,
	// .phy_timcon = {
	// 	.hs_trail = 14,
	// 	.clk_trail = 15,
	// },
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.esd_check_multi = 0,
	.lcm_esd_check_table[0] = {
                .cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
        },
	.lcm_esd_check_table[1] = {
		.cmd = 0xAB, .count = 2, .para_list[0] = 0x00, .para_list[1] = 0x00, .mask_list[0] = 0x0F, .mask_list[1] = 0x07,
	},
	.lcm_esd_check_table[2] = {
		.cmd = 0x91, .count = 2, .para_list[0] = 0xAB, .para_list[1] = 0x28,
	},
	.vdo_per_frame_lp_enable = 0,
	.change_fps_by_vfp_send_cmd = 1,
	.color_dual_panel_status = false,
	.color_dual_brightness_status = true,
	.color_vivid_status = true,
	.vendor = "AC312_P_3_A0025",
	.manufacture = "P_3",
	.lane_swap_en = 0,
	.lane_swap[0][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[0][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[0][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[0][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[0][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[0][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[1][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[1][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[1][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[1][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.oplus_display_color_mode_suppor = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 40,
		.rct_on = 1,
		.bit_per_channel = 10,
		.dsc_line_buf_depth = 11,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2376,
		.pic_width = 1080,
		.slice_height = 8,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 512,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 187,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 3511,
		.slice_bpg_offset = 3255,
		.initial_offset = 6144,
		.final_offset = 4336,
		.flatness_minqp = 7,
		.flatness_maxqp = 16,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 15,
		.rc_quant_incr_limit1 = 15,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
		.ext_pps_cfg = {
			.enable = 1,
			.rc_buf_thresh = nt37703a_vdo_dphy_buf_thresh,
			.range_min_qp = nt37703a_vdo_dphy_range_min_qp,
			.range_max_qp = nt37703a_vdo_dphy_range_max_qp,
			.range_bpg_ofs = nt37703a_vdo_dphy_range_bpg_ofs,
			},
		},
	.data_rate = DATA_RATE,
	.oplus_serial_para0 = 0xA3,
	.dyn_fps = {
		.switch_en = 1, .vact_timing_fps = 90,
		.dfps_cmd_table[0] = {0, 2 , {0x2F, 0x02}},
		.dfps_cmd_table[1] = {0, 6 , {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00}},
		.dfps_cmd_table[2] = {0, 2 , {0x6F, 0x31}},
		.dfps_cmd_table[3] = {0, 2 , {0xDF, 0x01}},
		},
	.panel_bpp = 10,
	},
	//fhd_sdc_60_mode
	{
	.pll_clk = MIPI_CLK,
	// .phy_timcon = {
	// 	.hs_trail = 14,
	// 	.clk_trail = 15,
	// },
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.esd_check_multi = 0,
	.lcm_esd_check_table[0] = {
                .cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
        },
	.lcm_esd_check_table[1] = {
		.cmd = 0xAB, .count = 2, .para_list[0] = 0x00, .para_list[1] = 0x00, .mask_list[0] = 0x0F, .mask_list[1] = 0x07,
	},
	.lcm_esd_check_table[2] = {
		.cmd = 0x91, .count = 2, .para_list[0] = 0xAB, .para_list[1] = 0x28,
	},
	.vdo_per_frame_lp_enable = 0,
	.change_fps_by_vfp_send_cmd = 1,
	.color_dual_panel_status = false,
	.color_dual_brightness_status = true,
	.color_vivid_status = true,
	.vendor = "AC312_P_3_A0025",
	.manufacture = "P_3",
	.lane_swap_en = 0,
	.lane_swap[0][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[0][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[0][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[0][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[0][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[0][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[1][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[1][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[1][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[1][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.oplus_display_color_mode_suppor = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 40,
		.rct_on = 1,
		.bit_per_channel = 10,
		.dsc_line_buf_depth = 11,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2376,
		.pic_width = 1080,
		.slice_height = 8,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 512,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 187,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 3511,
		.slice_bpg_offset = 3255,
		.initial_offset = 6144,
		.final_offset = 4336,
		.flatness_minqp = 7,
		.flatness_maxqp = 16,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 15,
		.rc_quant_incr_limit1 = 15,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
		.ext_pps_cfg = {
			.enable = 1,
			.rc_buf_thresh = nt37703a_vdo_dphy_buf_thresh,
			.range_min_qp = nt37703a_vdo_dphy_range_min_qp,
			.range_max_qp = nt37703a_vdo_dphy_range_max_qp,
			.range_bpg_ofs = nt37703a_vdo_dphy_range_bpg_ofs,
			},
		},
	.data_rate = DATA_RATE,
	.oplus_serial_para0 = 0xA3,
	.dyn_fps = {
		.switch_en = 1, .vact_timing_fps = 60,
		.dfps_cmd_table[0] = {0, 2 , {0x2F, 0x03}},
		.dfps_cmd_table[1] = {0, 6 , {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00}},
		.dfps_cmd_table[2] = {0, 2 , {0x6F, 0x31}},
		.dfps_cmd_table[3] = {0, 2 , {0xDF, 0x02}},
		},
	.panel_bpp = 10,
	},
	//fhd_sdc_30_mode
	{
	.pll_clk = MIPI_CLK,
	// .phy_timcon = {
	// 	.hs_trail = 14,
	// 	.clk_trail = 15,
	// },
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.esd_check_multi = 0,
	.lcm_esd_check_table[0] = {
                .cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
        },
	.vdo_per_frame_lp_enable = 0,
	.change_fps_by_vfp_send_cmd = 1,
	.color_dual_panel_status = false,
	.color_dual_brightness_status = true,
	.color_vivid_status = true,
	.vendor = "AC312_P_3_A0025",
	.manufacture = "P_3",
	.lane_swap_en = 0,
	.lane_swap[0][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[0][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[0][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[0][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[0][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[0][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[1][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[1][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[1][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[1][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 40,
		.rct_on = 1,
		.bit_per_channel = 10,
		.dsc_line_buf_depth = 11,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2376,
		.pic_width = 1080,
		.slice_height = 8,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 512,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 187,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 3511,
		.slice_bpg_offset = 3255,
		.initial_offset = 6144,
		.final_offset = 4336,
		.flatness_minqp = 7,
		.flatness_maxqp = 16,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 15,
		.rc_quant_incr_limit1 = 15,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
		.ext_pps_cfg = {
			.enable = 1,
			.rc_buf_thresh = nt37703a_vdo_dphy_buf_thresh,
			.range_min_qp = nt37703a_vdo_dphy_range_min_qp,
			.range_max_qp = nt37703a_vdo_dphy_range_max_qp,
			.range_bpg_ofs = nt37703a_vdo_dphy_range_bpg_ofs,
			},
		},
	.data_rate = DATA_RATE,
	.oplus_serial_para0 = 0xA3,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 30,
		.dfps_cmd_table[0] = {0, 2 , {0x2F, 0x04}},
	},
	.panel_bpp = 10,
	},
};

static int mtk_panel_ext_param_get(struct drm_panel *panel,
		struct drm_connector *connector,
		struct mtk_panel_params **ext_param,
		unsigned int id)
{
	int ret = 0;
	struct drm_display_mode *m = get_mode_by_id(connector, id);

	mode_id = get_mode_enum(m);

	if (mode_id == FHD_SDC120) {
		*ext_param = &ext_params[0];
	} else if (mode_id == FHD_SDC90) {
		*ext_param = &ext_params[1];
	} else if (mode_id == FHD_SDC60) {
		*ext_param = &ext_params[2];
	} else if (mode_id == FHD_SDC30) {
		*ext_param = &ext_params[3];
	} else {
		*ext_param = &ext_params[0];
	}

	if (*ext_param)
		DISP_DEBUG("data_rate:%d\n", (*ext_param)->data_rate);
	else
		DISP_ERR("ext_param is NULL;\n");

	return ret;
}

static int mtk_panel_ext_param_set(struct drm_panel *panel,
			 struct drm_connector *connector, unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	struct drm_display_mode *m = get_mode_by_id(connector, mode);

	mode_id = get_mode_enum(m);

	DISP_INFO("mode:%d,mode_id:%d\n",mode,mode_id);

	if (mode_id == FHD_SDC120) {
		ext->params = &ext_params[0];
	} else if (mode_id == FHD_SDC90) {
		ext->params = &ext_params[1];
	} else if (mode_id == FHD_SDC60) {
		ext->params = &ext_params[2];
	} else if (mode_id == FHD_SDC30) {
		ext->params = &ext_params[3];
	} else {
		ext->params = &ext_params[0];
	}

	return ret;
}

static int panel_ata_check(struct drm_panel *panel)
{
	/* Customer test by own ATA tool */
	return 1;
}

static int lcm_get_virtual_heigh(void)
{
	return VAC;
}

static int lcm_get_virtual_width(void)
{
	return HAC;
}

static void check_is_need_fake_resolution(struct device *dev)
{
	unsigned int ret = 0;

	ret = of_property_read_u32(dev->of_node, "fake-heigh", &fake_heigh);
	if (ret)
		need_fake_resolution = false;
	ret = of_property_read_u32(dev->of_node, "fake-width", &fake_width);
	if (ret)
		need_fake_resolution = false;
	if (fake_heigh > 0 && fake_heigh < VAC)
		need_fake_resolution = true;
	if (fake_width > 0 && fake_width < HAC)
		need_fake_resolution = true;

	DISP_INFO("need_fake_resolution:%d,%s-\n", need_fake_resolution,__func__);
}

static void change_drm_disp_mode_params(struct drm_display_mode *mode)
{
	int vtotal = mode->vtotal;
	int htotal = mode->htotal;
	int fps = mode->clock * 1000 / vtotal / htotal;

	if (fake_heigh > 0 && fake_heigh < VAC) {
		mode->vsync_start = mode->vsync_start - mode->vdisplay
					+ fake_heigh;
		mode->vsync_end = mode->vsync_end - mode->vdisplay + fake_heigh;
		mode->vtotal = mode->vtotal - mode->vdisplay + fake_heigh;
		mode->vdisplay = fake_heigh;
	}
	if (fake_width > 0 && fake_width < HAC) {
		mode->hsync_start = mode->hsync_start - mode->hdisplay
					+ fake_width;
		mode->hsync_end = mode->hsync_end - mode->hdisplay + fake_width;
		mode->htotal = mode->htotal - mode->hdisplay + fake_width;
		mode->hdisplay = fake_width;
	}

	if (fps > 100)
		fps = 120;
	else if (fps > 70)
		fps = 90;
	else if (fps > 50)
		fps = 60;
	else
		fps= 30;
	mode->clock = fps * mode->vtotal * mode->htotal / 1000;
	mode->clock += 1;

	DISP_INFO("mode->vdisplay:%d,%s-\n",mode->vdisplay, __func__);
}

static unsigned int demura_tap = 0;
static unsigned int lhbm_70nit_off_action = 0;
static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle, unsigned int level)
{
	unsigned int mapped_level = 0;
	int i = 0;
	unsigned char bl_level[] = {0x51, 0x03, 0xFF};

	if (!dsi || !cb) {
		return -EINVAL;
	}

	if (level == 0) {
		DISP_ERR("[%s:%d]backlight lvl:%u\n", __func__, __LINE__, level);
	}

	if (level == 1) {
		DISP_ERR("[%s:%d]backlight lvl:%u\n", __func__, __LINE__, level);
		return 0;
	} else if (level > 4095) {
		level = 4095;
	}

	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT && level > 0){
		level = 2047;
	}

	mapped_level = level;
	if (mapped_level > 1) {
		lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &mapped_level);
	}

	bl_level[1] = level >> 8;
	bl_level[2] = level & 0xFF;
	cb(dsi, handle, bl_level, ARRAY_SIZE(bl_level));
	DISP_ERR("ac312 backlight = %d bl_level[1]=%x, bl_level[2]=%x\n", level, bl_level[1], bl_level[2]);
	oplus_display_brightness = level;
	lhbm_last_backlight = level;
	lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &level);

	if (lhbm_70nit_off_action == 0 && demura_tap != 3) {
		if (mapped_level <= 1147 && demura_tap != 1) {
			demura_tap = 1;
			DISP_ERR("ac312_p_3_a0025 backlight send demura0\n");
			for (i = 0; i < sizeof(dsi_demura0_bl)/sizeof(struct LCM_setting_table); i++){
				cb(dsi, handle, dsi_demura0_bl[i].para_list, dsi_demura0_bl[i].count);
			}
		} else if(mapped_level > 1147 && demura_tap != 2) {
			demura_tap = 2;
			DISP_ERR("ac312_p_3_a0025 backlight send demura1\n");
			for (i = 0; i < sizeof(dsi_demura1_bl)/sizeof(struct LCM_setting_table); i++){
				cb(dsi, handle, dsi_demura1_bl[i].para_list, dsi_demura1_bl[i].count);
			}
		}
	} else if (lhbm_70nit_off_action == 1) {
		if (mapped_level <= 1147) {
			demura_tap = 1;
			DISP_ERR("ac312_p_3_a0025 backlight send demura0 , lhbm_70nit_off_action = 1 \n");
			for (i = 0; i < sizeof(dsi_demura0_bl)/sizeof(struct LCM_setting_table); i++){
				cb(dsi, handle, dsi_demura0_bl[i].para_list, dsi_demura0_bl[i].count);
			}
		} else if(mapped_level > 1147) {
			demura_tap = 2;
			DISP_ERR("ac312_p_3_a0025 backlight send demura1 , lhbm_70nit_off_action = 1 \n");
			for (i = 0; i < sizeof(dsi_demura1_bl)/sizeof(struct LCM_setting_table); i++){
				cb(dsi, handle, dsi_demura1_bl[i].para_list, dsi_demura1_bl[i].count);
			}
		}
		lhbm_70nit_off_action = 0;
	}

	return 0;
}

static int oplus_esd_backlight_recovery(void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int level = oplus_display_brightness;
	int i = 0;
	unsigned char esd_bl_level[] = {0x51, 0x03, 0xFF};

	if (!dsi || !cb) {
		return -EINVAL;
	}

	esd_bl_level[1] = level >> 8;
	esd_bl_level[2] = level & 0xFF;
	cb(dsi, handle, esd_bl_level, ARRAY_SIZE(esd_bl_level));
	lhbm_last_backlight = level;
	DISP_ERR("esd_bl_level[1]=%x, esd_bl_level[2]=%x backlight = %d\n", esd_bl_level[1], esd_bl_level[2], level);

	if (lhbm_70nit_off_action == 0 && demura_tap != 3) {
		if (level <= 1147 && demura_tap != 1) {
			demura_tap = 1;
			DISP_ERR("ac312_p_3_a0025 esd send demura_0\n");
			for (i = 0; i < sizeof(dsi_demura0_bl)/sizeof(struct LCM_setting_table); i++){
				cb(dsi, handle, dsi_demura0_bl[i].para_list, dsi_demura0_bl[i].count);
			}
		} else if(level > 1147 && demura_tap != 2){
			demura_tap = 2;
			DISP_ERR("ac312_p_3_a0025 esd send demura_1\n");
			for (i = 0; i < sizeof(dsi_demura1_bl)/sizeof(struct LCM_setting_table); i++){
				cb(dsi, handle, dsi_demura1_bl[i].para_list, dsi_demura1_bl[i].count);
			}
		}
	}

	return 0;
}

/* #ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
static int lcm_set_hbm(void *dsi, dcs_write_gce cb, void *handle, unsigned int hbm_mode)
{
	unsigned int i = 0;

	if (!dsi || !cb) {
		pr_err("Invalid params\n");
		return -EINVAL;
	}

	OFP_INFO("%s,oplus_display_brightness=%d, hbm_mode=%u\n", __func__, oplus_display_brightness, hbm_mode);
	if (hbm_mode == 1) {
		for (i = 0; i < sizeof(hbm_on_cmd)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, hbm_on_cmd[i].para_list, hbm_on_cmd[i].count);
		}
	} else if (hbm_mode == 0) {
		for (i = 0; i < sizeof(hbm_off_cmd)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, hbm_off_cmd[i].para_list, hbm_off_cmd[i].count);
		}
		lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);
	}

	OFP_DEBUG("end\n");

	return 0;
}

static int oplus_ofp_set_lhbm_pressed_icon(struct drm_panel *panel, void *dsi,
		dcs_write_gce cb, void *handle, bool en)
{
	unsigned int reg_count = 0;
	struct lcm *ctx = NULL;
	struct LCM_setting_table *lhbm_pressed_icon_cmd = NULL;
	struct LCM_setting_table *lhbm_pressed_icon_off_cmd = NULL;
	int i = 0;
	OFP_DEBUG("start\n");

	if (!oplus_ofp_local_hbm_is_enabled()) {
		OFP_DEBUG("local hbm is not enabled, should not set lhbm pressed icon\n");
	}

	if (!panel || !dsi || !cb) {
		OFP_ERR("Invalid input params\n");
		return -EINVAL;
	}

	ctx = panel_to_lcm(panel);
	if (!ctx) {
		OFP_ERR("Invalid ctx params\n");
	}

	OFP_INFO("%s,oplus_display_brightness=%d, hbm_mode=%d, lhbm_last_backlight %d\n",
			 __func__, oplus_display_brightness, en,  lhbm_last_backlight);
	if (en) {
		demura_tap = 3;
		OFP_INFO("[LHBM_ON_ACTION] set demura_tap = 3\n");
		if (lhbm_last_backlight <= 1147) {
			reg_count = sizeof(lhbm_pressed_icon_on_cmd_below_70nit) / sizeof(struct LCM_setting_table);
			lhbm_pressed_icon_cmd = lhbm_pressed_icon_on_cmd_below_70nit;
		} else {
			reg_count = sizeof(lhbm_pressed_icon_on_cmd_above_70nit) / sizeof(struct LCM_setting_table);
			lhbm_pressed_icon_cmd = lhbm_pressed_icon_on_cmd_above_70nit;
		}
		for (i = 0; i < reg_count; i++) {
			cb(dsi, handle, lhbm_pressed_icon_cmd[i].para_list, lhbm_pressed_icon_cmd[i].count);
		}
	} else if (en == 0) {
		if (lhbm_last_backlight <= 1147) {
			reg_count = sizeof(lhbm_pressed_icon_off_cmd_below_70nit) / sizeof(struct LCM_setting_table);
			lhbm_pressed_icon_off_cmd = lhbm_pressed_icon_off_cmd_below_70nit;
		} else {
			reg_count = sizeof(lhbm_pressed_icon_off_cmd_above_70nit) / sizeof(struct LCM_setting_table);
			lhbm_pressed_icon_off_cmd = lhbm_pressed_icon_off_cmd_above_70nit;
		}
		for (i = 0; i < reg_count; i++) {
			cb(dsi, handle, lhbm_pressed_icon_off_cmd[i].para_list, lhbm_pressed_icon_off_cmd[i].count);
		}
		lhbm_70nit_off_action = 1;
		lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);
	}
	return 0;
}

static int panel_doze_disable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int i = 0;
	unsigned int cmd;

	if (!panel || !dsi) {
		pr_err("Invalid dsi params\n");
	}

	for (i = 0; i < (sizeof(aod_off_cmd) / sizeof(struct LCM_setting_table)); i++) {

		cmd = aod_off_cmd[i].cmd;
		switch (cmd) {
			case REGFLAG_DELAY:
				if (handle == NULL) {
					usleep_range(aod_off_cmd[i].count * 1000, aod_off_cmd[i].count * 1000 + 100);
				} else {
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(aod_off_cmd[i].count * 1000), CMDQ_GPR_R14);
				}
				break;
			case REGFLAG_UDELAY:
				if (handle == NULL) {
					usleep_range(aod_off_cmd[i].count, aod_off_cmd[i].count + 100);
				} else {
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(aod_off_cmd[i].count), CMDQ_GPR_R14);
				}
				break;
			case REGFLAG_END_OF_TABLE:
				break;
			default:
				cb(dsi, handle, aod_off_cmd[i].para_list, aod_off_cmd[i].count);
		}
	}

	OFP_INFO("%s:success\n", __func__);
	//atomic_set(&esd_pending, 0);
	return 0;
}

static int panel_doze_enable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int i = 0;
	unsigned int cmd;
	unsigned int reg_count = 0;
	struct LCM_setting_table *lhbm_pressed_icon_off_cmd = NULL;
	if (!panel || !dsi) {
		pr_err("Invalid dsi params\n");
	}

	if(oplus_ofp_local_hbm_is_enabled() && oplus_ofp_get_hbm_state()) {
		if (oplus_display_brightness > 0 && oplus_display_brightness <= 1147) {
			reg_count = sizeof(lhbm_pressed_icon_off_cmd_below_70nit) / sizeof(struct LCM_setting_table);
			lhbm_pressed_icon_off_cmd = lhbm_pressed_icon_off_cmd_below_70nit;
		} else {
			reg_count = sizeof(lhbm_pressed_icon_off_cmd_above_70nit) / sizeof(struct LCM_setting_table);
			lhbm_pressed_icon_off_cmd = lhbm_pressed_icon_off_cmd_above_70nit;
		}
		for (i = 0; i < reg_count; i++) {
			cb(dsi, handle, lhbm_pressed_icon_off_cmd[i].para_list, lhbm_pressed_icon_off_cmd[i].count);
		}
		OFP_INFO("should off hbm\n");
	}

	//atomic_set(&esd_pending, 1);
	for (i = 0; i < (sizeof(aod_on_cmd)/sizeof(struct LCM_setting_table)); i++) {
		cmd = aod_on_cmd[i].cmd;
		switch (cmd) {
			case REGFLAG_DELAY:
				usleep_range(aod_on_cmd[i].count * 1000, aod_on_cmd[i].count * 1000 + 100);
				break;
			case REGFLAG_UDELAY:
				usleep_range(aod_on_cmd[i].count, aod_on_cmd[i].count + 100);
				break;
			case REGFLAG_END_OF_TABLE:
				break;
			default:
				{
					cb(dsi, handle, aod_on_cmd[i].para_list, aod_on_cmd[i].count);
				}
		}
	}

	OFP_INFO("%s:success\n", __func__);
	return 0;
}

static int panel_set_aod_light_mode(void *dsi, dcs_write_gce cb, void *handle, unsigned int level)
{
	int i = 0;

	if (level == 0) {
		for (i = 0; i < sizeof(aod_high_mode)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, aod_high_mode[i].para_list, aod_high_mode[i].count);
		}
	} else {
		for (i = 0; i < sizeof(aod_low_mode)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, aod_low_mode[i].para_list, aod_low_mode[i].count);
		}
	}
	OFP_INFO("level = %d\n", level);
	return 0;
}

static struct vdo_aod_params vdo_aod_on = {
	.porch_change_flag = 0x03,
	.dst_hfp = 2100,
	.dst_vfp = 48, //30fps
	.mode_idx = FHD_SDC30,
	.change_mmclk = false,
	.vdo_aod_cmd_table[0]={1, {0x39} },
	.vdo_aod_cmd_table[1]={2, {0x6F,0x04} },
	.vdo_aod_cmd_table[2]={3, {0x51,0x0F,0xFE}},
};


static struct vdo_aod_params vdo_aod_to_120hz = {
	.porch_change_flag = 0x03,
	.dst_hfp = 180,
	.dst_vfp = 48,
	.mode_idx = 0,
	.change_mmclk = false,
	.vdo_aod_cmd_table[0]={1, {0x38}},

};

static struct vdo_aod_params vdo_aod_to_120hz_unlocking = {
	.porch_change_flag = 0x03,
	.dst_hfp = 180,
	.dst_vfp = 48,
	.mode_idx = 0,
	.change_mmclk = false,
	.vdo_aod_cmd_table[0]={1, {0x38}},
	.vdo_aod_cmd_table[1]={3, {0x51, 0x00, 0x00}},

};

static struct vdo_aod_params vdo_aod_to_90hz = {
	.porch_change_flag = 0x03,
	.dst_hfp = 180,
	.dst_vfp = 864,
	.mode_idx = 1,
	.change_mmclk = false,
	.vdo_aod_cmd_table[0]={1, {0x38}},

};

static struct vdo_aod_params vdo_aod_to_90hz_unlocking = {
	.porch_change_flag = 0x03,
	.dst_hfp = 180,
	.dst_vfp = 864,
	.mode_idx = 1,
	.change_mmclk = false,
	.vdo_aod_cmd_table[0]={1, {0x38}},
	.vdo_aod_cmd_table[1]={3, {0x51, 0x00, 0x00}},
};

static struct vdo_aod_params vdo_aod_to_60hz = {
	.porch_change_flag = 0x03,
	.dst_hfp = 180,
	.dst_vfp = 2492,
	.mode_idx = 2,
	.change_mmclk = false,
	.vdo_aod_cmd_table[0]={1, {0x38}},

};

static struct vdo_aod_params vdo_aod_to_60hz_unlocking = {
	.porch_change_flag = 0x03,
	.dst_hfp = 180,
	.dst_vfp = 2492,
	.mode_idx = 2,
	.change_mmclk = false,
	.vdo_aod_cmd_table[0]={1, {0x38}},
	.vdo_aod_cmd_table[1]={3, {0x51, 0x00, 0x00}},
};

static unsigned int aod_status = 0;
static int mtk_get_vdo_aod_param(int aod_en, struct vdo_aod_params **vdo_aod_param)
{

	static int mode_id_before_aod = 2;
	if (aod_en) {
		*vdo_aod_param = &vdo_aod_on;
		mode_id_before_aod = mode_id;
	}
	else {
		if (mode_id_before_aod == FHD_SDC60) {
			if (oplus_ofp_get_aod_unlocking())
				*vdo_aod_param = &vdo_aod_to_60hz_unlocking;
			else {
				*vdo_aod_param = &vdo_aod_to_60hz;
				OFP_INFO("%s:before mode_id %d\n", __func__, mode_id_before_aod);
			}
		} else if (mode_id_before_aod == FHD_SDC90) {
			if (oplus_ofp_get_aod_unlocking())
				*vdo_aod_param = &vdo_aod_to_90hz_unlocking;
			else {
				*vdo_aod_param = &vdo_aod_to_90hz;
			}
		} else {
			if(oplus_ofp_get_aod_unlocking())
				*vdo_aod_param = &vdo_aod_to_120hz_unlocking;
			else {
				*vdo_aod_param = &vdo_aod_to_120hz;
				OFP_INFO("%s:before mode_id %d\n", __func__, mode_id_before_aod);
			}
		}
		if (oplus_ofp_get_aod_unlocking())
			lhbm_last_backlight = 0;
	}
	OFP_INFO("%s:aod_en %d, mode_id %d, unlocking =%d\n", __func__, aod_en, mode_id, oplus_ofp_get_aod_unlocking());
	aod_status = aod_en;
	return 0;
}

/*#endif OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT*/

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);

	DISP_INFO("reset on=%d\n",on);

	gpiod_set_value(ctx->reset_gpio, on);

	return 0;
}

static int lcm_panel_reset(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (ctx->prepared) {
		DISP_DEBUG("ctx->prepared:%d return! \n",ctx->prepared);
		return 0;
	}
	usleep_range(12000,12100);
	gpiod_set_value(ctx->reset_gpio,1);
	usleep_range(2000,2100);
	gpiod_set_value(ctx->reset_gpio,0);
	usleep_range(2000,2100);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(25000,25100);
	DISP_INFO("Successful\n");

	return 0;
}

static int lcm_panel_poweron(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared){
		return 0;
	}
	DISP_INFO("%s+\n", __func__);

	/* vddi 1.8v */
	ret = regulator_set_voltage(g_pmic->reg_vio1p8,
				1800000, 1800000);
	ret=lcm_enable_reg_vio1p8(1);
	usleep_range(5000, 5100);
	/* vddr 1.2v */
	gpiod_set_value(ctx->vddr_enable_gpio, 1);
	usleep_range(1000, 1100);
	/* vci 3.0v */
	ret = regulator_set_voltage(g_pmic->reg_vio3P0,
				3000000, 3000000);
	ret=lcm_enable_reg_vio3P0(1);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);
	usleep_range(11000, 11100);
	DISP_INFO("%s-\n", __func__);
	return 0;
}

static int lcm_panel_poweroff(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared) {
		return 0;
	}
	DISP_INFO("%s+\n", __func__);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(11000, 11100);
	/* vci 3.0v */
	ret=lcm_enable_reg_vio3P0(0);
	usleep_range(5000, 5100);
	/* vddr 1.2v */
	gpiod_set_value(ctx->vddr_enable_gpio, 0);
	/* vddi 1.8v */
	ret=lcm_enable_reg_vio1p8(0);
	usleep_range(5000, 5100);
	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);
	usleep_range(70000, 70100);
	demura_tap = 0;
	DISP_INFO("%s-\n", __func__);
	return 0;
}

static int lcm_get_modes(struct drm_panel *panel,
					struct drm_connector *connector)
{
    struct drm_display_mode *mode[MODE_NUM];
    int i = 0;

	if (need_fake_resolution) {
		change_drm_disp_mode_params(&display_mode[0]);
		change_drm_disp_mode_params(&display_mode[1]);
		change_drm_disp_mode_params(&display_mode[2]);
		change_drm_disp_mode_params(&display_mode[3]);
	}

    mode[0] = drm_mode_duplicate(connector->dev, &display_mode[0]);
    if (!mode[0]) {
        DISP_ERR("failed to add mode %ux%ux@%u\n",
            display_mode[0].hdisplay, display_mode[0].vdisplay,
             drm_mode_vrefresh(&display_mode[0]));
        return -ENOMEM;
    }

    drm_mode_set_name(mode[0]);
    mode[0]->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
    drm_mode_probed_add(connector, mode[0]);
    DISP_DEBUG("%s, clock=%d,htotal=%d,vtotal=%d,hskew=%d,vrefresh=%d\n",__func__, mode[0]->clock, mode[0]->htotal,
        mode[0]->vtotal, mode[0]->hskew, drm_mode_vrefresh(mode[0]));

    for (i = 1; i < MODE_NUM; i++) {
        mode[i] = drm_mode_duplicate(connector->dev, &display_mode[i]);
        DISP_DEBUG("mode %ux%ux@%u\n", display_mode[0].hdisplay, display_mode[0].vdisplay,drm_mode_vrefresh(&display_mode[0]));
        if (!mode[i]) {
            DISP_ERR("not enough memory\n");
            return -ENOMEM;
        }

        drm_mode_set_name(mode[i]);
        mode[i]->type = DRM_MODE_TYPE_DRIVER;
        drm_mode_probed_add(connector, mode[i]);
    }


	connector->display_info.width_mm = 69;
	connector->display_info.height_mm = 152;

	return 1;
}

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.panel_poweron = lcm_panel_poweron,
	.panel_reset = lcm_panel_reset,
	.panel_poweroff = lcm_panel_poweroff,
	.ata_check = panel_ata_check,
	.ext_param_set = mtk_panel_ext_param_set,
	.ext_param_get = mtk_panel_ext_param_get,
	.esd_backlight_recovery = oplus_esd_backlight_recovery,
/* #ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	.set_hbm = lcm_set_hbm,
	.oplus_ofp_set_lhbm_pressed_icon_single = oplus_ofp_set_lhbm_pressed_icon,
	.doze_disable = panel_doze_disable,
	.doze_enable = panel_doze_enable,
	.set_aod_light_mode = panel_set_aod_light_mode,
	.get_vdo_aod_param = mtk_get_vdo_aod_param,
	.get_virtual_heigh = lcm_get_virtual_heigh,
	.get_virtual_width = lcm_get_virtual_width,
/* #endif  *//* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
};
#endif

static const struct drm_panel_funcs lcm_drm_funcs = {
	.disable = lcm_disable,
	.unprepare = lcm_unprepare,
	.prepare = lcm_prepare,
	.enable = lcm_enable,
	.get_modes = lcm_get_modes,
};


static int lcm_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;
	struct lcm *ctx;
	struct device_node *backlight;
	int ret;
/* #ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	unsigned int fp_type = 0xA10;
/* #endif  *//* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	DISP_ERR("%s, ac312 lcm_probe+\n", __func__);

	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);

		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				DISP_ERR("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			DISP_ERR("device node name:%s, dev->of_node name:%s\n", remote_node->name, dev->of_node->name);
		}
	}
	if (remote_node != dev->of_node) {
		DISP_ERR("skip probe due to not current lcm\n");
		return -ENODEV;
	}

	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	g_pmic = kzalloc(sizeof(struct lcm_pmic_info), GFP_KERNEL);
	if (!g_pmic) {
		DISP_ERR("fail to alloc lcm_pmic_info (ENOMEM)\n");
		return -ENOMEM;
	}
	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_NO_EOT_PACKET | MIPI_DSI_CLOCK_NON_CONTINUOUS;

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		DISP_ERR("cannot get reset-gpios %ld\n",
			 PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	usleep_range(5000, 5100);

	g_pmic->reg_vio1p8= regulator_get(ctx->dev, "vddi_1p8");
	if (IS_ERR(g_pmic->reg_vio1p8)) {
		DISP_ERR("cannot get reg_vio1p8 %ld\n",
			PTR_ERR(g_pmic->reg_vio1p8));
	}
	ret = regulator_set_voltage(g_pmic->reg_vio1p8,
				1800000, 1800000);
	ret = lcm_enable_reg_vio1p8(1);
	usleep_range(5000, 5100);

	ctx->vddr_enable_gpio = devm_gpiod_get(dev, "vddr_enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddr_enable_gpio)) {
		DISP_ERR("cannot get vddr-enable-gpios %ld\n",
			 PTR_ERR(ctx->vddr_enable_gpio));
		return PTR_ERR(ctx->vddr_enable_gpio);
	}
	usleep_range(5000, 5100);

	g_pmic->reg_vio3P0= regulator_get(ctx->dev, "vci_3p0");
	if (IS_ERR(g_pmic->reg_vio3P0)) {
		DISP_ERR("cannot get reg_vio3P0 %ld\n",
			PTR_ERR(g_pmic->reg_vio3P0));
	}
	ret = regulator_set_voltage(g_pmic->reg_vio3P0,
				3000000, 3000000);
	ret = lcm_enable_reg_vio3P0(1);
	usleep_range(5000, 5100);

	ctx->prepared = true;
	ctx->enabled = true;
	drm_panel_init(&ctx->panel, dev, &lcm_drm_funcs, DRM_MODE_CONNECTOR_DSI);

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params[0], &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;

#endif

	register_device_proc("lcd", "AC312_P_3_A0025", "P_3");
/* #ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	oplus_ofp_set_fp_type(&fp_type);
/* #endif  *//* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	oplus_max_normal_brightness = MAX_NORMAL_BRIGHTNESS;
	check_is_need_fake_resolution(dev);
	DISP_ERR("%s, ac312 lcm_probe-\n", __func__);
	return ret;
}

static int lcm_remove(struct mipi_dsi_device *dsi)
{
	struct lcm *ctx = mipi_dsi_get_drvdata(dsi);
#if defined(CONFIG_MTK_PANEL_EXT)
	struct mtk_panel_ctx *ext_ctx = find_panel_ctx(&ctx->panel);
#endif

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_detach(ext_ctx);
	mtk_panel_remove(ext_ctx);
#endif

	return 0;
}

static const struct of_device_id lcm_of_match[] = {
	{
	    .compatible = "ac312,p,3,a0025,dsi,vdo",
	},
	{}
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "ac312_p_3_a0025_dsi_vdo",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

static int __init lcm_drv_init(void)
{
	int ret = 0;

	pr_notice("%s+\n", __func__);
	mtk_panel_lock();
	ret = mipi_dsi_driver_register(&lcm_driver);
	if (ret < 0)
		pr_notice("%s, Failed to register lcm driver: %d\n",
			__func__, ret);

	mtk_panel_unlock();
	pr_notice("%s- ret:%d\n", __func__, ret);
	return 0;
}

static void __exit lcm_drv_exit(void)
{
	pr_notice("%s+\n", __func__);
	mtk_panel_lock();
	mipi_dsi_driver_unregister(&lcm_driver);
	mtk_panel_unlock();
	pr_notice("%s-\n", __func__);
}
module_init(lcm_drv_init);
module_exit(lcm_drv_exit);

MODULE_AUTHOR("like");
MODULE_DESCRIPTION("lcm ac312 Panel Driver");
MODULE_LICENSE("GPL v2");

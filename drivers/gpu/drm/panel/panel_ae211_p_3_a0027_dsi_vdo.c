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
#include <soc/oplus/system/oplus_mm_kevent_fb.h>

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#endif

#include "../oplus/oplus_display_panel_power.h"
#include "../mediatek/mediatek_v2/mtk_log.h"
#include "../mediatek/mediatek_v2/mtk_dsi.h"

#include "panel_ae211_p_3_a0027_dsi_vdo.h"
#include "../../../../misc/mediatek/include/mt-plat/mtk_boot_common.h"
// #include "../mediatek/mediatek_v2/mtk_corner_pattern/ae045_p_1_data_hw_roundcornerpattern.h"
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

#define MAX_NORMAL_BRIGHTNESS   3760
#define LCM_BRIGHTNESS_TYPE 2
#define FINGER_HBM_BRIGHTNESS 3760
// #define OLED_VDDI_EN_HW_SUPPORT
extern unsigned int last_backlight;
extern void oplus_display_get_panel_brightness_time(void);
extern unsigned int oplus_display_brightness;
extern unsigned int oplus_max_normal_brightness;
extern int oplus_display_panel_dbv_probe(struct device *dev);
static int current_fps = 60;
extern atomic_t esd_pending;

extern void lcdinfo_notify(unsigned long val, void *v);

static unsigned int p_3_a0027_vdo_dphy_buf_thresh[14] ={896, 1792, 2688, 3584, 4480,
	5376, 6272, 6720, 7168, 7616, 7744, 7872, 8000, 8064};
static unsigned int p_3_a0027_vdo_dphy_range_min_qp[15] ={0, 4, 5, 5, 7, 7, 7, 7, 7,
	7, 9, 9, 9, 13, 16};
static unsigned int p_3_a0027_vdo_dphy_range_max_qp[15] ={8, 8, 9, 10, 11, 11, 11,
	12, 13, 14, 14, 15, 15, 16, 17};
static int p_3_a0027_vdo_dphy_range_bpg_ofs[15] ={2, 0, 0, -2, -4, -6, -8, -8, -8,
	-10, -10, -12, -12, -12, -12};

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
#ifdef OLED_VDDI_EN_HW_SUPPORT
	struct gpio_desc *vddi1p8_enable_gpio;
#endif
	struct gpio_desc *vddr1p2_enable_gpio;
	struct gpio_desc *vci3p0_enable_gpio;
	struct drm_display_mode *m;
	bool prepared;
	bool enabled;
	bool hbm_en;
	bool hbm_wait;
	int error;
};

#define lcm_dcs_write_seq(ctx, seq...) \
({\
	const u8 d[] = { seq };\
	BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 128, "DCS sequence too big for stack");\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

#define lcm_dcs_write_seq_static(ctx, seq...) \
({\
	static const u8 d[] = { seq };\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

static inline struct lcm *panel_to_lcm(struct drm_panel *panel)
{
	return container_of(panel, struct lcm, panel);
}

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

static void lcm_panel_init(struct lcm *ctx)
{
	pr_info("%s+\n", __func__);
	//power on/off timing & setting
	lcm_dcs_write_seq_static(ctx,0xF0,0x5A,0x5A);
	lcm_dcs_write_seq_static(ctx,0x90,0x40);
	lcm_dcs_write_seq_static(ctx,0x03,0x11);
	//DSC Setting
	lcm_dcs_write_seq_static(ctx,0xA3,0x12 ,0x00 ,0x00 ,0xAB ,0x30 ,0x80 ,0x09 ,0x44 ,0x04 ,0x38 ,0x00 ,0x08 ,0x02 ,0x1C ,0x02 ,0x1C, 0x02 ,0x00 ,0x02 ,0x0E ,0x00 ,0x20 ,0x00 ,0xBB ,0x00 ,0x07 ,0x00 ,0x0C ,0x0D ,0xB7 ,0x0C ,0xB7 ,0x18, 0x00 ,0x10 ,0xF0 ,0x07 ,0x10 ,0x20 ,0x00);
	lcm_dcs_write_seq_static(ctx,0xB0,0x28);
	lcm_dcs_write_seq_static(ctx,0xA3,0x06 ,0x0F ,0x0F ,0x33 ,0x0E ,0x1C ,0x2A ,0x38 ,0x46 ,0x54 ,0x62 ,0x69 ,0x70 ,0x77 ,0x79 ,0x7B, 0x7D ,0x7E ,0x02 ,0x02 ,0x22 ,0x00 ,0x2A ,0x40 ,0x2A ,0xBE ,0x3A ,0xFC ,0x3A ,0xFA ,0x3A ,0xF8 ,0x3B, 0x38 ,0x3B ,0x78 ,0x3B ,0xB6 ,0x4B ,0xB6);
	lcm_dcs_write_seq_static(ctx,0xB0,0x50);
	lcm_dcs_write_seq_static(ctx,0xA3,0x4B ,0xF4 ,0x4B ,0xF4 ,0x6C ,0x34 ,0x84 ,0x74 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00, 0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00);

	lcm_dcs_write_seq_static(ctx,0x2A,0x00,0x00,0x04,0x37);
	lcm_dcs_write_seq_static(ctx,0x2B,0x00,0x00,0x09,0x43);
	lcm_dcs_write_seq_static(ctx,0x43,0x02,0x01);
	lcm_dcs_write_seq_static(ctx,0x44,0x09,0x44);
	lcm_dcs_write_seq_static(ctx,0x35,0x00);
	lcm_dcs_write_seq_static(ctx,0x53,0x20);
	lcm_dcs_write_seq_static(ctx,0x51,0x00,0x00);
	if (current_fps == 60) {
		lcm_dcs_write_seq_static(ctx, 0x86,0x12);
	} else if (current_fps == 90) {
		lcm_dcs_write_seq_static(ctx, 0x86,0x11);
	} else if (current_fps == 120) {
		lcm_dcs_write_seq_static(ctx, 0x86,0x10);
	}
	lcm_dcs_write_seq_static(ctx,0x11,0x00);
	usleep_range(120*1000, 121*1000);
	lcm_dcs_write_seq_static(ctx,0xF0,0x5A,0x5A);
	lcm_dcs_write_seq_static(ctx,0xB0,0x0A);
	lcm_dcs_write_seq_static(ctx,0xC0,0x20,0xBF);
	lcm_dcs_write_seq_static(ctx,0xB0,0x0C);
	lcm_dcs_write_seq_static(ctx,0xC0,0x00,0xBF);
	lcm_dcs_write_seq_static(ctx,0xB0,0x0E);
	lcm_dcs_write_seq_static(ctx,0xC0,0x00,0xBF);
	lcm_dcs_write_seq_static(ctx,0xB0,0x10);
	lcm_dcs_write_seq_static(ctx,0xC0,0x00,0xBF);
	lcm_dcs_write_seq_static(ctx,0xB0,0x1E);
	lcm_dcs_write_seq_static(ctx,0xC0,0x00,0x42);
	lcm_dcs_write_seq_static(ctx,0xB3,0x12 ,0x00 ,0x00 ,0x11 ,0x00 ,0x9F ,0x00 ,0x31 ,0x08 ,0xC6 ,0x03 ,0x1A ,0x00 ,0x5A ,0x00 ,0x9F, 0x00 ,0x04 ,0x02 ,0x00 ,0x7D ,0x0F ,0x12 ,0x14 ,0x00 ,0x00 ,0x02 ,0xAC);
	lcm_dcs_write_seq_static(ctx,0xB0,0x00);
	lcm_dcs_write_seq_static(ctx,0xC9,0x9F);
	lcm_dcs_write_seq_static(ctx,0xB0,0xF5);
	lcm_dcs_write_seq_static(ctx,0xC9,0x20);
	lcm_dcs_write_seq_static(ctx,0xBF,0x29,0x00,0x43,0xFF,0x03,0x22,0x02);
	lcm_dcs_write_seq_static(ctx,0xB0,0x0C);
	lcm_dcs_write_seq_static(ctx,0xBF,0x20);
	lcm_dcs_write_seq_static(ctx,0xB0,0x14);
	lcm_dcs_write_seq_static(ctx,0xBF,0x01);
	lcm_dcs_write_seq_static(ctx,0xB0,0x08);
	lcm_dcs_write_seq_static(ctx,0xB2,0x70);
	lcm_dcs_write_seq_static(ctx,0xB0,0x34);
	lcm_dcs_write_seq_static(ctx,0xB2,0x00,0x00);
	lcm_dcs_write_seq_static(ctx,0xB0,0x5A);
	lcm_dcs_write_seq_static(ctx,0xC5,0x03,0xB0);
	lcm_dcs_write_seq_static(ctx,0xF7,0xFC,0xA2,0x02,0x77,0x01,0x7C,0x37,0x07);
	lcm_dcs_write_seq_static(ctx,0xB0,0x0E);
	lcm_dcs_write_seq_static(ctx,0xED,0xA0);
	//APL code
	lcm_dcs_write_seq_static(ctx,0xB0, 0x5C );
	lcm_dcs_write_seq_static(ctx,0xC1,0x01,0x00,0xEF);
	lcm_dcs_write_seq_static(ctx,0xB0, 0x88);
	lcm_dcs_write_seq_static(ctx,0xE8,0x01,0x10);
	lcm_dcs_write_seq_static(ctx,0xBD,0x80);
	lcm_dcs_write_seq_static(ctx,0xC7, 0x01);
	lcm_dcs_write_seq_static(ctx,0xB0, 0x02);
	lcm_dcs_write_seq_static(ctx,0xC7, 0x44);
	lcm_dcs_write_seq_static(ctx,0xB0, 0x03);
	lcm_dcs_write_seq_static(ctx,0xC7, 0x00,0x00,0x07,0xFF);
	lcm_dcs_write_seq_static(ctx,0xB0, 0x07);
	lcm_dcs_write_seq_static(ctx,0xC7,0x0D,0x14,0x0B,0xE2,0x0F,0xC3,0x0C,0xD7,0x0B,0xAE,0x0F,0x4F,0x0C,0x97,0x0B,0x7B,0x0E,0xE9);
	lcm_dcs_write_seq_static(ctx,0xB0, 0x19);
	lcm_dcs_write_seq_static(ctx,0xC7,0x0B,0xB0,0x0A,0xC3,0x0D,0xB6,0x09,0xFD,0x09,0x4D,0x0B,0x9E,0x09,0x8E,0x08,0xEE,0x0B,0x1E);
	lcm_dcs_write_seq_static(ctx,0xB0, 0x2B);
	lcm_dcs_write_seq_static(ctx,0xC7,0x09,0x16,0x08,0x84,0x0A,0x90,0x08,0xA0,0x08,0x19,0x0A,0x05,0x08,0x1F,0x07,0xA4,0x09,0x70);
	lcm_dcs_write_seq_static(ctx,0xB0, 0x3D);
	lcm_dcs_write_seq_static(ctx,0xC7,0x07,0x91,0x07,0x23,0x08,0xCE,0x07,0x42,0x06,0xDA,0x08,0x70,0x06,0xED,0x06,0x8E,0x08,0x11);
	lcm_dcs_write_seq_static(ctx,0xB0, 0x4F);
	lcm_dcs_write_seq_static(ctx,0xC7,0x06,0x92,0x06,0x38,0x07,0xA6,0x06,0x27,0x05,0xD4,0x07,0x2B,0x05,0xA4,0x05,0x5C,0x06,0x9C);
	lcm_dcs_write_seq_static(ctx,0xB0, 0x61);
	lcm_dcs_write_seq_static(ctx,0xC7,0x05,0x5B,0x05,0x17,0x06,0x4A,0x05,0x04,0x04,0xC5,0x05,0xE3,0x04,0x9D,0x04,0x64,0x05,0x6B);
	lcm_dcs_write_seq_static(ctx,0xB0, 0x73);
	lcm_dcs_write_seq_static(ctx,0xC7,0x04,0x5B,0x04,0x26,0x05,0x1F,0x04,0x14,0x03,0xEC,0x04,0xD1,0x03,0xC2,0x03,0xA4,0x04,0x72);
	lcm_dcs_write_seq_static(ctx,0xB0, 0x85);
	lcm_dcs_write_seq_static(ctx,0xC7,0x03,0x5E,0x03,0x4B,0x04,0x02,0x03,0x08,0x02,0xF7,0x03,0x9B,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
	lcm_dcs_write_seq_static(ctx,0xB0, 0x01);
	lcm_dcs_write_seq_static(ctx,0xC7, 0x5A);
	usleep_range(5000, 5100);
	lcm_dcs_write_seq_static(ctx,0xCE, 0x5A);
	lcm_dcs_write_seq_static(ctx, 0x82, 0x80,0x09);
	usleep_range(10000, 10100);
	lcm_dcs_write_seq_static(ctx,0xC7, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x82, 0x00,0x00);
	lcm_dcs_write_seq_static(ctx,0xF0,0xA5,0xA5);
	//display out
	lcm_dcs_write_seq_static(ctx, 0x29, 0x00);

	pr_info("%s-\n", __func__);
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

	pr_info("%s:prepared=%d\n", __func__, ctx->prepared);

	if (!ctx->prepared)
		return 0;

	lcm_dcs_write_seq_static(ctx, 0x28);
	usleep_range(10000, 11000);
	lcm_dcs_write_seq_static(ctx, 0x10);
	usleep_range(150*1000, 151*1000);

	ctx->error = 0;
	ctx->prepared = false;
	//ctx->hbm_en = false;
	pr_info("%s:success\n", __func__);

	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	pr_info("%s:prepared=%d\n", __func__, ctx->prepared);
	if (ctx->prepared)
		return 0;

	lcm_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;
	pr_info("%s:success\n", __func__);
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

static u32 fake_heigh = 2372;
static u32 fake_width = 1080;
static bool need_fake_resolution;

#define FRAME_WIDTH             (1080)
#define FRAME_HEIGHT            (2376)

#define HFP_30HZ                (2030)
#define HFP                     (146)
#define HBP                     (32)
#define HSA                     (4)
#define VFP_30HZ                (44)
#define VFP_60HZ                (2492)
#define VFP_90HZ                (860)
#define VFP_120HZ               (44)
#define VBP                     (24)
#define VSA                     (4)
#define DYN_PLL_CLK			    (554)
#define DYN_DATA_RATE           (1108)
#define HFP_DYN                 (141)
#define HFP_DYN_30HZ            (2030)
#define DATA_RATE				(1107)

static struct drm_display_mode disp_mode_30Hz = {
	.clock = ((FRAME_WIDTH + HFP_30HZ + HBP + HSA) * (FRAME_HEIGHT + VFP_30HZ + VBP + VSA) * 30) / 1000,
	.hdisplay = FRAME_WIDTH,
	.hsync_start = FRAME_WIDTH + HFP_30HZ,
	.hsync_end = FRAME_WIDTH + HFP_30HZ + HSA,
	.htotal = FRAME_WIDTH + HFP_30HZ + HSA + HBP,
	.vdisplay = FRAME_HEIGHT,
	.vsync_start = FRAME_HEIGHT + VFP_30HZ,
	.vsync_end = FRAME_HEIGHT + VFP_30HZ + VSA,
	.vtotal = FRAME_HEIGHT + VFP_30HZ + VSA + VBP,
};

static struct drm_display_mode disp_mode_60Hz = {
	.clock = ((FRAME_WIDTH + HFP + HBP + HSA) * (FRAME_HEIGHT + VFP_60HZ + VBP + VSA) * 60) / 1000,
	.hdisplay = FRAME_WIDTH,
	.hsync_start = FRAME_WIDTH + HFP,
	.hsync_end = FRAME_WIDTH + HFP + HSA,
	.htotal = FRAME_WIDTH + HFP + HSA + HBP,
	.vdisplay = FRAME_HEIGHT,
	.vsync_start = FRAME_HEIGHT + VFP_60HZ,
	.vsync_end = FRAME_HEIGHT + VFP_60HZ + VSA,
	.vtotal = FRAME_HEIGHT + VFP_60HZ + VSA + VBP,
};

static struct drm_display_mode disp_mode_90Hz = {
	.clock = ((FRAME_WIDTH + HFP + HBP + HSA) * (FRAME_HEIGHT + VFP_90HZ + VBP + VSA) * 90) / 1000,
	.hdisplay = FRAME_WIDTH,
	.hsync_start = FRAME_WIDTH + HFP,
	.hsync_end = FRAME_WIDTH + HFP + HSA,
	.htotal = FRAME_WIDTH + HFP + HSA + HBP,
	.vdisplay = FRAME_HEIGHT,
	.vsync_start = FRAME_HEIGHT + VFP_90HZ,
	.vsync_end = FRAME_HEIGHT + VFP_90HZ + VSA,
	.vtotal = FRAME_HEIGHT + VFP_90HZ + VSA + VBP,
};

static struct drm_display_mode disp_mode_120Hz = {
	.clock = ((FRAME_WIDTH + HFP + HBP + HSA) * (FRAME_HEIGHT + VFP_120HZ + VBP + VSA) * 120) / 1000,
	.hdisplay = FRAME_WIDTH,
	.hsync_start = FRAME_WIDTH + HFP,
	.hsync_end = FRAME_WIDTH + HFP + HSA,
	.htotal = FRAME_WIDTH + HFP + HSA + HBP,
	.vdisplay = FRAME_HEIGHT,
	.vsync_start = FRAME_HEIGHT + VFP_120HZ,
	.vsync_end = FRAME_HEIGHT + VFP_120HZ + VSA,
	.vtotal = FRAME_HEIGHT + VFP_120HZ + VSA + VBP,
};

static struct mtk_panel_params ext_params_30Hz = {
	.pll_clk = DATA_RATE/2,
	.data_rate = DATA_RATE,
	.change_fps_by_vfp_send_cmd = 1,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 30,
	},
	/* following MIPI hopping parameter might cause screen mess */
	.dyn = {
			.switch_en = 1,
			.pll_clk = DYN_PLL_CLK,
			.data_rate = DYN_DATA_RATE,
			.vsa = VSA,
			.vbp = VBP,
			.vfp = VFP_30HZ,
			.hsa = HSA,
			.hbp = HBP,
			.hfp = HFP_DYN_30HZ,
	},

	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.esd_check_multi = 0,
	.lcm_esd_check_table[0] = {
                .cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
        },
	//.round_corner_en = 1,
	//.corner_pattern_height = ROUND_CORNER_H_TOP,
	//.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	//.corner_pattern_tp_size = sizeof(top_rc_pattern),
	//.corner_pattern_lt_addr = (void *)top_rc_pattern,
	.oplus_display_global_dre = 1,
	.vendor = "A0027",
	.manufacture = "P_3_A0027",
	.oplus_display_color_mode_suppor = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	//.oplus_ofp_need_to_sync_data_in_aod_unlocking = false,
	//.oplus_ofp_aod_off_insert_black = 0,
	//.oplus_ofp_aod_off_black_frame_total_time = 42,
	.dsc_params = {
		.enable = 1,
		.ver = 18,
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
			.rc_buf_thresh = p_3_a0027_vdo_dphy_buf_thresh,
			.range_min_qp = p_3_a0027_vdo_dphy_range_min_qp,
			.range_max_qp = p_3_a0027_vdo_dphy_range_max_qp,
			.range_bpg_ofs = p_3_a0027_vdo_dphy_range_bpg_ofs,
		},
	},
};

static struct mtk_panel_params ext_params_60Hz = {
	.pll_clk = DATA_RATE/2,
	.data_rate = DATA_RATE,
	.change_fps_by_vfp_send_cmd = 1,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 60,
		.dfps_cmd_table[0] = {0, 2 , {0x86, 0x12}},
	},
	/* following MIPI hopping parameter might cause screen mess */
	.dyn = {
			.switch_en = 1,
			.pll_clk = DYN_PLL_CLK,
			.data_rate = DYN_DATA_RATE,
			.vsa = VSA,
			.vbp = VBP,
			.vfp = VFP_60HZ,
			.hsa = HSA,
			.hbp = HBP,
			.hfp = HFP_DYN,
	},

	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.esd_check_multi = 0,
	.lcm_esd_check_table[0] = {
                .cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
        },
	//.round_corner_en = 1,
	//.corner_pattern_height = ROUND_CORNER_H_TOP,
	//.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	//.corner_pattern_tp_size = sizeof(top_rc_pattern),
	//.corner_pattern_lt_addr = (void *)top_rc_pattern,
	.oplus_display_global_dre = 1,
	.vendor = "A0027",
	.manufacture = "P_3_A0027",
	.oplus_display_color_mode_suppor = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	//.oplus_ofp_need_to_sync_data_in_aod_unlocking = false,
	//.oplus_ofp_aod_off_insert_black = 0,
	//.oplus_ofp_aod_off_black_frame_total_time = 42,
	.dsc_params = {
		.enable = 1,
		.ver = 18,
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
			.rc_buf_thresh = p_3_a0027_vdo_dphy_buf_thresh,
			.range_min_qp = p_3_a0027_vdo_dphy_range_min_qp,
			.range_max_qp = p_3_a0027_vdo_dphy_range_max_qp,
			.range_bpg_ofs = p_3_a0027_vdo_dphy_range_bpg_ofs,
		},
	},
};

static struct mtk_panel_params ext_params_90Hz = {
	.pll_clk = DATA_RATE/2,
	.data_rate = DATA_RATE,
	.change_fps_by_vfp_send_cmd = 1,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 90,
		.dfps_cmd_table[0] = {0, 2 , {0x86, 0x11}},
	},

	/* following MIPI hopping parameter might cause screen mess */
	.dyn = {
			.switch_en = 1,
			.pll_clk = DYN_PLL_CLK,
			.data_rate = DYN_DATA_RATE,
			.vsa = VSA,
			.vbp = VBP,
			.vfp = VFP_90HZ,
			.hsa = HSA,
			.hbp = HBP,
			.hfp = HFP_DYN,
	},

	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,

	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.esd_check_multi = 0,
	.lcm_esd_check_table[0] = {
                .cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
        },
	//.round_corner_en = 1,
	//.corner_pattern_height = ROUND_CORNER_H_TOP,
	//.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	//.corner_pattern_tp_size = sizeof(top_rc_pattern),
	//.corner_pattern_lt_addr = (void *)top_rc_pattern,
	.oplus_display_global_dre = 1,
	.vendor = "A0027",
	.manufacture = "P_3_A0027",
	.oplus_display_color_mode_suppor = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	//.oplus_ofp_need_to_sync_data_in_aod_unlocking = false,
	//.oplus_ofp_aod_off_insert_black = 0,
	//.oplus_ofp_aod_off_black_frame_total_time = 42,
	.dsc_params = {
		.enable = 1,
		.ver = 18,
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
			.rc_buf_thresh = p_3_a0027_vdo_dphy_buf_thresh,
			.range_min_qp = p_3_a0027_vdo_dphy_range_min_qp,
			.range_max_qp = p_3_a0027_vdo_dphy_range_max_qp,
			.range_bpg_ofs = p_3_a0027_vdo_dphy_range_bpg_ofs,
		},
	},
};

static struct mtk_panel_params ext_params_120Hz = {
	.pll_clk = DATA_RATE/2,
	.data_rate = DATA_RATE,
	.change_fps_by_vfp_send_cmd = 1,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 120,
		.dfps_cmd_table[0] = {0, 2 , {0x86, 0x10}},
	},

	/* following MIPI hopping parameter might cause screen mess */
	.dyn = {
			.switch_en = 1,
			.pll_clk = DYN_PLL_CLK,
			.data_rate = DYN_DATA_RATE,
			.vsa = VSA,
			.vbp = VBP,
			.vfp = VFP_120HZ,
			.hsa = HSA,
			.hbp = HBP,
			.hfp = HFP_DYN,
	},
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.esd_check_multi = 0,
	.lcm_esd_check_table[0] = {
                .cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
        },
	//.round_corner_en = 1,
	//.corner_pattern_height = ROUND_CORNER_H_TOP,
	//.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	//.corner_pattern_tp_size = sizeof(top_rc_pattern),
	//.corner_pattern_lt_addr = (void *)top_rc_pattern,
	.oplus_display_global_dre = 1,
	.vendor = "A0027",
	.manufacture = "P_3_A0027",
	.oplus_display_color_mode_suppor = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	//.oplus_ofp_need_to_sync_data_in_aod_unlocking = false,
	//.oplus_ofp_aod_off_insert_black = 0,
	//.oplus_ofp_aod_off_black_frame_total_time = 42,
	.dsc_params = {
		.enable = 1,
		.ver = 18,
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
			.rc_buf_thresh = p_3_a0027_vdo_dphy_buf_thresh,
			.range_min_qp = p_3_a0027_vdo_dphy_range_min_qp,
			.range_max_qp = p_3_a0027_vdo_dphy_range_max_qp,
			.range_bpg_ofs = p_3_a0027_vdo_dphy_range_bpg_ofs,
		},
	},
};

static int panel_ata_check(struct drm_panel *panel)
{
	/* Customer test by own ATA tool */
	return 1;
}

static int lcm_get_virtual_heigh(void)
{
	return FRAME_HEIGHT;
}

static int lcm_get_virtual_width(void)
{
	return FRAME_WIDTH;
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
	if (fake_heigh > 0 && fake_heigh < FRAME_HEIGHT)
		need_fake_resolution = true;
	if (fake_width > 0 && fake_width < FRAME_WIDTH)
		need_fake_resolution = true;

	DISP_INFO("need_fake_resolution:%d,%s-\n", need_fake_resolution,__func__);
}

static void change_drm_disp_mode_params(struct drm_display_mode *mode)
{
	int vtotal = mode->vtotal;
	int htotal = mode->htotal;
	int fps = mode->clock * 1000 / vtotal / htotal;

	if (fake_heigh > 0 && fake_heigh < FRAME_HEIGHT) {
		mode->vsync_start = mode->vsync_start - mode->vdisplay
					+ fake_heigh;
		mode->vsync_end = mode->vsync_end - mode->vdisplay + fake_heigh;
		mode->vtotal = mode->vtotal - mode->vdisplay + fake_heigh;
		mode->vdisplay = fake_heigh;
	}
	if (fake_width > 0 && fake_width < FRAME_WIDTH) {
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
	else if (fps > 40)
		fps = 60;
	else
		fps = 30;
	mode->clock = fps * mode->vtotal * mode->htotal / 1000;
	mode->clock += 1;

	DISP_INFO("mode->vdisplay:%d,%s-\n",mode->vdisplay, __func__);
}

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle, unsigned int level)
{
	unsigned int mapped_level = 0;
	int i = 0;
	char bl_tb0[] = {0x51, 0x00, 0x00};

	if (!dsi || !cb) {
		return -EINVAL;
	}

	if (level == 1) {
		DISP_INFO("[INFO][%s:%d]filter backlight %d setting\n", __func__, __LINE__, level);
		return 0;
	}

	if ((last_backlight == 0 || last_backlight == 1) && (level != 0 && level != 1)) {
		oplus_display_get_panel_brightness_time();
	}

	if ((get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT) && (level > 1))
		level = 1023;

	mapped_level = level;

	if (mapped_level > 1 && mapped_level <= BRIGHTNESS_MAX) {
		oplus_display_brightness = mapped_level;
		lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &mapped_level);
	}

	bl_tb0[1] = mapped_level >> 8;
	bl_tb0[2] = mapped_level & 0xFF;
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	if ( mapped_level >= 1147) {
                for (i = 0; i < sizeof(lcm_set_demura_offset2)/sizeof(struct LCM_setting_table); i++){
	                cb(dsi, handle, lcm_set_demura_offset2[i].para_list, lcm_set_demura_offset2[i].count);
		}
	} else {
                for (i = 0; i < sizeof(lcm_set_demura_offset1)/sizeof(struct LCM_setting_table); i++){
                        cb(dsi, handle, lcm_set_demura_offset1[i].para_list, lcm_set_demura_offset1[i].count);
                }
	}
	last_backlight = level;
	pr_info("%s,level = %d,", __func__, level);

	return 0;
}

static int oplus_esd_backlight_recovery(void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int level = oplus_display_brightness;
	unsigned char esd_bl_level[] = {0x51, 0x03, 0xFF};

	if (!dsi || !cb) {
		return -EINVAL;
	}

	esd_bl_level[1] = level >> 8;
	esd_bl_level[2] = level & 0xFF;
	cb(dsi, handle, esd_bl_level, ARRAY_SIZE(esd_bl_level));
	pr_info("esd_bl_level[1]=%x, esd_bl_level[2]=%x\n", esd_bl_level[1], esd_bl_level[2]);

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
	struct lcm *ctx = NULL;
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

        pr_info("%s,oplus_display_brightness=%d, hbm_mode=%u\n", __func__, oplus_display_brightness, en);
        if (en == 1) {
                if ( oplus_display_brightness > 1147 ){
                for (i = 0; i < sizeof(lcm_finger_HBM_on_above_70nit)/sizeof(struct LCM_setting_table); i++){
                        cb(dsi, handle, lcm_finger_HBM_on_above_70nit[i].para_list, lcm_finger_HBM_on_above_70nit[i].count);
                }
                        } else {
                        for (i = 0; i < sizeof(lcm_finger_HBM_on_below_70nit)/sizeof(struct LCM_setting_table); i++){
                                cb(dsi, handle, lcm_finger_HBM_on_below_70nit[i].para_list, lcm_finger_HBM_on_below_70nit[i].count);
                                }
                        }
				usleep_range(17000, 17100);
        } else if (en == 0) {
                if  ( oplus_display_brightness > 1147 ){
                        for (i = 0; i < sizeof(lcm_finger_HBM_off_above_70nit)/sizeof(struct LCM_setting_table); i++){
                                cb(dsi, handle, lcm_finger_HBM_off_above_70nit[i].para_list, lcm_finger_HBM_off_above_70nit[i].count);
                                }
                        } else {
                        for (i = 0; i < sizeof(lcm_finger_HBM_off_below_70nit)/sizeof(struct LCM_setting_table); i++){
                                cb(dsi, handle, lcm_finger_HBM_off_below_70nit[i].para_list, lcm_finger_HBM_off_below_70nit[i].count);
                }
                        }
                 lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);
        }
	return 0;
}

static int oplus_display_panel_set_hbm_max(void *dsi, dcs_write_gce cb, void *handle, unsigned int en)
{
    unsigned int i = 0;

    pr_info(" %s en=%d\n", __func__, en);

    if (!dsi || !cb) {
        pr_info("Invalid params\n");
        return -EINVAL;
    }

    if (en) {
        for (i = 0; i < sizeof(dsi_switch_hbm_apl_on) / sizeof(struct LCM_setting_table); i++) {
            cb(dsi, handle, dsi_switch_hbm_apl_on[i].para_list, dsi_switch_hbm_apl_on[i].count);
        }
        pr_info("%s Enter hbm_max mode",__func__);
    } else if (!en) {
        dsi_switch_hbm_apl_off[0].para_list[1] = oplus_display_brightness >> 8;
        dsi_switch_hbm_apl_off[0].para_list[2] = oplus_display_brightness & 0xFF;
        for (i = 0; i < sizeof(dsi_switch_hbm_apl_off) / sizeof(struct LCM_setting_table); i++) {
            cb(dsi, handle, dsi_switch_hbm_apl_off[i].para_list, dsi_switch_hbm_apl_off[i].count);
        }
        pr_info(" %s hbm_max off, restore bl:%d\n", __func__, oplus_display_brightness);
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

	for (i = 0; i < (sizeof(AOD_off_setting) / sizeof(struct LCM_setting_table)); i++) {

		cmd = AOD_off_setting[i].cmd;
		switch (cmd) {
			case REGFLAG_DELAY:
				if (handle == NULL) {
					usleep_range(AOD_off_setting[i].count * 1000, AOD_off_setting[i].count * 1000 + 100);
				} else {
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(AOD_off_setting[i].count * 1000), CMDQ_GPR_R14);
				}
				break;
			case REGFLAG_UDELAY:
				if (handle == NULL) {
					usleep_range(AOD_off_setting[i].count, AOD_off_setting[i].count + 100);
				} else {
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(AOD_off_setting[i].count), CMDQ_GPR_R14);
				}
				break;
			case REGFLAG_END_OF_TABLE:
				break;
			default:
				cb(dsi, handle, AOD_off_setting[i].para_list, AOD_off_setting[i].count);
		}
	}

	pr_info("%s:success\n", __func__);
	return 0;
}


static int panel_doze_enable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int i = 0;
	unsigned int cmd;

	if (!panel || !dsi) {
		pr_err("Invalid dsi params\n");
	}

	for (i = 0; i < (sizeof(AOD_on_setting)/sizeof(struct LCM_setting_table)); i++) {
		cmd = AOD_on_setting[i].cmd;
		switch (cmd) {
			case REGFLAG_DELAY:
				usleep_range(AOD_on_setting[i].count * 1000, AOD_on_setting[i].count * 1000 + 100);
				break;
			case REGFLAG_UDELAY:
				usleep_range(AOD_on_setting[i].count, AOD_on_setting[i].count + 100);
				break;
			case REGFLAG_END_OF_TABLE:
				break;
			default:
				{
					cb(dsi, handle, AOD_on_setting[i].para_list, AOD_on_setting[i].count);
				}
		}
	}

	pr_info("%s:success\n", __func__);
	return 0;
}

static int panel_set_aod_light_mode(void *dsi, dcs_write_gce cb, void *handle, unsigned int level)
{
	int i = 0;

	if (level == 0) {
		for (i = 0; i < sizeof(aod_high_bl_level)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, aod_high_bl_level[i].para_list, aod_high_bl_level[i].count);
		}
	} else {
		for (i = 0; i < sizeof(aod_low_bl_level)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, aod_low_bl_level[i].para_list, aod_low_bl_level[i].count);
		}
	}
	pr_info("%s:success %d !\n", __func__, level);

	return 0;
}

static struct vdo_aod_params vdo_aod_on = {
	.porch_change_flag = 0x03,
	.dst_hfp = 2030,
	.dst_vfp = 44, //30fps
	.mode_idx = 3,
	.change_mmclk = true,
	.vdo_aod_cmd_table[0]={3, {0xF0,0x5A,0x5A} },
	.vdo_aod_cmd_table[1]={2, {0x90,0x41} },
	.vdo_aod_cmd_table[2]={2, {0xB0, 0x02}},
	.vdo_aod_cmd_table[3]={3, {0x51,0x0F, 0xFE}},
	.vdo_aod_cmd_table[4]={3, {0xF0,0xA5,0xA5} },
};


static struct vdo_aod_params vdo_aod_to_120hz = {
	.porch_change_flag = 0x03,
	.dst_hfp = 146,
	.dst_vfp = 44,
	.mode_idx = 0,
	.change_mmclk = true,
	.vdo_aod_cmd_table[0]={2, {0xB0,0x0B} },
	.vdo_aod_cmd_table[1]={2, {0xB2,0x48} },
	.vdo_aod_cmd_table[2]={2, {0x90,0x40} },
};

static struct vdo_aod_params vdo_aod_to_120hz_unlocking = {
	.porch_change_flag = 0x03,
	.dst_hfp = 146,
	.dst_vfp = 44,
	.mode_idx = 0,
	.change_mmclk = true,
	.vdo_aod_cmd_table[0]={2, {0x90,0x40} },
	.vdo_aod_cmd_table[1]={3, {0x51, 0x00, 0x00}},

};

static struct vdo_aod_params vdo_aod_to_90hz = {
	.porch_change_flag = 0x03,
	.dst_hfp = 146,
	.dst_vfp = 860,
	.mode_idx = 1,
	.change_mmclk = true,
	.vdo_aod_cmd_table[0]={2, {0xB0,0x0B} },
	.vdo_aod_cmd_table[1]={2, {0xB2,0x48} },
	.vdo_aod_cmd_table[2]={2, {0x90,0x40} },

};

static struct vdo_aod_params vdo_aod_to_90hz_unlocking = {
	.porch_change_flag = 0x03,
	.dst_hfp = 146,
	.dst_vfp = 860,
	.mode_idx = 1,
	.change_mmclk = true,
	.vdo_aod_cmd_table[0]={2, {0x90,0x40} },
	.vdo_aod_cmd_table[1]={3, {0x51, 0x00, 0x00}},
};

static struct vdo_aod_params vdo_aod_to_60hz = {
	.porch_change_flag = 0x03,
	.dst_hfp = 146,
	.dst_vfp = 2492,
	.mode_idx = 2,
	.change_mmclk = true,
	.vdo_aod_cmd_table[0]={2, {0xB0,0x0B} },
	.vdo_aod_cmd_table[1]={2, {0xB2,0x48} },
	.vdo_aod_cmd_table[2]={2, {0x90,0x40} },
};

static struct vdo_aod_params vdo_aod_to_60hz_unlocking = {
	.porch_change_flag = 0x03,
	.dst_hfp = 146,
	.dst_vfp = 2492,
	.mode_idx = 2,
	.change_mmclk = true,
	.vdo_aod_cmd_table[0]={2, {0x90,0x40} },
	.vdo_aod_cmd_table[1]={3, {0x51, 0x00, 0x00}},
};

static int mtk_get_vdo_aod_param(int aod_en, struct vdo_aod_params **vdo_aod_param)
{

	if(aod_en) {
		atomic_set(&esd_pending, 1);
		*vdo_aod_param = &vdo_aod_on;
	} else {
		if(current_fps == 60) {
			if(oplus_ofp_get_aod_unlocking())
				*vdo_aod_param = &vdo_aod_to_60hz_unlocking;
			else {
				*vdo_aod_param = &vdo_aod_to_60hz;
			}
		} else if (current_fps == 90) {
			if(oplus_ofp_get_aod_unlocking())
				*vdo_aod_param = &vdo_aod_to_90hz_unlocking;
			else {
			*vdo_aod_param = &vdo_aod_to_90hz;
			}
		} else {
			if(oplus_ofp_get_aod_unlocking())
				*vdo_aod_param = &vdo_aod_to_120hz_unlocking;
			else {
			*vdo_aod_param = &vdo_aod_to_120hz;
			}
		}
		atomic_set(&esd_pending, 0);
	}
	OFP_INFO("%s:aod_en %d, current_fps %d  %d\n", __func__, aod_en, current_fps, oplus_ofp_get_aod_unlocking());
	return 0;
}

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_info(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}

static int lcm_panel_poweron(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	pr_info("%s: AE211 lcm ctx->prepared %d\n", __func__, ctx->prepared);

#ifdef OLED_VDDI_EN_HW_SUPPORT
	//enable vddi 1.8v
	ctx->vddi1p8_enable_gpio =
		devm_gpiod_get(ctx->dev, "vddi-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddi1p8_enable_gpio)) {
		dev_err(ctx->dev, "%s: cannot get vddi1p8_enable_gpio %ld\n",
			__func__, PTR_ERR(ctx->vddi1p8_enable_gpio));
		return PTR_ERR(ctx->vddi1p8_enable_gpio);
	}
	gpiod_set_value(ctx->vddi1p8_enable_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->vddi1p8_enable_gpio);
	usleep_range(1000, 1100);
#endif

	ctx->vci3p0_enable_gpio =
		devm_gpiod_get(ctx->dev, "vci-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vci3p0_enable_gpio)) {
		dev_err(ctx->dev, "%s: cannot get vci3p0_enable_gpio %ld\n",
			__func__, PTR_ERR(ctx->vci3p0_enable_gpio));
		return PTR_ERR(ctx->vci3p0_enable_gpio);
	}
	gpiod_set_value(ctx->vci3p0_enable_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->vci3p0_enable_gpio);
	usleep_range(3000, 3100);

	//enable vddr
	ctx->vddr1p2_enable_gpio =
		devm_gpiod_get(ctx->dev, "vddr-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddr1p2_enable_gpio)) {
		dev_err(ctx->dev, "%s: cannot get vddr1p2_enable_gpio %ld\n",
			__func__, PTR_ERR(ctx->vddr1p2_enable_gpio));
		return PTR_ERR(ctx->vddr1p2_enable_gpio);
	}
	gpiod_set_value(ctx->vddr1p2_enable_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->vddr1p2_enable_gpio);
	usleep_range(1000, 1100);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

    pr_info("%s:Successful\n", __func__);
	return 0;
}

static int lcm_panel_poweroff(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	pr_info("%s: AE211 lcm ctx->prepared %d\n", __func__, ctx->prepared);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	usleep_range(5000, 5100);
	ctx->vddr1p2_enable_gpio =
		devm_gpiod_get(ctx->dev, "vddr-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddr1p2_enable_gpio)) {
		dev_err(ctx->dev, "%s: cannot get vddr1p2_enable_gpio %ld\n",
			__func__, PTR_ERR(ctx->vddr1p2_enable_gpio));
		return PTR_ERR(ctx->vddr1p2_enable_gpio);
	}
	gpiod_set_value(ctx->vddr1p2_enable_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->vddr1p2_enable_gpio);
	usleep_range(5000, 5100);
	//disable 3.0V
	ctx->vci3p0_enable_gpio =
		devm_gpiod_get(ctx->dev, "vci-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vci3p0_enable_gpio)) {
		dev_err(ctx->dev, "%s: cannot get vci3p0_enable_gpio %ld\n",
			__func__, PTR_ERR(ctx->vci3p0_enable_gpio));
		return PTR_ERR(ctx->vci3p0_enable_gpio);
	}
	gpiod_set_value(ctx->vci3p0_enable_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->vci3p0_enable_gpio);
	usleep_range(5000, 5100);
#ifdef OLED_VDDI_EN_HW_SUPPORT
	//enable vddi 1.8v
	ctx->vddi1p8_enable_gpio =
		devm_gpiod_get(ctx->dev, "vddi-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddi1p8_enable_gpio)) {
		dev_err(ctx->dev, "%s: cannot get vddi1p8_enable_gpio %ld\n",
			__func__, PTR_ERR(ctx->vddi1p8_enable_gpio));
		return PTR_ERR(ctx->vddi1p8_enable_gpio);
	}
	gpiod_set_value(ctx->vddi1p8_enable_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->vddi1p8_enable_gpio);
#endif

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);
	pr_info("%s:Successful\n", __func__);

	return 0;
}

static int lcm_panel_reset(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (ctx->prepared)
		return 0;

	pr_info("[LCM]debug for lcd reset :%s, ctx->prepared:%d\n", __func__, ctx->prepared);

	if(IS_ERR(ctx->reset_gpio)){
		pr_err("cannot get reset-gpios %ld\n",PTR_ERR(ctx->reset_gpio));
	}

	gpiod_set_value(ctx->reset_gpio,1);
	usleep_range(5000, 5100);
	gpiod_set_value(ctx->reset_gpio,0);
	usleep_range(5000, 5100);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(15000, 15100);

	return 0;
}

struct drm_display_mode *get_mode_by_id(struct drm_connector *connector,
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

static int mtk_panel_ext_param_set(struct drm_panel *panel,
			struct drm_connector *connector, unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	int m_vrefresh = 0;
	struct drm_display_mode *m = get_mode_by_id(connector, mode);

	m_vrefresh = drm_mode_vrefresh(m);
	pr_info("%s: mode=%d, vrefresh=%d\n", __func__, mode, drm_mode_vrefresh(m));

	if (m_vrefresh == 60) {
		ext->params = &ext_params_60Hz;
		current_fps = 60;
	} else if (m_vrefresh == 90) {
		ext->params = &ext_params_90Hz;
		current_fps = 90;
	}else if (m_vrefresh == 120) {
		ext->params = &ext_params_120Hz;
		current_fps = 120;
	}else if (m_vrefresh == 30) {
		ext->params = &ext_params_30Hz;
		current_fps = 30;
	} else {
		ret = 1;
	}

	return ret;
}

/*
static unsigned int last_fps_mode = 60;
static int mode_switch(struct drm_panel *panel,
		struct drm_connector *connector, unsigned int cur_mode,
		unsigned int dst_mode, enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	int ret = 0;
	struct drm_display_mode *m = get_mode_by_id(connector, dst_mode);
	struct lcm *ctx = panel_to_lcm(panel);
	pr_info("%s cur_mode = %d dst_mode %d\n", __func__, cur_mode, dst_mode);
	if (cur_mode == dst_mode)
		return ret;
	if (drm_mode_vrefresh(m) == 60) {
		if (stage == BEFORE_DSI_POWERDOWN){
			push_table(ctx, mode_switch_to_60, sizeof(mode_switch_to_60) / sizeof(struct LCM_setting_table));
			if (last_fps_mode == 120) {
				usleep_range(8300, 8400);
			}
			last_fps_mode = 60;
			pr_info("%s timing switch to 60 success\n", __func__);
			ret = 1;
		}
	} else if (drm_mode_vrefresh(m) == 120) {
		if (stage == AFTER_DSI_POWERON){
			push_table(ctx, mode_switch_to_120, sizeof(mode_switch_to_120) / sizeof(struct LCM_setting_table));
			last_fps_mode = 120;
			pr_info("%s timing switch to 120 success\n", __func__);
			ret = 1;
		}
	}
	ctx->m = m;
	return ret;
}
*/
static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.panel_poweron = lcm_panel_poweron,
	.panel_poweroff = lcm_panel_poweroff,
	.panel_reset = lcm_panel_reset,
	.ata_check = panel_ata_check,
	.ext_param_set = mtk_panel_ext_param_set,
	// .ext_param_get = mtk_panel_ext_param_get,
	//.mode_switch = mode_switch,
	.esd_backlight_recovery = oplus_esd_backlight_recovery,
/* #ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	.set_hbm = lcm_set_hbm,
	.oplus_ofp_set_lhbm_pressed_icon_single = oplus_ofp_set_lhbm_pressed_icon,
	.doze_disable = panel_doze_disable,
	.doze_enable = panel_doze_enable,
	.lcm_set_hbm_max_vdo = oplus_display_panel_set_hbm_max,
	.set_aod_light_mode = panel_set_aod_light_mode,
	.get_virtual_heigh = lcm_get_virtual_heigh,
	.get_virtual_width = lcm_get_virtual_width,
	.get_vdo_aod_param = mtk_get_vdo_aod_param,
/* #endif  *//* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
};

static int lcm_get_modes(struct drm_panel *panel,
					struct drm_connector *connector)
{
	struct drm_display_mode *mode[4];

	if (need_fake_resolution) {
		change_drm_disp_mode_params(&disp_mode_30Hz);
		change_drm_disp_mode_params(&disp_mode_60Hz);
		change_drm_disp_mode_params(&disp_mode_90Hz);
		change_drm_disp_mode_params(&disp_mode_120Hz);
	}

	mode[0] = drm_mode_duplicate(connector->dev, &disp_mode_120Hz);
	if (!mode[0]) {
		pr_info("%s failed to add mode %ux%ux@%u\n", __func__, disp_mode_120Hz.hdisplay, disp_mode_120Hz.vdisplay, drm_mode_vrefresh(&disp_mode_120Hz));
		return -ENOMEM;
	}
	drm_mode_set_name(mode[0]);
	mode[0]->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode[0]);
	pr_info("%s clock=%d,htotal=%d,vtotal=%d,hskew=%d,vrefresh=%d\n", __func__, mode[0]->clock, mode[0]->htotal,
		mode[0]->vtotal, mode[0]->hskew, drm_mode_vrefresh(mode[0]));

	mode[1] = drm_mode_duplicate(connector->dev, &disp_mode_90Hz);
	if (!mode[1]) {
		pr_info("%s failed to add mode %ux%ux@%u\n", __func__, disp_mode_90Hz.hdisplay, disp_mode_90Hz.vdisplay, drm_mode_vrefresh(&disp_mode_90Hz));
		return -ENOMEM;
	}
	drm_mode_set_name(mode[1]);
	mode[1]->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode[1]);

	mode[2] = drm_mode_duplicate(connector->dev, &disp_mode_60Hz);
	if (!mode[2]) {
		pr_info("%s failed to add mode %ux%ux@%u\n", __func__, disp_mode_60Hz.hdisplay, disp_mode_60Hz.vdisplay, drm_mode_vrefresh(&disp_mode_60Hz));
		return -ENOMEM;
	}
	drm_mode_set_name(mode[2]);
	mode[2]->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode[2]);

	mode[3] = drm_mode_duplicate(connector->dev, &disp_mode_30Hz);
	if (!mode[3]) {
		pr_info("%s failed to add mode %ux%ux@%u\n", __func__, disp_mode_30Hz.hdisplay, disp_mode_30Hz.vdisplay, drm_mode_vrefresh(&disp_mode_30Hz));
		return -ENOMEM;
	}
	drm_mode_set_name(mode[3]);
	mode[3]->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode[3]);

	connector->display_info.width_mm = 69;
	connector->display_info.height_mm = 152;

	return 1;
}

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
	unsigned int fp_type = 0xA10;
	int ret;

	pr_info("[LCM] ae211_p_3_a0027 %s+\n", __func__);

	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);
		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				pr_info("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			pr_info("device node name:%s\n", remote_node->name);
		}
	}

	if (remote_node != dev->of_node) {
		pr_info("%s+ skip probe due to not current lcm\n", __func__);
		return -ENODEV;
	}

	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE
			| MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_NO_EOT_PACKET
			| MIPI_DSI_CLOCK_NON_CONTINUOUS;

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_info(dev, "%s: cannot get reset-gpios %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(dev, ctx->reset_gpio);

#ifdef OLED_VDDI_EN_HW_SUPPORT
	ctx->vddi1p8_enable_gpio = devm_gpiod_get(dev, "vddi-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddi1p8_enable_gpio)) {
		dev_err(dev, "%s: cannot get vddi1p8_enable_gpio %ld\n",
			__func__, PTR_ERR(ctx->vddi1p8_enable_gpio));
		return PTR_ERR(ctx->vddi1p8_enable_gpio);
	}
	devm_gpiod_put(dev, ctx->vddi1p8_enable_gpio);
#endif

	ctx->vddr1p2_enable_gpio = devm_gpiod_get(dev, "vddr-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddr1p2_enable_gpio)) {
		dev_err(dev, "%s: cannot get vddr1p2_enable_gpio %ld\n",
			__func__, PTR_ERR(ctx->vddr1p2_enable_gpio));
		return PTR_ERR(ctx->vddr1p2_enable_gpio);
	}
	devm_gpiod_put(dev, ctx->vddr1p2_enable_gpio);

	//set vci 3.0v
	ctx->vci3p0_enable_gpio = devm_gpiod_get(dev, "vci-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vci3p0_enable_gpio)) {
		dev_err(dev, "%s: cannot get vci3p0_enable_gpio %ld\n",
			__func__, PTR_ERR(ctx->vci3p0_enable_gpio));
		return PTR_ERR(ctx->vci3p0_enable_gpio);
	}
	devm_gpiod_put(dev, ctx->vci3p0_enable_gpio);

	ctx->prepared = true;
	ctx->enabled = true;

	drm_panel_init(&ctx->panel, dev, &lcm_drm_funcs, DRM_MODE_CONNECTOR_DSI);

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params_120Hz, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;
#endif
	oplus_display_panel_dbv_probe(dev);
	register_device_proc("lcd", "A0027", "P_3_A0027");
	oplus_ofp_set_fp_type(&fp_type);
	ctx->hbm_en = false;
	oplus_max_normal_brightness = MAX_NORMAL_BRIGHTNESS;
	// oplus_ofp_init(dev);

	check_is_need_fake_resolution(dev);
	pr_info("[LCM] ae211_p_3_a0027 %s-\n", __func__);

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
	{ .compatible = "panel_ae211_p_3_a0027_dsi_vdo", },
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "panel_ae211_p_3_a0027_dsi_vdo",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("oplus");
MODULE_DESCRIPTION("lcm AMOLED Panel Driver");
MODULE_LICENSE("GPL v2");

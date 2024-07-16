/********************************************
 ** Copyright (C) 2023 OPLUS Mobile Comm Corp. Ltd.
 ** OPLUS_EDIT
 ** File: oplus24605_csot_td4377_fhd_dsi_vdo.h
 ** Description: Source file for LCD driver To Control LCD driver
 ** Version : 1.0
 ** Date : 2023/03/09
 ** ---------------- Revision History: --------------------------
 ** <version: >        <date>                  < author >                          <desc>
 ********************************************/

#include <linux/backlight.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_modes.h>
#include <linux/delay.h>
#include <drm/drm_connector.h>
#include <drm/drm_device.h>
#include <soc/oplus/system/oplus_mm_kevent_fb.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>
#include <soc/oplus/system/boot_mode.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <soc/oplus/device_info.h>
#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#endif

#if IS_ENABLED(CONFIG_DRM_OPLUS_PANEL_NOTIFY)
#include <linux/msm_drm_notify.h>
#elif IS_ENABLED(CONFIG_DRM_PANEL_NOTIFY)
#include <linux/soc/qcom/panel_event_notifier.h>
#include <linux/msm_drm_notify.h>
#elif IS_ENABLED(CONFIG_DRM_MSM) || IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY)
#include <linux/msm_drm_notify.h>
#elif IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
#include <linux/mtk_panel_ext.h>
#include <linux/mtk_disp_notify.h>
#endif
#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
#ifndef CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY
extern enum boot_mode_t get_boot_mode(void);
#endif
#else
extern int get_boot_mode(void);
#endif
#include "oplus24605_csot_td4377_fhd_dsi_vdo.h"
#include "../bias/oplus23661_aw37501_bias.h"
#include <linux/reboot.h>

extern unsigned int oplus_display_brightness;
extern unsigned int oplus_max_normal_brightness;

#define MAX_NORMAL_BRIGHTNESS   4095
#define LOW_BACKLIGHT_LEVEL     11
static int shutdown_lcd_drv = 0;
extern unsigned int backlight_level_esd;
/* default to launcher mode */
static int cabc_mode_backup = 3;

#if IS_ENABLED(CONFIG_TOUCHPANEL_NOTIFY)
extern int (*tp_gesture_enable_notifier)(unsigned int tp_index);
#endif

#define jdi_dcs_write_seq(ctx, seq...)                                                                                 \
        ({                                                                                                                                         \
                const u8 d[] = { seq };                                                                                \
                BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 64,                                                   \
                                "DCS sequence too big for stack");                        \
                jdi_dcs_write(ctx, d, ARRAY_SIZE(d));                                                  \
        })

#define jdi_dcs_write_seq_static(ctx, seq...)                                                                  \
        ({                                                                                                                                         \
                static const u8 d[] = { seq };                                                                 \
                jdi_dcs_write(ctx, d, ARRAY_SIZE(d));                                                  \
        })

static inline struct jdi *panel_to_jdi(struct drm_panel *panel)
{
        return container_of(panel, struct jdi, panel);
}

#ifdef PANEL_SUPPORT_READBACK
static int jdi_dcs_read(struct jdi *ctx, u8 cmd, void *data, size_t len)
{
        struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
        ssize_t ret;

        if (ctx->error < 0)
                return 0;

        ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
        if (ret < 0) {
                dev_info(ctx->dev, "error %d reading dcs seq:(%#x)\n", ret,
                         cmd);
                ctx->error = ret;
        }

        return ret;
}

static void jdi_panel_get_data(struct jdi *ctx)
{
        u8 buffer[3] = { 0 };
        static int ret;

        pr_info("%s+\n", __func__);

        if (ret == 0) {
                ret = jdi_dcs_read(ctx, 0x0A, buffer, 1);
                pr_info("%s  0x%08x\n", __func__, buffer[0] | (buffer[1] << 8));
                dev_info(ctx->dev, "return %d data(0x%08x) to dsi engine\n",
                        ret, buffer[0] | (buffer[1] << 8));
        }
}
#endif

static void jdi_dcs_write(struct jdi *ctx, const void *data, size_t len)
{
        struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
        ssize_t ret;
        char *addr;

        if (ctx->error < 0)
                return;

        addr = (char *)data;

        if (len > 1)
                udelay(100);

        if ((int)*addr < 0xB0)
                ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
        else
                ret = mipi_dsi_generic_write(dsi, data, len);
        if (ret < 0) {
                dev_info(ctx->dev, "error %zd writing seq: %ph\n", ret, data);
                ctx->error = ret;
        }
}

static void jdi_panel_init(struct jdi *ctx)
{
	jdi_dcs_write_seq_static(ctx, 0xB0, 0x04);
        jdi_dcs_write_seq_static(ctx, 0xF5, 0xE6, 0x78);
        jdi_dcs_write_seq_static(ctx, 0xF0, 0xE6, 0x05, 0x73);
        jdi_dcs_write_seq_static(ctx, 0xD6, 0x00);
        jdi_dcs_write_seq_static(ctx, 0x11);
        usleep_range(105000, 105100);
        jdi_dcs_write_seq_static(ctx, 0x29);
        usleep_range(5000, 5100);

        jdi_dcs_write_seq_static(ctx, 0xF0, 0xBE, 0x00, 0x00);
        jdi_dcs_write_seq_static(ctx, 0xF0, 0xBE, 0x01, 0x04);

        jdi_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x00, 0x71);
        jdi_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x01, 0x01);
        jdi_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x02, 0x01);
        jdi_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x03, 0x01);
        jdi_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x04, 0x1b);
        jdi_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x05, 0x2c);
        jdi_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x06, 0x35);
        jdi_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x07, 0x45);
        jdi_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x08, 0x56);
        jdi_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x09, 0x5d);
        jdi_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x0a, 0x6f);
        jdi_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x0b, 0x89);
        jdi_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x0c, 0x8f);
        jdi_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x0d, 0x9d);
        jdi_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x0e, 0xad);
        jdi_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x0f, 0xb9);
        jdi_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x10, 0xd7);
        jdi_dcs_write_seq_static(ctx, 0xF0, 0xCE, 0x11, 0x90);

        jdi_dcs_write_seq_static(ctx, 0x51, 0x00, 0x00);
        jdi_dcs_write_seq_static(ctx, 0x53, 0x2C);
        jdi_dcs_write_seq_static(ctx, 0x35, 0x00);
        jdi_dcs_write_seq_static(ctx, 0xB0, 0x03);
        pr_info("%s-\n", __func__);
}

static void cabc_mode_retore(struct jdi *ctx)
{
	if (cabc_mode_backup == 1) {
		jdi_dcs_write_seq_static(ctx, 0x55, 0x11);
	} else if (cabc_mode_backup == 2) {
		jdi_dcs_write_seq_static(ctx, 0x55, 0x12);
	} else if (cabc_mode_backup == 3) {
		jdi_dcs_write_seq_static(ctx, 0x55, 0x13);
	} else if (cabc_mode_backup == 0) {
		jdi_dcs_write_seq_static(ctx, 0x55, 0x10);
	}
	pr_info("%s- cabc_mode_backup=%d\n", __func__, cabc_mode_backup);
}

static int jdi_disable(struct drm_panel *panel)
{
        struct jdi *ctx = panel_to_jdi(panel);

        if (!ctx->enabled)
                return 0;

        if (ctx->backlight) {
                ctx->backlight->props.power = FB_BLANK_POWERDOWN;
                backlight_update_status(ctx->backlight);
        }

        ctx->enabled = false;

        return 0;
}

static int jdi_unprepare(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);
	int flag_poweroff = 1;

	if (!ctx->prepared) {
		return 0;
	}

	if (tp_gesture_enable_notifier && tp_gesture_enable_notifier(0)) {
                if (shutdown_lcd_drv == 1) {
                        flag_poweroff = 1;
                } else {
                        flag_poweroff = 0;
                        pr_err("[TP] tp gesture  is enable,Display not to poweroff\n");
                }
	} else {
		flag_poweroff = 1;
	}

	pr_info("%s+ enter jdi_unprepare \n", __func__);
        usleep_range(20000, 20100);
	jdi_dcs_write_seq_static(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	usleep_range(60000, 60100);
	jdi_dcs_write_seq_static(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	usleep_range(150000, 150100);
	if (flag_poweroff == 1) {
		if (shutdown_lcd_drv == 1) {
                        pr_err(" %s shutdown \n", __func__);
                }

                ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
                gpiod_set_value(ctx->reset_gpio, 0);
                devm_gpiod_put(ctx->dev, ctx->reset_gpio);
                usleep_range(2000, 2001);

                ctx->bias_neg = devm_gpiod_get_index(ctx->dev, "bias", 1, GPIOD_OUT_HIGH);
                gpiod_set_value(ctx->bias_neg, 0);
                devm_gpiod_put(ctx->dev, ctx->bias_neg);

                usleep_range(2000, 2001);
                ctx->bias_pos = devm_gpiod_get_index(ctx->dev, "bias", 0, GPIOD_OUT_HIGH);
                gpiod_set_value(ctx->bias_pos, 0);
                devm_gpiod_put(ctx->dev, ctx->bias_pos);
	}
        ctx->error = 0;
        ctx->prepared = false;

        return 0;
}

static int jdi_prepare(struct drm_panel *panel)
{
        struct jdi *ctx = panel_to_jdi(panel);
        int ret;
        int mode;
        int blank;

        if (ctx->prepared)
                return 0;
        pr_err("%s", __func__);

        ctx->bias_pos = devm_gpiod_get_index(ctx->dev, "bias", 0, GPIOD_OUT_HIGH);
        gpiod_set_value(ctx->bias_pos, 1);
        devm_gpiod_put(ctx->dev, ctx->bias_pos);

        usleep_range(2000, 2001);
        ctx->bias_neg = devm_gpiod_get_index(ctx->dev, "bias", 1, GPIOD_OUT_HIGH);
        gpiod_set_value(ctx->bias_neg, 1);
        devm_gpiod_put(ctx->dev, ctx->bias_neg);

        lcm_i2c_write_bytes(0x0, 0xf);
        lcm_i2c_write_bytes(0x1, 0xf);

        usleep_range(11000, 11100);
        ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
        gpiod_set_value(ctx->reset_gpio, 1);
        usleep_range(2000, 2001);
        gpiod_set_value(ctx->reset_gpio, 0);
        usleep_range(2000, 2001);
        gpiod_set_value(ctx->reset_gpio, 1);
        usleep_range(2000, 2001);
        gpiod_set_value(ctx->reset_gpio, 0);
        usleep_range(3000, 3001);
        gpiod_set_value(ctx->reset_gpio, 1);
        devm_gpiod_put(ctx->dev, ctx->reset_gpio);
        usleep_range(20000, 20100);



        mode = get_boot_mode();
        pr_info("[TP] in dis_panel_power_on,mode = %d\n", mode);
        if ((mode != MSM_BOOT_MODE__FACTORY) &&(mode != MSM_BOOT_MODE__RF) && (mode != MSM_BOOT_MODE__WLAN)) {
                #define LCD_CTL_TP_LOAD_FW 0x10
                #define LCD_CTL_CS_ON  0x19
                blank = LCD_CTL_CS_ON;
                mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
                pr_err("[TP]TP CS will chang to spi mode and high\n");
                usleep_range(5000, 5100);
                blank = LCD_CTL_TP_LOAD_FW;
                mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
                pr_info("[TP] start to load fw!\n");
        }

        jdi_panel_init(ctx);
        /* restore cabc mode to before suspend */
        cabc_mode_retore(ctx);

        ret = ctx->error;
        if (ret < 0)
                jdi_unprepare(panel);

        ctx->prepared = true;

        pr_info("%s-\n", __func__);
        return ret;
}

static int jdi_enable(struct drm_panel *panel)
{
        struct jdi *ctx = panel_to_jdi(panel);

        if (ctx->enabled)
                return 0;

        if (ctx->backlight) {
                ctx->backlight->props.power = FB_BLANK_UNBLANK;
                backlight_update_status(ctx->backlight);
        }

        ctx->enabled = true;

        return 0;
}

#define HFP (178)
#define HSA (16)
#define HBP (7)
#define VSA (10)
#define VBP (10)
#define VAC (2400)
#define HAC (1080)
#define VFP_45hz (4177)
#define VFP_48hz (3765)
#define VFP_50hz (3518)
#define VFP_60hz (2528)
#define VFP_90hz (879)
#define VFP_120hz (54)
#define DYN_PLL_CLK (514)
#define DYN_DATA_RATE (1028)
#define HFP_DYN (191)
#define PLL_CLOCK (502)
#define DATA_RATE (1004)

static const struct drm_display_mode default_mode_45hz = {
        .clock = 380284,
        .hdisplay = HAC,
        .hsync_start = HAC + HFP,
        .hsync_end = HAC + HFP + HSA,
        .htotal = HAC + HFP + HSA + HBP,
        .vdisplay = VAC,
        .vsync_start = VAC + VFP_45hz,
        .vsync_end = VAC + VFP_45hz + VSA,
        .vtotal = VAC + VFP_45hz + VSA + VBP,
};

static const struct drm_display_mode default_mode_48hz = {
        .clock = 380303,
        .hdisplay = HAC,
        .hsync_start = HAC + HFP,
        .hsync_end = HAC + HFP + HSA,
        .htotal = HAC + HFP + HSA + HBP,
        .vdisplay = VAC,
        .vsync_start = VAC + VFP_48hz,
        .vsync_end = VAC + VFP_48hz + VSA,
        .vtotal = VAC + VFP_48hz + VSA + VBP,
};

static const struct drm_display_mode default_mode_50hz = {
        .clock = 380328,
        .hdisplay = HAC,
        .hsync_start = HAC + HFP,
        .hsync_end = HAC + HFP + HSA,
        .htotal = HAC + HFP + HSA + HBP,
        .vdisplay = VAC,
        .vsync_start = VAC + VFP_50hz,
        .vsync_end = VAC + VFP_50hz + VSA,
        .vtotal = VAC + VFP_50hz + VSA + VBP,
};

static const struct drm_display_mode performance_mode_60hz = {
        .clock = 380303,
        .hdisplay = HAC,
        .hsync_start = HAC + HFP,
        .hsync_end = HAC + HFP + HSA,
        .htotal = HAC + HFP + HSA + HBP,
        .vdisplay = VAC,
        .vsync_start = VAC + VFP_60hz,
        .vsync_end = VAC + VFP_60hz + VSA,
        .vtotal = VAC + VFP_60hz + VSA + VBP,
};
static const struct drm_display_mode performance_mode_90hz = {
                .clock = 380341,
                .hdisplay = HAC,
                .hsync_start = HAC + HFP,
                .hsync_end = HAC + HFP + HSA,
                .htotal = HAC + HFP + HSA + HBP,
                .vdisplay = VAC,
                .vsync_start = VAC + VFP_90hz,
                .vsync_end = VAC + VFP_90hz + VSA,
                .vtotal = VAC + VFP_90hz + VSA + VBP,
};

static const struct drm_display_mode performance_mode_120hz = {
        .clock = 380303,
        .hdisplay = HAC,
        .hsync_start = HAC + HFP,
        .hsync_end = HAC + HFP + HSA,
        .htotal = HAC + HFP + HSA + HBP,
        .vdisplay = VAC,
        .vsync_start = VAC + VFP_120hz,
        .vsync_end = VAC + VFP_120hz + VSA,
        .vtotal = VAC + VFP_120hz + VSA + VBP,
};

#if defined(CONFIG_MTK_PANEL_EXT)

static struct mtk_panel_params ext_params_45hz = {
                .vendor = "td4377_barley",
                .manufacture = "24605_csot_td4377",
                .pll_clk = PLL_CLOCK,
                .cust_esd_check = 1,
                .esd_check_enable = 1,
                .lcm_degree = PROBE_FROM_DTS,
				.lcm_esd_check_table[0] = {
                        .cmd = 0x0A, .count = 1, .para_list[0] = 0x1C,
                },
                .ssc_enable = 0,
                .lane_swap_en = 0,
                .ap_tx_keep_hs_during_vact = 1,
                .output_mode = MTK_PANEL_DSC_SINGLE_PORT,
                .dsc_params = {
                                .enable = 1,
                                .ver = 17,
                                .slice_mode = 1,
                                .rgb_swap = 0,
                                .dsc_cfg = 34,
                                .rct_on = 1,
                                .bit_per_channel = 8,
                                .dsc_line_buf_depth = 9,
                                .bp_enable = 1,
                                .bit_per_pixel = 128,
                                .pic_height = 2400,
                                .pic_width = 1080,
                                .slice_height = 12,
                                .slice_width = 540,
                                .chunk_size = 540,
                                .xmit_delay = 512,
                                .dec_delay = 526,
                                .scale_value = 32,
                                .increment_interval = 287,
                                .decrement_interval = 7,
                                .line_bpg_offset = 12,
                                .nfl_bpg_offset = 2235,
                                .slice_bpg_offset = 2170,
                                .initial_offset = 6144,
                                .final_offset = 4336,
                                .flatness_minqp = 3,
                                .flatness_maxqp = 12,
                                .rc_model_size = 8192,
                                .rc_edge_factor = 6,
                                .rc_quant_incr_limit0 = 11,
                                .rc_quant_incr_limit1 = 11,
                                .rc_tgt_offset_hi = 3,
                                .rc_tgt_offset_lo = 3,
                                },
                .data_rate = DATA_RATE,
                .lfr_enable = 0,
                .lfr_minimum_fps = 60,
                .vdo_per_frame_lp_enable = 1,
                .dyn_fps = {
                                .switch_en = 1,
                                .vact_timing_fps = 120,
                },
                /* following MIPI hopping parameter might cause screen mess */
                .dyn = {
                                .switch_en = 1,
                                .pll_clk = DYN_PLL_CLK,
                                .data_rate = DYN_DATA_RATE,
                                .vsa = VSA,
                                .vbp = VBP,
                                .vfp = VFP_45hz,
                                .hsa = HSA,
                                .hbp = HBP,
                                .hfp = HFP_DYN,
                },
                .phy_timcon = {
                        .hs_trail = 10,
                },
                .oplus_display_global_dre = 1,
		.cabc_three_to_zero = 1,
};

static struct mtk_panel_params ext_params_48hz = {
                .pll_clk = PLL_CLOCK,
                .vendor = "td4377_barley",
                .manufacture = "24605_csot_td4377",
                .cust_esd_check = 1,
                .esd_check_enable = 1,
                .lcm_degree = PROBE_FROM_DTS,
				.lcm_esd_check_table[0] = {
                        .cmd = 0x0A, .count = 1, .para_list[0] = 0x1C,
                },
                .ssc_enable = 0,
                .lane_swap_en = 0,
                .ap_tx_keep_hs_during_vact = 1,
                .output_mode = MTK_PANEL_DSC_SINGLE_PORT,
                .dsc_params = {
                                .enable = 1,
                                .ver = 17,
                                .slice_mode = 1,
                                .rgb_swap = 0,
                                .dsc_cfg = 34,
                                .rct_on = 1,
                                .bit_per_channel = 8,
                                .dsc_line_buf_depth = 9,
                                .bp_enable = 1,
                                .bit_per_pixel = 128,
                                .pic_height = 2400,
                                .pic_width = 1080,
                                .slice_height = 12,
                                .slice_width = 540,
                                .chunk_size = 540,
                                .xmit_delay = 512,
                                .dec_delay = 526,
                                .scale_value = 32,
                                .increment_interval = 287,
                                .decrement_interval = 7,
                                .line_bpg_offset = 12,
                                .nfl_bpg_offset = 2235,
                                .slice_bpg_offset = 2170,
                                .initial_offset = 6144,
                                .final_offset = 4336,
                                .flatness_minqp = 3,
                                .flatness_maxqp = 12,
                                .rc_model_size = 8192,
                                .rc_edge_factor = 6,
                                .rc_quant_incr_limit0 = 11,
                                .rc_quant_incr_limit1 = 11,
                                .rc_tgt_offset_hi = 3,
                                .rc_tgt_offset_lo = 3,
                                },
                .data_rate = DATA_RATE,
                .lfr_enable = 0,
                .lfr_minimum_fps = 60,
                .vdo_per_frame_lp_enable = 1,
                .dyn_fps = {
                                .switch_en = 1,
                                .vact_timing_fps = 120,
                },

                .dyn = {
                                .switch_en = 1,
                                .pll_clk = DYN_PLL_CLK,
                                .data_rate = DYN_DATA_RATE,
                                .vsa = VSA,
                                .vbp = VBP,
                                .vfp = VFP_48hz,
                                .hsa = HSA,
                                .hbp = HBP,
                                .hfp = HFP_DYN,
                },
                .phy_timcon = {
                        .hs_trail = 10,
                },
                .oplus_display_global_dre = 1,
		.cabc_three_to_zero = 1,
};

static struct mtk_panel_params ext_params_50hz = {
                .pll_clk = PLL_CLOCK,
                .vendor = "td4377_barley",
                .manufacture = "24605_csot_td4377",
                .cust_esd_check = 1,
                .esd_check_enable = 1,
                .lcm_degree = PROBE_FROM_DTS,
				.lcm_esd_check_table[0] = {
                        .cmd = 0x0A, .count = 1, .para_list[0] = 0x1C,
                },
                .ssc_enable = 0,
                .lane_swap_en = 0,
                .ap_tx_keep_hs_during_vact = 1,
                .output_mode = MTK_PANEL_DSC_SINGLE_PORT,
                .dsc_params = {
                                .enable = 1,
                                .ver = 17,
                                .slice_mode = 1,
                                .rgb_swap = 0,
                                .dsc_cfg = 34,
                                .rct_on = 1,
                                .bit_per_channel = 8,
                                .dsc_line_buf_depth = 9,
                                .bp_enable = 1,
                                .bit_per_pixel = 128,
                                .pic_height = 2400,
                                .pic_width = 1080,
                                .slice_height = 12,
                                .slice_width = 540,
                                .chunk_size = 540,
                                .xmit_delay = 512,
                                .dec_delay = 526,
                                .scale_value = 32,
                                .increment_interval = 287,
                                .decrement_interval = 7,
                                .line_bpg_offset = 12,
                                .nfl_bpg_offset = 2235,
                                .slice_bpg_offset = 2170,
                                .initial_offset = 6144,
                                .final_offset = 4336,
                                .flatness_minqp = 3,
                                .flatness_maxqp = 12,
                                .rc_model_size = 8192,
                                .rc_edge_factor = 6,
                                .rc_quant_incr_limit0 = 11,
                                .rc_quant_incr_limit1 = 11,
                                .rc_tgt_offset_hi = 3,
                                .rc_tgt_offset_lo = 3,
                                },
                .data_rate = DATA_RATE,
                .lfr_enable = 0,
                .lfr_minimum_fps = 60,
                .vdo_per_frame_lp_enable = 1,
                .dyn_fps = {
                                .switch_en = 1,
                                .vact_timing_fps = 120,
                },
                /* following MIPI hopping parameter might cause screen mess */
                .dyn = {
                                .switch_en = 1,
                                .pll_clk = DYN_PLL_CLK,
                                .data_rate = DYN_DATA_RATE,

                                .vsa = VSA,
                                .vbp = VBP,
                                .vfp = VFP_50hz,
                                .hsa = HSA,
                                .hbp = HBP,
                                .hfp = HFP_DYN,
                },
                .phy_timcon = {
                        .hs_trail = 10,
                },
                .oplus_display_global_dre = 1,
		.cabc_three_to_zero = 1,
};
static struct mtk_panel_params ext_params_60hz = {
                .vendor = "td4377_barley",
                .manufacture = "24605_csot_td4377",
                .pll_clk = PLL_CLOCK,
                .cust_esd_check = 1,
                .esd_check_enable = 1,
                .lcm_degree = PROBE_FROM_DTS,
				.lcm_esd_check_table[0] = {
                        .cmd = 0x0A, .count = 1, .para_list[0] = 0x1C,
                },
                .ssc_enable = 0,
                .lane_swap_en = 0,
                .ap_tx_keep_hs_during_vact = 1,
                .output_mode = MTK_PANEL_DSC_SINGLE_PORT,
                .dsc_params = {
                                .enable = 1,
                                .ver = 17,
                                .slice_mode = 1,
                                .rgb_swap = 0,
                                .dsc_cfg = 34,
                                .rct_on = 1,
                                .bit_per_channel = 8,
                                .dsc_line_buf_depth = 9,
                                .bp_enable = 1,
                                .bit_per_pixel = 128,
                                .pic_height = 2400,
                                .pic_width = 1080,
                                .slice_height = 12,
                                .slice_width = 540,
                                .chunk_size = 540,
                                .xmit_delay = 512,
                                .dec_delay = 526,
                                .scale_value = 32,
                                .increment_interval = 287,
                                .decrement_interval = 7,
                                .line_bpg_offset = 12,
                                .nfl_bpg_offset = 2235,
                                .slice_bpg_offset = 2170,
                                .initial_offset = 6144,
                                .final_offset = 4336,
                                .flatness_minqp = 3,
                                .flatness_maxqp = 12,
                                .rc_model_size = 8192,
                                .rc_edge_factor = 6,
                                .rc_quant_incr_limit0 = 11,
                                .rc_quant_incr_limit1 = 11,
                                .rc_tgt_offset_hi = 3,
                                .rc_tgt_offset_lo = 3,
                                },
                .data_rate = DATA_RATE,
                .lfr_enable = 0,
                .lfr_minimum_fps = 60,
                .vdo_per_frame_lp_enable = 1,
                .dyn_fps = {
                                .switch_en = 1,
                                .vact_timing_fps = 120,
                },
                /* following MIPI hopping parameter might cause screen mess */
                .dyn = {
                                .switch_en = 1,
                                .pll_clk = DYN_PLL_CLK,
                                .data_rate = DYN_DATA_RATE,
                                .vsa = VSA,
                                .vbp = VBP,
                                .vfp = VFP_60hz,
                                .hsa = HSA,
                                .hbp = HBP,
                                .hfp = HFP_DYN,
                },
                .phy_timcon = {
                        .hs_trail = 10,
                },
                .oplus_display_global_dre = 1,
		.cabc_three_to_zero = 1,
};

static struct mtk_panel_params ext_params_90hz = {
                .pll_clk = PLL_CLOCK,
                .vendor = "td4377_barley",
                .manufacture = "24605_csot_td4377",
                .cust_esd_check = 1,
                .esd_check_enable = 1,
                .lcm_degree = PROBE_FROM_DTS,
				.lcm_esd_check_table[0] = {
                        .cmd = 0x0A, .count = 1, .para_list[0] = 0x1C,
                },
                .ssc_enable = 0,
                .lane_swap_en = 0,
                .ap_tx_keep_hs_during_vact = 1,
                .output_mode = MTK_PANEL_DSC_SINGLE_PORT,
                .dsc_params = {
                                .enable = 1,
                                .ver = 17,
                                .slice_mode = 1,
                                .rgb_swap = 0,
                                .dsc_cfg = 34,
                                .rct_on = 1,
                                .bit_per_channel = 8,
                                .dsc_line_buf_depth = 9,
                                .bp_enable = 1,
                                .bit_per_pixel = 128,
                                .pic_height = 2400,
                                .pic_width = 1080,
                                .slice_height = 12,
                                .slice_width = 540,
                                .chunk_size = 540,
                                .xmit_delay = 512,
                                .dec_delay = 526,
                                .scale_value = 32,
                                .increment_interval = 287,
                                .decrement_interval = 7,
                                .line_bpg_offset = 12,
                                .nfl_bpg_offset = 2235,
                                .slice_bpg_offset = 2170,
                                .initial_offset = 6144,
                                .final_offset = 4336,
                                .flatness_minqp = 3,
                                .flatness_maxqp = 12,
                                .rc_model_size = 8192,
                                .rc_edge_factor = 6,
                                .rc_quant_incr_limit0 = 11,
                                .rc_quant_incr_limit1 = 11,
                                .rc_tgt_offset_hi = 3,
                                .rc_tgt_offset_lo = 3,
                                },
                .data_rate = DATA_RATE,
                .lfr_enable = 0,
                .lfr_minimum_fps = 60,
                .vdo_per_frame_lp_enable = 1,
                .dyn_fps = {
                                .switch_en = 1,
                                .vact_timing_fps = 120,
                },

                .dyn = {
                                .switch_en = 1,
                                .pll_clk = DYN_PLL_CLK,
                                .data_rate = DYN_DATA_RATE,
                                .vsa = VSA,
                                .vbp = VBP,
                                .vfp = VFP_90hz,
                                .hsa = HSA,
                                .hbp = HBP,
                                .hfp = HFP_DYN,
                },
                .phy_timcon = {
                        .hs_trail = 10,
                },
                .oplus_display_global_dre = 1,
		.cabc_three_to_zero = 1,
};

static struct mtk_panel_params ext_params_120hz = {
                .pll_clk = PLL_CLOCK,
                .vendor = "td4377_barley",
                .manufacture = "24605_csot_td4377",
                .cust_esd_check = 1,
                .esd_check_enable = 1,
                .lcm_degree = PROBE_FROM_DTS,
				.lcm_esd_check_table[0] = {
                        .cmd = 0x0A, .count = 1, .para_list[0] = 0x1C,
                },
                .ssc_enable = 0,
                .lane_swap_en = 0,
                .ap_tx_keep_hs_during_vact = 1,
                .output_mode = MTK_PANEL_DSC_SINGLE_PORT,
                .dsc_params = {
                                .enable = 1,
                                .ver = 17,
                                .slice_mode = 1,
                                .rgb_swap = 0,
                                .dsc_cfg = 34,
                                .rct_on = 1,
                                .bit_per_channel = 8,
                                .dsc_line_buf_depth = 9,
                                .bp_enable = 1,
                                .bit_per_pixel = 128,
                                .pic_height = 2400,
                                .pic_width = 1080,
                                .slice_height = 12,
                                .slice_width = 540,
                                .chunk_size = 540,
                                .xmit_delay = 512,
                                .dec_delay = 526,
                                .scale_value = 32,
                                .increment_interval = 287,
                                .decrement_interval = 7,
                                .line_bpg_offset = 12,
                                .nfl_bpg_offset = 2235,
                                .slice_bpg_offset = 2170,
                                .initial_offset = 6144,
                                .final_offset = 4336,
                                .flatness_minqp = 3,
                                .flatness_maxqp = 12,
                                .rc_model_size = 8192,
                                .rc_edge_factor = 6,
                                .rc_quant_incr_limit0 = 11,
                                .rc_quant_incr_limit1 = 11,
                                .rc_tgt_offset_hi = 3,
                                .rc_tgt_offset_lo = 3,
                                },
                .data_rate = DATA_RATE,
                .lfr_enable = 0,
                .lfr_minimum_fps = 60,
                .vdo_per_frame_lp_enable = 1,
                .dyn_fps = {
                                .switch_en = 1,
                                .vact_timing_fps = 120,
                },

                .dyn = {
                                .switch_en = 1,
                                .pll_clk = DYN_PLL_CLK,
                                .data_rate = DYN_DATA_RATE,
                                .vsa = VSA,
                                .vbp = VBP,
                                .vfp = VFP_120hz,
                                .hsa = HSA,
                                .hbp = HBP,
                                .hfp = HFP_DYN,
                },
                .phy_timcon = {
                        .hs_trail = 10,
                },
                .oplus_display_global_dre = 1,
		.cabc_three_to_zero = 1,
};

static int panel_ata_check(struct drm_panel *panel)
{
        return 1;
}

static int map_exp[4096] = {0};

static void init_global_exp_backlight(void)
{
        int lut_index[41] = {0, 4, 99, 144, 187, 227, 264, 300, 334, 366, 397, 427, 456, 484, 511, 537, 563, 587, 611, 635, 658, 680,
                                                702, 723, 744, 764, 784, 804, 823, 842, 861, 879, 897, 915, 933, 950, 967, 984, 1000, 1016, 1023};
        int lut_value1[41] = {0, 4, 6, 14, 24, 37, 52, 69, 87, 107, 128, 150, 173, 197, 222, 248, 275, 302, 330, 358, 387, 416, 446,
                                                479, 509, 541, 572, 604, 636, 669, 702, 735, 769, 803, 837, 871, 905, 938, 973, 1008, 1023};
        int index_start = 0, index_end = 0;
        int value1_start = 0, value1_end = 0;
        int i, j;
        int index_len = sizeof(lut_index) / sizeof(int);
        int value_len = sizeof(lut_value1) / sizeof(int);
        if (index_len == value_len) {
                for (i = 0; i < index_len - 1; i++) {
                        index_start = lut_index[i] * MAX_NORMAL_BRIGHTNESS / 1023;
                        index_end = lut_index[i+1] * MAX_NORMAL_BRIGHTNESS / 1023;
                        value1_start = lut_value1[i] * MAX_NORMAL_BRIGHTNESS / 1023;
                        value1_end = lut_value1[i+1] * MAX_NORMAL_BRIGHTNESS / 1023;
                        for (j = index_start; j <= index_end; j++) {
                                map_exp[j] = value1_start + (value1_end - value1_start) * (j - index_start) / (index_end - index_start);
                        }
                }
        }
}

static int csot_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle,
				 unsigned int level)
{
	char bl_tb0[] = {0x51, 0x0F, 0xFF};
	char bl_tb1[] = {0x53, 0x24};
	int bl_map;

	if (level > 4095) {
		level = 4095;
	}

	oplus_display_brightness = level;
	bl_map = level;
	backlight_level_esd = level;
	if ((level > 0) && (level < MAX_NORMAL_BRIGHTNESS)) {
		bl_map = map_exp[level];
	}

        if (!cb)
                return -1;

	bl_tb0[1] = bl_map >> 8;
	bl_tb0[2] = bl_map & 0xFF;

	if (bl_map < LOW_BACKLIGHT_LEVEL) {
		cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	}

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	pr_info("%s, exit backlight level = %d, bl_map = %d, oplus_display_brightness = %d\n", __func__, level, bl_map, oplus_display_brightness);
	return 0;
}

static int oplus_esd_backlight_recovery(void *dsi, dcs_write_gce cb,
		void *handle)
{
	char bl_tb0[] = {0x51, 0x03, 0xff};

	bl_tb0[1] = backlight_level_esd >> 8;
	bl_tb0[2] = backlight_level_esd & 0xFF;
	if (!cb)
		return -1;
	pr_err("%s bl_tb0[1]=%x, bl_tb0[2]=%x\n", __func__, bl_tb0[1], bl_tb0[2]);
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	return 1;
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
        int target_fps;
        struct drm_display_mode *m = get_mode_by_id(connector, mode);
        target_fps = drm_mode_vrefresh(m);

        if (target_fps == 120) {
                ext->params = &ext_params_120hz;
        } else if (target_fps == 90) {
                ext->params = &ext_params_90hz;
        } else if (target_fps == 60) {
                ext->params = &ext_params_60hz;
        } else if (target_fps == 50) {
                ext->params = &ext_params_50hz;
        } else if (target_fps == 48) {
                ext->params = &ext_params_48hz;
        } else if (target_fps == 45) {
                ext->params = &ext_params_45hz;
        } else {
                pr_err("[ %s : %d ] : No mode to set fps = %d \n", __func__ ,  __LINE__ , target_fps);
                ret = 1;
        }
        return ret;
}

static int mtk_panel_ext_param_get(struct drm_panel *panel,
                struct drm_connector *connector,
                struct mtk_panel_params **ext_param, unsigned int id)
{
        int ret = 0;

        if (id == 0) {
                *ext_param = &ext_params_120hz;
        } else if (id == 1) {
                *ext_param = &ext_params_90hz;
        } else if (id == 2) {
                *ext_param = &ext_params_60hz;
        } else if (id == 3) {
                *ext_param = &ext_params_50hz;
        } else if (id == 4) {
                *ext_param = &ext_params_48hz;
        } else if (id == 5) {
                *ext_param = &ext_params_45hz;
        } else {
                ret = 1;
		}
        return ret;
}

static int panel_ext_reset(struct drm_panel *panel, int on)
{
        struct jdi *ctx = panel_to_jdi(panel);

        ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
        gpiod_set_value(ctx->reset_gpio, on);
        devm_gpiod_put(ctx->dev, ctx->reset_gpio);

        return 0;
}

static void cabc_mode_switch(void *dsi, dcs_write_gce cb, void *handle, unsigned int cabc_mode)
{
	char bl_tb0[] = {0x55, 0x10};

	pr_err("%s cabc_mode = %d\n", __func__, cabc_mode);
	if (cabc_mode > 3) {
		pr_err("%s: Invaild params skiped!\n", __func__);
		return;
	}

	if (cabc_mode == 1) {
		bl_tb0[1] = 0x11;
		cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	} else if (cabc_mode == 2) {
		bl_tb0[1] = 0x12;
		cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	} else if (cabc_mode == 3) {
		bl_tb0[1] = 0x13;
		cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	} else if (cabc_mode == 0) {
		bl_tb0[1] = 0x10;
		cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	}
	cabc_mode_backup = cabc_mode;
}
static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = csot_setbacklight_cmdq,
	.ext_param_set = mtk_panel_ext_param_set,
	.ext_param_get = mtk_panel_ext_param_get,
	.ata_check = panel_ata_check,
	.cabc_switch = cabc_mode_switch,
	.esd_backlight_recovery = oplus_esd_backlight_recovery,
};
#endif

struct panel_desc {
        const struct drm_display_mode *modes;
        unsigned int num_modes;

        unsigned int bpc;

        struct {
                unsigned int width;
                unsigned int height;
        } size;

        /**
         * @prepare: the time (in milliseconds) that it takes for the panel to
         *           become ready and start receiving video data
         * @enable: the time (in milliseconds) that it takes for the panel to
         *          display the first valid frame after starting to receive
         *          video data
         * @disable: the time (in milliseconds) that it takes for the panel to
         *           turn the display off (no content is visible)
         * @unprepare: the time (in milliseconds) that it takes for the panel
         *                 to power itself down completely
         */
        struct {
                unsigned int prepare;
                unsigned int enable;
                unsigned int disable;
                unsigned int unprepare;
        } delay;
};

static int jdi_get_modes(struct drm_panel *panel,
                            struct drm_connector *connector)
{
        struct drm_display_mode *mode;
        struct drm_display_mode *mode2;
        struct drm_display_mode *mode3;
        struct drm_display_mode *mode4;
        struct drm_display_mode *mode5;
        struct drm_display_mode *mode6;

        mode = drm_mode_duplicate(connector->dev, &performance_mode_60hz);
        if (!mode) {
                dev_info(connector->dev->dev, "failed to add mode1 %ux%ux@%u\n",
                         performance_mode_60hz.hdisplay, performance_mode_60hz.vdisplay,
                         drm_mode_vrefresh(&performance_mode_60hz));
                return -ENOMEM;
        }

        drm_mode_set_name(mode);
        mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
        drm_mode_probed_add(connector, mode);


        mode2 = drm_mode_duplicate(connector->dev, &default_mode_45hz);
        if (!mode2) {
                dev_info(connector->dev->dev, "failed to add mode2 %ux%ux@%u\n",
                         default_mode_45hz.hdisplay, default_mode_45hz.vdisplay,
                         drm_mode_vrefresh(&default_mode_45hz));
                return -ENOMEM;
        }
        drm_mode_set_name(mode2);
        mode2->type = DRM_MODE_TYPE_DRIVER;
        drm_mode_probed_add(connector, mode2);

        mode3 = drm_mode_duplicate(connector->dev, &default_mode_48hz);
        if (!mode3) {
                dev_info(connector->dev->dev, "failed to add mode3 %ux%ux@%u\n",
                         default_mode_48hz.hdisplay, default_mode_48hz.vdisplay,
                         drm_mode_vrefresh(&default_mode_48hz));
                return -ENOMEM;
        }
        drm_mode_set_name(mode3);
        mode3->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
        drm_mode_probed_add(connector, mode3);

        mode4 = drm_mode_duplicate(connector->dev, &default_mode_50hz);
        if (!mode4) {
                dev_info(connector->dev->dev, "failed to add mode4 %ux%ux@%u\n",
                         default_mode_50hz.hdisplay, default_mode_50hz.vdisplay,
                         drm_mode_vrefresh(&default_mode_50hz));
                return -ENOMEM;
        }
        drm_mode_set_name(mode4);
        mode4->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
        drm_mode_probed_add(connector, mode4);

        mode5 = drm_mode_duplicate(connector->dev, &performance_mode_90hz);
        if (!mode5) {
                dev_info(connector->dev->dev, "failed to add mode5 %ux%ux@%u\n",
                         performance_mode_90hz.hdisplay, performance_mode_90hz.vdisplay,
                         drm_mode_vrefresh(&performance_mode_90hz));
                return -ENOMEM;
        }
        drm_mode_set_name(mode5);
        mode5->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
        drm_mode_probed_add(connector, mode5);

        mode6 = drm_mode_duplicate(connector->dev, &performance_mode_120hz);
        if (!mode6) {
                dev_info(connector->dev->dev, "failed to add mode7 %ux%ux@%u\n",
                         performance_mode_120hz.hdisplay, performance_mode_120hz.vdisplay,
                         drm_mode_vrefresh(&performance_mode_120hz));
                return -ENOMEM;
        }
        drm_mode_set_name(mode6);
        mode6->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
        drm_mode_probed_add(connector, mode6);

        connector->display_info.width_mm = 70;
        connector->display_info.height_mm = 156;

        return 1;
}

static const struct drm_panel_funcs jdi_drm_funcs = {
        .disable = jdi_disable,
        .unprepare = jdi_unprepare,
        .prepare = jdi_prepare,
        .enable = jdi_enable,
        .get_modes = jdi_get_modes,
};
static int lcd_vsn_reset_notify_callback(struct notifier_block *np, unsigned long type, void *_unused)
{
        switch (type) {
        case SYS_DOWN:
                shutdown_lcd_drv = 1;
                pr_info("[lcm] reboot_notify: SYS_DOWN!\n");
                break;
        case SYS_POWER_OFF:
                shutdown_lcd_drv = 1;
                pr_info("[lcm] lcd_vsn_reset_notify_callback, shutdown_lcd_drv = %d, reboot_notify: SYS_POWER_OFF!\n", shutdown_lcd_drv);
                break;

        case SYS_HALT:
                pr_info("[lcm] reboot_notify: SYS_HALT !\n");
                break;

        default:
                pr_info("[lcm] reboot_notify: default !\n");
                break;
        }
        return NOTIFY_OK;
}

static struct notifier_block lcd_vsn_reset_notifier = {
        .notifier_call = lcd_vsn_reset_notify_callback,
        .priority = 128,
};

static int jdi_probe(struct mipi_dsi_device *dsi)
{
        struct device *dev = &dsi->dev;
        struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;
        struct jdi *ctx;
        struct device_node *backlight;
        unsigned int lcm_degree;
        int ret;
        int probe_ret;

        pr_info("%s+ jdi_probe enter csot,td4377,vdo,120hz,6382\n", __func__);

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

        ctx = devm_kzalloc(dev, sizeof(struct jdi), GFP_KERNEL);
        if (!ctx)
                return -ENOMEM;

        mipi_dsi_set_drvdata(dsi, ctx);

        ctx->dev = dev;
        dsi->lanes = 4;
        dsi->format = MIPI_DSI_FMT_RGB888;
        dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
                        MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_NO_EOT_PACKET;

        backlight = of_parse_phandle(dev->of_node, "backlight", 0);
        if (backlight) {
                ctx->backlight = of_find_backlight_by_node(backlight);
                of_node_put(backlight);

                if (!ctx->backlight)
                        return -EPROBE_DEFER;
        }

        ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
        if (IS_ERR(ctx->reset_gpio)) {
                dev_info(dev, "cannot get reset-gpio %ld\n",
                         PTR_ERR(ctx->reset_gpio));
                return PTR_ERR(ctx->reset_gpio);
        }
        devm_gpiod_put(dev, ctx->reset_gpio);

        ctx->bias_pos = devm_gpiod_get_index(dev, "bias", 0, GPIOD_OUT_HIGH);
        if (IS_ERR(ctx->bias_pos)) {
                dev_info(dev, "cannot get bias-gpios 0 %ld\n",
                PTR_ERR(ctx->bias_pos));
                return PTR_ERR(ctx->bias_pos);
        }
        devm_gpiod_put(dev, ctx->bias_pos);

        ctx->bias_neg = devm_gpiod_get_index(dev, "bias", 1, GPIOD_OUT_HIGH);
        if (IS_ERR(ctx->bias_neg)) {
                dev_info(dev, "cannot get bias-gpios 1 %ld\n",
                PTR_ERR(ctx->bias_neg));
                return PTR_ERR(ctx->bias_neg);
        }
        devm_gpiod_put(dev, ctx->bias_neg);

	ctx->esd_te_gpio = devm_gpiod_get(ctx->dev, "esd-te", GPIOD_IN);
	if (IS_ERR(ctx->esd_te_gpio)) {
		dev_info(dev, "cannot get esd-te-gpios %ld\n",
			PTR_ERR(ctx->esd_te_gpio));
		return PTR_ERR(ctx->esd_te_gpio);
	}
	gpiod_direction_input(ctx->esd_te_gpio);

        ctx->prepared = true;
        ctx->enabled = true;
        drm_panel_init(&ctx->panel, dev, &jdi_drm_funcs, DRM_MODE_CONNECTOR_DSI);

        drm_panel_add(&ctx->panel);

        ret = mipi_dsi_attach(dsi);
        if (ret < 0)
                drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
        mtk_panel_tch_handle_reg(&ctx->panel);
        ret = mtk_panel_ext_create(dev, &ext_params_60hz, &ext_funcs, &ctx->panel);
        if (ret < 0)
                return ret;
        probe_ret = of_property_read_u32(dev->of_node, "lcm-degree", &lcm_degree);
        if (probe_ret < 0)
                lcm_degree = 0;
        else
                ext_params_60hz.lcm_degree = lcm_degree;
        pr_info("lcm_degree: %d\n", ext_params_60hz.lcm_degree);
#endif
	pr_info(" %s+ jdi_probe exit \n", __func__);
	init_global_exp_backlight();
	register_device_proc("lcd", "td4377_barley", "24605_csot_td4377");

	ctx->lcd_vsn_reset_nb = lcd_vsn_reset_notifier;
	register_reboot_notifier(&ctx->lcd_vsn_reset_nb);

	oplus_max_normal_brightness = MAX_NORMAL_BRIGHTNESS;
	return ret;
}

static int jdi_remove(struct mipi_dsi_device *dsi)
{
        struct jdi *ctx = mipi_dsi_get_drvdata(dsi);
#if defined(CONFIG_MTK_PANEL_EXT)
        struct mtk_panel_ctx *ext_ctx = find_panel_ctx(&ctx->panel);
#endif

        if (ext_ctx == NULL) {
                return 0;
        }
        mipi_dsi_detach(dsi);
        drm_panel_remove(&ctx->panel);
#if defined(CONFIG_MTK_PANEL_EXT)
        mtk_panel_detach(ext_ctx);
        mtk_panel_remove(ext_ctx);
#endif

        unregister_reboot_notifier(&ctx->lcd_vsn_reset_nb);

        return 0;
}

static const struct of_device_id jdi_of_match[] = {
        {
                .compatible = "oplus24605,csot,td4377,fhd,dsi,vdo",
        },
        {
        }
};

MODULE_DEVICE_TABLE(of, jdi_of_match);

static struct mipi_dsi_driver jdi_driver = {
        .probe = jdi_probe,
        .remove = jdi_remove,
        .driver = {
                .name = "oplus24605_csot_td4377_fhd_dsi_vdo",
                .owner = THIS_MODULE,
                .of_match_table = jdi_of_match,
        },
};

module_mipi_dsi_driver(jdi_driver);

MODULE_AUTHOR("shaohua deng <shaohua.deng@mediatek.com>");
MODULE_DESCRIPTION("JDI TD4377 VDO 120HZ LCD Panel Driver");
MODULE_LICENSE("GPL v2");

/********************************************
 ** Copyright (C) 2025 OPLUS Mobile Comm Corp. Ltd.
 ** OPLUS_EDIT
 ** File: ae152_p_d_a0032_hd_dsi_vdo.c
 ** Description: Source file for LCD driver To Control LCD driver
 ** Version : 1.0
 ** Date : 2025/05/30
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
#include <mtk_boot_common.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <soc/oplus/device_info.h>
#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#include "../mediatek/mediatek_v2/mtk_mipi_tx.h"
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
#include "ae152_p_d_a0032_hd_dsi_vdo.h"
#include "../bias/oplus23661_aw37501_bias.h"
/* add for ofp */
#include "../oplus/oplus_display_onscreenfingerprint.h"
#include <linux/reboot.h>

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mediatek/mediatek_v2/mtk_corner_pattern/ae152_p_d_a0032_hd_data_hw_roundedpattern.h"
#endif

extern unsigned int oplus_display_brightness;
extern unsigned int oplus_max_normal_brightness;
static bool aod_state = false;
static bool aod_off_dimming = false;
static bool aod_display_on = false;
extern unsigned int esd_enable;

#define MAX_NORMAL_BRIGHTNESS   3301
#define LOW_BACKLIGHT_LEVEL     8

#define LCD_CTL_TP_LOAD_FW 0x10
#define LCD_CTL_RST_ON  0x11
#define LCD_CTL_RST_OFF 0x12
#define LCD_CTL_CS_ON   0x19
#define LCD_CTL_CS_OFF  0x1A
#define LCD_CTL_IRQ_ON  0x1B
#define LCD_CTL_IRQ_OFF 0x1C

static int shutdown_lcd_drv = 0;
extern unsigned int backlight_level_esd;
/* default to launcher mode */
static int cabc_mode_backup = 3;
static int backlight_last_level = 1;
#define MIPI_DRIVER_VOLTAGE   0x6
#define OFF_CABC_LOW_BRIGHTNESS_LEVEL  396
int set_low_brightness_cabc_mode = 0;

static void lcm_cabc_mode_switch_to0(void *dsi, dcs_write_gce cb, \
		void *handle, unsigned int cabc_mode);
static void cabc_mode_switch(void *dsi, dcs_write_gce cb, void *handle, \
unsigned int cabc_mode);

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
	jdi_dcs_write_seq_static(ctx, 0xB0, 0x80);
	jdi_dcs_write_seq_static(ctx, 0xD6, 0x00);
	jdi_dcs_write_seq_static(ctx, 0xB8, 0x2d, 0x1d, 0x00, 0xbe, 0x00, 0x00);
	jdi_dcs_write_seq_static(ctx, 0xB9, 0x59, 0x39, 0x00, 0xbe, 0x00, 0x00);
	jdi_dcs_write_seq_static(ctx, 0xBA, 0x85, 0x55, 0x00, 0xbe, 0x00, 0x00);
        jdi_dcs_write_seq_static(ctx, 0xC0, 0x00, 0x33, 0x00, 0x14, 0x06, 0x22, 0x00, 0x00, 0x08);
        jdi_dcs_write_seq_static(ctx, 0xB6, 0x30, 0x73, 0x00, 0x86, 0xC3, 0x0b, 0xFF, 0xFF);
	jdi_dcs_write_seq_static(ctx, 0xCE, 0x77, 0x52, 0x66, 0x7a, 0x82, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0,
		0xc0, 0xc0, 0xc0, 0x00, 0xa7, 0x04, 0x04, 0x00, 0x04, 0x04, 0x62, 0x43, 0x69, 0x5a, 0x74);
        jdi_dcs_write_seq_static(ctx, 0xEB, 0x0F, 0x00, 0xF0);
        jdi_dcs_write_seq_static(ctx, 0xED, 0x00, 0x00, 0xF0);
	jdi_dcs_write_seq_static(ctx, 0xCF, 0x02);
	jdi_dcs_write_seq_static(ctx, 0x35, 0x00);
	jdi_dcs_write_seq_static(ctx, 0x51, 0x00, 0x00);
	jdi_dcs_write_seq_static(ctx, 0x53, 0x24);
	jdi_dcs_write_seq_static(ctx, 0x55, 0x03);
	jdi_dcs_write_seq_static(ctx, 0xB0, 0x83);
	jdi_dcs_write_seq_static(ctx, 0x11);
	usleep_range(150000, 150100);
	/*jdi_dcs_write_seq_static(ctx, 0x29);*/
	if (backlight_last_level) {
		jdi_dcs_write_seq_static(ctx, 0x29);
	        usleep_range(20000, 20100);
        }
	pr_info("%s-\n", __func__);
}

static void cabc_mode_retore(struct jdi *ctx)
{
	jdi_dcs_write_seq_static(ctx, 0x53, 0x2c);

	if(set_low_brightness_cabc_mode == 1) {
		jdi_dcs_write_seq_static(ctx, 0x55, 0x00);
		pr_info("%s()  set_low_brightness_cabc_mode ture, cabc_mode =%d return!\n",	__func__, cabc_mode_backup);
		return;
	}

	if (cabc_mode_backup == 1) {
		jdi_dcs_write_seq_static(ctx, 0x55, 0x01);
	} else if (cabc_mode_backup == 2) {
		jdi_dcs_write_seq_static(ctx, 0x55, 0x02);
	} else if (cabc_mode_backup == 3) {
		jdi_dcs_write_seq_static(ctx, 0x55, 0x03);
	} else if (cabc_mode_backup == 0) {
		jdi_dcs_write_seq_static(ctx, 0x55, 0x00);
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
	int blank = 0;

	if (!ctx->prepared) {
		return 0;
	}
        esd_enable = 0;
	aod_state = false;
	aod_display_on = false;

	if (tp_gesture_enable_notifier && tp_gesture_enable_notifier(0)) {
                if (shutdown_lcd_drv == 1) {
                        flag_poweroff = 1;
                } else {
                        flag_poweroff = 0;
                        pr_info("[TP] tp gesture  is enable,Display not to poweroff\n");
                }
	} else {
		flag_poweroff = 1;
	}

	pr_info("%s+ enter jdi_unprepare \n", __func__);
        usleep_range(20000, 20100);
	jdi_dcs_write_seq_static(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	usleep_range(67000, 67100);
	jdi_dcs_write_seq_static(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	usleep_range(150000, 150100);
	if (flag_poweroff == 1) {
		ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
		gpiod_set_value(ctx->reset_gpio, 0);
		devm_gpiod_put(ctx->dev, ctx->reset_gpio);

		usleep_range(2000, 2100);
		if (shutdown_lcd_drv == 1) {
			ctx->reset_gpio = devm_gpiod_get(ctx->dev, "tpreset", GPIOD_OUT_HIGH);
			gpiod_set_value(ctx->tpreset_gpio, 0);
			devm_gpiod_put(ctx->dev, ctx->tpreset_gpio);
			pr_info("[TP] g_shutdown_flag ==1, Display goto power off , And TP reset will low\n");
		} else {
			blank = LCD_CTL_RST_OFF;
			mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
			pr_info("[TP] tp gesture is disable, Display goto power off , And TP reset will low\n");
		}

		usleep_range(2000, 2100);
		ctx->bias_neg = devm_gpiod_get_index(ctx->dev, "bias", 1, GPIOD_OUT_HIGH);
		gpiod_set_value(ctx->bias_neg, 0);
		devm_gpiod_put(ctx->dev, ctx->bias_neg);

		usleep_range(2000, 2100);
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
        int blank = 0;

        if (ctx->prepared)
                return 0;
        pr_info("%s", __func__);

        blank = LCD_CTL_IRQ_OFF;
        mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
        pr_info("[TP]TP LCD_CTL_IRQ_OFF will chang to off\n");
        usleep_range(2000, 2100);
        blank = LCD_CTL_RST_ON;
        mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
        pr_info("[TP]TP reset will set high\n");
        usleep_range(2000, 2100);

        ctx->bias_pos = devm_gpiod_get_index(ctx->dev, "bias", 0, GPIOD_OUT_HIGH);
        gpiod_set_value(ctx->bias_pos, 1);
        devm_gpiod_put(ctx->dev, ctx->bias_pos);

        usleep_range(5000, 5100);
        ctx->bias_neg = devm_gpiod_get_index(ctx->dev, "bias", 1, GPIOD_OUT_HIGH);
        gpiod_set_value(ctx->bias_neg, 1);
        devm_gpiod_put(ctx->dev, ctx->bias_neg);
        usleep_range(5000, 5100);

        lcm_i2c_write_bytes(0x0, 0x14);
        lcm_i2c_write_bytes(0x1, 0x14);

        usleep_range(3000, 3100);
        ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
        gpiod_set_value(ctx->reset_gpio, 1);
        usleep_range(3000, 3100);
        gpiod_set_value(ctx->reset_gpio, 0);
        usleep_range(3000, 3100);
        gpiod_set_value(ctx->reset_gpio, 1);
        devm_gpiod_put(ctx->dev, ctx->reset_gpio);
        usleep_range(22000, 22100);



        mode = get_boot_mode();
        pr_info("[TP] in dis_panel_power_on,mode = %d\n", mode);
        if ((mode != MSM_BOOT_MODE__FACTORY) &&(mode != MSM_BOOT_MODE__RF) && (mode != MSM_BOOT_MODE__WLAN)) {
                blank = LCD_CTL_IRQ_ON;
                mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
                pr_info("[TP]TP LCD_CTL_IRQ_ON will chang to spi mode and high\n");
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

#define HFP (138)
#define HFP_144hz (44)
#define HSA (6)
#define HBP (30)
#define VSA (4)
#define VBP (16)
#define VAC (1570)
#define HAC (720)
#define VFP_45hz (3348)
#define VFP_60hz (2116)
#define VFP_90hz (880)
#define VFP_120hz (260)
#define VFP_144hz (136)
#define DYN_PLL_CLK (614)
#define DYN_DATA_RATE (1228)
#define HFP_DYN (160)
#define HFP_DYN_144hz (64)
#define PLL_CLOCK (599)
#define DATA_RATE (1198)
#define PHYSICAL_WIDTH  (71474)
#define PHYSICAL_HEIGHT (155853)

static const struct drm_display_mode performance_mode_60hz = {
        .clock = ((HAC + HFP + HSA + HBP) * (VAC + VFP_60hz + VSA + VBP) * 60) / 1000,
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
        .clock = ((HAC + HFP + HSA + HBP) * (VAC + VFP_90hz + VSA + VBP) * 90) / 1000,
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
        .clock = ((HAC + HFP + HSA + HBP) * (VAC + VFP_120hz + VSA + VBP) * 120) / 1000,
        .hdisplay = HAC,
        .hsync_start = HAC + HFP,
        .hsync_end = HAC + HFP + HSA,
        .htotal = HAC + HFP + HSA + HBP,
        .vdisplay = VAC,
        .vsync_start = VAC + VFP_120hz,
        .vsync_end = VAC + VFP_120hz + VSA,
        .vtotal = VAC + VFP_120hz + VSA + VBP,
};

static const struct drm_display_mode performance_mode_45hz = {
        .clock = ((HAC + HFP + HSA + HBP) * (VAC + VFP_45hz + VSA + VBP) * 45) / 1000,
        .hdisplay = HAC,
        .hsync_start = HAC + HFP,
        .hsync_end = HAC + HFP + HSA,
        .htotal = HAC + HFP + HSA + HBP,
        .vdisplay = VAC,
        .vsync_start = VAC + VFP_45hz,
        .vsync_end = VAC + VFP_45hz + VSA,
        .vtotal = VAC + VFP_45hz + VSA + VBP,
};

static const struct drm_display_mode performance_mode_144hz = {
        .clock = ((HAC + HFP_144hz + HSA + HBP) * (VAC + VFP_144hz + VSA + VBP) * 144) / 1000,
        .hdisplay = HAC,
        .hsync_start = HAC + HFP_144hz,
        .hsync_end = HAC + HFP_144hz + HSA,
        .htotal = HAC + HFP_144hz + HSA + HBP,
        .vdisplay = VAC,
        .vsync_start = VAC + VFP_144hz,
        .vsync_end = VAC + VFP_144hz + VSA,
        .vtotal = VAC + VFP_144hz + VSA + VBP,
};

#if defined(CONFIG_MTK_PANEL_EXT)

static struct mtk_panel_params ext_params_60hz = {
                .vendor = "A0032",
                .manufacture = "P_D",
                .pll_clk = PLL_CLOCK,
                .cust_esd_check = 1,
                .esd_check_enable = 1,
                .lcm_degree = PROBE_FROM_DTS,
                .lcm_esd_check_table[0] = {
                        .cmd = 0xA1,
                        .count = 1,
                        .para_list[0] = 0x9C,
                },
                .lcm_esd_check_table[1] = {
                        .cmd = 0x0A,
                        .count = 1,
                        .para_list[0] = 0x9C,
                },
                .ssc_enable = 0,
                .bdg_ssc_enable = 0,
                .lane_swap_en = 0,
                .bdg_lane_swap_en = 0,
                .physical_width_um = PHYSICAL_WIDTH,
                .physical_height_um = PHYSICAL_HEIGHT,
                .data_rate = DATA_RATE,
                .lfr_enable = 0,
                .lfr_minimum_fps = 60,
                .change_fps_by_vfp_send_cmd = 1,
                .dyn_fps = {
                                .switch_en = 1,
                                .vact_timing_fps = 120,
                                .dfps_cmd_table[0] = {0, 2 , {0xB0, 0x80}},
                                .dfps_cmd_table[1] = {0, 4 , {0xF0, 0xEB, 0x00, 0x0F}},
                                .dfps_cmd_table[2] = {0, 4 , {0xF0, 0xEB, 0x01, 0x00}},
                                .dfps_cmd_table[3] = {0, 4 , {0xF0, 0xEB, 0x02, 0xF0}},
                                .dfps_cmd_table[4] = {0, 4 , {0xF0, 0xED, 0x02, 0xF0}},
                                .dfps_cmd_table[5] = {0, 2 , {0xB0, 0x03}},
                },
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
                        .hs_trail = 12,
                },
                .oplus_display_global_dre = 1,
                .vdo_per_frame_lp_enable = 1,
                .oplus_display_lcd_tp_aod = 1,
                .oplus_display_color_mode_suppor = MTK_DRM_COLOR_MODE_DISPLAY_P3,
                .cabc_three_to_zero = 1,
                #ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
                .round_corner_en = 1,
                .corner_pattern_height = ROUND_CORNER_H_TOP,
                .corner_pattern_height_bot = ROUND_CORNER_H_BOT,
                .corner_pattern_tp_size = sizeof(top_rc_pattern),
                .corner_pattern_lt_addr = (void *)top_rc_pattern,
                 #endif
};

static struct mtk_panel_params ext_params_90hz = {
                .pll_clk = PLL_CLOCK,
                .vendor = "A0032",
                .manufacture = "P_D",
                .cust_esd_check = 1,
                .esd_check_enable = 1,
                .lcm_degree = PROBE_FROM_DTS,
                .lcm_esd_check_table[0] = {
                        .cmd = 0xA1,
                        .count = 1,
                        .para_list[0] = 0x9C,
                },
                .lcm_esd_check_table[1] = {
                        .cmd = 0x0A,
                        .count = 1,
                        .para_list[0] = 0x9C,
                },
                .ssc_enable = 0,
                .bdg_ssc_enable = 0,
                .lane_swap_en = 0,
                .bdg_lane_swap_en = 0,
                .physical_width_um = PHYSICAL_WIDTH,
                .physical_height_um = PHYSICAL_HEIGHT,
                .data_rate = DATA_RATE,
                .lfr_enable = 0,
                .lfr_minimum_fps = 60,
                .change_fps_by_vfp_send_cmd = 1,
                .dyn_fps = {
                                .switch_en = 1,
                                .vact_timing_fps = 120,
                                .dfps_cmd_table[0] = {0, 2 , {0xB0, 0x80}},
                                .dfps_cmd_table[1] = {0, 4 , {0xF0, 0xEB, 0x00, 0x0F}},
                                .dfps_cmd_table[2] = {0, 4 , {0xF0, 0xEB, 0x01, 0x00}},
                                .dfps_cmd_table[3] = {0, 4 , {0xF0, 0xEB, 0x02, 0xF0}},
                                .dfps_cmd_table[4] = {0, 4 , {0xF0, 0xED, 0x02, 0xF0}},
                                .dfps_cmd_table[5] = {0, 2 , {0xB0, 0x03}},
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
                        .hs_trail = 12,
                },
                .oplus_display_global_dre = 1,
                .vdo_per_frame_lp_enable = 1,
                .oplus_display_lcd_tp_aod = 1,
                .oplus_display_color_mode_suppor = MTK_DRM_COLOR_MODE_DISPLAY_P3,
                .cabc_three_to_zero = 1,
                #ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
                .round_corner_en = 1,
                .corner_pattern_height = ROUND_CORNER_H_TOP,
                .corner_pattern_height_bot = ROUND_CORNER_H_BOT,
                .corner_pattern_tp_size = sizeof(top_rc_pattern),
                .corner_pattern_lt_addr = (void *)top_rc_pattern,
                #endif
};

static struct mtk_panel_params ext_params_120hz = {
                .pll_clk = PLL_CLOCK,
                .vendor = "A0032",
                .manufacture = "P_D",
                .cust_esd_check = 1,
                .esd_check_enable = 1,
                .lcm_degree = PROBE_FROM_DTS,
                .lcm_esd_check_table[0] = {
                        .cmd = 0xA1,
                        .count = 1,
                        .para_list[0] = 0x9C,
                },
                .lcm_esd_check_table[1] = {
                        .cmd = 0x0A,
                        .count = 1,
                        .para_list[0] = 0x9C,
                },
                .ssc_enable = 0,
                .bdg_ssc_enable = 0,
                .lane_swap_en = 0,
                .bdg_lane_swap_en = 0,
                .physical_width_um = PHYSICAL_WIDTH,
                .physical_height_um = PHYSICAL_HEIGHT,
                .data_rate = DATA_RATE,
                .lfr_enable = 0,
                .lfr_minimum_fps = 60,
                .change_fps_by_vfp_send_cmd = 1,
                .dyn_fps = {
                                .switch_en = 1,
                                .vact_timing_fps = 120,
                                .dfps_cmd_table[0] = {0, 2 , {0xB0, 0x80}},
                                .dfps_cmd_table[1] = {0, 4 , {0xF0, 0xEB, 0x00, 0x0F}},
                                .dfps_cmd_table[2] = {0, 4 , {0xF0, 0xEB, 0x01, 0x00}},
                                .dfps_cmd_table[3] = {0, 4 , {0xF0, 0xEB, 0x02, 0xF0}},
                                .dfps_cmd_table[4] = {0, 4 , {0xF0, 0xED, 0x02, 0xF0}},
                                .dfps_cmd_table[5] = {0, 2 , {0xB0, 0x03}},
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
                        .hs_trail = 12,
                },
                .oplus_display_global_dre = 1,
                .vdo_per_frame_lp_enable = 1,
                .oplus_display_lcd_tp_aod = 1,
                .oplus_display_color_mode_suppor = MTK_DRM_COLOR_MODE_DISPLAY_P3,
                .cabc_three_to_zero = 1,
                #ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
                .round_corner_en = 1,
                .corner_pattern_height = ROUND_CORNER_H_TOP,
                .corner_pattern_height_bot = ROUND_CORNER_H_BOT,
                .corner_pattern_tp_size = sizeof(top_rc_pattern),
                .corner_pattern_lt_addr = (void *)top_rc_pattern,
                #endif
};

static struct mtk_panel_params ext_params_45hz = {
        .pll_clk = PLL_CLOCK,
        .vendor = "A0032",
        .manufacture = "P_D",
        .cust_esd_check = 1,
        .esd_check_enable = 1,
        .lcm_degree = PROBE_FROM_DTS,
        .lcm_esd_check_table[0] = {
                .cmd = 0xA1,
                .count = 1,
                .para_list[0] = 0x9C,
        },
        .lcm_esd_check_table[1] = {
                .cmd = 0x0A,
                .count = 1,
                .para_list[0] = 0x9C,
        },
        .ssc_enable = 0,
        .bdg_ssc_enable = 0,
        .lane_swap_en = 0,
        .bdg_lane_swap_en = 0,
        .physical_width_um = PHYSICAL_WIDTH,
        .physical_height_um = PHYSICAL_HEIGHT,
        .data_rate = DATA_RATE,
        .lfr_enable = 0,
        .lfr_minimum_fps = 60,
        .change_fps_by_vfp_send_cmd = 1,
        .dyn_fps = {
                        .switch_en = 1,
                        .vact_timing_fps = 120,
                        .dfps_cmd_table[0] = {0, 2 , {0xB0, 0x80}},
                        .dfps_cmd_table[1] = {0, 4 , {0xF0, 0xEB, 0x00, 0x0F}},
                        .dfps_cmd_table[2] = {0, 4 , {0xF0, 0xEB, 0x01, 0x00}},
                        .dfps_cmd_table[3] = {0, 4 , {0xF0, 0xEB, 0x02, 0xF0}},
                        .dfps_cmd_table[4] = {0, 4 , {0xF0, 0xED, 0x02, 0xF0}},
                        .dfps_cmd_table[5] = {0, 2 , {0xB0, 0x03}},
        },
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
                .hs_trail = 12,
        },
        .oplus_display_global_dre = 1,
        .vdo_per_frame_lp_enable = 1,
        .oplus_display_lcd_tp_aod = 1,
        .oplus_display_color_mode_suppor = MTK_DRM_COLOR_MODE_DISPLAY_P3,
        .cabc_three_to_zero = 1,
                #ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
                .round_corner_en = 1,
                .corner_pattern_height = ROUND_CORNER_H_TOP,
                .corner_pattern_height_bot = ROUND_CORNER_H_BOT,
                .corner_pattern_tp_size = sizeof(top_rc_pattern),
                .corner_pattern_lt_addr = (void *)top_rc_pattern,
                #endif
};

static struct mtk_panel_params ext_params_144hz = {
        .pll_clk = PLL_CLOCK,
        .vendor = "A0032",
        .manufacture = "P_D",
        .cust_esd_check = 1,
        .esd_check_enable = 1,
        .lcm_degree = PROBE_FROM_DTS,
        .lcm_esd_check_table[0] = {
                .cmd = 0xA1,
                .count = 1,
                .para_list[0] = 0x9C,
        },
        .lcm_esd_check_table[1] = {
                .cmd = 0x0A,
                .count = 1,
                .para_list[0] = 0x9C,
        },
        .ssc_enable = 0,
        .bdg_ssc_enable = 0,
        .lane_swap_en = 0,
        .bdg_lane_swap_en = 0,
        .physical_width_um = PHYSICAL_WIDTH,
        .physical_height_um = PHYSICAL_HEIGHT,
        .data_rate = DATA_RATE,
        .lfr_enable = 0,
        .lfr_minimum_fps = 60,
        .change_fps_by_vfp_send_cmd = 1,
        .dyn_fps = {
                        .switch_en = 1,
                        .vact_timing_fps = 144,
                        .dfps_cmd_table[0] = {0, 2 , {0xB0, 0x80}},
                        .dfps_cmd_table[1] = {0, 4 , {0xF0, 0xEB, 0x00, 0x07}},
                        .dfps_cmd_table[2] = {0, 4 , {0xF0, 0xEB, 0x01, 0xC0}},
                        .dfps_cmd_table[3] = {0, 4 , {0xF0, 0xEB, 0x02, 0x7C}},
                        .dfps_cmd_table[4] = {0, 4 , {0xF0, 0xED, 0x02, 0x7c}},
                        .dfps_cmd_table[5] = {0, 2 , {0xB0, 0x03}},
        },
        .dyn = {
                        .switch_en = 1,
                        .pll_clk = DYN_PLL_CLK,
                        .data_rate = DYN_DATA_RATE,
                        .vsa = VSA,
                        .vbp = VBP,
                        .vfp = VFP_144hz,
                        .hsa = HSA,
                        .hbp = HBP,
                        .hfp = HFP_DYN_144hz,
        },
        .phy_timcon = {
                .hs_trail = 12,
        },
        .oplus_display_global_dre = 1,
        .vdo_per_frame_lp_enable = 1,
        .oplus_display_lcd_tp_aod = 1,
        .oplus_display_color_mode_suppor = MTK_DRM_COLOR_MODE_DISPLAY_P3,
        .cabc_three_to_zero = 1,
                #ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
                .round_corner_en = 1,
                .corner_pattern_height = ROUND_CORNER_H_TOP,
                .corner_pattern_height_bot = ROUND_CORNER_H_BOT,
                .corner_pattern_tp_size = sizeof(top_rc_pattern),
                .corner_pattern_lt_addr = (void *)top_rc_pattern,
                #endif
};

static int panel_ata_check(struct drm_panel *panel)
{
        return 1;
}

static int map_exp[4096] = {0};

static void init_global_exp_backlight(void)
{
	int j;
	int max_hbm = 3710;
	int ud_level = 396;
	int ud_51 = 11;

	for (j = 1; j <= ud_level; j++) {
		map_exp[j] = ud_51;
	}

	for(j = ud_level + 1; j < max_hbm + ud_level - ud_51 + 1; j++) {
		map_exp[j] = (j - ud_level) + ud_51;
	}
}

static int boe_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle,
				 unsigned int level)
{
	char bl_tb0[] = {0x51, 0x0F, 0xFF};
	char bl_tb1[] = {0x53, 0x24};
	char bl_tb3[] = {0x29};
	char bl_tb4[] = {0x53, 0x2c};
	int bl_map;

	if (!cb)
		return -1;

	if ((aod_off_dimming == true) && (level > 0) && (backlight_last_level > 0)
		&& (aod_state == false)) {
		cb(dsi, handle, bl_tb4, ARRAY_SIZE(bl_tb4));
		aod_off_dimming = false;
	}

	if ((backlight_last_level == 0) && (level > 0)) {
		if (aod_off_dimming == false)
			aod_off_dimming = true;
		if (aod_display_on == true) {
			aod_display_on = false;
		} else {
			udelay(6000);
			cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));
                        usleep_range(20000, 20100);
			pr_info("%s,backlight on", __func__);
		}
	}

	if((0 < level) && (level <= OFF_CABC_LOW_BRIGHTNESS_LEVEL) && (set_low_brightness_cabc_mode == 0)) {
		lcm_cabc_mode_switch_to0(dsi, cb, handle, 0);
		set_low_brightness_cabc_mode = 1;
		pr_info("level <= %d ,set set_low_brightness_cabc_mode 0\n", OFF_CABC_LOW_BRIGHTNESS_LEVEL);
	} else if ((level > OFF_CABC_LOW_BRIGHTNESS_LEVEL) && (set_low_brightness_cabc_mode == 1)) {
		set_low_brightness_cabc_mode = 0;
		cabc_mode_switch(dsi, cb, handle, cabc_mode_backup);
		pr_info("level > %d ,cabc_status %d\n", OFF_CABC_LOW_BRIGHTNESS_LEVEL, cabc_mode_backup);
	}

	oplus_display_brightness = level;
	bl_map = level;
	backlight_level_esd = level;
	backlight_last_level = level;

	bl_map = map_exp[level];

	pr_info("%s, enter backlight level = %d, bl_map = %d, oplus_display_brightness = %d\n", __func__, level, bl_map, oplus_display_brightness);

	if ((aod_state == 1) && (level == 1000 || level == 1003 || level == 1080)) {
		backlight_last_level = 0;
		aod_display_on = true;
		return 0;
	}

	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT && bl_map > 0) {
		bl_map = 1638;
	}

	bl_tb0[1] = bl_map >> 8;
	bl_tb0[2] = bl_map & 0xFF;

	if (bl_map < LOW_BACKLIGHT_LEVEL) {
		cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	}

        if (!esd_enable && bl_map > 0) {
		esd_enable = 1;
		pr_info("%s, esd_enable", __func__);
	}
	if (bl_map == 0) {
		esd_enable = 0;
		aod_state = false;
		aod_display_on = false;
	}

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	pr_info("%s, p_d_a0032 exit backlight level = %d, bl_map = %d, oplus_display_brightness = %d, backlight_last_level=%d\n",
		__func__, level, bl_map, oplus_display_brightness, backlight_last_level);

	return 0;
}

static int panel_doze_disable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
        struct jdi *ctx = panel_to_jdi(panel);
        int mode;
        int blank;
        char bl_tb2[] = {0x55, 0x03};
        aod_state = false;

        pr_err("debug for lcm %s\n", __func__);

        if (!cb)
                return -1;

        cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
        aod_off_dimming = true;
        aod_state = false;

        if (!ctx->prepared) {
                return 0;
        }

        pr_err(" %s : AOD reset start\n", __func__);
        mode = get_boot_mode();
        pr_info("[TP] in dis_panel_power_on,mode = %d\n", mode);
        if ((mode != MSM_BOOT_MODE__FACTORY) &&(mode != MSM_BOOT_MODE__RF) && (mode != MSM_BOOT_MODE__WLAN)) {
                #define LCD_CTL_TP_LOAD_FW 0x10
                #define LCD_CTL_CS_ON  0x19
                #define LCD_CTL_AOD_OFF  0x30
                blank = LCD_CTL_CS_ON;
                mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
                pr_info("[TP]TP CS will chang to spi mode and high\n");
                usleep_range(5000, 5100);
              /* blank = LCD_CTL_TP_LOAD_FW;
                mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
                pr_info("[TP] start to load fw!\n");*/
                blank =  LCD_CTL_AOD_OFF;
                mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
                pr_info("[TP] EXIT AOD!\n");
        }
        pr_err(" %s : AOD reset end\n", __func__);
        return 0;
}

static int panel_doze_enable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
        char bl_tb0[] = {0x51, 0x0F, 0xFF};
        char bl_tb1[] = {0x53, 0x24};
        char bl_tb2[] = {0x55, 0x00};

        int level;
        /*50nit*/
        level = 208;
        aod_state = true;


        bl_tb0[1] = level >> 8;
        bl_tb0[2] = level & 0xFF;

        if (!cb)
                return -1;

        pr_err("debug for lcm %s\n", __func__);
        cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
        cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
        cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
        pr_info("%s, AOD backlight level = %d\n", __func__, level);
        return 0;
}

static int panel_set_aod_light_mode(void *dsi, dcs_write_gce cb, void *handle, unsigned int level)
{
        int backlight;
        char bl_tb0[] = {0x51, 0x0F, 0xFF};
        char bl_tb2[] = {0x55, 0x00};

        pr_err("debug for lcm %s+\n", __func__);

        if (level == 0) {
                backlight = 208;

                bl_tb0[1] = backlight >> 8;
                bl_tb0[2] = backlight & 0xFF;

                if (!cb)
                        return -1;

                cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
                cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
                pr_info("%s, AOD backlight backlight = %d\n", __func__, backlight);
        } else {
                /*10nit*/
                backlight = 39;

                bl_tb0[1] = backlight >> 8;
                bl_tb0[2] = backlight & 0xFF;

                if (!cb)
                        return -1;

                cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
                cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
                pr_info("%s, AOD backlight backlight = %d\n", __func__, backlight);
        }
        pr_err("debug for lcm %s- level = %d !\n", __func__, level);

        return 0;
}

static int oplus_esd_backlight_recovery(void *dsi, dcs_write_gce cb,
		void *handle)
{
	char bl_tb0[] = {0x51, 0x03, 0xff};
	char bl_tb1[] = {0x29};

	bl_tb0[1] = backlight_level_esd >> 8;
	bl_tb0[2] = backlight_level_esd & 0xFF;
	if (!cb)
		return -1;
	pr_err("%s bl_tb0[1]=%x, bl_tb0[2]=%x\n", __func__, bl_tb0[1], bl_tb0[2]);
	cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
        esd_enable = 1;
	pr_info("%s, esd_enable", __func__);

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
        } else if (target_fps == 45) {
                ext->params = &ext_params_45hz;
        }  else if (target_fps == 144) {
                ext->params = &ext_params_144hz;
        }  else {
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
                *ext_param = &ext_params_144hz;
        } else if (id == 1) {
                *ext_param = &ext_params_120hz;
        } else if (id == 2) {
                *ext_param = &ext_params_90hz;
        } else if (id == 3) {
                *ext_param = &ext_params_60hz;
        } else if (id == 4) {
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

static void lcm_cabc_mode_switch_to0(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int cabc_mode)
{
	char bl_tb1[] = {0x55, 0x00};

	cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	pr_info("%s [lcd_info]:cabc mode_%d, set cabc_para=%#x\n", __func__, cabc_mode, bl_tb1[1]);
}

static void cabc_mode_switch(void *dsi, dcs_write_gce cb, void *handle, unsigned int cabc_mode)
{
	char bl_tb0[] = {0x53, 0x2c};
	char bl_tb1[] = {0x55, 0x00};

	pr_err("%s cabc_mode = %d\n", __func__, cabc_mode);
	if (cabc_mode > 3) {
		pr_err("%s: Invaild params skiped!\n", __func__);
		return;
	}

	if(set_low_brightness_cabc_mode == 1) {
		cabc_mode_backup = cabc_mode;
		pr_info("%s() set_low_brightness_cabc_mode ture, cabc_mode =%d return!\n",	__func__, cabc_mode);
		return;
	}

	if (!backlight_last_level)
		bl_tb0[1] = 0x24;

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	if (cabc_mode == 1) {
		bl_tb1[1] = 0x01;
		cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	} else if (cabc_mode == 2) {
		bl_tb1[1] = 0x02;
		cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	} else if (cabc_mode == 3) {
		bl_tb1[1] = 0x03;
		cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	} else if (cabc_mode == 0) {
		bl_tb1[1] = 0x00;
		cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	}
	cabc_mode_backup = cabc_mode;
}
static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = boe_setbacklight_cmdq,
	.ext_param_set = mtk_panel_ext_param_set,
	.ext_param_get = mtk_panel_ext_param_get,
	.ata_check = panel_ata_check,
	.doze_enable = panel_doze_enable,
	.doze_disable = panel_doze_disable,
	.set_aod_light_mode = panel_set_aod_light_mode,
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
        struct drm_display_mode *mode5;
        struct drm_display_mode *mode6;
        struct drm_display_mode *mode7;
        struct drm_display_mode *mode8;

        mode7 = drm_mode_duplicate(connector->dev, &performance_mode_45hz);
        if (!mode7) {
                dev_info(connector->dev->dev, "failed to add mode7 %ux%ux@%u\n",
                         performance_mode_45hz.hdisplay, performance_mode_45hz.vdisplay,
                         drm_mode_vrefresh(&performance_mode_45hz));
                return -ENOMEM;
        }
        drm_mode_set_name(mode7);
        mode7->type = DRM_MODE_TYPE_DRIVER;
        drm_mode_probed_add(connector, mode7);

        mode = drm_mode_duplicate(connector->dev, &performance_mode_60hz);
        if (!mode) {
                dev_info(connector->dev->dev, "failed to add mode1 %ux%ux@%u\n",
                         performance_mode_60hz.hdisplay, performance_mode_60hz.vdisplay,
                         drm_mode_vrefresh(&performance_mode_60hz));
                return -ENOMEM;
        }

        drm_mode_set_name(mode);
        mode->type = DRM_MODE_TYPE_DRIVER;
        drm_mode_probed_add(connector, mode);

        mode5 = drm_mode_duplicate(connector->dev, &performance_mode_90hz);
        if (!mode5) {
                dev_info(connector->dev->dev, "failed to add mode5 %ux%ux@%u\n",
                         performance_mode_90hz.hdisplay, performance_mode_90hz.vdisplay,
                         drm_mode_vrefresh(&performance_mode_90hz));
                return -ENOMEM;
        }
        drm_mode_set_name(mode5);
        mode5->type = DRM_MODE_TYPE_DRIVER;
        drm_mode_probed_add(connector, mode5);

        mode6 = drm_mode_duplicate(connector->dev, &performance_mode_120hz);
        if (!mode6) {
                dev_info(connector->dev->dev, "failed to add mode6 %ux%ux@%u\n",
                         performance_mode_120hz.hdisplay, performance_mode_120hz.vdisplay,
                         drm_mode_vrefresh(&performance_mode_120hz));
                return -ENOMEM;
        }
        drm_mode_set_name(mode6);
        mode6->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
        drm_mode_probed_add(connector, mode6);

        mode8 = drm_mode_duplicate(connector->dev, &performance_mode_144hz);
        if (!mode8) {
                dev_info(connector->dev->dev, "failed to add mode6 %ux%ux@%u\n",
                         performance_mode_144hz.hdisplay, performance_mode_144hz.vdisplay,
                         drm_mode_vrefresh(&performance_mode_144hz));
                return -ENOMEM;
        }
        drm_mode_set_name(mode8);
        mode8->type = DRM_MODE_TYPE_DRIVER;
        drm_mode_probed_add(connector, mode8);

        connector->display_info.width_mm = 71;
        connector->display_info.height_mm = 155;

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
        unsigned int fp_type = 0x01;
        int ret;
        int probe_ret;

        pr_info("%s+ jdi_probe enter p_d_a0032, vdo,120hz\n", __func__);

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

        ctx->tpreset_gpio = devm_gpiod_get(dev, "tpreset", GPIOD_OUT_HIGH);
        if (IS_ERR(ctx->tpreset_gpio)) {
                dev_info(dev, "cannot get tpreset_gpio %ld\n",
                         PTR_ERR(ctx->tpreset_gpio));
                return PTR_ERR(ctx->tpreset_gpio);
        }
        devm_gpiod_put(dev, ctx->tpreset_gpio);

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

        /* check_is_bdg_support(dev); */
#if defined(CONFIG_MTK_PANEL_EXT)
        mtk_panel_tch_handle_reg(&ctx->panel);
        ret = mtk_panel_ext_create(dev, &ext_params_120hz, &ext_funcs, &ctx->panel);
        if (ret < 0)
                return ret;
        probe_ret = of_property_read_u32(dev->of_node, "lcm-degree", &lcm_degree);
        if (probe_ret < 0)
                lcm_degree = 0;
        else
                ext_params_120hz.lcm_degree = lcm_degree;
        pr_info("lcm_degree: %d\n", ext_params_120hz.lcm_degree);
#endif
	pr_info(" %s+ jdi_probe exit \n", __func__);
	init_global_exp_backlight();
	register_device_proc("lcd", "A0032", "P_D");

	ctx->lcd_vsn_reset_nb = lcd_vsn_reset_notifier;
	register_reboot_notifier(&ctx->lcd_vsn_reset_nb);

	oplus_ofp_set_fp_type(&fp_type);
	oplus_max_normal_brightness = MAX_NORMAL_BRIGHTNESS;
	mipi_volt = MIPI_DRIVER_VOLTAGE;
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
                .compatible = "ae152,p_d,a0032,hd,dsi,vdo",
        },
        {
        }
};

MODULE_DEVICE_TABLE(of, jdi_of_match);

static struct mipi_dsi_driver jdi_driver = {
        .probe = jdi_probe,
        .remove = jdi_remove,
        .driver = {
                .name = "ae152_p_d_a0032_hd_dsi_vdo",
                .owner = THIS_MODULE,
                .of_match_table = jdi_of_match,
        },
};

module_mipi_dsi_driver(jdi_driver);

MODULE_AUTHOR("shaohua deng <shaohua.deng@mediatek.com>");
MODULE_DESCRIPTION("P_D A0032 VDO 120HZ LCD Panel Driver");
MODULE_LICENSE("GPL v2");

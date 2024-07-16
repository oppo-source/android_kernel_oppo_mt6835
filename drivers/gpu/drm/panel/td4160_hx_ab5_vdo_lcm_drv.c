/***********************************************************
** Copyright (C), 2008-2016, OPPO Mobile Comm Corp., Ltd.
** File: td4160_hx_ab5_vdo_lcm_drv.c
** Description: source file for lcm ab5 in kernel stage
**
** Version: 1.0
** Date: 2023/12/29
** Author: display.lcd
**
** ------------------------------- Revision History: -------------------------------
**      <author>        <data>        <version >           <desc>
**       display        2023/12/29      1.0              source file for lcm ab5 in kernel stage
**
****************************************************************/

#include <linux/backlight.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_modes.h>
#include <linux/delay.h>
#include <drm/drm_connector.h>
#include <drm/drm_device.h>

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
#include "../mediatek/mediatek_v2/mtk_log.h"
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
#if IS_ENABLED(CONFIG_TOUCHPANEL_NOTIFY)
extern int (*tp_gesture_enable_notifier)(unsigned int tp_index);
#endif
static bool is_pd_with_guesture = false;
extern unsigned int g_shutdown_flag;


#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mediatek/mediatek_v2/mtk_corner_pattern/td4160_hx_ab5_data_hw_roundedpattern.h"
#endif
#include "include/td4160_hx_ab5_vdo_lcm_drv.h"
#include "../bias/oplus23661_aw37501_bias.h"
#include <linux/reboot.h>

#define MAX_NORMAL_BRIGHTNESS    (3311)

static int cabc_status = 1;
static int esd_brightness;
static unsigned char g_gammaflag = 0;
static int mode = 0;

extern unsigned int oplus_display_brightness;
extern unsigned int oplus_max_normal_brightness;
extern unsigned long esd_flag;
#define LCD_CTL_TP_LOAD_FW 0x10
#define LCD_CTL_RST_ON  0x11
#define LCD_CTL_RST_OFF 0x12
#define LCD_CTL_CS_ON  0x19
#define LCD_CTL_CS_OFF  0x1A


struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *bias_ldo, *bias_pos, *bias_neg;

	bool prepared;
	bool enabled;
	int error;
};

#define lcm_dcs_write_seq(ctx, seq...)                                     \
	({                                                                     \
		const u8 d[] = {seq};                                          \
		BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 128,                           \
				 "DCS sequence too big for stack");            \
		lcm_dcs_write(ctx, d, ARRAY_SIZE(d));                      \
	})

#define lcm_dcs_write_seq_static(ctx, seq...)                              \
	({                                                                     \
		static const u8 d[] = {seq};                                   \
		lcm_dcs_write(ctx, d, ARRAY_SIZE(d));                      \
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
	if (len > 1)
		 udelay(100);

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
			usleep_range(table[i].count * 1000, table[i].count * 1000 + 100);
			break;
		case REGFLAG_UDELAY:
			usleep_range(table[i].count, table[i].count + 1000);
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

static void lcm_init_set_cabc(struct lcm *ctx, int cabc_mode)
{
	pr_info("%s [lcm] init set cabc_mode %d\n", __func__, cabc_mode);
	if (cabc_mode == 0) {
		lcm_dcs_write_seq_static(ctx, 0x55, 0x03);
	} else if (cabc_mode == 1) {
		lcm_dcs_write_seq_static(ctx, 0x55, 0x01);
	} else if (cabc_mode == 2) {
		lcm_dcs_write_seq_static(ctx, 0x55, 0x02);
	} else if (cabc_mode == 3) {
		lcm_dcs_write_seq_static(ctx, 0x55, 0x02);
	} else {
		lcm_dcs_write_seq_static(ctx, 0x55, 0x03);
	}
}

static void lcm_panel_init(struct lcm *ctx)
{
	pr_info("%s+\n", __func__);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xB6, 0x30, 0x6b, 0x00, 0x86, 0xc3, 0x03);
	lcm_dcs_write_seq_static(ctx, 0xB7, 0x31, 0x00, 0x00, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xB8, 0x02, 0x4A, 0x06, 0x00, 0x04, 0x81);
	lcm_dcs_write_seq_static(ctx, 0xB9, 0x02, 0x4A, 0x18, 0x00, 0x09, 0xb4);
	lcm_dcs_write_seq_static(ctx, 0xBA, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xBB, 0x00, 0xb4, 0xa0);
	lcm_dcs_write_seq_static(ctx, 0xBC, 0x00, 0xb4, 0xa0);
	lcm_dcs_write_seq_static(ctx, 0xBD, 0x00, 0xb4, 0xa0);
	lcm_dcs_write_seq_static(ctx, 0xBE, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xC0, 0x00, 0x43, 0x24, 0x06, 0x44, 0x00, 0x0a, 0x06, 0x94, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xC1, 0x30, 0x11, 0x50, 0xfa, 0x01, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x40, 0x0f, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xC2, 0x04, 0x00, 0x3c, 0x12, 0x07, 0x10, 0x12, 0x10, 0x00, 0x04, 0x00, 0x3c, 0x12, 0x07, 0x10, 0x10, 0x10, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x04, 0x00, 0x3c, 0x10, 0x10, 0x07, 0x05, 0xc1, 0x00, 0xe0, 0x02, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x11, 0x00, 0xe0, 0x02, 0x10,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xC3, 0x04, 0x00, 0x3c, 0xc1, 0x01, 0x60, 0x0A, 0x00, 0x00, 0x00, 0x20, 0x00, 0x04, 0x00, 0x3c, 0xc1, 0x01, 0x60,
	0x0c, 0x00, 0x00, 0x00, 0x20, 0x02, 0x04, 0x00, 0x3c, 0xc1, 0x01, 0x60, 0x0c, 0x00, 0x00, 0x00, 0x20, 0x01, 0x04, 0x00, 0x3c, 0xc1, 0x01, 0x60,
	0x0a, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xC4, 0x00, 0x00, 0x00, 0x64, 0x03, 0x07, 0x21, 0x1F, 0x1D, 0x1B, 0x19, 0x17, 0x15, 0x13, 0x5D, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5D, 0x14, 0x16, 0x18, 0x1A, 0x1C, 0x1E, 0x20, 0x22, 0x08, 0x04,
	0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x1F, 0x00, 0x80, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55);
	lcm_dcs_write_seq_static(ctx, 0xC5, 0x08, 0x00, 0x00, 0x00, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xC6, 0x00, 0x00, 0x00, 0x00, 0x01, 0x22, 0x04, 0x22, 0x01, 0x00, 0x46, 0x00, 0x00, 0x00, 0x01, 0x00, 0x46, 0x00,
	0x01, 0x05, 0x01, 0x0b, 0x01, 0x35, 0xff, 0x0f, 0x06, 0xc5, 0x02, 0xc0, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x20, 0x00, 0x00, 0x00, 0xc0, 0x11, 0x00, 0x00, 0x00, 0x10, 0x10, 0x00,
	0x00, 0x00, 0x01, 0x00, 0x50, 0x00, 0x33, 0x03, 0x00, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xCB, 0x02, 0xd0, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x40, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff);
	lcm_dcs_write_seq_static(ctx, 0xCE, 0x1e, 0x40, 0x60, 0x88, 0x92, 0x98, 0xa4, 0xad, 0xb8, 0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0x07,
	0x0f, 0x00, 0x00, 0x00, 0x04, 0x8c);
	lcm_dcs_write_seq_static(ctx, 0xCF, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xD0, 0xc1, 0x50, 0x81, 0x66, 0x09, 0x90, 0x00, 0xd5, 0x92, 0x9f, 0x11, 0x3c, 0x06, 0x7e, 0x09, 0x08, 0xdc, 0x1b,
	0xf0, 0x06);
	lcm_dcs_write_seq_static(ctx, 0xD1, 0xd8, 0xd8, 0x1b, 0x33, 0x33, 0x17, 0x07, 0xbb, 0x55, 0x55, 0x55, 0x55, 0x00, 0x3b, 0x77, 0x07, 0x3b, 0x30,
	0x06, 0x72, 0x33, 0x13, 0x00, 0xd7, 0x0c, 0x55, 0x02, 0x00, 0x18, 0x70, 0x18, 0x77, 0x11, 0x11, 0x11, 0x20, 0x20);
	lcm_dcs_write_seq_static(ctx, 0xD2, 0x00, 0x00, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xD3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff,
	0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff,
	0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff,
	0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff,
	0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff,
	0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff,
	0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff);
	lcm_dcs_write_seq_static(ctx, 0xD4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xD7, 0x01, 0x00, 0x12, 0x12, 0x00, 0x43, 0x00, 0x18, 0x00, 0x43, 0x00, 0x18, 0x03, 0x83, 0x80, 0x85, 0x85, 0x85,
	0x87, 0x84, 0x45, 0x86, 0x87, 0x80, 0x88, 0x86, 0x89, 0x83, 0x83, 0x87, 0x84, 0x88, 0x8A, 0x0c, 0x0b, 0x0a, 0x0a, 0x0a, 0x07, 0x07, 0x06, 0x06,
	0x00, 0x08, 0x0a, 0x0a);
	lcm_dcs_write_seq_static(ctx, 0xD8, 0x40, 0x99, 0x26, 0xed, 0x16, 0x6c, 0x16, 0x6c, 0x16, 0x6c, 0x00, 0x14, 0x00, 0x14, 0x00, 0x14, 0x01, 0x0c,
	0x00, 0x00, 0x01, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xD9, 0x01, 0x02, 0x7f, 0x22, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xDD, 0x30, 0x06, 0x23, 0x65);
	lcm_dcs_write_seq_static(ctx, 0xDE, 0x00, 0x00, 0x00, 0x0f, 0xff, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xE6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xEA, 0x02, 0x07, 0x07, 0x04, 0x80, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x03, 0x23, 0x00, 0x07, 0x00, 0xed, 0x00,
	0xed, 0x00, 0xed, 0x01, 0x28, 0x01, 0x28, 0x00, 0x47, 0x00, 0x47, 0x00, 0x47, 0x01, 0x0f, 0x01, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xEB, 0x09, 0xf0, 0x9f, 0x00, 0x01, 0x01, 0x00, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xEC, 0x02, 0xd0, 0x01, 0x70, 0x73, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x02, 0x2a, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xED, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x67, 0x47, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x02, 0xb0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x10, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xEE, 0x05, 0x40, 0x01, 0x00, 0xc0, 0x0f, 0x00, 0xc0, 0x0f, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0xc0, 0x0f,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x03, 0x40, 0xc5, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x3f, 0x00, 0x00, 0x00,
	0x00, 0x05, 0xc5, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00,
	0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x03);

	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xD6, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x03);
	lcm_dcs_write_seq_static(ctx, 0x35, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x51, 0x00, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x53, 0x2C);
	lcm_dcs_write_seq_static(ctx, 0x55, 0x01);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x03);

	lcm_dcs_write_seq_static(ctx, 0x11, 0x00);
	usleep_range(72000, 72100);
	lcm_dcs_write_seq_static(ctx, 0x29, 0x00);
	usleep_range(22000, 22100);

	lcm_init_set_cabc(ctx, cabc_status);
	pr_info("%s-\n", __func__);
}

static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	pr_info("%s\n", __func__);
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
	if (!ctx->prepared)
		return 0;

	pr_info("%s enter+, send lcm off: esd_flag = %d\n", __func__, esd_flag);
	push_table(ctx, lcm_off_setting, sizeof(lcm_off_setting)/sizeof(struct LCM_setting_table));

	ctx->error = 0;
	ctx->prepared = false;
	pr_info("%s Successful-\n", __func__);
	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;
	int blank = 0;
	if (ctx->prepared)
		return 0;

	pr_info("%s enter:prepared = %d\n", __func__, ctx->prepared);

	lcm_panel_init(ctx);
	pr_info("[TP] in dis_panel_power_on,mode = %d\n", mode);
	if ((mode != MSM_BOOT_MODE__FACTORY) &&(mode != MSM_BOOT_MODE__RF) && (mode != MSM_BOOT_MODE__WLAN)) {
			blank = LCD_CTL_CS_ON;
			mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
			pr_err("[TP]TP CS will chang to spi mode and high\n");
			usleep_range(5000, 5100);
			blank = LCD_CTL_TP_LOAD_FW;
			mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
			pr_info("[TP] start to load fw!\n");
	}

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;
	pr_info("%s Successful-\n", __func__);
	return ret;
}

static int lcm_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	pr_info("%s begin+!\n", __func__);
	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = true;
	pr_info("%s begin-!\n", __func__);
	return 0;
}

static void lcm_gamma_enter (void *dsi, dcs_write_gce cb, void *handle)
{
	/*
	char bl_tb1[] = {0xB0, 0x00};
	char bl_tb2[] = {0xC7, 0x02, 0x20, 0x02, 0x29, 0x02, 0x3C, 0x02, 0x4E, 0x02, 0x6F, 0x02, 0x5D, 0x02, 0x12, 0x01,
	0xE3, 0x01, 0xC0, 0x01, 0xB7, 0x01, 0xB0, 0x01, 0xB1, 0x01, 0xBC, 0x01, 0xB2, 0x01, 0xBF, 0x02, 0x49, 0x02, 0x7B,
	0x02, 0xE8, 0x02, 0xFF, 0x02, 0x20, 0x02, 0x29, 0x02, 0x3C, 0x02, 0x4E, 0x02, 0x6F, 0x02, 0x5D, 0x02, 0x12, 0x01,
	0xE3, 0x01, 0xC0, 0x01, 0xB7, 0x01, 0xB0, 0x01, 0xB1, 0x01, 0xBC, 0x01, 0xB2, 0x01, 0xBF, 0x02, 0x49, 0x02, 0x7B,
	0x02, 0xE8, 0x02, 0xFF};
	cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
	*/
}

static void lcm_gamma_exit (void *dsi, dcs_write_gce cb, void *handle)
{
	/*
	char bl_tb1[] = {0xB0, 0x00};
	char bl_tb2[] = {0xC7, 0x00, 0x6C, 0x00, 0x9B, 0x00, 0xBB, 0x00, 0xF7, 0x01, 0x50, 0x01, 0x7E, 0x01, 0x72, 0x01,
	0x7C, 0x01, 0x7C, 0x01, 0x87, 0x01, 0x8E, 0x01, 0x96, 0x01, 0xA7, 0x01, 0x9E, 0x01, 0xB0, 0x02, 0x37, 0x02, 0x6F,
	0x02, 0xAF, 0x02, 0xDB, 0x00, 0x24, 0x00, 0x42, 0x00, 0xAF, 0x00, 0xF7, 0x01, 0x50, 0x01, 0x7E, 0x01, 0x72, 0x01,
	0x7C, 0x01, 0x7C, 0x01, 0x87, 0x01, 0x8E, 0x01, 0x96, 0x01, 0xA7, 0x01, 0x9E, 0x01, 0xB0, 0x02, 0x37, 0x02, 0x6F,
	0x02, 0x93, 0x02, 0x93};
	cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
	*/
}

static const struct drm_display_mode disp_mode_60HZ = {
	.clock = ((FRAME_WIDTH + HFP + HSA + HBP) * (FRAME_HEIGHT + MODE_60_VFP + VSA + VBP) * MODE_60_FPS) / 1000,
	.hdisplay = FRAME_WIDTH,
	.hsync_start = FRAME_WIDTH + HFP,
	.hsync_end = FRAME_WIDTH + HFP + HSA,
	.htotal = FRAME_WIDTH + HFP + HSA + HBP,
	.vdisplay = FRAME_HEIGHT,
	.vsync_start = FRAME_HEIGHT + MODE_60_VFP,
	.vsync_end = FRAME_HEIGHT + MODE_60_VFP + VSA,
	.vtotal = FRAME_HEIGHT + MODE_60_VFP + VSA + VBP,
};

static const struct drm_display_mode disp_mode_90HZ = {
	.clock = ((FRAME_WIDTH + HFP + HSA + HBP) * (FRAME_HEIGHT + MODE_90_VFP + VSA + VBP) * MODE_90_FPS) / 1000,
	.hdisplay = FRAME_WIDTH,
	.hsync_start = FRAME_WIDTH + HFP,
	.hsync_end = FRAME_WIDTH + HFP + HSA,
	.htotal = FRAME_WIDTH + HFP + HSA + HBP,
	.vdisplay = FRAME_HEIGHT,
	.vsync_start = FRAME_HEIGHT + MODE_90_VFP,
	.vsync_end = FRAME_HEIGHT + MODE_90_VFP + VSA,
	.vtotal = FRAME_HEIGHT + MODE_90_VFP + VSA + VBP,
};

static const struct drm_display_mode disp_mode_120HZ = {
	.clock = ((FRAME_WIDTH + HFP + HSA + HBP) * (FRAME_HEIGHT + MODE_120_VFP + VSA + VBP) * MODE_120_FPS) / 1000,
	.hdisplay = FRAME_WIDTH,
	.hsync_start = FRAME_WIDTH + HFP,
	.hsync_end = FRAME_WIDTH + HFP + HSA,
	.htotal = FRAME_WIDTH + HFP + HSA + HBP,
	.vdisplay = FRAME_HEIGHT,
	.vsync_start = FRAME_HEIGHT + MODE_120_VFP,
	.vsync_end = FRAME_HEIGHT + MODE_120_VFP + VSA,
	.vtotal = FRAME_HEIGHT + MODE_120_VFP + VSA + VBP,
};

static struct mtk_panel_params ext_params_60HZ = {/* 60hz */
	.pll_clk = MIPI_CLK,
	.data_rate = DATA_RATE,
	.vendor = "TD4160_HX_02",
	.manufacture = "60_02",
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A,
		.count = 1,
		.para_list[0] = 0x9C,
	},
	.ssc_enable = 0,
	.bdg_ssc_enable = 0,
	.lane_swap_en = 0,
	/*.ap_tx_keep_hs_during_vact = 1,*/
	.physical_width_um = PHYSICAL_WIDTH,
	.physical_height_um = PHYSICAL_HEIGHT,
	/*.lfr_enable = 0,
	.lfr_minimum_fps = 60,*/
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 60,
	},
	/* .phy_timcon = {
		.hs_trail = 9,
		.clk_trail = 10,
	}, */
	.ssc_enable = 0,
	.bdg_ssc_enable = 0,
	.dyn = {
		.switch_en = 1,
		.pll_clk = HOPPING_MIPI_CLK,
		.data_rate = HOPPING_DATA_RATE,
		.hbp = HOPPING_HBP,
	},
	.vdo_per_frame_lp_enable = 1,
	/* .cabc_three_to_zero = 1, */
/* #ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT */
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
/* #endif */
};

static struct mtk_panel_params ext_params_90HZ = {/* 90hz */
	.pll_clk = MIPI_CLK,
	.data_rate = DATA_RATE,
	.vendor = "TD4160_HX_02",
	.manufacture = "90_02",
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A,
		.count = 1,
		.para_list[0] = 0x9C,
	},
	.ssc_enable = 0,
	.bdg_ssc_enable = 0,
	.lane_swap_en = 0,
	/* .ap_tx_keep_hs_during_vact = 1, */
	.physical_width_um = PHYSICAL_WIDTH,
	.physical_height_um = PHYSICAL_HEIGHT,
	/*.lfr_enable = 0,
	.lfr_minimum_fps = 60,*/
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 90,
	},
	/*.phy_timcon = {
		.hs_trail = 9,
		.clk_trail = 10,
	},*/
	.dyn = {
		.switch_en = 1,
		.pll_clk = HOPPING_MIPI_CLK,
		.data_rate = HOPPING_DATA_RATE,
		.hbp = HOPPING_HBP,
	},
	.vdo_per_frame_lp_enable = 1,
	/* .cabc_three_to_zero = 1, */
/* #ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT */
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
/* #endif */
};

static struct mtk_panel_params ext_params_120HZ = {/* 120hz */
	.pll_clk = MIPI_CLK,
	.data_rate = DATA_RATE,
	.vendor = "TD4160_HX_02",
	.manufacture = "120_02",
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A,
		.count = 1,
		.para_list[0] = 0x9C,
	},
	.ssc_enable = 0,
	.bdg_ssc_enable = 0,
	.lane_swap_en = 0,
	/* .ap_tx_keep_hs_during_vact = 1, */
	.physical_width_um = PHYSICAL_WIDTH,
	.physical_height_um = PHYSICAL_HEIGHT,
	/*.lfr_enable = 0,
	.lfr_minimum_fps = 60,*/
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 120,
	},
	/*.phy_timcon = {
		.hs_trail = 9,
		.clk_trail = 10,
	},*/
	.dyn = {
		.switch_en = 1,
		.pll_clk = HOPPING_MIPI_CLK,
		.data_rate = HOPPING_DATA_RATE,
		.hbp = HOPPING_HBP,
	},
	.vdo_per_frame_lp_enable = 1,
	/* .cabc_three_to_zero = 1, */
/* #ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT */
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
/* #endif */
};

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb,
	void *handle, unsigned int level)
{
	char bl_tb0[] = {0x51, 0x03, 0xff};

	if (level > 4095) {
		level = 4095;
	}
	if (!cb)
		return -1;
	pr_info("%s+: level =  %d\n", __func__, level);

	if (level < 9 && level > 0 && g_gammaflag == 0) {
		g_gammaflag = 1;
		pr_info("%s: backlight < 9 enter gamma!\n", __func__);
		lcm_gamma_enter(dsi, cb, handle);
	} else if (level > 8 && g_gammaflag == 1) {
		g_gammaflag = 0;
		pr_info("%s: backlight > 8 exit gamma!\n", __func__);
		lcm_gamma_exit(dsi, cb, handle);
	} else if (level == 0) {
		g_gammaflag = 0;
	}

	if (level == 0) {
		bl_tb0[1] = 0;
		bl_tb0[2] = 0;
	} else {
		bl_tb0[1] = level >> 8;
		bl_tb0[2] = level & 0xFF;
	}

	pr_info("%s: backlight =%d,bl0=0x%x,bl1=0x%x\n", __func__, level, bl_tb0[1], bl_tb0[2]);
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	oplus_display_brightness = level;
	esd_brightness = level;
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
	int target_fps;
	struct drm_display_mode *m = get_mode_by_id(connector, mode);
	target_fps = drm_mode_vrefresh(m);

	if (target_fps== MODE_60_FPS) {
		ext->params = &ext_params_60HZ;
	} else if (target_fps == MODE_90_FPS) {
		ext->params = &ext_params_90HZ;
	} else if (target_fps == MODE_120_FPS) {
		ext->params = &ext_params_120HZ;
	} else {
		pr_err("[ %s : %d ] : No mode to set fps = %d \n", __func__ ,  __LINE__ , target_fps);
		ret = 1;
	}

	return ret;
}

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);
	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		pr_err("%s[error]: cannot get reset_gpio %ld\n", __func__,
			PTR_ERR(ctx->reset_gpio));
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
	int blank = 0;

	if (ctx->prepared)
		return 0;
	pr_info("%s: td4160_hx_ab5 lcm ctx->prepared %d\n", __func__, ctx->prepared);

	blank = LCD_CTL_RST_ON;
	mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
	pr_err("[TP]TP reset will set high\n");
	usleep_range(2000, 2100);
	blank = LCD_CTL_CS_ON;
	mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
	pr_err("[TP]TP CS will chang to spi mode and high\n");
	usleep_range(2000, 2100);

	ctx->bias_pos = devm_gpiod_get_index(ctx->dev, "bias", 0, GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_pos, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);

	usleep_range(5000, 5100);
	ctx->bias_neg = devm_gpiod_get_index(ctx->dev, "bias", 1, GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_neg, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);
	usleep_range(5000, 5100);

	/* set VSP voltage etc. 4.0+ (parameter 2)* 0.10 */
	ret = lcm_i2c_write_bytes(0x0, 0x14);
	pr_debug("%s:  aw37051_write_byte return value = %d\n", __func__, ret);
	usleep_range(1000, 1100);
	/* set VSN voltage etc. 4.0+ (parameter 2)* 0.10 */
	ret = lcm_i2c_write_bytes(0x1, 0x14);
	pr_debug("%s:  aw37051_write_byte return value = %d\n", __func__, ret);

	/* lcm reset 1-0-1 */
	usleep_range(3000, 3100);
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(3000, 3100);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(3000, 3100);
	gpiod_set_value(ctx->reset_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	usleep_range(22000, 22100);
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
	int blank = 0;
	int flag_poweroff = 1;
	if (ctx->prepared)
		return 0;

	pr_err("[TP] td4160_hx_ab5 g_shutdown_flag is %d, esd_flag is %d\n", g_shutdown_flag, esd_flag);
	if (tp_gesture_enable_notifier && tp_gesture_enable_notifier(0) && (g_shutdown_flag == 0)) {
		is_pd_with_guesture = true;
		flag_poweroff = 0;
		pr_err("[TP] tp gesture is enable,Display not to poweroff\n");
	} else {
		is_pd_with_guesture = false;
		flag_poweroff = 1;
		pr_err("[TP] set poweroff to 1\n");
	}
	if (flag_poweroff == 1) {
	pr_info("%s enter+, lcm ctx->prepared = %d, esd_flag = %d\n", __func__, ctx->prepared, esd_flag);
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	usleep_range(2000, 2100);

	blank = LCD_CTL_RST_OFF;
	mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
	pr_err("[TP] tp gesture is disable, Display goto power off , And TP reset will low\n");
	blank = LCD_CTL_CS_OFF;
	mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
	pr_err("[TP]TP CS will change to gpio mode and low\n");
	usleep_range(2000, 2100);

	ctx->bias_neg = devm_gpiod_get_index(ctx->dev, "bias", 1, GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_neg, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);

	usleep_range(2000, 2100);
	ctx->bias_pos = devm_gpiod_get_index(ctx->dev, "bias", 0, GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_pos, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);
	}
	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	pr_info("%s:Successful\n", __func__);

	return 0;
}

static int oplus_esd_backlight_recovery(void *dsi, dcs_write_gce cb,
		void *handle)
{
	unsigned char bl_tb0[] = {0x51, 0x03, 0xff};
	bl_tb0[1] = esd_brightness >> 8;
	bl_tb0[2] = esd_brightness & 0xFF;
	if (!cb)
		return -1;
	pr_debug("%s:esd_brightness=%d bl_tb0[1]=%x, bl_tb0[2]=%x\n", __func__, esd_brightness, bl_tb0[1], bl_tb0[2]);
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	return 1;
}

static void lcm_cabc_mode_switch(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int cabc_mode)
{
	unsigned char cabc_cmd_1[] = {0x55, 0x00};
	unsigned char cabc_cmd_2[] = {0x53, 0x2C};
	cabc_status = cabc_mode;

	pr_info("%s: begin+ cabc_mode = %d!\n", __func__, cabc_mode);
	if (0 == cabc_mode) {
		/* off mode */
		cabc_cmd_1[1] = 0x03;
	} else if (1 == cabc_mode) {
		/* ui image mode */
		cabc_cmd_1[1] = 0x01;
	} else if (2 == cabc_mode) {
		/* still image mode */
		cabc_cmd_1[1] = 0x02;
	} else if (3 == cabc_mode) {
		/* still image mode */
		cabc_cmd_1[1] = 0x02;
	} else {
		/* default mode */
		cabc_cmd_1[1] = 0x03;
		pr_err("%s [lcd_info]: cabc_mode=%d is not support, close cabc !\n", __func__, cabc_mode);
	}

	cb(dsi, handle, cabc_cmd_2, ARRAY_SIZE(cabc_cmd_2));
	cb(dsi, handle, cabc_cmd_1, ARRAY_SIZE(cabc_cmd_1));
	pr_info("%s [lcd_info]:cabc mode_%d, set cabc_para=%#x\n", __func__, cabc_mode, cabc_cmd_2[1]);
}

static struct mtk_panel_funcs ext_funcs = {
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.reset = panel_ext_reset,
	.panel_poweron = lcm_panel_poweron,
	.panel_poweroff = lcm_panel_poweroff,
	.ext_param_set = mtk_panel_ext_param_set,
	.cabc_switch = lcm_cabc_mode_switch,
	.esd_backlight_recovery = oplus_esd_backlight_recovery,
};

static int lcm_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	struct drm_display_mode *mode_60HZ;
	struct drm_display_mode *mode_90HZ;
	struct drm_display_mode *mode_120HZ;

	pr_info("%s begin+!\n", __func__);
	mode_60HZ = drm_mode_duplicate(connector->dev, &disp_mode_60HZ);
	if (!mode_60HZ) {
		pr_err("[error]failed to add mode_60HZ %ux%ux@%u\n",
			disp_mode_60HZ.hdisplay, disp_mode_60HZ.vdisplay,
			drm_mode_vrefresh(&disp_mode_60HZ));
		return -ENOMEM;
	}
	drm_mode_set_name(mode_60HZ);
	mode_60HZ->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode_60HZ);

	mode_90HZ = drm_mode_duplicate(connector->dev, &disp_mode_90HZ);
	if (!mode_90HZ) {
		pr_err("[error]failed to add mode %ux%ux@%u\n",
			disp_mode_90HZ.hdisplay,
			disp_mode_90HZ.vdisplay,
			drm_mode_vrefresh(&disp_mode_90HZ));
		return -ENOMEM;
	}
	drm_mode_set_name(mode_90HZ);
	mode_90HZ->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode_90HZ);

	mode_120HZ = drm_mode_duplicate(connector->dev, &disp_mode_120HZ);
	if (!mode_120HZ) {
		pr_err("[error]failed to add mode %ux%ux@%u\n",
			disp_mode_120HZ.hdisplay,
			disp_mode_120HZ.vdisplay,
			drm_mode_vrefresh(&disp_mode_120HZ));
		return -ENOMEM;
	}
	drm_mode_set_name(mode_120HZ);
	mode_120HZ->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode_120HZ);

	connector->display_info.width_mm = 70;
	connector->display_info.height_mm = 155;
	pr_info("%s Successful-\n", __func__);
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
	struct lcm *ctx;
	struct device_node *backlight;
	int ret;
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;
	pr_info("%s begin+!\n", __func__);

	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);
		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				pr_err("%s No panel connected,skip probe lcm\n", __func__);
				return -ENODEV;
			}
			pr_info("%s device_node name %s\n", __func__, remote_node->name);
		}
	}

	if (remote_node != dev->of_node) {
		pr_err("%s skip probe due to not current lcm\n", __func__);
		return -ENODEV;
	}

	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO |
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

	ctx->prepared = true;
	ctx->enabled = true;

	drm_panel_init(&ctx->panel, dev, &lcm_drm_funcs, DRM_MODE_CONNECTOR_DSI);
	ctx->panel.dev = dev;
	ctx->panel.funcs = &lcm_drm_funcs;

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params_60HZ, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;
#endif

	register_device_proc("lcd", "td4160_hx_01", "AB5_V02");
	oplus_max_normal_brightness = MAX_NORMAL_BRIGHTNESS;
	mode = get_boot_mode();
	pr_info("%s Successful-\n", __func__);
	return ret;
}

static int lcm_remove(struct mipi_dsi_device *dsi)
{
	struct lcm *ctx = mipi_dsi_get_drvdata(dsi);
#if defined(CONFIG_MTK_PANEL_EXT)
	struct mtk_panel_ctx *ext_ctx = find_panel_ctx(&ctx->panel);
	if (ext_ctx == NULL) {
			return 0;
	}
#endif

	pr_info("%s begin+!\n", __func__);
	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_detach(ext_ctx);
	mtk_panel_remove(ext_ctx);
#endif

	pr_info("%s exit-!\n", __func__);
	return 0;
}

static const struct of_device_id lcm_of_match[] = {
	{ .compatible = "td4160,hx,ab5,vdo,lcm", },
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "td4160_hx_ab5_vdo_lcm_drv",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("oplus.display.lcd");
MODULE_DESCRIPTION("td4160_hx_ab5_vdo_lcm_drv Panel Driver");
MODULE_LICENSE("GPL v2");

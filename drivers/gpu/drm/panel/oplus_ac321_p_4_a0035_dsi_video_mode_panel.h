/***********************************************************
** Copyright (C), 2008-2016, OPPO Mobile Comm Corp., Ltd.
** File: oplus_ac321_p_4_a0035_dsi_video_mode_panel.h
** Description: source file for lcm AC182 in kernel stage
**
** Version: 1.0
** Date: 2025/07/08
** Author: display.lcd
**
** ------------------------------- Revision History: -------------------------------
**      <author>        <data>        <version >           <desc>
**       display        2025/07/08      1.0              source file for lcm oplus_ac321_p_4_a0035_dsi_video_mode_panel in kernel stage
**
****************************************************************/

#ifndef AC321_P_4_A0035_VDO_LCM_DRV_H
#define AC321_P_4_A0035_VDO_LCM_DRV_H

#define REGFLAG_CMD                 0xFFFA
#define REGFLAG_DELAY               0xFFFC
#define REGFLAG_UDELAY              0xFFFB
#define REGFLAG_END_OF_TABLE        0xFFFD

#define FRAME_WIDTH                 720
#define FRAME_HEIGHT                1570
#define PHYSICAL_WIDTH              71474
#define PHYSICAL_HEIGHT             155853

#define HSA                         10
#define HFP                         20
#define HBP                         20
#define VSA                         4
#define VBP                         20
#define MIPI_CLK                    528
#define DATA_RATE                   1056
#define HOPPING_MIPI_CLK            599
#define HOPPING_DATA_RATE           1198
#define HOPPING_HBP                 20
/*Parameter setting for mode 0 Start*/
#define MODE_60_FPS                  60
#define MODE_60_VFP                  2210
/*Parameter setting for mode 0 End*/

/*Parameter setting for mode 1 Start*/
#define MODE_90_FPS                  90
#define MODE_90_VFP                  944
/*Parameter setting for mode 1 End*/

/*Parameter setting for mode 2 Start*/
#define MODE_120_FPS                  120
#define MODE_120_VFP                  310
/*Parameter setting for mode 2 End*/

struct LCM_setting_table {
	unsigned int cmd;
	unsigned int count;
	unsigned char para_list[128];
};

/* ------------------------- initial code start------------------------- */
static struct LCM_setting_table init_setting[] = {
	/* 11 dont reload */
	{REGFLAG_CMD, 4, {0xFF, 0x98, 0x83, 0x06}},
	{REGFLAG_CMD, 2, {0x07, 0xA4}},

	{REGFLAG_CMD, 4, {0xFF, 0x5A, 0xA5, 0x03}},
	{REGFLAG_CMD, 2, {0x83, 0x20}},
	{REGFLAG_CMD, 2, {0x84, 0x01}},

	{REGFLAG_CMD, 4, {0xFF, 0x5A, 0xA5, 0x00}},
	{REGFLAG_CMD, 2, {0x35, 0x00}},
	{REGFLAG_CMD, 3, {0x51, 0x00, 0x00}},
	{REGFLAG_CMD, 2, {0x53, 0x2C}},
	{REGFLAG_CMD, 2, {0x55, 0x01}},
	{REGFLAG_CMD, 3, {0x68, 0x04, 0x00}},
	{REGFLAG_CMD, 2, {0x11, 0x00}},

	{REGFLAG_DELAY, 80, {}},
	{REGFLAG_CMD, 2, {0x29, 0x00}},
	{REGFLAG_DELAY, 20, {}},
};
/* ------------------------- initial code end-------------------------- */

/* ------------------------- Display off sequence start --------------- */
static struct LCM_setting_table lcm_off_setting[] = {
	/* Delay 5ms */
	{REGFLAG_DELAY, 5, {}},
	/* Page 0 */
	{REGFLAG_CMD, 4, {0xFF, 0x98, 0x83, 0x00}},
	{REGFLAG_CMD, 2, {0x28, 0x00}},
	{REGFLAG_DELAY, 20, {}},
	{REGFLAG_CMD, 2, {0x10, 0x00}},
	{REGFLAG_DELAY, 80, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
/* ------------------------- Display off sequence end ---------------- */
#endif /* end of oplus_ac321_p_4_a0035_dsi_video_mode_panel */

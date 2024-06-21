/***********************************************************
** Copyright (C), 2008-2016, OPPO Mobile Comm Corp., Ltd.
** File: AC182_P_3_A0013_VDO_PANEL.h
** Description: source file for lcm AC182 in kernel stage
**
** Version: 1.0
** Date: 2023/12/29
** Author: display.lcd
**
** ------------------------------- Revision History: -------------------------------
**      <author>        <data>        <version >           <desc>
**       display        2023/12/29      1.0              source file for lcm AC182 in kernel stage
**
****************************************************************/

#ifndef AC182_P_3_A0013_VDO_LCM_DRV_H
#define AC182_P_3_A0013_VDO_LCM_DRV_H

#define REGFLAG_CMD                 0xFFFA
#define REGFLAG_DELAY               0xFFFC
#define REGFLAG_UDELAY              0xFFFB
#define REGFLAG_END_OF_TABLE        0xFFFD

#define FRAME_WIDTH                 720
#define FRAME_HEIGHT                1604
#define PHYSICAL_WIDTH              69401
#define PHYSICAL_HEIGHT             154610

#define HSA                         12
#define HFP                         20
#define HBP                         18
#define VSA                         2
#define VBP                         28
#define MIPI_CLK                    597
#define DATA_RATE                   1195
#define HOPPING_MIPI_CLK            600
#define HOPPING_DATA_RATE           1200
#define HOPPING_HBP                 20
/*Parameter setting for mode 0 Start*/
#define MODE_60_FPS                  60
#define MODE_60_VFP                  2670
/*Parameter setting for mode 0 End*/

/*Parameter setting for mode 1 Start*/
#define MODE_90_FPS                  90
#define MODE_90_VFP                  1235
/*Parameter setting for mode 1 End*/

/*Parameter setting for mode 2 Start*/
#define MODE_120_FPS                  120
#define MODE_120_VFP                  518
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
	{REGFLAG_CMD, 2, {0x3E, 0xE2}},

	/* GIP Setting */
	{REGFLAG_CMD, 4, {0xFF, 0x98, 0x83, 0x01}},
	{REGFLAG_CMD, 2, {0x00, 0x51}},
	{REGFLAG_CMD, 2, {0x01, 0x17}},
	{REGFLAG_CMD, 2, {0x02, 0x00}},
	{REGFLAG_CMD, 2, {0x03, 0x00}},
	{REGFLAG_CMD, 2, {0x04, 0xCD}},
	{REGFLAG_CMD, 2, {0x05, 0x17}},
	{REGFLAG_CMD, 2, {0x06, 0x00}},
	{REGFLAG_CMD, 2, {0x07, 0x00}},
	{REGFLAG_CMD, 2, {0x08, 0x8E}},
	{REGFLAG_CMD, 2, {0x09, 0x01}},
	{REGFLAG_CMD, 2, {0x0A, 0xF7}},
	{REGFLAG_CMD, 2, {0x0B, 0x00}},
	{REGFLAG_CMD, 2, {0x10, 0x10}},
	{REGFLAG_CMD, 2, {0x11, 0x00}},
	{REGFLAG_CMD, 2, {0x12, 0x00}},
	{REGFLAG_CMD, 2, {0x16, 0x8E}},
	{REGFLAG_CMD, 2, {0x17, 0x01}},
	{REGFLAG_CMD, 2, {0x18, 0x77}},
	{REGFLAG_CMD, 2, {0x19, 0x00}},
	{REGFLAG_CMD, 2, {0x1E, 0x10}},
	{REGFLAG_CMD, 2, {0x0C, 0x5D}},
	{REGFLAG_CMD, 2, {0x0D, 0x5D}},
	{REGFLAG_CMD, 2, {0x0E, 0x00}},
	{REGFLAG_CMD, 2, {0x0F, 0x00}},
	{REGFLAG_CMD, 2, {0x1A, 0x5D}},
	{REGFLAG_CMD, 2, {0x1B, 0x5D}},
	{REGFLAG_CMD, 2, {0x1C, 0x00}},
	{REGFLAG_CMD, 2, {0x1D, 0x00}},
	{REGFLAG_CMD, 2, {0x28, 0x52}},
	{REGFLAG_CMD, 2, {0x29, 0x89}},
	{REGFLAG_CMD, 2, {0x2A, 0x8A}},
	{REGFLAG_CMD, 2, {0x2B, 0x53}},

	/* FW_R */
	{REGFLAG_CMD, 2, {0x31, 0x07}},
	{REGFLAG_CMD, 2, {0x32, 0x2A}},
	{REGFLAG_CMD, 2, {0x33, 0x23}},
	{REGFLAG_CMD, 2, {0x34, 0x09}},
	{REGFLAG_CMD, 2, {0x35, 0x0D}},
	{REGFLAG_CMD, 2, {0x36, 0x11}},
	{REGFLAG_CMD, 2, {0x37, 0x13}},
	{REGFLAG_CMD, 2, {0x38, 0x15}},
	{REGFLAG_CMD, 2, {0x39, 0x17}},
	{REGFLAG_CMD, 2, {0x3A, 0x19}},
	{REGFLAG_CMD, 2, {0x3B, 0x1B}},
	{REGFLAG_CMD, 2, {0x3C, 0x1D}},
	{REGFLAG_CMD, 2, {0x3D, 0x1F}},
	{REGFLAG_CMD, 2, {0x3E, 0x22}},
	{REGFLAG_CMD, 2, {0x3F, 0x07}},
	{REGFLAG_CMD, 2, {0x40, 0x07}},
	{REGFLAG_CMD, 2, {0x41, 0x07}},
	{REGFLAG_CMD, 2, {0x42, 0x07}},
	{REGFLAG_CMD, 2, {0x43, 0x07}},
	{REGFLAG_CMD, 2, {0x44, 0x07}},
	{REGFLAG_CMD, 2, {0x45, 0x07}},
	{REGFLAG_CMD, 2, {0x46, 0x07}},
	/* FW_L */
	{REGFLAG_CMD, 2, {0x47, 0x07}},
	{REGFLAG_CMD, 2, {0x48, 0x2A}},
	{REGFLAG_CMD, 2, {0x49, 0x23}},
	{REGFLAG_CMD, 2, {0x4A, 0x08}},
	{REGFLAG_CMD, 2, {0x4B, 0x0C}},
	{REGFLAG_CMD, 2, {0x4C, 0x10}},
	{REGFLAG_CMD, 2, {0x4D, 0x12}},
	{REGFLAG_CMD, 2, {0x4E, 0x14}},
	{REGFLAG_CMD, 2, {0x4F, 0x16}},
	{REGFLAG_CMD, 2, {0x50, 0x18}},
	{REGFLAG_CMD, 2, {0x51, 0x1A}},
	{REGFLAG_CMD, 2, {0x52, 0x1C}},
	{REGFLAG_CMD, 2, {0x53, 0x1E}},
	{REGFLAG_CMD, 2, {0x54, 0x22}},
	{REGFLAG_CMD, 2, {0x55, 0x07}},
	{REGFLAG_CMD, 2, {0x56, 0x07}},
	{REGFLAG_CMD, 2, {0x57, 0x07}},
	{REGFLAG_CMD, 2, {0x58, 0x07}},
	{REGFLAG_CMD, 2, {0x59, 0x07}},
	{REGFLAG_CMD, 2, {0x5A, 0x07}},
	{REGFLAG_CMD, 2, {0x5B, 0x07}},
	{REGFLAG_CMD, 2, {0x5C, 0x07}},

	{REGFLAG_CMD, 2, {0xC3, 0x00}},
	{REGFLAG_CMD, 2, {0xD0, 0x01}},
	{REGFLAG_CMD, 2, {0xD1, 0x50}},
	{REGFLAG_CMD, 2, {0xD6, 0x80}},
	{REGFLAG_CMD, 2, {0xD3, 0x30}},
	{REGFLAG_CMD, 2, {0xF1, 0x40}},
	{REGFLAG_CMD, 2, {0xE0, 0x7E}},
	{REGFLAG_CMD, 2, {0xE2, 0x10}},
	{REGFLAG_CMD, 2, {0xE6, 0x22}},
	{REGFLAG_CMD, 2, {0xE7, 0x54}},
	{REGFLAG_CMD, 2, {0xEE, 0x14}},

	/* RTN. Internal VBP, Internal VFP */
	{REGFLAG_CMD, 4, {0xFF, 0x98, 0x83, 0x02}},
	{REGFLAG_CMD, 2, {0x01, 0x35}},
	{REGFLAG_CMD, 2, {0x06, 0x3D}},
	{REGFLAG_CMD, 2, {0x0A, 0x9B}},
	{REGFLAG_CMD, 2, {0x0C, 0x00}},
	{REGFLAG_CMD, 2, {0x0D, 0x20}},
	{REGFLAG_CMD, 2, {0x0E, 0x00}},
	{REGFLAG_CMD, 2, {0x39, 0x09}},
	{REGFLAG_CMD, 2, {0x3A, 0x20}},
	{REGFLAG_CMD, 2, {0x3B, 0x00}},
	{REGFLAG_CMD, 2, {0x3C, 0x76}},
	{REGFLAG_CMD, 2, {0xF0, 0x04}},
	{REGFLAG_CMD, 2, {0xF1, 0xCC}},
	{REGFLAG_CMD, 2, {0x29, 0x48}},
	{REGFLAG_CMD, 2, {0x2A, 0x48}},
	{REGFLAG_CMD, 2, {0x48, 0x01}},
	{REGFLAG_CMD, 2, {0x44, 0x68}},
	{REGFLAG_CMD, 2, {0x40, 0x45}},

	{REGFLAG_CMD, 4, {0xFF, 0x98, 0x83, 0x03}},
	{REGFLAG_CMD, 2, {0x83, 0x20}},
	{REGFLAG_CMD, 2, {0x84, 0x01}},
	{REGFLAG_CMD, 2, {0x86, 0x6C}},
	{REGFLAG_CMD, 2, {0x88, 0xE1}},
	{REGFLAG_CMD, 2, {0x89, 0xE8}},
	{REGFLAG_CMD, 2, {0x8A, 0xF0}},
	{REGFLAG_CMD, 2, {0x8B, 0xF7}},
	{REGFLAG_CMD, 2, {0x8C, 0xBF}},
	{REGFLAG_CMD, 2, {0x8D, 0xC5}},
	{REGFLAG_CMD, 2, {0x8E, 0xC8}},
	{REGFLAG_CMD, 2, {0x8F, 0xCE}},
	{REGFLAG_CMD, 2, {0x90, 0xD1}},
	{REGFLAG_CMD, 2, {0x91, 0xD6}},
	{REGFLAG_CMD, 2, {0x92, 0xDC}},
	{REGFLAG_CMD, 2, {0x93, 0xE3}},
	{REGFLAG_CMD, 2, {0x94, 0xED}},
	{REGFLAG_CMD, 2, {0x95, 0xFA}},
	{REGFLAG_CMD, 2, {0xAF, 0x18}},

	/* Power Setting */
	{REGFLAG_CMD, 4, {0xFF, 0x98, 0x83, 0x05}},
	{REGFLAG_CMD, 2, {0x03, 0x00}},
	{REGFLAG_CMD, 2, {0x04, 0xD7}},
	{REGFLAG_CMD, 2, {0x69, 0xA1}},
	{REGFLAG_CMD, 2, {0x6A, 0x97}},
	{REGFLAG_CMD, 2, {0x6D, 0x8D}},
	{REGFLAG_CMD, 2, {0x73, 0x93}},
	{REGFLAG_CMD, 2, {0x79, 0xC9}},
	{REGFLAG_CMD, 2, {0x7F, 0xBB}},
	{REGFLAG_CMD, 2, {0x68, 0x3E}},
	{REGFLAG_CMD, 2, {0x66, 0x33}},
	{REGFLAG_CMD, 2, {0x20, 0x22}},

	/* Resolution */
	{REGFLAG_CMD, 4, {0xFF, 0x98, 0x83, 0x06}},
	{REGFLAG_CMD, 2, {0x06, 0xA4}},
	{REGFLAG_CMD, 2, {0xD9, 0x1F}},
	{REGFLAG_CMD, 2, {0xC0, 0x44}},
	{REGFLAG_CMD, 2, {0xC1, 0x16}},
	{REGFLAG_CMD, 2, {0x0A, 0x50}},
	{REGFLAG_CMD, 2, {0x48, 0x05}},
	{REGFLAG_CMD, 2, {0x4D, 0x80}},
	{REGFLAG_CMD, 2, {0x4E, 0x40}},
	{REGFLAG_CMD, 2, {0xC7, 0x05}},

	/* OSC Auto Trim Setting */
	{REGFLAG_CMD, 4, {0xFF, 0x98, 0x83, 0x0B}},
	{REGFLAG_CMD, 2, {0x9A, 0x47}},
	{REGFLAG_CMD, 2, {0x9B, 0xC2}},
	{REGFLAG_CMD, 2, {0x9C, 0x05}},
	{REGFLAG_CMD, 2, {0x9D, 0x05}},
	{REGFLAG_CMD, 2, {0x9E, 0xC2}},
	{REGFLAG_CMD, 2, {0x9F, 0xC2}},
	{REGFLAG_CMD, 2, {0xAA, 0x22}},
	{REGFLAG_CMD, 2, {0xAB, 0xE0}},

	/* TP Setting */
	{REGFLAG_CMD, 4, {0xFF, 0x98, 0x83, 0x0E}},
	{REGFLAG_CMD, 2, {0x11, 0x50}},
	{REGFLAG_CMD, 2, {0x12, 0x02}},
	{REGFLAG_CMD, 2, {0x13, 0x14}},
	{REGFLAG_CMD, 2, {0x00, 0xA0}},

	{REGFLAG_CMD, 4, {0xFF, 0x98, 0x83, 0x00}},
	{REGFLAG_CMD, 2, {0x35, 0x00}},
	{REGFLAG_CMD, 3, {0x51, 0x00, 0x00}},
	{REGFLAG_CMD, 2, {0x53, 0x24}},
	{REGFLAG_CMD, 2, {0x11, 0x00}},
	{REGFLAG_DELAY, 120, {}},
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
#endif /* end of AC182_P_3_A0013_VDO_LCM_DRV_H */

/***********************************************************
** Copyright (C), 2008-2016, OPPO Mobile Comm Corp., Ltd.
** File: td4160_hx_ab5_vdo_lcm_drv.h
** Description: source file for lcm AC182 in kernel stage
**
** Version: 1.0
** Date: 2023/12/29
** Author: display.lcd
**
** ------------------------------- Revision History: -------------------------------
**      <author>        <data>        <version >           <desc>
**       display        2023/12/29      1.0              source file for lcm td4160_hx_ab5_vdo_lcm_drv in kernel stage
**
****************************************************************/

#ifndef TD4160_HX_AB5_VDO_LCM_DRV_H
#define TD4160_HX_AB5_VDO_LCM_DRV_H

#define REGFLAG_CMD                 0xFFFA
#define REGFLAG_DELAY               0xFFFC
#define REGFLAG_UDELAY              0xFFFB
#define REGFLAG_END_OF_TABLE        0xFFFD

#define FRAME_WIDTH                 720
#define FRAME_HEIGHT                1604
#define PHYSICAL_WIDTH              69401
#define PHYSICAL_HEIGHT             154610

#define HSA                         6
#define HFP                         84
#define HBP                         100
#define VSA                         4
#define VBP                         32
#define MIPI_CLK                    597
#define DATA_RATE                   1195
#define HOPPING_MIPI_CLK            600
#define HOPPING_DATA_RATE           1200
#define HOPPING_HBP                 103
/*Parameter setting for mode 0 Start*/
#define MODE_60_FPS                  60
#define MODE_60_VFP                  2000
/*Parameter setting for mode 0 End*/

/*Parameter setting for mode 1 Start*/
#define MODE_90_FPS                  90
#define MODE_90_VFP                  786
/*Parameter setting for mode 1 End*/

/*Parameter setting for mode 2 Start*/
#define MODE_120_FPS                  120
#define MODE_120_VFP                  180
/*Parameter setting for mode 2 End*/

struct LCM_setting_table {
	unsigned int cmd;
	unsigned int count;
	unsigned char para_list[128];
};

/* ------------------------- Display off sequence start --------------- */
static struct LCM_setting_table lcm_off_setting[] = {
	/* Delay 5ms */
	{REGFLAG_DELAY, 5, {}},
	/* Page 0 */
	{REGFLAG_CMD, 2, {0x28, 0x00}},
	{REGFLAG_DELAY, 20, {}},
	{REGFLAG_CMD, 2, {0x10, 0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
/* ------------------------- Display off sequence end ---------------- */
#endif /* end of td4160_hx_ab5_vdo_lcm_drv */

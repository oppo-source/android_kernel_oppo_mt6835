#ifndef __PANEL_AE211_P_3_A0027_DSI_VDO_H__
#define __PANEL_AE211_P_3_A0027_DSI_VDO_H__

#define REGFLAG_CMD				0xFFFA
#define REGFLAG_DELAY			0xFFFC
#define REGFLAG_UDELAY			0xFFFB
#define REGFLAG_END_OF_TABLE	0xFFFD

#define BRIGHTNESS_HALF         3515
#define BRIGHTNESS_MAX          4094

enum MODE_ID {
	FHD_SDC60 = 0,
	FHD_SDC90 = 1,
	FHD_SDC120 = 2,
	FHD_SDC30 = 3,
};

struct ba {
	u32 brightness;
	u32 alpha;
};

struct LCM_setting_table {
	unsigned int cmd;
	unsigned int count;
	unsigned char para_list[256];
};

//LHBM
static struct LCM_setting_table lcm_finger_HBM_on_above_70nit[] = {
	{REGFLAG_CMD, 9, {0xAE, 0x82, 0x1C, 0xA0, 0x08, 0x00, 0x80, 0x08, 0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_finger_HBM_on_below_70nit[] = {
	{REGFLAG_CMD, 9, {0xAE, 0x82, 0x1C, 0xA0, 0x08, 0x00, 0x80, 0x08, 0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_finger_HBM_off_above_70nit[] = {
	/* HBM Mode OFF */
	{REGFLAG_CMD, 9, {0xAE, 0x02, 0x1C, 0xA0, 0x08, 0x00, 0x80, 0x08, 0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_finger_HBM_off_below_70nit[] = {
	/* HBM Mode OFF */
	{REGFLAG_CMD, 9, {0xAE, 0x02, 0x1C, 0xA0, 0x08, 0x00, 0x80, 0x08, 0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

/* -------------------------doze mode setting start------------------------- */
static struct LCM_setting_table AOD_off_setting[] = {
       // {REGFLAG_CMD, 1, {0x38}},  //30hz
        {REGFLAG_CMD, 2, {0x90, 0x40}},
};

static struct LCM_setting_table AOD_on_setting[] = {
	{REGFLAG_CMD, 2, {0x90, 0x41}},
	{REGFLAG_CMD, 2, {0xB0, 0x02}},
	{REGFLAG_CMD, 3, {0x51, 0x0F,0xFE}},
};

static struct LCM_setting_table aod_high_bl_level[] = {
	{REGFLAG_CMD, 2, {0xB0, 0x02}},
	{REGFLAG_CMD, 3, {0x51, 0x0F,0xFE}},
};

static struct LCM_setting_table aod_low_bl_level[] = {
	{REGFLAG_CMD, 2, {0xB0, 0x02}},
	{REGFLAG_CMD, 3, {0x51, 0x00,0x03}},
};
/* -------------------------doze mode setting end------------------------- */

//Demura
static struct LCM_setting_table lcm_set_demura_offset1[] = {
        {REGFLAG_CMD, 3, {0xF0, 0x5A, 0x5A}},
        {REGFLAG_CMD, 2, {0xB0, 0x31}},
        {REGFLAG_CMD, 2, {0xB6, 0x96}},
        {REGFLAG_CMD, 2, {0xB0, 0x3D}},
        {REGFLAG_CMD, 2, {0xB6, 0x96}},
        {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_set_demura_offset2[] = {
        {REGFLAG_CMD, 3, {0xF0, 0x5A, 0x5A}},
        {REGFLAG_CMD, 2, {0xB0, 0x31}},
        {REGFLAG_CMD, 2, {0xB6, 0x32}},
        {REGFLAG_CMD, 2, {0xB0, 0x3D}},
        {REGFLAG_CMD, 2, {0xB6, 0x32}},
        {REGFLAG_END_OF_TABLE, 0x00, {}}
};

//APL
struct LCM_setting_table dsi_switch_hbm_apl_on[] = {
	{REGFLAG_CMD, 3, {0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD, 2, {0x53, 0x21}},
	{REGFLAG_CMD, 2, {0xC9, 0x00}},
	{REGFLAG_CMD, 2, {0xCD, 0x01}},
	{REGFLAG_CMD, 3, {0xF0, 0xA5, 0xA5}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

struct LCM_setting_table dsi_switch_hbm_apl_off[] = {
	{REGFLAG_CMD, 3, {0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD, 2, {0x53, 0x20}},
	{REGFLAG_CMD, 2, {0xC9, 0x9F}},
	{REGFLAG_CMD, 2, {0xCD, 0x02}},
	{REGFLAG_CMD, 3, {0xF0, 0xA5, 0xA5}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

//HBM

struct LCM_setting_table hbm_on_cmd[] = {
	{REGFLAG_CMD, 3, {0x51,0x0F,0xFF}},
};

struct LCM_setting_table hbm_off_cmd[] = {
	{REGFLAG_CMD, 3, {0x51,0x0D,0xBB}},
};

/* -------------------------frame mode switch start------------------------- */
static struct LCM_setting_table mode_switch_to_60[] = {
    {REGFLAG_CMD, 3, {0xF0,0x5A,0x5A}},
    {REGFLAG_CMD, 2, {0x60,0x21}},
    {REGFLAG_CMD, 2, {0xF7,0x0B}},
    {REGFLAG_CMD, 3, {0xF0,0xA5,0xA5}},
};

static struct LCM_setting_table mode_switch_to_120[] = {
    {REGFLAG_CMD, 3, {0xF0,0x5A,0x5A}},
    {REGFLAG_CMD, 2, {0x60,0x01}},
    {REGFLAG_CMD, 2, {0xF7,0x0B}},
    {REGFLAG_CMD, 3, {0xF0,0xA5,0xA5}},
};
/* -------------------------frame mode switch end------------------------- */
#endif

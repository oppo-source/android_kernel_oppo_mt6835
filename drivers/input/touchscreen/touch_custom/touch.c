/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2018-2020 . All rights reserved.
 */
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/serio.h>
#include <linux/regulator/consumer.h>
#ifndef CONFIG_REMOVE_OPLUS_FUNCTION
#include <soc/oplus/system/oplus_project.h>
#endif
#include "touch.h"

#define MAX_CMDLINE_PARAM_LEN 512
char tp_dsi_display_primary[MAX_CMDLINE_PARAM_LEN];
char tp_dsi_display_secondary[MAX_CMDLINE_PARAM_LEN];

EXPORT_SYMBOL(tp_dsi_display_primary);
EXPORT_SYMBOL(tp_dsi_display_secondary);

#define MAX_LIMIT_DATA_LENGTH         100
int g_tp_prj_id = 0;
int g_tp_dev_vendor = TP_UNKNOWN;
int j = 0;
char *chip_name = NULL;
/*if can not compile success, please update vendor/oplus_touchsreen*/
struct tp_dev_name tp_dev_names[] = {
	{TP_OFILM, "OFILM"},
	{TP_BIEL, "BIEL"},
	{TP_TRULY, "TRULY"},
	{TP_BOE, "BOE"},
	{TP_G2Y, "G2Y"},
	{TP_TPK, "TPK"},
	{TP_JDI, "JDI"},
	{TP_TIANMA, "TIANMA"},
	{TP_SAMSUNG, "SAMSUNG"},
	{TP_DSJM, "DSJM"},
	{TP_BOE_B8, "BOEB8"},
	{TP_INNOLUX, "INNOLUX"},
	{TP_HIMAX_DPT, "DPT"},
	{TP_AUO, "AUO"},
	{TP_DEPUTE, "DEPUTE"},
	{TP_HUAXING, "HUAXING"},
	{TP_HLT, "HLT"},
	{TP_DJN, "DJN"},
	{TP_VXN, "VXN"},
	{TP_TXD, "TXD"},
	{TP_TOKEN, "TOKEN"},
	{TP_UNKNOWN, "UNKNOWN"},
};
typedef enum {
	TP_INDEX_NULL,
	SAMSUNG_Y791,
	SAMSUNG_Y792,
	BOE_S3908,
	SAMSUNG_Y771
} TP_USED_INDEX;
TP_USED_INDEX tp_used_index  = TP_INDEX_NULL;


#define GET_TP_DEV_NAME(tp_type) ((tp_dev_names[tp_type].type == (tp_type))?tp_dev_names[tp_type].name:"UNMATCH")

bool tp_judge_ic_match(char *tp_ic_name)
{
	pr_err("[TP] tp_ic_name = %s \n", tp_ic_name);
	pr_err("[TP] tp_dsi_display_primary = %s \n", tp_dsi_display_primary);
	if (strstr(tp_dsi_display_primary, tp_ic_name)) {
		pr_err("[TP] tp_judge_ic_match match ok\n");
		return true;
	}
	pr_err("[TP] tp_judge_ic_match not match ok\n");
	return false;

}
EXPORT_SYMBOL(tp_judge_ic_match);

int tp_judge_ic_match_commandline(struct panel_info *panel_data)
{
	int prj_id = 0;
	int i = 0;
	prj_id = get_project();

	pr_err("[TP] tp_dsi_display_primary = %s \n", tp_dsi_display_primary);
	for(i = 0; i < panel_data->project_num; i++) {
		if(prj_id == panel_data->platform_support_project[i]) {
			g_tp_prj_id = panel_data->platform_support_project_dir[i];
			pr_err("[TP] Driver match support project [%d]\n", panel_data->platform_support_project[i]);

			for(j = 0; j < panel_data->panel_num; j++) {
				if(strstr(tp_dsi_display_primary, panel_data->platform_support_commandline[j]) || strstr("default_commandline", panel_data->platform_support_commandline[j])) {
					panel_data->tp_type = panel_data->panel_type[j];
					if(panel_data->chip_num > 1) {
						chip_name = panel_data->chip_name[j];
						pr_err("[TP] WGL--1 chip_name = %s, panel_data->chip_name = %s", chip_name, panel_data->chip_name[j]);
					}
					pr_err("[TP] match panel type OK , panel type is [%d]\n", panel_data->tp_type);
					return j;
				}
				pr_err("[TP] Panel not found\n");
			}
		}
	}
	pr_err("[TP] Driver does not match the project\n");
	return -1;
}
EXPORT_SYMBOL(tp_judge_ic_match_commandline);

#ifndef CONFIG_REMOVE_OPLUS_FUNCTION
int tp_util_get_vendor(struct hw_resource *hw_res, struct panel_info *panel_data)
{
	char *vendor;
	int prj_id = 0;
	panel_data->test_limit_name = kzalloc(MAX_LIMIT_DATA_LENGTH, GFP_KERNEL|GFP_DMA);
	if (panel_data->test_limit_name == NULL) {
		pr_err("[TP]panel_data.test_limit_name kzalloc error\n");
	}
	prj_id = g_tp_prj_id;

	if (panel_data->tp_type == TP_UNKNOWN) {
		pr_err("[TP]%s type is unknown\n", __func__);
		return 0;
	}
	if (panel_data->firmware_name[j]) {
		memcpy(panel_data->manufacture_info.version, panel_data->firmware_name[j], strlen(panel_data->firmware_name[j]));
		panel_data->vid_len = strlen(panel_data->firmware_name[j]);
	}

	vendor = GET_TP_DEV_NAME(panel_data->tp_type);
	if(panel_data->chip_num == 1) {
		chip_name = panel_data->chip_name[0];
	}
	strcpy(panel_data->manufacture_info.manufacture, vendor);
	snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
		"tp/%d/FW_%s_%s.img",
		g_tp_prj_id, chip_name, vendor);

	if (panel_data->test_limit_name) {
		snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
			"tp/%d/LIMIT_%s_%s.img",
			g_tp_prj_id, chip_name, vendor);
	}

	if ((prj_id == 24683) || (prj_id == 24601) || (prj_id == 24615) || (prj_id == 24617) || (prj_id == 24722) || (prj_id == 24723) || (prj_id == 24727) || (prj_id == 24745) || (prj_id == 24746) || (prj_id == 24747)) {
		if (strstr(tp_dsi_display_primary, "oplus24601_hkc_ili9883c_hd_dsi_vdo_lcm_drv-nf_ili7807s")) {
			pr_err("[TP]project is rado-HD 1\n");
			strcpy(panel_data->manufacture_info.manufacture, vendor);
			memcpy(panel_data->manufacture_info.version, "0xAB879HK00", 11);
			panel_data->firmware_headfile.firmware_data = FW_24601_ILI7807S_HKC;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_24601_ILI7807S_HKC);
		} else if (strstr(tp_dsi_display_primary, "oplus24601_djn_icnl9916ac_hd_dsi_vdo_lcm_drv-chipone,icnl9916")) {
			pr_err("[TP]project is rado-HD 2\n");
			strcpy(panel_data->manufacture_info.manufacture, vendor);
			memcpy(panel_data->manufacture_info.version, "0xAB879DJ00", 11);
			panel_data->firmware_headfile.firmware_data = FW_24601_ICNL9916AC_DJN;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_24601_ICNL9916AC_DJN);
		}  else if (strstr(tp_dsi_display_primary, "oplus24615_txd_ili9883c_hd_dsi_vdo_lcm_drv-nf_ili7807s")) {
			pr_err("[TP]project is rado-HD 3\n");
			strcpy(panel_data->manufacture_info.manufacture, vendor);
			memcpy(panel_data->manufacture_info.version, "0xAB879TXD00", 12);
			panel_data->firmware_headfile.firmware_data = FW_24615_ILI7807S_TXD;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_24615_ILI7807S_TXD);
		}
	}
	if ((prj_id == 24707) || (prj_id == 24609) || (prj_id == 24710) || (prj_id == 24711)) {
		if (strstr(tp_dsi_display_primary, "oplus24707_tm_ili7807s_fhd_dsi_vdo_lcm_drv-nf_ili7807s")) {
			pr_err("[TP]project is rado-FHD 1\n");
			strcpy(panel_data->manufacture_info.manufacture, vendor);
			memcpy(panel_data->manufacture_info.version, "0xAB936TM00", 11);
			panel_data->firmware_headfile.firmware_data = FW_24707_ILI7807S_TM;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_24707_ILI7807S_TM);
		} else if (strstr(tp_dsi_display_primary, "oplus24707_ds_ili7807s_fhd_dsi_vdo_lcm_drv-nf_ili7807s")) {
			pr_err("[TP]project is rado-FHD 2\n");
			strcpy(panel_data->manufacture_info.manufacture, vendor);
			memcpy(panel_data->manufacture_info.version, "0xAB936DS00", 11);
			panel_data->firmware_headfile.firmware_data = FW_24707_ILI7807S_DS;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_24707_ILI7807S_DS);
		}
	}

	panel_data->manufacture_info.fw_path = panel_data->fw_name;

	pr_info("[TP]vendor:%s fw:%s limit:%s\n",
		vendor,
		panel_data->fw_name,
		panel_data->test_limit_name==NULL?"NO Limit":panel_data->test_limit_name);
	return 0;
}
EXPORT_SYMBOL(tp_util_get_vendor);
#endif
int preconfig_power_control(struct touchpanel_data *ts)
{
	return 0;
}
EXPORT_SYMBOL(preconfig_power_control);

int reconfig_power_control(struct touchpanel_data *ts)
{

	return 0;
}
EXPORT_SYMBOL(reconfig_power_control);

void display_esd_check_enable_bytouchpanel(bool enable)
{
	return;
}
EXPORT_SYMBOL(display_esd_check_enable_bytouchpanel);


module_param_string(dsi_display0, tp_dsi_display_primary, MAX_CMDLINE_PARAM_LEN,
                                        0600);
MODULE_PARM_DESC(dsi_display0,
	"oplus_bsp_tp_custom.dsi_display0=<display node> for primary dsi display node name");
module_param_string(dsi_display1, tp_dsi_display_secondary, MAX_CMDLINE_PARAM_LEN,
                                        0600);
MODULE_PARM_DESC(dsi_display1,
	"oplus_bsp_tp_custom.dsi_display1=<display node> for secondary dsi display node name");
MODULE_DESCRIPTION("Touchscreen Driver");
MODULE_LICENSE("GPL");

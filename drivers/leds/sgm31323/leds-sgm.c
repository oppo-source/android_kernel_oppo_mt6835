/*
 * Copyright (c) 2017, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include "leds-sgm.h"
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <soc/oplus/boot/boot_mode.h>
#ifdef CONFIG_OPLUS_CHARGER_MTK
#include <mt-plat/mtk_boot_common.h>
#endif

#define SGM_DRIVER_VERSION "V99999.0.2"
#define AW2027_DEVICE  "sgm"
#define AW2027_DEBUG(a, arg...) pr_err("[LED]"AW2027_DEVICE ": " a, ##arg);

#define LED_RED_TYPE     0x01
#define LED_GREEN_TYPE   0x04
#define LED_BLUE_TYPE    0x10
#define LED_RED_BREATH_TYPE     0x02
#define LED_GREEN_BREATH_TYPE   0x08
#define LED_BLUE_BREATH_TYPE    0x20

#define LED_EN           0x40

#define LED_RED_REG_ADDR    0x06
#define LED_GREEN_REG_ADDR  0x07
#define LED_BLUE_REG_ADDR   0x08

/* register bits */
#define AW2027_LED_BREATH_MODE_MASK			0x10
#define AW2027_LED_MANUAL_MODE_MASK			0x00

/*  config */
#define AW_LED_RESET_DELAY					8
#define AW_LED_POWER_ON_DELAY				2
#define AW_LED_POWER_OFF_DELAY				2
#define AW2027_VDD_MIN_UV					2600000
#define AW2027_VDD_MAX_UV					3300000
#define AW2027_VI2C_MIN_UV					1800000
#define AW2027_VI2C_MAX_UV					1800000

#define MAX_RISE_TIME_MS					15
#define MAX_HOLD_TIME_MS					15
#define MAX_FALL_TIME_MS					15
#define MAX_OFF_TIME_MS						15

#define SGM_LEDMODE_MAX_NUM						5

#define LED_SUPPORT_TYPE					"support"
#define LED_ESD_WORK_TIME					3

#define MAX_COLOR_DEFINE 13
enum SGM_LED_MODE {
	AW2027_LED_NONE = 0,
	AW2027_LED_CCMODE,
	AW2027_LED_BLINKMODE,
	AW2027_LED_BREATHMODE,
	AW2027_LED_INDIVIDUAL_CTL_BREATH,
	AW2027_LED_NEW_ALWAYSON,
	AW2027_LED_MAXMODE,
};

enum AW2027_LED_TYPE {
	AW2027_LED_RED = 0,
	AW2027_LED_GREEN,
	AW2027_LED_BLUE,
	AW2027_LED_ON,
	AW2027_LED_OFF,
};

struct sgm_led {
	struct i2c_client *client;
	struct led_classdev cdev;
	struct sgm_platform_data *pdata;
	struct work_struct brightness_work;
	struct mutex lock;
	struct regulator *vdd;
	struct regulator *vcc;
	int num_leds;
	int id;
	bool poweron;
	bool esd_flag;
	int vbled_enable_gpio;
	struct delayed_work   sgm_led_work;
	struct workqueue_struct *sgm_led_wq;
};

struct sgm_led *sgm_led_default;
static unsigned int sgm_debug = 2;
static char s_led_rgb_mask;
struct i2c_client *i2c_client_p;
static int rgb_color[3]={0};
static int new_always_on_color[3] ={0};

static const unsigned int brightness_source[] = {
	110, 110, 390, 510, 640, 900, 1170, 1730, 2230,
	2730, 3230, 4330, 5330};
int sgm31323_set_blink_param(struct sgm_led *led);

unsigned char SN_IIC_WriteReg(unsigned char bRegAddr, unsigned char bRegData)
{
	u8 databuf[10];
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);
	databuf[0] = bRegAddr;
	databuf[1] = bRegData;

	res = i2c_master_send(i2c_client_p, databuf, 0x2);

	if (res <= 0) {
		AW2027_DEBUG("%s err\n", __func__);
		return -1;
	}

	return 0;
}

static bool oplus_boot_mode_is_power_off_charging(void)
{
#ifdef CONFIG_OPLUS_CHARGER_MTK
	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT) {
		return true;
	} else {
		return false;
	}
#else
				return qpnp_is_power_off_charging();
#endif
}

void led_rgb_color_on(struct sgm_led *led, int level)
{
	u8 enable = 0;

	if (led->id == 0) {
		SN_IIC_WriteReg(LED_RED_REG_ADDR, level);
		if (level == 0)
			goto brightness_0;
		s_led_rgb_mask |= LED_RED_TYPE | LED_EN;
	}

	if (led->id == 1) {
		SN_IIC_WriteReg(LED_GREEN_REG_ADDR, level);
		if (level == 0)
			goto brightness_0;
		s_led_rgb_mask |= LED_GREEN_TYPE | LED_EN;

	}

	if (led->id == 2) {
		SN_IIC_WriteReg(LED_BLUE_REG_ADDR, level);
		if (level == 0)
			goto brightness_0;
		s_led_rgb_mask |= LED_BLUE_TYPE | LED_EN;
	}
	SN_IIC_WriteReg(0x00, 0x00);

	AW2027_DEBUG("%s: s_led_rgb_mask= 0x%x\n", __func__, s_led_rgb_mask);
	SN_IIC_WriteReg(0x04, s_led_rgb_mask);
	return;
brightness_0:
	if (led->cdev.brightness == 0) {
		if (led->id == 0) {    //red
			s_led_rgb_mask &= (~LED_RED_TYPE);
		} else if (led->id == 1) {	 //green
			s_led_rgb_mask &= (~LED_GREEN_TYPE);
		} else if (led->id == 2) {//blue
			s_led_rgb_mask &= (~LED_BLUE_TYPE);
		}
		enable = s_led_rgb_mask & (LED_RED_TYPE|LED_GREEN_TYPE|LED_BLUE_TYPE);
		if (enable == 0) {
			s_led_rgb_mask = 0;
			SN_IIC_WriteReg(0x04, s_led_rgb_mask);
		}
		AW2027_DEBUG("%s: enable == %d	 s_led_rgb_mask= 0x%x\n", __func__, enable, s_led_rgb_mask);
	}
}

void led_rgb_color_alway_on(struct sgm_led *led)
{
	AW2027_DEBUG("%s: r:%d g:%d b:%d\n", __func__, new_always_on_color[0], new_always_on_color[1], new_always_on_color[2]);
	SN_IIC_WriteReg(LED_RED_REG_ADDR, new_always_on_color[0]);
	s_led_rgb_mask |= LED_RED_TYPE | LED_EN;

	SN_IIC_WriteReg(LED_GREEN_REG_ADDR, new_always_on_color[1]);
	s_led_rgb_mask |= LED_GREEN_TYPE | LED_EN;


	SN_IIC_WriteReg(LED_BLUE_REG_ADDR, new_always_on_color[2]);
	s_led_rgb_mask |= LED_BLUE_TYPE | LED_EN;
	SN_IIC_WriteReg(0x00, 0x00);

	AW2027_DEBUG("%s: s_led_rgb_mask= 0x%x\n", __func__, s_led_rgb_mask);
	SN_IIC_WriteReg(0x04, s_led_rgb_mask);
	if (led->cdev.brightness == 0 && led->id == 0)
		new_always_on_color[0] = 0;
	if (led->cdev.brightness == 0 && led->id == 1)
		new_always_on_color[1] = 0;
	if (led->cdev.brightness == 0 && led->id == 2)
		new_always_on_color[2] = 0;

	if (new_always_on_color[0] == 0 && new_always_on_color[1] == 0 && new_always_on_color[2] == 0) {
		memset(rgb_color , 0, sizeof(rgb_color)); //clear rgb_color when always_on_color is 0
		s_led_rgb_mask = 0;
		SN_IIC_WriteReg(0x04, s_led_rgb_mask);
		AW2027_DEBUG("enter AW2023_LED_NEW_ALWAYSON off all\n");
	}
}


void led_rgb_color_breath_on(struct sgm_led *led, int level)
{
	u8 enable = 0;

	if (led->id == 0) {
		SN_IIC_WriteReg(LED_RED_REG_ADDR, level);
		if (level == 0)
			goto brightness_0;
		s_led_rgb_mask |= LED_RED_BREATH_TYPE | LED_EN;
	}

	if (led->id == 1) {
		SN_IIC_WriteReg(LED_GREEN_REG_ADDR, level);
		if (level == 0)
			goto brightness_0;
		s_led_rgb_mask |= LED_GREEN_BREATH_TYPE | LED_EN;

	}

	if (led->id == 2) {
		SN_IIC_WriteReg(LED_BLUE_REG_ADDR, level);
		if (level == 0)
			goto brightness_0;
		s_led_rgb_mask |= LED_BLUE_BREATH_TYPE | LED_EN;
	}

	sgm31323_set_blink_param(led);
	AW2027_DEBUG("%s: s_led_rgb_mask= 0x%x\n", __func__, s_led_rgb_mask);
	SN_IIC_WriteReg(0x04, s_led_rgb_mask);//turn on led
	return;
brightness_0:

	if (led->cdev.brightness == 0) {
		if (led->id == 0) {    //red
			s_led_rgb_mask &= (~LED_RED_BREATH_TYPE);
		} else if (led->id == 1) {	 //green
			s_led_rgb_mask &= (~LED_GREEN_BREATH_TYPE);
		} else if (led->id == 2) {	 //blue
			s_led_rgb_mask &= (~LED_BLUE_BREATH_TYPE);
		}
		enable = s_led_rgb_mask & (LED_RED_BREATH_TYPE|LED_GREEN_BREATH_TYPE|LED_BLUE_BREATH_TYPE);
		if (enable == 0) {
			s_led_rgb_mask = 0;
			SN_IIC_WriteReg(0x04, s_led_rgb_mask);
		}
		AW2027_DEBUG("%s: enable == %d	 s_led_rgb_mask= 0x%x\n", __func__, enable, s_led_rgb_mask);
	}
	return;
}

void led_rgb_color_individual_ctl_on(struct sgm_led *led)
{
	AW2027_DEBUG("%s: r:%d g:%d b:%d\n", __func__, rgb_color[0], rgb_color[1], rgb_color[2]);
	SN_IIC_WriteReg(LED_RED_REG_ADDR, rgb_color[0]);
	s_led_rgb_mask |= LED_RED_BREATH_TYPE | LED_EN;
	SN_IIC_WriteReg(LED_GREEN_REG_ADDR, rgb_color[1]);
	s_led_rgb_mask |= LED_GREEN_BREATH_TYPE | LED_EN;
	SN_IIC_WriteReg(LED_BLUE_REG_ADDR, rgb_color[2]);
	s_led_rgb_mask |= LED_BLUE_BREATH_TYPE | LED_EN;

	sgm31323_set_blink_param(led);
	AW2027_DEBUG("%s: s_led_rgb_mask= 0x%x\n", __func__, s_led_rgb_mask);
	SN_IIC_WriteReg(0x04, s_led_rgb_mask);//turn on led
	if (led->cdev.brightness == 0 && led->id == 0)
		rgb_color[0] = 0;
	if (led->cdev.brightness == 0 && led->id == 1)
		rgb_color[1] = 0;
	if (led->cdev.brightness == 0 && led->id == 2)
		rgb_color[2] = 0;

	if (rgb_color[0] == 0 && rgb_color[1] == 0 && rgb_color[2] == 0) {
		memset(new_always_on_color , 0, sizeof(new_always_on_color));
		s_led_rgb_mask = 0;
		SN_IIC_WriteReg(0x04, s_led_rgb_mask);
		AW2027_DEBUG("enter AW2023_LED_INDIVIDUAL_CTL_BREATH_MASK off all\n");
	}
}


static int sgm_power_on(struct sgm_led *led, bool on)
{
	int rc = 0;

	if (on) {
		if (!IS_ERR_OR_NULL(led->vdd)) {
			rc = regulator_enable(led->vdd);
			if (rc) {
				AW2027_DEBUG("Regulator vdd enable failed rc=%d\n", rc);
				return rc;
			}
		}
		if (!IS_ERR_OR_NULL(led->vcc)) {
			rc = regulator_enable(led->vcc);
			if (rc) {
				AW2027_DEBUG("Regulator vcc enable failed rc=%d\n", rc);
				goto fail_enable_vdd;
			}
		}
		led->poweron = true;
		msleep(AW_LED_POWER_ON_DELAY);
	} else {
		if (!IS_ERR_OR_NULL(led->vdd)) {
			rc = regulator_disable(led->vdd);
			if (rc) {
				AW2027_DEBUG("Regulator vdd disable failed rc=%d\n", rc);
				return rc;
			}
		}
		if (!IS_ERR_OR_NULL(led->vcc)) {
			rc = regulator_disable(led->vcc);
			if (rc) {
				AW2027_DEBUG("Regulator vcc disable failed rc=%d\n", rc);
				goto fail_disable_vdd;
			}
		}
		led->poweron = false;
		msleep(AW_LED_POWER_OFF_DELAY);
	}
	return rc;
fail_enable_vdd:
	rc = regulator_disable(led->vdd);
	if (rc)
		AW2027_DEBUG("Regulator vdd disable failed rc=%d\n", rc);
	return rc;

fail_disable_vdd:
	rc = regulator_enable(led->vdd);
	if (rc)
		AW2027_DEBUG("Regulator vdd enable failed rc=%d\n", rc);
	return rc;
}

static int sgm_power_init(struct sgm_led *led, bool on)
{
	int rc = 0;

	if (on) {
		led->vdd = regulator_get(&led->client->dev, "vdd");
		if (IS_ERR(led->vdd)) {
			rc = PTR_ERR(led->vdd);
			AW2027_DEBUG("Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(led->vdd) > 0) {
			rc = regulator_set_voltage(led->vdd, AW2027_VDD_MIN_UV,
						AW2027_VDD_MAX_UV);
			if (rc) {
				AW2027_DEBUG("Regulator set_vtg failed vdd rc=%d\n",
					rc);
				goto reg_vdd_put;
			}
		}

		led->vcc = regulator_get(&led->client->dev, "vcc");
		if (IS_ERR(led->vcc)) {
			rc = PTR_ERR(led->vcc);
			AW2027_DEBUG("Regulator get failed vcc rc=%d\n", rc);
			goto reg_vdd_set_vtg;
		}

		if (regulator_count_voltages(led->vcc) > 0) {
			rc = regulator_set_voltage(led->vcc, AW2027_VI2C_MIN_UV,
						AW2027_VI2C_MAX_UV);
			if (rc) {
				AW2027_DEBUG("Regulator set_vtg failed vcc rc=%d\n", rc);
				goto reg_vcc_put;
			}
		}
	} else {
		if (regulator_count_voltages(led->vdd) > 0)
			regulator_set_voltage(led->vdd, 0, AW2027_VDD_MAX_UV);

		regulator_put(led->vdd);

		if (regulator_count_voltages(led->vcc) > 0)
			regulator_set_voltage(led->vcc, 0, AW2027_VI2C_MAX_UV);

		regulator_put(led->vcc);
	}
	return 0;

reg_vcc_put:
	regulator_put(led->vcc);
reg_vdd_set_vtg:
	if (regulator_count_voltages(led->vdd) > 0)
		regulator_set_voltage(led->vdd, 0, AW2027_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(led->vdd);
	return rc;
}


int sgm31323_set_blink_param(struct sgm_led *led)
{
	unsigned int t_flash = 0;
	unsigned int t_on = 0;
	unsigned int rise_time_temp = 0;
	unsigned int fall_time_temp = 0;

	uint8_t reg0_value = 0;
	uint8_t reg1_value = 0;
	uint8_t reg5_rise_value = 0;
	uint8_t reg5_fall_value = 0;
	uint8_t reg5_total_value = 0;
	uint8_t reg2_value = 0;
	int period = 0;

	unsigned int rise_time_ms = 0;
	unsigned int hold_time_ms = 0;
	unsigned int fall_time_ms = 0;
	unsigned int off_time_ms = 0;


	rise_time_ms  = brightness_source[led->pdata->rise_time_ms];
	hold_time_ms = brightness_source[led->pdata->hold_time_ms];
	fall_time_ms = brightness_source[led->pdata->fall_time_ms];
	off_time_ms = brightness_source[led->pdata->off_time_ms];
	//AW2027_DEBUG("origin value: rise_time_ms =%d,hold_time_ms =%d,fall_time_ms=%d,off_time_ms =%d",rise_time_ms,hold_time_ms,fall_time_ms,off_time_ms);

	//calc rise and fall
	if (rise_time_ms < 96) {
		reg5_rise_value = 0;
		rise_time_temp = 2;
	} else if (rise_time_ms <= 1440) {
		reg5_rise_value = rise_time_ms/96;
		rise_time_temp = reg5_rise_value*96;
	} else if (rise_time_ms <= 2880) {
		reg0_value = 1 << 5;
		reg5_rise_value = rise_time_ms/192;
		rise_time_temp = reg5_rise_value*192;
	} else if (rise_time_ms <= 5760) {
		reg0_value = 2 << 5;
		reg5_rise_value = rise_time_ms/384;
		rise_time_temp = reg5_rise_value*384;
	} else {
		AW2027_DEBUG("rise_time_ms is out range err");
		return -1;
	}

	if (fall_time_ms < 96) {
		reg5_fall_value = 0;
		fall_time_temp = 2;
	} else if (fall_time_ms <= 1440) {
		reg5_fall_value = fall_time_ms/96;
		fall_time_temp = reg5_fall_value*96;
	} else if (fall_time_ms <= 2880) {
		reg0_value = 1 << 5;
		reg5_fall_value = fall_time_ms/192;
		fall_time_temp = reg5_fall_value*192;
	} else if (fall_time_ms <= 5760) {
		reg0_value = 2 << 5;
		reg5_fall_value = fall_time_ms/384;
		fall_time_temp = reg5_fall_value*384;
	} else {
		AW2027_DEBUG("fall_time_ms is out range err");
		return -1;
	}

	t_flash = rise_time_temp + hold_time_ms + fall_time_temp + off_time_ms;
	t_on = rise_time_temp + hold_time_ms;

	if (t_on > 16512) {
		AW2027_DEBUG("t_on is out range err");
		return -1;
	}
		//calc flash period
	if (t_flash < 384) {
		reg1_value = 0;
		t_flash = 128;
	} else {
		if (t_flash > 16512) {
			reg1_value = 127;
			t_flash = 16512;
		} else {
			reg1_value = t_flash / 128 - 2;
			t_flash = (reg1_value + 2) * 128;
		}
	}

	//reg1_value = reg1_value | 0x80; //reg01 çš„bit7 å†™1 ï¼Œæ¢æˆçº¿æ€§æ¨¡å¼ã€‚
	reg5_total_value = reg5_fall_value << 4 | reg5_rise_value;
	//calc reg2 percentage
	reg2_value = t_on * 10000 / t_flash / 39;
	//AW2027_DEBUG("origin value: reg1_value =0x%x,reg5_total_value =0x%x,reg2_value=0x%x,reg0_value=0x%x",reg1_value,reg5_total_value,reg2_value,reg0_value);
	SN_IIC_WriteReg(0, reg0_value);			//reg0[6:5] ramp scaling
	SN_IIC_WriteReg(1, reg1_value);			//reg1 flash period
	SN_IIC_WriteReg(5, reg5_total_value);	//reg5  rase timeã€fall time
	SN_IIC_WriteReg(2, reg2_value);			//reg2 percentage
	//AW2027_DEBUG("actual value: reg5_rise_value =%d,flash_period_ms =%d,reg5_fall_value=%d,percentage =%d",reg5_rise_value,t_flash,reg5_fall_value,reg2_value);
	///AW2027_DEBUG("actual value: rise_ms =%d,hold_time_ms =%d,fall_ms=%d, t_flash=%d",rise_time_temp,hold_time_ms,fall_time_temp,t_flash);
	period = t_flash;
	return period;
}

static int sgm_led_change_mode(struct sgm_led *led, enum SGM_LED_MODE mode)
{
	int ret = 0;

	AW2027_DEBUG("=================>    mode = %d\n", mode);
	switch (mode) {
	case AW2027_LED_CCMODE:
		led->pdata->led_mode = AW2027_LED_MANUAL_MODE_MASK;
		break;
	case AW2027_LED_BLINKMODE:
		led->pdata->hold_time_ms = 4;
		led->pdata->off_time_ms =  4;
		led->pdata->rise_time_ms = 0;
		led->pdata->fall_time_ms = 0;
		led->pdata->led_mode = AW2027_LED_BREATH_MODE_MASK;
		break;
	case AW2027_LED_BREATHMODE:
		led->pdata->hold_time_ms = 0;
		led->pdata->off_time_ms =  0;
		led->pdata->rise_time_ms = 6;
		led->pdata->fall_time_ms = 6;
		led->pdata->led_mode = AW2027_LED_BREATH_MODE_MASK;
		break;
	case AW2027_LED_INDIVIDUAL_CTL_BREATH:
		led->pdata->hold_time_ms = 0;
		led->pdata->off_time_ms =  0;
		led->pdata->rise_time_ms = 6;
		led->pdata->fall_time_ms = 6;
		led->pdata->led_mode = AW2027_LED_INDIVIDUAL_CTL_BREATH;
		break;
	case AW2027_LED_NEW_ALWAYSON:
		led->pdata->led_mode = AW2027_LED_NEW_ALWAYSON;
		break;
	default:
		led->pdata->led_mode = AW2027_LED_MANUAL_MODE_MASK;
		break;
	}
	return ret;
}

static int cale_brightness_fsnode(unsigned int brightness_in)
{
	int bright_tmp;

	if (sgm_led_default == NULL) {
		AW2027_DEBUG("%s sgm_led_default is NULL\n", __func__);
		return -EINVAL;
	}

	AW2027_DEBUG("%s max_brightness = %d ", __func__, sgm_led_default->cdev.max_brightness);

	if (brightness_in > 0xf)
		bright_tmp = sgm_led_default->cdev.max_brightness;
	else
		bright_tmp = brightness_in * 100 / 0xf * sgm_led_default->cdev.max_brightness / 100;

	AW2027_DEBUG("%s brightness_in = %d bright_tmp = %d", __func__,
		brightness_in, bright_tmp);
	return bright_tmp;
}

static int cale_brightness_breath(struct sgm_led *led)
{
	int bright_tmp;


	AW2027_DEBUG("%s max_brightness = %d ", __func__, sgm_led_default->cdev.max_brightness);
	if (led->cdev.brightness < 0)
		led->cdev.brightness = 0;
	if (led->cdev.brightness > 0xff)
		led->cdev.brightness = sgm_led_default->cdev.max_brightness;
	if (led->cdev.brightness > 0xf)
		bright_tmp = led->cdev.brightness * 100 / 0xff * sgm_led_default->cdev.max_brightness / 100;
	else
		bright_tmp = led->cdev.brightness * 100 / 0xf * sgm_led_default->cdev.max_brightness / 100;

	AW2027_DEBUG("%sled->cdev.brightness[%d] = %d bright_tmp = %d", __func__,
		led->id, led->cdev.brightness, bright_tmp);
	return bright_tmp;
}


static int cale_brightness(struct sgm_led *led)
{
	int bright_tmp;


	AW2027_DEBUG("%s max_brightness = %d ", __func__, sgm_led_default->cdev.max_brightness);

	if (led->cdev.brightness < 0)
		led->cdev.brightness = 0;
	if (led->cdev.brightness > 0xff)
		led->cdev.brightness = sgm_led_default->cdev.max_brightness;
	//bright_tmp = sgm_bright[led->cdev.brightness];
	bright_tmp = led->cdev.brightness * 100 / 0xff * sgm_led_default->cdev.max_brightness / 100;
	//AW2027_DEBUG("bright_tmp = %d",bright_tmp);
	//bright_tmp /= 100;

	AW2027_DEBUG("%sled->cdev.brightness[%d] = %d bright_tmp = %d", __func__,
		led->id, led->cdev.brightness, bright_tmp);
	return bright_tmp;
}

static void sgm_brightness(struct sgm_led *led)
{
	u8 enable = 0;
	int bright_tmp = 0;

	if (oplus_boot_mode_is_power_off_charging()) {
		AW2027_DEBUG("boot_mode is power_off_charging");
		return;
	}

	mutex_lock(&led->pdata->led->lock);
	if (led->pdata->led_mode == AW2027_LED_BREATH_MODE_MASK) {
		bright_tmp = cale_brightness_breath(led);
	} else if (led->pdata->led_mode == AW2027_LED_MANUAL_MODE_MASK){
		bright_tmp = cale_brightness(led);
	}

	if (!led->pdata->led->poweron) {
		if (sgm_power_on(led->pdata->led, true)) {
			AW2027_DEBUG("power on failed\n");
			mutex_unlock(&led->pdata->led->lock);
			return;
		}
	}

	AW2027_DEBUG("%s led->id = %d", __func__, led->id);
	/*aw2027 led breath time*/
	if (led->pdata->led_mode == AW2027_LED_BREATH_MODE_MASK) {
		led_rgb_color_breath_on(led, bright_tmp);
		AW2027_DEBUG("%s:BREATH_MODE brightness[%d] = 0x%x led_mode[%d]=%x enable= %d\n", __func__,
			led->id, bright_tmp, led->id, led->pdata->led_mode, enable);
	} else if (led->pdata->led_mode == AW2027_LED_MANUAL_MODE_MASK) {
		led_rgb_color_on(led, bright_tmp);
		AW2027_DEBUG("%s:MANUAL_MODE brightness[%d] = 0x%x led_mode[%d]=%x s_led_rgb_mask= 0x%x\n",
			__func__, led->id,
			bright_tmp, led->id, led->pdata->led_mode, s_led_rgb_mask);
	}else if (led->pdata->led_mode == AW2027_LED_INDIVIDUAL_CTL_BREATH) {
		led_rgb_color_individual_ctl_on(led);
		AW2027_DEBUG("enter AW2023_LED_INDIVIDUAL_CTL_BREATH_MASK\n");
	} else if(led->pdata->led_mode == AW2027_LED_NEW_ALWAYSON) {
		AW2027_DEBUG("enter AW2023_LED_NEW_ALWAYSON\n");
		led_rgb_color_alway_on(led);
	}

	mutex_unlock(&led->pdata->led->lock);

}

static void sgm_brightness_work(struct work_struct *work)
{
	struct sgm_led *led = container_of(work, struct sgm_led,
					brightness_work);
	sgm_brightness(led);
}

static void sgm_set_brightness(struct led_classdev *cdev,
				enum led_brightness brightness)
{
	struct sgm_led *led = container_of(cdev, struct sgm_led, cdev);
	led->cdev.brightness = brightness;

	if (led->cdev.trigger != NULL) {
		if (strcmp(led->cdev.trigger->name, "timer") == 0) {
			sgm_led_change_mode(led, AW2027_LED_BLINKMODE);
			AW2027_DEBUG("%s[%d]: trigger = %s\n", __func__, led->id, led->cdev.trigger->name);
		}
	}
	schedule_work(&led->brightness_work);
}

static ssize_t sgm_led_support_attr_show (struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct sgm_led *led = container_of(led_cdev, struct sgm_led, cdev);

	return snprintf(buf, PAGE_SIZE, "%s-%d\n", LED_SUPPORT_TYPE, led->cdev.max_brightness);
}

static ssize_t sgm_led_breath_on_attr_show (struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct sgm_led *led = container_of(led_cdev, struct sgm_led, cdev);

	return snprintf(buf, PAGE_SIZE, "%d (max:15)\n", led->pdata->rise_time_ms);
}

static ssize_t sgm_led_breath_on_attr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t cnt)
{
	unsigned long data = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct sgm_led *led = container_of(led_cdev, struct sgm_led, cdev);
	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &data);
	if (ret)
		return ret;

	mutex_lock(&led->pdata->led->lock);
	led->pdata->rise_time_ms = data;
	mutex_unlock(&led->pdata->led->lock);

	AW2027_DEBUG("%s[%d]: rise_time_ms = %d (max:15)\n", __func__, led->id, led->pdata->rise_time_ms);

	return cnt;
}

static ssize_t sgm_led_breath_off_attr_show (struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct sgm_led *led = container_of(led_cdev, struct sgm_led, cdev);

	return snprintf(buf, PAGE_SIZE, "%d (max:15)\n", led->pdata->fall_time_ms);
}

static ssize_t sgm_led_breath_off_attr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t cnt)
{
	unsigned long data = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct sgm_led *led = container_of(led_cdev, struct sgm_led, cdev);
	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &data);
	if (ret)
		return ret;

	mutex_lock(&led->pdata->led->lock);
	led->pdata->fall_time_ms = data;
	mutex_unlock(&led->pdata->led->lock);

	AW2027_DEBUG("%s[%d]: fall_time_ms = %d (max:15)\n", __func__, led->id, led->pdata->fall_time_ms);
	return cnt;
}

static ssize_t sgm_led_ton_attr_show (struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct sgm_led *led = container_of(led_cdev, struct sgm_led, cdev);

	return snprintf(buf, PAGE_SIZE, "%d (max:15)\n", led->pdata->hold_time_ms);
}

static ssize_t sgm_led_ton_attr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t cnt)
{
	unsigned long data = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct sgm_led *led = container_of(led_cdev, struct sgm_led, cdev);

	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &data);
	if (ret)
		return ret;

	mutex_lock(&led->pdata->led->lock);
	led->pdata->hold_time_ms = data;
	mutex_unlock(&led->pdata->led->lock);
	AW2027_DEBUG("%s[%d]: hold_time_ms = %d (max:15)\n", __func__, led->id, led->pdata->hold_time_ms);

	return cnt;
}

static ssize_t sgm_led_tr1_attr_show (struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct sgm_led *led = container_of(led_cdev, struct sgm_led, cdev);

	return snprintf(buf, PAGE_SIZE, "%d (max:15)\n", led->pdata->rise_time_ms);
}

static ssize_t sgm_led_tr1_attr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t cnt)
{
	unsigned long data = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct sgm_led *led = container_of(led_cdev, struct sgm_led, cdev);

	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &data);
	if (ret)
		return ret;

	mutex_lock(&led->pdata->led->lock);
	led->pdata->rise_time_ms = data;
	mutex_unlock(&led->pdata->led->lock);

	AW2027_DEBUG("%s[%d]: rise_time_ms = %d (max:15)\n", __func__, led->id, led->pdata->rise_time_ms);

	return cnt;
}

static ssize_t sgm_led_tf1_attr_show (struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct sgm_led *led = container_of(led_cdev, struct sgm_led, cdev);

	return snprintf(buf, PAGE_SIZE, "%d (max:15)\n", led->pdata->fall_time_ms);

}

static ssize_t sgm_led_tf1_attr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t cnt)
{
	unsigned long data = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct sgm_led *led = container_of(led_cdev, struct sgm_led, cdev);

	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &data);
	if (ret)
		return ret;

	mutex_lock(&led->pdata->led->lock);
	led->pdata->fall_time_ms = data;
	mutex_unlock(&led->pdata->led->lock);

	AW2027_DEBUG("%s[%d]: fall_time_ms = %d (max:15)\n", __func__, led->id, led->pdata->fall_time_ms);

	return cnt;
}

static ssize_t sgm_led_toff_attr_show (struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct sgm_led *led = container_of(led_cdev, struct sgm_led, cdev);

	return snprintf(buf, PAGE_SIZE, "%d (max:15)\n", led->pdata->off_time_ms);
}

static ssize_t sgm_led_toff_attr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t cnt)
{
	unsigned long data = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct sgm_led *led = container_of(led_cdev, struct sgm_led, cdev);

	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &data);
	if (ret)
		return ret;

	mutex_lock(&led->pdata->led->lock);
	led->pdata->off_time_ms = data;
	mutex_unlock(&led->pdata->led->lock);
	AW2027_DEBUG("%s[%d]: off_time_ms = %d (max:15)\n", __func__, led->id, led->pdata->off_time_ms);

	return cnt;
}

static ssize_t sgm_reg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;

	return len;
}

static ssize_t sgm_reg_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	unsigned int databuf[2];

	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
		SN_IIC_WriteReg((unsigned char)databuf[0], (unsigned char)databuf[1]);
	}

	return len;
}
static ssize_t aw2027_led_color_attr_show (struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct sgm_led *led = container_of(led_cdev, struct sgm_led, cdev);

	return snprintf(buf, PAGE_SIZE, "%d (max:15)\n",led->pdata->color);
}

static ssize_t aw2027_led_color_attr_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t cnt)
{
    unsigned long data = 0;
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct sgm_led *led = container_of(led_cdev, struct sgm_led, cdev);
    char *p_buf = NULL;
	char *tmp_buf = NULL;
	char *str = NULL;
    ssize_t ret = -EINVAL;

    p_buf = kstrdup(buf, GFP_KERNEL);
    tmp_buf = p_buf;
    str = strsep(&tmp_buf, ",");  // ÌáÈ¡µÚÒ»¸öÖµ

    if (str) {
        ret = kstrtoul(str, 10, &data);
        if (!ret && data <= 15) {  // ÑéÖ¤·¶Î§0-15
            mutex_lock(&led->pdata->led->lock);
                rgb_color[led->id] = cale_brightness_fsnode(data);
            mutex_unlock(&led->pdata->led->lock);
            AW2027_DEBUG("[%d]: led_color= %lu\n", led->id, data);
        } else {
            ret = -EINVAL;  // ·Ç·¨ÊýÖµ
        }
    }

    kfree(p_buf);
    return ret ? ret : cnt;
}

void store_effect_color_and_brightness(char *tmp_buf)
{
	unsigned long data = 0;
	char *delim = ",";
	char *str = NULL;
	int ret=0;
	int effect_index=0;
	unsigned int br_rgb = 0;
	unsigned int rgb = 0;

	str = strsep(&tmp_buf, delim);
	if (str == NULL) {
		AW2027_DEBUG("error in parsing 1.1");
		return;
	}
	ret = kstrtoul(str, 10, &data);
	if (ret) {
		AW2027_DEBUG("error in parsing 1.2");
		return;
	}
	effect_index = data;

	str = strsep(&tmp_buf, delim);
	if (str == NULL) {
		AW2027_DEBUG("error in parsing 1.1");
		return;
	}
	ret = kstrtoul(str, 16, &data);
	if (ret) {
		AW2027_DEBUG("error in parsing 1.2");
		return;
	}
	br_rgb = data;

	str = strsep(&tmp_buf, delim);
	if (str == NULL) {
		AW2027_DEBUG("error in parsing 1.1");
		return;
	}
	ret = kstrtoul(str, 16, &data);
	if (ret) {
		AW2027_DEBUG("error in parsing 1.2");
		return;
	}
	rgb = data;

	new_always_on_color[0] = cale_brightness_fsnode((rgb >> 16) & 0xff);
	new_always_on_color[1] = cale_brightness_fsnode((rgb >> 8) & 0xff);
	new_always_on_color[2] = cale_brightness_fsnode(rgb & 0xff);
	return;
}

static ssize_t aw2027_effectdata_attr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t cnt)
{
	unsigned long data = 0;
	char *delim = ",";
	char *str = NULL;
	char *p_buf, *tmp_buf;

	ssize_t ret = -EINVAL;

	AW2027_DEBUG("effectdata raw %s\n", buf);

	p_buf = kstrdup(buf, GFP_KERNEL);
	tmp_buf = p_buf;

	str = strsep(&tmp_buf, delim);
	if (str == NULL) {
		goto errout;
	}
	ret = kstrtoul(str, 10, &data);
	if (ret) {
		goto errout;
	}

	if(data == 0){
		store_effect_color_and_brightness(tmp_buf);
	}

errout:
	kfree(p_buf);
	return cnt;
}

static ssize_t sgm_led_debug_attr_show (struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u (max:1)\n", sgm_debug);
}

static ssize_t sgm_led_debug_attr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t cnt)
{
	unsigned long data = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct sgm_led *led = container_of(led_cdev, struct sgm_led, cdev);
	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &data);
	if (ret)
		return ret;

	mutex_lock(&led->pdata->led->lock);
	sgm_debug = data;
	mutex_unlock(&led->pdata->led->lock);

	AW2027_DEBUG("%s: aw2027_debug = %u (max:15)\n", __func__, sgm_debug);

	return cnt;
}

static DEVICE_ATTR(support, 0664, sgm_led_support_attr_show, NULL);
static DEVICE_ATTR(breath_on, 0664, sgm_led_breath_on_attr_show,  sgm_led_breath_on_attr_store);
static DEVICE_ATTR(breath_off, 0664, sgm_led_breath_off_attr_show,  sgm_led_breath_off_attr_store);
static DEVICE_ATTR(ton, 0664, sgm_led_ton_attr_show, sgm_led_ton_attr_store);
static DEVICE_ATTR(toff, 0664, sgm_led_toff_attr_show, sgm_led_toff_attr_store);
static DEVICE_ATTR(tr1, 0664, sgm_led_tr1_attr_show, sgm_led_tr1_attr_store);
static DEVICE_ATTR(tf1, 0664, sgm_led_tf1_attr_show, sgm_led_tf1_attr_store);
static DEVICE_ATTR(reg, 0664, sgm_reg_show, sgm_reg_store);
static DEVICE_ATTR(color, 0664, aw2027_led_color_attr_show, aw2027_led_color_attr_store);
static DEVICE_ATTR(effectdata, 0664, NULL, aw2027_effectdata_attr_store);
static DEVICE_ATTR(debug, 0664, sgm_led_debug_attr_show, sgm_led_debug_attr_store);

static struct attribute *sgm_led_attributes[] = {
	&dev_attr_support.attr,
	&dev_attr_ton.attr,
	&dev_attr_toff.attr,
	&dev_attr_tr1.attr,
	&dev_attr_tf1.attr,
	&dev_attr_reg.attr,
	&dev_attr_color.attr,
	&dev_attr_effectdata.attr,
	&dev_attr_debug.attr,
	NULL
};

static struct attribute_group sgm_led_attr_group = {
	.attrs = sgm_led_attributes,
};

static struct attribute *sgm_led_breath_mode_attrs[] = {
	&dev_attr_breath_on.attr,
	&dev_attr_breath_off.attr,
	NULL
};

ATTRIBUTE_GROUPS(sgm_led_breath_mode);

static int  sgm_led_cc_activate(struct led_classdev *cdev)
{
	int ret = 0;
	struct sgm_led *led = container_of(cdev, struct sgm_led, cdev);

	AW2027_DEBUG("%s[%d]: activate", __func__, led->id);

	ret = sgm_led_change_mode(led, AW2027_LED_CCMODE);
	if (ret < 0) {
		AW2027_DEBUG("%s: aw2027_led_change_mode fail\n", __func__);
		return ret;
	}

	return ret;
}

static void  sgm_led_cc_deactivate(struct led_classdev *cdev)
{
	struct sgm_led *led = container_of(cdev, struct sgm_led, cdev);

	AW2027_DEBUG("%s[%d]: deactivate", __func__, led->id);
}

static int  sgm_led_blink_activate(struct led_classdev *cdev)
{
	int ret = 0;
	struct sgm_led *led = container_of(cdev, struct sgm_led, cdev);

	AW2027_DEBUG("%s[%d]: activate", __func__, led->id);

	ret = sgm_led_change_mode(led, AW2027_LED_BLINKMODE);
	if (ret < 0) {
		AW2027_DEBUG("%s: aw2027_led_change_mode fail\n", __func__);
		return ret;
	}

	return ret;
}

static void sgm_led_blink_deactivate(struct led_classdev *cdev)
{
	struct sgm_led *led = container_of(cdev, struct sgm_led, cdev);

	AW2027_DEBUG("%s[%d]: deactivate", __func__, led->id);
}

static int sgm_led_breath_activate(struct led_classdev *cdev)
{
	int ret = 0;
	struct sgm_led *led = container_of(cdev, struct sgm_led, cdev);

	AW2027_DEBUG("%s[%d]: activate", __func__, led->id);

	ret = sgm_led_change_mode(led, AW2027_LED_BREATHMODE);
	if (ret < 0) {
		AW2027_DEBUG("%s: aw2027_led_change_mode fail\n", __func__);
		return ret;
	}
	return ret;
}

static void sgm_led_breath_deactivate(struct led_classdev *cdev)
{
	struct sgm_led *led = container_of(cdev, struct sgm_led, cdev);

	AW2027_DEBUG("%s[%d]: deactivate", __func__, led->id);
}
static int sgm_led_individual_ctl_breath_activate(struct led_classdev *cdev)
{
	int ret = 0;
	struct sgm_led *led = container_of(cdev, struct sgm_led, cdev);

	AW2027_DEBUG("%s[%d]: activate", __func__, led->id);

	ret = sgm_led_change_mode(led, AW2027_LED_INDIVIDUAL_CTL_BREATH);
	if (ret < 0) {
		dev_err(led->cdev.dev, "%s: sgm_led_change_mode fail\n", __func__);
		return ret;
	}
	return ret;
}

static void sgm_led_individual_ctl_breath_deactivate(struct led_classdev *cdev)
{
	struct sgm_led *led = container_of(cdev, struct sgm_led, cdev);

	AW2027_DEBUG("%s[%d]: deactivate", __func__, led->id);
}

static int sgm_led_new_always_on_activate(struct led_classdev *cdev)
{
	int ret = 0;
	struct sgm_led *led = container_of(cdev, struct sgm_led, cdev);

	AW2027_DEBUG("%s[%d]: activate", __func__, led->id);

	ret = sgm_led_change_mode(led, AW2027_LED_NEW_ALWAYSON);
	if (ret < 0) {
		dev_err(led->cdev.dev, "%s: aw2023_led_change_mode fail\n", __func__);
		return ret;
	}
	return ret;
}

static void sgm_led_new_always_on_deactivate(struct led_classdev *cdev)
{
	struct sgm_led *led = container_of(cdev, struct sgm_led, cdev);

	AW2027_DEBUG("%s[%d]: deactivate", __func__, led->id);
}

static struct led_trigger sgm_led_trigger[SGM_LEDMODE_MAX_NUM] = {
	{
		.name = "cc_mode",
		.activate = sgm_led_cc_activate,
		.deactivate = sgm_led_cc_deactivate,
	},
	{
		.name = "blink_mode",
		.activate = sgm_led_blink_activate,
		.deactivate = sgm_led_blink_deactivate,
	},
	{
		.name = "breath_mode",
		.activate = sgm_led_breath_activate,
		.deactivate = sgm_led_breath_deactivate,
		.groups = sgm_led_breath_mode_groups,
	},
	{
		.name = "individual_ctl_breath",
		.activate = sgm_led_individual_ctl_breath_activate,
		.deactivate = sgm_led_individual_ctl_breath_deactivate,
		//.groups = aw2023_led_breath_mode_groups,
	},
	{
		.name = "new_always_on_mode",
		.activate = sgm_led_new_always_on_activate,
		.deactivate = sgm_led_new_always_on_deactivate,
//		.groups = aw210xx_led_new_always_on_mode_groups,
	},
};


static int sgm_check_chipid(struct sgm_led *led)
{
	u8 ret = 0;
	u8 cnt = 0;

	AW2027_DEBUG("%s[%d]: ", __func__, led->id);

	for (cnt = 5; cnt > 0; cnt--) {
		ret = SN_IIC_WriteReg(0, 0x1f);//reset
		pr_err("sgm chip id check i2c commu", ret);
		if (ret == 0)
			return 0;
	}
	return -EINVAL;
}

static int sgm_init(struct sgm_led *led)
{
	AW2027_DEBUG("%s[%d]: ", __func__, led->id);

	SN_IIC_WriteReg(0, 0x1f);//reset
	return 0;
}

static int sgm_led_err_handle(struct sgm_led *led_array,
				int parsed_leds)
{
	int i = 0;
	/*
	* If probe fails, cannot free resource of all LEDs, only free
	* resources of LEDs which have allocated these resource really.
	*/
	for (i = 0; i < parsed_leds; i++) {
		sysfs_remove_group(&led_array[i].cdev.dev->kobj,
				&sgm_led_attr_group);
		led_classdev_unregister(&led_array[i].cdev);
		cancel_work_sync(&led_array[i].brightness_work);
		devm_kfree(&led_array->client->dev, led_array[i].pdata);
		led_array[i].pdata = NULL;
	}
	return i;
}

static void sgm_work_func(struct work_struct *sgm_work)
{
	return;
}

static int sgm_led_parse_child_node(struct sgm_led *led_array,
				struct device_node *node)
{
	struct sgm_led *led;
	struct device_node *temp;
	struct sgm_platform_data *pdata;
	int rc = 0;
	int parsed_leds = 0;
	int i = 0;

	for_each_child_of_node(node, temp) {
		led = &led_array[parsed_leds];
		led->client = led_array->client;

		pdata = devm_kzalloc(&led->client->dev,
				sizeof(struct sgm_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			AW2027_DEBUG("Failed to allocate memory\n");
			parsed_leds--;
			goto free_pdata;
		}
		pdata->led = led_array;
		led->pdata = pdata;

		rc = of_property_read_string(temp, "sgm,name",
			&led->cdev.name);
		if (rc < 0) {
			AW2027_DEBUG("Failure reading led name, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "sgm,id",
			&led->id);
		if (rc < 0) {
			AW2027_DEBUG("Failure reading id, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "sgm,imax",
			&led->pdata->imax);
		if (rc < 0) {
			AW2027_DEBUG("Failure reading id, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_string(temp, "sgm,led_default_trigger",
									&led->pdata->led_default_trigger);
		if (rc < 0) {
			AW2027_DEBUG("Failure led_default_trigger, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "sgm,brightness",
			&led->cdev.brightness);
		if (rc < 0) {
			AW2027_DEBUG("Failure reading brightness, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "sgm,max-brightness",
			&led->cdev.max_brightness);
		if (rc < 0) {
			AW2027_DEBUG("Failure reading max-brightness, rc = %d\n",
				rc);
			goto free_pdata;
		}
		rc = of_property_read_u32(temp, "sgm,rise-time-ms",
			&led->pdata->rise_time_ms);
		if (rc < 0) {
			AW2027_DEBUG("Failure reading rise-time-ms, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "sgm,hold-time-ms",
			&led->pdata->hold_time_ms);
		if (rc < 0) {
			AW2027_DEBUG("Failure reading hold-time-ms, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "sgm,fall-time-ms",
			&led->pdata->fall_time_ms);
		if (rc < 0) {
			AW2027_DEBUG("Failure reading fall-time-ms, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "sgm,off-time-ms",
			&led->pdata->off_time_ms);
		if (rc < 0) {
			AW2027_DEBUG("Failure reading off-time-ms, rc = %d\n", rc);
			goto free_pdata;
		}
		parsed_leds++;
	}
	return 0;

free_pdata:
	for (i = 0; i <= parsed_leds; i++) {
		devm_kfree(&led_array->client->dev, led_array[i].pdata);
	}
	return rc;
}

static int sgm_led_register(struct sgm_led *led_array)
{
	struct sgm_led *led;
	int i = 0;
	int rc = 0;

	if (led_array == NULL) {
		AW2027_DEBUG("%s led_array is NULL\n", __func__);
		return -1;
	}

	for (i = 0; i < led_array->num_leds; i++) {
		led = &led_array[i];

		INIT_WORK(&led->brightness_work, sgm_brightness_work);
		led->cdev.brightness_set = sgm_set_brightness;
		led->cdev.default_trigger = led->pdata->led_default_trigger;

		rc = led_classdev_register(&led->client->dev, &led->cdev);
		if (rc) {
			AW2027_DEBUG("unable to register led %d,rc=%d\n",
				led->id, rc);
			goto free_sync;
		}

		rc = sysfs_create_group(&led->cdev.dev->kobj,
				&sgm_led_attr_group);
		if (rc) {
			AW2027_DEBUG("led sysfs rc: %d\n", rc);
			goto free_dev;
		}
	}

	return 0;
free_dev:
	led_classdev_unregister(&led_array[i].cdev);
free_sync:
	cancel_work_sync(&led_array[i].brightness_work);
	sgm_led_err_handle(led_array, i);
	return rc;
}

int sgm_led_trigger_register(struct sgm_led *led_array)
{
	int i = 0;
	int ret = 0;

	if (led_array == NULL) {
		AW2027_DEBUG("%s led_array is NULL\n", __func__);
		return -1;
	}
	for (i = 0; i < SGM_LEDMODE_MAX_NUM; i++) {
		AW2027_DEBUG("%s trigger %d\n", __func__, i);
		ret = led_trigger_register(&sgm_led_trigger[i]);
		if (ret < 0) {
			AW2027_DEBUG("register %d trigger fail\n", i);
			goto fail_led_trigger;
		}
	}

	return 0;
fail_led_trigger:
	while (--i >= 0)
		led_trigger_unregister(&sgm_led_trigger[i]);
	return ret;
}

static int sgm_led_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct sgm_led *led_array;
	struct device_node *node;
	int ret = 0;
	int num_leds = 0;
	int i = 0;

	node = client->dev.of_node;
	if (node == NULL) {
		AW2027_DEBUG("sgm node is NULL skip probe");
		return -EINVAL;
	}
	if (oplus_boot_mode_is_power_off_charging()) {
		AW2027_DEBUG("sgm boot_mode is power_off_charging skip probe");
		return 0;
	}
	num_leds = of_get_child_count(node);
	if (!num_leds) {
		AW2027_DEBUG("sgm boot_mode is power_off_charging skip probe");
		return -EINVAL;
	}
	led_array = devm_kzalloc(&client->dev,
			(sizeof(struct sgm_led) * num_leds), GFP_KERNEL);
	if (!led_array) {
		AW2027_DEBUG("sgm boot_mode is power_off_charging skip probe");
		return -ENOMEM;
	}

	sgm_led_default = led_array;

	led_array->client = client;
	led_array->num_leds = num_leds;
	AW2027_DEBUG("sgm num_leds = %d\n", led_array->num_leds);
	mutex_init(&led_array->lock);

	led_array->vbled_enable_gpio = of_get_named_gpio(node, "vbled-enable-gpio", 0);
	if (led_array->vbled_enable_gpio < 0) {
		led_array->vbled_enable_gpio = -1;
		AW2027_DEBUG("sgm no vbled enable gpio provided, HW enable unsupported\n");
	}

	if (gpio_is_valid(led_array->vbled_enable_gpio)) {
		ret = devm_gpio_request_one(&client->dev, led_array->vbled_enable_gpio,
				GPIOF_OUT_INIT_HIGH, "vbled-enable-gpio");
		AW2027_DEBUG("sgm vbled-enable-gpio run\n");
		if (ret) {
			AW2027_DEBUG("sgm vbled-enable-gpio request failed\n");
			goto free_led_arry;
		}
	}
	AW2027_DEBUG("sgm parsed node\n");

	ret = sgm_led_parse_child_node(led_array, node);
	if (ret) {
		AW2027_DEBUG("sgm parsed node error\n");
		goto free_led_arry;
	}

	i2c_set_clientdata(client, led_array);
	i2c_client_p = client;

	ret = sgm_power_init(led_array, true);
	if (ret) {
		AW2027_DEBUG("sgm power init failed");
		goto free_led_arry;
	}

	led_array->poweron = false;
	if (!led_array->poweron) {
		ret = sgm_power_on(led_array->pdata->led, true);
		if (ret) {
			if (sgm_power_on(led_array->pdata->led, true)) {
				AW2027_DEBUG("sgm  Probe power on fail\n");
			}
		}
	}

	ret = sgm_check_chipid(led_array);
	if (ret) {
		AW2027_DEBUG("sgm Check chip id error\n");
		goto check_chipid_failed;
	}
	AW2027_DEBUG("sgm sgm_led_register\n");

	ret = sgm_led_register(led_array);
	if (ret) {
		AW2027_DEBUG("sgm led register error\n");
		goto check_chipid_failed;
	}
	/* aw2027 led trigger register */
	ret = sgm_led_trigger_register(led_array);
	if (ret) {
		AW2027_DEBUG("sgm led trigger register error\n");
		goto free_led_register;
	}
	/* aw2027 led init */
	ret = sgm_init(led_array);
	if (ret) {
		AW2027_DEBUG("sgm  aw2027_led_init_default: led[%d] error\n", i);
		goto free_led_register;
	}


	sgm_led_default->sgm_led_wq = create_singlethread_workqueue("sgm_led_workqueue");
	if (!sgm_led_default->sgm_led_wq) {
		dev_err(&client->dev, "sgm aw2027_led_workqueue error\n");
		goto free_led_register;
	}
	INIT_DELAYED_WORK(&sgm_led_default->sgm_led_work, sgm_work_func);
	queue_delayed_work(sgm_led_default->sgm_led_wq,
		&sgm_led_default->sgm_led_work, LED_ESD_WORK_TIME * HZ);
	AW2027_DEBUG("sgm probe end\n");

	return 0;

free_led_register:
	sgm_led_err_handle(led_array, num_leds);
check_chipid_failed:
	if (gpio_is_valid(led_array->vbled_enable_gpio)) {
		AW2027_DEBUG("sgm gpio free");
		gpio_free(led_array->vbled_enable_gpio);
	}
free_led_arry:
	mutex_destroy(&led_array->lock);

	devm_kfree(&client->dev, led_array);
	//i2c_client_p = NULL;
	led_array = NULL;
	return ret;
}

static int sgm_led_remove(struct i2c_client *client)
{
	struct sgm_led *led_array = i2c_get_clientdata(client);
	int i = 0;
	int parsed_leds = led_array->num_leds;

	pr_err("sgm_led_remove start.\n");

	for (i = 0; i < parsed_leds; i++) {
		sysfs_remove_group(&led_array[i].cdev.dev->kobj,
				&sgm_led_attr_group);
		led_classdev_unregister(&led_array[i].cdev);
		cancel_work_sync(&led_array[i].brightness_work);
		devm_kfree(&client->dev, led_array[i].pdata);
		led_array[i].pdata = NULL;
	}

	pr_err("sgm_led_remove start..\n");
	mutex_destroy(&led_array->lock);

	if (gpio_is_valid(led_array->vbled_enable_gpio)) {
		pr_err("sgm gpio free");
		gpio_free(led_array->vbled_enable_gpio);
	}
	devm_kfree(&client->dev, led_array);
	led_array = NULL;
	pr_err("sgm_led_remove done.\n");
	return 0;
}

static const struct i2c_device_id sgm_led_id[] = {
	{"sgm_led", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, sgm_led_id);

static struct of_device_id sgm_match_table[] = {
	{ .compatible = "sgm,sgm31323_led",},
	{ },
};

static struct i2c_driver sgm_led_driver = {
	.probe = sgm_led_probe,
	.remove = sgm_led_remove,
	//.shutdown = sgm_led_shutdown,
	.driver = {
		.name = "sgm_led",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(sgm_match_table),
	},
	.id_table = sgm_led_id,
};

static int __init sgm_led_init(void)
{
	pr_err("%s: driver version1: %s\n", __func__, SGM_DRIVER_VERSION);
	return i2c_add_driver(&sgm_led_driver);
}

late_initcall(sgm_led_init);

static void __exit sgm_led_exit(void)
{
	i2c_del_driver(&sgm_led_driver);
}
module_exit(sgm_led_exit);

MODULE_AUTHOR("yangcl");
MODULE_DESCRIPTION("SGM LED driver");
MODULE_LICENSE("GPL v2");

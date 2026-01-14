// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2021 Oplus

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/videodev2.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>
#if IS_ENABLED(CONFIG_MTK_FLASHLIGHT)
#include "flashlight-core.h"
#include <linux/power_supply.h>
#endif

#ifndef OPLUS_FEATURE_CAMERA_COMMON
#define OPLUS_FEATURE_CAMERA_COMMON
#endif
#ifdef OPLUS_FEATURE_CAMERA_COMMON
#include <soc/oplus/system/oplus_project.h>
#endif

#define AW36410_BAIKALB5_CHANNEL_NUM  1
#define AW36410_BAIKALB5_LEVEL_TORCH 15
#define AW36410_BAIKALB5_LEVEL_NUM   32
#define AW36410_BAIKALB5_FLASH_TOUT_DEFAULT 600
#define AW36410_BAIKALB5_LEVEL_FLASH AW36410_BAIKALB5_LEVEL_NUM
#define FLASHLIGHT_NAME_SIZE 32
static struct hrtimer aw36410_baikalb5_timer;
static unsigned int aw36410_baikalb5_timeout_ms[AW36410_BAIKALB5_CHANNEL_NUM];
static const int aw36410_baikalb5_current[AW36410_BAIKALB5_LEVEL_NUM] = {
	  25,   50,  75, 85, 110, 150, 175,  200,  225,  250,
   //275,  300, 325, 350, 375, 400, 450,  500,  550,  600,
	275,  300, 325, 350, 375, 400, 450,  500,  575,  650,
  //650,  700, 750, 800, 850,  900,  950,  1000, 1050, 1100,
	725,  800, 875, 950, 1025, 1100, 1175, 1250, 1325, 1400,
  //1150, 1200
	1475, 1500
};

#define AW36410_BAIKALB5_NAME	"aw36410_baikalb5"
#define AW36410_BAIKALB5_I2C_ADDR	(0x63)

/* registers definitions */
#define AW36410_REG_ENABLE			0x01
#define AW36410_REG_LED0_FLASH_BR	0x03
#define AW36410_REG_LED0_TORCH_BR	0x05
#define AW36410_REG_FLASH_TOUT		0x08
#define AW36410_REG_SOFT_RST		0x07
#define AW36410_REG_FLAG 			0x0A
#define AW36410_REG_DEVICE_ID		0x0C
#define AW36410_DEVICE_ID			0x12
/* fault mask */
#define FAULT_TIMEOUT 				(1<<0)
#define FAULT_THERMAL_SHUTDOWN		(1<<2)
#define FAULT_LED0_SHORT_CIRCUIT	(1<<5)

/*  FLASH Brightness
 *	min 2.94 mA, step 7.83 mA, max 1500 mA
 */
#define AW36410_BAIKALB5_FLASH_BRT_MIN    2940
#define AW36410_BAIKALB5_FLASH_BRT_STEP   5870
#define AW36410_BAIKALB5_FLASH_BRT_MAX    1500000
#define AW36410_BAIKALB5_FLASH_BRT_uA_TO_REG(a)	\
	((a) < AW36410_BAIKALB5_FLASH_BRT_MIN ? 0 :	\
	(((a) - AW36410_BAIKALB5_FLASH_BRT_MIN) / AW36410_BAIKALB5_FLASH_BRT_STEP))
#define AW36410_BAIKALB5_FLASH_BRT_REG_TO_uA(a)	\
	((a) * AW36410_BAIKALB5_FLASH_BRT_STEP + AW36410_BAIKALB5_FLASH_BRT_MIN)

/*  FLASH TIMEOUT DURATION
 *	min 40ms, step 200ms, max 1600ms
 */
#define AW36410_BAIKALB5_FLASH_TOUT_MIN    40
#define AW36410_BAIKALB5_FLASH_TOUT_STEP   200
#define AW36410_BAIKALB5_FLASH_TOUT_MAX    1600

/*  TORCH BRT
 *	min 0.73 mA, step 1.47 mA, max 375 mA
 */
#define AW36410_BAIKALB5_TORCH_BRT_MIN     730
#define AW36410_BAIKALB5_TORCH_BRT_STEP    1470
#define AW36410_BAIKALB5_TORCH_BRT_MAX     375000
#define AW36410_BAIKALB5_TORCH_BRT_uA_TO_REG(a)	\
	((a) < AW36410_BAIKALB5_TORCH_BRT_MIN ? 0 :	\
	(((a) - AW36410_BAIKALB5_TORCH_BRT_MIN) / AW36410_BAIKALB5_TORCH_BRT_STEP))
#define AW36410_BAIKALB5_TORCH_BRT_REG_TO_uA(a)	\
	((a) * AW36410_BAIKALB5_TORCH_BRT_STEP + AW36410_BAIKALB5_TORCH_BRT_MIN)


#define SGM37863_BAIKAL_I2C_ADDR	(0x64)

/* registers definitions */
#define SGM37863_REG_ENABLE			0x01
#define SGM37863_REG_CONFIG			0x02
#define SGM37863_REG_LED0_FLASH_BR	0x03
#define SGM37863_REG_LED0_TORCH_BR	0x04
#define SGM37863_REG_FLAG 			0x05
#define SGM37863_REG_DEVICE_ID		0x06
#define SGM37863_DEVICE_ID			0x21

/*  FLASH Brightness
 *	min 11 mA, step 11.7 mA, max 1500 mA
 */
#define SGM37863_BAIKAL_FLASH_BRT_MIN  11000
#define SGM37863_BAIKAL_FLASH_BRT_STEP 11725
#define SGM37863_BAIKAL_FLASH_BRT_MAX  1500000
#define SGM37863_BAIKAL_FLASH_BRT_uA_TO_REG(a)	\
	((a) < SGM37863_BAIKAL_FLASH_BRT_MIN ? 0 :	\
	 (((a) - SGM37863_BAIKAL_FLASH_BRT_MIN) / SGM37863_BAIKAL_FLASH_BRT_STEP))
#define SGM37863_BAIKAL_FLASH_BRT_REG_TO_uA(a)		\
	((a) * SGM37863_BAIKAL_FLASH_BRT_STEP + SGM37863_BAIKAL_FLASH_BRT_MIN)

/*  FLASH TIMEOUT DURATION
 *	min 40ms, step 40、200ms, max 1600ms
 */
#define SGM37863_BAIKAL_FLASH_TOUT_MIN  40
#define SGM37863_BAIKAL_FLASH_TOUT_STEP 200
#define SGM37863_BAIKAL_FLASH_TOUT_MAX  1600

/*  TORCH BRT
 *	min 4 mA, step 3 mA, max 386 mA
 */
#define SGM37863_BAIKAL_TORCH_BRT_MIN  4000
#define SGM37863_BAIKAL_TORCH_BRT_STEP 3000
#define SGM37863_BAIKAL_TORCH_BRT_MAX  386000
#define SGM37863_BAIKAL_TORCH_BRT_uA_TO_REG(a)	\
	((a) < SGM37863_BAIKAL_TORCH_BRT_MIN ? 0 :	\
	 (((a) - SGM37863_BAIKAL_TORCH_BRT_MIN) / SGM37863_BAIKAL_TORCH_BRT_STEP))
#define SGM37863_BAIKAL_TORCH_BRT_REG_TO_uA(a)		\
	((a) * SGM37863_BAIKAL_TORCH_BRT_STEP + SGM37863_BAIKAL_TORCH_BRT_MIN)


enum flash_device_type {
	FLASH_DEVICE_AW36410 = 1,
	FLASH_DEVICE_SGM37863,
};
enum flash_device_type flash_device_info;

enum aw36410_baikalb5_led_id {
	AW36410_BAIKALB5_LED0 = 0,
	AW36410_BAIKALB5_LED_MAX,
};

enum flash_led_mode {
	FLASH_LED_MODE_NONE,
	FLASH_LED_MODE_FLASH,
	FLASH_LED_MODE_TORCH,
};

/* struct aw36410_baikalb5_platform_data
 *
 * @max_flash_timeout: flash timeout
 * @max_flash_brt: flash mode led brightness
 * @max_torch_brt: torch mode led brightness
 */
struct aw36410_baikalb5_platform_data {
	u32 max_flash_timeout;
	u32 max_flash_brt[AW36410_BAIKALB5_LED_MAX];
	u32 max_torch_brt[AW36410_BAIKALB5_LED_MAX];
};

enum led_enable {
	MODE_SHDN = 0x0,
	SGM37863_MODE_TORCH = 0x02,
	SGM37863_MODE_FLASH = 0x03,
	AW36410_MODE_TORCH = 0x08,
	AW36410_MODE_FLASH = 0x0C,
};

struct baikalb5_platform_data {
    int channel_num;
    struct flashlight_device_id *dev_id;
};

/**
 * struct aw36410_baikalb5_flash
 *
 * @dev: pointer to &struct device
 * @pdata: platform data
 * @regmap: reg. map for i2c
 * @lock: muxtex for serial access.
 * @led_mode: LED mode
 */
struct aw36410_baikalb5_flash {
	struct device *dev;
	struct aw36410_baikalb5_platform_data *pdata;
	struct regmap *regmap;
	struct mutex lock;

	enum flash_led_mode led_mode;
	struct device_node *dnode[AW36410_BAIKALB5_LED_MAX];
	struct pinctrl *aw36410_baikalb5_hwen_pinctrl;
	struct pinctrl_state *aw36410_baikalb5_hwen_high;
	struct pinctrl_state *aw36410_baikalb5_hwen_low;
#if IS_ENABLED(CONFIG_MTK_FLASHLIGHT)
	struct flashlight_device_id flash_dev_id[AW36410_BAIKALB5_LED_MAX];
#endif
	int level;
};

/* define usage count */
static int use_count;
static unsigned int flash_device_id;

static struct aw36410_baikalb5_flash *aw36410_baikalb5_flash_data;


/* define pinctrl */
#define AW36410_BAIKALB5_PINCTRL_PIN_HWEN 0
#define AW36410_BAIKALB5_PINCTRL_PINSTATE_LOW 0
#define AW36410_BAIKALB5_PINCTRL_PINSTATE_HIGH 1
#define AW36410_BAIKALB5_PINCTRL_STATE_HWEN_HIGH "hwen-high"
#define AW36410_BAIKALB5_PINCTRL_STATE_HWEN_LOW  "hwen-low"
/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
 /* define work queue */
static struct work_struct aw36410_baikalb5_work;
static enum hrtimer_restart aw36410_baikalb5_timer_func(struct hrtimer *timer)
{
	schedule_work(&aw36410_baikalb5_work);
	return HRTIMER_NORESTART;
}

static int aw36410_baikalb5_pinctrl_init(struct aw36410_baikalb5_flash *flash)
{
	int ret = 0;

	/* get pinctrl */
	flash->aw36410_baikalb5_hwen_pinctrl = devm_pinctrl_get(flash->dev);
	if (IS_ERR(flash->aw36410_baikalb5_hwen_pinctrl)) {
		pr_info("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(flash->aw36410_baikalb5_hwen_pinctrl);
		return ret;
	}

	/* Flashlight HWEN pin initialization */
	flash->aw36410_baikalb5_hwen_high = pinctrl_lookup_state(
			flash->aw36410_baikalb5_hwen_pinctrl,
			AW36410_BAIKALB5_PINCTRL_STATE_HWEN_HIGH);
	if (IS_ERR(flash->aw36410_baikalb5_hwen_high)) {
		pr_info("Failed to init (%s)\n",
			AW36410_BAIKALB5_PINCTRL_STATE_HWEN_HIGH);
		ret = PTR_ERR(flash->aw36410_baikalb5_hwen_high);
	}
	flash->aw36410_baikalb5_hwen_low = pinctrl_lookup_state(
			flash->aw36410_baikalb5_hwen_pinctrl,
			AW36410_BAIKALB5_PINCTRL_STATE_HWEN_LOW);
	if (IS_ERR(flash->aw36410_baikalb5_hwen_low)) {
		pr_info("Failed to init (%s)\n", AW36410_BAIKALB5_PINCTRL_STATE_HWEN_LOW);
		ret = PTR_ERR(flash->aw36410_baikalb5_hwen_low);
	}

	return ret;
}

static int aw36410_baikalb5_pinctrl_set(struct aw36410_baikalb5_flash *flash, int pin, int state)
{
	int ret = 0;

	if (IS_ERR(flash->aw36410_baikalb5_hwen_pinctrl)) {
		pr_info("pinctrl is not available\n");
		return -1;
	}

	switch (pin) {
	case AW36410_BAIKALB5_PINCTRL_PIN_HWEN:
		if (state == AW36410_BAIKALB5_PINCTRL_PINSTATE_LOW &&
				!IS_ERR(flash->aw36410_baikalb5_hwen_low))
			pinctrl_select_state(flash->aw36410_baikalb5_hwen_pinctrl,
					flash->aw36410_baikalb5_hwen_low);
		else if (state == AW36410_BAIKALB5_PINCTRL_PINSTATE_HIGH &&
				!IS_ERR(flash->aw36410_baikalb5_hwen_high))
			pinctrl_select_state(flash->aw36410_baikalb5_hwen_pinctrl,
					flash->aw36410_baikalb5_hwen_high);
		else
			pr_info("set err, pin(%d) state(%d)\n", pin, state);
		break;
	default:
		pr_info("set err, pin(%d) state(%d)\n", pin, state);
		break;
	}
	pr_info("pin(%d) state(%d)\n", pin, state);

	return ret;
}

/* enable mode control */
static int aw36410_baikalb5_mode_ctrl(struct aw36410_baikalb5_flash *flash)
{
	int rval = -EINVAL;

	pr_info("%s mode:%d", __func__, flash->led_mode);
	switch (flash->led_mode) {
	case FLASH_LED_MODE_NONE:
		if (flash_device_info == FLASH_DEVICE_AW36410)
			rval = regmap_update_bits(flash->regmap, AW36410_REG_ENABLE, 0x0C, MODE_SHDN);
		else
		    rval = regmap_update_bits(flash->regmap, SGM37863_REG_ENABLE, 0x03, MODE_SHDN);
		break;
	case FLASH_LED_MODE_TORCH:
		if (flash_device_info == FLASH_DEVICE_AW36410)
			rval = regmap_update_bits(flash->regmap, AW36410_REG_ENABLE, 0x0C, AW36410_MODE_TORCH);
		else
			rval = regmap_update_bits(flash->regmap, SGM37863_REG_ENABLE, 0x03, SGM37863_MODE_TORCH);
		break;
	case FLASH_LED_MODE_FLASH:
		if (flash_device_info == FLASH_DEVICE_AW36410)
			rval = regmap_update_bits(flash->regmap, AW36410_REG_ENABLE, 0x0C, AW36410_MODE_FLASH);
		else
			rval = regmap_update_bits(flash->regmap, SGM37863_REG_ENABLE, 0x03, SGM37863_MODE_FLASH);
		break;
	}
	return rval;
}

/* led1/2 enable/disable */
static int aw36410_baikalb5_enable_ctrl(struct aw36410_baikalb5_flash *flash,
			      enum aw36410_baikalb5_led_id led_no, bool on)
{
	int rval;

	pr_info("%s: enable:%d ", __func__, on);
	if (flash_device_info == FLASH_DEVICE_AW36410) {
		if (on) {
			rval = regmap_update_bits(flash->regmap,
					AW36410_REG_ENABLE, 0x03, 0x03);
		} else {
			rval = regmap_update_bits(flash->regmap,
					AW36410_REG_ENABLE, 0x03, 0x00);
		}
		pr_debug("%s: aw36410_return val:%d", __func__,  rval);
	} else if (flash_device_info == FLASH_DEVICE_SGM37863) {
		if (on) {
			rval = regmap_update_bits(flash->regmap,
					SGM37863_REG_ENABLE, 0x04, 0x04);
		} else {
			rval = regmap_update_bits(flash->regmap,
					SGM37863_REG_ENABLE, 0x04, 0x00);
		}
		pr_debug("%s: sgm37863_return val:%d", __func__,  rval);
	} else {
		rval = 0;
	}

	return rval;
}

/* torch1/2 brightness control */
static int aw36410_baikalb5_torch_brt_ctrl(struct aw36410_baikalb5_flash *flash,
				 enum aw36410_baikalb5_led_id led_no, unsigned int brt)
{
	int rval;
	u8 br_bits;

	pr_info("%s %d brt:%u", __func__, led_no, brt);
	if (flash_device_info == FLASH_DEVICE_AW36410) {
		if (brt < AW36410_BAIKALB5_TORCH_BRT_MIN)
			return aw36410_baikalb5_enable_ctrl(flash, led_no, false);

		if (brt >= AW36410_BAIKALB5_TORCH_BRT_MAX) {
			brt = AW36410_BAIKALB5_TORCH_BRT_MAX;
			pr_info("%s limit to torch max %d", __func__, AW36410_BAIKALB5_TORCH_BRT_MAX);
		}
		br_bits = AW36410_BAIKALB5_TORCH_BRT_uA_TO_REG(brt);
		rval = regmap_update_bits(flash->regmap,
					AW36410_REG_LED0_TORCH_BR, 0xFF, br_bits);
		pr_info("%s: aw36410_return val:%d  br_bits = 0x%x", __func__,  rval, br_bits);
	} else if (flash_device_info == FLASH_DEVICE_SGM37863) {
		if (brt < SGM37863_BAIKAL_TORCH_BRT_MIN)
			return aw36410_baikalb5_enable_ctrl(flash, led_no, false);

		if (brt >= SGM37863_BAIKAL_TORCH_BRT_MAX) {
			brt = SGM37863_BAIKAL_TORCH_BRT_MAX;
			pr_info("%s limit to torch max %d", __func__, SGM37863_BAIKAL_TORCH_BRT_MAX);
		}
		br_bits = SGM37863_BAIKAL_TORCH_BRT_uA_TO_REG(brt);
		rval = regmap_update_bits(flash->regmap,
					SGM37863_REG_LED0_TORCH_BR, 0x7F, br_bits);
	} else {
		rval = 0;
	}
	return rval;
}

/* flash1/2 brightness control */
static int aw36410_baikalb5_flash_brt_ctrl(struct aw36410_baikalb5_flash *flash,
				 enum aw36410_baikalb5_led_id led_no, unsigned int brt)
{
	int rval;
	u8 br_bits;

	pr_info("%s %d brt:%u", __func__, led_no, brt);
	if (flash_device_info == FLASH_DEVICE_AW36410) {
		pr_info("%s: decision aw36410_flash_device", __func__);
		if (brt < AW36410_BAIKALB5_FLASH_BRT_MIN)
			return aw36410_baikalb5_enable_ctrl(flash, led_no, false);

		if (brt >= AW36410_BAIKALB5_FLASH_BRT_MAX) {
			brt = AW36410_BAIKALB5_FLASH_BRT_MAX;
			pr_info("%s limit to flash max %d", __func__, AW36410_BAIKALB5_FLASH_BRT_MAX);
		}
		br_bits = AW36410_BAIKALB5_FLASH_BRT_uA_TO_REG(brt);
		rval = regmap_update_bits(flash->regmap, AW36410_REG_LED0_FLASH_BR, 0xFF, br_bits);
		pr_info("%s: aw36410 return val:%d  br_bits = 0x%x", __func__,  rval, br_bits);
	} else if (flash_device_info == FLASH_DEVICE_SGM37863) {
		pr_info("%s: decision sgm37863_flash_device", __func__);
		if (brt < SGM37863_BAIKAL_FLASH_BRT_MIN)
			return aw36410_baikalb5_enable_ctrl(flash, led_no, false);

		if (brt >= SGM37863_BAIKAL_FLASH_BRT_MAX) {
			brt = SGM37863_BAIKAL_FLASH_BRT_MAX;
			pr_info("%s limit to flash max %d", __func__, SGM37863_BAIKAL_FLASH_BRT_MAX);
		}

		br_bits = SGM37863_BAIKAL_FLASH_BRT_uA_TO_REG(brt);
		rval = regmap_update_bits(flash->regmap, SGM37863_REG_LED0_FLASH_BR, 0x7F, br_bits);
		pr_info("%s: sgm37863 return val:%d  br_bits = 0x%x", __func__,  rval, br_bits);
	} else {
		rval = 0;
		br_bits = 0;
	}
	return rval;
}

/* flash1/2 timeout control */
static int aw36410_baikalb5_flash_tout_ctrl(struct aw36410_baikalb5_flash *flash,
				unsigned int tout)
{
	int rval;
	u8 tout_bits = 0x00;

	if (tout > 1600) {
		tout = 1600;
		pr_info("%s: limit to max tout = %d", __func__, tout);
	}

	if (tout <= 40) {
		tout_bits = 0x00;
	} else if (tout <= 400) {
		tout_bits = (tout - 40) / 40;
	} else if (tout <= 1600) {
		tout_bits = (tout - 400) / 200 + 0x09;
	}
	if (flash_device_info == FLASH_DEVICE_AW36410) {
		rval = regmap_update_bits(flash->regmap, AW36410_REG_FLASH_TOUT, 0x0F, tout_bits);
	} else if (flash_device_info == FLASH_DEVICE_SGM37863) {
		rval = regmap_update_bits(flash->regmap, SGM37863_REG_CONFIG, 0x1E, tout_bits);
	} else {
		rval = 0;
	}
	pr_info("%s: tout = %d , tout_bits = 0x%x, ", __func__, tout, tout_bits);

	return rval;
}

static const struct regmap_config aw36410_baikalb5_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xF,
};

/* flashlight init */
static int aw36410_baikalb5_init(struct aw36410_baikalb5_flash *flash)
{
	int rval = 0;
	unsigned int reg_val;

	aw36410_baikalb5_pinctrl_set(flash, AW36410_BAIKALB5_PINCTRL_PIN_HWEN, AW36410_BAIKALB5_PINCTRL_PINSTATE_HIGH);

	/* set timeout */
	rval = aw36410_baikalb5_flash_tout_ctrl(flash, 400);
	if (rval < 0)
		return rval;
	/* output disable */
	flash->led_mode = FLASH_LED_MODE_NONE;
	rval = aw36410_baikalb5_mode_ctrl(flash);
	if (rval < 0)
		return rval;
	if (flash_device_info == FLASH_DEVICE_AW36410) {
		pr_info("%s: decision aw36410_flash_device", __func__);
		rval = regmap_update_bits(flash->regmap,
					AW36410_REG_LED0_TORCH_BR, 0xFF, 0x00);
		if (rval < 0)
			return rval;
		rval = regmap_update_bits(flash->regmap,
					AW36410_REG_LED0_FLASH_BR, 0xFF, 0x00);
		if (rval < 0)
			return rval;
		/* reset faults */
		rval = regmap_read(flash->regmap, AW36410_REG_FLAG, &reg_val);
	} else if (flash_device_info == FLASH_DEVICE_SGM37863) {
		pr_info("%s: decision sgm37863_flash_device", __func__);
		rval = regmap_update_bits(flash->regmap,
					SGM37863_REG_LED0_TORCH_BR, 0x7F, 0x00);
		if (rval < 0)
			return rval;
		rval = regmap_update_bits(flash->regmap,
					SGM37863_REG_LED0_FLASH_BR, 0x7F, 0x00);
		if (rval < 0)
			return rval;
		/* reset faults */
		rval = regmap_read(flash->regmap, SGM37863_REG_FLAG, &reg_val);
	}
	return rval;
}

/* flashlight uninit */
static int aw36410_baikalb5_uninit(struct aw36410_baikalb5_flash *flash)
{
	aw36410_baikalb5_pinctrl_set(flash,
			AW36410_BAIKALB5_PINCTRL_PIN_HWEN, AW36410_BAIKALB5_PINCTRL_PINSTATE_LOW);
	return 0;
}

static int aw36410_baikalb5_flash_open(void)
{
	int rval = -EINVAL;
	unsigned int flash_val,torch_val;
	rval = regmap_read(aw36410_baikalb5_flash_data->regmap, AW36410_REG_DEVICE_ID, &flash_device_id);

	if (flash_device_id == AW36410_DEVICE_ID) {
		pr_info("%s: aw36410_flash_device_id :%x", __func__, flash_device_id);
		flash_device_info = FLASH_DEVICE_AW36410;
	} else {
		rval = regmap_read(aw36410_baikalb5_flash_data->regmap, SGM37863_REG_DEVICE_ID, &flash_device_id);
		pr_info("%s: sgm37863_flash_device_id :%x", __func__, flash_device_id);
		flash_device_info = FLASH_DEVICE_SGM37863;
	}

	if (flash_device_info == FLASH_DEVICE_AW36410) {
		rval = regmap_read(aw36410_baikalb5_flash_data->regmap, AW36410_REG_LED0_FLASH_BR, &flash_val);
		rval = regmap_read(aw36410_baikalb5_flash_data->regmap, AW36410_REG_LED0_TORCH_BR, &torch_val);
		rval = regmap_update_bits(aw36410_baikalb5_flash_data->regmap, AW36410_REG_SOFT_RST, 0x80, 0x80);

		msleep(2);
		rval = regmap_update_bits(aw36410_baikalb5_flash_data->regmap, AW36410_REG_LED0_FLASH_BR, 0xFF, flash_val);//flash current
		rval = regmap_update_bits(aw36410_baikalb5_flash_data->regmap, AW36410_REG_LED0_TORCH_BR, 0xFF, torch_val);//torch current
	} else if (flash_device_info == FLASH_DEVICE_SGM37863) {
		rval = regmap_read(aw36410_baikalb5_flash_data->regmap, SGM37863_REG_LED0_FLASH_BR, &flash_val);
		rval = regmap_read(aw36410_baikalb5_flash_data->regmap, SGM37863_REG_LED0_TORCH_BR, &torch_val);

		msleep(2);
		rval = regmap_update_bits(aw36410_baikalb5_flash_data->regmap, SGM37863_REG_LED0_FLASH_BR, 0x7F, flash_val);//flash current
		rval = regmap_update_bits(aw36410_baikalb5_flash_data->regmap, SGM37863_REG_LED0_TORCH_BR, 0x7F, torch_val);//torch current
	}
	return 0;
}

static int aw36410_baikalb5_flash_release(void)
{
	return 0;
}

static int aw36410_baikalb5_is_torch(int level)
{
	if (level >= AW36410_BAIKALB5_LEVEL_TORCH)
		return -1;

	return 0;
}
static int aw36410_baikalb5_operate(int channel, int enable)
{
	ktime_t ktime;
	unsigned int s;
	unsigned int ns;

	if (enable && (aw36410_baikalb5_flash_data->level >= 0)) {
		/* set timer */
		if (aw36410_baikalb5_timeout_ms[channel]) {
			s = aw36410_baikalb5_timeout_ms[channel] / 1000;
			ns = aw36410_baikalb5_timeout_ms[channel] % 1000 * 1000000;
			ktime = ktime_set(s, ns);
			hrtimer_start(&aw36410_baikalb5_timer, ktime,
					HRTIMER_MODE_REL);
		}

		if (!aw36410_baikalb5_is_torch(aw36410_baikalb5_flash_data->level)) {
			aw36410_baikalb5_flash_data->led_mode = FLASH_LED_MODE_TORCH;
		} else {
			aw36410_baikalb5_flash_data->led_mode = FLASH_LED_MODE_FLASH;
		}
		aw36410_baikalb5_mode_ctrl(aw36410_baikalb5_flash_data);
		aw36410_baikalb5_enable_ctrl(aw36410_baikalb5_flash_data, channel, true);
	} else {
		aw36410_baikalb5_flash_data->led_mode = FLASH_LED_MODE_NONE;
		aw36410_baikalb5_mode_ctrl(aw36410_baikalb5_flash_data);
		aw36410_baikalb5_enable_ctrl(aw36410_baikalb5_flash_data, channel, false);
		hrtimer_cancel(&aw36410_baikalb5_timer);
	}
	return 0;
}
static int aw36410_baikalb5_verify_level(int level)
{
	if (level < 0)
		level = 0;
	else if (level >= AW36410_BAIKALB5_LEVEL_NUM)
		level = AW36410_BAIKALB5_LEVEL_NUM - 1;

	return level;
}
static int aw36410_baikalb5_set_level(int channel, int level)
{
	aw36410_baikalb5_flash_data->level = aw36410_baikalb5_verify_level(level);
	if (!aw36410_baikalb5_is_torch(aw36410_baikalb5_flash_data->level)) {
		aw36410_baikalb5_torch_brt_ctrl(aw36410_baikalb5_flash_data, channel,
				1000 * aw36410_baikalb5_current[aw36410_baikalb5_flash_data->level]);
	} else {
		aw36410_baikalb5_flash_brt_ctrl(aw36410_baikalb5_flash_data, channel,
				1000 * aw36410_baikalb5_current[aw36410_baikalb5_flash_data->level]);
	}
	return 0;
}
static int aw36410_baikalb5_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	switch (cmd) {
	case FLASH_IOC_SET_ONOFF:
		pr_info("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		aw36410_baikalb5_operate(channel, (int)fl_arg->arg);
		break;

	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		pr_debug("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		aw36410_baikalb5_timeout_ms[channel] = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		pr_info("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		aw36410_baikalb5_set_level(channel, (int)fl_arg->arg);
		break;

	case FLASH_IOC_GET_DUTY_NUMBER:
		pr_info("FLASH_IOC_GET_DUTY_NUMBER(%d): %d\n",
				channel, (int)fl_arg->arg);
		fl_arg->arg = AW36410_BAIKALB5_LEVEL_NUM;
		break;

	case FLASH_IOC_GET_MAX_TORCH_DUTY:
		pr_info("FLASH_IOC_GET_MAX_TORCH_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		fl_arg->arg = AW36410_BAIKALB5_LEVEL_TORCH - 1;
		break;

	case FLASH_IOC_GET_DUTY_CURRENT:
		pr_info("FLASH_IOC_GET_DUTY_CURRENT(%d): %d\n",
				channel, (int)fl_arg->arg);
		fl_arg->arg = aw36410_baikalb5_current[(int)fl_arg->arg];
		break;

	case FLASH_IOC_GET_HW_TIMEOUT:
		pr_debug("FLASH_IOC_GET_HW_TIMEOUT(%d)\n", channel);
		fl_arg->arg = AW36410_BAIKALB5_FLASH_TOUT_DEFAULT;
		break;

	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int aw36410_baikalb5_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	//mutex_lock(&aw36410_baikalb5_mutex);
	if (set) {
		if (!use_count)
			ret = aw36410_baikalb5_init(aw36410_baikalb5_flash_data);
		use_count++;
		pr_debug("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = aw36410_baikalb5_uninit(aw36410_baikalb5_flash_data);
		if (use_count < 0)
			use_count = 0;
		pr_debug("Unset driver: %d\n", use_count);
	}
	//mutex_unlock(&aw36410_baikalb5_mutex);

	return 0;
}

static ssize_t aw36410_baikalb5_strobe_store(struct flashlight_arg arg)
{
	pr_debug("Unset driver: %d\n", use_count);
	aw36410_baikalb5_set_driver(1);
	//aw36410_baikalb5_set_level(arg.channel, arg.level);
	//aw36410_baikalb5_timeout_ms[arg.channel] = 0;
	//aw36410_baikalb5_enable(arg.channel);
	aw36410_baikalb5_torch_brt_ctrl(aw36410_baikalb5_flash_data, arg.channel,
				arg.level * 25000);
	aw36410_baikalb5_flash_data->led_mode = FLASH_LED_MODE_TORCH;
	aw36410_baikalb5_mode_ctrl(aw36410_baikalb5_flash_data);
	aw36410_baikalb5_enable_ctrl(aw36410_baikalb5_flash_data, arg.channel, true);
	msleep(arg.dur);
	//aw36410_baikalb5_disable(arg.channel);
	aw36410_baikalb5_flash_data->led_mode = FLASH_LED_MODE_NONE;
	aw36410_baikalb5_mode_ctrl(aw36410_baikalb5_flash_data);
	aw36410_baikalb5_enable_ctrl(aw36410_baikalb5_flash_data, arg.channel, false);
	aw36410_baikalb5_set_driver(0);
	return 0;
}

static struct flashlight_operations aw36410_baikalb5_flash_ops = {
	aw36410_baikalb5_flash_open,
	aw36410_baikalb5_flash_release,
	aw36410_baikalb5_ioctl,
	aw36410_baikalb5_strobe_store,
	aw36410_baikalb5_set_driver
};

static int aw36410_baikalb5_parse_dt(struct aw36410_baikalb5_flash *flash)
{
	struct device_node *np, *cnp;
	struct device *dev = flash->dev;
	u32 decouple = 0;
	int i = 0;

	if (!dev || !dev->of_node)
		return -ENODEV;

	np = dev->of_node;
	for_each_child_of_node(np, cnp) {
		if (of_property_read_u32(cnp, "type",
					&flash->flash_dev_id[i].type))
			goto err_node_put;
		if (of_property_read_u32(cnp,
					"ct", &flash->flash_dev_id[i].ct))
			goto err_node_put;
		if (of_property_read_u32(cnp,
					"part", &flash->flash_dev_id[i].part))
			goto err_node_put;
		flash->flash_dev_id[i].channel = i;
		flash->flash_dev_id[i].decouple = decouple;

		pr_info("Parse dt (type,ct,part,name,channel,decouple)=(%d,%d,%d,%s,%d,%d).\n",
				flash->flash_dev_id[i].type,
				flash->flash_dev_id[i].ct,
				flash->flash_dev_id[i].part,
				flash->flash_dev_id[i].name,
				flash->flash_dev_id[i].channel,
				flash->flash_dev_id[i].decouple);
		if (flashlight_dev_register_by_device_id(&flash->flash_dev_id[i],
			&aw36410_baikalb5_flash_ops))
			return -EFAULT;
		i++;
	}

	return 0;

err_node_put:
	of_node_put(cnp);
	return -EINVAL;
}

static int aw36410_baikalb5_probe(struct i2c_client *client,
			const struct i2c_device_id *devid)
{
	struct aw36410_baikalb5_flash *flash;
	struct aw36410_baikalb5_platform_data *pdata = dev_get_platdata(&client->dev);
	int rval;
	int ret;
	int err = 0;
    bool curProject = false;
	pr_info("%s:%d", __func__, __LINE__);

    pr_info("i2c_probe Probe start.\n");
    curProject = (is_project(24366) || is_project(24361) || is_project(24368));

    if (!curProject) {
        err = -ENODEV;
        goto err_out;
    }

	flash = devm_kzalloc(&client->dev, sizeof(*flash), GFP_KERNEL);
	if (flash == NULL)
		return -ENOMEM;

	ret = i2c_smbus_read_byte_data(client, AW36410_REG_DEVICE_ID);
	if (ret == AW36410_DEVICE_ID) {
		flash_device_info = FLASH_DEVICE_AW36410;
		pr_info("%s: read aw36410 id success, id=%x", __func__, ret);
	} else {
		pr_info("%s: read aw36410 id fail", __func__);
		client->addr = SGM37863_BAIKAL_I2C_ADDR;
		ret = i2c_smbus_read_byte_data(client, SGM37863_REG_DEVICE_ID);
		if (ret == SGM37863_DEVICE_ID) {
			flash_device_info = FLASH_DEVICE_SGM37863;
			pr_info("%s: read sgm37863 id success, id=%x", __func__, ret);
		} else {
			pr_info("%s: read sgm37863 id fail", __func__);
		}
	}
	pr_info("%s: client->addr is:%x", __func__, client->addr);

	flash->regmap = devm_regmap_init_i2c(client, &aw36410_baikalb5_regmap);
	if (IS_ERR(flash->regmap)) {
		rval = PTR_ERR(flash->regmap);
		return rval;
	}

	/* if there is no platform data, use chip default value */
	if (pdata == NULL) {
		pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
		if (pdata == NULL)
			return -ENODEV;
		if (flash_device_info == FLASH_DEVICE_AW36410) {
			pdata->max_flash_timeout = AW36410_BAIKALB5_FLASH_TOUT_MAX;
			/* led 1 */
			pdata->max_flash_brt[AW36410_BAIKALB5_LED0] = AW36410_BAIKALB5_FLASH_BRT_MAX;
			pdata->max_torch_brt[AW36410_BAIKALB5_LED0] = AW36410_BAIKALB5_TORCH_BRT_MAX;
		} else if (flash_device_info == FLASH_DEVICE_SGM37863) {
			pdata->max_flash_timeout = SGM37863_BAIKAL_FLASH_TOUT_MAX;
			/* led 1 */
			pdata->max_flash_brt[AW36410_BAIKALB5_LED0] = SGM37863_BAIKAL_FLASH_BRT_MAX;
			pdata->max_torch_brt[AW36410_BAIKALB5_LED0] = SGM37863_BAIKAL_TORCH_BRT_MAX;
		}
	}

	/* init timer */
	hrtimer_init(&aw36410_baikalb5_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw36410_baikalb5_timer.function = aw36410_baikalb5_timer_func;
	aw36410_baikalb5_timeout_ms[0] = AW36410_BAIKALB5_FLASH_TOUT_DEFAULT;

	/* register flashlight operations */
	if (flashlight_dev_register(AW36410_BAIKALB5_NAME, &aw36410_baikalb5_flash_ops)) {
		pr_err("Failed to register flashlight device.\n");
		err = -EFAULT;
		return 0;
	}

	flash->pdata = pdata;
	flash->dev = &client->dev;
	mutex_init(&flash->lock);
	aw36410_baikalb5_flash_data = flash;

	rval = aw36410_baikalb5_pinctrl_init(flash);

	pm_runtime_enable(flash->dev);

	rval = aw36410_baikalb5_parse_dt(flash);

	i2c_set_clientdata(client, flash);

	if (flash_device_info == FLASH_DEVICE_AW36410) {
		rval = regmap_update_bits(flash->regmap, AW36410_REG_LED0_TORCH_BR, 0xFF, 0x33);//torch current 75mA
	} else if (flash_device_info == FLASH_DEVICE_SGM37863) {
		rval = regmap_update_bits(flash->regmap, SGM37863_REG_LED0_TORCH_BR, 0x7F, 0x18);//torch current 75mA
	}
	pr_info("%s:%d", __func__, __LINE__);

	return 0;

err_out:
    return err;
}

static int aw36410_baikalb5_remove(struct i2c_client *client)
{
    struct aw36410_baikalb5_flash *flash = i2c_get_clientdata(client);

    pr_info("%s: remove", __func__);

    //Cancel work queue
    cancel_work_sync(&aw36410_baikalb5_work);

    //Unregister flashlight device
    flashlight_dev_unregister(AW36410_BAIKALB5_NAME);

    //Stop and clean up timer
    hrtimer_cancel(&aw36410_baikalb5_timer);

    //Destroy mutex
    mutex_destroy(&flash->lock);

    //Release pinctrl resources
    if (flash->aw36410_baikalb5_hwen_pinctrl) {
        devm_pinctrl_put(flash->aw36410_baikalb5_hwen_pinctrl);
        flash->aw36410_baikalb5_hwen_pinctrl = NULL;
    }

    pm_runtime_disable(&client->dev);
    pm_runtime_set_suspended(&client->dev);

	return 0;
}

static const struct i2c_device_id aw36410_baikalb5_id_table[] = {
	{AW36410_BAIKALB5_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, aw36410_baikalb5_id_table);

static const struct of_device_id aw36410_baikalb5_of_table[] = {
	{ .compatible = "mediatek,aw36410_baikalb5" },
	{ },
};
MODULE_DEVICE_TABLE(of, aw36410_baikalb5_of_table);

static struct i2c_driver aw36410_baikalb5_i2c_driver = {
	.driver = {
		   .name = AW36410_BAIKALB5_NAME,
		   .of_match_table = aw36410_baikalb5_of_table,
		   },
	.probe = aw36410_baikalb5_probe,
	.remove = aw36410_baikalb5_remove,
	.id_table = aw36410_baikalb5_id_table,
};

module_i2c_driver(aw36410_baikalb5_i2c_driver);

MODULE_AUTHOR("XXX");
MODULE_DESCRIPTION("aw36410_baikalb5 LED flash driver");
MODULE_LICENSE("GPL");

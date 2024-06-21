/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/pinctrl/consumer.h>

#include "flashlight-core.h"
#include "flashlight-dt.h"
#include <mt-plat/mtk_pwm.h>
#include <mt-plat/mtk_pwm_hal_pub.h>
#define CONFIG_PM_WAKELOCKS 1

#ifdef OPLUS_FEATURE_CAMERA_COMMON
#include <soc/oplus/system/oplus_project.h>
#include <soc/oplus/system/oplus_project_oldcdt.h>
#endif

/* define device tree */
#ifndef BARLEY_DTNAME
#define BARLEY_DTNAME "mediatek,flashlights_barley"
#endif

#define BARLEY_NAME "flashlights_barley"

#ifndef FLASHLIGHT_BRIGHTNESS_ADD
#define FLASHLIGHT_BRIGHTNESS_ADD
#endif

#ifdef CONFIG_PM_WAKELOCKS
#include <linux/pm_wakeup.h>
#endif

/* define registers */

/* define mutex and work queue */
static DEFINE_MUTEX(barley_mutex);
static struct work_struct barley_work;
//static struct pwm_spec_config pwm_setting;

#define BARLEY_LEVEL_NUM 26
#define BARLEY_LEVEL_TORCH 9
											//-----------------------------------------------------------------flash mode-----------------------|
static int g_duty_array[BARLEY_LEVEL_NUM] = {4, 8, 11, 14, 17, 20, 23,26, 29, 32, 36, 40, 44, 48, 52, 56, 60, 64, 68, 72, 76, 80, 84, 88, 92, 96};
												  //|------toruch mode 40ma-300ma  ----|
static int g_torch_duty_array[BARLEY_LEVEL_TORCH] = { 4, 14, 23, 32, 44, 56, 68, 80, 88};

static const int *barley_current;
                                                  // |---torch mode 40ma ~ 300ma ----|------------------------flash mode-----------------------------------|
static const int sgm3785_current[BARLEY_LEVEL_NUM] = {40,76,108,140,176,210,248,280,300,320,360,400,440,480,520,560,600,640,680,720,760,800,840,880,920,960};

/* define pinctrl */
#define BARLEY_PINCTRL_PIN_FLASH_EN 0
#define BARLEY_PINCTRL_PIN_PWM_EN 1
#define BARLEY_PINCTRL_PIN_PWM_GPIO 2
#define BARLEY_PINCTRL_PIN_STATE_LOW 0
#define BARLEY_PINCTRL_PIN_STATE_HIGH 1
#define BARLEY_PINCTRL_STATE_FLASH_EN_HIGH "flash_light_en_pin_1"  //ENF 55
#define BARLEY_PINCTRL_STATE_FLASH_EN_LOW  "flash_light_en_pin_0"

#define BARLEY_PINCTRL_STATE_PWM_GPIO_HIGH "flash_light_flash_pin_1"//ENM 147
#define BARLEY_PINCTRL_STATE_PWM_GPIO_LOW  "flash_light_flash_pin_0"
#define BARLEY_PINCTRL_STATE_PWM "flash_light_pwm_pin"	//ENM PWM 147

static struct pinctrl *barley_pinctrl;
static struct pinctrl_state *barley_flash_en_high;
static struct pinctrl_state *barley_flash_en_low;
static struct pinctrl_state *barley_pwm_gpio_high;
static struct pinctrl_state *barley_pwm_gpio_low;
static struct pinctrl_state *barley_flash_pwm;

/* define usage count */
static int use_count;

static int g_flash_duty = -1;

#ifdef CONFIG_PM_WAKELOCKS
struct wakeup_source flashlight_wake_lock;
#endif

/* platform data */
struct barley_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};


/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
static int barley_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;
	if(barley_pinctrl == NULL){
	/* get pinctrl */
	barley_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(barley_pinctrl)) {
		pr_info("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(barley_pinctrl);
		barley_pinctrl = NULL ;
		return ret;
	}

	/*  Flashlight pin initialization */
	barley_flash_en_high = pinctrl_lookup_state(
			barley_pinctrl, BARLEY_PINCTRL_STATE_FLASH_EN_HIGH);
	if (IS_ERR(barley_flash_en_high)) {
		pr_info("Failed to init (%s)\n", BARLEY_PINCTRL_STATE_FLASH_EN_HIGH);
		ret = PTR_ERR(barley_flash_en_high);
	}
	barley_flash_en_low = pinctrl_lookup_state(
			barley_pinctrl, BARLEY_PINCTRL_STATE_FLASH_EN_LOW);
	if (IS_ERR(barley_flash_en_low)) {
		pr_info("Failed to init (%s)\n", BARLEY_PINCTRL_STATE_FLASH_EN_LOW);
		ret = PTR_ERR(barley_flash_en_low);
	}
	barley_pwm_gpio_high = pinctrl_lookup_state(
			barley_pinctrl, BARLEY_PINCTRL_STATE_PWM_GPIO_HIGH);
	if (IS_ERR(barley_pwm_gpio_high)) {
		pr_info("Failed to init (%s)\n", BARLEY_PINCTRL_STATE_PWM_GPIO_HIGH);
		ret = PTR_ERR(barley_pwm_gpio_high);
	}
	barley_pwm_gpio_low = pinctrl_lookup_state(
			barley_pinctrl, BARLEY_PINCTRL_STATE_PWM_GPIO_LOW);
	if (IS_ERR(barley_pwm_gpio_low)) {
		pr_info("Failed to init (%s)\n", BARLEY_PINCTRL_STATE_PWM_GPIO_LOW);
		ret = PTR_ERR(barley_pwm_gpio_low);
	}
	barley_flash_pwm = pinctrl_lookup_state(
			barley_pinctrl, BARLEY_PINCTRL_STATE_PWM);
	if (IS_ERR(barley_flash_pwm)) {
		pr_info("Failed to init (%s)\n", BARLEY_PINCTRL_STATE_PWM);
		ret = PTR_ERR(barley_flash_pwm);
	}
	}
	return ret;
}

static int barley_pinctrl_set(int pin, int state)
{
	int ret = 0;

	if (IS_ERR(barley_pinctrl)) {
		pr_info("pinctrl is not available\n");
		return -1;
	}

	switch (pin) {
		case BARLEY_PINCTRL_PIN_FLASH_EN:
			if (state == BARLEY_PINCTRL_PIN_STATE_LOW &&!IS_ERR(barley_flash_en_low))
				ret = pinctrl_select_state(barley_pinctrl, barley_flash_en_low);
			else if (state == BARLEY_PINCTRL_PIN_STATE_HIGH &&!IS_ERR(barley_flash_en_high))
				ret = pinctrl_select_state(barley_pinctrl, barley_flash_en_high);
			else
				pr_info("set err, pin(%d) state(%d)\n", pin, state);
			break;
		case BARLEY_PINCTRL_PIN_PWM_EN:
			ret =pinctrl_select_state(barley_pinctrl, barley_flash_pwm);
			break;
		case BARLEY_PINCTRL_PIN_PWM_GPIO:
			if (state == BARLEY_PINCTRL_PIN_STATE_LOW &&!IS_ERR(barley_pwm_gpio_low))
				ret = pinctrl_select_state(barley_pinctrl, barley_pwm_gpio_low);
			else if (state == BARLEY_PINCTRL_PIN_STATE_HIGH &&!IS_ERR(barley_pwm_gpio_high))
				ret = pinctrl_select_state(barley_pinctrl, barley_pwm_gpio_high);
			else
				pr_info("set err, pin(%d) state(%d)\n", pin, state);
			break;
		default:
			pr_info("set err, pin(%d) state(%d)\n", pin, state);
			break;
	}
	pr_info("pin(%d) state(%d), ret:%d\n", pin, state, ret);

	return ret;
}


/* flashlight enable  pwm function */
/* 52M/32/100 = 16KHZ  actually 21KHZ for SGM3785*/
int mt_flashlight_led_set_pwm(int pwm_num,u32 level )
{
	struct pwm_spec_config pwm_setting;
	memset(&pwm_setting, 0, sizeof(struct pwm_spec_config));
	pwm_setting.pwm_no = pwm_num; /* PWM0 set 0,PWM1 set 1,PWM2 set 2,PWM3 set 3 */
	pwm_setting.mode = PWM_MODE_OLD;
	pwm_setting.pmic_pad = 0;
	pwm_setting.clk_div = CLK_DIV16;
	pwm_setting.clk_src = PWM_CLK_OLD_MODE_BLOCK;
	pwm_setting.PWM_MODE_OLD_REGS.IDLE_VALUE = 0;
	pwm_setting.PWM_MODE_OLD_REGS.GUARD_VALUE = 0;
	pwm_setting.PWM_MODE_OLD_REGS.GDURATION = 0;
	pwm_setting.PWM_MODE_OLD_REGS.WAVE_NUM = 0;
	pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH = 100;
	pwm_setting.PWM_MODE_OLD_REGS.THRESH = level;
	mt_pwm_clk_sel_hal(0,CLK_26M);
	pwm_set_spec_config(&pwm_setting);

	return 0;
}



/******************************************************************************
 * barley operations
 *****************************************************************************/
static int barley_verify_level(int level)
{
	if (level < 0)
		level = 0;
	else if (level >= BARLEY_LEVEL_NUM)
		level = BARLEY_LEVEL_NUM - 1;

	return level;
}

static int barley_is_torch(int level)
{
	if (level >= BARLEY_LEVEL_TORCH)
		return -1;

	return 0;
}

/* flashlight enable function */
#ifdef FLASHLIGHT_BRIGHTNESS_ADD
bool fl_state=false;
#endif

static int barley_enable(void)
{
	int tempPWM = 0;

 #ifdef CONFIG_PM_WAKELOCKS
 	__pm_stay_awake(&flashlight_wake_lock);
 #endif

#ifdef FLASHLIGHT_BRIGHTNESS_ADD
	if (fl_state){
		g_flash_duty = BARLEY_LEVEL_NUM - 1;
		pr_info("fl_state=true\n");
	}
#endif

	if (!barley_is_torch(g_flash_duty)) { //troch mode  40ma - 520ma
		if (g_flash_duty <0)
			tempPWM = g_torch_duty_array[0];
		else
			tempPWM = g_torch_duty_array[g_flash_duty];

		barley_pinctrl_set(BARLEY_PINCTRL_PIN_FLASH_EN,0);	//pull down ENF
		barley_pinctrl_set(BARLEY_PINCTRL_PIN_PWM_GPIO, 1);	//pull up ENM
		mdelay(5);											//delay more than 5ms
		barley_pinctrl_set(BARLEY_PINCTRL_PIN_PWM_EN,1);	//set pwm mode
		mt_flashlight_led_set_pwm(0,tempPWM);
		pr_info("BARLEY Flash torch mode %s g_flash_duty = %d,%d\n",__FUNCTION__,g_flash_duty,tempPWM);

	} else {//flash mode  520ma - 960ma
		if ( g_flash_duty < BARLEY_LEVEL_NUM)
			tempPWM = g_duty_array[g_flash_duty];
		else
			tempPWM = g_duty_array[BARLEY_LEVEL_NUM - 1];

		barley_pinctrl_set(BARLEY_PINCTRL_PIN_FLASH_EN,0);	//pull down ENF
		barley_pinctrl_set(BARLEY_PINCTRL_PIN_PWM_EN,1);	//set pwm mode
		mt_flashlight_led_set_pwm(0,tempPWM);				//flash pwm tempPWM%
		udelay(5);
		barley_pinctrl_set(BARLEY_PINCTRL_PIN_FLASH_EN,1);
		pr_info("BARLEY Flash flash mode %s g_flash_duty = %d,%d\n",__FUNCTION__,g_flash_duty,tempPWM);
	}
	return 0;
}

/* flashlight disable function */
static int barley_disable(void)
{
	int state = BARLEY_PINCTRL_PIN_STATE_LOW;

	barley_pinctrl_set(BARLEY_PINCTRL_PIN_FLASH_EN, state);
	barley_pinctrl_set(BARLEY_PINCTRL_PIN_PWM_GPIO, state);
	mt_pwm_disable(0,0);
 #ifdef CONFIG_PM_WAKELOCKS
 	__pm_relax(&flashlight_wake_lock);
 #endif

	return 0;
}

#ifdef FLASHLIGHT_BRIGHTNESS_ADD
int barley_enable_flash(int level)
{
	if (level==0) {
		fl_state=false;
		pr_info("flash mode close\n");
	}
	else {
		fl_state=true;//flash 1A
		pr_info("flash mode open\n");
	}
	return 0;
}
EXPORT_SYMBOL(barley_enable_flash);
#endif

/* set flashlight level */
static int barley_set_level(int level)
{
	g_flash_duty = level;
	return 0;
}

/* flashlight init */
static int barley_init(void)
{
	int state = BARLEY_PINCTRL_PIN_STATE_LOW;

	barley_pinctrl_set(BARLEY_PINCTRL_PIN_FLASH_EN, state);
	barley_pinctrl_set(BARLEY_PINCTRL_PIN_PWM_GPIO, state);
	return 0;
}

/* flashlight uninit */
static int barley_uninit(void)
{
	int state = BARLEY_PINCTRL_PIN_STATE_LOW;

	barley_pinctrl_set(BARLEY_PINCTRL_PIN_FLASH_EN, state);
	barley_pinctrl_set(BARLEY_PINCTRL_PIN_PWM_GPIO, state);
	return 0;
}

/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer barley_timer;
static unsigned int barley_timeout_ms;

static void barley_work_disable(struct work_struct *data)
{
	pr_info("work queue callback\n");
	barley_disable();
}

static enum hrtimer_restart barley_timer_func(struct hrtimer *timer)
{
	schedule_work(&barley_work);
	return HRTIMER_NORESTART;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int barley_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	ktime_t ktime;
	unsigned int s;
	unsigned int ns;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;
    /* verify channel */
    if (channel < 0 || channel >= 1) {
        pr_err("Failed with error channel\n");
        return -EINVAL;
    }
	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		pr_info("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		barley_timeout_ms = 0;//fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		pr_info("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		barley_set_level(fl_arg->arg);
		break;
	case FLASH_IOC_SET_ONOFF:
		pr_info("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (barley_timeout_ms) {
				s = barley_timeout_ms / 1000;
				ns = barley_timeout_ms % 1000 * 1000000;
				ktime = ktime_set(s, ns);
				hrtimer_start(&barley_timer, ktime,
						HRTIMER_MODE_REL);
			}
			barley_enable();
		} else {
			barley_disable();
			hrtimer_cancel(&barley_timer);
		}
		break;
	case FLASH_IOC_GET_DUTY_NUMBER:
		pr_info("FLASH_IOC_GET_DUTY_NUMBER(%d)\n", channel);
		fl_arg->arg = BARLEY_LEVEL_NUM;
		break;

	case FLASH_IOC_GET_MAX_TORCH_DUTY:
		pr_info("FLASH_IOC_GET_MAX_TORCH_DUTY(%d)\n", channel);
		fl_arg->arg = BARLEY_LEVEL_TORCH - 1;
		break;

	case FLASH_IOC_GET_DUTY_CURRENT:
		fl_arg->arg = barley_verify_level(fl_arg->arg);
		pr_info("FLASH_IOC_GET_DUTY_CURRENT(%d): %d\n",
				channel, (int)fl_arg->arg);
		fl_arg->arg = barley_current[fl_arg->arg];
		break;

	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int barley_open(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int barley_release(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int barley_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&barley_mutex);
	if (set) {
		if (!use_count)
			ret = barley_init();
		use_count++;
		pr_info("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = barley_uninit();
		if (use_count < 0)
			use_count = 0;
		pr_info("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&barley_mutex);

	return ret;
}

static ssize_t barley_strobe_store(struct flashlight_arg arg)
{
	barley_set_driver(1);
	barley_set_level(arg.level);
	barley_timeout_ms = 0;
	barley_enable();
	msleep(arg.dur);
	barley_disable();
	barley_set_driver(0);

	return 0;
}

static struct flashlight_operations barley_ops = {
	barley_open,
	barley_release,
	barley_ioctl,
	barley_strobe_store,
	barley_set_driver
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int barley_chip_init(void)
{
	/* NOTE: Chip initialication move to "set driver" for power saving.
	 * barley_init();
	 */

	return 0;
}

static int barley_parse_dt(struct device *dev,
		struct barley_platform_data *pdata)
{
	struct device_node *np, *cnp;
	u32 decouple = 0;
	int i = 0;

	if (!dev || !dev->of_node || !pdata)
		return -ENODEV;

	np = dev->of_node;

	pdata->channel_num = of_get_child_count(np);
	if (!pdata->channel_num) {
		pr_info("Parse no dt, node.\n");
		return 0;
	}
	pr_info("Channel number(%d).\n", pdata->channel_num);

	if (of_property_read_u32(np, "decouple", &decouple))
		pr_info("Parse no dt, decouple.\n");

	pdata->dev_id = devm_kzalloc(dev,
			pdata->channel_num *
			sizeof(struct flashlight_device_id),
			GFP_KERNEL);
	if (!pdata->dev_id)
		return -ENOMEM;

	for_each_child_of_node(np, cnp) {
		if (of_property_read_u32(cnp, "type", &pdata->dev_id[i].type))
			goto err_node_put;
		if (of_property_read_u32(cnp, "ct", &pdata->dev_id[i].ct))
			goto err_node_put;
		if (of_property_read_u32(cnp, "part", &pdata->dev_id[i].part))
			goto err_node_put;
		snprintf(pdata->dev_id[i].name, FLASHLIGHT_NAME_SIZE,
				BARLEY_NAME);
		pdata->dev_id[i].channel = i;
		pdata->dev_id[i].decouple = decouple;

		pr_info("Parse dt (type,ct,part,name,channel,decouple)=(%d,%d,%d,%s,%d,%d).\n",
				pdata->dev_id[i].type, pdata->dev_id[i].ct,
				pdata->dev_id[i].part, pdata->dev_id[i].name,
				pdata->dev_id[i].channel,
				pdata->dev_id[i].decouple);
		i++;
	}

	return 0;

err_node_put:
	of_node_put(cnp);
	return -EINVAL;
}

static int barley_probe(struct platform_device *pdev)
{
	struct barley_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int err;
	int i;
	pr_info("Probe start.\n");

	/* init pinctrl */
	if (barley_pinctrl_init(pdev)) {
		pr_info("Failed to init pinctrl.\n");
		err = -EFAULT;
		goto err;
	}

	/* init platform data */
	if (!pdata) {
		pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err;
		}
		pdev->dev.platform_data = pdata;
		err = barley_parse_dt(&pdev->dev, pdata);
		if (err)
			goto err;
	}

	/* init work queue */
	INIT_WORK(&barley_work, barley_work_disable);

	/* init timer */
	hrtimer_init(&barley_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	barley_timer.function = barley_timer_func;
	barley_timeout_ms = 100;

	/* init chip hw */
	barley_chip_init();

	/* clear usage count */
	use_count = 0;

	/* register flashlight device */
	if (pdata->channel_num) {
		for (i = 0; i < pdata->channel_num; i++)
			if (flashlight_dev_register_by_device_id(
						&pdata->dev_id[i],
						&barley_ops)) {
				err = -EFAULT;
				goto err;
			}
	} else {
		if (flashlight_dev_register(BARLEY_NAME, &barley_ops)) {
			err = -EFAULT;
			goto err;
		}
	}
	barley_current = sgm3785_current;

 #ifdef CONFIG_PM_WAKELOCKS
 	//wakeup_source_init(&flashlight_wake_lock, "flashlight_lock_wakelock");
    wakeup_source_add(&flashlight_wake_lock);
 #endif

	pr_info("Probe done.\n");

	return 0;
err:
	return err;
}

static int barley_remove(struct platform_device *pdev)
{
	struct barley_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int i;

	pr_info("Remove start.\n");

	pdev->dev.platform_data = NULL;

	/* unregister flashlight device */
	if (pdata && pdata->channel_num)
		for (i = 0; i < pdata->channel_num; i++)
			flashlight_dev_unregister_by_device_id(
					&pdata->dev_id[i]);
	else
		flashlight_dev_unregister(BARLEY_NAME);

	/* flush work queue */
	flush_work(&barley_work);

	pr_info("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id barley_pwm_of_match[] = {
	{.compatible = BARLEY_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, barley_pwm_of_match);
#else
static struct platform_device barley_pwm_platform_device[] = {
	{
		.name = BARLEY_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, barley_pwm_platform_device);
#endif

static struct platform_driver barley_platform_driver = {
	.probe = barley_probe,
	.remove = barley_remove,
	.driver = {
		.name = BARLEY_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = barley_pwm_of_match,
#endif
	},
};

static int __init flashlight_barley_init(void)
{
	int ret;

	pr_info("Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&barley_pwm_platform_device);
	if (ret) {
		pr_info("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&barley_platform_driver);
	if (ret) {
		pr_info("Failed to register platform driver\n");
		return ret;
	}

	pr_info("Init done.\n");

	return 0;
}



static void __exit flashlight_barley_exit(void)
{
	pr_info("Exit start.\n");

	platform_driver_unregister(&barley_platform_driver);

	pr_info("Exit done.\n");
}


module_init(flashlight_barley_init);
module_exit(flashlight_barley_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Wang <Simon-TCH.Wang@mediatek.com>");
MODULE_DESCRIPTION("MTK Flashlight BARLEY PWM Driver");


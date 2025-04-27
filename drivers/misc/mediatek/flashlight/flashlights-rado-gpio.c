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
#ifndef RADO_GPIO_DTNAME
#define RADO_GPIO_DTNAME "mediatek,flashlights_rado_gpio"
#endif

#define RADO_GPIO_NAME "flashlights_rado_gpio"

#ifndef FLASHLIGHT_BRIGHTNESS_ADD
#define FLASHLIGHT_BRIGHTNESS_ADD
#endif

#ifdef CONFIG_PM_WAKELOCKS
#include <linux/pm_wakeup.h>
#endif

/* define registers */

/* define mutex and work queue */
static DEFINE_MUTEX(rado_gpio_mutex);
static struct work_struct rado_gpio_work;
//static struct pwm_spec_config pwm_setting;

#define RADO_GPIO_LEVEL_NUM 26
#define RADO_GPIO_LEVEL_TORCH 9
static int g_duty_array[RADO_GPIO_LEVEL_NUM] = {4, 8, 11, 14, 17, 20, 23,26, 29, 32, 36, 40, 44, 48, 52, 56, 60, 64, 68, 72, 76, 80, 84, 88, 92, 96};

static int g_torch_duty_array[RADO_GPIO_LEVEL_TORCH] = {4, 14, 23, 32, 44, 56, 68, 80, 88};

static const int *rado_gpio_current;
static const int sgm3785_current[RADO_GPIO_LEVEL_NUM] = {40,76,108,140,176,210,248,280,300,320,360,400,440,480,520,560,600,640,680,720,760,800,840,880,920,960};

/* define pinctrl */
#define RADO_GPIO_PINCTRL_PIN_FLASH_EN 0
#define RADO_GPIO_PINCTRL_PIN_PWM_EN 1
#define RADO_GPIO_PINCTRL_PIN_PWM_GPIO 2
#define RADO_GPIO_PINCTRL_PIN_STATE_LOW 0
#define RADO_GPIO_PINCTRL_PIN_STATE_HIGH 1
#define RADO_GPIO_PINCTRL_STATE_FLASH_EN_HIGH "flash_light_en_pin_1"  //ENF 55
#define RADO_GPIO_PINCTRL_STATE_FLASH_EN_LOW  "flash_light_en_pin_0"

#define RADO_GPIO_PINCTRL_STATE_PWM_GPIO_HIGH "flash_light_flash_pin_1"//ENM 147
#define RADO_GPIO_PINCTRL_STATE_PWM_GPIO_LOW  "flash_light_flash_pin_0"
#define RADO_GPIO_PINCTRL_STATE_PWM "flash_light_pwm_pin"	//ENM PWM 147

static struct pinctrl *rado_gpio_pinctrl;
static struct pinctrl_state *rado_gpio_flash_en_high;
static struct pinctrl_state *rado_gpio_flash_en_low;
static struct pinctrl_state *rado_gpio_pwm_gpio_high;
static struct pinctrl_state *rado_gpio_pwm_gpio_low;
static struct pinctrl_state *rado_gpio_flash_pwm;

/* define usage count */
static int use_count;

static int g_flash_duty = -1;

#ifdef CONFIG_PM_WAKELOCKS
struct wakeup_source flashlight_wake_lock;
#endif

/* platform data */
struct rado_gpio_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};


/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
static int rado_gpio_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;
	if(rado_gpio_pinctrl == NULL){
	/* get pinctrl */
	rado_gpio_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(rado_gpio_pinctrl)) {
		pr_info("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(rado_gpio_pinctrl);
		rado_gpio_pinctrl = NULL ;
		return ret;
	}

	/*  Flashlight pin initialization */
	rado_gpio_flash_en_high = pinctrl_lookup_state(
			rado_gpio_pinctrl, RADO_GPIO_PINCTRL_STATE_FLASH_EN_HIGH);
	if (IS_ERR(rado_gpio_flash_en_high)) {
		pr_info("Failed to init (%s)\n", RADO_GPIO_PINCTRL_STATE_FLASH_EN_HIGH);
		ret = PTR_ERR(rado_gpio_flash_en_high);
	}
	rado_gpio_flash_en_low = pinctrl_lookup_state(
			rado_gpio_pinctrl, RADO_GPIO_PINCTRL_STATE_FLASH_EN_LOW);
	if (IS_ERR(rado_gpio_flash_en_low)) {
		pr_info("Failed to init (%s)\n", RADO_GPIO_PINCTRL_STATE_FLASH_EN_LOW);
		ret = PTR_ERR(rado_gpio_flash_en_low);
	}
	rado_gpio_pwm_gpio_high = pinctrl_lookup_state(
			rado_gpio_pinctrl, RADO_GPIO_PINCTRL_STATE_PWM_GPIO_HIGH);
	if (IS_ERR(rado_gpio_pwm_gpio_high)) {
		pr_info("Failed to init (%s)\n", RADO_GPIO_PINCTRL_STATE_PWM_GPIO_HIGH);
		ret = PTR_ERR(rado_gpio_pwm_gpio_high);
	}
	rado_gpio_pwm_gpio_low = pinctrl_lookup_state(
			rado_gpio_pinctrl, RADO_GPIO_PINCTRL_STATE_PWM_GPIO_LOW);
	if (IS_ERR(rado_gpio_pwm_gpio_low)) {
		pr_info("Failed to init (%s)\n", RADO_GPIO_PINCTRL_STATE_PWM_GPIO_LOW);
		ret = PTR_ERR(rado_gpio_pwm_gpio_low);
	}
	rado_gpio_flash_pwm = pinctrl_lookup_state(
			rado_gpio_pinctrl, RADO_GPIO_PINCTRL_STATE_PWM);
	if (IS_ERR(rado_gpio_flash_pwm)) {
		pr_info("Failed to init (%s)\n", RADO_GPIO_PINCTRL_STATE_PWM);
		ret = PTR_ERR(rado_gpio_flash_pwm);
	}
	}
	return ret;
}

static int rado_gpio_pinctrl_set(int pin, int state)
{
	int ret = 0;

	if (IS_ERR(rado_gpio_pinctrl)) {
		pr_info("pinctrl is not available\n");
		return -1;
	}

	switch (pin) {
		case RADO_GPIO_PINCTRL_PIN_FLASH_EN:
			if (state == RADO_GPIO_PINCTRL_PIN_STATE_LOW &&!IS_ERR(rado_gpio_flash_en_low))
				ret = pinctrl_select_state(rado_gpio_pinctrl, rado_gpio_flash_en_low);
			else if (state == RADO_GPIO_PINCTRL_PIN_STATE_HIGH &&!IS_ERR(rado_gpio_flash_en_high))
				ret = pinctrl_select_state(rado_gpio_pinctrl, rado_gpio_flash_en_high);
			else
				pr_info("set err, pin(%d) state(%d)\n", pin, state);
			break;
		case RADO_GPIO_PINCTRL_PIN_PWM_EN:
			ret =pinctrl_select_state(rado_gpio_pinctrl, rado_gpio_flash_pwm);
			break;
		case RADO_GPIO_PINCTRL_PIN_PWM_GPIO:
			if (state == RADO_GPIO_PINCTRL_PIN_STATE_LOW &&!IS_ERR(rado_gpio_pwm_gpio_low))
				ret = pinctrl_select_state(rado_gpio_pinctrl, rado_gpio_pwm_gpio_low);
			else if (state == RADO_GPIO_PINCTRL_PIN_STATE_HIGH &&!IS_ERR(rado_gpio_pwm_gpio_high))
				ret = pinctrl_select_state(rado_gpio_pinctrl, rado_gpio_pwm_gpio_high);
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
 * rado_gpio operations
 *****************************************************************************/
static int rado_gpio_verify_level(int level)
{
	if (level < 0)
		level = 0;
	else if (level >= RADO_GPIO_LEVEL_NUM)
		level = RADO_GPIO_LEVEL_NUM - 1;

	return level;
}

static int rado_gpio_is_torch(int level)
{
	if (level >= RADO_GPIO_LEVEL_TORCH)
		return -1;

	return 0;
}

/* flashlight enable function */
#ifdef FLASHLIGHT_BRIGHTNESS_ADD
bool fl_state=false;
#endif

static int rado_gpio_enable(void)
{
	int tempPWM = 0;

 #ifdef CONFIG_PM_WAKELOCKS
 	__pm_stay_awake(&flashlight_wake_lock);
 #endif

#ifdef FLASHLIGHT_BRIGHTNESS_ADD
	if (fl_state){
		g_flash_duty = RADO_GPIO_LEVEL_NUM - 1;
		pr_info("fl_state=true\n");
	}
#endif
	pr_info("RADO_GPIO Flash %s g_flash_duty = %d\n",__FUNCTION__,g_flash_duty);
	if (!rado_gpio_is_torch(g_flash_duty)) { //troch mode  40ma - 520ma
		if (g_flash_duty <0)
			tempPWM = g_torch_duty_array[0];
		else
			tempPWM = g_torch_duty_array[g_flash_duty];

		rado_gpio_pinctrl_set(RADO_GPIO_PINCTRL_PIN_FLASH_EN,0);	//pull down ENF
		rado_gpio_pinctrl_set(RADO_GPIO_PINCTRL_PIN_PWM_GPIO, 1);	//pull up ENM
		mdelay(5);											//delay more than 5ms
		rado_gpio_pinctrl_set(RADO_GPIO_PINCTRL_PIN_PWM_EN,1);	//set pwm mode
		mt_flashlight_led_set_pwm(0,92);
		pr_info("92 RADO GPIO Flash torch mode %s g_flash_duty = %d,%d\n",__FUNCTION__,g_flash_duty,tempPWM);

    } else {//flash mode
		if (g_flash_duty < RADO_GPIO_LEVEL_NUM)
			tempPWM = g_duty_array[g_flash_duty];
		else
			tempPWM = g_duty_array[RADO_GPIO_LEVEL_NUM - 1];   //flash mode max 936MA

		rado_gpio_pinctrl_set(RADO_GPIO_PINCTRL_PIN_FLASH_EN,0);	//pull down ENF
		rado_gpio_pinctrl_set(RADO_GPIO_PINCTRL_PIN_PWM_EN,1);	//set pwm mode
		mt_flashlight_led_set_pwm(0,tempPWM);               //flash pwm tempPWM%
		udelay(5);
		rado_gpio_pinctrl_set(RADO_GPIO_PINCTRL_PIN_FLASH_EN,1);	//pull up ENF
		pr_info("RADO GPIO Flash %s g_flash_duty = %d,%d\n",__FUNCTION__,g_flash_duty,tempPWM);
	}
	return 0;
}

/* flashlight disable function */
static int rado_gpio_disable(void)
{
	int state = RADO_GPIO_PINCTRL_PIN_STATE_LOW;

	rado_gpio_pinctrl_set(RADO_GPIO_PINCTRL_PIN_FLASH_EN, state);
	rado_gpio_pinctrl_set(RADO_GPIO_PINCTRL_PIN_PWM_GPIO, state);
	mt_pwm_disable(0,0);
 #ifdef CONFIG_PM_WAKELOCKS
 	__pm_relax(&flashlight_wake_lock);
 #endif

	return 0;
}

#ifdef FLASHLIGHT_BRIGHTNESS_ADD
int rado_gpio_enable_flash(int level)
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
EXPORT_SYMBOL(rado_gpio_enable_flash);
#endif

/* set flashlight level */
static int rado_gpio_set_level(int level)
{
	g_flash_duty = level;
	return 0;
}

/* flashlight init */
static int rado_gpio_init(void)
{
	int state = RADO_GPIO_PINCTRL_PIN_STATE_LOW;

	rado_gpio_pinctrl_set(RADO_GPIO_PINCTRL_PIN_FLASH_EN, state);
	rado_gpio_pinctrl_set(RADO_GPIO_PINCTRL_PIN_PWM_GPIO, state);
	return 0;
}

/* flashlight uninit */
static int rado_gpio_uninit(void)
{
	int state = RADO_GPIO_PINCTRL_PIN_STATE_LOW;

	rado_gpio_pinctrl_set(RADO_GPIO_PINCTRL_PIN_FLASH_EN, state);
	rado_gpio_pinctrl_set(RADO_GPIO_PINCTRL_PIN_PWM_GPIO, state);
	return 0;
}

/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer rado_gpio_timer;
static unsigned int rado_gpio_timeout_ms;

static void rado_gpio_work_disable(struct work_struct *data)
{
	pr_info("work queue callback\n");
	rado_gpio_disable();
}

static enum hrtimer_restart rado_gpio_timer_func(struct hrtimer *timer)
{
	schedule_work(&rado_gpio_work);
	return HRTIMER_NORESTART;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int rado_gpio_ioctl(unsigned int cmd, unsigned long arg)
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
		rado_gpio_timeout_ms = 0;//fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		pr_info("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		rado_gpio_set_level(fl_arg->arg);
		break;
	case FLASH_IOC_SET_ONOFF:
		pr_info("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (rado_gpio_timeout_ms) {
				s = rado_gpio_timeout_ms / 1000;
				ns = rado_gpio_timeout_ms % 1000 * 1000000;
				ktime = ktime_set(s, ns);
				hrtimer_start(&rado_gpio_timer, ktime,
						HRTIMER_MODE_REL);
			}
			rado_gpio_enable();
		} else {
			rado_gpio_disable();
			hrtimer_cancel(&rado_gpio_timer);
		}
		break;
	case FLASH_IOC_GET_DUTY_NUMBER:
		pr_info("FLASH_IOC_GET_DUTY_NUMBER(%d)\n", channel);
		fl_arg->arg = RADO_GPIO_LEVEL_NUM;
		break;

	case FLASH_IOC_GET_MAX_TORCH_DUTY:
		pr_info("FLASH_IOC_GET_MAX_TORCH_DUTY(%d)\n", channel);
		fl_arg->arg = RADO_GPIO_LEVEL_TORCH - 1;
		break;

	case FLASH_IOC_GET_DUTY_CURRENT:
		fl_arg->arg = rado_gpio_verify_level(fl_arg->arg);
		pr_info("FLASH_IOC_GET_DUTY_CURRENT(%d): %d\n",
				channel, (int)fl_arg->arg);
		fl_arg->arg = rado_gpio_current[fl_arg->arg];
		break;

	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int rado_gpio_open(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int rado_gpio_release(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int rado_gpio_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&rado_gpio_mutex);
	if (set) {
		if (!use_count)
			ret = rado_gpio_init();
		use_count++;
		pr_info("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = rado_gpio_uninit();
		if (use_count < 0)
			use_count = 0;
		pr_info("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&rado_gpio_mutex);

	return ret;
}

static ssize_t rado_gpio_strobe_store(struct flashlight_arg arg)
{
	rado_gpio_set_driver(1);
	rado_gpio_set_level(arg.level);
	rado_gpio_timeout_ms = 0;
	rado_gpio_enable();
	msleep(arg.dur);
	rado_gpio_disable();
	rado_gpio_set_driver(0);

	return 0;
}

static struct flashlight_operations rado_gpio_ops = {
	rado_gpio_open,
	rado_gpio_release,
	rado_gpio_ioctl,
	rado_gpio_strobe_store,
	rado_gpio_set_driver
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int rado_gpio_chip_init(void)
{
	/* NOTE: Chip initialication move to "set driver" for power saving.
	 * rado_gpio_init();
	 */

	return 0;
}

static int rado_gpio_parse_dt(struct device *dev,
		struct rado_gpio_platform_data *pdata)
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
				RADO_GPIO_NAME);
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

static int rado_gpio_probe(struct platform_device *pdev)
{
	struct rado_gpio_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int err;
	int i;
	pr_info("Probe start.\n");

	/* init pinctrl */
	if (rado_gpio_pinctrl_init(pdev)) {
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
		err = rado_gpio_parse_dt(&pdev->dev, pdata);
		if (err)
			goto err;
	}

	/* init work queue */
	INIT_WORK(&rado_gpio_work, rado_gpio_work_disable);

	/* init timer */
	hrtimer_init(&rado_gpio_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	rado_gpio_timer.function = rado_gpio_timer_func;
	rado_gpio_timeout_ms = 100;

	/* init chip hw */
	rado_gpio_chip_init();

	/* clear usage count */
	use_count = 0;

	/* register flashlight device */
	if (pdata->channel_num) {
		for (i = 0; i < pdata->channel_num; i++)
			if (flashlight_dev_register_by_device_id(
						&pdata->dev_id[i],
						&rado_gpio_ops)) {
				err = -EFAULT;
				goto err;
			}
	} else {
		if (flashlight_dev_register(RADO_GPIO_NAME, &rado_gpio_ops)) {
			err = -EFAULT;
			goto err;
		}
	}
	rado_gpio_current = sgm3785_current;

 #ifdef CONFIG_PM_WAKELOCKS
 	//wakeup_source_init(&flashlight_wake_lock, "flashlight_lock_wakelock");
    wakeup_source_add(&flashlight_wake_lock);
 #endif

	pr_info("Probe done.\n");

	return 0;
err:
	return err;
}

static int rado_gpio_remove(struct platform_device *pdev)
{
	struct rado_gpio_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int i;

	pr_info("Remove start.\n");

	pdev->dev.platform_data = NULL;

	/* unregister flashlight device */
	if (pdata && pdata->channel_num)
		for (i = 0; i < pdata->channel_num; i++)
			flashlight_dev_unregister_by_device_id(
					&pdata->dev_id[i]);
	else
		flashlight_dev_unregister(RADO_GPIO_NAME);

	/* flush work queue */
	flush_work(&rado_gpio_work);

	pr_info("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id rado_gpio_pwm_of_match[] = {
	{.compatible = RADO_GPIO_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, rado_gpio_pwm_of_match);
#else
static struct platform_device rado_gpio_pwm_platform_device[] = {
	{
		.name = RADO_GPIO_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, rado_gpio_pwm_platform_device);
#endif

static struct platform_driver rado_gpio_platform_driver = {
	.probe = rado_gpio_probe,
	.remove = rado_gpio_remove,
	.driver = {
		.name = RADO_GPIO_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = rado_gpio_pwm_of_match,
#endif
	},
};

static int __init flashlight_rado_gpio_init(void)
{
	int ret;

	pr_info("Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&rado_gpio_pwm_platform_device);
	if (ret) {
		pr_info("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&rado_gpio_platform_driver);
	if (ret) {
		pr_info("Failed to register platform driver\n");
		return ret;
	}

	pr_info("Init done.\n");

	return 0;
}



static void __exit flashlight_rado_gpio_exit(void)
{
	pr_info("Exit start.\n");

	platform_driver_unregister(&rado_gpio_platform_driver);

	pr_info("Exit done.\n");
}

// int high_cct_led_strobe_enable_part1(void)
// {
//     pr_info("!!!\n");
//     return 0;
// }

// int high_cct_led_strobe_setduty_part1(int duty)
// {

//     rado_gpio_set_level(RADO_GPIO_TORCH_MODE_DUTY);
//     return 0;

// }

// int high_cct_led_strobe_on_part1(int onoff)
// {
//     if(onoff)
//         rado_gpio_enable();
//     else
//         rado_gpio_disable();
//     return 0;
// }

// int low_cct_led_strobe_enable_part1(void)
// {
//     pr_info("!!!\n");
//     return 0;
// }

// int low_cct_led_strobe_setduty_part1(int duty)
// {

//   rado_gpio_set_level(RADO_GPIO_TORCH_MODE_DUTY);
//   return 0;

// }

// int low_cct_led_strobe_on_part1(int onoff)
// {
//     if(onoff)
//         rado_gpio_enable();
//     else
//         rado_gpio_disable();
//     return 0;
// }



module_init(flashlight_rado_gpio_init);
module_exit(flashlight_rado_gpio_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Wang <Simon-TCH.Wang@mediatek.com>");
MODULE_DESCRIPTION("MTK Flashlight RADO GPIO PWM Driver");

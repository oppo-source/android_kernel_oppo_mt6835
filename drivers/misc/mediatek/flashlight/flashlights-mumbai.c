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

#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

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
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/pinctrl/consumer.h>
#ifndef OPLUS_FEATURE_CAMERA_COMMON
#define OPLUS_FEATURE_CAMERA_COMMON
#endif
#ifdef OPLUS_FEATURE_CAMERA_COMMON
#include <soc/oplus/system/oplus_project.h>
#endif

#include "flashlight-core.h"
#include "flashlight-dt.h"
#include "flashlight.h"
/* device tree should be defined in flashlight-dt.h */
#ifndef MUMBAI_DTNAME
#define MUMBAI_DTNAME "mediatek,flashlights_mumbai"
#endif
#ifndef MUMBAI_DTNAME_I2C
#define MUMBAI_DTNAME_I2C   "mediatek,strobe_main"
#endif

#define MUMBAI_NAME "flashlights_mumbai"

/* define registers */
#define MUMBAI_REG_ENABLE           (0x01)
#define MUMBAI_MASK_ENABLE_LED1     (0x01)
#define MUMBAI_MASK_ENABLE_LED2     (0x02)
#define MUMBAI_DISABLE              (0x00)
#define MUMBAI_TORCH_MODE           (0x08)
#define MUMBAI_FLASH_MODE           (0x0C)
#define MUMBAI_ENABLE_LED1          (0x01)
#define MUMBAI_ENABLE_LED1_TORCH    (0x09)
#define MUMBAI_ENABLE_LED1_FLASH    (0x0D)
#define MUMBAI_ENABLE_LED2          (0x02)
#define MUMBAI_ENABLE_LED2_TORCH    (0x0A)
#define MUMBAI_ENABLE_LED2_FLASH    (0x0E)

#define MUMBAI_REG_TORCH_LEVEL_LED1 (0x05)
#define MUMBAI_REG_FLASH_LEVEL_LED1 (0x03)
#define MUMBAI_REG_TORCH_LEVEL_LED2 (0x06)
#define MUMBAI_REG_FLASH_LEVEL_LED2 (0x04)

#define MUMBAI_REG_TIMING_CONF      (0x08)
#define MUMBAI_TORCH_RAMP_TIME      (0x10)
#define MUMBAI_FLASH_TIMEOUT        (0x0F)

#define MUMBAI_AW36515_SOFT_RESET_ENABLE (0x80)
#define MUMBAI_AW36515_REG_BOOST_CONFIG (0x07)


/* define channel, level */
#define MUMBAI_CHANNEL_NUM          2
#define MUMBAI_CHANNEL_CH1          0
#define MUMBAI_CHANNEL_CH2          1
/* define level */
#define MUMBAI_LEVEL_NUM 28
#define MUMBAI_LEVEL_TORCH 7

#define MUMBAI_HW_TIMEOUT 400 /* ms */

/* define mutex and work queue */
static DEFINE_MUTEX(mumbai_mutex);
static struct work_struct mumbai_work_ch1;
static struct work_struct mumbai_work_ch2;

/* define pinctrl */
#define MUMBAI_PINCTRL_PIN_HWEN 0
#define MUMBAI_PINCTRL_PINSTATE_LOW 0
#define MUMBAI_PINCTRL_PINSTATE_HIGH 1
#define MUMBAI_PINCTRL_STATE_HWEN_HIGH "mumbai_hwen_high"
#define MUMBAI_PINCTRL_STATE_HWEN_LOW  "mumbai_hwen_low"

/* define device id */
#define USE_AW36515_IC  0x1111

struct i2c_client *mumbai_flashlight_client;

/* define usage count */
static int use_count;

/* define i2c */
static struct i2c_client *mumbai_i2c_client;

/* platform data */
struct mumbai_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};

/* mumbai chip data */
struct mumbai_chip_data {
	struct i2c_client *client;
	struct mumbai_platform_data *pdata;
	struct mutex lock;
	u8 last_flag;
	u8 no_pdata;
};

enum FLASHLIGHT_DEVICE {
	AW36515_SM = 0x02,
};


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// i2c write and read
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
static int mumbai_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	struct mumbai_chip_data *chip = i2c_get_clientdata(client);
	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		pr_err("failed writing at 0x%02x\n", reg);

	return ret;
}

/* i2c wrapper function */
static int mumbai_read_reg(struct i2c_client *client, u8 reg)
{
	int val;
	struct mumbai_chip_data *chip = i2c_get_clientdata(client);
	mutex_lock(&chip->lock);
	val = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);

	if (val < 0)
		pr_err("failed read at 0x%02x\n", reg);

	return val;
}


/******************************************************************************
 * mumbai operations
 *****************************************************************************/

static const int *mumbai_current;
static const unsigned char *mumbai_torch_level;
static const unsigned char *mumbai_flash_level;

static const int AW36515_current[MUMBAI_LEVEL_NUM] = {
	24,   46,   70,   93,   116,  141,  164,  199,  246,  304,
	351,  398,  445,  504,  551,  598,  656,  703,  750,  797,
	856,  903,  949,  996,  1055, 1100, 1147, 1202
};

/*Offset: 0.98mA(00000000)
Step:1.96mA
Range: 0.98mA(00000000)~500mA(11111111)*/
static const unsigned char AW36515_torch_level[MUMBAI_LEVEL_NUM] = {
	0x0C, 0x17, 0x23, 0x31, 0x3B, 0x47, 0x53, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/*Offset: 3.91mA(00000000)
Step:7.83mA
Range: 3.91 mA(00000000)~2.0 A(11111111)*/
static const unsigned char AW36515_flash_level[MUMBAI_LEVEL_NUM] = {
	0x03, 0x05, 0x08, 0x0C, 0x0E, 0x12, 0x14, 0x19, 0x1F, 0x26,
	0x2C, 0x32, 0x38, 0x40, 0x46, 0x4C, 0x53, 0x59, 0x5F, 0x65,
	0x6D, 0x73, 0x79, 0x7F, 0x86, 0x8C, 0x92, 0x99
};


static volatile unsigned char mumbai_reg_enable;
static volatile int mumbai_level_ch1 = -1;
static volatile int mumbai_level_ch2 = -1;

static int mumbai_is_torch(int level)
{
	if (level >= MUMBAI_LEVEL_TORCH)
		return -1;

	return 0;
}

static int mumbai_verify_level(int level)
{
	if (level < 0)
		level = 0;
	else if (level >= MUMBAI_LEVEL_NUM)
		level = MUMBAI_LEVEL_NUM - 1;

	return level;
}

/* flashlight enable function */
static int mumbai_enable_ch1(void)
{
	unsigned char reg, val;

	reg = MUMBAI_REG_ENABLE;
	if (!mumbai_is_torch(mumbai_level_ch1)) {
		/* torch mode */
		mumbai_reg_enable |= MUMBAI_ENABLE_LED1_TORCH;
	} else {
		/* flash mode */
		mumbai_reg_enable |= MUMBAI_ENABLE_LED1_FLASH;
	}
	val = mumbai_reg_enable;

	return mumbai_write_reg(mumbai_i2c_client, reg, val);
}

static int mumbai_enable_ch2(void)
{
	unsigned char reg, val;

	reg = MUMBAI_REG_ENABLE;
	if (!mumbai_is_torch(mumbai_level_ch2)) {
		/* torch mode */
		mumbai_reg_enable |= MUMBAI_ENABLE_LED2_TORCH;
	} else {
		/* flash mode */
		mumbai_reg_enable |= MUMBAI_ENABLE_LED2_FLASH;
	}
	val = mumbai_reg_enable;

	return mumbai_write_reg(mumbai_i2c_client, reg, val);
}

static int mumbai_enable(int channel)
{
	if (channel == MUMBAI_CHANNEL_CH1)
		mumbai_enable_ch1();
	else if (channel == MUMBAI_CHANNEL_CH2)
		mumbai_enable_ch2();
	else {
		pr_err("Error channel\n");
		return -1;
	}
	return 0;
}

/* flashlight disable function */
static int mumbai_disable_ch1(void)
{
	unsigned char reg, val;

	reg = MUMBAI_REG_ENABLE;
	if (mumbai_reg_enable & MUMBAI_MASK_ENABLE_LED2) {
		/* if LED 2 is enable, disable LED 1 */
		mumbai_reg_enable &= (~MUMBAI_ENABLE_LED1);
	} else {
		/* if LED 2 is enable, disable LED 1 and clear mode */
		mumbai_reg_enable &= (~MUMBAI_ENABLE_LED1_FLASH);
	}
	val = mumbai_reg_enable;
	return mumbai_write_reg(mumbai_i2c_client, reg, val);
}

static int mumbai_disable_ch2(void)
{
	unsigned char reg, val;

	reg = MUMBAI_REG_ENABLE;
	if (mumbai_reg_enable & MUMBAI_MASK_ENABLE_LED1) {
		/* if LED 1 is enable, disable LED 2 */
		mumbai_reg_enable &= (~MUMBAI_ENABLE_LED2);
	} else {
		/* if LED 1 is enable, disable LED 2 and clear mode */
		mumbai_reg_enable &= (~MUMBAI_ENABLE_LED2_FLASH);
	}
	val = mumbai_reg_enable;

	return mumbai_write_reg(mumbai_i2c_client, reg, val);
}

static int mumbai_disable(int channel)
{
	if (channel == MUMBAI_CHANNEL_CH1) {
		mumbai_disable_ch1();
		pr_info("MUMBAI_CHANNEL_CH1\n");
	} else if (channel == MUMBAI_CHANNEL_CH2) {
		mumbai_disable_ch2();
		pr_info("MUMBAI_CHANNEL_CH2\n");
	} else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

/* set flashlight level */
static int mumbai_set_level_ch1(int level)
{
	int ret;
	unsigned char reg, val;

	level = mumbai_verify_level(level);

	/* set torch brightness level */
	reg = MUMBAI_REG_TORCH_LEVEL_LED1;
	val = mumbai_torch_level[level];
	ret = mumbai_write_reg(mumbai_i2c_client, reg, val);

	mumbai_level_ch1 = level;

	/* set flash brightness level */
	reg = MUMBAI_REG_FLASH_LEVEL_LED1;
	val = mumbai_flash_level[level];
	ret = mumbai_write_reg(mumbai_i2c_client, reg, val);

	return ret;
}

int mumbai_set_level_ch2(int level)
{
	int ret;
	unsigned char reg, val;

	level = mumbai_verify_level(level);

	/* set torch brightness level */
	reg = MUMBAI_REG_TORCH_LEVEL_LED2;
	val = mumbai_torch_level[level];
	ret = mumbai_write_reg(mumbai_i2c_client, reg, val);

	mumbai_level_ch2 = level;

	/* set flash brightness level */
	reg = MUMBAI_REG_FLASH_LEVEL_LED2;
	val = mumbai_flash_level[level];
	ret = mumbai_write_reg(mumbai_i2c_client, reg, val);

	return ret;
}

static int mumbai_set_level(int channel, int level)
{
	if (channel == MUMBAI_CHANNEL_CH1)
		mumbai_set_level_ch1(level);
	else if (channel == MUMBAI_CHANNEL_CH2)
		mumbai_set_level_ch2(level);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}
/* flashlight init */
int mumbai_init(void)
{
	int ret;
	unsigned char reg, val, reg_val;
	int chip_id;


	chip_id = mumbai_read_reg(mumbai_i2c_client, 0x0c);
	msleep(2);
	pr_info("flashlight chip id: reg:0x0c, chip_id 0x%x",chip_id);
	if ( chip_id == AW36515_SM ) {
		reg_val = mumbai_read_reg(mumbai_i2c_client, MUMBAI_AW36515_REG_BOOST_CONFIG);
		reg_val |= MUMBAI_AW36515_SOFT_RESET_ENABLE;
		pr_info("flashlight chip id: reg:0x0c, data:0x%x;boost confgiuration: reg:0x07, reg_val: 0x%x", chip_id, reg_val);
		ret = mumbai_write_reg(mumbai_i2c_client, MUMBAI_AW36515_REG_BOOST_CONFIG, reg_val);
		msleep(2);
	}
	/* clear enable register */
	reg = MUMBAI_REG_ENABLE;
	val = MUMBAI_DISABLE;
	ret = mumbai_write_reg(mumbai_i2c_client, reg, val);

	mumbai_reg_enable = val;

	/* set torch current ramp time and flash timeout */
	reg = MUMBAI_REG_TIMING_CONF;
	val = MUMBAI_TORCH_RAMP_TIME | MUMBAI_FLASH_TIMEOUT;
	ret = mumbai_write_reg(mumbai_i2c_client, reg, val);

	return ret;
}

/* flashlight uninit */
int mumbai_uninit(void)
{
	mumbai_disable(MUMBAI_CHANNEL_CH1);
	mumbai_disable(MUMBAI_CHANNEL_CH2);

	return 0;
}


/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer mumbai_timer_ch1;
static struct hrtimer mumbai_timer_ch2;
static unsigned int mumbai_timeout_ms[MUMBAI_CHANNEL_NUM];

static void mumbai_work_disable_ch1(struct work_struct *data)
{
	pr_info("ht work queue callback\n");
	mumbai_disable_ch1();
}

static void mumbai_work_disable_ch2(struct work_struct *data)
{
	pr_info("lt work queue callback\n");
	mumbai_disable_ch2();
}

static enum hrtimer_restart mumbai_timer_func_ch1(struct hrtimer *timer)
{
	schedule_work(&mumbai_work_ch1);
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart mumbai_timer_func_ch2(struct hrtimer *timer)
{
	schedule_work(&mumbai_work_ch2);
	return HRTIMER_NORESTART;
}

int mumbai_timer_start(int channel, ktime_t ktime)
{
	if (channel == MUMBAI_CHANNEL_CH1)
		hrtimer_start(&mumbai_timer_ch1, ktime, HRTIMER_MODE_REL);
	else if (channel == MUMBAI_CHANNEL_CH2)
		hrtimer_start(&mumbai_timer_ch2, ktime, HRTIMER_MODE_REL);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

int mumbai_timer_cancel(int channel)
{
	if (channel == MUMBAI_CHANNEL_CH1)
		hrtimer_cancel(&mumbai_timer_ch1);
	else if (channel == MUMBAI_CHANNEL_CH2)
		hrtimer_cancel(&mumbai_timer_ch2);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int mumbai_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	ktime_t ktime;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	/* verify channel */
	if (channel < 0 || channel >= MUMBAI_CHANNEL_NUM) {
		pr_err("Failed with error channel\n");
		return -EINVAL;
	}
	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		pr_info("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		mumbai_timeout_ms[channel] = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		pr_info("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		mumbai_set_level(channel, fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		pr_info("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (mumbai_timeout_ms[channel]) {
				ktime = ktime_set(mumbai_timeout_ms[channel] / 1000,
						(mumbai_timeout_ms[channel] % 1000) * 1000000);
				mumbai_timer_start(channel, ktime);
			}
			mumbai_enable(channel);
		} else {
			mumbai_disable(channel);
			mumbai_timer_cancel(channel);
		}
		break;

	case FLASH_IOC_GET_DUTY_NUMBER:
		pr_info("FLASH_IOC_GET_DUTY_NUMBER(%d)\n", channel);
		fl_arg->arg = MUMBAI_LEVEL_NUM;
		break;

	case FLASH_IOC_GET_MAX_TORCH_DUTY:
		pr_info("FLASH_IOC_GET_MAX_TORCH_DUTY(%d)\n", channel);
		fl_arg->arg = MUMBAI_LEVEL_TORCH - 1;
		break;

	case FLASH_IOC_GET_DUTY_CURRENT:
		fl_arg->arg = mumbai_verify_level(fl_arg->arg);
		pr_info("FLASH_IOC_GET_DUTY_CURRENT(%d): %d\n",
				channel, (int)fl_arg->arg);
		fl_arg->arg = mumbai_current[fl_arg->arg];
		break;

	case FLASH_IOC_GET_HW_TIMEOUT:
		pr_info("FLASH_IOC_GET_HW_TIMEOUT(%d)\n", channel);
		fl_arg->arg = MUMBAI_HW_TIMEOUT;
		break;

	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int mumbai_open(void)
{
	/* Actual behavior move to set driver function since power saving issue */
	return 0;
}

static int mumbai_release(void)
{
	/* uninit chip and clear usage count */
/*
	mutex_lock(&mumbai_mutex);
	use_count--;
	if (!use_count)
		mumbai_uninit();
	if (use_count < 0)
		use_count = 0;
	mutex_unlock(&mumbai_mutex);

	pr_info("Release: %d\n", use_count);
*/
	return 0;
}

static int mumbai_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&mumbai_mutex);
	if (set) {
		if (!use_count)
			ret = mumbai_init();
		use_count++;
		pr_info("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = mumbai_uninit();
		if (use_count < 0)
			use_count = 0;
		pr_info("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&mumbai_mutex);

	return ret;
}

static ssize_t mumbai_strobe_store(struct flashlight_arg arg)
{
	mumbai_set_driver(1);
	mumbai_set_level(arg.channel, arg.level);
	mumbai_timeout_ms[arg.channel] = 0;
	mumbai_enable(arg.channel);
	msleep(arg.dur);
	mumbai_disable(arg.channel);
	//mumbai_release(NULL);
	mumbai_set_driver(0);
	return 0;
}

static struct flashlight_operations mumbai_ops = {
	mumbai_open,
	mumbai_release,
	mumbai_ioctl,
	mumbai_strobe_store,
	mumbai_set_driver
};


/******************************************************************************
 * I2C device and driver
 *****************************************************************************/
static int mumbai_chip_init(struct mumbai_chip_data *chip)
{
	/* NOTE: Chip initialication move to "set driver" operation for power saving issue.
	 * mumbai_init();
	 */

	return 0;
}

static int mumbai_parse_dt(struct device *dev,
		struct mumbai_platform_data *pdata)
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
				MUMBAI_NAME);
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

static int mumbai_chip_id(void)
{
	int chip_id;
	int reg00_id = -1;
	msleep(1);
	chip_id = mumbai_read_reg(mumbai_i2c_client, 0x0c);
	pr_info("flashlight chip id: reg:0x0c, data:0x%x", chip_id);
	if (chip_id == AW36515_SM) {
		reg00_id = mumbai_read_reg(mumbai_i2c_client, 0x00);
		pr_info("flashlight chip id: reg:0x00, data:0x%x", reg00_id);
		if (reg00_id == 0x30) {
			chip_id = AW36515_SM;
			pr_info("flashlight reg00_id = 0x%x, set chip_id to AW36515_SM", reg00_id);
		}
	}
    if (chip_id == AW36515_SM){
		pr_info(" the device's flashlight driver IC is AW36515\n");
		return USE_AW36515_IC;
	} else {
		pr_err(" the device's flashlight driver IC is not used in our project!\n");
		return USE_AW36515_IC;
	}
}

static int mumbai_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct mumbai_chip_data *chip;
    struct mumbai_platform_data *pdata = client->dev.platform_data;
    int err;
    int i;
    int chip_id;
    bool curProject = false;
    pr_info("mumbai_i2c_probe Probe start.\n");
    curProject = is_project(25610) || is_project(25676) || is_project(25686) || is_project(25685);

    if (!curProject) {
        err = -ENODEV;
        goto err_out;
    }

    /* check i2c */
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        pr_err("Failed to check i2c functionality.\n");
        err = -ENODEV;
        goto err_out;
    }

    /* init chip private data */
    chip = kzalloc(sizeof(struct mumbai_chip_data), GFP_KERNEL);
    if (!chip) {
        err = -ENOMEM;
        goto err_out;
    }
    chip->client = client;

    /* init platform data */
    if (!pdata) {
        pr_err("Platform data does not exist\n");
        pdata = kzalloc(sizeof(struct mumbai_platform_data), GFP_KERNEL);
        if (!pdata) {
            err = -ENOMEM;
            goto err_free_chip;
        }
        client->dev.platform_data = pdata;
        err = mumbai_parse_dt(&client->dev, pdata);
        if (err) {
            goto err_free_pdata;
        }
        chip->no_pdata = 1; // Mark that pdata is dynamically allocated
    }
    chip->pdata = pdata;
    i2c_set_clientdata(client, chip);
    mumbai_i2c_client = client;

    /* init mutex and spinlock */
    mutex_init(&chip->lock);

    /* init work queue */
    INIT_WORK(&mumbai_work_ch1, mumbai_work_disable_ch1);
    INIT_WORK(&mumbai_work_ch2, mumbai_work_disable_ch2);

    /* init timer */
    hrtimer_init(&mumbai_timer_ch1, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    mumbai_timer_ch1.function = mumbai_timer_func_ch1;
    hrtimer_init(&mumbai_timer_ch2, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    mumbai_timer_ch2.function = mumbai_timer_func_ch2;
    mumbai_timeout_ms[MUMBAI_CHANNEL_CH1] = 100;
    mumbai_timeout_ms[MUMBAI_CHANNEL_CH2] = 100;

    /* init chip hw */
    mumbai_chip_init(chip);
    chip_id = mumbai_chip_id();
    if (chip_id == USE_AW36515_IC){
        mumbai_current = AW36515_current;
        mumbai_torch_level = AW36515_torch_level;
        mumbai_flash_level = AW36515_flash_level;
    }

    /* register flashlight operations */
    if (pdata->channel_num) {
        for (i = 0; i < pdata->channel_num; i++)
            if (flashlight_dev_register_by_device_id(
                        &pdata->dev_id[i],
                        &mumbai_ops)) {
                pr_err("Failed to register flashlight device.\n");
                err = -EFAULT;
                goto err_free_pdata;
            }
    } else {
        if (flashlight_dev_register(MUMBAI_NAME, &mumbai_ops)) {
            pr_err("Failed to register flashlight device.\n");
            err = -EFAULT;
            goto err_free_pdata;
        }
    }

    //mumbai_create_sysfs(client);

    pr_info("Probe done.\n");

    return 0;

err_free_pdata:
    if (chip->no_pdata)
        kfree(chip->pdata);
err_free_chip:
    kfree(chip);
err_out:
    return err;
}

static int mumbai_i2c_remove(struct i2c_client *client)
{
    struct mumbai_platform_data *pdata = dev_get_platdata(&client->dev);
    struct mumbai_chip_data *chip = i2c_get_clientdata(client);
    int i;

    pr_info("Remove start.\n");

    client->dev.platform_data = NULL;

    /* unregister flashlight device */
    if (pdata && pdata->channel_num)
        for (i = 0; i < pdata->channel_num; i++)
            flashlight_dev_unregister_by_device_id(
                    &pdata->dev_id[i]);
    else
        flashlight_dev_unregister(MUMBAI_NAME);

    /* flush work queue */
    flush_work(&mumbai_work_ch1);
    flush_work(&mumbai_work_ch2);

    /* unregister flashlight operations */
    flashlight_dev_unregister(MUMBAI_NAME);

    /* free resource */
    if (chip && chip->no_pdata)
        kfree(chip->pdata);
    kfree(chip);

    pr_info("Remove done.\n");

    return 0;
}

static const struct i2c_device_id mumbai_i2c_id[] = {
	{MUMBAI_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id mumbai_i2c_of_match[] = {
	{.compatible = MUMBAI_DTNAME_I2C},
	{},
};
MODULE_DEVICE_TABLE(of, mumbai_i2c_of_match);
#endif

static struct i2c_driver mumbai_i2c_driver = {
	.driver = {
		   .name = MUMBAI_NAME,
#ifdef CONFIG_OF
		   .of_match_table = mumbai_i2c_of_match,
#endif
		   },
	.probe = mumbai_i2c_probe,
	.remove = mumbai_i2c_remove,
	.id_table = mumbai_i2c_id,
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int mumbai_probe(struct platform_device *dev)
{
	pr_info("Probe start %s.\n", MUMBAI_DTNAME_I2C);

	if (i2c_add_driver(&mumbai_i2c_driver)) {
		pr_err("Failed to add i2c driver.\n");
		return -1;
	}

	pr_info("Probe done.\n");

	return 0;
}

static int mumbai_remove(struct platform_device *dev)
{
	pr_info("Remove start.\n");

	i2c_del_driver(&mumbai_i2c_driver);

	pr_info("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id mumbai_of_match[] = {
	{.compatible = MUMBAI_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, mumbai_of_match);
#else
static struct platform_device mumbai_platform_device[] = {
	{
		.name = MUMBAI_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, mumbai_platform_device);
#endif

static struct platform_driver mumbai_platform_driver = {
	.probe = mumbai_probe,
	.remove = mumbai_remove,
	.driver = {
		.name = MUMBAI_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = mumbai_of_match,
#endif
	},
};

static int __init flashlight_mumbai_init(void)
{
	int ret;

	pr_info("flashlight_mumbai-Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&mumbai_platform_device);
	if (ret) {
		pr_err("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&mumbai_platform_driver);
	if (ret) {
		pr_err("Failed to register platform driver\n");
		return ret;
	}

	pr_info("flashlight_mumbai Init done.\n");

	return 0;
}

static void __exit flashlight_mumbai_exit(void)
{
	pr_info("flashlight_mumbai-Exit start.\n");

	platform_driver_unregister(&mumbai_platform_driver);

	pr_info("flashlight_mumbai Exit done.\n");
}


module_init(flashlight_mumbai_init);
module_exit(flashlight_mumbai_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Joseph <zhangzetao@awinic.com.cn>");
MODULE_DESCRIPTION("AW Flashlight MUMBAI Driver");


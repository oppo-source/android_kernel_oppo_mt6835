/*
 * sc080cs.c  sc080cs yuv module
 *
 * Author: Bruce <sunchengwei@longcheer.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/irq.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/gameport.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/kthread.h>
#include <linux/regulator/consumer.h>
#include "sc_sc080cs_iii.h"
#include <linux/videodev2.h>
#include <linux/cdev.h>
#include <linux/atomic.h>
#include <linux/types.h>
#include <linux/sysfs.h>

extern int seninf_clk_set_open(unsigned int on, unsigned int TG);
extern int qvga_sensor_open(char *qvga_sensor_name, unsigned int qvga_idx, bool power_on);

#define kal_uint16 unsigned short
#define kal_uint32 unsigned int
/*****************************************************************
* qvga marco
******************************************************************/
#define QVGA_DRIVER_VERSION	"V2.0"
#define SC080CS_SENSOR_ID   0x3a6c //一供SC080CS
#define SP0821_SENSOR_ID   0x9c //二供SP0821
#define SENSOR_IDX 2
/*****************************************************************
* qvga global global variable
******************************************************************/
static unsigned char read_reg_id = 0;
static unsigned char read_reg_value = 0;
static int read_reg_flag = 0;
static int driver_flag = 0;
struct qvga *qvga = NULL;
struct device *dev;
struct class *qvga_class;
enum qvga_sensor_type {
	QVGA_SENSOR_SC080CS = 1,
	QVGA_SENSOR_SP0821,
};
enum qvga_sensor_type qvga_sensor_info;
/**********************************************************
* i2c write and read
**********************************************************/
static void sc080cs_i2c_write(struct qvga *sc080cs, int address, int data)
{
	u8 i2c_buf[8];
	struct i2c_client *client = sc080cs->i2c_client;
	struct i2c_msg msg[1];
	msg[0].flags = !I2C_M_RD;
	msg[0].addr = client->addr;
	msg[0].len = 3;
	msg[0].buf = i2c_buf;

	i2c_buf[0] = (address & 0xff00)>>8;
	i2c_buf[1] = (address & 0xff);
	i2c_buf[2] = data;

	i2c_transfer(client->adapter, msg, 1);
}
static int sp0821_i2c_write(struct qvga *sp0821,
			     unsigned char reg_addr, unsigned char reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < 1) {
		ret = i2c_smbus_write_byte_data(sp0821->i2c_client,
						reg_addr, reg_data);
		if (ret < 0) {
			qvga_dev_err(sp0821->dev,
				   "%s: i2c_write cnt=%d error=%d\n", __func__,
				   cnt, ret);
		} else {
			break;
		}
		cnt++;
		mdelay(2);
	}

	return ret;
}

static int sc080cs_i2c_read(struct qvga *sc080cs, int address, unsigned char *rxdata)
{
	unsigned char i2c_buf[4];
	int ret = 0;
	int retry = 2;
	u8 i2c_addr[2];
	struct i2c_client *client = sc080cs->i2c_client;
	struct i2c_msg msgs[2];

	i2c_addr[0] = (address & 0xff00)>>8;
	i2c_addr[1] = (address & 0xff);

	msgs[0].flags = 0;
	msgs[0].addr = (client->addr);
	msgs[0].len = 2;
	msgs[0].buf = i2c_addr;

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr = (client->addr);
	msgs[1].len = 1;
	msgs[1].buf = i2c_buf;

	while (retry > 0) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		qvga_dev_err(&client->dev, "%s: read step1 ret:%d  msgs[1].addr=%x\n", __func__, ret, msgs[1].addr);
		if (retry > 0)
		{
			mdelay(20);
			if (ret == 2) {
				*rxdata = i2c_buf[0];
				return ret;
			} else {
				qvga_dev_err(&client->dev, "%s: sc080cs_i2c_read fail\n", __func__);
				break;
			}
		}
		retry--;
		mdelay(2);
	}
	return -1;
}
static int sp0821_i2c_read(struct qvga *sp0821,
			    unsigned char reg_addr, unsigned char *reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < 1) {
		ret = i2c_smbus_read_byte_data(sp0821->i2c_client, reg_addr);
		if (ret < 0) {
			qvga_dev_err(sp0821->dev,
				   "%s: i2c_read cnt=%d error=%d\n", __func__,
				   cnt, ret);
		} else {
			*reg_data = ret;
			return ret;
		}
		cnt++;
		mdelay(2);
	}

	return -1;
}
static struct qvga *qvga_malloc_init(struct i2c_client *client)
{
	struct qvga *qvga =
	    devm_kzalloc(&client->dev, sizeof(struct qvga), GFP_KERNEL);
	if (qvga == NULL) {
		qvga_dev_err(&client->dev, "%s: devm_kzalloc failed.\n", __func__);
		return NULL;
	}

	qvga->i2c_client = client;

	pr_info("%s enter , client_addr = 0x%02x\n", __func__,
		qvga->i2c_client->addr);

	return qvga;
}

void sc080cs_Init(struct qvga *sc080cs)
{
    /*SYS  MCLK=24MHZ*/
	sc080cs_i2c_write(sc080cs, 0x0103,0x01);
	sc080cs_i2c_write(sc080cs, 0x0100,0x00);
	sc080cs_i2c_write(sc080cs, 0x309b,0xf0);
	sc080cs_i2c_write(sc080cs, 0x30b0,0x0a);
	sc080cs_i2c_write(sc080cs, 0x30b8,0x21);
	sc080cs_i2c_write(sc080cs, 0x320c,0x01);
	sc080cs_i2c_write(sc080cs, 0x320d,0x6a);
	sc080cs_i2c_write(sc080cs, 0x320e,0x01);
	sc080cs_i2c_write(sc080cs, 0x320f,0xba);
	sc080cs_i2c_write(sc080cs, 0x3301,0x04);
	sc080cs_i2c_write(sc080cs, 0x3304,0x0c);
	sc080cs_i2c_write(sc080cs, 0x3305,0x00);
	sc080cs_i2c_write(sc080cs, 0x3306,0x10);
	sc080cs_i2c_write(sc080cs, 0x3307,0x02);
	sc080cs_i2c_write(sc080cs, 0x3308,0x04);
	sc080cs_i2c_write(sc080cs, 0x330a,0x00);
	sc080cs_i2c_write(sc080cs, 0x330b,0x30);
	sc080cs_i2c_write(sc080cs, 0x330e,0x01);
	sc080cs_i2c_write(sc080cs, 0x330f,0x01);
	sc080cs_i2c_write(sc080cs, 0x3310,0x01);
	sc080cs_i2c_write(sc080cs, 0x331e,0x09);
	sc080cs_i2c_write(sc080cs, 0x3333,0x10);
	sc080cs_i2c_write(sc080cs, 0x3334,0x40);
	sc080cs_i2c_write(sc080cs, 0x334c,0x01);
	sc080cs_i2c_write(sc080cs, 0x33b3,0x3e);
	sc080cs_i2c_write(sc080cs, 0x349f,0x02);
	sc080cs_i2c_write(sc080cs, 0x34a6,0x01);
	sc080cs_i2c_write(sc080cs, 0x34a7,0x07);
	sc080cs_i2c_write(sc080cs, 0x34a8,0x3a);
	sc080cs_i2c_write(sc080cs, 0x34a9,0x38);
	sc080cs_i2c_write(sc080cs, 0x34e9,0x38);
	sc080cs_i2c_write(sc080cs, 0x34f8,0x07);
	sc080cs_i2c_write(sc080cs, 0x3630,0x65);
	sc080cs_i2c_write(sc080cs, 0x3637,0x47);
	sc080cs_i2c_write(sc080cs, 0x363a,0xe0);
	sc080cs_i2c_write(sc080cs, 0x3670,0x03);
	sc080cs_i2c_write(sc080cs, 0x3674,0x75);
	sc080cs_i2c_write(sc080cs, 0x3675,0x65);
	sc080cs_i2c_write(sc080cs, 0x3676,0x65);
	sc080cs_i2c_write(sc080cs, 0x367c,0x01);
	sc080cs_i2c_write(sc080cs, 0x367d,0x03);
	sc080cs_i2c_write(sc080cs, 0x3690,0xe0);
	sc080cs_i2c_write(sc080cs, 0x3691,0xe1);
	sc080cs_i2c_write(sc080cs, 0x3692,0xe1);
	sc080cs_i2c_write(sc080cs, 0x3693,0xe1);
	sc080cs_i2c_write(sc080cs, 0x3694,0x03);
	sc080cs_i2c_write(sc080cs, 0x3695,0x07);
	sc080cs_i2c_write(sc080cs, 0x3696,0x07);
	sc080cs_i2c_write(sc080cs, 0x37f9,0x29);
	sc080cs_i2c_write(sc080cs, 0x3900,0x91);
	sc080cs_i2c_write(sc080cs, 0x3904,0x0f);
	sc080cs_i2c_write(sc080cs, 0x3908,0x00);
	sc080cs_i2c_write(sc080cs, 0x391b,0x07);
	sc080cs_i2c_write(sc080cs, 0x391c,0x0a);
	sc080cs_i2c_write(sc080cs, 0x391d,0x15);
	sc080cs_i2c_write(sc080cs, 0x391e,0x28);
	sc080cs_i2c_write(sc080cs, 0x391f,0x41);
	sc080cs_i2c_write(sc080cs, 0x3948,0x00);//blc
	sc080cs_i2c_write(sc080cs, 0x4509,0x10);
	sc080cs_i2c_write(sc080cs, 0x470b,0x0a);
	sc080cs_i2c_write(sc080cs, 0x470d,0x06);
	sc080cs_i2c_write(sc080cs, 0x5000,0xc2);
	sc080cs_i2c_write(sc080cs, 0x5001,0x01);
	sc080cs_i2c_write(sc080cs, 0x5170,0x2c);
	sc080cs_i2c_write(sc080cs, 0x5172,0xc1);
	sc080cs_i2c_write(sc080cs, 0x518b,0x00);//again
	sc080cs_i2c_write(sc080cs, 0x518c,0x20);
	sc080cs_i2c_write(sc080cs, 0x518d,0x01);//shutter
	sc080cs_i2c_write(sc080cs, 0x518e,0x7c);
	sc080cs_i2c_write(sc080cs, 0x518f,0x00);
	sc080cs_i2c_write(sc080cs, 0x519e,0x10);
	sc080cs_i2c_write(sc080cs, 0x300a,0x00);//SIP input
	sc080cs_i2c_write(sc080cs, 0x0100,0x01);
	/*shutter gain must write after stream on */
	sc080cs_i2c_write(sc080cs, 0x518b,0x03);//again=4x
	sc080cs_i2c_write(sc080cs, 0x518c,0x20);
	sc080cs_i2c_write(sc080cs, 0x518d,0x01);//shutter=20ms
	sc080cs_i2c_write(sc080cs, 0x518e,0xb0);
	sc080cs_i2c_write(sc080cs, 0x518f,0x00);
	sc080cs_i2c_write(sc080cs, 0x519e,0x10);

}   /*    sensor_init  */

void sp0821_Init(struct qvga *sp0821)
{
    /*SYS*/
	sp0821_i2c_write(sp0821, 0x30, 0x00);
	qvga_dev_info(sp0821->dev, "%s zho0 qvga reg[0x30]\n", __func__);
	sp0821_i2c_write(sp0821, 0x32, 0x00);
	sp0821_i2c_write(sp0821, 0x03, 0x01);
	sp0821_i2c_write(sp0821, 0x04, 0x2c);
	sp0821_i2c_write(sp0821, 0x24, 0x80);
	sp0821_i2c_write(sp0821, 0x9b, 0x32);
	sp0821_i2c_write(sp0821, 0xd7, 0x00);
	sp0821_i2c_write(sp0821, 0xc5, 0xc7);
	sp0821_i2c_write(sp0821, 0xc6, 0xe2);
	sp0821_i2c_write(sp0821, 0xe7, 0x03);
	qvga_dev_info(sp0821->dev, "%s zho1 qvga reg[0xe7]\n", __func__);
	sp0821_i2c_write(sp0821, 0x32, 0x00);
	sp0821_i2c_write(sp0821, 0x32, 0x01);
	sp0821_i2c_write(sp0821, 0x32, 0x00);
	sp0821_i2c_write(sp0821, 0xbf, 0x0f);
	sp0821_i2c_write(sp0821, 0xba, 0x5a);
	sp0821_i2c_write(sp0821, 0xbb, 0x69);
	sp0821_i2c_write(sp0821, 0xe7, 0x00);
	sp0821_i2c_write(sp0821, 0x32, 0x07);
	sp0821_i2c_write(sp0821, 0x31, 0x03);
	sp0821_i2c_write(sp0821, 0x19, 0x04);
	sp0821_i2c_write(sp0821, 0x2c, 0x0f);
	sp0821_i2c_write(sp0821, 0x2e, 0x7c);
	qvga_dev_info(sp0821->dev, "%s zho2 qvga reg[0x2e]\n", __func__);
	sp0821_i2c_write(sp0821, 0x30, 0x00);
	sp0821_i2c_write(sp0821, 0x28, 0x2e);
	sp0821_i2c_write(sp0821, 0x29, 0x1f);
	sp0821_i2c_write(sp0821, 0x0f, 0x30);
	sp0821_i2c_write(sp0821, 0x14, 0xb0);
	sp0821_i2c_write(sp0821, 0x38, 0x50);
	sp0821_i2c_write(sp0821, 0x39, 0x52);
	sp0821_i2c_write(sp0821, 0x3a, 0x60);
	sp0821_i2c_write(sp0821, 0x3b, 0x10);
	sp0821_i2c_write(sp0821, 0x3c, 0xe0);
	sp0821_i2c_write(sp0821, 0x85, 0x01);
	sp0821_i2c_write(sp0821, 0xe0, 0x02);
	qvga_dev_info(sp0821->dev, "%s zho3 qvga reg[0xe0]\n", __func__);
	sp0821_i2c_write(sp0821, 0xe5, 0x60);
	sp0821_i2c_write(sp0821, 0xf5, 0x02);
	sp0821_i2c_write(sp0821, 0xf1, 0x03);
	sp0821_i2c_write(sp0821, 0xf3, 0x40);
	sp0821_i2c_write(sp0821, 0x41, 0x00);
	sp0821_i2c_write(sp0821, 0x05, 0x00);
	sp0821_i2c_write(sp0821, 0x06, 0x00);
	sp0821_i2c_write(sp0821, 0x07, 0x00);
	sp0821_i2c_write(sp0821, 0x08, 0x00);
	sp0821_i2c_write(sp0821, 0x09, 0x00);
	sp0821_i2c_write(sp0821, 0x0a, 0x34);
	sp0821_i2c_write(sp0821, 0x0D, 0x01);
	sp0821_i2c_write(sp0821, 0xc8, 0x10);
	sp0821_i2c_write(sp0821, 0x29, 0x1e);
	qvga_dev_info(sp0821->dev, "%s zho4 qvga reg[0x29]\n", __func__);
	sp0821_i2c_write(sp0821, 0xa2, 0x26);
	sp0821_i2c_write(sp0821, 0xa3, 0x02);
	sp0821_i2c_write(sp0821, 0xa4, 0x32);
	sp0821_i2c_write(sp0821, 0xa5, 0x00);
	sp0821_i2c_write(sp0821, 0xa8, 0x32);
	sp0821_i2c_write(sp0821, 0xa9, 0x00);
	sp0821_i2c_write(sp0821, 0xaa, 0x01);
	sp0821_i2c_write(sp0821, 0xab, 0x00);
	sp0821_i2c_write(sp0821, 0x4c, 0x80);
	sp0821_i2c_write(sp0821, 0x4d, 0x80);
	sp0821_i2c_write(sp0821, 0xa6, 0xf0);
	sp0821_i2c_write(sp0821, 0xa7, 0x20);
	sp0821_i2c_write(sp0821, 0xac, 0xf0);
	sp0821_i2c_write(sp0821, 0xad, 0x20);
	sp0821_i2c_write(sp0821, 0x8a, 0x3e);
	qvga_dev_info(sp0821->dev, "%s zho5 qvga reg[0x8a]\n", __func__);
	sp0821_i2c_write(sp0821, 0x8b, 0x30);
	sp0821_i2c_write(sp0821, 0x8c, 0x2a);
	sp0821_i2c_write(sp0821, 0x8d, 0x26);
	sp0821_i2c_write(sp0821, 0x8e, 0x26);
	sp0821_i2c_write(sp0821, 0x8f, 0x24);
	sp0821_i2c_write(sp0821, 0x90, 0x24);
	sp0821_i2c_write(sp0821, 0x91, 0x22);
	sp0821_i2c_write(sp0821, 0x92, 0x22);
	sp0821_i2c_write(sp0821, 0x93, 0x22);
	sp0821_i2c_write(sp0821, 0x94, 0x20);
	sp0821_i2c_write(sp0821, 0x95, 0x20);
	sp0821_i2c_write(sp0821, 0x96, 0x20);
	sp0821_i2c_write(sp0821, 0x17, 0x88);
	sp0821_i2c_write(sp0821, 0x18, 0x80);
	sp0821_i2c_write(sp0821, 0x4e, 0x78);
	sp0821_i2c_write(sp0821, 0x4f, 0x78);
	sp0821_i2c_write(sp0821, 0x58, 0x8a);
	sp0821_i2c_write(sp0821, 0x59, 0xa8);
	sp0821_i2c_write(sp0821, 0x5a, 0x80);
	qvga_dev_info(sp0821->dev, "%s zho6 qvga reg[0x5a]\n", __func__);
	sp0821_i2c_write(sp0821, 0xca, 0x00);
	sp0821_i2c_write(sp0821, 0x86, 0x08);
	sp0821_i2c_write(sp0821, 0x87, 0x0f);
	sp0821_i2c_write(sp0821, 0x88, 0x30);
	sp0821_i2c_write(sp0821, 0x89, 0x45);
	sp0821_i2c_write(sp0821, 0x9e, 0x94);
	sp0821_i2c_write(sp0821, 0x9f, 0x88);
	sp0821_i2c_write(sp0821, 0x97, 0x84);
	sp0821_i2c_write(sp0821, 0x98, 0x88);
	sp0821_i2c_write(sp0821, 0x99, 0x74);
	sp0821_i2c_write(sp0821, 0x9a, 0x84);
	sp0821_i2c_write(sp0821, 0xa0, 0x7c);
	sp0821_i2c_write(sp0821, 0xa1, 0x78);
	sp0821_i2c_write(sp0821, 0x9d, 0x09);
	sp0821_i2c_write(sp0821, 0xB1, 0x04);
	sp0821_i2c_write(sp0821, 0xb3, 0x00);
	qvga_dev_info(sp0821->dev, "%s zho7 qvga reg[0xb3]\n", __func__);
	sp0821_i2c_write(sp0821, 0x47, 0x40);
	sp0821_i2c_write(sp0821, 0xb8, 0x04);
	sp0821_i2c_write(sp0821, 0xb9, 0x28);
	sp0821_i2c_write(sp0821, 0x3f, 0x18);
	sp0821_i2c_write(sp0821, 0xc1, 0xff);
	sp0821_i2c_write(sp0821, 0xc2, 0x40);
	sp0821_i2c_write(sp0821, 0xc3, 0xff);
	sp0821_i2c_write(sp0821, 0xc4, 0x40);
	sp0821_i2c_write(sp0821, 0xc5, 0xc7);
	sp0821_i2c_write(sp0821, 0xc6, 0xe2);
	sp0821_i2c_write(sp0821, 0xc7, 0xef);
	sp0821_i2c_write(sp0821, 0xc8, 0x10);
	sp0821_i2c_write(sp0821, 0x50, 0x2a);
	sp0821_i2c_write(sp0821, 0x51, 0x2a);
	sp0821_i2c_write(sp0821, 0x52, 0x2f);
	sp0821_i2c_write(sp0821, 0x53, 0xcf);
	sp0821_i2c_write(sp0821, 0x54, 0xd0);
	qvga_dev_info(sp0821->dev, "%s zho8 qvga reg[0x54]\n", __func__);
	sp0821_i2c_write(sp0821, 0x5c, 0x1e);
	sp0821_i2c_write(sp0821, 0x5d, 0x21);
	sp0821_i2c_write(sp0821, 0x5e, 0x1a);
	sp0821_i2c_write(sp0821, 0x5f, 0xe9);
	sp0821_i2c_write(sp0821, 0x60, 0x98);
	sp0821_i2c_write(sp0821, 0xcb, 0x3f);
	sp0821_i2c_write(sp0821, 0xcc, 0x3f);
	sp0821_i2c_write(sp0821, 0xcd, 0x3f);
	sp0821_i2c_write(sp0821, 0xce, 0x85);
	sp0821_i2c_write(sp0821, 0xcf, 0xff);
	sp0821_i2c_write(sp0821, 0x79, 0x5a);
	sp0821_i2c_write(sp0821, 0x7a, 0xDC);
	sp0821_i2c_write(sp0821, 0x7b, 0x0A);
	sp0821_i2c_write(sp0821, 0x7c, 0xFD);
	sp0821_i2c_write(sp0821, 0x7d, 0x46);
	sp0821_i2c_write(sp0821, 0x7e, 0xFD);
	sp0821_i2c_write(sp0821, 0x7f, 0xFD);
	sp0821_i2c_write(sp0821, 0x80, 0xEF);
	sp0821_i2c_write(sp0821, 0x81, 0x54);
	qvga_dev_info(sp0821->dev, "%s zho9 qvga reg[0x81]\n", __func__);
	sp0821_i2c_write(sp0821, 0x1b, 0x0a);
	sp0821_i2c_write(sp0821, 0x1c, 0x0f);
	sp0821_i2c_write(sp0821, 0x1d, 0x15);
	sp0821_i2c_write(sp0821, 0x1e, 0x15);
	sp0821_i2c_write(sp0821, 0x1f, 0x15);
	sp0821_i2c_write(sp0821, 0x20, 0x1f);
	sp0821_i2c_write(sp0821, 0x21, 0x2a);
	sp0821_i2c_write(sp0821, 0x22, 0x2a);
	sp0821_i2c_write(sp0821, 0x56, 0x49);
	sp0821_i2c_write(sp0821, 0x1a, 0x14);
	sp0821_i2c_write(sp0821, 0x34, 0x1f);
	sp0821_i2c_write(sp0821, 0x82, 0x10);
	sp0821_i2c_write(sp0821, 0x83, 0x00);
	qvga_dev_info(sp0821->dev, "%s zho10 qvga reg[0x83]\n", __func__);
	sp0821_i2c_write(sp0821, 0x84, 0xff);
	sp0821_i2c_write(sp0821, 0xd7, 0x50);
	sp0821_i2c_write(sp0821, 0xd8, 0x1a);
	sp0821_i2c_write(sp0821, 0xd9, 0x20);
	sp0821_i2c_write(sp0821, 0xc9, 0x1f);
	sp0821_i2c_write(sp0821, 0xbf, 0x33);
	sp0821_i2c_write(sp0821, 0xba, 0x37);
	sp0821_i2c_write(sp0821, 0xbb, 0x38);
	qvga_dev_info(sp0821->dev, "%s zho11 qvga reg[0xbb]\n", __func__);
}   /*    sensor_init  */

int sc080cs_GetSensorID(struct qvga *sc080cs)
{
	int retry = 2;
	unsigned short reg_data = 0x00;
	unsigned char msb;
	unsigned char lsb;

	do {
        sc080cs_i2c_read(sc080cs, 0x3107, &msb);
        sc080cs_i2c_read(sc080cs, 0x3108, &lsb);
        reg_data = (msb << 8) | lsb;
		qvga_dev_err(sc080cs->dev, "drv-%s: Read MSB Sensor ID = 0x%x\n", __func__, reg_data);
		if (reg_data == SC080CS_SENSOR_ID) {
			qvga_dev_err(sc080cs->dev, "drv-%s: Read Sensor ID sucess = 0x%x\n", __func__, reg_data);
			driver_flag = 1;
			return 0;
		} else {
			qvga_dev_err(sc080cs->dev, "rv-%s: Read Sensor ID Fail = 0x%x\n", __func__, reg_data);
			driver_flag = 0;
		}
		mdelay(10);
		retry--;
	} while (retry > 0);

	return -1;
}
int sp0821_GetSensorID(struct qvga *sp0821)
{
	int retry = 5;
	int len;
	unsigned char reg_data = 0x00;

	do {
		len = sp0821_i2c_read(sp0821, 0x02, &reg_data);
		qvga_dev_err(sp0821->dev, "drv-%s: Read MSB Sensor ID sucess = 0x%02x\n", __func__, reg_data);
		if (reg_data == SP0821_SENSOR_ID) {
			qvga_dev_err(sp0821->dev, "drv-%s: Read Sensor ID sucess = 0x%02x\n", __func__, reg_data);
			driver_flag = 1;
			return 0;
		} else {
			qvga_dev_err(sp0821->dev, "rv-%s: Read Sensor ID Fail = 0x%02x\n", __func__, reg_data);
			driver_flag = 0;
		}
		mdelay(10);
		retry--;
	} while (retry > 0);

	return -1;
}

static ssize_t qvga_get_name(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	pr_err("get_name entry");

	if (driver_flag) {
		if (qvga_sensor_info == QVGA_SENSOR_SC080CS) {
			len += snprintf(buf + len, PAGE_SIZE - len, "%s\n",
					"sc080cs");
			pr_err("get_name :【sc080cs】【len:%d】",len);
		} else if (qvga_sensor_info == QVGA_SENSOR_SP0821){
			len += snprintf(buf + len, PAGE_SIZE - len, "%s\n",
					"sp0821");
			pr_err("get_name :【sp0821】【len:%d】",len);
		}
	} else {
		len += snprintf(buf + len, PAGE_SIZE - len, "%s\n",
				"none");
		pr_err("get_name 【none】【len:%d】",len);
	}

	return len;
}

static ssize_t qvga_get_reg(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	pr_err("Enter get_reg function [flag:%d]\n", read_reg_flag);

	if (read_reg_flag) {
		len += snprintf(buf + len, PAGE_SIZE - len, "The reg 0x%02X value is 0x%02X\n",
				read_reg_id, read_reg_value);
		pr_err("Read register value: 0x%x\n", read_reg_value);
		read_reg_flag = 0;
		read_reg_id = 0;
		read_reg_value = 0;
	} else {
		len += snprintf(buf + len, PAGE_SIZE - len, "Please echo reg id into reg\n");
	}

	return len;
}

static ssize_t qvga_set_reg(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t len)
{
	unsigned int databuf[2] = { 0 };
	unsigned char reg_data = 0x00;

	pr_err("set_reg entry \n");
	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		pr_err("Set register: [Address:0x%x] [Value:0x%x]\n", databuf[0], databuf[1]);
		if (qvga_sensor_info == QVGA_SENSOR_SC080CS) {
			sc080cs_i2c_write(qvga, databuf[0], databuf[1]);
		} else if (qvga_sensor_info == QVGA_SENSOR_SP0821){
			sp0821_i2c_write(qvga, databuf[0], databuf[1]);
		}

	}
	else if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 1) {
		pr_err("Read register: [Address:0x%x]\n", databuf[0]);
		if (qvga_sensor_info == QVGA_SENSOR_SC080CS) {
			sc080cs_i2c_read(qvga, databuf[0], &reg_data);
		} else if (qvga_sensor_info == QVGA_SENSOR_SP0821){
			sp0821_i2c_read(qvga, databuf[0], &reg_data);
		}

		read_reg_id = databuf[0];
		read_reg_value = reg_data;
		read_reg_flag = 1;
	}

	return len;
}

static ssize_t qvga_get_light(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	unsigned char reg_data1 = 0x00;
	//unsigned char reg_data2 = 0x00;
	u16 light = 0;
	int ret = 0;


	if (qvga_sensor_info == QVGA_SENSOR_SC080CS) {
		ret = sc080cs_i2c_read(qvga, 0x5160, &reg_data1);
	} else if (qvga_sensor_info == QVGA_SENSOR_SP0821){
		ret = sp0821_i2c_read(qvga, 0xb0, &reg_data1);
	}

	if (ret == -1) {
		len = snprintf(buf, PAGE_SIZE, "%d\n", ret);
		qvga_dev_err(qvga->dev, "%s: I2C read failed, returned -1\n", __func__);
		return len;
	}

	//sc080cs_i2c_read(qvga, 0x516b, &reg_data2);
	//light = (reg_data1<<8) + reg_data2;
	light = reg_data1;
	qvga_dev_err(qvga->dev, "%s: qvga light=%d\n",   __func__, light);
	len += snprintf(buf + len, PAGE_SIZE - len, "%d\n",
			light);

	return len;
}

static ssize_t qvga_set_light(struct device *dev,
                                struct device_attribute *attr, const char *buf,
                                size_t len)
{
    ssize_t ret;
    unsigned int state;
    static bool first_run = true;

    ret = kstrtouint(buf, 10, &state);
    if (ret) {
        qvga_dev_err(qvga->dev, "%s: fail to change str to int\n",
                    __func__);
        return ret;
    }

    if (state == 0) {
		qvga_dev_err(qvga->dev, "%s: power off, %d\n", __func__, state);
		if (qvga_sensor_info == QVGA_SENSOR_SC080CS) {
			qvga_sensor_open("qvga_sensor_sc080cs", SENSOR_IDX, 0);
		} else if (qvga_sensor_info == QVGA_SENSOR_SP0821) {
			qvga_sensor_open("qvga_sensor_sp0821_powerdown", SENSOR_IDX, 0);
		}
		seninf_clk_set_open(0, SENSOR_IDX);
    } else {
    	if (first_run) {
    	    first_run = false;

    	    seninf_clk_set_open(1, SENSOR_IDX);
    	    qvga_sensor_open("qvga_sensor_sc080cs", SENSOR_IDX, 1);
    	    qvga_dev_err(qvga->dev, "%s: sc080cs_i2c_addr:0x%x】\n", __func__,qvga->i2c_client->addr);
    	    ret = sc080cs_GetSensorID(qvga);
    	    if (ret < 0) {
    	        qvga_sensor_open("qvga_sensor_sc080cs", SENSOR_IDX, 0);

    	        qvga_dev_err(qvga->dev,"%s: sc080cs read sensor id failed ret=%d\n", __func__, ret);
    	        qvga_sensor_open("qvga_sensor_sp0821", SENSOR_IDX, 1);
    	        qvga->i2c_client->addr = 0x43;
    	        qvga_dev_err(qvga->dev, "%s: sp0821_i2c_addr:0x%x】\n", __func__,qvga->i2c_client->addr);
    	        ret = sp0821_GetSensorID(qvga);
    	        if (ret < 0) {
    	            qvga_dev_err(qvga->dev,"%s: sp0821 read sensor id failed ret=%d\n", __func__, ret);
    	            qvga_sensor_open("qvga_sensor_sp0821_powerdown", SENSOR_IDX, 0);
    	            seninf_clk_set_open(0, SENSOR_IDX);
    	        } else {
    	            qvga_sensor_info = QVGA_SENSOR_SP0821;
    	            udelay(4000);
    	            sp0821_Init(qvga);
    	            qvga_dev_err(qvga->dev,"%s: sp0821 read sensor id success ret=%d\n", __func__, ret);
    	            return len;
    	        }
    	    } else {
    	        qvga_sensor_info = QVGA_SENSOR_SC080CS;
    	        udelay(4000);
    	    	sc080cs_Init(qvga);
    	        qvga_dev_err(qvga->dev,"%s: sc080cs read sensor id success ret=%d\n", __func__, ret);
    	        return len;
    	    }
    	}

    	if (qvga_sensor_info == QVGA_SENSOR_SC080CS) {
    	    qvga_dev_err(qvga->dev, "%s: sc080cs_power on, %d\n", __func__, state);
    	    seninf_clk_set_open(1, SENSOR_IDX);
    	    qvga_sensor_open("qvga_sensor_sc080cs", SENSOR_IDX, 1);
    	    udelay(4000);
    	    sc080cs_Init(qvga);
    	} else if (qvga_sensor_info == QVGA_SENSOR_SP0821){
    	    qvga_dev_err(qvga->dev, "%s: sp0821_power on, %d\n", __func__, state);
    	    seninf_clk_set_open(1, SENSOR_IDX);
    	    qvga_sensor_open("qvga_sensor_sp0821", SENSOR_IDX, 1);
    	    udelay(4000);
    	    sp0821_Init(qvga);
    	}
    }

    return len;
}

static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO,
		qvga_get_reg, qvga_set_reg);
static DEVICE_ATTR(cam_name, S_IWUSR | S_IRUGO,
		qvga_get_name, NULL);
static DEVICE_ATTR(light, S_IWUSR | S_IRUGO,
		qvga_get_light, qvga_set_light);

static struct attribute *qvga_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_cam_name.attr,
	&dev_attr_light.attr,
	NULL
};

static struct attribute_group qvga_attribute_group = {
	.attrs = qvga_attributes
};

/****************************************************************************
* sc080cs i2c driver
*****************************************************************************/
static int qvga_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	int ret = -1;

	pr_err("scw %s enter , i2c%d@0x%02x\n", __func__,
		client->adapter->nr, client->addr);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		qvga_dev_err(&client->dev, "%s: check_functionality failed\n",
			   __func__);
		ret = -ENODEV;
		goto exit_check_functionality_failed;
	}

	qvga = qvga_malloc_init(client);

	qvga->i2c_seq = qvga->i2c_client->adapter->nr;
	qvga->i2c_addr = qvga->i2c_client->addr;
	if (qvga == NULL) {
		dev_err(&client->dev, "%s: failed to parse device tree node\n",
			__func__);
		ret = -ENOMEM;
		goto exit_devm_kzalloc_failed;
	}

	qvga->dev = &client->dev;
	i2c_set_clientdata(client, qvga);

	qvga_class = class_create(THIS_MODULE, "qvga_cam");
	if (IS_ERR(qvga_class)) {
		ret = PTR_ERR(qvga_class);
		pr_err("Failed to create class: %d\n", ret);
		goto err_class_create;
	}

	dev = device_create(qvga_class, NULL, client->dev.devt, NULL, "qvga_depth");
	if (IS_ERR(dev)) {
		ret = PTR_ERR(dev);
		pr_err("Failed to create device: %d\n", ret);
		goto err_device_create;
	}

	ret = sysfs_create_group(&dev->kobj, &qvga_attribute_group);
	if (ret < 0) {
		pr_err("Failed to create sysfs group: %d\n", ret);
		goto err_sysfs_group;
	}

	return 0;

err_class_create:
	devm_kfree(&client->dev, qvga);
	qvga = NULL;
err_device_create:
	class_destroy(qvga_class);
	qvga_class = NULL;
err_sysfs_group:
	device_destroy(qvga_class, client->dev.devt);
	class_destroy(qvga_class);
	qvga_class = NULL;
exit_devm_kzalloc_failed:
exit_check_functionality_failed:
	return ret;
}

static int qvga_i2c_remove(struct i2c_client *client)
{
	pr_info("%s qvga_i2c_remove\n", __func__);

	if (qvga_attribute_group.attrs) {
	    sysfs_remove_group(&dev->kobj, &qvga_attribute_group);
	    pr_err("sysfs attribute group removed");
	} else {
	    pr_err("sysfs attribute group not registered");
	}

	if (qvga_class && client) {
	    device_destroy(qvga_class, client->dev.devt);
	    pr_err("qvga_device destroyed successfully");
	} else {
	    pr_err("qvga_device does not exist");
	}

	if (qvga_class) {
	    class_destroy(qvga_class);
	    pr_err("qvga_class destroyed successfully");
	    qvga_class = NULL;
	} else {
	    pr_err("qvga_class does not exist");
	}

	return 0;
}

static const struct of_device_id qvga_of_match[] = {
	{.compatible = "sc,sc_sc080cs_iii"},
	{},
};

static struct i2c_driver qvga_i2c_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name =  "sc_sc080cs_iii",
		   .of_match_table = qvga_of_match,
		   },
	.probe = qvga_i2c_probe,
	.remove = qvga_i2c_remove,
};

static int __init qvga_yuv_init(void)
{
	int ret;

	pr_info("%s: driver version: %s\n", __func__,
				QVGA_DRIVER_VERSION);

	ret = i2c_add_driver(&qvga_i2c_driver);
	if (ret) {
		pr_info("****[%s] Unable to register driver (%d)\n",
			__func__, ret);
		return ret;
	}
	return 0;
}

static void __exit qvga_yuv_exit(void)
{
	pr_info("%s enter\n", __func__);
	i2c_del_driver(&qvga_i2c_driver);
}

module_init(qvga_yuv_init);
module_exit(qvga_yuv_exit);

MODULE_AUTHOR("v-weibiao@vanyol.com>");
MODULE_DESCRIPTION("qvga yuv driver");
MODULE_LICENSE("GPL v2");


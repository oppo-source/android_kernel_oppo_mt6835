/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */
#define PFX "CAM_CAL"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/of.h>
#include "cam_cal.h"
#include "cam_cal_define.h"
//#include "cam_cal_list.h"
#include <linux/dma-mapping.h>
#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif
#include "kd_camera_typedef.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_typedef.h"
//#include "eeprom_i2c_custom_driver.c"
#define EEPROM_I2C_MSG_SIZE_READ 2
#include "gc08a8_2_otp.h"

static struct i2c_client *g_pstI2CclientG;

#define GC08A8_OTP_DEBUG  0
#define GC08A8_I2C_ID     0x20 /*0x20*/

static int iReadRegI2C1(struct i2c_client *client,
		u8 *a_pSendData, u16 a_sizeSendData,
		u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId)
{
	int i4RetValue = 0;
	struct i2c_msg msg[EEPROM_I2C_MSG_SIZE_READ];

	client->addr = (i2cId >> 1);

	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = a_sizeSendData;
	msg[0].buf = a_pSendData;

	msg[1].addr = client->addr;
	msg[1].flags = client->flags & I2C_M_TEN;
	msg[1].flags |= I2C_M_RD;
	msg[1].len = a_sizeRecvData;
	msg[1].buf = a_pRecvData;

	i4RetValue = i2c_transfer(client->adapter, msg,
				EEPROM_I2C_MSG_SIZE_READ);

	if (i4RetValue != EEPROM_I2C_MSG_SIZE_READ) {
		pr_debug("I2C read failed!!\n");
		return -1;
	}
	return 0;
}

static int iWriteRegI2C(u8 *a_pSendData, u16 a_sizeSendData)
{
	int  i4RetValue = 0;

	i4RetValue = i2c_master_send(g_pstI2CclientG,
		a_pSendData, a_sizeSendData);
	if (i4RetValue != a_sizeSendData) {
		pr_debug("I2C send failed!!, Addr = 0x%x\n", a_pSendData[0]);
		return -1;
	}
	return 0;

}

static u16 read_cmos_sensor(u16 addr)
{
	u16 get_byte=0;
	char pu_send_cmd[2] = { (char)((addr >> 8) & 0xff), (char)(addr & 0xff) };

	iReadRegI2C1(g_pstI2CclientG,pu_send_cmd, 2, (u8 *)&get_byte, 1, GC08A8_I2C_ID);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[4] = {(char)(addr >> 8),
		(char)(addr & 0xFF), (char)(para & 0xFF)};

	iWriteRegI2C(pu_send_cmd, 3);
}

static void gc08a8_2_otp_init(void)
{
	write_cmos_sensor(0x0315, 0x80);
	write_cmos_sensor(0x031c, 0x60);

	write_cmos_sensor(0x0324, 0x42);
	write_cmos_sensor(0x0316, 0x09);
	write_cmos_sensor(0x0a67, 0x80);
	write_cmos_sensor(0x0313, 0x00);
	write_cmos_sensor(0x0a53, 0x0e);
	write_cmos_sensor(0x0a65, 0x17);
	write_cmos_sensor(0x0a68, 0xa1);
	write_cmos_sensor(0x0a47, 0x00);
	write_cmos_sensor(0x0a58, 0x00);
	write_cmos_sensor(0x0ace, 0x0c);
}

static void gc08a8_2_otp_close(void)
{
	write_cmos_sensor(0x0316, 0x01);
	write_cmos_sensor(0x0a67, 0x00);
}

static u16 gc08a8_2_otp_read_group(u16 addr, u8 *data, u16 length)
{
	u16 i = 0;

	write_cmos_sensor(0x0313, 0x00);
	write_cmos_sensor(0x0a69, (addr >> 8) & 0xff);
	write_cmos_sensor(0x0a6a, addr & 0xff);
	write_cmos_sensor(0x0313, 0x20);
	write_cmos_sensor(0x0313, 0x12);

	for (i = 0; i < length; i++) {
		data[i] = read_cmos_sensor(0x0a6c);
#if GC08A8_OTP_DEBUG
	pr_debug("-----gc08a8_2 otp addr = 0x%x, data = 0x%x\n", addr + i * 8, data[i]);
#endif
	}
	return 0;
}


int gc08a8_2_iReadData(unsigned int ui4_offset,
	unsigned int ui4_length, unsigned char *pinputdata)
{
	int i4RetValue = 0;

	pr_debug("gc08a8_2 otp ui4_offset = 0x%x, ui4_length = %d \n", ui4_offset, ui4_length);

	gc08a8_2_otp_init();
	mdelay(10);

	i4RetValue = gc08a8_2_otp_read_group(ui4_offset, pinputdata, ui4_length);
	if (i4RetValue != 0) {
		pr_debug("I2C iReadData failed!!\n");
		return -1;
	}

	gc08a8_2_otp_close();

	return 0;
}

unsigned int Gc08a8_2_read_region(struct i2c_client *client, unsigned int addr,
				unsigned char *data, unsigned int size)
{
	g_pstI2CclientG = client;
	if (gc08a8_2_iReadData(addr, size, data) == 0)
		return size;
	else
		return 0;
}


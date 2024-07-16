/***********************************************************
 * ** Copyright (C), 2008-2016, OPPO Mobile Comm Corp., Ltd.
 * ** ODM_HQ_EDIT
 * ** File: - otp_insensor_dev.c
 * ** Description: Source file for CBufferList.
 * **           To allocate and free memory block safely.
 * ** Version: 1.0
 * ** Date : 2018/12/07
 * ** Author: YanKun.Zhai@Mutimedia.camera.driver.otp
 * **
 * ** ------------------------------- Revision History: -------------------------------
 * **   <author>History<data>      <version >version       <desc>
 * **  YanKun.Zhai 2018/12/07     1.0     build this module
 * **
 * ****************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/slab.h>

#include "cam_cal.h"
#include "cam_cal_define.h"
#include "cam_cal_list.h"

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#ifdef ODM_HQ_EDIT
/*Houbing.Peng@ODM_HQ Cam.Drv 20200915 add for otp*/
#include <soc/oplus/system/oplus_project.h>
#endif

#define LOG_TAG "OTP_InSensor"
#define LOG_ERR(format, ...) pr_err(LOG_TAG "Line:%d  FUNC: %s:  "format, __LINE__, __func__, ## __VA_ARGS__)
#define LOG_INFO(format, ...) pr_info(LOG_TAG "  %s:  "format, __func__, ## __VA_ARGS__)
#define LOG_DEBUG(format, ...) pr_debug(LOG_TAG "  %s:  "format, __func__, ## __VA_ARGS__)

#define ERROR_I2C       1
#define ERROR_CHECKSUM  2
#define ERROR_READ_FLAG 3
#define OTP_THREAHOLD   3

static DEFINE_SPINLOCK(g_spinLock);

static struct i2c_client *g_pstI2CclientG;
/* add for linux-4.4 */
#ifndef I2C_WR_FLAG
#define I2C_WR_FLAG		(0x1000)
#define I2C_MASK_FLAG	(0x00ff)
#endif

struct i2c_client *g_pstI2Cclients[3]; /* I2C_DEV_IDX_MAX */

#define MAX_EEPROM_BYTE 0x1FFF
#define CHECKSUM_FLAG_ADDR MAX_EEPROM_BYTE-1
#define READ_FLAG_ADDR MAX_EEPROM_BYTE-2

#define OTP_DATA_GOOD_FLAG 0x88
char g_otp_buf[3][MAX_EEPROM_BYTE] = {{0}, {0}, {0}};

#define MAX_NAME_LENGTH 20
#define MAX_GROUP_NUM 8
#define MAX_GROUP_ADDR_NUM 3


typedef struct
{
	int group_start_addr;
	int group_end_addr;
	int group_flag_addr;
	int group_checksum_addr;
}GROUP_ADDR_INFO;

typedef struct
{
	char group_name[MAX_NAME_LENGTH];
	GROUP_ADDR_INFO group_addr_info[MAX_GROUP_ADDR_NUM];
}GROUP_INFO;


typedef struct
{
	char module_name[MAX_NAME_LENGTH];
	int group_num;
	int group_addr_info_num;
	GROUP_INFO group_info[MAX_GROUP_NUM];
	int  (* readFunc) (u16 addr, u8 *data);
}OTP_MAP;

/*This sensor only have two group otp data.in diffrent page in sensor*/
int sc520cs_read_data(u16 addr, u8 *data);
OTP_MAP sc520cs_otp_map_avatar5 = {
		.module_name = "SC520CS_AVATAR5",
		.group_num = 3,
		.group_addr_info_num = 2,
		.readFunc = sc520cs_read_data,
		.group_info = {
						{"awb",
							{
								{0x80B7, 0x80BE, 0x80B6, 0x80BF},
								{0x888E, 0x8895, 0x80B6, 0x8896},
							},
						},
						{"lsc",
							{
								{0x80C0, 0x8871, 0x80B6, 0x8872},
								{0x8897, 0x8FE2, 0x80B6, 0x8FE3},
							},
						},
						{"sn",
							{
								{0x8873, 0x888C, 0x80B6, 0x888D},
								{0x8FE4, 0x8FFD, 0x80B6, 0x8FFE},
							},
						},
			},
};

static int read_reg16_data8(u16 addr, u8 *data)
{
	int ret = 0;
	char puSendCmd[2] = {(char)(addr >> 8), (char)(addr & 0xff)};

	spin_lock(&g_spinLock);
		g_pstI2CclientG->addr =
			g_pstI2CclientG->addr & (I2C_MASK_FLAG | I2C_WR_FLAG);
	spin_unlock(&g_spinLock);

	ret = i2c_master_send(g_pstI2CclientG, puSendCmd, 2);
	if (ret != 2) {
		pr_err("I2C send failed!!, Addr = 0x%x\n", addr);
		return -1;
	}
	ret = i2c_master_recv(g_pstI2CclientG, (char *)data, 1);
	if (ret != 1) {
		pr_err("I2C read failed!!\n");
		return -1;
	}
	spin_lock(&g_spinLock);
	g_pstI2CclientG->addr = g_pstI2CclientG->addr & I2C_MASK_FLAG;
	spin_unlock(&g_spinLock);

	return 0;
}
static int write_reg16_data8(u16 addr, u8 data)
{
	int ret = 0;
	char puSendCmd[3] = {(char)(addr >> 8), (char)(addr & 0xff),
						(char)(data & 0xff)};
	spin_lock(&g_spinLock);
	g_pstI2CclientG->addr =
			g_pstI2CclientG->addr & (I2C_MASK_FLAG | I2C_WR_FLAG);
	spin_unlock(&g_spinLock);

	ret = i2c_master_send(g_pstI2CclientG, puSendCmd, 3);

	if (ret != 3) {
		pr_err("I2C send failed!!, Addr = 0x%x\n", addr);
		return -1;
	}
	spin_lock(&g_spinLock);
	g_pstI2CclientG->addr = g_pstI2CclientG->addr & I2C_MASK_FLAG;
	spin_unlock(&g_spinLock);

	return 0;
}

void sc520cs_sensor_otp_init(int section, kal_uint8 threshold)
{
	/*0x4412, 0xxx, read 0x8000-0x87FF data��0x4412,0x01
	 read 0x8800-0x8FFF��0x4412,0x03
	 0x4407, 0xxx, read 0x8000-0x87FF��0x4407,0x0c
	 read 0x8800-0x8FFF��0x4407,0x00
	*/
	write_reg16_data8(0x36b0, 0x4c);
	write_reg16_data8(0x36b1, threshold);	/* set threshold */
	write_reg16_data8(0x36b2, 0xc1);
	if (section == 1) {
		write_reg16_data8(0x4408, (0x8000 >> 8)&0xff);        /* High byte start address for OTP load Group1 */
		write_reg16_data8(0x4409, (0x8000)&0xff);             /* Low byte start address for OTP load  Group1 */
		write_reg16_data8(0x440a, (0x87FF >> 8)&0xff);        /* High byte end address for OTP load Group1 */
		write_reg16_data8(0x440b, (0x87FF)&0xff);             /* Low byte end address for OTP load Group1 */
		write_reg16_data8(0x4401, 0x13);
		write_reg16_data8(0x4412, 0x01);
		write_reg16_data8(0x4407, 0x0c);
		write_reg16_data8(0x4400, 0x11);                      /* 0x4400, 0x11 manual load */
		LOG_INFO("load section 1 \n");
	} else if (section == 2) {
		write_reg16_data8(0x4408, (0x8800 >> 8)&0xff);        /* High byte start address for OTP load Group1 */
		write_reg16_data8(0x4409, (0x8800)&0xff);             /* Low byte start address for OTP load  Group1 */
		write_reg16_data8(0x440a, (0x8FFF >> 8)&0xff);        /* High byte end address for OTP load Group1 */
		write_reg16_data8(0x440b, (0x8FFF)&0xff);             /* Low byte end address for OTP load Group1 */
		write_reg16_data8(0x4401, 0x13);
		write_reg16_data8(0x4412, 0x03);
		write_reg16_data8(0x4407, 0x00);
		write_reg16_data8(0x4400, 0x11);	                     /* 0x4400, 0x11 manual load */
		LOG_INFO("load section 2 \n");
	} else {
		LOG_INFO("section input error \n");
	}
	mdelay(30);                                              /* wait page load finished */
}

int sc520cs_read_data(u16 addr, u8 *data)
{
	static u16 last_addr = 0;
	last_addr = addr;
	return read_reg16_data8(addr, data);
}

static int parse_otp_map_data_sc520cs(OTP_MAP * map, char * data, int temp)
{
	int i = 0;
	int addr = 0, size = 0, curr_addr = 0;
	int  ret = 0;
	char group_flag = 0;
	int checksum = -1;
	int group_layer = 0;  /*0:group1, 1:group2*/

	LOG_INFO("module: %s ......", map->module_name);

	ret = map->readFunc(0x80B6, &group_flag);
	LOG_INFO("parse_otp_map_data_sc520cs 0x%x", ret);
	if (ret < 0) {
		LOG_ERR("read group flag error addr 0x%04x", 0x80B6);
		return -ERROR_I2C;
	}
	LOG_INFO("readFunc: flag_addr = 0x%04x, value = 0x%02x\n", 0x80B6, group_flag);
	if ((group_flag & 0x3) == 0x01) {
		group_layer = 0;
		sc520cs_sensor_otp_init(1, temp);
		LOG_INFO("group 1 selected\n");
	} else if ((group_flag & 0xc) == 0x04) {
		group_layer = 1;
        sc520cs_sensor_otp_init(2, temp);
		LOG_INFO("group 2 selected\n");
	} else {
		LOG_ERR("group selected error!!!\n");
		return -ERROR_READ_FLAG;
	}

	for (i = 0; i < map->group_num; i++) {
		checksum = 0;
		size = 0;
		LOG_INFO("%d groupinfo: %s, start_addr 0x%0x(0x%0x)", group_layer, map->group_info[i].group_name, curr_addr, curr_addr);
		if (!strcmp(map->group_info[i].group_name, "lsc") && (group_layer == 0)) {
			/* There is need cross-page stitching to read, go here */
			for (addr = map->group_info[i].group_addr_info[group_layer].group_start_addr; addr <= 0x87FF; addr++) {
				ret = map->readFunc(addr, data);	/*read lsc group 2 part 2 data*/
				LOG_INFO("Get SC520CS OTP Data of: %s, addr: 0x%04x, data: 0x%04x", map->group_info[i].group_name, addr, *data);
				if (ret < 0) {
					LOG_ERR("read data error");
				}
				LOG_INFO("lsc part 1 group%d: %s, addr: 0x%04x, viraddr: 0x%04x(%04d), data: 0x%04x(%04d) ",
							group_layer + 1, map->group_info[i].group_name, addr, curr_addr, curr_addr, *data, *data);

				checksum += *data;
				curr_addr++;
				size++;
				data++;
			}

			sc520cs_sensor_otp_init(2, temp);    /*load page 2 for lsc part one data */
			for (addr = 0x8866; addr <= map->group_info[i].group_addr_info[group_layer].group_end_addr; addr++) {
				ret = map->readFunc(addr, data);    /*read lsc group 2 part 1 data*/
				LOG_INFO("Get SC520CS OTP Data of: %s, addr: 0x%04x, data: 0x%04x", map->group_info[i].group_name, addr, *data);
				if (ret < 0) {
					LOG_ERR("read data error");
				}
				LOG_INFO("lsc part 2 group %d: %s, addr: 0x%04x, viraddr: 0x%04x(%04d), data: 0x%04x(%04d)",
							group_layer, map->group_info[i].group_name, addr, curr_addr, curr_addr, *data, *data);

				checksum += *data;
				curr_addr++;
				size++;
				data++;
			}

		} else {    /*There is no cross-page stitching to read, go here*/
			for (addr = map->group_info[i].group_addr_info[group_layer].group_start_addr;
					addr <= map->group_info[i].group_addr_info[group_layer].group_end_addr;
					addr++) {
				ret = map->readFunc(addr, data);
				LOG_INFO("Get SC520CS OTP group%d Data of: %s, addr: 0x%04x, data: 0x%04x", (group_layer + 1), map->group_info[i].group_name, addr, *data);
				if (ret < 0) {
					LOG_ERR("read data error");
				}
				LOG_DEBUG("group%d: %s, addr: 0x%04x, viraddr: 0x%04x(%04d), data: 0x%04x(%04d) ",
							group_layer + 1, map->group_info[i].group_name, addr, curr_addr, curr_addr, *data, *data);

				checksum += *data;
				curr_addr++;
				size++;
				data++;
			}
		}

		checksum = checksum % 0xFF;
		ret = map->readFunc(map->group_info[i].group_addr_info[group_layer].group_checksum_addr, data);
		if (checksum == *data) {
			curr_addr++;
			size++;
			data++;
			LOG_INFO("groupinfo: %s, checksum OK c(%04d) r(%04d), viraddr: 0x%04x(%04d)", map->group_info[i].group_name, checksum, *data, *data);
		} else {
			LOG_ERR("groupinfo: %s, checksum ERROR ret=%d, checksum=%04d data=%04d", map->group_info[i].group_name, ret, checksum, *data);
			return -ERROR_CHECKSUM;
		}
		LOG_INFO("groupinfo: %s, end_addr 0x%04x(%04d) size 0x%04x(%04d)",
						map->group_info[i].group_name, curr_addr-1, curr_addr-1, size, size);
	}
	return ret;
}

unsigned int sc520cs_read_region(struct i2c_client *client, unsigned int addr, unsigned char *data, unsigned int size)
{
	int ret = 0;
	int i;
	int otpThreshold[3] = {0x38, 0x28, 0x48}; /* the Threshold for sc520cs*/
	int tempotpthreshold =  0;
	if (g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][READ_FLAG_ADDR]) {
		LOG_INFO("read otp data from g_otp_buf: addr 0x%x, size %d", addr, size);
		memcpy((void *)data, &g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][addr], size);
		return size;
	}
	if (client != NULL) {
		g_pstI2CclientG = client;
	} else if (g_pstI2Cclients[IMGSENSOR_SENSOR_IDX_SUB] != NULL) {
		g_pstI2CclientG = g_pstI2Cclients[IMGSENSOR_SENSOR_IDX_SUB];
		g_pstI2CclientG->addr = 0x20 >> 1;
	}
	for (i = 0; i < OTP_THREAHOLD; i++) {
		tempotpthreshold = otpThreshold[i];
		LOG_INFO("Get SC520CS OTP data used Threshold: 0x%x", tempotpthreshold);
		sc520cs_sensor_otp_init(1, tempotpthreshold);  /*we need reload the data page before read anther page data.*/
		ret = parse_otp_map_data_sc520cs(&sc520cs_otp_map_avatar5, &g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][0], tempotpthreshold);
		if(!ret)
			break;
	}
	if (!ret) {
		g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][CHECKSUM_FLAG_ADDR] = OTP_DATA_GOOD_FLAG;
		g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][READ_FLAG_ADDR] = 1;
	}

	if (NULL != data) {
		memcpy((void *)data, &g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][addr], size);
	}
	return ret;
}


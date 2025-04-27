/*****************************************************************************
 *
 * Filename:
 * ---------
 *     gc16b3mipi_Sensor.c
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     Source code of Sensor driver
 *
 * Driver Version:
 * ------------
 *     V0.6040.204
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "gc16b3cmipiraw_Sensor.h"
#include "gc16b3mipi_otp.h"

/****************************Modify Following Strings for Debug**************************/
#define PFX "GC16B3MIPI_OTP"
/****************************   Modify end    *******************************************/
#define GC16B3_OTP_DEBUG 1
#if GC16B3_OTP_DEBUG
#define LOG_INF(format, args...)	pr_debug(PFX "[%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif

/*for debug*/
#define DPC_DEBUG           0
#define CT_DEBUG            0

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[2] = { (char)(addr >> 8), (char)(addr & 0xff) };

	iReadRegI2C(pu_send_cmd, 2, (u8 *) &get_byte, 1, OTP_WRITE_ID);
	return get_byte;
}

static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
	char pusendcmd[3] = { (char)(addr >> 8), (char)(addr & 0xff), (char)(para & 0xff) };

	iWriteRegI2C(pusendcmd, 3, OTP_WRITE_ID);
}

static kal_uint16 read_eeprom(kal_uint16 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xff) };

	iReadRegI2C(pu_send_cmd, 2, (u8 *) &get_byte, 1, EEPROM_WRITE_ID);
	return get_byte;
}

static kal_uint8 ct_read_data[CROSSTALK_BUF_SIZE];

void gc16b3_read_crosstalk_data(void)
{
	kal_uint16 i = 0;
	kal_uint16 ct_flag = 0;
	kal_uint32 checksum = 0;
	kal_uint16 ct_checksum = 0;
    LOG_INF("start hyper crosstalk");
	memset(&ct_read_data, 0, CROSSTALK_BUF_SIZE * sizeof(kal_uint8));

	ct_flag = read_eeprom(CROSSTALK_FLAG_OFFSET);
	ct_checksum = read_eeprom(CROSSTALK_CHECKSUM_OFFSET);
	LOG_INF("ct_flag = %d\n", ct_flag);
//	ct_flag = 1;
	if (ct_flag == OTP_FLAG_VALID) {
		for (i = 0; i < CROSSTALK_BUF_SIZE; i++) {
			ct_read_data[i] = read_eeprom(CROSSTALK_START_ADDR + i);
			LOG_INF("ct_data[%d] = %x\n", i, ct_read_data[i]);
		}

		for (i = 0; i < (CROSSTALK_BUF_SIZE - 2); i++) {
			checksum += ct_read_data[i];
			LOG_INF("checksum[%d] = %x\n", i, checksum);
		}

		if ((checksum % 255) == ct_checksum) {
			LOG_INF("check success! calc_checksum = %d, read_checksum = %d\n",
			(checksum % 255), ct_read_data[CROSSTALK_BUF_SIZE - 1]);
		} else {
			memset(ct_read_data, 0, CROSSTALK_BUF_SIZE);
			LOG_INF("check error! calc_checksum = %d, read_checksum = %d\n",
			(checksum % 255), ct_read_data[CROSSTALK_BUF_SIZE - 1]);
		}
#if CT_DEBUG
		for (i = 0; i < CROSSTALK_BUF_SIZE; i++)
			LOG_INF("ct_data[%d] = %x\n", i, ct_read_data[i]);
#endif
	} else
		LOG_INF("crosstalk data is zero!\n");
}

void gc16b3_copy_crosstalk_data(kal_uint8 *ct_data)
{
	LOG_INF("start copy crosstalk_data!\n");
	memcpy(ct_data, (void *)&ct_read_data[0], CROSSTALK_BUF_SIZE);
}

static kal_uint8 gc16b3_otp_read_byte(kal_uint16 addr)
{
	mDELAY(10);
	write_cmos_sensor(0x0a69, (addr >> 8) & 0x1f);
	write_cmos_sensor(0x0a6a, addr & 0xff);
	write_cmos_sensor(0x0313, 0x20);
	return read_cmos_sensor(0x0a6c);
}

static void gc16b3_otp_read_group_ssmode(kal_uint16 addr, kal_uint8 *data, kal_uint16 length)
{
	kal_uint16 i = 0 , j = 0 ,m = 0;
	kal_uint16 group = 0,group_addr = 0x00;
	kal_uint16 group_data[64];
	memset(&group_data, 0, sizeof(group_data));
    mDELAY(10);
	write_cmos_sensor(0x0ace, 0x08);
	write_cmos_sensor(0x0ae0, 0x11);

	group = 1+(length+addr/8)/64;
	 j = addr/8;
	for (group_addr = 0; group_addr < group; group_addr++)
	{
		write_cmos_sensor(0x0a02, group_addr);
		LOG_INF("group_addr = 0x%x\n",group_addr);
		for (i = j; i < 64 & m < length ; i++,m++)
		{
			group_data[i] = read_cmos_sensor(0x0a04+i);
			data[m]=group_data[i];
			LOG_INF("ssmode_group_data= 0x%x,data= 0x%x\n",group_data[i],data[m]);
		}
		j = 0;
	}

	write_cmos_sensor(0x0ace, 0x00);
	write_cmos_sensor(0x0ae0, 0x00);
}
static kal_uint8 dpc_read_data[DPC_BUF_SIZE];

void gc16b3_otp_read_dpc_info(void)
{
#if	DPC_DEBUG
	kal_uint16 i = 0;
#endif
	kal_uint8 flag_dpc = 0;
	kal_uint16 total_number = 0;


	memset(&dpc_read_data, 0, DPC_BUF_SIZE * sizeof(kal_uint8));

	flag_dpc = gc16b3_otp_read_byte(DPC_FLAG_OFFSET);
	LOG_INF("flag_dpc = 0x%x\n", flag_dpc);


	switch (flag_dpc & 0x03) {
	case OTP_FLAG_EMPTY:
		LOG_INF("DPC is empty!\n");
		break;
	case OTP_FLAG_VALID:
		LOG_INF("DPC is valid!\n");
		total_number = gc16b3_otp_read_byte(DPC_TOTAL_NUMBER_OFFSET)
					+ gc16b3_otp_read_byte(DPC_ERROR_NUMBER_OFFSET);
		LOG_INF("total_number = %d\n", total_number);
		LOG_INF("read DPC_data start!\n");
		gc16b3_otp_read_group_ssmode(DPC_DATA_OFFSET, &dpc_read_data[2], 4 * total_number);
		LOG_INF("read DPC_data end!\n");
		break;
	case OTP_FLAG_INVALID:
	case OTP_FLAG_INVALID2:
		LOG_INF("DPC is invalid!\n");
		break;
	default:
		break;
	}

	dpc_read_data[0] = total_number & 0xff;
	dpc_read_data[1] = (total_number >> 8) & 0xff;
#if DPC_DEBUG
	for (i = 0; i < (4 * total_number + 2); i++)
		LOG_INF("dpc_data[%d] = 0x%x\n", i, dpc_read_data[i]);
#endif
}

void gc16b3_otp_copy_dpc_data(kal_uint8 *dpc_data)
{
	LOG_INF("start copy dpc_data!\n");
	memcpy(dpc_data, (void *)&dpc_read_data[0], DPC_BUF_SIZE);
}

static void gc16b3_otp_read(void)
{
	gc16b3_otp_read_dpc_info();
}

void gc16b3_otp_identify(void)
{
	LOG_INF("start hyper otp_identify");
	gc16b3_read_crosstalk_data();

	/* Read init */
	write_cmos_sensor(0x0316, 0x08);
	write_cmos_sensor(0x0a67, 0x80);
	write_cmos_sensor(0x0313, 0x00);
	write_cmos_sensor(0x0ace, 0x08);
	write_cmos_sensor(0x0a53, 0x04);
	write_cmos_sensor(0x0a65, 0x05);
	write_cmos_sensor(0x0a68, 0x11);
	write_cmos_sensor(0x0a58, 0x00);
	gc16b3_otp_read();
}

#define PFX "CAM_CAL"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__

#include <linux/kernel.h>
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/slab.h>
#include <linux/types.h>
#include "cam_cal.h"
#include "cam_cal_define.h"
#include "cam_cal_list.h"

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "eeprom_i2c_sc800cs_driver.h"
#ifdef ODM_HQ_EDIT
/*Houbing.Peng@ODM_HQ Cam.Drv 20200915 add for otp*/
#include <soc/oplus/system/oplus_project.h>
#endif

#define OTP_I2C_ADDR 0x20
#define EEPROM_I2C_MSG_SIZE_READ 2
#define SC800CS_OTP_RET_FAIL -1
#define SC800CS_OTP_RET_SUCCESS 0
#define SC800CS_OTP_PAGE2 2
#define SC800CS_TXD_FRONT_OTP_MODULE_LENS 17
#define SC800CS_TXD_FRONT_OTP_AWB_LENS 18
#define SC800CS_TXD_FRONT_OTP_SN_LENS 25
#define SC800CS_TXD_FRONT_OTP_LRC_LENS 6
#define SC800CS_GROUP1_FLAG 0x01
#define SC800CS_GROUP2_FLAG 0x13
//flag addr
#define SC800CS_OTP_GROUP_FLAGADDR 0x827A
#define SC800CS_OTP_MODULE_GROUP1_STARTADDR 0x827B
#define SC800CS_OTP_AWB5000K_GROUP1_STARTADDR 0x828C
#define SC800CS_OTP_AWB5000K_GROUP2_STARTADDR 0x8C9E
#define SC800CS_OTP_AWB2850K_GROUP1_STARTADDR 0x829E
#define SC800CS_OTP_AWB2850K_GROUP2_STARTADDR 0x8CB0
#define SC800CS_OTP_MODULE_GROUP2_STARTADDR 0x8C8D
#define SC800CS_OTP_LSC_GROUP1_CHECKSUMADDR 0x82D6
#define SC800CS_OTP_LSC_GROUP2_CHECKSUMADDR 0x8CE8
#define SC800CS_OTP_SN_GROUP1_STARTADDR 0x82BC
#define SC800CS_OTP_SN_GROUP2_STARTADDR 0x8CCE
#define SC800CS_OTP_LRC5000K_GROUP1_STARTADDR 0x82B0
#define SC800CS_OTP_LRC5000K_GROUP2_STARTADDR 0x8CC2
#define SC800CS_OTP_LRC2850K_GROUP1_STARTADDR 0x82B6
#define SC800CS_OTP_LRC2850K_GROUP2_STARTADDR 0x8CC8
//data start addr
#define MAX_EEPROM_BYTE 0x1FFF

static int GroupFlag = 0;
char sc800cs_otp_buf[MAX_EEPROM_BYTE] = {0};
#define READ_FLAG_ADDR 0x1FFD
#define CHECKSUM_FLAG_ADDR 0X1FFE

enum sc800cs_sensor_otp_page{
	page_0 = 0,
	page_1,
	page_2,
	page_3,
	page_4,
	page_5,
	page_6,
	page_7,
	page_8,
	page_9,
	page_10,
	page_11,
	page_12,
	page_max
};

static struct i2c_client *g_pstI2CclientG;
extern int iBurstWriteReg(u8 *pData, u32 bytes, u16 i2cId);

extern int iBurstWriteReg_multi(u8 *pData, u32 bytes, u16 i2cId,
	u16 transfer_length, u16 timing);

int sc800cs_txd_front_check_flag = 0;

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
		printk("I2C read failed!!\n");
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

	iReadRegI2C1(g_pstI2CclientG,pu_send_cmd, 2, (u8 *)&get_byte, 1, 0x20);

	return get_byte;
}

static void write_cmos_sensor8(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[4] = {(char)(addr >> 8),
		(char)(addr & 0xFF), (char)(para & 0xFF)};

	iWriteRegI2C(pu_send_cmd, 3);
}

static int sc800cs_checksum(u16 checkAddr, unsigned char *checkArray, int length)
{
	int checksum = read_cmos_sensor(checkAddr);
	int sum = 0, i;
	for (i = 0; i < length; i++) {
		sum += checkArray[i];
	}
	pr_info("sum = %d, sum %% 255 = 0x%02x, checksum = 0x%02x\n", sum, sum % 255, checksum);

	if (sum % 255 != checksum)
		return SC800CS_OTP_RET_FAIL;
	return SC800CS_OTP_RET_SUCCESS;
}

static void sc800cs_read(unsigned int start, unsigned int end, unsigned char *pinputdata)
{
	u16 addr = start;
	int i;
	for (i = 0; i < end - start + 1; i++) {
		pinputdata[i] = read_cmos_sensor(addr++);
		//printk("dbgmsg - lsc addr[%x], pinputdata[%d] = 0x%02x\n", addr, i, pinputdata[i]);
	}
}

static int sc800cs_set_threshold(u8 threshold) //set thereshold
{
	int threshold_reg1[3] = { 0x48, 0x48, 0x48 };
	int threshold_reg2[3] = { 0x38, 0x18, 0x58 };
	int threshold_reg3[3] = { 0x41, 0x41, 0x41 };

	if (threshold < 3 && threshold >= 0) {
		write_cmos_sensor8(0x36b0, threshold_reg1[threshold]);
		write_cmos_sensor8(0x36b1, threshold_reg2[threshold]);
		write_cmos_sensor8(0x36b2, threshold_reg3[threshold]);
		pr_info("sc800cs_otp set_threshold %d\n", threshold);
	} else {
		pr_err("sc800cs_otp set invalid threshold %d\n", threshold);

		return SC800CS_OTP_RET_FAIL;
	}

	return SC800CS_OTP_RET_SUCCESS;
}

static int sc800cs_set_page_and_load_data(int page) //set page
{
	uint64_t Startaddress = 0;
	uint64_t EndAddress = 0;
	int delay = 0;
	int pag = 0;

	Startaddress = page * 0x200 + 0x7E00; //set start address in page
	EndAddress = Startaddress + 0x1ff; //set end address in page
	pag = page * 2 - 1; //change page
	write_cmos_sensor8(0x4408, (Startaddress >> 8) & 0xff);
	write_cmos_sensor8(0x4409, Startaddress & 0xff);
	write_cmos_sensor8(0x440a, (EndAddress >> 8) & 0xff);
	write_cmos_sensor8(0x440b, EndAddress & 0xff);

	write_cmos_sensor8(0x4401, 0x13); // address set finished
	write_cmos_sensor8(0x4412, pag & 0xff); // set page
	write_cmos_sensor8(0x4407, 0x00); // set page finished
	write_cmos_sensor8(0x4400, 0x11); // manual load begin
	while ((read_cmos_sensor(0x4420) & 0x01) == 0x01) {
		delay++;
		pr_info("sc800cs_otp set_page waitting, OTP is still busy for loading %d times\n", delay);
		if (delay == 10) {
			pr_err("sc800cs_otp set_page fail, load timeout!!!\n");

			return SC800CS_OTP_RET_FAIL;
		}
		mdelay(10);
	}
	pr_info("sc800cs_otp set_page success\n");

	return SC800CS_OTP_RET_SUCCESS;
}

static int sc800cs_sensor_otp_read_data(unsigned int ui4_offset,
					unsigned int ui4_length, unsigned char *pinputdata)
{
	int sum = 0, i = 0;
	for (i = 0; i < ui4_length; i++) {
		pinputdata[i] = read_cmos_sensor(ui4_offset++);
        pr_info("sc800cs_sensor_otp_read_data %x\n",pinputdata[i]);
		if (i < ui4_length - 2)
			sum += pinputdata[i];
		printk("dbgmsg - func: %s, pinputdata[%d] = 0x%02x\n", __FUNCTION__, i, pinputdata[i]);
	}

	pr_info("sum = %x, sum %% 255 = %x, checksum = %x\n", sum, sum % 255, pinputdata[i - 1]);
	if (sum % 255 != pinputdata[i - 1])
		return -1;
	return 0;
}

static int sc800cs_sensor_otp_read_module_info(unsigned char *pinputdata)
{
	int ret = SC800CS_OTP_RET_FAIL;

	GroupFlag =
		read_cmos_sensor(SC800CS_OTP_GROUP_FLAGADDR);
	pr_info("sc800cs_otp Read GroupFlag addr :0x%x, data:0x%x\n",
			SC800CS_OTP_GROUP_FLAGADDR,
			GroupFlag);

	if (GroupFlag == SC800CS_GROUP1_FLAG) {
		ret = sc800cs_sensor_otp_read_data(SC800CS_OTP_MODULE_GROUP1_STARTADDR,
			SC800CS_TXD_FRONT_OTP_MODULE_LENS, pinputdata);
		pr_info("sc800cs_otp group1 ret = %d!\n", ret);
	} else if (GroupFlag == SC800CS_GROUP2_FLAG) {
		sc800cs_set_page_and_load_data(page_7);
		ret = sc800cs_sensor_otp_read_data(SC800CS_OTP_MODULE_GROUP2_STARTADDR,
			SC800CS_TXD_FRONT_OTP_MODULE_LENS, pinputdata);
		pr_info("sc800cs_otp group2 ret = %d!\n", ret);
	} else {
		pr_err("sc800cs_otp invalid flag :0x%x\n",
				GroupFlag);
	}

	return ret;
}

static int sc800cs_sensor_otp_read_awb_5000k_info(unsigned char *pinputdata)
{
	int ret = SC800CS_OTP_RET_FAIL;
	if (GroupFlag == SC800CS_GROUP1_FLAG) {
		ret = sc800cs_sensor_otp_read_data(SC800CS_OTP_AWB5000K_GROUP1_STARTADDR,
			SC800CS_TXD_FRONT_OTP_AWB_LENS, pinputdata);
		pr_info("sc800cs_otp group1 ret = %d!\n", ret);
	} else if(GroupFlag == SC800CS_GROUP2_FLAG) {
        sc800cs_set_page_and_load_data(page_7);
		ret = sc800cs_sensor_otp_read_data(SC800CS_OTP_AWB5000K_GROUP2_STARTADDR,
			SC800CS_TXD_FRONT_OTP_AWB_LENS, pinputdata);
		pr_info("sc800cs_otp group2 ret = %d!\n", ret);
	} else
		pr_err("sc800cs_otp invalid flag = 0x%x!\n", GroupFlag);

	return ret;
}

static int sc800cs_sensor_otp_read_awb_2850k_info(unsigned char *pinputdata)
{
	int ret = SC800CS_OTP_RET_FAIL;
	if(GroupFlag == SC800CS_GROUP1_FLAG){
		ret = sc800cs_sensor_otp_read_data(SC800CS_OTP_AWB2850K_GROUP1_STARTADDR,
			SC800CS_TXD_FRONT_OTP_AWB_LENS, pinputdata);
		pr_info("sc800cs_otp group1 ret = %d!\n", ret);
	} else if(GroupFlag == SC800CS_GROUP2_FLAG) {
        sc800cs_set_page_and_load_data(page_7);
		ret = sc800cs_sensor_otp_read_data(SC800CS_OTP_AWB2850K_GROUP2_STARTADDR,
			SC800CS_TXD_FRONT_OTP_AWB_LENS, pinputdata);
		pr_info("sc800cs_otp group2 ret = %d!\n", ret);
	} else
		pr_err("sc800cs_otp invalid flag = 0x%x!\n", GroupFlag);

	return ret;
}

static int sc800cs_sensor_otp_read_lsc_info(unsigned char *pinputdata)
{
	int ret = SC800CS_OTP_RET_FAIL;
	if (GroupFlag == SC800CS_GROUP1_FLAG) {
		// lsc 1:
		sc800cs_set_page_and_load_data(page_2);
		sc800cs_read(0x82D6, 0x83FF, pinputdata);
		// lsc 2:
		sc800cs_set_page_and_load_data(page_3);
		sc800cs_read(0x847A, 0x85FF, &pinputdata[298]);
		// lsc 3:
		sc800cs_set_page_and_load_data(page_4);
		sc800cs_read(0x867A, 0x87FF, &pinputdata[688]);
		// lsc 4:
		sc800cs_set_page_and_load_data(page_5);
		sc800cs_read(0x887A, 0x89FF, &pinputdata[1078]);
		// lsc 5:
		sc800cs_set_page_and_load_data(page_6);
		sc800cs_read(0x8A7A, 0x8BFF, &pinputdata[1468]);
		// lsc 6:
		sc800cs_set_page_and_load_data(page_7);
		sc800cs_read(0x8C7A, 0x8C83, &pinputdata[1858]);
		// checksum
		ret = sc800cs_checksum(0x8C85, &pinputdata[0], 1868);
	} else if (GroupFlag == SC800CS_GROUP2_FLAG) {
		sc800cs_set_page_and_load_data(page_7);
		sc800cs_read(0x8CE8, 0x8DFF, pinputdata);
		// lsc 2:
		sc800cs_set_page_and_load_data(page_8);
		sc800cs_read(0x8E7A, 0x8FFF, &pinputdata[280]);
		// lsc 3:
		sc800cs_set_page_and_load_data(page_9);
		sc800cs_read(0x907A, 0x91FF, &pinputdata[670]);
		// lsc 4:
		sc800cs_set_page_and_load_data(page_10);
		sc800cs_read(0x927A, 0x93FF, &pinputdata[1060]);
		// lsc 5:
		sc800cs_set_page_and_load_data(page_11);
		sc800cs_read(0x947A, 0x95FF, &pinputdata[1450]);
		// lsc 6:
		sc800cs_set_page_and_load_data(page_12);
		sc800cs_read(0x967A, 0x9695, &pinputdata[1840]);
		// checksum
		ret = sc800cs_checksum(0x9697, &pinputdata[0], 1868);
	} else {
		pr_info("dbgmsg - error");
		// err
	}
	return ret;
}

static int sc800cs_sensor_otp_read_sn_info(unsigned char *pinputdata)
{
	int ret = SC800CS_OTP_RET_FAIL;
	pr_info("sc800cs_otp Read ModuleFlag addr :0x%x, data:0x%x\n",
			SC800CS_OTP_GROUP_FLAGADDR, GroupFlag);
	if (GroupFlag == SC800CS_GROUP1_FLAG) {
		ret = sc800cs_sensor_otp_read_data(SC800CS_OTP_SN_GROUP1_STARTADDR,
			SC800CS_TXD_FRONT_OTP_SN_LENS, pinputdata);
		pr_info("sc800cs_otp group1 ret = %d!\n", ret);
	} else if (GroupFlag == SC800CS_GROUP2_FLAG) {
		sc800cs_set_page_and_load_data(page_7);
		ret = sc800cs_sensor_otp_read_data(SC800CS_OTP_SN_GROUP2_STARTADDR,
			SC800CS_TXD_FRONT_OTP_SN_LENS, pinputdata);
		pr_info("sc800cs_otp group2 ret = %d!\n", ret);
	} else
		pr_err("sc800cs_otp sn_info invalid flag \n");

	return ret;
}

static int sc800cs_sensor_otp_read_lrc_5000k_info(unsigned char *pinputdata)
{
	int ret = SC800CS_OTP_RET_FAIL;

	if(GroupFlag == SC800CS_GROUP1_FLAG){
		ret = sc800cs_sensor_otp_read_data(SC800CS_OTP_LRC5000K_GROUP1_STARTADDR,
			SC800CS_TXD_FRONT_OTP_LRC_LENS, pinputdata);
		pr_info("sc800cs_otp group1 ret = %d!\n", ret);
	} else if(GroupFlag == SC800CS_GROUP2_FLAG) {
		sc800cs_set_page_and_load_data(page_7);
		ret = sc800cs_sensor_otp_read_data(SC800CS_OTP_LRC5000K_GROUP2_STARTADDR,
			SC800CS_TXD_FRONT_OTP_LRC_LENS, pinputdata);
		pr_info("sc800cs_otp group2 ret = %d!\n", ret);
	} else
		pr_err("sc800cs_otp invalid!\n");

	return ret;
}

static int sc800cs_sensor_otp_read_lrc_2850k_info(unsigned char *pinputdata)
{
	int ret = SC800CS_OTP_RET_FAIL;

	if(GroupFlag == SC800CS_GROUP1_FLAG){
		ret = sc800cs_sensor_otp_read_data(SC800CS_OTP_LRC2850K_GROUP1_STARTADDR,
			SC800CS_TXD_FRONT_OTP_LRC_LENS, pinputdata);
		pr_info("sc800cs_otp group1 ret = %d!\n", ret);
	} else if(GroupFlag == SC800CS_GROUP2_FLAG) {
		sc800cs_set_page_and_load_data(page_7);
		ret = sc800cs_sensor_otp_read_data(SC800CS_OTP_LRC2850K_GROUP2_STARTADDR,
			SC800CS_TXD_FRONT_OTP_LRC_LENS, pinputdata);
		pr_info("sc800cs_otp group2 ret = %d!\n", ret);
	} else
		pr_err("sc800cs_otp invalid!\n");

	return ret;
}

static int sc800cs_sensor_otp_read_by_group()
{
	int threshold = 0;
	int ret = SC800CS_OTP_RET_FAIL;

	for (threshold = 0; threshold < 3; threshold++) {
		sc800cs_set_threshold(threshold);
		sc800cs_set_page_and_load_data(SC800CS_OTP_PAGE2);
		ret = sc800cs_sensor_otp_read_module_info(sc800cs_otp_buf);
		if (ret == SC800CS_OTP_RET_FAIL) {
			pr_err("sc800cs_otp read module info in threshold R%d fail\n", threshold);
			continue;
		}

		ret = sc800cs_sensor_otp_read_awb_5000k_info(&sc800cs_otp_buf[17]);
		if (ret == SC800CS_OTP_RET_FAIL) {
			pr_err("sc800cs_otp read awb info  in threshold R%d fail\n", threshold);
			continue;
		}

		ret = sc800cs_sensor_otp_read_awb_2850k_info(&sc800cs_otp_buf[35]);
		if (ret == SC800CS_OTP_RET_FAIL) {
			pr_err("sc800cs_otp read awb info  in threshold R%d fail\n", threshold);
			continue;
		}

		ret = sc800cs_sensor_otp_read_lrc_5000k_info(&sc800cs_otp_buf[53]);
		if (ret == SC800CS_OTP_RET_FAIL) {
			pr_err("sc800cs_otp read light source in threshold R%d fail\n", threshold);
			continue;
		}

		ret = sc800cs_sensor_otp_read_lrc_2850k_info(&sc800cs_otp_buf[59]);
		if (ret == SC800CS_OTP_RET_FAIL) {
			pr_err("sc800cs_otp read light source in threshold R%d fail\n", threshold);
			continue;
		}

		ret = sc800cs_sensor_otp_read_sn_info(&sc800cs_otp_buf[65]);
		if (ret == SC800CS_OTP_RET_FAIL) {
			pr_err("sc800cs_otp read sn_data in threshold R%d fail\n", threshold);
			continue;
		}

		ret = sc800cs_sensor_otp_read_lsc_info(&sc800cs_otp_buf[90]);
		if (ret == SC800CS_OTP_RET_FAIL) {
			pr_err("sc800cs_otp read lsc info	in threshold R%d fail\n", threshold);
			continue;
		}

		pr_info("sc800cs_otp read all otp data in threshold R%d success\n", threshold);

		break;
	}
	if (ret == SC800CS_OTP_RET_FAIL) {
		pr_err("sc800cs_otp read otp data  in threshold R1 R2 R3 all failed!\n");

		return ret;
	}

	return ret;
}

static int sc800cs_sensor_otp_read_all_data()
{
	int ret = SC800CS_OTP_RET_FAIL;
	int delay = 0;

	ret = sc800cs_sensor_otp_read_by_group();
	if (ret == SC800CS_OTP_RET_FAIL) {
		pr_err("sc800cs_otp read lsc info in threshold R1 R2 R3 all failed!!!\n");

		return ret;
	}
	pr_info("sc800cs_otp read otp data success\n");

	write_cmos_sensor8(0x4408, 0x80);
	write_cmos_sensor8(0x4409, 0x00);
	write_cmos_sensor8(0x440a, 0x81);
	write_cmos_sensor8(0x440b, 0xff);

	write_cmos_sensor8(0x4401, 0x13);
	write_cmos_sensor8(0x4412, 0x1);
	write_cmos_sensor8(0x4407, 0x0e);
	write_cmos_sensor8(0x4400, 0x11);

	while ((read_cmos_sensor(0x4420) & 0x01) == 0x01) {
		delay++;
		pr_info("sc800cs_otp read otp is waitting, OTP is still busy for loading %d times\n", delay);
		if (delay == 10) {
			pr_err("sc800cs_otp read otp data fail, load timeout!\n");

			return SC800CS_OTP_RET_FAIL;
		}
		mdelay(10);
	}
	sc800cs_txd_front_check_flag = 1;
	return ret;
}


unsigned int sc800cs_read_region(struct i2c_client *client, unsigned int addr, unsigned char *data, unsigned int size)
{
	int ret = 0, i;
	g_pstI2CclientG = client;
	if (sc800cs_otp_buf[READ_FLAG_ADDR]) {
		pr_info("read otp data from sc800cs_otp_buf: addr 0x%x, size %d", addr, size);
		memcpy((void *)data, &sc800cs_otp_buf[addr], size);
		return size;
	}

    ret = sc800cs_sensor_otp_read_all_data();
	if (!ret) {
		sc800cs_otp_buf[READ_FLAG_ADDR] = 1;
	}

	if (NULL != data) {
		memcpy((void *)data, &sc800cs_otp_buf[addr], size);
	}
	for (i = 0; i < size; i = i + 4){
		pr_debug("sc800cs common cam data : %02x %02x %02x %02x\n",
			sc800cs_otp_buf[i],
			sc800cs_otp_buf[i + 1],
			sc800cs_otp_buf[i + 2],
			sc800cs_otp_buf[i + 3]);
	}
	return ret;
}


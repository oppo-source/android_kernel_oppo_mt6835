/*****************************************************************************
 *
 * Filename:
 * ---------
 *     GC16b3mipi_otp.h
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     CMOS sensor header file
 *
 ****************************************************************************/
#ifndef _GC16B3MIPI_OTP_H
#define _GC16B3MIPI_OTP_H

#include "kd_camera_typedef.h"

#define OTP_WRITE_ID              0x76 /*0x20*/
#define EEPROM_WRITE_ID           0xA2

#define CROSSTALK_BUF_SIZE        775 /*770+2+2*/
#define CROSSTALK_FLAG_OFFSET     0x1325
#define CROSSTALK_CHECKSUM_OFFSET     0x1326
#define CROSSTALK_START_ADDR      0x1020

/* OTP FLAG TYPE */
#define OTP_FLAG_EMPTY            0x00
#define OTP_FLAG_VALID            0x01
#define OTP_FLAG_INVALID          0x02
#define OTP_FLAG_INVALID2         0x03

#define DPC_FLAG_OFFSET           0x00e0
#define DPC_TOTAL_NUMBER_OFFSET   0x00e8
#define DPC_ERROR_NUMBER_OFFSET   0x00f0
#define DPC_DATA_OFFSET           0x00f8
#define DPC_BUF_SIZE              882


extern int iReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u16 i2cId);
extern int iWriteReg(u16 a_u2Addr, u32 a_u4Data, u32 a_u4Bytes, u16 i2cId);
void gc16b3_otp_identify(void);
void gc16b3_otp_copy_dpc_data(kal_uint8 *dpc_data);
void gc16b3_copy_crosstalk_data(kal_uint8 *ct_data);
#endif

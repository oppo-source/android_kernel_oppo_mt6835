/*****************************************************************************
 *
 * Filename:
 * ---------
 *     gc50f6mipi_otp_mumbai.h
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
#ifndef _GC50F6MIPI_OTP_MUMBAI_H
#define _GC50F6MIPI_OTP_MUMBAI_H

#define EEPROM_WRITE_ID             0xA0

#define GC_SXTC_DATA_SIZE           784
#define FLAG_AND_CHECK_SUM_OF_SXTC_SIZE     2
#define GC_PDXTC_DATA_SIZE          491
#define SUNNY_RESERVED_SIZE         14

#define CROSSTALK_BUF_SIZE        GC_PDXTC_DATA_SIZE
#define CROSSTALK_FLAG_OFFSET       0x17EA
#define CROSSTALK_START_ADDR        0x1600


/* OTP FLAG TYPE */
#define OTP_FLAG_EMPTY              0x00
#define OTP_FLAG_VALID              0x01
#define OTP_FLAG_INVALID            0x02
#define OTP_FLAG_INVALID2           0x03

extern int iReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u16 i2cId);
extern int iWriteReg(u16 a_u2Addr, u32 a_u4Data, u32 a_u4Bytes, u16 i2cId);
#endif

/****************************************Copyright (c)****************************************************
**                               Guangzhou ZHIYUAN electronics Co.,LTD.
**                                     
**                                 http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:           Mouse.Drive.h
** Last modified Date:  
** Last Version: 
** Description:         底层驱动程序头文件
** 
**--------------------------------------------------------------------------------------------------------
** Created By: 
** Created date: 
** Version: 
** Descriptions: 
**
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
** Version:
** Description:
**
*********************************************************************************************************/


#ifndef __iic_h
#define __iic_h


/*********************************************************************************************************
  包含头文件
*********************************************************************************************************/
#include "hw_memmap.h"
#include "hw_ints.h"
#include "hw_types.h"
#include "sysctl.h"
#include "Systick.h"
#include "Type.h"
#include "Micromouse.h"
#include "Mouse_Config.h"

#define __SCL                 GPIO_PIN_2                                  /*  IIC时钟                     */
#define __SDA                 GPIO_PIN_3                                  /*  IIC数据                     */


// 定义MPU6050内部地址
//****************************************
#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIG			0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_XOUT_H	0x3B	//0
#define	ACCEL_XOUT_L	0x3C	//1
#define	ACCEL_YOUT_H	0x3D	//2
#define	ACCEL_YOUT_L	0x3E	//3     
#define	ACCEL_ZOUT_H	0x3F	//4
#define	ACCEL_ZOUT_L	0x40	//5
#define	TEMP_OUT_H		0x41	//6
#define	TEMP_OUT_L		0x42	//7
#define	GYRO_XOUT_H		0x43	//8
#define	GYRO_XOUT_L		0x44	//9
#define	GYRO_YOUT_H		0x45	//10
#define	GYRO_YOUT_L		0x46	//11
#define	GYRO_ZOUT_H		0x47  //12
#define	GYRO_ZOUT_L		0x48	//13

#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I		0x75	//IIC地址寄存器(默认数值0x68，只读)
#define	SlaveAddress	0xD0	//IIC写入时的地址字节数据，+1为读取

/*********************************************************************************************************
  常量宏定义
*********************************************************************************************************/
void SCL_IN (void);
void SCL_OUT (void);
void SDA_IN (void);
void SDA_OUT (void);
void SCL_0 (void);
void SCL_1 (void);
void SDA_0 (void);
void SDA_1 (void);
void Delay (uint32 Time);
void I2Cstart(void);
void I2Cstop(void);
void I2CACK(void);
void I2CNACK(void);
uint32 I2CCACK(void);
void send_fm24LC16(uint32 data);
uint32 write_fm24LC16(uint32 *array,uint32 address,uint32 control,uint32 len);
uint32 receive_fm24LC16(void);
uint32 read_fm24LC16(uint32 *array,uint32 address,uint32 control,uint32 len);
uint16 GetData(uint8 REG_Address);
void MPU6050_Init(void);
#endif
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/

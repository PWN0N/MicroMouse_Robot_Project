#include "hw_memmap.h"
#include "hw_ints.h"
#include "hw_types.h"
#include "interrupt.h"
#include "gpio.h"
#include "sysctl.h"
#include "Systick.h"
#include "Type.h"
#include "iic.h"
#include "Micromouse.h"
#include "Mouse_Config.h"

/*********************************************************************************************************
** Function name:       Delay
** Descriptions:        软件延时
** input parameters:    ulTime
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void SCL_IN (void)
{
	GPIODirModeSet(GPIO_PORTB_BASE, __SCL, GPIO_DIR_MODE_IN);	   
}
void SCL_OUT (void)
{
	GPIODirModeSet(GPIO_PORTB_BASE, __SCL, GPIO_DIR_MODE_OUT);	 
}
void SDA_IN (void)
{
	GPIODirModeSet(GPIO_PORTB_BASE, __SDA, GPIO_DIR_MODE_IN);		  
}
void SDA_OUT (void)
{
	GPIODirModeSet(GPIO_PORTB_BASE, __SDA, GPIO_DIR_MODE_OUT);	 
}

void SCL_0 (void)
{
	GPIOPinWrite(GPIO_PORTB_BASE, __SCL,0x00);   
}
void SCL_1 (void)
{
	GPIOPinWrite(GPIO_PORTB_BASE, __SCL,0xff); 
}
void SDA_0 (void)
{
	GPIOPinWrite(GPIO_PORTB_BASE, __SDA,0x00);  
}
void SDA_1 (void)
{
	GPIOPinWrite(GPIO_PORTB_BASE, __SDA,0xff); 
}
void Delay (uint32 Time)						   // 1us
{
    uint32 i;
   
    while (Time--) {
        for (i = 0; i < 5; i++);
    }
}

void __Delay (uint32 Time)						   // 1us
{
    uint32 i;
   
    while (Time--) {
        for (i = 0; i < 1; i++);
    }
}

void I2Cstart(void)
{ 
	SDA_OUT();
	SCL_OUT();  
	SCL_0();
	SDA_1();
  	Delay(5);//5us
	SCL_1();
  	Delay(5);
	SDA_0();
	Delay(5);//5us
  	SCL_0();
	Delay(5);
 
 }
void I2Cstop(void)
{ 
	SDA_OUT();
	SCL_OUT();
	SDA_0();
	SCL_0();
  	Delay(5);
  	SCL_1();
  	Delay(5);
  	SDA_1();
  	Delay(5);
	SCL_0();
	Delay(5);
}

void I2CACK(void)
{ 	
	//SCL_OUT();
  	SDA_OUT();
  	SDA_0();
  	SCL_0();
	Delay(5);
	SCL_1();
  	Delay(5);
  	SCL_0();
	Delay(5);
  	//Delay(1);
  	//SDA_1();
	//SCL_0();
	//Delay(1);
	//SCL_1();
}
void I2CNACK(void)
{
	//SCL_OUT();
 	SDA_OUT();
	Delay(5);
  	SDA_1();
  	SCL_0();
	Delay(5);
	SCL_1();
  	Delay(5);
  	SCL_0();
	Delay(5);
 }

//检查应答位子程序
uint32 I2CCACK(void)
{ 
	SDA_IN();
	SCL_0();
	Delay(5);
  	SCL_1();
  	//Delay(5);
	//SCL_0();
  	return(GPIOPinRead(GPIO_PORTB_BASE, __SDA));

}

//写I2C总线
void send_fm24LC16(uint32 data)
{ 
	uint32 flag;
	uint32 sz;
  	SDA_OUT();
	//SCL_OUT();
	SDA_0();
	SCL_0();
	Delay(5);
  	for(flag=0x0080;flag!=0x00;flag=flag/2)
  	{  
		//SCL_1();
     	//Delay(5);
   		sz = data & flag;
 		if(sz==0){ SDA_0();}
    	else  { SDA_1();}
		Delay(5);
		SCL_1();
		Delay(5);
        SCL_0();
		//Delay(5);
		SDA_0();
		//Delay(5);  
  	}
	//SDA_IN();
}


//写数据子程序
uint32 write_fm24LC16(uint32 *array,uint32 address,uint32 control,uint32 len)
{ 
	uint32 dat;
  	I2Cstart();
	dat=control;
  	send_fm24LC16(dat);
  	dat=I2CCACK();
  	if(dat==0)
  	{
		dat = address;
   		send_fm24LC16(dat);
   		dat = I2CCACK();
   		if(dat==0)
    	        { 
		      for(;len!=0;len--,array++)
      		      {
			send_fm24LC16(*array);
       			dat = I2CCACK();
       			if(dat==1)
                          break;
      		      }
    
 		}
  	}
    I2Cstop();
    return(dat);
} 

//读I2C总线
uint32 receive_fm24LC16(void)
{ 
	uint32 sz=0;
	uint32 i;
 	// SCLOUT();
   	SDA_IN();
	//Delay(5);
   	SCL_0();
   	Delay(5);
  	for(i=0;i<8;i++)
   	{  
		//SCL_0();
		if(GPIOPinRead(GPIO_PORTB_BASE, __SDA))
      	        {sz|=0x01;}
		Delay(3);
		SCL_1();
      	Delay(5);
		SCL_0();
	  	if(i<7)
	  	{sz=sz<<1;}
		Delay(5);
   	}
	//SCL_0();
	//Delay(5);
   	return(sz);
  }

//读数据子程序
uint32 read_fm24LC16(uint32 *array,uint32 address,uint32 control,uint32 len)
{ 
	uint32 dat;
  	I2Cstart();
  	dat=(control & 0xfe);
	send_fm24LC16(dat);
	SDA_0();
	SCL_0();
	Delay(5);
  	dat=I2CCACK();
  	if(dat==0)
  	{  
	dat=address;
     	send_fm24LC16(dat);
     	dat=I2CCACK();
     	if(dat==0)
     	{    
	I2Cstart();
	dat=control;
        send_fm24LC16(dat); 
        dat=I2CCACK();
        if(dat==0)
         {  
	for(;len!=0;len--,array++)
          { 
	dat=receive_fm24LC16();
         //Delay(5);
	I2CACK();
         *array=dat;
	//Delay(500);
      	}
     //dat=receive_fm24LC16();
      	//*array=dat;
       	I2CNACK();
        I2Cstop();
         dat=0;
            }
       	}
    }
      
   	return(dat);
}

void IIC_Start(void)
{ 
	SDA_OUT();
	SCL_OUT();  
	SCL_0();
	SDA_1();
  	__Delay (1);
	SCL_1();
  	__Delay (1);
	SDA_0();
	__Delay (1);
  	SCL_0();
	__Delay (1);
}

void IIC_Stop(void)
{ 
	SDA_OUT();
	SDA_0();
	SCL_0();
  	__Delay (1);
  	SCL_1();
  	__Delay (1);
  	SDA_1();
  	__Delay (1);
	SCL_0();
	__Delay (1);
}

//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8 IIC_Wait_Ack(void)
{
    uint8 ucErrTime=0;
    SDA_IN();      //SDA设置为输入  
    SDA_1();__Delay (1);	   
    SCL_1();__Delay (1); 
    while(GPIOPinRead(GPIO_PORTB_BASE, __SDA))
    {
        ucErrTime++;
	if(ucErrTime>250)
	{
	    IIC_Stop();
	    return 1;
	}
    }
    SCL_0();//时钟输出0 	   
    return 0;  
}

void IIC_Ack(void)
{ 	
  	SDA_OUT();
  	SDA_0();
  	SCL_0();
	__Delay (1); 
	SCL_1();
  	__Delay (1); 
  	SCL_0();
	__Delay (1); 
}
//不产生ACK应答		    
void IIC_NAck(void)
{
 	SDA_OUT();
	__Delay (1); 
  	SDA_1();
  	SCL_0();
	__Delay (1); 
	SCL_1();
  	__Delay (1); 
  	SCL_0();
	__Delay (1); 
}

//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(uint8 txd)
{                        
    uint8 t;   
   	SDA_OUT(); 	    
    SCL_0();//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {   
      if((txd&0x80)>>7)
      {
          SDA_1();
      }
      else
      {
          SDA_0();
      }
        //IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
        __Delay (1);   
        SCL_1();
        __Delay (1);
        SCL_0();	
        __Delay (1);
    }	 
}

//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
uint8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
        for(i=0;i<8;i++ )
	{
        SCL_0(); 
        __Delay (1);
        SCL_1();
        receive<<=1;
        if(GPIOPinRead(GPIO_PORTB_BASE, __SDA))receive++;   
	__Delay (1); 
  }					 
    if (!ack)
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}

uint8 MPU6050_ReadI2C(uint8 REG_Address)
{
	uint8 REG_data;
	IIC_Start();                  //起始信号
	IIC_Send_Byte(SlaveAddress);  //发送设备地址+写信号
	REG_data=IIC_Wait_Ack();	   
	IIC_Send_Byte(REG_Address);   //发送存储单元地址，从0开始
	REG_data=IIC_Wait_Ack();	   
	IIC_Start();                  //起始信号
	IIC_Send_Byte(SlaveAddress+1);//发送设备地址+读信号
	REG_data=IIC_Wait_Ack();	   
        REG_data=IIC_Read_Byte(0);		//读取一个字节,不继续再读,发送NAK,读出寄存器数据
	IIC_Stop();	                  //停止信号
	return REG_data;
}

void MPU6050_WriteI2C(uint8 REG_Address,uint8 REG_data)
{
    IIC_Start();                   //起始信号
    IIC_Send_Byte(SlaveAddress);   //发送设备地址+写信号
		IIC_Wait_Ack();	   
    IIC_Send_Byte(REG_Address);    //内部寄存器地址
		IIC_Wait_Ack(); 	 										  		   
    IIC_Send_Byte(REG_data);       //内部寄存器数据
		IIC_Wait_Ack(); 	 										  		   
    IIC_Stop();                    //发送停止信号
}

void MPU6050_Init(void)
{
    uint8 Add;//器件地址
    MPU6050_WriteI2C(PWR_MGMT_1, 0x00);	//解除休眠状态
    MPU6050_WriteI2C(SMPLRT_DIV, 0x07);
    MPU6050_WriteI2C(CONFIG, 0x06);
    MPU6050_WriteI2C(GYRO_CONFIG, 0x18);
}

uint16 GetData(uint8 REG_Address)
{
    uint8 H,L;
    H=MPU6050_ReadI2C(REG_Address);
    L=MPU6050_ReadI2C(REG_Address+1);
    return (H<<8)+L; 
}

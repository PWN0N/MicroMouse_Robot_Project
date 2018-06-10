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


#ifndef __Mouse_Drive_h
#define __Mouse_Drive_h


/*********************************************************************************************************
  包含头文件
*********************************************************************************************************/
#include "hw_memmap.h"
#include "hw_ints.h"
#include "hw_types.h"
#include "hw_uart.h"
#include "interrupt.h"
#include "gpio.h"
#include "sysctl.h"
#include "Systick.h"
#include "Timer.h"
#include "Pwm.h"
#include "adc.h"
#include "iic.h"
#include "Zlg7289.h"
#include "Type.h"
#include "Micromouse.h"
#include "Mouse_Config.h"
#include "uart.h"

/*********************************************************************************************************
  PA端口定义
*********************************************************************************************************/                                
#define __UART0RX             GPIO_PIN_0
#define __UART0TX             GPIO_PIN_1
/*********************************************************************************************************
  PB端口定义
*********************************************************************************************************/
#define __IRSEND1_FRONT       GPIO_PIN_0                                  /*  驱动前方红外发射      */
#define __IRSEND3_LR          GPIO_PIN_1                                  /*  驱动左右红外的调制信号      */
#define __FRONTSIDE_L         GPIO_PIN_4                                  /*  45左传感器输出的信号      */
#define __FRONTSIDE_R         GPIO_PIN_5                                  /*  45右传感器输出的信号      */
#define __LED                 GPIO_PIN_6
/*********************************************************************************************************
  PC端口定义
*********************************************************************************************************/
#define __KEY                 GPIO_PIN_5                                  /*  按键连接的端口              */
#define __IRSEND2_BEVEL       GPIO_PIN_6                                  /*  驱动斜角红外的调制信号      */
#define __START               GPIO_PIN_7                                  /*  按键连接的端口              */

/*********************************************************************************************************
  PD端口定义
*********************************************************************************************************/
#define __D1_1                GPIO_PIN_0                                  /*  PWM0 电机 M1 PWMH      */
#define __D1_2                GPIO_PIN_1                                  /*  PWM1 电机 M1 PWML      */
#define __FRONT_R             GPIO_PIN_2                                  /*  前方右传感器输出的信号        */
#define __RIGHTSIDE           GPIO_PIN_3                                  /*  右方传感器输出的信号        */
#define __SIG_M2_B            GPIO_PIN_4                                  /*  编码器 电机 M2 B相      */
#define __DIR2                GPIO_PIN_5                                  /*  编码器 电机 M2 A相      */
#define __LEFTSIDE            GPIO_PIN_6                                  /*  左方传感器输出的信号        */
#define __FRONT_L             GPIO_PIN_7                                  /*  前方左传感器输出的信号        */
/*********************************************************************************************************
  PE端口定义
*********************************************************************************************************/
#define __D2_1                GPIO_PIN_0                                  /*  PWM4 电机 M2 PWMH      */
#define __D2_2                GPIO_PIN_1                                  /*  PWM5 电机 M2 PWML      */
#define __DIR1                GPIO_PIN_2                                  /*  编码器 电机 M1 A相      */
#define __SIG_M1_B            GPIO_PIN_3                                  /*  编码器 电机 M1 B相      */


/*********************************************************************************************************
  常量宏定义--传感器
*********************************************************************************************************/
#define __LEFT              0                                           /*  左方传感器                  */
#define __FRONTL            1                                           /*  左前方传感器                */
#define __FRONT             2                                           /*  前方传感器                  */
#define __FRONTR            3                                           /*  右前方传感器                */
#define __RIGHT             4                                           /*  右方传感器                  */


/*********************************************************************************************************
  常量宏定义--电脑鼠状态
*********************************************************************************************************/
#define __STOP              0                                           /*  电脑鼠停止                  */
#define __GOAHEAD           1                                           /*  电脑鼠前进                  */
#define __TURNLEFT          3                                           /*  电脑鼠向左转                */
#define __TURNRIGHT         4                                           /*  电脑鼠向右转                */
#define __TURNBACK          5                                           /*  电脑鼠向后转                */
#define __TURNLEFTY         7                                           /*  电脑鼠向左转                */
#define __TURNRIGHTY        8                                           /*  电脑鼠向右转                */

/*********************************************************************************************************
  常量宏定义--电机加减速度
*********************************************************************************************************/
#define __SPEEDUP         0                                           /*  电机加速                    */
#define __SPEEDDOWN       1                                           /*  电机减速                */

/*********************************************************************************************************
  常量宏定义--电机状态
*********************************************************************************************************/
#define __MOTORSTOP         0                                           /*  电机停止                    */
#define __WAITONESTEP       1                                           /*  电机暂停一步                */
#define __MOTORRUN          2                                           /*  电机运行                    */
#define __CORRECT           6                                           /*  电机向后矫正                    */

/*********************************************************************************************************
  常量宏定义--电机运行方向
*********************************************************************************************************/
#define __MOTORGOAHEAD      0                                           /*  电机前进                    */
#define __MOTORGOBACK       1                                           /*  电机后退                    */
#define __MOTORGOSTOP       2                                           /*  电机反向制动                */

/*********************************************************************************************************
  常量宏定义--PID
*********************************************************************************************************/
#define __KP 40     //比例 30
#define __KI 0.01    //积分 0.01
#define __KD 0            //微分

#define U_MAX 1000       //返回的最大值,是pwm的周期值 
#define U_MIN 5 
#define error_IMAX 10     //积分限幅 
#define Deadband 1   //速度PID，设置死区范围

/*********************************************************************************************************
  结构体定义
*********************************************************************************************************/
struct __motor {
    int8    cState;                                                     /*  电机运行状态                */
    int8    cDir;                                                       /*  电机运行方向                */
    int8    cRealDir;                                                   /*  电机运行方向                */
    uint32  uiPulse;                                                    /*  电机需要运行的脉冲          */
    uint32  uiPulseCtr;                                                 /*  电机已运行的脉冲            */
    int16   sSpeed;                                                    /*  当前占空比                    */
};
typedef struct __motor __MOTOR;

struct __pid       //定义数法核心数据 
{ 
    //uint16 usRef;      //速度PID，速度设定值 
    uint16 usFeedBack;  //速度PID，速度反馈值
    uint16 usEncoder_new; //编码器
    uint16 usEncoder_last; //编码器
    
    int16 sRef;
    int16 sFeedBack;
    int16 sPreError;  //速度PID，前一次，速度误差,,vi_Ref - vi_FeedBack 
    int16 sPreDerror; //速度PID，前一次，速度误差之差，d_error-PreDerror; 
  
    fp32 fKp;      //速度PID，Ka = Kp 
    fp32 fKi;      //速度PID，Kb = Kp * ( T / Ti ) 
    fp32 fKd;      //速度PID， 
       
    int16 iPreU;    //电机控制输出值      
};
typedef struct __pid __PID;

extern uint8 flag_bug1;
/*********************************************************************************************************
  常量宏定义
*********************************************************************************************************/

uint8 keyCheck(void);
void mouseInit(void);
void mouseGoahead(int8  cNBlock);                                       /*  前进N格                     */
void mazeSearch(void);                                                  /*  前进N格                     */
void mouseTurnleft(void);                                               /*  向左转90度                  */
void mouseTurnright(void);                                              /*  向右转90度                  */
void mouseTurnback(void);                                               /*  向后转                      */
void sensorDebug(void);                                                 /*  传感器调试                  */
void voltageDetect(void);
void voltageDetectRef(void);
void mouseGoaheadhui(int8  cNBlock);                                       /*  前进N格                     */
void mouseGoahead_Llow (int8  cNBlock);
void __UART0Init (void);
void SendChar(uint8 dat);
void SendStr(uint8 *s);
void mouseTurnback_Y(void);

static void __delay(uint32  uiD);
static void __rightMotorContr(void);
static void __leftMotorContr(void);
static void __mouseCoorUpdate(void);
static void __mazeInfDebug(void);
static void __irSendFreq(uint32  __uiFreq, int8  __cNumber);
static void __irCheck(void);
static void __wallCheck(void);                                          /*  墙壁检测                    */
static void __sensorInit(void);
static void __MotorIint(void);
static void __keyInit(void);
static void __sysTickInit(void);


void mouseTurnleft_L(void);                                               /*  向左转90度                  */
void mouseTurnright_L(void);                                              /*  向右转90度                  */
void mouseGoahead_L(int8  cNBlock);                                       /*  前进N格                     */
void mouseStop(void); 
void mouseGo(void);
void __keyIntDisable (void);
void mouseTurnback_C(void);                                               /*  向后转                      */
void mouseTurnback_C1(void);                                               /*  向后转                      */
void mouseTurnleft_C(void);                                               /*  向左转90度                  */
void mouseTurnright_C(void);                                              /*  向右转90度                  */
void mouseTurnleft_KC(void);                                               /*  向左转90度                  */
void mouseTurnright_KC(void);                                              /*  向右转90度                  */
uint8 DenggaoCheck (void);
uint8 PulseCtrCheck (void);
void mouseTurnleft_Y(void);                                               /*  向左转90度                  */
void mouseTurnright_Y(void);                                              /*  向右转90度                  */
uint8 startCheck (void);
void GYRO_Z_Angle(void);
void Go_one_grid();                                                  //向前走一格

void stop();
void mouseTurnback_correct(uint16 Pulse);                                 //原地转弯矫正   
uint8 startCheck2(void);                                                  //起跑检测
void putstring(uint8 *string);                                              //串口打印一串字符
void delayms(uint16 ms);                                                  //延时毫秒
#endif
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/

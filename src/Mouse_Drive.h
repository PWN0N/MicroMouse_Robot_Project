/****************************************Copyright (c)****************************************************
**                               Guangzhou ZHIYUAN electronics Co.,LTD.
**                                     
**                                 http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:           Mouse.Drive.h
** Last modified Date:  
** Last Version: 
** Description:         �ײ���������ͷ�ļ�
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
  ����ͷ�ļ�
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
  PA�˿ڶ���
*********************************************************************************************************/                                
#define __UART0RX             GPIO_PIN_0
#define __UART0TX             GPIO_PIN_1
/*********************************************************************************************************
  PB�˿ڶ���
*********************************************************************************************************/
#define __IRSEND1_FRONT       GPIO_PIN_0                                  /*  ����ǰ�����ⷢ��      */
#define __IRSEND3_LR          GPIO_PIN_1                                  /*  �������Һ���ĵ����ź�      */
#define __FRONTSIDE_L         GPIO_PIN_4                                  /*  45�󴫸���������ź�      */
#define __FRONTSIDE_R         GPIO_PIN_5                                  /*  45�Ҵ�����������ź�      */
#define __LED                 GPIO_PIN_6
/*********************************************************************************************************
  PC�˿ڶ���
*********************************************************************************************************/
#define __KEY                 GPIO_PIN_5                                  /*  �������ӵĶ˿�              */
#define __IRSEND2_BEVEL       GPIO_PIN_6                                  /*  ����б�Ǻ���ĵ����ź�      */
#define __START               GPIO_PIN_7                                  /*  �������ӵĶ˿�              */

/*********************************************************************************************************
  PD�˿ڶ���
*********************************************************************************************************/
#define __D1_1                GPIO_PIN_0                                  /*  PWM0 ��� M1 PWMH      */
#define __D1_2                GPIO_PIN_1                                  /*  PWM1 ��� M1 PWML      */
#define __FRONT_R             GPIO_PIN_2                                  /*  ǰ���Ҵ�����������ź�        */
#define __RIGHTSIDE           GPIO_PIN_3                                  /*  �ҷ�������������ź�        */
#define __SIG_M2_B            GPIO_PIN_4                                  /*  ������ ��� M2 B��      */
#define __DIR2                GPIO_PIN_5                                  /*  ������ ��� M2 A��      */
#define __LEFTSIDE            GPIO_PIN_6                                  /*  �󷽴�����������ź�        */
#define __FRONT_L             GPIO_PIN_7                                  /*  ǰ���󴫸���������ź�        */
/*********************************************************************************************************
  PE�˿ڶ���
*********************************************************************************************************/
#define __D2_1                GPIO_PIN_0                                  /*  PWM4 ��� M2 PWMH      */
#define __D2_2                GPIO_PIN_1                                  /*  PWM5 ��� M2 PWML      */
#define __DIR1                GPIO_PIN_2                                  /*  ������ ��� M1 A��      */
#define __SIG_M1_B            GPIO_PIN_3                                  /*  ������ ��� M1 B��      */


/*********************************************************************************************************
  �����궨��--������
*********************************************************************************************************/
#define __LEFT              0                                           /*  �󷽴�����                  */
#define __FRONTL            1                                           /*  ��ǰ��������                */
#define __FRONT             2                                           /*  ǰ��������                  */
#define __FRONTR            3                                           /*  ��ǰ��������                */
#define __RIGHT             4                                           /*  �ҷ�������                  */


/*********************************************************************************************************
  �����궨��--������״̬
*********************************************************************************************************/
#define __STOP              0                                           /*  ������ֹͣ                  */
#define __GOAHEAD           1                                           /*  ������ǰ��                  */
#define __TURNLEFT          3                                           /*  ����������ת                */
#define __TURNRIGHT         4                                           /*  ����������ת                */
#define __TURNBACK          5                                           /*  ���������ת                */
#define __TURNLEFTY         7                                           /*  ����������ת                */
#define __TURNRIGHTY        8                                           /*  ����������ת                */

/*********************************************************************************************************
  �����궨��--����Ӽ��ٶ�
*********************************************************************************************************/
#define __SPEEDUP         0                                           /*  �������                    */
#define __SPEEDDOWN       1                                           /*  �������                */

/*********************************************************************************************************
  �����궨��--���״̬
*********************************************************************************************************/
#define __MOTORSTOP         0                                           /*  ���ֹͣ                    */
#define __WAITONESTEP       1                                           /*  �����ͣһ��                */
#define __MOTORRUN          2                                           /*  �������                    */
#define __CORRECT           6                                           /*  ���������                    */

/*********************************************************************************************************
  �����궨��--������з���
*********************************************************************************************************/
#define __MOTORGOAHEAD      0                                           /*  ���ǰ��                    */
#define __MOTORGOBACK       1                                           /*  �������                    */
#define __MOTORGOSTOP       2                                           /*  ��������ƶ�                */

/*********************************************************************************************************
  �����궨��--PID
*********************************************************************************************************/
#define __KP 40     //���� 30
#define __KI 0.01    //���� 0.01
#define __KD 0            //΢��

#define U_MAX 1000       //���ص����ֵ,��pwm������ֵ 
#define U_MIN 5 
#define error_IMAX 10     //�����޷� 
#define Deadband 1   //�ٶ�PID������������Χ

/*********************************************************************************************************
  �ṹ�嶨��
*********************************************************************************************************/
struct __motor {
    int8    cState;                                                     /*  �������״̬                */
    int8    cDir;                                                       /*  ������з���                */
    int8    cRealDir;                                                   /*  ������з���                */
    uint32  uiPulse;                                                    /*  �����Ҫ���е�����          */
    uint32  uiPulseCtr;                                                 /*  ��������е�����            */
    int16   sSpeed;                                                    /*  ��ǰռ�ձ�                    */
};
typedef struct __motor __MOTOR;

struct __pid       //���������������� 
{ 
    //uint16 usRef;      //�ٶ�PID���ٶ��趨ֵ 
    uint16 usFeedBack;  //�ٶ�PID���ٶȷ���ֵ
    uint16 usEncoder_new; //������
    uint16 usEncoder_last; //������
    
    int16 sRef;
    int16 sFeedBack;
    int16 sPreError;  //�ٶ�PID��ǰһ�Σ��ٶ����,,vi_Ref - vi_FeedBack 
    int16 sPreDerror; //�ٶ�PID��ǰһ�Σ��ٶ����֮�d_error-PreDerror; 
  
    fp32 fKp;      //�ٶ�PID��Ka = Kp 
    fp32 fKi;      //�ٶ�PID��Kb = Kp * ( T / Ti ) 
    fp32 fKd;      //�ٶ�PID�� 
       
    int16 iPreU;    //����������ֵ      
};
typedef struct __pid __PID;

extern uint8 flag_bug1;
/*********************************************************************************************************
  �����궨��
*********************************************************************************************************/

uint8 keyCheck(void);
void mouseInit(void);
void mouseGoahead(int8  cNBlock);                                       /*  ǰ��N��                     */
void mazeSearch(void);                                                  /*  ǰ��N��                     */
void mouseTurnleft(void);                                               /*  ����ת90��                  */
void mouseTurnright(void);                                              /*  ����ת90��                  */
void mouseTurnback(void);                                               /*  ���ת                      */
void sensorDebug(void);                                                 /*  ����������                  */
void voltageDetect(void);
void voltageDetectRef(void);
void mouseGoaheadhui(int8  cNBlock);                                       /*  ǰ��N��                     */
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
static void __wallCheck(void);                                          /*  ǽ�ڼ��                    */
static void __sensorInit(void);
static void __MotorIint(void);
static void __keyInit(void);
static void __sysTickInit(void);


void mouseTurnleft_L(void);                                               /*  ����ת90��                  */
void mouseTurnright_L(void);                                              /*  ����ת90��                  */
void mouseGoahead_L(int8  cNBlock);                                       /*  ǰ��N��                     */
void mouseStop(void); 
void mouseGo(void);
void __keyIntDisable (void);
void mouseTurnback_C(void);                                               /*  ���ת                      */
void mouseTurnback_C1(void);                                               /*  ���ת                      */
void mouseTurnleft_C(void);                                               /*  ����ת90��                  */
void mouseTurnright_C(void);                                              /*  ����ת90��                  */
void mouseTurnleft_KC(void);                                               /*  ����ת90��                  */
void mouseTurnright_KC(void);                                              /*  ����ת90��                  */
uint8 DenggaoCheck (void);
uint8 PulseCtrCheck (void);
void mouseTurnleft_Y(void);                                               /*  ����ת90��                  */
void mouseTurnright_Y(void);                                              /*  ����ת90��                  */
uint8 startCheck (void);
void GYRO_Z_Angle(void);
void Go_one_grid();                                                  //��ǰ��һ��

void stop();
void mouseTurnback_correct(uint16 Pulse);                                 //ԭ��ת�����   
uint8 startCheck2(void);                                                  //���ܼ��
void putstring(uint8 *string);                                              //���ڴ�ӡһ���ַ�
void delayms(uint16 ms);                                                  //��ʱ����
#endif
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/

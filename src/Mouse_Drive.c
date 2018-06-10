/****************************************Copyright (c)****************************************************
**                               Guangzhou ZHIYUAN electronics Co.,LTD.
**                                     
**                                 http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:           Mouse_Drive.c
** Last modified Date: 
** Last Version: 
** Description:         ������ײ�����
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
** Description:                          ���ж���stop
**
*********************************************************************************************************/


/*********************************************************************************************************
  ����ͷ�ļ�
*********************************************************************************************************/
#include "Mouse_Drive.h"
#include "Maze.h"
/*********************************************************************************************************
  ����ȫ�ֱ���
*********************************************************************************************************/
MAZECOOR          GmcMouse                        = {0,0};              /*  ���������ǰλ������      */
uint8             GucMouseDir                     = UP;                 /*  ���������ǰ����          */
uint8             GucMapBlock[MAZETYPE][MAZETYPE] = {0};                /*  GucMapBlock[x][y]           */
                                                                        /*  x,������;y,������;          */
uint8             GucMapBlock0[MAZETYPE][MAZETYPE] = {0};               /*  �����㷨         */
                                                                   /*  bit3~bit0�ֱ������������   */
                                                                        /*  0:�÷�����·��1:�÷�����·  */
uint8             GucMapBlock1[MAZETYPE][MAZETYPE]= {0x0f};
uint8             GucMouseStart                    = 0;                 /* ����������        */
uint8             GucFrontJinju                    = 0;                 /* ǰ���������������ʱ�ȸ�ͼ��������   */
uint8             GucCrossroad                     = 0;                 /* ʮ��·���������ʱ�ã���ʮ��·�ڶཱུ����߳���ٶ�   */
static uint32     GW;                                                       /*С��ת���Ƕ�*/

static __MOTOR  __GmLeft                          = {0, 0, 0, 0, 0, 0};    /*  ���岢��ʼ������״̬      */
static __MOTOR  __GmRight                         = {0, 0, 0, 0, 0, 0};    /*  ���岢��ʼ���ҵ��״̬      */
static __PID    __GmLPID;                                                 /*  ��������PID      */
static __PID    __GmRPID;                                                 /*  �����ҵ��PID     */
static __PID    __GmSPID;                                                 /*  ֱ��PID     */
static __PID    __GmWPID;                                                 /*  ��תPID     */
static uint8    __GucMouseState                   = __STOP;             /*  ���������ǰ����״̬      */
static int32    __GiMaxSpeed                      = SEARCHSPEED;        /*  �����������е�����ٶ�      */
static uint8    __GucDistance[5]                  = {0};                /*  ��¼������״̬              */
uint16   GusFreq_F                         = 36200;   //33.8,33,327        /*  ǰ������Ƶ��              */
uint16   GusFreq_FJ                        = 19200;   //26.3,266,275              /*  ǰ���������Ƶ��              */
uint16   GusFreq_X                         = 30000;   //35,33.8          /*  б45�Ⱥ���Ƶ��              */
uint16   GusFreq_LF                        = 31700;   //34000           /*  ���Һ���Զ��Ƶ��              */
uint16   GusFreq_L                         = 18300;              /*  ���Һ������Ƶ��              */
static  int16   GsTpusle_T                       = 0;                  /*  ����У�����ٵ��ٶ�ֵ              */
static uint8    GuiSpeedCtr                       = 0;
static int16   GuiTpusle_LR                      = 0;
static int16   GuiTpusle_back_LR                 =0;
static int16   GuiTpusle_S                       = 0;
static uint8    GucFrontNear                      = 0;
uint8    GucFangXiang                      = 0;
uint8    GucDirTemp                        = 0;
uint32    DIS[10]                           = {0};
uint8 Tab=0;//�������ѡ��

uint16 buffer=0; //�ٶ�����
uint8 s=0;  //UARTCharPut(UART0_BASE, s++); ������
uint8 flag_bug=0;  //��bugʱ��־λ
uint8 flag_bug1=0;

/*********************************************************************************************************
** Function name:       __delay
** Descriptions:        ��ʱ����
** input parameters:    uiD :��ʱ������ֵԽ����ʱԽ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __delay (uint32  uiD)
{
    for (; uiD; uiD--);
}
/*********************************************************************************************************
** Function name:       PIDInit
** Descriptions:        PID��ʼ��
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void PIDInit(void) 
{  
    __GmLPID.usEncoder_new = 0;
    __GmLPID.usEncoder_last = 65535;
    __GmLPID.usFeedBack = 0 ;  //�ٶȷ���ֵ
    __GmLPID.sFeedBack = 0 ;
    
    __GmRPID.usEncoder_new = 0;
    __GmRPID.usEncoder_last = 65535;
    __GmRPID.usFeedBack = 0 ;  //�ٶȷ���ֵ
    __GmRPID.sFeedBack = 0 ;
    
    __GmSPID.sRef = 0 ;        //�ٶ��趨ֵ 
    __GmSPID.sFeedBack = 0 ;        
    __GmSPID.sPreError = 0 ;   //ǰһ�Σ��ٶ����,,vi_Ref - vi_FeedBack 
    __GmSPID.sPreDerror = 0 ;   //ǰһ�Σ��ٶ����֮�d_error-PreDerror; 
        
    __GmSPID.fKp = __KP; 
    __GmSPID.fKi = __KI;
    __GmSPID.fKd = __KD; 
       
    __GmSPID.iPreU = 0 ;      //����������ֵ 
    
    __GmWPID.sRef = 0 ;        //�ٶ��趨ֵ 
    __GmWPID.sFeedBack = 0 ;       
    __GmWPID.sPreError = 0 ;   //ǰһ�Σ��ٶ����,,vi_Ref - vi_FeedBack 
    __GmWPID.sPreDerror = 0 ;   //ǰһ�Σ��ٶ����֮�d_error-PreDerror; 
    
    __GmWPID.fKp = __KP;  //30
    __GmWPID.fKi = __KI;  //0.1,0.01
    __GmWPID.fKd = __KD; 
       
    __GmWPID.iPreU = 0 ;      //����������ֵ 
    
}
/*********************************************************************************************************
** Function name:       __Encoder
** Descriptions:        �ɼ����������������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __Encoder(void)                                          
{
    __GmLPID.usEncoder_new = TimerValueGet(TIMER0_BASE, TIMER_B);    
    if((__GmLPID.usEncoder_last - __GmLPID.usEncoder_new) < -32767)
      __GmLPID.usFeedBack = 65536 + __GmLPID.usEncoder_last - __GmLPID.usEncoder_new;
    else
      __GmLPID.usFeedBack = __GmLPID.usEncoder_last - __GmLPID.usEncoder_new;
    __GmLPID.usEncoder_last = __GmLPID.usEncoder_new;
    __GmLeft.uiPulseCtr += __GmLPID.usFeedBack;
  
    __GmRPID.usEncoder_new = TimerValueGet(TIMER0_BASE, TIMER_A);
    if((__GmRPID.usEncoder_last - __GmRPID.usEncoder_new) < -32767)
      __GmRPID.usFeedBack = 65536 + __GmRPID.usEncoder_last - __GmRPID.usEncoder_new;
    else
      __GmRPID.usFeedBack = __GmRPID.usEncoder_last - __GmRPID.usEncoder_new;
    __GmRPID.usEncoder_last = __GmRPID.usEncoder_new;
    __GmRight.uiPulseCtr += __GmRPID.usFeedBack;
    
    if(GPIOPinRead(GPIO_PORTE_BASE, __DIR1) == 0){
      __GmLeft.cRealDir = __MOTORGOAHEAD;
      __GmLPID.sFeedBack= __GmLPID.usFeedBack;       
    }
    else{      
      __GmLeft.cRealDir = __MOTORGOBACK;
      __GmLPID.sFeedBack= -1*__GmLPID.usFeedBack;
    }
    
    if(GPIOPinRead(GPIO_PORTD_BASE, __DIR2) == 0){
      __GmRight.cRealDir = __MOTORGOBACK;
      __GmRPID.sFeedBack = -1*__GmRPID.usFeedBack;
    }
    else{     
      __GmRight.cRealDir = __MOTORGOAHEAD;
      __GmRPID.sFeedBack = __GmRPID.usFeedBack;
    }
    
    __GmSPID.sFeedBack = (__GmRPID.sFeedBack + __GmLPID.sFeedBack)/2 ;
    __GmWPID.sFeedBack = (__GmRPID.sFeedBack - __GmLPID.sFeedBack)/2 ;    
}
/*********************************************************************************************************
** Function name:       __SPIDContr
** Descriptions:        ֱ��PID����
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __SPIDContr(void) 
{ 
    int16  error,d_error,dd_error;
    static uint8   K_I=1;
    error = __GmSPID.sRef - __GmSPID.sFeedBack; // ƫ�����
    d_error = error - __GmSPID.sPreError; 
    dd_error = d_error - __GmSPID.sPreDerror;
    if(error> Deadband)
      error -= Deadband;
    else if(error < -Deadband)
      error += Deadband;
    else
      error = 0;
    if((error > error_IMAX)||(error < -error_IMAX))
      K_I=0;
    else
      K_I=1;
    
    __GmSPID.sPreError = error; //�洢��ǰƫ�� 
    __GmSPID.sPreDerror = d_error;
    
    __GmSPID.iPreU += (int16)(  __GmSPID.fKp * d_error + K_I*__GmSPID.fKi * error  + __GmSPID.fKd*dd_error); 
}
/*********************************************************************************************************
** Function name:       __WPIDContr
** Descriptions:        ��ת����PID����
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __WPIDContr(void) 
{ 
    int16  error,d_error,dd_error; 
    static uint8   K_I=1;
    error = __GmWPID.sRef + GsTpusle_T- __GmWPID.sFeedBack; // ƫ����� 
    d_error = error - __GmWPID.sPreError; 
    dd_error = d_error - __GmWPID.sPreDerror;
    if(error> Deadband)
      error -= Deadband;
    else if(error < -Deadband)
      error += Deadband;
    else
      error = 0;
    if((error > error_IMAX)||(error < -error_IMAX))
      K_I=0;
    else
      K_I=1;
    
    __GmWPID.sPreError = error; //�洢��ǰƫ�� 
    __GmWPID.sPreDerror = d_error;
    __GmWPID.iPreU += (int16)(  __GmWPID.fKp * d_error + K_I*__GmWPID.fKi * error  + __GmWPID.fKd*dd_error);
        
}
/*********************************************************************************************************
** Function name:      __PIDContr
** Descriptions:        PID���ƣ�ͨ�����������Ƶ��
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __PIDContr(void)
{
    __SPIDContr();
    __WPIDContr();
    __GmLeft.sSpeed = __GmSPID.iPreU - __GmWPID.iPreU ;
    if(__GmLeft.sSpeed>=0){
     __GmLeft.cDir=__MOTORGOAHEAD; 
    if( __GmLeft.sSpeed >= U_MAX )   //�ٶ�PID����ֹ���������� 
       __GmLeft.sSpeed = U_MAX;      
    if( __GmLeft.sSpeed <= U_MIN ) //�ٶ�PID����ֹ����������  
       __GmLeft.sSpeed = U_MIN;
    }
    else{
      __GmLeft.cDir=__MOTORGOBACK;
      __GmLeft.sSpeed *=-1; 
    if( __GmLeft.sSpeed >= U_MAX )   //�ٶ�PID����ֹ���������� 
       __GmLeft.sSpeed = U_MAX;      
    if( __GmLeft.sSpeed <= U_MIN ) //�ٶ�PID����ֹ����������  
       __GmLeft.sSpeed = U_MIN;
    }
      
    __GmRight.sSpeed = __GmSPID.iPreU + __GmWPID.iPreU ;
    if(__GmRight.sSpeed>=0){
     __GmRight.cDir=__MOTORGOAHEAD; 
    if( __GmRight.sSpeed >= U_MAX )   //�ٶ�PID����ֹ���������� 
       __GmRight.sSpeed = U_MAX;      
    if( __GmRight.sSpeed <= U_MIN ) //�ٶ�PID����ֹ����������  
       __GmRight.sSpeed = U_MIN;
    }
    else{
      __GmRight.cDir=__MOTORGOBACK;
      __GmRight.sSpeed *=-1; 
    if( __GmRight.sSpeed >= U_MAX )   //�ٶ�PID����ֹ���������� 
       __GmRight.sSpeed = U_MAX;      
    if( __GmRight.sSpeed <= U_MIN ) //�ٶ�PID����ֹ����������  
       __GmRight.sSpeed = U_MIN;
    }
    __rightMotorContr();
    __leftMotorContr();
    
}
/*********************************************************************************************************
** Function name:       __rightMotorContr
** Descriptions:        ��ֱ���������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __rightMotorContr(void)
{
    switch (__GmRight.cDir) 
    {
    case __MOTORGOAHEAD:                                                /*  ��ǰ����                    */
      PWMPulseWidthSet(PWM_BASE, PWM_OUT_5, 0);                         /*  ����PWM5�����������      */
      PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, __GmRight.sSpeed);          /*  ����PWM4�����������      */
        break;

    case __MOTORGOBACK:                                                 /*  ��󲽽�                    */
      PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, 0);                         /*  ����PWM4�����������      */
      PWMPulseWidthSet(PWM_BASE, PWM_OUT_5, __GmRight.sSpeed);          /*  ����PWM5�����������      */
        break;
    case __MOTORGOSTOP:                                                  /*  �����ƶ�                   */
      PWMPulseWidthSet(PWM_BASE, PWM_OUT_5, 0);                     /*  ����PWM5�����������      */
      PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, 0);                     /*  ����PWM4�����������      */
        break;

    default:
        break;
    }
    PWMGenEnable(PWM_BASE, PWM_GEN_2);
}
/*********************************************************************************************************
** Function name:       __leftMotorContr
** Descriptions:        ��ֱ���������
** input parameters:    __GmLeft.cDir :������з���
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __leftMotorContr(void)
{
    switch (__GmLeft.cDir) 
    {
    case __MOTORGOAHEAD:                                                /*  ��ǰ����                    */
      PWMPulseWidthSet(PWM_BASE, PWM_OUT_0, 0);                         /*  ����PWM0�����������      */
      PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, __GmLeft.sSpeed);          /*  ����PWM1�����������      */
        break;

    case __MOTORGOBACK:                                                 /*  ��󲽽�                    */
      PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, 0);                         /*  ����PWM1�����������      */
      PWMPulseWidthSet(PWM_BASE, PWM_OUT_0, __GmLeft.sSpeed);          /*  ����PWM0�����������      */
        break;
    case __MOTORGOSTOP:                                                  /*  �����ƶ�                   */
      PWMPulseWidthSet(PWM_BASE, PWM_OUT_0, 0);                     /*  ����PWM0�����������      */
      PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, 0);                     /*  ����PWM1�����������      */
        break;

    default:
        break;
    }
    PWMGenEnable(PWM_BASE, PWM_GEN_0);
}
/*********************************************************************************************************
** Function name:       __SpeedUp
** Descriptions:        ��������ٳ���
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __SpeedUp (void)
{
    uint16 Speed;
    Speed=__GmSPID.sFeedBack;
    if(__GmSPID.sRef<__GiMaxSpeed){
      if(Speed >=__GmSPID.sRef)
      {
        __GmSPID.sRef=__GmSPID.sRef+2;
      }
    }   
}
/*********************************************************************************************************
** Function name:       __SpeedDown
** Descriptions:        ��������ٳ���
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __SpeedDown (void)
{
    uint16 Speed;
    Speed=__GmSPID.sFeedBack;
    if(__GmSPID.sRef>=MINSPEED){
      if(Speed <=__GmSPID.sRef+4)
      {
       __GmSPID.sRef=__GmSPID.sRef-4;
      }
    }
}
/*********************************************************************************************************
** Function name:       Timer0A_ISR
** Descriptions:        ����������ƥ���ж�
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void Timer0A_ISR(void)
{
   TimerIntClear(TIMER0_BASE, TIMER_CAPA_MATCH);                     /*  �����ʱ��0A�жϡ�           */
   TimerEnable(TIMER0_BASE, TIMER_A); 
}

/*********************************************************************************************************
** Function name:       Timer0B_ISR
** Descriptions:        �ҵ��������ƥ���ж�
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void Timer0B_ISR(void)
{
   TimerIntClear(TIMER0_BASE, TIMER_CAPB_MATCH);                     /*  �����ʱ��0B�жϡ�           */
   TimerEnable(TIMER0_BASE, TIMER_B); 
}

/*********************************************************************************************************
** Function name:       Timer2A_ISR
** Descriptions:        Timer2�жϷ�����
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void Timer2A_ISR(void)
{
    static int8 n = 0,m = 0,t=0,k=0;
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);                     /*  �����ʱ��2A�жϡ�           */
    __Encoder();
    switch (__GmRight.cState) {
        
    case __MOTORSTOP:                                                   /*  ֹͣ��ͬʱ�����ٶȺ�����ֵ  */
        __GmRight.uiPulse    = 0;
        __GmRight.uiPulseCtr = 0;
        __GmLeft.uiPulse    = 0;
        __GmLeft.uiPulseCtr = 0;
        break;

    case __WAITONESTEP:                                                 /*  ��ͣһ��                    */
        __GmRight.cState = __MOTORRUN;
//        if ((__GucDistance[__FRONTL] && (__GucDistance[__FRONTR] == 0))||((__GucDistance[__RIGHT] == 1) && (__GucDistance[__LEFT] == 0)))
//           GsTpusle_T = -2;
//        else if((__GucDistance[__FRONTR] &&(__GucDistance[__FRONTL]==0))||((__GucDistance[__LEFT] == 1) && (__GucDistance[__RIGHT] == 0)))
//           GsTpusle_T = 2;
        
       if((__GucDistance[__RIGHT]!=0) && (__GucDistance[__LEFT] == 0))
        {
          GsTpusle_T = -1;
        }
        else if((__GucDistance[__LEFT]!=0) && (__GucDistance[__RIGHT] == 0))
        {
          GsTpusle_T = 1;
        }      
         if((__GucDistance[__RIGHT]==0x03)&&(__GucDistance[__LEFT]!=0x03))
        {GsTpusle_T=3;}
        else if((__GucDistance[__RIGHT]!=0x03)&&(__GucDistance[__LEFT]==0x03))
        {GsTpusle_T=-3;}
        
        if(__GucDistance[__FRONTL] && (__GucDistance[__FRONTR] == 0))
        {
          GsTpusle_T = -3;
          if(__GmSPID.sFeedBack>50)
          {GsTpusle_T = -2;}
          if(__GmSPID.sFeedBack>80)
          {GsTpusle_T = -1;}
        }
        else if(__GucDistance[__FRONTR] &&(__GucDistance[__FRONTL]==0))
        {
          GsTpusle_T = 3;
           if(__GmSPID.sFeedBack>50)
          {GsTpusle_T = 2;}
           if(__GmSPID.sFeedBack>80)
          {GsTpusle_T = 1;}
        }
       
       
        __PIDContr();
        break;

    case __MOTORRUN:                                                    /*  �������                    */
      if (__GucMouseState == __GOAHEAD)                                 /*  ���ݴ�����״̬΢�����λ��  */
      {                             
            if ((__GucDistance[__FRONTL] && (__GucDistance[__FRONTR] == 0))||(__GucDistance[__FRONTR] &&(__GucDistance[__FRONTL]==0)))     /* ƫ��,ƫ�� */
            {
              if (n ==2)
              {
                    __GmRight.cState = __WAITONESTEP;
                    //__GmRight.uiPulseCtr += 2;
                 /*if (__GucDistance[__FRONTL] && (__GucDistance[__FRONTR] == 0))
                    GsTpusle_T = -1;
                 else
                    GsTpusle_T = 1;*/
              }               
              n++;
              n %= 4;
            }            
            else if (((__GucDistance[__RIGHT] == 1) && (__GucDistance[__LEFT] == 0))||((__GucDistance[__LEFT] == 1) && (__GucDistance[__RIGHT] == 0)))   /* Զƫ��Զƫ�� */
            {
                if(m ==2) 
                {
                    __GmRight.cState = __WAITONESTEP;
                }
                m++;
                m %=4;
            } 
             else if(((__GucDistance[__RIGHT]==0x03)&&(__GucDistance[__LEFT]!=0x03))||((__GucDistance[__RIGHT]!=0x03)&&(__GucDistance[__LEFT]==0x03)))
            {
              if(t==3)
              {
                __GmRight.cState = __WAITONESTEP;
              }
              t++;
              t%=6;
            }
            else 
            {
                m  = 0;
                n = 0;
                t=0;
                GsTpusle_T = 0;
            }
        
            if(GuiSpeedCtr==__SPEEDUP)
            { 
              k=(k+1)%20;
              if(k==19)
              __SpeedUp();              
            }
            else if(GuiSpeedCtr==__SPEEDDOWN)
            {
              k=(k+1)%10;
              if(k==9)
              __SpeedDown(); 
            }
            else;
        }
      else{
        GsTpusle_T = 0;
        GYRO_Z_Angle();
      }     
        __PIDContr();
        break;
    case 4:
      GsTpusle_T = 0;
      __PIDContr();
      break;
      
    case 5:
      __GmRight.sSpeed = 5;
      __rightMotorContr();
      __GmRight.cState = __MOTORSTOP;
      __GmLeft.sSpeed = 5;
      __leftMotorContr();
      __GmLeft.cState = __MOTORSTOP;
      break;
      
     case __CORRECT:
      __GmRight.cDir=__MOTORGOBACK;
      __GmLeft.cDir=__MOTORGOBACK;
      __GmRight.sSpeed =buffer;
      __rightMotorContr();
      __GmLeft.sSpeed =buffer;
      __leftMotorContr();
      if(buffer<500)
      {buffer++;}
      break;
      
    default:
        break;
    }
   
}
/*********************************************************************************************************
** Function name:       UART0_ISR
** Descriptions:        UART0�жϷ�����
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void UART0_ISR(void)
{
    unsigned long Status;
    uint8 dat;
    Status=UARTIntStatus(UART0_BASE, true);
    UARTIntClear(UART0_BASE, Status);
    dat=(uint8)UARTCharGet(UART0_BASE);
    UARTCharPut(UART0_BASE, dat);
}

/*********************************************************************************************************
** Function name:       GPIO_Port_C_ISR
** Descriptions:        START�����жϷ�����
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void GPIO_Port_C_ISR (void)
{
   uint8 status;
   status = GPIOPinIntStatus(GPIO_PORTC_BASE, true);              /*  ��PC���ж�״̬              */
      if(status & __START)                                       /*  start�����ж�          */
    {
        GPIOPinIntClear(GPIO_PORTC_BASE, __START);                  /*  ���ж�                      */
        if (GPIOPinRead(GPIO_PORTC_BASE, __START) == 0) 
        {
          __delay(50);
          while(GPIOPinRead(GPIO_PORTC_BASE, __START) == 0);
        }
    }
}
/*********************************************************************************************************
** Function name:       GPIO_Port_A_ISR
** Descriptions:        ZLG7289�����жϷ�����
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void GPIO_Port_A_ISR (void)                                         /*  �������ں���Ƶ��             */
{
    uint8 status;
    uint8 key;
    uint8 B,S,G;
    status = GPIOPinIntStatus(GPIO_PORTA_BASE, true);              /*  ��PA���ж�״̬              */

    if(status & ZLG7289_KEY)                                       /*  �ж��Ƿ�Ϊ�����ж�          */
    {
        GPIOPinIntClear(GPIO_PORTA_BASE, ZLG7289_KEY);                  /*  ���ж�                      */
        
        key = zlg7289Key();                                           /*  ������ֵ                    */
        /*��������Ч������8�������һ����ʾ*/
        if (key != 0xff) 
        {
          if(key>=4)
          {
              switch(key)
              {
              case 4:
                Tab=0;
                B=GusFreq_F/10000;
                S=(GusFreq_F/1000)%10;
                G=(GusFreq_F/100)%10;
                zlg7289Download(1, 5, 0, B);
                zlg7289Download(1, 6, 1, S);
                zlg7289Download(1, 7, 0, G); 
                break;
                
              case 5:
                Tab=1;
                B=GusFreq_X/10000;
                S=(GusFreq_X/1000)%10;
                G=(GusFreq_X/100)%10;
                zlg7289Download(1, 5, 0, B);
                zlg7289Download(1, 6, 1, S);
                zlg7289Download(1, 7, 0, G);
                break;
                
              case 6:
                Tab=2;
                 B=GusFreq_LF/10000;
                 S=(GusFreq_LF/1000)%10;
                 G=(GusFreq_LF/100)%10;
                 zlg7289Download(1, 5, 0, B);
                 zlg7289Download(1, 6, 1, S);
                 zlg7289Download(1, 7, 0, G);
                break;
              case 7:
                Tab=3;
                 B=GusFreq_L/10000;
                 S=(GusFreq_L/1000)%10;
                 G=(GusFreq_L/100)%10;
                 zlg7289Download(1, 5, 0, B);
                 zlg7289Download(1, 6, 1, S);
                 zlg7289Download(1, 7, 0, G);
                break;
              case 8:
                Tab=4;
               B=GusFreq_FJ/10000;
               S=(GusFreq_FJ/1000)%10;
               G=(GusFreq_FJ/100)%10;
               zlg7289Download(1, 5, 0, B);
               zlg7289Download(1, 6, 1, S);
               zlg7289Download(1, 7, 0, G);
               break;
               
               case 10:              
                DIS[0]=GusFreq_F/1000;
                DIS[1]=GusFreq_F%1000/100;
                DIS[2]=(GusFreq_X/1000);
                DIS[3]=(GusFreq_X%1000/100);
                DIS[4]=(GusFreq_LF/1000);
                DIS[5]=(GusFreq_LF%1000/100);
                DIS[6]=(GusFreq_L/1000);
                DIS[7]=(GusFreq_L%1000/100);
                DIS[8]=(GusFreq_FJ/1000);
                DIS[9]=(GusFreq_FJ%1000/100);
                write_fm24LC16(DIS,0x00,0xa0,10);
                __delay(1000000);
                break;
                
              default:
                break;
            }
          }
          else
          {
              if(key==1)
              {
                switch(Tab)
                {
                case 0:
                     GusFreq_F +=100;
                     B=GusFreq_F/10000;
                     S=(GusFreq_F/1000)%10;
                     G=(GusFreq_F/100)%10;
                     zlg7289Download(1, 5, 0, B);
                     zlg7289Download(1, 6, 1, S);
                     zlg7289Download(1, 7, 0, G); 
                     break;
                     
                   case 1:
                     GusFreq_X +=100;
                     B=GusFreq_X/10000;
                     S=(GusFreq_X/1000)%10;
                     G=(GusFreq_X/100)%10;
                     zlg7289Download(1, 5, 0, B);
                     zlg7289Download(1, 6, 1, S);
                     zlg7289Download(1, 7, 0, G);
                     break;
                     
                   case 2:
                     GusFreq_LF +=100;
                     B=GusFreq_LF/10000;
                     S=(GusFreq_LF/1000)%10;
                     G=(GusFreq_LF/100)%10;
                     zlg7289Download(1, 5, 0, B);
                     zlg7289Download(1, 6, 1, S);
                     zlg7289Download(1, 7, 0, G);
                     break;
                     
                   case 3:
                     GusFreq_L +=100;
                     B=GusFreq_L/10000;
                     S=(GusFreq_L/1000)%10;
                     G=(GusFreq_L/100)%10;
                     zlg7289Download(1, 5, 0, B);
                     zlg7289Download(1, 6, 1, S);
                     zlg7289Download(1, 7, 0, G);
                     break;
                     
                   case 4:
                     GusFreq_FJ +=100;
                     B=GusFreq_FJ/10000;
                     S=(GusFreq_FJ/1000)%10;
                     G=(GusFreq_FJ/100)%10;
                     zlg7289Download(1, 5, 0, B);
                     zlg7289Download(1, 6, 1, S);
                     zlg7289Download(1, 7, 0, G);
                     break;
                     
                   default:
                     break;
                           
               }
            }
            
            if(key==3)
            {
               switch(Tab)
                {
                case 0:
                     GusFreq_F -=100;
                     B=GusFreq_F/10000;
                     S=(GusFreq_F/1000)%10;
                     G=(GusFreq_F/100)%10;
                     zlg7289Download(1, 5, 0, B);
                     zlg7289Download(1, 6, 1, S);
                     zlg7289Download(1, 7, 0, G); 
                     break;
                     
                   case 1:
                     GusFreq_X -=100;
                     B=GusFreq_X/10000;
                     S=(GusFreq_X/1000)%10;
                     G=(GusFreq_X/100)%10;
                     zlg7289Download(1, 5, 0, B);
                     zlg7289Download(1, 6, 1, S);
                     zlg7289Download(1, 7, 0, G);
                     break;
                     
                   case 2:
                     GusFreq_LF -=100;
                     B=GusFreq_LF/10000;
                     S=(GusFreq_LF/1000)%10;
                     G=(GusFreq_LF/100)%10;
                     zlg7289Download(1, 5, 0, B);
                     zlg7289Download(1, 6, 1, S);
                     zlg7289Download(1, 7, 0, G);
                     break;
                     
                   case 3:
                     GusFreq_L -=100;
                     B=GusFreq_L/10000;
                     S=(GusFreq_L/1000)%10;
                     G=(GusFreq_L/100)%10;
                     zlg7289Download(1, 5, 0, B);
                     zlg7289Download(1, 6, 1, S);
                     zlg7289Download(1, 7, 0, G);
                     break;
                     
                   case 4:
                     GusFreq_FJ -=100;
                     B=GusFreq_FJ/10000;
                     S=(GusFreq_FJ/1000)%10;
                     G=(GusFreq_FJ/100)%10;
                     zlg7289Download(1, 5, 0, B);
                     zlg7289Download(1, 6, 1, S);
                     zlg7289Download(1, 7, 0, G);
                     break;
                     
                   default:
                     break;
                           
               }
            }       
          }
                    
        }
    }
}

/*********************************************************************************************************
** Function name:       mazeSearch
** Descriptions:        ǰ��N��
** input parameters:    iNblock: ǰ���ĸ���
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void mouseGoahead (int8  cNBlock)                                    //����ת����
{
    int8 cL = 0, cR = 0, cCoor = 1,cB;
    
    GuiSpeedCtr=__SPEEDUP;
    if (__GmLeft.cState) {
        cCoor = 0;
    }
       if((__GucMouseState==__TURNRIGHT)||(__GucMouseState==__TURNLEFT))
    {
        if(GucDirTemp==0)  //����Ŀ�ĵز���Ҫת��
        {
           GuiTpusle_back_LR = 1400;  //  <3000
           
        }
        else
        {
           GuiTpusle_back_LR = 2000;  //����Ŀ�ĵ���Ҫת��
           GuiTpusle_S  = 1600;     //  <1000
        }
    }
    else
    {
        if(GucDirTemp==0)
        {
           GuiTpusle_back_LR = 300;  //<3000
        }
        else
        {
           GuiTpusle_back_LR = 1100;
           GuiTpusle_S  = 1600;
        }
    }
    if(cNBlock==1)
    {
        cL = 1;
        cR = 1;
        __GiMaxSpeed = 43;
    }
    else{
    }
    GucFangXiang = GucDirTemp;
    if(((GmcMouse.cX==7)&&(GmcMouse.cY==7))|| 
         ((GmcMouse.cX==8)&&(GmcMouse.cY==8))||
         ((GmcMouse.cX==7)&&(GmcMouse.cY==8))||
           ((GmcMouse.cX==8)&&(GmcMouse.cY==7))){
       cL = 0;
       cR = 0;
    }
    if((__GucMouseState==__TURNRIGHT)||(__GucMouseState==__TURNLEFT))
    {
        __GmLeft.uiPulseCtr = 9000;       //1182(34mm)  //7200
        __GmRight.uiPulseCtr = 9000;      //7200
    }
    
    cB=cNBlock;
    __GucMouseState   = __GOAHEAD;
    __GmRight.uiPulse = __GmRight.uiPulse + cNBlock * ONEBLOCK ;
    __GmLeft.uiPulse  = __GmLeft.uiPulse  + cNBlock * ONEBLOCK ;
    __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
    if(cNBlock >2)
    {
      __GiMaxSpeed=110;
    }
    else
     ;
    while (__GmLeft.cState != __MOTORSTOP) {
       
        if (__GmLeft.uiPulseCtr >= ONEBLOCK) {                          /*  �ж��Ƿ�����һ��            */
            __GmLeft.uiPulse    -= ONEBLOCK;
            __GmLeft.uiPulseCtr -= ONEBLOCK;
            if (cCoor) {
                cNBlock--;
                if(cNBlock==0)
                   goto End;
                if(cNBlock<cB-1)//������һ��ʱ��
                  GuiSpeedCtr=__SPEEDUP;
            } else {
                cCoor = 1;
            }
        }
        
        if (__GmRight.uiPulseCtr >= ONEBLOCK) {                         /*  �ж��Ƿ�����һ��            */
            __GmRight.uiPulse    -= ONEBLOCK;
            __GmRight.uiPulseCtr -= ONEBLOCK;
        }
        if(cNBlock==2)
        {
          if(__GmSPID.sFeedBack>60){
              GuiSpeedCtr= 3;
             __GmSPID.sRef=60;
             __GiMaxSpeed = 60;
          } 
        }
        if (cNBlock < 2) {
          if(__GmSPID.sFeedBack>43){
              GuiSpeedCtr= 3;
             __GmSPID.sRef=43;
             __GiMaxSpeed = 43;
          }  
          if (cL) 
          {                                                       /*  �Ƿ����������            */
            if ((__GucDistance[ __LEFT] & 0x01) == 0)             /*  �����֧·����������        */
            {                 
                __GmRight.uiPulse = __GmRight.uiPulseCtr  + 3600- GuiTpusle_back_LR;    //3094(89mm)
                __GmLeft.uiPulse  = __GmLeft.uiPulseCtr   + 3600- GuiTpusle_back_LR;    //2600
                while ((__GucDistance[ __LEFT] & 0x01) == 0) 
                {
                    
                    if ((__GmLeft.uiPulseCtr + 600) > __GmLeft.uiPulse) 
                    {
                        goto End;
                    }
                }
            }
            } else {                                                    /*  �����ǽʱ��ʼ���������  */
                if ( __GucDistance[ __LEFT] & 0x01) {
                    cL = 1;
                }
            }
         if (cR) 
            {                                                       /*  �Ƿ��������ұ�            */
            if ((__GucDistance[__RIGHT] & 0x01) == 0)               /*  �ұ���֧·����������        */
            {                
                __GmRight.uiPulse = __GmRight.uiPulseCtr + 3600- GuiTpusle_back_LR;
                __GmLeft.uiPulse  = __GmLeft.uiPulseCtr + 3600- GuiTpusle_back_LR;
                while ((__GucDistance[ __RIGHT] & 0x01) ==0) 
                {
                    
                    if ((__GmLeft.uiPulseCtr + 600) > __GmLeft.uiPulse) 
                    {
                        goto End;
                    }
                }
            }
            } else {
                if ( __GucDistance[__RIGHT] & 0x01) {                   /*  �ұ���ǽʱ��ʼ�������ұ�  */
                    cR = 1;
                }
            }
        }
   }
End: ;
  
}

void mouseGoahead_Llow (int8  cNBlock)                                    //����ת����,����ʮ��·�ڵ��١�������
{
    int8 cL = 0, cR = 0, cCoor = 1,cB;
    GuiSpeedCtr=__SPEEDUP;
    if (__GmLeft.cState) {
        cCoor = 0;
    }
    if((__GucMouseState==__TURNRIGHT)||(__GucMouseState==__TURNLEFT))
    {
        if(GucDirTemp==0)  //����Ŀ�ĵز���Ҫת��
        {
           GuiTpusle_back_LR = 1400;  //  <3000
        }
        else
        {
           GuiTpusle_back_LR = 2000;  //����Ŀ�ĵ���Ҫת��
           GuiTpusle_S  = 1600;     //  <1000
        }
    }
    else
    {
        if(GucDirTemp==0)
        {
           GuiTpusle_back_LR = 300;  //<3000
        }
        else
        {
           GuiTpusle_back_LR = 1100;
           GuiTpusle_S  = 1600;
        }
    }
    if(cNBlock==1)
    {
        cL = 1;
        cR = 1;        
       __GiMaxSpeed = 43;//40
       if(flag_bug1==1)
       {
         if(((__GucDistance[ __LEFT] & 0x01) == 0)||((__GucDistance[ __RIGHT] & 0x01) == 0))
           {
            cL = 0;
            cR = 0;    
           }
         flag_bug1=0;
      }
    }
    else{
        __GiMaxSpeed = 60;//60
    }
    GucFangXiang = GucDirTemp;
    GucDirTemp=0;
    if(((GmcMouse.cX==7)&&(GmcMouse.cY==7))|| 
         ((GmcMouse.cX==8)&&(GmcMouse.cY==8))||
         ((GmcMouse.cX==7)&&(GmcMouse.cY==8))||
           ((GmcMouse.cX==8)&&(GmcMouse.cY==7))){
       cL = 0;
       cR = 0;  
    }
    if((__GucMouseState==__TURNRIGHT)||(__GucMouseState==__TURNLEFT))
    {
        __GmLeft.uiPulseCtr = 9000;       //1182(34mm)
        __GmRight.uiPulseCtr = 9000;
    }
    
    cB=cNBlock;
    __GucMouseState   = __GOAHEAD;
    __GmRight.uiPulse = __GmRight.uiPulse + cNBlock * ONEBLOCK ;
    __GmLeft.uiPulse  = __GmLeft.uiPulse  + cNBlock * ONEBLOCK ;
    __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
    
    while (__GmLeft.cState != __MOTORSTOP) {                                                      
       
        if (__GmLeft.uiPulseCtr >= ONEBLOCK) {                          /*  �ж��Ƿ�����һ��            */
            __GmLeft.uiPulse    -= ONEBLOCK;
            __GmLeft.uiPulseCtr -= ONEBLOCK;
            if (cCoor) {
                cNBlock--;
                if(cNBlock==0)
                   goto End;
                if(cNBlock<cB)
                  GuiSpeedCtr=__SPEEDUP;
            } else {
                cCoor = 1;
            }
        }
        
        if (__GmRight.uiPulseCtr >= ONEBLOCK) {                         /*  �ж��Ƿ�����һ��            */
            __GmRight.uiPulse    -= ONEBLOCK;
            __GmRight.uiPulseCtr -= ONEBLOCK;
        }
        if (cNBlock < 2) {
          if(__GmSPID.sFeedBack>43){
              GuiSpeedCtr= 3;
              __GmSPID.sRef=43;
              __GiMaxSpeed = 43;
          }  
          if (cL) 
          {                                                       /*  �Ƿ����������            */
            if ((__GucDistance[ __LEFT] & 0x01) == 0)             /*  �����֧·����������        */
            {                 
                __GmRight.uiPulse = __GmRight.uiPulseCtr + 3600- GuiTpusle_back_LR;    //3094(89mm)
                __GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 3600- GuiTpusle_back_LR;
                while ((__GucDistance[ __LEFT] & 0x01) == 0) 
                {
                    
                    if ((__GmLeft.uiPulseCtr + 600) > __GmLeft.uiPulse) 
                    {
                        goto End;
                    }
                }
            }
            } else {                                                    /*  �����ǽʱ��ʼ���������  */
                if ( __GucDistance[ __LEFT] & 0x01) {
                    cL = 1;
                }
            }
         if (cR) 
            {                                                       /*  �Ƿ��������ұ�            */
            if ((__GucDistance[__RIGHT] & 0x01) == 0)               /*  �ұ���֧·����������        */
            {                
                __GmRight.uiPulse = __GmRight.uiPulseCtr + 3600- GuiTpusle_back_LR;
                __GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 3600- GuiTpusle_back_LR;
                while ((__GucDistance[ __RIGHT] & 0x01) ==0) 
                {
                    
                    if ((__GmLeft.uiPulseCtr + 600) > __GmLeft.uiPulse) 
                    {
                        goto End;
                    }
                }
            }
            } else {
                if ( __GucDistance[__RIGHT] & 0x01) {                   /*  �ұ���ǽʱ��ʼ�������ұ�  */
                    cR = 1;
                }
            }
        }
   }
    /*
     *  �趨���������õ������ߵ�֧·������λ��
     */
End:    ;
        
}
/*********************************************************************************************************
** Function name:       mazeSearch
** Descriptions:        ǰ��N��
** input parameters:    iNblock: ǰ���ĸ���
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void mazeSearch(void)                  //��������ת��
{
    int8 cL = 0, cR = 0, cCoor = 1,cj=0;
    uint16 bug_LR=0;

    __GmSPID.sRef=36;
    __GiMaxSpeed=36;
    if (__GmLeft.cState) {
        cCoor = 0;
    }
    /*
     *  �趨��������
    
     */
    if((__GucMouseState==__TURNRIGHT)||(__GucMouseState==__TURNLEFT))
    {
        __GmLeft.uiPulseCtr = 10000;       //1182(34mm) //7600
        __GmRight.uiPulseCtr = 10000;      //7600
        cL = 1;
        cR = 1;
        if((__GucDistance[__FRONT]!=0)||((__GucDistance[ __LEFT] & 0x01) == 0)||((__GucDistance[__RIGHT] & 0x01) == 0)){
          if(__GucDistance[__FRONT]!=0)
            GuiTpusle_LR = 1600;  //2200
          else
            GuiTpusle_LR =1600;   //2000
        }
        else{
          GuiTpusle_LR =0; 
        }
    }
    else{
      GuiTpusle_LR =0;  //0
    }
    __GucMouseState   = __GOAHEAD;
    __GiMaxSpeed      =   SEARCHSPEED;       
    __GmRight.uiPulse =   MAZETYPE * ONEBLOCK;
    __GmLeft.uiPulse  =   MAZETYPE * ONEBLOCK;
    __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
     GuiSpeedCtr=__SPEEDUP;
    while (__GmLeft.cState != __MOTORSTOP) {
      
        if (__GmLeft.uiPulseCtr >= ONEBLOCK) {                          /*  �ж��Ƿ�����һ��            */
            cj++;
            __GmLeft.uiPulse    -= ONEBLOCK;
            __GmLeft.uiPulseCtr -= ONEBLOCK;
            if (cCoor) {
             GuiTpusle_LR =0;
              if((__GucDistance[__FRONT])&&(__GucDistance[ __LEFT] & 0x01)&&(__GucDistance[__RIGHT] & 0x01)){
                GucFrontNear=1;
                goto End;
              }
              if(!cL)
              {
                  if ((__GucDistance[ __LEFT] & 0x01) == 0) {                 /*  �����֧·����������        */            
                    __GmRight.uiPulse = __GmRight.uiPulseCtr+bug_LR;
                    __GmLeft.uiPulse  = __GmLeft.uiPulseCtr+bug_LR;
                    while ((__GucDistance[ __LEFT] & 0x01) == 0) {
                     
                        if ((__GmLeft.uiPulseCtr + 600) > __GmLeft.uiPulse) {
                          flag_bug=0;//ԭ��ת��־
                          GucFrontNear=0;
                            goto End;
                        }
                    }
                    __GmRight.uiPulse = MAZETYPE * ONEBLOCK;
                    __GmLeft.uiPulse  = MAZETYPE * ONEBLOCK;
                    GuiSpeedCtr=__SPEEDUP;
                    
                 }
              }
             if(!cR)
             {
                 if ((__GucDistance[__RIGHT] & 0x01) == 0) {                 /*  �ұ���֧·����������        */
                
                    __GmRight.uiPulse = __GmRight.uiPulseCtr+bug_LR;     //3300
                    __GmLeft.uiPulse  = __GmLeft.uiPulseCtr+bug_LR;
                    while ((__GucDistance[ __RIGHT] & 0x01) == 0) {
                     
                        if ((__GmLeft.uiPulseCtr + 600) > __GmLeft.uiPulse) {
                          flag_bug=0;//ԭ��ת��־
                          GucFrontNear=0;
                         //mouseStop();//
                         //while(1);//
                            goto End;
                        }
                    }
                    __GmRight.uiPulse = MAZETYPE * ONEBLOCK;
                    __GmLeft.uiPulse  = MAZETYPE * ONEBLOCK;
                    GuiSpeedCtr=__SPEEDUP;
                }
             }
             
              __mouseCoorUpdate();                                    /*  ��������                    */
            } else {
                cCoor = 1;
            }
        }
        if (__GmRight.uiPulseCtr >= ONEBLOCK) {                         /*  �ж��Ƿ�����һ��            */
            __GmRight.uiPulse    -= ONEBLOCK;
            __GmRight.uiPulseCtr -= ONEBLOCK;
        }
        
        if (cL) {                                                       /*  �Ƿ����������            */
          if(cj){GuiTpusle_LR =0;}
            if ((__GucDistance[ __LEFT] & 0x01) == 0) {                 /*  �����֧·����������        */            
                __GmRight.uiPulse = __GmRight.uiPulseCtr +3000 - GuiTpusle_LR;      //1500
                __GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 3000 - GuiTpusle_LR;
                while ((__GucDistance[ __LEFT] & 0x01) == 0) {
                 
                    if ((__GmLeft.uiPulseCtr + 600) > __GmLeft.uiPulse) {
                      GucFrontNear=0;
                        goto End;                       
                    }
                }
                __GmRight.uiPulse = MAZETYPE * ONEBLOCK;
                __GmLeft.uiPulse  = MAZETYPE * ONEBLOCK;
                GuiSpeedCtr=__SPEEDUP;              
            }
        } else {                           
          /*  �����ǽʱ��ʼ���������  */
            if ( __GucDistance[ __LEFT] & 0x01) {
                cL = 1;
            }
        }
        if (cR) {                                                       /*  �Ƿ��������ұ�            */
            if(cj){GuiTpusle_LR =0;}
            if ((__GucDistance[__RIGHT] & 0x01) == 0) {                 /*  �ұ���֧·����������        */
                __GmRight.uiPulse = __GmRight.uiPulseCtr + 3000 - GuiTpusle_LR;     //3300
                __GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 3000 - GuiTpusle_LR;
                while ((__GucDistance[ __RIGHT] & 0x01) == 0) {
                 
                    if ((__GmLeft.uiPulseCtr + 600) > __GmLeft.uiPulse) {
                      GucFrontNear=0;
                     //mouseStop();//
                     //while(1);//
                      
                        goto End;                      
                    }
                }
                __GmRight.uiPulse = MAZETYPE * ONEBLOCK;
                __GmLeft.uiPulse  = MAZETYPE * ONEBLOCK;
                GuiSpeedCtr=__SPEEDUP;
            }
        } else {
            if ( __GucDistance[__RIGHT] & 0x01) {                       /*  �ұ���ǽʱ��ʼ�������ұ�  */
                cR = 1;
            }
        }
    }
End:   
        __mouseCoorUpdate();                                            /*  ��������                    */
    
}

/*********************************************************************************************************
** Function name:       Go_one_grid
** Descriptions:        ��ʱ�ж�ɨ�衣
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void Go_one_grid()
{
    __GmSPID.sRef=36;
    __GiMaxSpeed=36;
    __GucMouseState   = __GOAHEAD;
    __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
    __GmRight.uiPulse = ONEBLOCK;
    __GmLeft.uiPulse  = ONEBLOCK;
    __GmLeft.uiPulseCtr = 0;      
    __GmRight.uiPulseCtr = 0;  
    while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
    while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
    __mouseCoorUpdate();                                            /*  ��������                    */
}


/*********************************************************************************************************
** Function name:       SysTick_ISR
** Descriptions:        ��ʱ�ж�ɨ�衣
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void SysTick_ISR(void)
{
  __irCheck();
}

/*********************************************************************************************************
** Function name:       __irSendFreq
** Descriptions:        ���ͺ����ߡ�
** input parameters:    __uiFreq:  �����ߵ���Ƶ��
**                      __cNumber: ѡ����Ҫ���õ�PWMģ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __irSendFreq (uint32  __uiFreq, int8  __cNumber)
{
   __uiFreq = SysCtlClockGet() / __uiFreq;
    switch (__cNumber) {

    case 1:                                                             /*ǰ������*/
        PWMGenPeriodSet(PWM_BASE, PWM_GEN_1, __uiFreq);                   /*  ����PWM������1������        */
        PWMPulseWidthSet(PWM_BASE, PWM_OUT_2, __uiFreq / 2);              /*  ����PWM2�����������      */
        PWMOutputState(PWM_BASE,PWM_OUT_2_BIT,true);                  /*  ʹ��PWM2              */                    
        break;

    case 2:                                                            /*���Һ���*/
        PWMGenPeriodSet(PWM_BASE, PWM_GEN_1, __uiFreq);                   /*  ����PWM������1������        */
        PWMPulseWidthSet(PWM_BASE, PWM_OUT_3, __uiFreq / 2);              /*  ����PWM3�����������      */
        PWMOutputState(PWM_BASE,PWM_OUT_3_BIT,true);                   /*  ʹ��PWM3              */  
        break;
    case 3:                                                            /*б45����*/
        TimerLoadSet(TIMER1_BASE, TIMER_B, __uiFreq);                     //    ����TimerB��ֵ 
        TimerMatchSet(TIMER1_BASE, TIMER_B, __uiFreq / 2);                 //   ����TimerB�� PWMƥ��ֵ ,��ռ�ձȵ����ⷢ��ǿ��
        TimerEnable(TIMER1_BASE, TIMER_B);
        break;
    default:
        break;
    }
}


/*********************************************************************************************************
** Function name:       __irCheck
** Descriptions:        �����ߴ�������⡣
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __irCheck (void)
{
    static uint8 ucState = 0;
    static uint8 ucIRCheck,ucIRCheck1;
    
    switch (ucState) {

    case 0:
        __irSendFreq(GusFreq_X, 3);                                          /*  ����б45�Ƚ��ϵĴ��������      */
       __irSendFreq(GusFreq_L, 2);                                         /*  ̽�������������    30000        */
        break;
        
    case 1:
        ucIRCheck1 = GPIOPinRead(GPIO_PORTB_BASE, 0x30);                 /*  ��ȡб45�ȴ�����״̬         */
        ucIRCheck = GPIOPinRead(GPIO_PORTD_BASE, 0x48);                   /*  ��ȡ���Ҵ�����״̬         */
       TimerDisable(TIMER1_BASE, TIMER_B);                            /*  ��ֹб45�Ⱥ���         */
        PWMOutputState(PWM_BASE,PWM_OUT_3_BIT,false);                  /*  ��ֹPWM3�����Һ��⣩     */        
        if (ucIRCheck1 & __FRONTSIDE_R) {
            __GucDistance[__FRONTR]  = 0x00;
        } else {
            __GucDistance[__FRONTR]  = 0x01;
        }
        if (ucIRCheck1 & __FRONTSIDE_L) {
            __GucDistance[__FRONTL]  = 0x00;
        } else {
            __GucDistance[__FRONTL]  = 0x01;
        }
        
        if (ucIRCheck & __RIGHTSIDE) {
            __GucDistance[__RIGHT]  &= 0xfd;
        } else {
            __GucDistance[__RIGHT]  |= 0x02;
        }
        if (ucIRCheck & __LEFTSIDE) {
            __GucDistance[__LEFT]   &= 0xfd;
        } else {
            __GucDistance[__LEFT]   |= 0x02;
        }
        break;    
    case 2:
        __irSendFreq(GusFreq_F, 1);                                       /*  �������ǰ����Զ�� 36000 */
        break;
        
    case 3:
       ucIRCheck = GPIOPinRead(GPIO_PORTD_BASE, 0x84);                    /*  ��ȡǰ��������״̬      */
       PWMOutputState(PWM_BASE,PWM_OUT_2_BIT,false);                     /*  ��ֹPWM2(ǰ������)     */
        if ((ucIRCheck & __FRONT_R)||(ucIRCheck & __FRONT_L)) {
            __GucDistance[__FRONT] &= 0xfe;                               /*  ǰ���޵��� */
            GucMouseStart = 0;
        } else {
            __GucDistance[__FRONT] |= 0x01;                               /*  ǰ�����ڵ���  */
            GucMouseStart = 1;
        }
        break;
        
    case 4:
        __irSendFreq(GusFreq_LF, 2);                                         /*  ������ҷ���Զ��  34000*/
        break;
        
    case 5:
       ucIRCheck = GPIOPinRead(GPIO_PORTD_BASE, 0x48);                  /*  ��ȡ���Ҵ�����״̬        */
       PWMOutputState(PWM_BASE,PWM_OUT_3_BIT,false);                  /*  ��ֹPWM3�����Һ��⣩     */
        if (ucIRCheck & __RIGHTSIDE) {
            __GucDistance[__RIGHT] &= 0xfe;
        } else {
            __GucDistance[__RIGHT] |= 0x01;
        }
        if (ucIRCheck & __LEFTSIDE) {
            __GucDistance[__LEFT]  &= 0xfe;
        } else {
            __GucDistance[__LEFT]  |= 0x01;
        }
        break;
        
     case 6:
        __irSendFreq(GusFreq_FJ, 1);                                       /*  �������ǰ�������  */
        break;
        
     case 7:
       ucIRCheck = GPIOPinRead(GPIO_PORTD_BASE, 0x84);                    /*  ��ȡǰ��������״̬      */
       PWMOutputState(PWM_BASE,PWM_OUT_2_BIT,false);                     /*  ��ֹPWM2(ǰ������)     */
        if ((ucIRCheck & __FRONT_R)||(ucIRCheck & __FRONT_L)) {
            __GucDistance[__FRONT] &= 0xfd;                               /*  ǰ���޵��� */
            GucFrontJinju = 0;
        } else {
            __GucDistance[__FRONT] |= 0x02;                               /*  ǰ�����ڵ���  */
            GucFrontJinju = 1;
        }
        break;
    default:
        break;
    }
    ucState = (ucState + 1) % 8;                                        /*  ѭ�����                    */
}
void mouseStop(void)
{   
  __GmSPID.sRef=0;
  __GmWPID.sRef=0;
  GuiSpeedCtr=5;  
}
/****************************************************************************************************
** Function name:       mouseTurnright
** Descriptions:        ��ת
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��         ������ת��
*********************************************************************************************************/
void mouseTurnright(void)
{ 
  int16 sRef_tmp=0;
  __GmSPID.sRef=36;
  __GiMaxSpeed=36;
  if(flag_bug==0)
  {  
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
     __GmLeft.uiPulse =3000;            
     __GmLeft.uiPulseCtr=0;
     __GmRight.uiPulse =3000;          
     __GmRight.uiPulseCtr=0;
     __GmWPID.sRef=-18;
     GW=0;
     __GucMouseState   = __TURNRIGHT; 
     __GmRight.cState =__MOTORRUN;        //�üӵ�����״̬
     __GmLeft.cState  =__MOTORRUN;
     GucMouseDir     = (GucMouseDir + 1) % 4;                            /*  ������                    */  
     while(1)
     {
         if(GW>66000)   
         {
           break;
         }                    
     }
     //__mazeInfDebug();
     __GmWPID.sRef=0;     
     __GucMouseState   = __GOAHEAD;
     GuiSpeedCtr=3;
     __GmLeft.uiPulse =2400;    //1200   //2200       
     __GmLeft.uiPulseCtr=0;
     __GmRight.uiPulse =2400;           
     __GmRight.uiPulseCtr=0;
    while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
    while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
       
     __GucMouseState   = __TURNRIGHT;
     GuiSpeedCtr=__SPEEDUP;
     __GmRight.cState = __MOTORSTOP;       
     __GmLeft.cState  = __MOTORSTOP;
     __GmRight.uiPulseCtr = 0;
     __GmLeft.uiPulseCtr = 0;
     //s++;
   if(s==5)
   {stop();}

  }
  else if(flag_bug==1)
  {    
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
     
     __GmLeft.uiPulse =3000;            // 22*32*1024/(12*25*3.14)  R_l=14mm  R_r=14+72=86
     __GmLeft.uiPulseCtr=0;
     __GmRight.uiPulse =3000;           // 135*32*1024/(12*25*3.14)
     __GmRight.uiPulseCtr=0;
     GuiSpeedCtr=3;
     sRef_tmp=__GmSPID.sRef;
     __GmSPID.sRef=0;
     __delay(3000000);
     __GmWPID.sRef=-10;
     GW=0;
     __GucMouseState   = __TURNLEFT; 
     __GmRight.cState = __MOTORRUN;        //�üӵ�����״̬
     __GmLeft.cState  = __MOTORRUN;
     GucMouseDir     = (GucMouseDir + 1) % 4;                            /*  ������                    */
     while(1)
     {
         if(GW>80000)  
         {
           break;
         }                      
     }
     //__mazeInfDebug();
     __GmWPID.sRef=0;
     __delay(3000000);
     __GmSPID.sRef=sRef_tmp;
     __GucMouseState   = __GOAHEAD;
     GuiSpeedCtr=3;
     __GmLeft.uiPulse =3000;         //2200  
     __GmLeft.uiPulseCtr=0;
     __GmRight.uiPulse =3000;           
     __GmRight.uiPulseCtr=0;
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
       
     __GucMouseState   = __TURNRIGHT;
     GuiSpeedCtr=__SPEEDUP;
     __GmRight.cState = __MOTORSTOP;       
     __GmLeft.cState  = __MOTORSTOP;
     __GmRight.uiPulseCtr = 0;
     __GmLeft.uiPulseCtr = 0;
     flag_bug=0;    
  }
}
/*********************************************************************************************************
** Function name:       mouseTurnleft
** Descriptions:        ��ת
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void mouseTurnleft(void)
{
  int16 sRef_tmp=0;
  __GmSPID.sRef=36;
  __GiMaxSpeed=36;
  if(flag_bug==0)
  {
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   
   __GmLeft.uiPulse =3000;            // 22*32*1024/(12*25*3.14)  R_l=14mm  R_r=14+72=86
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =3000;           // 135*32*1024/(12*25*3.14)
   __GmRight.uiPulseCtr=0;
   
   __GmWPID.sRef=18;
   GW=0;
   __GucMouseState   = __TURNLEFT; 
   __GmRight.cState = __MOTORRUN;        //�üӵ�����״̬
   __GmLeft.cState  = __MOTORRUN;
   GucMouseDir     = (GucMouseDir + 3) % 4;                            /*  ������                    */
   while(1)
   {
       if(GW>66000)  
       {
         break;
       }                      
   }
   //__mazeInfDebug();
   __GmWPID.sRef=0;
   __GucMouseState   = __GOAHEAD;
   GuiSpeedCtr=3;
   __GmLeft.uiPulse =2400;//2200       
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =2400;         
   __GmRight.uiPulseCtr=0;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
     
   __GucMouseState   = __TURNLEFT;
   GuiSpeedCtr=__SPEEDUP;
   __GmRight.cState = __MOTORSTOP;       
   __GmLeft.cState  = __MOTORSTOP;
   __GmRight.uiPulseCtr = 0;
   __GmLeft.uiPulseCtr = 0;
      //s++;
   if(s==6)
   {stop();}
   
  }
  else if(flag_bug==1)
  {
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
     
     __GmLeft.uiPulse =3000;            // 22*32*1024/(12*25*3.14)  R_l=14mm  R_r=14+72=86
     __GmLeft.uiPulseCtr=0;
     __GmRight.uiPulse =3000;           // 135*32*1024/(12*25*3.14)
     __GmRight.uiPulseCtr=0;
     GuiSpeedCtr=3;
     sRef_tmp=__GmSPID.sRef;
     __GmSPID.sRef=0;
     __delay(3000000);
     __GmWPID.sRef=10;
     GW=0;
     __GucMouseState   = __TURNLEFT; 
     __GmRight.cState = __MOTORRUN;        //�üӵ�����״̬
     __GmLeft.cState  = __MOTORRUN;
     GucMouseDir     = (GucMouseDir + 3) % 4;                            /*  ������                    */
     while(1)
     {
         if(GW>52000)  
         {
           break;
         }                      
     }
     //__mazeInfDebug();
     __GmWPID.sRef=0;
     __delay(3000000);
     __GmSPID.sRef=sRef_tmp;
     __GucMouseState   = __GOAHEAD;
     GuiSpeedCtr=3;
     __GmLeft.uiPulse =3000;         //2200  
     __GmLeft.uiPulseCtr=0;
     __GmRight.uiPulse =3000;           
     __GmRight.uiPulseCtr=0;
     while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
     while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
       
     __GucMouseState   = __TURNLEFT;
     GuiSpeedCtr=__SPEEDUP;
     __GmRight.cState = __MOTORSTOP;       
     __GmLeft.cState  = __MOTORSTOP;
     __GmRight.uiPulseCtr = 0;
     __GmLeft.uiPulseCtr = 0;
     flag_bug=0;    
  }
}
/*********************************************************************************************************
** Function name:       mouseTurnleft
** Descriptions:        ��ת-������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void mouseTurnright_C(void)
{     
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);     
   __GmLeft.uiPulse =3000;            
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =3000;           
   __GmRight.uiPulseCtr=0;
   __GmWPID.sRef=-20;
   GW=0;
   __GucMouseState   = __TURNRIGHT;   
   __GmRight.cState =__MOTORRUN;        //�üӵ�����״̬
   __GmLeft.cState  =__MOTORRUN;
   GucMouseDir     = (GucMouseDir + 1) % 4;                            /*  ������                    */
   
   while(1)
   {
       if(GW>63000)   
       {
         break;
       }
   }
   //__mazeInfDebug();
   __GmWPID.sRef=0;  
   __GucMouseState   = __GOAHEAD;
   GuiSpeedCtr=3;
   __GmLeft.uiPulse =2800- GuiTpusle_S;          
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =2800- GuiTpusle_S;           
   __GmRight.uiPulseCtr=0;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
     
   __GucMouseState   = __TURNRIGHT;   
   GuiSpeedCtr=__SPEEDUP;
   __GmRight.cState = __MOTORSTOP;       
   __GmLeft.cState  = __MOTORSTOP;
   __GmRight.uiPulseCtr = 0;
   __GmLeft.uiPulseCtr = 0;
   //s++;
   if(s==11)
   {stop();}
}

/*********************************************************************************************************
** Function name:       mouseTurnleft
** Descriptions:        ��ת-������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void mouseTurnleft_C(void)
{
  
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
     
   __GmLeft.uiPulse =3000;            
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =3000;          
   __GmRight.uiPulseCtr=0;
   
   __GmWPID.sRef=20;
   GW=0;
   __GucMouseState   = __TURNLEFT; 
   __GmRight.cState = __MOTORRUN;        //�üӵ�����״̬
   __GmLeft.cState  = __MOTORRUN;
   GucMouseDir     = (GucMouseDir + 3) % 4;                            /*  ������                    */
   while(1)
   {
       if(GW>63000)//85000
       {
         break;
       }
     
   }
   //__mazeInfDebug();
   __GmWPID.sRef=0; 
   __GucMouseState   = __GOAHEAD;
   GuiSpeedCtr=3;
   __GmLeft.uiPulse =2800-GuiTpusle_S;             
   __GmLeft.uiPulseCtr=0;
   __GmRight.uiPulse =2800- GuiTpusle_S;           
   __GmRight.uiPulseCtr=0;
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
     
   __GucMouseState   = __TURNLEFT;
   GuiSpeedCtr=__SPEEDUP;
   __GmRight.cState = __MOTORSTOP;       
   __GmLeft.cState  = __MOTORSTOP;
   __GmRight.uiPulseCtr = 0;
   __GmLeft.uiPulseCtr = 0;
   //s++;
   if(s==1)
   {stop();}
}
/*********************************************************************************************************
** Function name:       mouseTurnback
** Descriptions:        ����ǰ�����࣬��ת180��
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void mouseTurnback(void)
{
    uint8 w=0;
    uint16 q=0;
   if(GucFrontNear)
   {
     while(1)
     {
       if(GucFrontJinju){
         while(GucFrontJinju){
           q++;
           if(q>10){
             w=1;
             break;
           }          
         }
         q=0;
       }
      if(w)
        break;
     }
   }
   else
   {
     __GmRight.uiPulse +=4500;
     __GmLeft.uiPulse +=4500;
    while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
    while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse);
   } 
   
    __GmSPID.sRef=0; 
    __GucMouseState   = __TURNBACK; 
    __delay(1500000); 
    __GmRight.uiPulse =7200;            
    __GmRight.uiPulseCtr=0;
    __GmLeft.uiPulse =7200; 
    __GmLeft.uiPulseCtr=0;
 
    GW=0;
    __GucMouseState   = 6;
    __GmWPID.sRef=16;
    __GmLeft.cState   = __MOTORRUN;
    __GmRight.cState  = __MOTORRUN;
    GucMouseDir = (GucMouseDir + 2) % 4;                                  
    while(1)
   {
       if(GW>162000)  
       {
         break;
       }                        
   }
    //__mazeInfDebug();
    __GmWPID.sRef=0;
    __delay(500000);
    __GmSPID.sRef=28;
    __GmWPID.sRef=0;
    GuiSpeedCtr=__SPEEDUP;
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.uiPulse =6200;            
    __GmRight.uiPulseCtr=0;
    __GmLeft.uiPulse =6200; 
    __GmLeft.uiPulseCtr=0;
    while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
    while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse); 
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;

}
/*********************************************************************************************************
** Function name:       mouseTurnback_Y 
** Descriptions:        ԭ����ת180��
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void mouseTurnback_Y(void)
{
    __GmRight.uiPulse +=1500;
    __GmLeft.uiPulse +=1500;
   
   while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);    
   while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse); 
   __GmSPID.sRef=0;  
    __GucMouseState   = __TURNBACK;    
    __delay(1500000);
    __GmRight.uiPulse =5000;             
    __GmRight.uiPulseCtr=0;
    __GmLeft.uiPulse =5000; 
    __GmLeft.uiPulseCtr=0;
    GW=0;
     __GucMouseState   = 6;
    __GmWPID.sRef=16;
    __GmLeft.cState   = __MOTORRUN;
    __GmRight.cState  = __MOTORRUN;
    GucMouseDir = (GucMouseDir + 2) % 4;                                  
    while(1)
   {
       if(GW>152000)   
       {
         break;
       }                        
   }
    __GmSPID.sRef=10;
    __GmWPID.sRef=0;
    __delay(500000);
    GuiSpeedCtr=__SPEEDUP;
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
}
/*********************************************************************************************************
** Function name:       __mouseCoorUpdate
** Descriptions:        ���ݵ�ǰ�����������ֵ
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __mouseCoorUpdate (void)
{
    switch (GucMouseDir) {

    case 0:
        GmcMouse.cY++;
        break;

    case 1:
        GmcMouse.cX++;
        break;

    case 2:
        GmcMouse.cY--;
        break;

    case 3:
        GmcMouse.cX--;
        break;

    default:
        break;
    }
    //__mazeInfDebug();
    __wallCheck();
//    UARTCharPut(UART0_BASE,GmcMouse.cX);
//    UARTCharPut(UART0_BASE, GmcMouse.cY);
//    UARTCharPut(UART0_BASE, 0xff); 
}


/*********************************************************************************************************
** Function name:       __wallCheck
** Descriptions:        ���ݴ�����������ж��Ƿ����ǽ��
** input parameters:    ��
** output parameters:   ��
** Returned value:      cValue: ����λ������һ�δ�����ǰ�ҡ�1Ϊ��ǽ��0Ϊûǽ��
*********************************************************************************************************/
void __wallCheck (void)
{
    uint8 ucMap = 0;
    uint8 uctemp = 0;
    ucMap |= MOUSEWAY_B;
    
    if (__GucDistance[__LEFT]  & 0x01) {
        ucMap &= ~MOUSEWAY_L;
        uctemp |= 0x06;
    }else {
        ucMap |=  MOUSEWAY_L;
    }
    if (__GucDistance[__FRONT] & 0x01) {
        ucMap &= ~MOUSEWAY_F;
         uctemp |= 0x40;
    }else {
        ucMap |=  MOUSEWAY_F;
    }
    if (__GucDistance[__RIGHT] & 0x01) {
        ucMap &= ~MOUSEWAY_R;
         uctemp |= 0x30;
    }else {
        ucMap |=  MOUSEWAY_R;
    }
    
    
    GucMapBlock0[GmcMouse.cX][GmcMouse.cY]=ucMap;
    GucMapBlock[GmcMouse.cX][GmcMouse.cY]=ucMap;
    GucMapBlock1[GmcMouse.cX][GmcMouse.cY]=ucMap;
    if(GmcMouse.cY<(MAZETYPE-1))
        {GucMapBlock1[GmcMouse.cX][GmcMouse.cY+1] &= ~(((~ucMap)&0x01)*4);}       /*����������Χ����ǽ�����ϸ���  ע����ˮ��*/
       if(GmcMouse.cX<(MAZETYPE-1))
        { GucMapBlock1[GmcMouse.cX+1][GmcMouse.cY]&= ~(((~ucMap)&0x02)*4);}
         if(GmcMouse.cY>0)
         {GucMapBlock1[GmcMouse.cX][GmcMouse.cY-1]&= ~(((~ucMap)&0x04)/4);}
        if(GmcMouse.cX>0)
         {GucMapBlock1[GmcMouse.cX-1][GmcMouse.cY]&= ~(((~ucMap)&0x08)/4);}
              
      if(GmcMouse.cY<(MAZETYPE-1))
         {GucMapBlock[GmcMouse.cX][GmcMouse.cY+1] |=    ((ucMap&0x01)*4);}        /*����������Χ����ǽ�����ϸ���  ע���ڳ�ʼΪ��ǽʱ����*/
      if(GmcMouse.cX<(MAZETYPE-1))
         { GucMapBlock[GmcMouse.cX+1][GmcMouse.cY]|=  ((ucMap&0x02)*4);}
        if(GmcMouse.cY>0)
         {GucMapBlock[GmcMouse.cX][GmcMouse.cY-1]|=  ((ucMap&0x04)/4);}
        if(GmcMouse.cX>0)
          {GucMapBlock[GmcMouse.cX-1][GmcMouse.cY]|=  ((ucMap&0x08)/4);}
    
    zlg7289Download(2, 2, 0, uctemp);
}
/*********************************************************************************************************
** Function name:       SensorDebug
** Descriptions:        ���������ʾ��������״̬���������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void sensorDebug (void)
{
    zlg7289Download(2, 0, 0, __GucDistance[__LEFT  ]);
    zlg7289Download(2, 1, 0, __GucDistance[__FRONTL]);
    zlg7289Download(2, 2, 0, __GucDistance[__FRONT ]);
    zlg7289Download(2, 3, 0, __GucDistance[__FRONTR]);    
    zlg7289Download(2, 4, 0, __GucDistance[__RIGHT ]);
}


/*********************************************************************************************************
** Function name:       __mazeInfDebug
** Descriptions:        ���������ʾ����ǰ������ǰ�����������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __mazeInfDebug (void)
{
    /*
     *  ��ʾ����
     */
    switch (GucMouseDir) {
        
    case 0:
        zlg7289Download(2, 3, 0, 0x47);                                /*  ��ǰ����F��ʾ               */
        break;
        
    case 1:
        zlg7289Download(2, 3, 0, 0x77);                                /*  ���ң���R��ʾ               */
        break;
        
    case 2:
        zlg7289Download(2, 3, 0, 0x1f);                                /*  �����b��ʾ               */
        break;
        
    case 3:
        zlg7289Download(2, 3, 0, 0x0e);                                /*  ������L��ʾ               */
        break;
        
    default :
        zlg7289Download(2, 3, 0, 0x4f);                                /*  ������E��ʾ               */
        break;
    }
    /*
     *  ��ʾ����
     */
    zlg7289Download(1, 0, 0, GmcMouse.cX / 10);
    zlg7289Download(1, 1, 0, GmcMouse.cX % 10);
    zlg7289Download(1, 6, 0, GmcMouse.cY / 10);
    zlg7289Download(1, 7, 0, GmcMouse.cY % 10);
}

/*********************************************************************************************************
** Function name:       startCheck
** Descriptions:        ��ȡ����
** input parameters:    ��
** output parameters:   ��
** Returned value:      true:  �����Ѱ���
**                      false: ����δ����
*********************************************************************************************************/
uint8 startCheck (void)
{
    if (GPIOPinRead(GPIO_PORTC_BASE, __START) == 0) {
        __delay(50);
        while(GPIOPinRead(GPIO_PORTC_BASE, __START) == 0);
        return(true);
    }else {
        return(false);
    }
}
/*********************************************************************************************************
** Function name:       keyCheck
** Descriptions:        ��ȡ����
** input parameters:    ��
** output parameters:   ��
** Returned value:      true:  �����Ѱ���
**                      false: ����δ����
*********************************************************************************************************/
uint8 keyCheck (void)
{
    if (GPIOPinRead(GPIO_PORTC_BASE, __KEY) == 0) {
        __delay(50);
        while(GPIOPinRead(GPIO_PORTC_BASE, __KEY) == 0);
        return(true);
    }else {
        return(false);
    }
}
/*********************************************************************************************************
** Function name:       mouseInit
** Descriptions:        ��LM3S615���������г�ʼ��
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void mouseInit (void)
{
    SysCtlClockSet( SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                    SYSCTL_XTAL_6MHZ );                                 /*  ʹ��PLL��50M                */
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOA );
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOB );                      /*  ʹ��GPIO B������            */
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOC );                      /*  ʹ��GPIO C������            */
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOD );                      /*  ʹ��GPIO D������            */
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOE );                      /*  ʹ��GPIO E������            */
    PIDInit();                                                          /*  PID��ʼ��                  */
    __keyInit();                                                        /*  ������ʼ��                  */
    __sensorInit();                                                     /*  ��������ʼ��                */
    __MotorIint();                                                      /*  ֱ��������Ƴ�ʼ��          */
    __sysTickInit();                                                    /*  ϵͳʱ�ӳ�ʼ��              */
    __UART0Init ();
    MPU6050_Init();
    GPIODirModeSet(GPIO_PORTE_BASE, __DIR1, GPIO_DIR_MODE_IN);
    GPIODirModeSet(GPIO_PORTD_BASE, __DIR2, GPIO_DIR_MODE_IN);
    
    GPIODirModeSet(GPIO_PORTC_BASE,  GPIO_PIN_4, GPIO_DIR_MODE_OUT);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00);
    GucMapBlock[0][0] = 0x01;
    GucMapBlock0[0][0] = 0x01;
}
/*********************************************************************************************************
** Function name:       __sensorInit
** Descriptions:        ���������Ƴ�ʼ��
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __sensorInit (void)
{
     /*
     *  �������ӵ��������ź�����ŵ�I/O��Ϊ����ģʽ
     */
    GPIODirModeSet(GPIO_PORTB_BASE,
                   __FRONTSIDE_L |
                   __FRONTSIDE_R,  
                   GPIO_DIR_MODE_IN);
    GPIODirModeSet(GPIO_PORTD_BASE,
                   __RIGHTSIDE | __FRONT_L | __LEFTSIDE | __FRONT_R,  
                   GPIO_DIR_MODE_IN);
    /*
     *  ��PWM���������߷���ͷ�������Ƶĺ������ź�
     */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM);                          /*  ʹ��PWMģ��                 */
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);                                 /*  PWMʱ�����ã�����Ƶ         */

    GPIOPinTypePWM(GPIO_PORTB_BASE, __IRSEND1_FRONT | __IRSEND3_LR);    /*  PB0,PB1����ΪPWM����            */
    PWMGenConfigure(PWM_BASE, PWM_GEN_1,                                /*  ����PWM������1              */
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);       /*  �Ӽ�������������            */

    PWMOutputState(PWM_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, false);      /*  ��ֹPWM2,PWM3���                */
    PWMGenEnable(PWM_BASE, PWM_GEN_1);
    
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);     //   ʹ��Timer1ģ�� 
    GPIOPinTypeTimer(GPIO_PORTC_BASE, __IRSEND2_BEVEL);     //   ����CCP3 �ܽ�ΪPWM ���      
    TimerConfigure(TIMER1_BASE, TIMER_CFG_16_BIT_PAIR |  //   ����TimerΪ˫16λ PWM 
                    TIMER_CFG_B_PWM); 
    TimerDisable(TIMER1_BASE, TIMER_B);     //   ��ֹTimer������ PWM��ʼ���
    TimerControlLevel(TIMER1_BASE, TIMER_B, true);   //   ����PWM ������� 
}


/*********************************************************************************************************
** Function name:       __MotorIint
** Descriptions:        ������Ƴ�ʼ��
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __MotorIint (void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM);                          /*  ʹ��PWMģ��                 */
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);  
    GPIOPinTypePWM(GPIO_PORTD_BASE, __D1_1 | __D1_2);                       /*  PD0(D1_1),PD1(D1_2)����ΪPWM���� */
    GPIOPinTypePWM(GPIO_PORTE_BASE, __D2_1 | __D2_2);                       /*  PE0(D2_1),PE1(D2_2)����ΪPWM����  */
    PWMGenConfigure(PWM_BASE, PWM_GEN_0,                                /*  ����PWM������0              */
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);       /*  �Ӽ�������������            */
    PWMGenConfigure(PWM_BASE, PWM_GEN_2,                                /*  ����PWM������2              */
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);       /*  �Ӽ�������������            */
    PWMOutputState(PWM_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT | PWM_OUT_4_BIT | PWM_OUT_5_BIT , true);     /*ʹ��PWM0,1,4,5���  */
    PWMGenDisable(PWM_BASE, PWM_GEN_0);                                 /*  ��ֹPWM������0       */
    PWMGenDisable(PWM_BASE, PWM_GEN_2);                                 /*  ��ֹPWM������2       */
    PWMGenPeriodSet(PWM_BASE, PWM_GEN_0, 2000);                   /*  ����PWM������0������  =50M/f  25KHz  3500 3400 3300 2000*/
    PWMGenPeriodSet(PWM_BASE, PWM_GEN_2, 2000);                   /*  ����PWM������2������  =50M/f  25KHz*/
    
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);                       /*  ʹ�ܶ�ʱ��2ģ��             */
    TimerConfigure(TIMER2_BASE, TIMER_CFG_32_BIT_PER);                  /*  ����Ϊ32λ���ڼ���ģʽ      */
    TimerLoadSet(TIMER2_BASE, TIMER_A, 50000);                          /*  ���ö�ʱʱ��                */
    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);                    /*  ����Ϊ����ж�              */
    //IntPrioritySet(INT_TIMER2A,2<<5);
    IntEnable(INT_TIMER2A);                                             /*  ʹ�ܶ�ʱ��2�ж�             */
    TimerEnable(TIMER2_BASE, TIMER_A);                                  /*  ʹ�ܶ�ʱ��2                 */
    
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);                         //ʹ��Timer0ģ��
    GPIOPinTypeTimer(GPIO_PORTD_BASE,__SIG_M2_B);                         //����D�ڵ�4Ϊ��������
    GPIOPinTypeTimer(GPIO_PORTE_BASE,__SIG_M1_B);                         //����E�ڵ�3Ϊ��������
    TimerConfigure(TIMER0_BASE, TIMER_CFG_16_BIT_PAIR | TIMER_CFG_A_CAP_COUNT | TIMER_CFG_B_CAP_COUNT);// ����Timer0Ϊ16λ�¼�������
    TimerControlEvent(TIMER0_BASE,TIMER_BOTH,TIMER_EVENT_POS_EDGE);       // ����Timer0A����CCP������
    TimerLoadSet(TIMER0_BASE, TIMER_BOTH, 65535);                          // ���ü�������ֵ
    TimerMatchSet(TIMER0_BASE, TIMER_BOTH, 0);                             // �����¼�����ƥ��ֵ
    TimerIntEnable(TIMER0_BASE, TIMER_CAPA_MATCH);                        // ʹ��Timer0A����ƥ���ж�
    TimerIntEnable(TIMER0_BASE, TIMER_CAPB_MATCH);                        // ʹ��Timer0B����ƥ���ж�
    IntEnable(INT_TIMER0A);                                                // ʹ��Timer0A�ж�
    IntEnable(INT_TIMER0B);                                               // ʹ��Timer0B�ж�
    TimerEnable(TIMER0_BASE, TIMER_BOTH);                                // ʹ��Timer0AB����
}
/*********************************************************************************************************
** Function name:       __keyInit
** Descriptions:        �����Ӱ�����GPIO�ڳ�ʼ��
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __keyInit (void)
{
    GPIODirModeSet(GPIO_PORTC_BASE, __KEY, GPIO_DIR_MODE_IN);           /*  ���ð�����Ϊ����            */   
    GPIODirModeSet(GPIO_PORTA_BASE, ZLG7289_KEY, GPIO_DIR_MODE_IN);     /*  ����KEY�˿�Ϊ����           */
    GPIODirModeSet(GPIO_PORTC_BASE, __START, GPIO_DIR_MODE_IN);         /*  ���ð�����Ϊ����            */
    GPIOIntTypeSet(GPIO_PORTA_BASE, ZLG7289_KEY, GPIO_FALLING_EDGE);    /*  ���������½��ش����ж�      */
    
    GPIOPinIntEnable(GPIO_PORTA_BASE, ZLG7289_KEY);                     /*  ʹ�����������ж�            */
    IntEnable(INT_GPIOA);  /*  ʹ��GPIO PA���ж�           */       
}
/*********************************************************************************************************
** Function name:       __keyIntDisable
** Descriptions:        ZLG7289�����жϽ��� ����
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void  __keyIntDisable (void) 
{
    GPIOPinIntDisable(GPIO_PORTA_BASE, ZLG7289_KEY);
}
/*********************************************************************************************************
** Function name:       __sysTickInit
** Descriptions:        ϵͳ���Ķ�ʱ����ʼ����
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __sysTickInit (void)
{
    SysTickPeriodSet(SysCtlClockGet() / 1600);                          /*  ���ö�ʱʱ��Ϊ625us           */
    //IntPrioritySet(FAULT_SYSTICK,1<<5);
    SysTickEnable();                                                    /*  ʹ��ϵͳʱ��                */
    SysTickIntEnable();                                                 /*  ʹ��ϵͳʱ���ж�            */
}
/*********************************************************************************************************
** Function name:      __UART0Init
** Descriptions:        �Դ���0���г�ʼ��
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __UART0Init (void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinTypeUART(GPIO_PORTA_BASE,GPIO_PIN_0  | GPIO_PIN_1);    //PA0 PA1����Ϊ����
    UARTConfigSet(UART0_BASE, 115200, (UART_CONFIG_WLEN_8 |  //���ô���0��8λ���ݣ�1λ��ʼλ��1λֹͣλ���û�������
                                       UART_CONFIG_STOP_ONE |
                                       UART_CONFIG_PAR_NONE) & 0xFFFFFFEF);
    //IntEnable(INT_UART0);                                      //ʹ�ܴ���0ϵͳ�ж�
    //UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);      //ʹ�ܴ���0�����жϺͽ��ճ�ʱ�ж� 
    UARTEnable(UART0_BASE);    
}

void GYRO_Z_Angle(void)
{
    uint16 w=0;
    w=GetData(0x47);
    if(w>=32768)
    {
      w=65535-w;
    }
    w=w/16.4;
    GW=GW+w;
}
/*********************************************************************************************************
** Function name:      stop
** Descriptions:        ��ͣ
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void stop()
{
  while(1)
  {
    __delay(40000);
    __GmRight.cState=0;
    __GmWPID.sRef=0;
    __GmSPID.sRef=0;
     GsTpusle_T=0;
    __PIDContr();
  }
  
}
/*********************************************************************************************************
** Function name:       mouseTurnback_correct
** Descriptions:        ԭ��ת�����λ��
** input parameters:    Pulse��������������
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void mouseTurnback_correct(uint16 time)
{
    __GmRight.cState = __CORRECT;       
    __GmLeft.cState  = __CORRECT;
    delayms(time);
  
    __GmSPID.sRef=28;
    __GmWPID.sRef=0;
    GuiSpeedCtr=__SPEEDUP;
    __GmRight.cState = __MOTORSTOP;       
    __GmLeft.cState  = __MOTORSTOP;
    __GmRight.uiPulse =6200;            
    __GmRight.uiPulseCtr=0;
    __GmLeft.uiPulse =6200; 
    __GmLeft.uiPulseCtr=0;
    while ((__GmRight.uiPulseCtr+200 ) <= __GmRight.uiPulse);
    while ((__GmLeft.uiPulseCtr+200 ) <= __GmLeft.uiPulse); 
    __GmRight.sSpeed = 0;
    __rightMotorContr();
    __GmLeft.sSpeed = 0;
    __leftMotorContr();
    __GmRight.uiPulseCtr = 0;
    __GmLeft.uiPulseCtr = 0;
    buffer=0;
   
    
}

/*********************************************************************************************************
** Function name:       startCheck2
** Descriptions:        �������������
** input parameters:    
** output parameters:   ��
** Returned value:      ture=1 false=0
*********************************************************************************************************/
uint8 startCheck2(void)
{
  if (__GucDistance[__FRONT] & 0x01) {
        __delay(50);
        if (__GucDistance[__FRONT] & 0x01) 
        {
          while(__GucDistance[__FRONT] & 0x01);
          __delay(10000000);
          return(true);
        }
        else{return(false);}
    }else {
        return(false);
    }
}
/*********************************************************************************************************
** Function name:       putstring
** Descriptions:        ����һ���ַ�
** input parameters:    
** output parameters:   ��
** Returned value:      
*********************************************************************************************************/
void putstring(uint8 *string)
{
  while(*string!='\0')
  {
     UARTCharPut(UART0_BASE, *string);
     string++;
  }
}


/*********************************************************************************************************
** Function name:       delayms
** Descriptions:        ��ʱ����
** input parameters:    
** output parameters:   ��
** Returned value:      
*********************************************************************************************************/
#define ewarm
#if defined(ewarm)
static void
SysCtlDelay2(unsigned long ulCount)
{
    __asm("    subs    r0, #1\n"
          "    bne.n   SysCtlDelay2\n"
          "    bx      lr");
}
#endif

void delayms(uint16 ms)
{
  SysCtlDelay2(ms * (SysCtlClockGet() / 3000));
}
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/





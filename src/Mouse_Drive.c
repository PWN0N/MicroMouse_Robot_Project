/****************************************Copyright (c)****************************************************
**                               Guangzhou ZHIYUAN electronics Co.,LTD.
**                                     
**                                 http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:           Mouse_Drive.c
** Last modified Date: 
** Last Version: 
** Description:         电脑鼠底层驱动
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
** Description:                          在中断中stop
**
*********************************************************************************************************/


/*********************************************************************************************************
  包含头文件
*********************************************************************************************************/
#include "Mouse_Drive.h"
#include "Maze.h"
/*********************************************************************************************************
  定义全局变量
*********************************************************************************************************/
MAZECOOR          GmcMouse                        = {0,0};              /*  保存电脑鼠当前位置坐标      */
uint8             GucMouseDir                     = UP;                 /*  保存电脑鼠当前方向          */
uint8             GucMapBlock[MAZETYPE][MAZETYPE] = {0};                /*  GucMapBlock[x][y]           */
                                                                        /*  x,横坐标;y,纵坐标;          */
uint8             GucMapBlock0[MAZETYPE][MAZETYPE] = {0};               /*  中心算法         */
                                                                   /*  bit3~bit0分别代表左下右上   */
                                                                        /*  0:该方向无路，1:该方向有路  */
uint8             GucMapBlock1[MAZETYPE][MAZETYPE]= {0x0f};
uint8             GucMouseStart                    = 0;                 /* 电脑鼠启动        */
uint8             GucFrontJinju                    = 0;                 /* 前方红外近距在搜索时等高图制作中用   */
uint8             GucCrossroad                     = 0;                 /* 十字路口数，冲刺时用，若十字路口多降低最高冲刺速度   */
static uint32     GW;                                                       /*小车转动角度*/

static __MOTOR  __GmLeft                          = {0, 0, 0, 0, 0, 0};    /*  定义并初始化左电机状态      */
static __MOTOR  __GmRight                         = {0, 0, 0, 0, 0, 0};    /*  定义并初始化右电机状态      */
static __PID    __GmLPID;                                                 /*  定义左电机PID      */
static __PID    __GmRPID;                                                 /*  定义右电机PID     */
static __PID    __GmSPID;                                                 /*  直线PID     */
static __PID    __GmWPID;                                                 /*  旋转PID     */
static uint8    __GucMouseState                   = __STOP;             /*  保存电脑鼠当前运行状态      */
static int32    __GiMaxSpeed                      = SEARCHSPEED;        /*  保存允许运行的最大速度      */
static uint8    __GucDistance[5]                  = {0};                /*  记录传感器状态              */
uint16   GusFreq_F                         = 36200;   //33.8,33,327        /*  前方红外频率              */
uint16   GusFreq_FJ                        = 19200;   //26.3,266,275              /*  前方近距红外频率              */
uint16   GusFreq_X                         = 30000;   //35,33.8          /*  斜45度红外频率              */
uint16   GusFreq_LF                        = 31700;   //34000           /*  左右红外远距频率              */
uint16   GusFreq_L                         = 18300;              /*  左右红外近距频率              */
static  int16   GsTpusle_T                       = 0;                  /*  左轮校正减少的速度值              */
static uint8    GuiSpeedCtr                       = 0;
static int16   GuiTpusle_LR                      = 0;
static int16   GuiTpusle_back_LR                 =0;
static int16   GuiTpusle_S                       = 0;
static uint8    GucFrontNear                      = 0;
uint8    GucFangXiang                      = 0;
uint8    GucDirTemp                        = 0;
uint32    DIS[10]                           = {0};
uint8 Tab=0;//红外调试选项

uint16 buffer=0; //速度增量
uint8 s=0;  //UARTCharPut(UART0_BASE, s++); 调试用
uint8 flag_bug=0;  //出bug时标志位
uint8 flag_bug1=0;

/*********************************************************************************************************
** Function name:       __delay
** Descriptions:        延时函数
** input parameters:    uiD :延时参数，值越大，延时越久
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __delay (uint32  uiD)
{
    for (; uiD; uiD--);
}
/*********************************************************************************************************
** Function name:       PIDInit
** Descriptions:        PID初始化
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void PIDInit(void) 
{  
    __GmLPID.usEncoder_new = 0;
    __GmLPID.usEncoder_last = 65535;
    __GmLPID.usFeedBack = 0 ;  //速度反馈值
    __GmLPID.sFeedBack = 0 ;
    
    __GmRPID.usEncoder_new = 0;
    __GmRPID.usEncoder_last = 65535;
    __GmRPID.usFeedBack = 0 ;  //速度反馈值
    __GmRPID.sFeedBack = 0 ;
    
    __GmSPID.sRef = 0 ;        //速度设定值 
    __GmSPID.sFeedBack = 0 ;        
    __GmSPID.sPreError = 0 ;   //前一次，速度误差,,vi_Ref - vi_FeedBack 
    __GmSPID.sPreDerror = 0 ;   //前一次，速度误差之差，d_error-PreDerror; 
        
    __GmSPID.fKp = __KP; 
    __GmSPID.fKi = __KI;
    __GmSPID.fKd = __KD; 
       
    __GmSPID.iPreU = 0 ;      //电机控制输出值 
    
    __GmWPID.sRef = 0 ;        //速度设定值 
    __GmWPID.sFeedBack = 0 ;       
    __GmWPID.sPreError = 0 ;   //前一次，速度误差,,vi_Ref - vi_FeedBack 
    __GmWPID.sPreDerror = 0 ;   //前一次，速度误差之差，d_error-PreDerror; 
    
    __GmWPID.fKp = __KP;  //30
    __GmWPID.fKi = __KI;  //0.1,0.01
    __GmWPID.fKd = __KD; 
       
    __GmWPID.iPreU = 0 ;      //电机控制输出值 
    
}
/*********************************************************************************************************
** Function name:       __Encoder
** Descriptions:        采集编码器输出的脉冲
** input parameters:    无
** output parameters:   无
** Returned value:      无
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
** Descriptions:        直线PID控制
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __SPIDContr(void) 
{ 
    int16  error,d_error,dd_error;
    static uint8   K_I=1;
    error = __GmSPID.sRef - __GmSPID.sFeedBack; // 偏差计算
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
    
    __GmSPID.sPreError = error; //存储当前偏差 
    __GmSPID.sPreDerror = d_error;
    
    __GmSPID.iPreU += (int16)(  __GmSPID.fKp * d_error + K_I*__GmSPID.fKi * error  + __GmSPID.fKd*dd_error); 
}
/*********************************************************************************************************
** Function name:       __WPIDContr
** Descriptions:        旋转方向PID控制
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __WPIDContr(void) 
{ 
    int16  error,d_error,dd_error; 
    static uint8   K_I=1;
    error = __GmWPID.sRef + GsTpusle_T- __GmWPID.sFeedBack; // 偏差计算 
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
    
    __GmWPID.sPreError = error; //存储当前偏差 
    __GmWPID.sPreDerror = d_error;
    __GmWPID.iPreU += (int16)(  __GmWPID.fKp * d_error + K_I*__GmWPID.fKi * error  + __GmWPID.fKd*dd_error);
        
}
/*********************************************************************************************************
** Function name:      __PIDContr
** Descriptions:        PID控制，通过脉冲数控制电机
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __PIDContr(void)
{
    __SPIDContr();
    __WPIDContr();
    __GmLeft.sSpeed = __GmSPID.iPreU - __GmWPID.iPreU ;
    if(__GmLeft.sSpeed>=0){
     __GmLeft.cDir=__MOTORGOAHEAD; 
    if( __GmLeft.sSpeed >= U_MAX )   //速度PID，防止调节最高溢出 
       __GmLeft.sSpeed = U_MAX;      
    if( __GmLeft.sSpeed <= U_MIN ) //速度PID，防止调节最低溢出  
       __GmLeft.sSpeed = U_MIN;
    }
    else{
      __GmLeft.cDir=__MOTORGOBACK;
      __GmLeft.sSpeed *=-1; 
    if( __GmLeft.sSpeed >= U_MAX )   //速度PID，防止调节最高溢出 
       __GmLeft.sSpeed = U_MAX;      
    if( __GmLeft.sSpeed <= U_MIN ) //速度PID，防止调节最低溢出  
       __GmLeft.sSpeed = U_MIN;
    }
      
    __GmRight.sSpeed = __GmSPID.iPreU + __GmWPID.iPreU ;
    if(__GmRight.sSpeed>=0){
     __GmRight.cDir=__MOTORGOAHEAD; 
    if( __GmRight.sSpeed >= U_MAX )   //速度PID，防止调节最高溢出 
       __GmRight.sSpeed = U_MAX;      
    if( __GmRight.sSpeed <= U_MIN ) //速度PID，防止调节最低溢出  
       __GmRight.sSpeed = U_MIN;
    }
    else{
      __GmRight.cDir=__MOTORGOBACK;
      __GmRight.sSpeed *=-1; 
    if( __GmRight.sSpeed >= U_MAX )   //速度PID，防止调节最高溢出 
       __GmRight.sSpeed = U_MAX;      
    if( __GmRight.sSpeed <= U_MIN ) //速度PID，防止调节最低溢出  
       __GmRight.sSpeed = U_MIN;
    }
    __rightMotorContr();
    __leftMotorContr();
    
}
/*********************************************************************************************************
** Function name:       __rightMotorContr
** Descriptions:        右直流电机驱动
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __rightMotorContr(void)
{
    switch (__GmRight.cDir) 
    {
    case __MOTORGOAHEAD:                                                /*  向前步进                    */
      PWMPulseWidthSet(PWM_BASE, PWM_OUT_5, 0);                         /*  设置PWM5输出的脉冲宽度      */
      PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, __GmRight.sSpeed);          /*  设置PWM4输出的脉冲宽度      */
        break;

    case __MOTORGOBACK:                                                 /*  向后步进                    */
      PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, 0);                         /*  设置PWM4输出的脉冲宽度      */
      PWMPulseWidthSet(PWM_BASE, PWM_OUT_5, __GmRight.sSpeed);          /*  设置PWM5输出的脉冲宽度      */
        break;
    case __MOTORGOSTOP:                                                  /*  反向制动                   */
      PWMPulseWidthSet(PWM_BASE, PWM_OUT_5, 0);                     /*  设置PWM5输出的脉冲宽度      */
      PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, 0);                     /*  设置PWM4输出的脉冲宽度      */
        break;

    default:
        break;
    }
    PWMGenEnable(PWM_BASE, PWM_GEN_2);
}
/*********************************************************************************************************
** Function name:       __leftMotorContr
** Descriptions:        左直流电机驱动
** input parameters:    __GmLeft.cDir :电机运行方向
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __leftMotorContr(void)
{
    switch (__GmLeft.cDir) 
    {
    case __MOTORGOAHEAD:                                                /*  向前步进                    */
      PWMPulseWidthSet(PWM_BASE, PWM_OUT_0, 0);                         /*  设置PWM0输出的脉冲宽度      */
      PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, __GmLeft.sSpeed);          /*  设置PWM1输出的脉冲宽度      */
        break;

    case __MOTORGOBACK:                                                 /*  向后步进                    */
      PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, 0);                         /*  设置PWM1输出的脉冲宽度      */
      PWMPulseWidthSet(PWM_BASE, PWM_OUT_0, __GmLeft.sSpeed);          /*  设置PWM0输出的脉冲宽度      */
        break;
    case __MOTORGOSTOP:                                                  /*  反向制动                   */
      PWMPulseWidthSet(PWM_BASE, PWM_OUT_0, 0);                     /*  设置PWM0输出的脉冲宽度      */
      PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, 0);                     /*  设置PWM1输出的脉冲宽度      */
        break;

    default:
        break;
    }
    PWMGenEnable(PWM_BASE, PWM_GEN_0);
}
/*********************************************************************************************************
** Function name:       __SpeedUp
** Descriptions:        电脑鼠加速程序
** input parameters:    无
** output parameters:   无
** Returned value:      无
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
** Descriptions:        电脑鼠减速程序
** input parameters:    无
** output parameters:   无
** Returned value:      无
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
** Descriptions:        左电机计数器匹配中断
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void Timer0A_ISR(void)
{
   TimerIntClear(TIMER0_BASE, TIMER_CAPA_MATCH);                     /*  清除定时器0A中断。           */
   TimerEnable(TIMER0_BASE, TIMER_A); 
}

/*********************************************************************************************************
** Function name:       Timer0B_ISR
** Descriptions:        右电机计数器匹配中断
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void Timer0B_ISR(void)
{
   TimerIntClear(TIMER0_BASE, TIMER_CAPB_MATCH);                     /*  清除定时器0B中断。           */
   TimerEnable(TIMER0_BASE, TIMER_B); 
}

/*********************************************************************************************************
** Function name:       Timer2A_ISR
** Descriptions:        Timer2中断服务函数
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void Timer2A_ISR(void)
{
    static int8 n = 0,m = 0,t=0,k=0;
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);                     /*  清除定时器2A中断。           */
    __Encoder();
    switch (__GmRight.cState) {
        
    case __MOTORSTOP:                                                   /*  停止，同时清零速度和脉冲值  */
        __GmRight.uiPulse    = 0;
        __GmRight.uiPulseCtr = 0;
        __GmLeft.uiPulse    = 0;
        __GmLeft.uiPulseCtr = 0;
        break;

    case __WAITONESTEP:                                                 /*  暂停一步                    */
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

    case __MOTORRUN:                                                    /*  电机运行                    */
      if (__GucMouseState == __GOAHEAD)                                 /*  根据传感器状态微调电机位置  */
      {                             
            if ((__GucDistance[__FRONTL] && (__GucDistance[__FRONTR] == 0))||(__GucDistance[__FRONTR] &&(__GucDistance[__FRONTL]==0)))     /* 偏左,偏右 */
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
            else if (((__GucDistance[__RIGHT] == 1) && (__GucDistance[__LEFT] == 0))||((__GucDistance[__LEFT] == 1) && (__GucDistance[__RIGHT] == 0)))   /* 远偏左，远偏右 */
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
** Descriptions:        UART0中断服务函数
** input parameters:    无
** output parameters:   无
** Returned value:      无
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
** Descriptions:        START按键中断服务函数
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void GPIO_Port_C_ISR (void)
{
   uint8 status;
   status = GPIOPinIntStatus(GPIO_PORTC_BASE, true);              /*  读PC口中断状态              */
      if(status & __START)                                       /*  start按键中断          */
    {
        GPIOPinIntClear(GPIO_PORTC_BASE, __START);                  /*  清中断                      */
        if (GPIOPinRead(GPIO_PORTC_BASE, __START) == 0) 
        {
          __delay(50);
          while(GPIOPinRead(GPIO_PORTC_BASE, __START) == 0);
        }
    }
}
/*********************************************************************************************************
** Function name:       GPIO_Port_A_ISR
** Descriptions:        ZLG7289按键中断服务函数
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void GPIO_Port_A_ISR (void)                                         /*  按键调节红外频率             */
{
    uint8 status;
    uint8 key;
    uint8 B,S,G;
    status = GPIOPinIntStatus(GPIO_PORTA_BASE, true);              /*  读PA口中断状态              */

    if(status & ZLG7289_KEY)                                       /*  判断是否为按键中断          */
    {
        GPIOPinIntClear(GPIO_PORTA_BASE, ZLG7289_KEY);                  /*  清中断                      */
        
        key = zlg7289Key();                                           /*  读按键值                    */
        /*若按键有效，则让8个数码管一起显示*/
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
** Descriptions:        前进N格
** input parameters:    iNblock: 前进的格数
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void mouseGoahead (int8  cNBlock)                                    //连续转弯用
{
    int8 cL = 0, cR = 0, cCoor = 1,cB;
    
    GuiSpeedCtr=__SPEEDUP;
    if (__GmLeft.cState) {
        cCoor = 0;
    }
       if((__GucMouseState==__TURNRIGHT)||(__GucMouseState==__TURNLEFT))
    {
        if(GucDirTemp==0)  //到达目的地不需要转弯
        {
           GuiTpusle_back_LR = 1400;  //  <3000
           
        }
        else
        {
           GuiTpusle_back_LR = 2000;  //到达目的地需要转弯
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
       
        if (__GmLeft.uiPulseCtr >= ONEBLOCK) {                          /*  判断是否走完一格            */
            __GmLeft.uiPulse    -= ONEBLOCK;
            __GmLeft.uiPulseCtr -= ONEBLOCK;
            if (cCoor) {
                cNBlock--;
                if(cNBlock==0)
                   goto End;
                if(cNBlock<cB-1)//给回速一个时间
                  GuiSpeedCtr=__SPEEDUP;
            } else {
                cCoor = 1;
            }
        }
        
        if (__GmRight.uiPulseCtr >= ONEBLOCK) {                         /*  判断是否走完一格            */
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
          {                                                       /*  是否允许检测左边            */
            if ((__GucDistance[ __LEFT] & 0x01) == 0)             /*  左边有支路，跳出程序        */
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
            } else {                                                    /*  左边有墙时开始允许检测左边  */
                if ( __GucDistance[ __LEFT] & 0x01) {
                    cL = 1;
                }
            }
         if (cR) 
            {                                                       /*  是否允许检测右边            */
            if ((__GucDistance[__RIGHT] & 0x01) == 0)               /*  右边有支路，跳出程序        */
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
                if ( __GucDistance[__RIGHT] & 0x01) {                   /*  右边有墙时开始允许检测右边  */
                    cR = 1;
                }
            }
        }
   }
End: ;
  
}

void mouseGoahead_Llow (int8  cNBlock)                                    //连续转弯用,如有十字路口低速、不加速
{
    int8 cL = 0, cR = 0, cCoor = 1,cB;
    GuiSpeedCtr=__SPEEDUP;
    if (__GmLeft.cState) {
        cCoor = 0;
    }
    if((__GucMouseState==__TURNRIGHT)||(__GucMouseState==__TURNLEFT))
    {
        if(GucDirTemp==0)  //到达目的地不需要转弯
        {
           GuiTpusle_back_LR = 1400;  //  <3000
        }
        else
        {
           GuiTpusle_back_LR = 2000;  //到达目的地需要转弯
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
       
        if (__GmLeft.uiPulseCtr >= ONEBLOCK) {                          /*  判断是否走完一格            */
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
        
        if (__GmRight.uiPulseCtr >= ONEBLOCK) {                         /*  判断是否走完一格            */
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
          {                                                       /*  是否允许检测左边            */
            if ((__GucDistance[ __LEFT] & 0x01) == 0)             /*  左边有支路，跳出程序        */
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
            } else {                                                    /*  左边有墙时开始允许检测左边  */
                if ( __GucDistance[ __LEFT] & 0x01) {
                    cL = 1;
                }
            }
         if (cR) 
            {                                                       /*  是否允许检测右边            */
            if ((__GucDistance[__RIGHT] & 0x01) == 0)               /*  右边有支路，跳出程序        */
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
                if ( __GucDistance[__RIGHT] & 0x01) {                   /*  右边有墙时开始允许检测右边  */
                    cR = 1;
                }
            }
        }
   }
    /*
     *  设定运行任务，让电脑鼠走到支路的中心位置
     */
End:    ;
        
}
/*********************************************************************************************************
** Function name:       mazeSearch
** Descriptions:        前进N格
** input parameters:    iNblock: 前进的格数
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void mazeSearch(void)                  //搜索连续转弯
{
    int8 cL = 0, cR = 0, cCoor = 1,cj=0;
    uint16 bug_LR=0;

    __GmSPID.sRef=36;
    __GiMaxSpeed=36;
    if (__GmLeft.cState) {
        cCoor = 0;
    }
    /*
     *  设定运行任务
    
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
      
        if (__GmLeft.uiPulseCtr >= ONEBLOCK) {                          /*  判断是否走完一格            */
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
                  if ((__GucDistance[ __LEFT] & 0x01) == 0) {                 /*  左边有支路，跳出程序        */            
                    __GmRight.uiPulse = __GmRight.uiPulseCtr+bug_LR;
                    __GmLeft.uiPulse  = __GmLeft.uiPulseCtr+bug_LR;
                    while ((__GucDistance[ __LEFT] & 0x01) == 0) {
                     
                        if ((__GmLeft.uiPulseCtr + 600) > __GmLeft.uiPulse) {
                          flag_bug=0;//原地转标志
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
                 if ((__GucDistance[__RIGHT] & 0x01) == 0) {                 /*  右边有支路，跳出程序        */
                
                    __GmRight.uiPulse = __GmRight.uiPulseCtr+bug_LR;     //3300
                    __GmLeft.uiPulse  = __GmLeft.uiPulseCtr+bug_LR;
                    while ((__GucDistance[ __RIGHT] & 0x01) == 0) {
                     
                        if ((__GmLeft.uiPulseCtr + 600) > __GmLeft.uiPulse) {
                          flag_bug=0;//原地转标志
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
             
              __mouseCoorUpdate();                                    /*  更新坐标                    */
            } else {
                cCoor = 1;
            }
        }
        if (__GmRight.uiPulseCtr >= ONEBLOCK) {                         /*  判断是否走完一格            */
            __GmRight.uiPulse    -= ONEBLOCK;
            __GmRight.uiPulseCtr -= ONEBLOCK;
        }
        
        if (cL) {                                                       /*  是否允许检测左边            */
          if(cj){GuiTpusle_LR =0;}
            if ((__GucDistance[ __LEFT] & 0x01) == 0) {                 /*  左边有支路，跳出程序        */            
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
          /*  左边有墙时开始允许检测左边  */
            if ( __GucDistance[ __LEFT] & 0x01) {
                cL = 1;
            }
        }
        if (cR) {                                                       /*  是否允许检测右边            */
            if(cj){GuiTpusle_LR =0;}
            if ((__GucDistance[__RIGHT] & 0x01) == 0) {                 /*  右边有支路，跳出程序        */
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
            if ( __GucDistance[__RIGHT] & 0x01) {                       /*  右边有墙时开始允许检测右边  */
                cR = 1;
            }
        }
    }
End:   
        __mouseCoorUpdate();                                            /*  更新坐标                    */
    
}

/*********************************************************************************************************
** Function name:       Go_one_grid
** Descriptions:        定时中断扫描。
** input parameters:    无
** output parameters:   无
** Returned value:      无
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
    __mouseCoorUpdate();                                            /*  更新坐标                    */
}


/*********************************************************************************************************
** Function name:       SysTick_ISR
** Descriptions:        定时中断扫描。
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void SysTick_ISR(void)
{
  __irCheck();
}

/*********************************************************************************************************
** Function name:       __irSendFreq
** Descriptions:        发送红外线。
** input parameters:    __uiFreq:  红外线调制频率
**                      __cNumber: 选择需要设置的PWM模块
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __irSendFreq (uint32  __uiFreq, int8  __cNumber)
{
   __uiFreq = SysCtlClockGet() / __uiFreq;
    switch (__cNumber) {

    case 1:                                                             /*前方红外*/
        PWMGenPeriodSet(PWM_BASE, PWM_GEN_1, __uiFreq);                   /*  设置PWM发生器1的周期        */
        PWMPulseWidthSet(PWM_BASE, PWM_OUT_2, __uiFreq / 2);              /*  设置PWM2输出的脉冲宽度      */
        PWMOutputState(PWM_BASE,PWM_OUT_2_BIT,true);                  /*  使能PWM2              */                    
        break;

    case 2:                                                            /*左右红外*/
        PWMGenPeriodSet(PWM_BASE, PWM_GEN_1, __uiFreq);                   /*  设置PWM发生器1的周期        */
        PWMPulseWidthSet(PWM_BASE, PWM_OUT_3, __uiFreq / 2);              /*  设置PWM3输出的脉冲宽度      */
        PWMOutputState(PWM_BASE,PWM_OUT_3_BIT,true);                   /*  使能PWM3              */  
        break;
    case 3:                                                            /*斜45红外*/
        TimerLoadSet(TIMER1_BASE, TIMER_B, __uiFreq);                     //    设置TimerB初值 
        TimerMatchSet(TIMER1_BASE, TIMER_B, __uiFreq / 2);                 //   设置TimerB的 PWM匹配值 ,改占空比调红外发射强度
        TimerEnable(TIMER1_BASE, TIMER_B);
        break;
    default:
        break;
    }
}


/*********************************************************************************************************
** Function name:       __irCheck
** Descriptions:        红外线传感器检测。
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __irCheck (void)
{
    static uint8 ucState = 0;
    static uint8 ucIRCheck,ucIRCheck1;
    
    switch (ucState) {

    case 0:
        __irSendFreq(GusFreq_X, 3);                                          /*  驱动斜45度角上的传感器检测      */
       __irSendFreq(GusFreq_L, 2);                                         /*  探测左右两侧近距    30000        */
        break;
        
    case 1:
        ucIRCheck1 = GPIOPinRead(GPIO_PORTB_BASE, 0x30);                 /*  读取斜45度传感器状态         */
        ucIRCheck = GPIOPinRead(GPIO_PORTD_BASE, 0x48);                   /*  读取左右传感器状态         */
       TimerDisable(TIMER1_BASE, TIMER_B);                            /*  禁止斜45度红外         */
        PWMOutputState(PWM_BASE,PWM_OUT_3_BIT,false);                  /*  禁止PWM3（左右红外）     */        
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
        __irSendFreq(GusFreq_F, 1);                                       /*  驱动检测前方向远距 36000 */
        break;
        
    case 3:
       ucIRCheck = GPIOPinRead(GPIO_PORTD_BASE, 0x84);                    /*  读取前方传感器状态      */
       PWMOutputState(PWM_BASE,PWM_OUT_2_BIT,false);                     /*  禁止PWM2(前方红外)     */
        if ((ucIRCheck & __FRONT_R)||(ucIRCheck & __FRONT_L)) {
            __GucDistance[__FRONT] &= 0xfe;                               /*  前方无挡板 */
            GucMouseStart = 0;
        } else {
            __GucDistance[__FRONT] |= 0x01;                               /*  前方存在挡板  */
            GucMouseStart = 1;
        }
        break;
        
    case 4:
        __irSendFreq(GusFreq_LF, 2);                                         /*  检测左右方向远距  34000*/
        break;
        
    case 5:
       ucIRCheck = GPIOPinRead(GPIO_PORTD_BASE, 0x48);                  /*  读取左右传感器状态        */
       PWMOutputState(PWM_BASE,PWM_OUT_3_BIT,false);                  /*  禁止PWM3（左右红外）     */
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
        __irSendFreq(GusFreq_FJ, 1);                                       /*  驱动检测前方向近距  */
        break;
        
     case 7:
       ucIRCheck = GPIOPinRead(GPIO_PORTD_BASE, 0x84);                    /*  读取前方传感器状态      */
       PWMOutputState(PWM_BASE,PWM_OUT_2_BIT,false);                     /*  禁止PWM2(前方红外)     */
        if ((ucIRCheck & __FRONT_R)||(ucIRCheck & __FRONT_L)) {
            __GucDistance[__FRONT] &= 0xfd;                               /*  前方无挡板 */
            GucFrontJinju = 0;
        } else {
            __GucDistance[__FRONT] |= 0x02;                               /*  前方存在挡板  */
            GucFrontJinju = 1;
        }
        break;
    default:
        break;
    }
    ucState = (ucState + 1) % 8;                                        /*  循环检测                    */
}
void mouseStop(void)
{   
  __GmSPID.sRef=0;
  __GmWPID.sRef=0;
  GuiSpeedCtr=5;  
}
/****************************************************************************************************
** Function name:       mouseTurnright
** Descriptions:        右转
** input parameters:    无
** output parameters:   无
** Returned value:      无         按步数转弯
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
     __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
     __GmLeft.cState  =__MOTORRUN;
     GucMouseDir     = (GucMouseDir + 1) % 4;                            /*  方向标记                    */  
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
     __GmRight.cState = __MOTORRUN;        //得加电脑鼠状态
     __GmLeft.cState  = __MOTORRUN;
     GucMouseDir     = (GucMouseDir + 1) % 4;                            /*  方向标记                    */
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
** Descriptions:        左转
** input parameters:    无
** output parameters:   无
** Returned value:      无
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
   __GmRight.cState = __MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  = __MOTORRUN;
   GucMouseDir     = (GucMouseDir + 3) % 4;                            /*  方向标记                    */
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
     __GmRight.cState = __MOTORRUN;        //得加电脑鼠状态
     __GmLeft.cState  = __MOTORRUN;
     GucMouseDir     = (GucMouseDir + 3) % 4;                            /*  方向标记                    */
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
** Descriptions:        右转-返回用
** input parameters:    无
** output parameters:   无
** Returned value:      无
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
   __GmRight.cState =__MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  =__MOTORRUN;
   GucMouseDir     = (GucMouseDir + 1) % 4;                            /*  方向标记                    */
   
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
** Descriptions:        左转-返回用
** input parameters:    无
** output parameters:   无
** Returned value:      无
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
   __GmRight.cState = __MOTORRUN;        //得加电脑鼠状态
   __GmLeft.cState  = __MOTORRUN;
   GucMouseDir     = (GucMouseDir + 3) % 4;                            /*  方向标记                    */
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
** Descriptions:        根据前方近距，旋转180度
** input parameters:    无
** output parameters:   无
** Returned value:      无
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
** Descriptions:        原地旋转180度
** input parameters:    无
** output parameters:   无
** Returned value:      无
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
** Descriptions:        根据当前方向更新坐标值
** input parameters:    无
** output parameters:   无
** Returned value:      无
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
** Descriptions:        根据传感器检测结果判断是否存在墙壁
** input parameters:    无
** output parameters:   无
** Returned value:      cValue: 低三位从左到右一次代表左前右。1为有墙，0为没墙。
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
        {GucMapBlock1[GmcMouse.cX][GmcMouse.cY+1] &= ~(((~ucMap)&0x01)*4);}       /*将该坐标周围坐标墙壁资料更改  注：洪水用*/
       if(GmcMouse.cX<(MAZETYPE-1))
        { GucMapBlock1[GmcMouse.cX+1][GmcMouse.cY]&= ~(((~ucMap)&0x02)*4);}
         if(GmcMouse.cY>0)
         {GucMapBlock1[GmcMouse.cX][GmcMouse.cY-1]&= ~(((~ucMap)&0x04)/4);}
        if(GmcMouse.cX>0)
         {GucMapBlock1[GmcMouse.cX-1][GmcMouse.cY]&= ~(((~ucMap)&0x08)/4);}
              
      if(GmcMouse.cY<(MAZETYPE-1))
         {GucMapBlock[GmcMouse.cX][GmcMouse.cY+1] |=    ((ucMap&0x01)*4);}        /*将该坐标周围坐标墙壁资料更改  注：在初始为有墙时管用*/
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
** Descriptions:        用数码管显示出传感器状态，方便调试
** input parameters:    无
** output parameters:   无
** Returned value:      无
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
** Descriptions:        用数码管显示出当前电脑鼠前进方向和坐标
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __mazeInfDebug (void)
{
    /*
     *  显示方向
     */
    switch (GucMouseDir) {
        
    case 0:
        zlg7289Download(2, 3, 0, 0x47);                                /*  向前，用F表示               */
        break;
        
    case 1:
        zlg7289Download(2, 3, 0, 0x77);                                /*  向右，用R表示               */
        break;
        
    case 2:
        zlg7289Download(2, 3, 0, 0x1f);                                /*  向后，用b表示               */
        break;
        
    case 3:
        zlg7289Download(2, 3, 0, 0x0e);                                /*  向左，用L表示               */
        break;
        
    default :
        zlg7289Download(2, 3, 0, 0x4f);                                /*  错误，用E表示               */
        break;
    }
    /*
     *  显示坐标
     */
    zlg7289Download(1, 0, 0, GmcMouse.cX / 10);
    zlg7289Download(1, 1, 0, GmcMouse.cX % 10);
    zlg7289Download(1, 6, 0, GmcMouse.cY / 10);
    zlg7289Download(1, 7, 0, GmcMouse.cY % 10);
}

/*********************************************************************************************************
** Function name:       startCheck
** Descriptions:        读取按键
** input parameters:    无
** output parameters:   无
** Returned value:      true:  按键已按下
**                      false: 按键未按下
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
** Descriptions:        读取按键
** input parameters:    无
** output parameters:   无
** Returned value:      true:  按键已按下
**                      false: 按键未按下
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
** Descriptions:        对LM3S615处理器进行初始化
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void mouseInit (void)
{
    SysCtlClockSet( SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                    SYSCTL_XTAL_6MHZ );                                 /*  使能PLL，50M                */
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOA );
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOB );                      /*  使能GPIO B口外设            */
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOC );                      /*  使能GPIO C口外设            */
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOD );                      /*  使能GPIO D口外设            */
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOE );                      /*  使能GPIO E口外设            */
    PIDInit();                                                          /*  PID初始化                  */
    __keyInit();                                                        /*  按键初始化                  */
    __sensorInit();                                                     /*  传感器初始化                */
    __MotorIint();                                                      /*  直流电机控制初始化          */
    __sysTickInit();                                                    /*  系统时钟初始化              */
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
** Descriptions:        传感器控制初始化
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __sensorInit (void)
{
     /*
     *  设置连接到传感器信号输出脚的I/O口为输入模式
     */
    GPIODirModeSet(GPIO_PORTB_BASE,
                   __FRONTSIDE_L |
                   __FRONTSIDE_R,  
                   GPIO_DIR_MODE_IN);
    GPIODirModeSet(GPIO_PORTD_BASE,
                   __RIGHTSIDE | __FRONT_L | __LEFTSIDE | __FRONT_R,  
                   GPIO_DIR_MODE_IN);
    /*
     *  用PWM驱动红外线发射头产生调制的红外线信号
     */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM);                          /*  使能PWM模块                 */
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);                                 /*  PWM时钟配置：不分频         */

    GPIOPinTypePWM(GPIO_PORTB_BASE, __IRSEND1_FRONT | __IRSEND3_LR);    /*  PB0,PB1配置为PWM功能            */
    PWMGenConfigure(PWM_BASE, PWM_GEN_1,                                /*  配置PWM发生器1              */
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);       /*  加计数，立即更新            */

    PWMOutputState(PWM_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, false);      /*  禁止PWM2,PWM3输出                */
    PWMGenEnable(PWM_BASE, PWM_GEN_1);
    
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);     //   使能Timer1模块 
    GPIOPinTypeTimer(GPIO_PORTC_BASE, __IRSEND2_BEVEL);     //   配置CCP3 管脚为PWM 输出      
    TimerConfigure(TIMER1_BASE, TIMER_CFG_16_BIT_PAIR |  //   配置Timer为双16位 PWM 
                    TIMER_CFG_B_PWM); 
    TimerDisable(TIMER1_BASE, TIMER_B);     //   禁止Timer计数， PWM开始输出
    TimerControlLevel(TIMER1_BASE, TIMER_B, true);   //   控制PWM 输出反相 
}


/*********************************************************************************************************
** Function name:       __MotorIint
** Descriptions:        电机控制初始化
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __MotorIint (void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM);                          /*  使能PWM模块                 */
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);  
    GPIOPinTypePWM(GPIO_PORTD_BASE, __D1_1 | __D1_2);                       /*  PD0(D1_1),PD1(D1_2)配置为PWM功能 */
    GPIOPinTypePWM(GPIO_PORTE_BASE, __D2_1 | __D2_2);                       /*  PE0(D2_1),PE1(D2_2)配置为PWM功能  */
    PWMGenConfigure(PWM_BASE, PWM_GEN_0,                                /*  配置PWM发生器0              */
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);       /*  加计数，立即更新            */
    PWMGenConfigure(PWM_BASE, PWM_GEN_2,                                /*  配置PWM发生器2              */
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);       /*  加计数，立即更新            */
    PWMOutputState(PWM_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT | PWM_OUT_4_BIT | PWM_OUT_5_BIT , true);     /*使能PWM0,1,4,5输出  */
    PWMGenDisable(PWM_BASE, PWM_GEN_0);                                 /*  禁止PWM发生器0       */
    PWMGenDisable(PWM_BASE, PWM_GEN_2);                                 /*  禁止PWM发生器2       */
    PWMGenPeriodSet(PWM_BASE, PWM_GEN_0, 2000);                   /*  设置PWM发生器0的周期  =50M/f  25KHz  3500 3400 3300 2000*/
    PWMGenPeriodSet(PWM_BASE, PWM_GEN_2, 2000);                   /*  设置PWM发生器2的周期  =50M/f  25KHz*/
    
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);                       /*  使能定时器2模块             */
    TimerConfigure(TIMER2_BASE, TIMER_CFG_32_BIT_PER);                  /*  配置为32位周期计数模式      */
    TimerLoadSet(TIMER2_BASE, TIMER_A, 50000);                          /*  设置定时时间                */
    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);                    /*  设置为溢出中断              */
    //IntPrioritySet(INT_TIMER2A,2<<5);
    IntEnable(INT_TIMER2A);                                             /*  使能定时器2中断             */
    TimerEnable(TIMER2_BASE, TIMER_A);                                  /*  使能定时器2                 */
    
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);                         //使能Timer0模块
    GPIOPinTypeTimer(GPIO_PORTD_BASE,__SIG_M2_B);                         //配置D口的4为脉冲输入
    GPIOPinTypeTimer(GPIO_PORTE_BASE,__SIG_M1_B);                         //配置E口的3为脉冲输入
    TimerConfigure(TIMER0_BASE, TIMER_CFG_16_BIT_PAIR | TIMER_CFG_A_CAP_COUNT | TIMER_CFG_B_CAP_COUNT);// 配置Timer0为16位事件计数器
    TimerControlEvent(TIMER0_BASE,TIMER_BOTH,TIMER_EVENT_POS_EDGE);       // 控制Timer0A捕获CCP负边沿
    TimerLoadSet(TIMER0_BASE, TIMER_BOTH, 65535);                          // 设置计数器初值
    TimerMatchSet(TIMER0_BASE, TIMER_BOTH, 0);                             // 设置事件计数匹配值
    TimerIntEnable(TIMER0_BASE, TIMER_CAPA_MATCH);                        // 使能Timer0A捕获匹配中断
    TimerIntEnable(TIMER0_BASE, TIMER_CAPB_MATCH);                        // 使能Timer0B捕获匹配中断
    IntEnable(INT_TIMER0A);                                                // 使能Timer0A中断
    IntEnable(INT_TIMER0B);                                               // 使能Timer0B中断
    TimerEnable(TIMER0_BASE, TIMER_BOTH);                                // 使能Timer0AB计数
}
/*********************************************************************************************************
** Function name:       __keyInit
** Descriptions:        对连接按键的GPIO口初始化
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __keyInit (void)
{
    GPIODirModeSet(GPIO_PORTC_BASE, __KEY, GPIO_DIR_MODE_IN);           /*  设置按键口为输入            */   
    GPIODirModeSet(GPIO_PORTA_BASE, ZLG7289_KEY, GPIO_DIR_MODE_IN);     /*  设置KEY端口为输入           */
    GPIODirModeSet(GPIO_PORTC_BASE, __START, GPIO_DIR_MODE_IN);         /*  设置按键口为输入            */
    GPIOIntTypeSet(GPIO_PORTA_BASE, ZLG7289_KEY, GPIO_FALLING_EDGE);    /*  配置引脚下降沿触发中断      */
    
    GPIOPinIntEnable(GPIO_PORTA_BASE, ZLG7289_KEY);                     /*  使能引脚输入中断            */
    IntEnable(INT_GPIOA);  /*  使能GPIO PA口中断           */       
}
/*********************************************************************************************************
** Function name:       __keyIntDisable
** Descriptions:        ZLG7289按键中断禁能 重启
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void  __keyIntDisable (void) 
{
    GPIOPinIntDisable(GPIO_PORTA_BASE, ZLG7289_KEY);
}
/*********************************************************************************************************
** Function name:       __sysTickInit
** Descriptions:        系统节拍定时器初始化。
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __sysTickInit (void)
{
    SysTickPeriodSet(SysCtlClockGet() / 1600);                          /*  设置定时时钟为625us           */
    //IntPrioritySet(FAULT_SYSTICK,1<<5);
    SysTickEnable();                                                    /*  使能系统时钟                */
    SysTickIntEnable();                                                 /*  使能系统时钟中断            */
}
/*********************************************************************************************************
** Function name:      __UART0Init
** Descriptions:        对串口0进行初始化
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __UART0Init (void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinTypeUART(GPIO_PORTA_BASE,GPIO_PIN_0  | GPIO_PIN_1);    //PA0 PA1设置为串口
    UARTConfigSet(UART0_BASE, 115200, (UART_CONFIG_WLEN_8 |  //配置串口0，8位数据，1位起始位，1位停止位，用户波特率
                                       UART_CONFIG_STOP_ONE |
                                       UART_CONFIG_PAR_NONE) & 0xFFFFFFEF);
    //IntEnable(INT_UART0);                                      //使能串口0系统中断
    //UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);      //使能串口0接收中断和接收超时中断 
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
** Descriptions:        急停
** input parameters:    无
** output parameters:   无
** Returned value:      无
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
** Descriptions:        原地转弯矫正位置
** input parameters:    Pulse：向后矫正脉冲数
** output parameters:   无
** Returned value:      无
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
** Descriptions:        传感器检测起跑
** input parameters:    
** output parameters:   无
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
** Descriptions:        发送一串字符
** input parameters:    
** output parameters:   无
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
** Descriptions:        延时毫秒
** input parameters:    
** output parameters:   无
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





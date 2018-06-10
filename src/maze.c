/****************************************Copyright (c)****************************************************
**                               Guangzhou ZHIYUAN electronics Co.,LTD.
**                                     
**                                 http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:           maze.c
** Last modified Date:  2007/09/24
** Last Version:        V1.0
** Description:         根据底层程序取得的迷宫信息，经过该智能算法控制电脑鼠的下一状态，并送往底层驱动程
**                      序执行。
** 
**--------------------------------------------------------------------------------------------------------
** Created By:          Liao Maogang
** Created date:        2007/09/08
** Version:             V1.0
** Descriptions: 
**
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
** Version:
** Description:
**                                  在中断中stop
*********************************************************************************************************/


/*********************************************************************************************************
  包含头文件
*********************************************************************************************************/
#include "Maze.h"
#include "Mouse_Drive.h"
/*********************************************************************************************************
  全局变量定义
*********************************************************************************************************/
static uint8    GucXStart                           = 0;                /*  起点横坐标                  */
static uint8    GucYStart                           = 0;                /*  起点纵坐标                  */
static uint8    GucXGoal0                           = XDST0;            /*  终点X坐标，有两个值         */
static uint8    GucXGoal1                           = XDST1;
static uint8    GucYGoal0                           = YDST0;            /*  终点Y坐标，有两个值         */
static uint8    GucYGoal1                           = YDST1;
static uint8    GucMouseTask                        = WAIT;             /*  状态机，初始状态为等待      */
static uint8    GucMapStep[MAZETYPE][MAZETYPE]      = {0xff};           /*  保存各坐标的等高值          */
static MAZECOOR GmcStack[MAZETYPE * MAZETYPE]       = {0};              /*  在mapStepEdit()中作堆栈使用 */
static MAZECOOR GmcCrossway[MAZETYPE * MAZETYPE]    = {0};              /*  Main()中暂存未走过支路坐标  */
uint8 t=1;  //UARTCharPut(UART0_BASE, t++); 
/*********************************************************************************************************
** Function name:       Delay
** Descriptions:        延时函数
** input parameters:    uiD :延时参数，值越大，延时越久
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void delay (uint32 uiD)
{
    for (; uiD; uiD--);
}

/*********************************************************************************************************
** Function name:       mapStepEdit
** Descriptions:        制作以目标点为起点的等高图
** input parameters:    uiX:    目的地横坐标
**                      uiY:    目的地纵坐标
** output parameters:   GucMapStep[][]:  各坐标上的等高值
** Returned value:      无
*********************************************************************************************************/
void mapStepEdit (int8  cX, int8  cY)
{
    uint8 n         = 0;                                                /*  GmcStack[]下标              */
    uint8 ucStep    = 1;                                                /*  等高值                      */
    uint8 ucStat    = 0;                                                /*  统计可前进的方向数          */
    uint8 i,j;
    
    GmcStack[n].cX  = cX;                                               /*  起点X值入栈                 */
    GmcStack[n].cY  = cY;                                               /*  起点Y值入栈                 */
    n++;
    /*
     *  初始化各坐标等高值
     */
    for (i = 0; i < MAZETYPE; i++) {
        for (j = 0; j < MAZETYPE; j++) {
            GucMapStep[i][j] = 0xff;
        }
    }
    /*
     *  制作等高图，直到堆栈中所有数据处理完毕
     */
    while (n) {
        GucMapStep[cX][cY] = ucStep++;                                  /*  填入等高值                  */

        /*
         *  对当前坐标格里可前进的方向统计
         */
        ucStat = 0;
        if ((GucMapBlock[cX][cY] & 0x01) &&                             /*  前方有路                    */
            (GucMapStep[cX][cY + 1] > (ucStep))) {                      /*  前方等高值大于计划设定值    */
            ucStat++;                                                   /*  可前进方向数加1             */
        }
        if ((GucMapBlock[cX][cY] & 0x02) &&                             /*  右方有路                    */
            (GucMapStep[cX + 1][cY] > (ucStep))) {                      /*  右方等高值大于计划设定值    */
            ucStat++;                                                   /*  可前进方向数加1             */
        }
        if ((GucMapBlock[cX][cY] & 0x04) &&
            (GucMapStep[cX][cY - 1] > (ucStep))) {
            ucStat++;                                                   /*  可前进方向数加1             */
        }
        if ((GucMapBlock[cX][cY] & 0x08) &&
            (GucMapStep[cX - 1][cY] > (ucStep))) {
            ucStat++;                                                   /*  可前进方向数加1             */
        }
        /*
         *  没有可前进的方向，则跳转到最近保存的分支点
         *  否则任选一可前进方向前进
         */
        if (ucStat == 0) {
            n--;
            cX = GmcStack[n].cX;
            cY = GmcStack[n].cY;
            ucStep = GucMapStep[cX][cY];
        } else {
            if (ucStat > 1) {                                           /*  有多个可前进方向，保存坐标  */
                GmcStack[n].cX = cX;                                    /*  横坐标X值入栈               */
                GmcStack[n].cY = cY;                                    /*  纵坐标Y值入栈               */
                n++;
            }
            /*
             *  任意选择一条可前进的方向前进
             */
            if ((GucMapBlock[cX][cY] & 0x01) &&                         /*  上方有路                    */
                (GucMapStep[cX][cY + 1] > (ucStep))) {                  /*  上方等高值大于计划设定值    */
                cY++;                                                   /*  修改坐标                    */
                continue;
            }
            if ((GucMapBlock[cX][cY] & 0x02) &&                         /*  右方有路                    */
                (GucMapStep[cX + 1][cY] > (ucStep))) {                  /*  右方等高值大于计划设定值    */
                cX++;                                                   /*  修改坐标                    */
                continue;
            }
            if ((GucMapBlock[cX][cY] & 0x04) &&                         /*  下方有路                    */
                (GucMapStep[cX][cY - 1] > (ucStep))) {                  /*  下方等高值大于计划设定值    */
                cY--;                                                   /*  修改坐标                    */
                continue;
            }
            if ((GucMapBlock[cX][cY] & 0x08) &&                         /*  左方有路                    */
                (GucMapStep[cX - 1][cY] > (ucStep))) {                  /*  左方等高值大于计划设定值    */
                cX--;                                                   /*  修改坐标                    */
                continue;
            }
        }
    }
}


void mouseSpurt (void)
{
    uint8 ucTemp = 0xff;
    int8 cXdst = 0,cYdst = 0;
    /*
     *  对终点的四个坐标分别制作等高图
     *  取离起点最近的一个点作为目标点
     */
    if (GucMapBlock[GucXGoal0][GucYGoal0] & 0x0c) {                     /*  判断该终点坐标是否有出口    */
        mapStepEdit(GucXGoal0,GucYGoal0);                               /*  制作等高图                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart]) {                /*  保存离起点最近的坐标        */
            cXdst  = GucXGoal0;
            cYdst  = GucYGoal0;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal0][GucYGoal1] & 0x09) {                     /*  判断该终点坐标是否有出口    */
        mapStepEdit(GucXGoal0,GucYGoal1);                               /*  制作等高图                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart]) {                /*  保存离起点最近的坐标        */
            cXdst  = GucXGoal0;
            cYdst  = GucYGoal1;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal1][GucYGoal0] & 0x06) {                     /*  判断该终点坐标是否有出口    */
        mapStepEdit(GucXGoal1,GucYGoal0);                               /*  制作等高图                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart]) {                /*  保存离起点最近的坐标        */
            cXdst  = GucXGoal1;
            cYdst  = GucYGoal0;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal1][GucYGoal1] & 0x03) {                     /*  判断该终点坐标是否有出口    */
        mapStepEdit(GucXGoal1,GucYGoal1);                               /*  制作等高图                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart]) {                /*  保存离起点最近的坐标        */
            cXdst  = GucXGoal1;
            cYdst  = GucYGoal1;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    
    objectGoTo(cXdst,cYdst);                                            /*  运行到指定目标点            */
    
}

void objectGoTo(int8  cXdst, int8  cYdst)
{
    uint8 ucStep = 1;
    int8  cNBlock = 0, cDirTemp;
    int8 cX,cY;
    GucCrossroad=0;
    cX = GmcMouse.cX;
    cY = GmcMouse.cY;
    mapStepEdit(cXdst,cYdst);                                           /*  制作等高图                  */
    
    /*
     *  根据等高值向目标点运动，直到达到目的地
     */
    while ((cX != cXdst) || (cY != cYdst)) {
        
        ucStep = GucMapStep[cX][cY];
        /*
         *  任选一个等高值比当前自身等高值小的方向前进
         */
        if ((GucMapBlock[cX][cY] & 0x01) &&                             /*  上方有路                    */
            (GucMapStep[cX][cY + 1] < ucStep)) {                        /*  上方等高值较小              */
            cDirTemp = UP;                                              /*  记录方向                    */
            if (cDirTemp == GucMouseDir) {                              /*  优先选择不需要转弯的方向    */
                cNBlock++;                                              /*  前进一个方格                */
                cY++;
                if((GucMapBlock[cX][cY] & 0x0f)==0x0f)
                  GucCrossroad++;
                continue;                                               /*  跳过本次循环                */
            }
        }
        if ((GucMapBlock[cX][cY] & 0x02) &&                             /*  右方有路                    */
            (GucMapStep[cX + 1][cY] < ucStep)) {                        /*  右方等高值较小              */
            cDirTemp = RIGHT;                                           /*  记录方向                    */
            if (cDirTemp == GucMouseDir) {                              /*  优先选择不需要转弯的方向    */
                cNBlock++;                                              /*  前进一个方格                */
                cX++;
                if((GucMapBlock[cX][cY] & 0x0f)==0x0f)
                  GucCrossroad++;
                continue;                                               /*  跳过本次循环                */
            }
        }
        if ((GucMapBlock[cX][cY] & 0x04) &&                             /*  下方有路                    */
            (GucMapStep[cX][cY - 1] < ucStep)) {                        /*  下方等高值较小              */
            cDirTemp = DOWN;                                            /*  记录方向                    */
            if (cDirTemp == GucMouseDir) {                              /*  优先选择不需要转弯的方向    */
                cNBlock++;                                              /*  前进一个方格                */
                cY--;
                if((GucMapBlock[cX][cY] & 0x0f)==0x0f)
                  GucCrossroad++;
                continue;                                               /*  跳过本次循环                */
            }
        }
        if ((GucMapBlock[cX][cY] & 0x08) &&                             /*  左方有路                    */
            (GucMapStep[cX - 1][cY] < ucStep)) {                        /*  左方等高值较小              */
            cDirTemp = LEFT;                                            /*  记录方向                    */
            if (cDirTemp == GucMouseDir) {                              /*  优先选择不需要转弯的方向    */
                cNBlock++;                                              /*  前进一个方格                */
                cX--;
                if((GucMapBlock[cX][cY] & 0x0f)==0x0f)
                  GucCrossroad++;
                continue;                                               /*  跳过本次循环                */
            }
        }
        cDirTemp = (cDirTemp + 4 - GucMouseDir)%4;                      /*  计算方向偏移量              */
        GucDirTemp = cDirTemp;
        if (cNBlock) {
          if((GucCrossroad <= 1)&&(cNBlock>1))
              mouseGoahead(cNBlock);                                      /*  前进cNBlock步               */
          else{
            mouseGoahead_Llow(cNBlock);
            GucCrossroad = 0;
          }         
        }        
        cNBlock = 0;  
        /*  任务清零                    */
        
        /*
         *  控制电脑鼠转弯
         */
         
       switch (cDirTemp) {

        case 1:
            mouseTurnright_C();
            break;

        case 2:
            mouseTurnback();
            mouseTurnback_correct(900);
            flag_bug1=1;
            break;

        case 3:
            mouseTurnleft_C();
            break;

        default:
            break;
        }
      GmcMouse.cX=cX;
      GmcMouse.cY=cY;
    }
    /*
     *  判断任务是否完成，否则继续前进
     */
    
      if (cNBlock) {
          if((GucCrossroad <= 1)&&(cNBlock>1))
            mouseGoahead(cNBlock);                                      /*  前进cNBlock步               */
          else{
            mouseGoahead_Llow(cNBlock);
            GucCrossroad = 0;
          }
          GmcMouse.cX=cX;
          GmcMouse.cY=cY;
      }
}

/*********************************************************************************************************
** Function name:       mazeBlockDataGet
** Descriptions:        根据电脑鼠的相对方向，取出该方向上迷宫格的墙壁资料
** input parameters:    ucDir: 电脑鼠的相对方向
** output parameters:   无
** Returned value:      GucMapBlock[cX][cY] : 墙壁资料
*********************************************************************************************************/
uint8 mazeBlockDataGet (uint8  ucDirTemp)
{
    int8 cX = 0,cY = 0;
    
    /*
     *  把电脑鼠的相对方向转换为绝对方向
     */
    switch (ucDirTemp) {

    case MOUSEFRONT:
        ucDirTemp = GucMouseDir;
        break;

    case MOUSELEFT:
        ucDirTemp = (GucMouseDir + 3) % 4;
        break;

    case MOUSERIGHT:
        ucDirTemp = (GucMouseDir + 1) % 4;
        break;

    default:
        break;
    }
    
    /*
     *  根据绝对方向计算该方向上相邻格的坐标
     */
    switch (ucDirTemp) {

    case 0:
        cX = GmcMouse.cX;
        cY = GmcMouse.cY + 1;
        break;
        
    case 1:
        cX = GmcMouse.cX + 1;
        cY = GmcMouse.cY;
        break;
        
    case 2:
        cX = GmcMouse.cX;
        cY = GmcMouse.cY - 1;
        break;
        
    case 3:
        cX = GmcMouse.cX - 1;
        cY = GmcMouse.cY;
        break;
        
    default:
        break;
    }
    
    return(GucMapBlock0[cX][cY]);                                        /*  返回迷宫格上的资料          */
}
/*********************************************************************************************************
** Function name:       rightMethod
** Descriptions:        右手法则，优先向右前进
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void rightMethod (void)
{

    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  电脑鼠的右边有路            */
        (mazeBlockDataGet(MOUSERIGHT) == 0x00)) {                       /*  电脑鼠的右边没有走过        */
        mouseTurnright();                                               /*  电脑鼠右转                  */
        return;
    }
    
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  电脑鼠的左边有路            */
        (mazeBlockDataGet(MOUSELEFT ) == 0x00)) {                       /*  电脑鼠的左边没有走过        */
        mouseTurnleft();                                                /*  电脑鼠左转                  */
        return;
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  电脑鼠的前方有路            */
        (mazeBlockDataGet(MOUSEFRONT) == 0x00)) {                       /*  电脑鼠的前方没有走过        */   
        return;                                                         /*  电脑鼠不用转弯              */
    }
}
/*********************************************************************************************************
** Function name:       leftMethod
** Descriptions:        左手法则，优先向左运动
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void leftMethod (void)
{
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  电脑鼠的左边有路            */
        (mazeBlockDataGet(MOUSELEFT ) == 0x00)) {                       /*  电脑鼠的左边没有走过        */
        mouseTurnleft();                                                /*  电脑鼠左转                  */
        return;
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  电脑鼠的前方有路            */
        (mazeBlockDataGet(MOUSEFRONT) == 0x00)) {                       /*  电脑鼠的前方没有走过        */
        return;                                                         /*  电脑鼠不用转弯              */
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  电脑鼠的右边有路            */
        (mazeBlockDataGet(MOUSERIGHT) == 0x00)) {                       /*  电脑鼠的右边没有走过        */
        mouseTurnright();                                               /*  电脑鼠右转                  */
        return;
    }
}
/*********************************************************************************************************
** Function name:       frontRightMethod
** Descriptions:        中右法则，优先向前运行，其次向右
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void frontRightMethod (void)
 {
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  电脑鼠的前方有路            */
        (mazeBlockDataGet(MOUSEFRONT) == 0x00)) {                       /*  电脑鼠的前方没有走过        */
        
        return;                                                         /*  电脑鼠不用转弯              */
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  电脑鼠的右边有路            */
        (mazeBlockDataGet(MOUSERIGHT) == 0x00)) {                       /*  电脑鼠的右边没有走过        */
        mouseTurnright();                                               /*  电脑鼠右转                  */
        return;
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  电脑鼠的左边有路            */
        (mazeBlockDataGet(MOUSELEFT ) == 0x00)) {                       /*  电脑鼠的左边没有走过        */
        mouseTurnleft();                                                /*  电脑鼠左转                  */
        return;
    }
}
/*********************************************************************************************************
** Function name:       frontLeftMethod
** Descriptions:        中左法则，优先向前运行，其次向左
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void frontLeftMethod (void)
{
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  电脑鼠的前方有路            */
        (mazeBlockDataGet(MOUSEFRONT) == 0x00)) {                       /*  电脑鼠的前方没有走过        */
        return;                                                         /*  电脑鼠不用转弯              */
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  电脑鼠的左边有路            */
        (mazeBlockDataGet(MOUSELEFT ) == 0x00)) {                       /*  电脑鼠的左边没有走过        */
        mouseTurnleft();                                                /*  电脑鼠左转                  */
        return;
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  电脑鼠的右边有路            */
        (mazeBlockDataGet(MOUSERIGHT) == 0x00)) {                       /*  电脑鼠的右边没有走过        */
        mouseTurnright();                                               /*  电脑鼠右转                  */
        return;
    }
}

/*********************************************************************************************************
** Function name:       centralMethod
** Descriptions:        中心法则，根据电脑鼠目前在迷宫中所处的位置觉定使用何种搜索法则
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void centralMethod (void)
{
    if (GmcMouse.cX & 0x08) {
        if (GmcMouse.cY & 0x08) {

            /*
             *  此时电脑鼠在迷宫的右上角
             */ 
            switch (GucMouseDir) {
                
            case UP:                                                    /*  当前电脑鼠向上              */
                leftMethod();                                           /*  左手法则                    */
                break;

            case RIGHT:                                                 /*  当前电脑鼠向右              */
                rightMethod();                                          /*  右手法则                    */
                break;

            case DOWN:                                                  /*  当前电脑鼠向下              */
                frontRightMethod();                                     /*  中右法则                    */
                break;

            case LEFT:                                                  /*  当前电脑鼠向左              */
                frontLeftMethod();                                      /*  中左法则                    */
                break;

            default:
                break;
            }
        } else {

            /*
             *  此时电脑鼠在迷宫的右下角
             */    
            switch (GucMouseDir) {
                
            case UP:                                                    /*  当前电脑鼠向上              */
                frontLeftMethod();                                      /*  中左法则                    */
                break;

            case RIGHT:                                                 /*  当前电脑鼠向右              */
                leftMethod();                                           /*  左手法则                    */
                break;

            case DOWN:                                                  /*  当前电脑鼠向下              */
                rightMethod();                                          /*  右手法则                    */
                break;

            case LEFT:                                                  /*  当前电脑鼠向左              */
                frontRightMethod();                                     /*  中右法则                    */
                break;

            default:
                break;
            }
        }
    } else {
        if (GmcMouse.cY & 0x08) {

            /*
             *  此时电脑鼠在迷宫的左上角
             */    
            switch (GucMouseDir) {
                
            case UP:                                                    /*  当前电脑鼠向上              */
                rightMethod();                                          /*  右手法则                    */
                break;

            case RIGHT:                                                 /*  当前电脑鼠向右              */
                frontRightMethod();                                     /*  中右法则                    */
                break;

            case DOWN:                                                  /*  当前电脑鼠向下              */
                frontLeftMethod();                                      /*  中左法则                    */
                break;

            case LEFT:                                                  /*  当前电脑鼠向左              */
                leftMethod();                                           /*  左手法则                    */
                break;

            default:
                break;
            }
        } else {

            /*
             *  此时电脑鼠在迷宫的左下角
             */    
            switch (GucMouseDir) {
                
            case UP:                                                    /*  当前电脑鼠向上              */
                frontRightMethod();                                     /*  中右法则                    */
                break;

            case RIGHT:                                                 /*  当前电脑鼠向右              */
                frontLeftMethod();                                      /*  中左法则                    */
                break;

            case DOWN:                                                  /*  当前电脑鼠向下              */
                leftMethod();                                           /*  左手法则                    */
                break;

            case LEFT:                                                  /*  当前电脑鼠向左              */
                rightMethod();                                          /*  右手法则                    */
                break;

            default:
                break;
            }
        }
    }
}
/*********************************************************************************************************
** Function name:       crosswayCheck
** Descriptions:        统计某坐标存在还未走过的支路数
** input parameters:    ucX，需要检测点的横坐标
**                      ucY，需要检测点的纵坐标
** output parameters:   无
** Returned value:      ucCt，未走过的支路数
*********************************************************************************************************/
uint8 crosswayCheck (int8  cX, int8  cY)
{
    uint8 ucCt = 0;
    if ((GucMapBlock[cX][cY] & 0x01) &&                                 /*  绝对方向，迷宫上方有路      */
        (GucMapBlock0[cX][cY + 1]) == 0x00) {                            /*  绝对方向，迷宫上方未走过    */
        ucCt++;                                                         /*  可前进方向数加1             */
    }
    if ((GucMapBlock[cX][cY] & 0x02) &&                                 /*  绝对方向，迷宫右方有路      */
        (GucMapBlock0[cX + 1][cY]) == 0x00) {                            /*  绝对方向，迷宫右方没有走过  */
        ucCt++;                                                         /*  可前进方向数加1             */
    }
    if ((GucMapBlock[cX][cY] & 0x04) &&                                 /*  绝对方向，迷宫下方有路      */
        (GucMapBlock0[cX][cY - 1]) == 0x00) {                            /*  绝对方向，迷宫下方未走过    */
        ucCt++;                                                         /*  可前进方向数加1             */
    }
    if ((GucMapBlock[cX][cY] & 0x08) &&                                 /*  绝对方向，迷宫左方有路      */
        (GucMapBlock0[cX - 1][cY]) == 0x00) {                            /*  绝对方向，迷宫左方未走过    */
        ucCt++;                                                         /*  可前进方向数加1             */
    }
    return ucCt;
}
/*********************************************************************************************************
** Function name:       crosswayCheck_back
** Descriptions:        区别向后转或者gotoback
** input parameters:    ucX，需要检测点的横坐标
**                      ucY，需要检测点的纵坐标
** output parameters:   无
** Returned value:      
*********************************************************************************************************/
uint8 crosswayCheck_back (int8  cX, int8  cY)
{
   uint8 go1=0,go2=0;
   if(GucMapBlock[cX][cY] & 0x01)  /*  绝对方向，迷宫上方有路      */
   {
    go1++;
    if((GucMapBlock0[cX][cY + 1]) == 0x00)   /*  绝对方向，迷宫上方未走过    */
    {
    go2++;
    }
   }
   
   if(GucMapBlock[cX][cY] & 0x02)            /*  绝对方向，迷宫右方有路      */
   {
     go1++;
     if((GucMapBlock0[cX + 1][cY]) == 0x00)    /*  绝对方向，迷宫右方没有走过  */
     {
     go2++;
     }
   }
   
   if(GucMapBlock[cX][cY] & 0x04)            /*  绝对方向，迷宫下方有路      */
   {
    go1++;
    if((GucMapBlock0[cX][cY - 1]) == 0x00)     /*  绝对方向，迷宫下方未走过    */
    {
      go2++;
    }
   }
   
   if(GucMapBlock[cX][cY] & 0x08)            /*  绝对方向，迷宫左方有路      */
   {
      go1++;
      if((GucMapBlock0[cX - 1][cY]) == 0x00) /*  绝对方向，迷宫左方未走过    */
      {
      go2++;
      }
   }
   
   if(go1==1)
   {
    return true;
   }
   else
   {
    return false;
   }
}
/*********************************************************************************************************
** Function name:       crosswayChoice
** Descriptions:        选择一条支路作为前进方向
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void crosswayChoice (void)
{
    switch (SEARCHMETHOD) {
        
    case RIGHTMETHOD:
        rightMethod();
        break;
    
    case LEFTMETHOD:
        leftMethod();
        break;
    
    case CENTRALMETHOD:
        centralMethod();
        break;

    case FRONTRIGHTMETHOD:
        frontRightMethod();
        break;

    case FRONTLEFTMETHOD:
        frontLeftMethod();
        break;
       

    default:
        break;
    }
}
void weight_mapStepEdit (int8  cX, int8  cY)
{
    uint8 n         = 0;                                                /*  GmcStack[]下标              */
    uint8 ucStep    = 1;                                                /*  等高值                      */
    uint8 ucStat    = 0;                                                /*  统计可前进的方向数          */
    uint8 i,j;
    uint8 XGoal0 = cX;
    uint8 YGoal0 = cY;
    
    //初始方向要事先确定  1上 2右 4下 8左 0起点
    //uint8 dirold;
    uint8 dirnew;
    
    GmcStack[n].cX  = cX;                                               /*  起点X值入栈                 */
    GmcStack[n].cY  = cY;                                               /*  起点Y值入栈                 */
    n++;
    /*
     *  初始化各坐标等高值
     */
    for (i = 0; i < MAZETYPE; i++) {
        for (j = 0; j < MAZETYPE; j++) {
            GucMapStep[i][j] = 0xff;
        }
    }
    /*
     *  制作等高图，直到堆栈中所有数据处理完毕
     */
    while (n) {
        GucMapStep[cX][cY] = ucStep++;                                  /*  填入等高值                  */
        
        if (cX == XGoal0 && cY ==YGoal0)
	{
		if ((GucMapBlock[cX][cY] & 0x01)&& (GucMapStep[cX][cY + 1]-ucStep)>0){ /*  前方有路                    */
			dirnew =1;
			//dirold = 1;
		}
		else if ((GucMapBlock[cX][cY] & 0x02)&& (GucMapStep[cX+1][cY]-ucStep)>0){ /*  右方有路                    */
			dirnew =2;
			//dirold = 2;
		}
		else if ((GucMapBlock[cX][cY] & 0x04)&& (GucMapStep[cX][cY - 1]-ucStep)>0){ /*  后方有路                    */
			dirnew =4;
			//dirold = 4;
		}
		else if ((GucMapBlock[cX][cY] & 0x08)&& (GucMapStep[cX-1][cY ]-ucStep)>0){ /*  左方有路                    */
			dirnew =8;
			//dirold = 8;
		}
	}

        /*
         *  对当前坐标格里可前进的方向统计
         */
        ucStat = 0;
        if (GucMapBlock[cX][cY] & 0x01) {                      /*  前方有路                    */
	  if(dirnew==1 && (GucMapStep[cX][cY + 1]-ucStep)>0)   //相同方向  等高值大1
	  {
		ucStat++;
	  }
	  else if ((GucMapStep[cX][cY + 1]-ucStep)>1)         //不同方向  等高值大2
	  {
		ucStat++;
	  }
        }
        if (GucMapBlock[cX][cY] & 0x02) {                      /*  右方有路                    */
	  if(dirnew==2 && (GucMapStep[cX + 1][cY]-ucStep)>0)   //相同方向  等高值大1
	  {
		ucStat++;
	  }
	  else if ((GucMapStep[cX + 1][cY]-ucStep)>1)         //不同方向  等高值大2
	  {
		ucStat++;
	  }
        }
        if (GucMapBlock[cX][cY] & 0x04) {                      /*  下方有路                    */
	  if(dirnew==4 && (GucMapStep[cX][cY - 1]-ucStep)>0)   //相同方向  等高值大1
	  {
		ucStat++;
	  }
	  else if ((GucMapStep[cX][cY - 1]-ucStep)>1)         //不同方向  等高值大2
	  {
		ucStat++;
	  }
        }
        if (GucMapBlock[cX][cY] & 0x08) {                      /*  左方有路                    */
	  if(dirnew==8 && (GucMapStep[cX - 1][cY]-ucStep)>0)   //相同方向  等高值大1
	  {
		ucStat++;
	  }
	  else if ((GucMapStep[cX - 1][cY]-ucStep)>1)         //不同方向  等高值大2
	  {
		ucStat++;
	  }
        }
        
        
        /*
         *  没有可前进的方向，则跳转到最近保存的分支点
         *  否则任选一可前进方向前进
         */
        if (ucStat == 0) {
            n--;
            cX = GmcStack[n].cX;
            cY = GmcStack[n].cY;
            dirnew = GmcStack[n].dir;
	    //dirold = GmcStack[n].dir;
            ucStep = GucMapStep[cX][cY];
        } else {
            if (ucStat > 1) {                                           /*  有多个可前进方向，保存坐标  */
                GmcStack[n].cX = cX;                                    /*  横坐标X值入栈               */
                GmcStack[n].cY = cY;                                    /*  纵坐标Y值入栈               */
                GmcStack[n].dir = dirnew;
                n++;
            }
            /*
             *  任意选择一条可前进的方向前进
             */
            if (GucMapBlock[cX][cY] & 0x01) {                  /*  上方等高值大于计划设定值    */
		if(dirnew==1 && (GucMapStep[cX][cY + 1]-ucStep)>0) //相同方向  等高值大1
		{
		    cY++;                                                   /*  修改坐标                    */					
		    continue;
		}
		else if ((GucMapStep[cX][cY + 1]-ucStep)>1)   //不同方向  等高值大2
		{
		    cY++;                                                   /*  修改坐标                    */
		    dirnew=1;
		    ucStep++;
		    //dirold = dirnew;
		    continue;
		}
            }
            if (GucMapBlock[cX][cY] & 0x02) {                  /*  右方等高值大于计划设定值    */
		if(dirnew==2 && (GucMapStep[cX+1][cY]-ucStep)>0) //相同方向  等高值大1
		{
		    cX++;                                                   /*  修改坐标                    */					
		    continue;
		}
		else if ((GucMapStep[cX+1][cY]-ucStep)>1)   //不同方向  等高值大2
		{
		    cX++;                                                   /*  修改坐标                    */
		    dirnew=2;
		    ucStep++;
		    //dirold = dirnew;
		    continue;
		}
            }
            if (GucMapBlock[cX][cY] & 0x04) {                  /*  下方等高值大于计划设定值    */
		if(dirnew==4 && (GucMapStep[cX][cY - 1]-ucStep)>0) //相同方向  等高值大1
		{
		    cY--;                                                   /*  修改坐标                    */					
		    continue;
		}
		else if ((GucMapStep[cX][cY - 1]-ucStep)>1)   //不同方向  等高值大2
		{
		    cY--;                                                   /*  修改坐标                    */
		    dirnew=4;
		    ucStep++;
		    //dirold = dirnew;
		    continue;
		}
            }
            if (GucMapBlock[cX][cY] & 0x08) {                  /*  左方等高值大于计划设定值    */
		if(dirnew==8 && (GucMapStep[cX-1][cY]-ucStep)>0) //相同方向  等高值大1
		{
		    cX--;                                                   /*  修改坐标                    */					
		    continue;
		}
		else if ((GucMapStep[cX-1][cY]-ucStep)>1)   //不同方向  等高值大2
		{
		    cX--;                                                  /*  修改坐标                    */
		    dirnew=8;
		    ucStep++;
		    //dirold = dirnew;
		    continue;
		}
            }
        }
    }
}
void smartStack(uint8 n)
{
  if(n<1)//GmcCrossway[0]为起始点  不参与排序
    return;
  uint8 record = 1;
  uint8 length = 0xff;
  
  for(uint8 i = 1;i<=n;i++)
  {
    if(GucMapStep[GmcCrossway[i].cX][GmcCrossway[i].cY]<length)//等高值小
    {
      record = i;
      length = GucMapStep[GmcCrossway[i].cX][GmcCrossway[i].cY];
    }
  }
  
  GmcCrossway[n+1].cX = GmcCrossway[record].cX;//最小值压入栈顶
  GmcCrossway[n+1].cY = GmcCrossway[record].cY;
  
  for(uint8 i = record;i<=n;i++)
  {
    GmcCrossway[i].cX = GmcCrossway[i+1].cX;//依次顺移
    GmcCrossway[i].cY = GmcCrossway[i+1].cY;
  }
}
/*********************************************************************************************************
** Function name:       __ir_Get
** Descriptions:        读取E2PROM中的红外频率
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __ir_Get(void) 
{
  
  uint32    DIS1[10] = {0};
  read_fm24LC16(DIS1,0x00,0xa1,10);
  delay(1000000);
  GusFreq_F=DIS1[0]*1000+DIS1[1]*100;
  GusFreq_X=DIS1[2]*1000+DIS1[3]*100;
  GusFreq_LF=DIS1[4]*1000+DIS1[5]*100;
  GusFreq_L=DIS1[6]*1000+DIS1[7]*100;
  GusFreq_FJ=DIS1[8]*1000+DIS1[9]*100;
}
main (void)
{
    uint8 n          = 0;                                               /*  GmcCrossway[]下标           */
    uint8 ucRoadStat = 0;                                               /*  统计某一坐标可前进的支路数  */
    uint8 ucTemp     = 0;                                               /*  用于START状态中坐标转换     */
    uint8 start_flag =0;
    mouseInit();                                                        /*  底层驱动的初始化            */
    zlg7289Init();                                                      /*  显示模块初始化              */
    __ir_Get();
    delay(10000);
  
    while (1) {
        switch (GucMouseTask) {                                         /*  状态机处理                  */     
        case WAIT:
            sensorDebug();
            delay(100000);
            if(startCheck()==true)
            {
              start_flag=1;
            }
            if(start_flag==1)
            {
               delay(100000);
               if (startCheck2() == true) 
               {                                   /*  检测按键等待启动            */
                zlg7289Reset();                                         /*  复位ZLG7289                 */
                GucMouseTask = START;
               }   
            }
            
            if(keyCheck() == true)
            {
               GucMouseTask = SPURT; 
            }
            break;
            
        case START:                                                     /*  判断电脑鼠起点的横坐标      */
            mazeSearch();                                               /*  向前搜索                    */
            if (GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0x08) {         /*  判断电老鼠左边是否存在出口  */
                if (MAZETYPE == 16) {                                    /*  修改四分之一迷宫的终点坐标  */
                    GucXGoal0 = 8;
                    GucXGoal1 = 7;
                }
                GucXStart   = MAZETYPE - 1;                             /*  修改电脑鼠起点的横坐标      */
                GmcMouse.cX = MAZETYPE - 1;                             /*  修改电脑鼠当前位置的横坐标  */    
                /*
                 *  由于默认的起点为(0,0)，现在需要把已记录的墙壁资料转换过来
                 */
                ucTemp = GmcMouse.cY;
                do {
                    GucMapBlock[MAZETYPE - 1][ucTemp] = GucMapBlock[0][ucTemp];
                    GucMapBlock[0 ][ucTemp] = 0;
                }while (ucTemp--);
                /*
                 *  在OFFSHOOT[0]中保存起点坐标
                 */
                GmcCrossway[n].cX = MAZETYPE - 1;
                GmcCrossway[n].cY = 0;
                n++;
                GucMouseTask = MAZESEARCH;                              /*  状态转换为搜寻状态          */
            }
            if (GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0x02) {         /*  判断电老鼠右边是否存在出口  */
                /*
                 *  在OFFSHOOT[0]中保存起点坐标
                 */
                GmcCrossway[n].cX = 0;
                GmcCrossway[n].cY = 0;
                n++;
                GucMouseTask = MAZESEARCH;                              /*  状态转换为搜寻状态          */
            }
            break;
            
        case MAZESEARCH: 
          if (((GmcMouse.cX==GucXGoal0)&&(GmcMouse.cY==GucYGoal0))||((GmcMouse.cX==GucXGoal0)&&(GmcMouse.cY==GucYGoal1))
           ||((GmcMouse.cX==GucXGoal1)&&(GmcMouse.cY==GucYGoal0))||((GmcMouse.cX==GucXGoal1)&&(GmcMouse.cY==GucYGoal1)))
            /*   判断是否到达终点   */
          {  
             Go_one_grid();
             mouseTurnback();
             objectGoTo(GucXStart,GucYStart);
             mouseTurnback_Y();
             mouseTurnback_correct(900);
             GucMouseTask = SPURT;
             break;
          }          
          else{
//            ucRoadStat = crosswayCheck(GmcMouse.cX,GmcMouse.cY);        /*  统计可前进的支路数          */
//            if (ucRoadStat) 
//            {                                                                                                    /*  有可前进方向                */
//                if (ucRoadStat > 1) {                                   /*  有多条可前进方向，保存坐标  */
//                    GmcCrossway[n].cX = GmcMouse.cX;
//                    GmcCrossway[n].cY = GmcMouse.cY;
//                    n++;
//                }
//                crosswayChoice();                                       /*  用右手法则搜索选择前进方向  */
//                mazeSearch();                                           /*  前进一格                    */
//            } 
//              else 
//             {                                                    /*  没有可前进方向，回到最近支路*/
//               if(crosswayCheck_back(GmcMouse.cX,GmcMouse.cY)==true)
//               {
//                mouseTurnback();
//                mouseTurnback_correct(700);            //向后矫正位置
//                n=n-1;
//                objectGoTo(GmcCrossway[n].cX,GmcCrossway[n].cY);
//               } 
//                
//                ucRoadStat = crosswayCheck(GmcMouse.cX,GmcMouse.cY);
//                if (ucRoadStat > 1) {
//                    GmcCrossway[n].cX = GmcMouse.cX;
//                    GmcCrossway[n].cY = GmcMouse.cY;
//                    n++;     
//                } 
//                if (ucRoadStat==0)
//                {
//                  n=n-1;
//                  objectGoTo(GmcCrossway[n].cX,GmcCrossway[n].cY);
//                }
//                crosswayChoice();
//                mazeSearch();                            
//            }
            
            ucRoadStat = crosswayCheck(GmcMouse.cX,GmcMouse.cY);        /*  统计可前进的支路数          */
            if (ucRoadStat) 
            {                                                           /*  有可前进方向                */
                if (ucRoadStat > 1) {                                   /*  有多条可前进方向，保存坐标  */
                    GmcCrossway[n].cX = GmcMouse.cX;
                    GmcCrossway[n].cY = GmcMouse.cY;
                    n++;
                }
                crosswayChoice();                                       /*  用右手法则搜索选择前进方向  */
                mazeSearch();                                           /*  前进一格                    */
            } 
              else 
             {                                                    /*  没有可前进方向，回到最近支路*/
                if(crosswayCheck_back(GmcMouse.cX,GmcMouse.cY)==true)
                {
                 mouseTurnback();
                 mouseTurnback_correct(700);            //向后矫正位置
                }
                weight_mapStepEdit(GmcMouse.cX,GmcMouse.cY);
                
                while (--n) {
                  smartStack(n);
                  
                    ucRoadStat = crosswayCheck(GmcCrossway[n].cX,
                                               GmcCrossway[n].cY);
                    if (ucRoadStat) {
                        objectGoTo(GmcCrossway[n].cX,
                                   GmcCrossway[n].cY);
                        if (ucRoadStat > 1) {
                            n++;
                        }
                        crosswayChoice();
                        mazeSearch();
                        break;
                    }
                }                     
            }
            
          }
            break;

        case SPURT:
             mouseSpurt();                                          /*  以最优路径冲向终点          */   
             Go_one_grid();
             mouseTurnback();
             objectGoTo(GucXStart,GucYStart);                      /*  回起点          */     
             mouseTurnback_Y();                                            /*  向后转，恢复出发姿势        */    
             mouseTurnback_correct(700);            //向后矫正位置
             while(1)
             {
              if(keyCheck() == true)
              {break;}
             }            
            break;
       
        default:
            break;
        }
    }
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/

/*--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
** Version:
** Description:
**                                  ���ж���stop
*********************************************************************************************************/


/*********************************************************************************************************
  ����ͷ�ļ�
*********************************************************************************************************/
#include "Maze.h"
#include "iic.h"
/*********************************************************************************************************
  ȫ�ֱ�������
*********************************************************************************************************/
static uint32    GucXStart                           = 0;                /*  ��������   ls changed 8 to 32 */
static uint8    GucYStart                           = 0;                /*  ���������                  */
static uint8    GucXGoal0                           = XDST0;            /*  �յ�X���꣬������ֵ         */
static uint8    GucXGoal1                           = XDST1;
static uint8    GucYGoal0                           = YDST0;            /*  �յ�Y���꣬������ֵ         */
static uint8    GucYGoal1                           = YDST1;
static uint8    GucMouseTask                        = WAIT;             /*  ״̬������ʼ״̬Ϊ�ȴ�      */
static uint8    GucMapStep[MAZETYPE][MAZETYPE]      = {0xff};           /*  ���������ĵȸ�ֵ          */
static MAZECOOR GmcStack[MAZETYPE * MAZETYPE]       = {0};              /*  ��mapStepEdit()������ջʹ�� */
static MAZECOOR GmcCrossway[MAZETYPE * MAZETYPE]    = {0};              /*  Main()���ݴ�δ�߹�֧·����  */
/*********************************************************************************************************
** Function name:       Delay
** Descriptions:        ��ʱ����
** input parameters:    uiD :��ʱ������ֵԽ����ʱԽ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void delay (uint32 uiD)
{
    for (; uiD; uiD--);
}

/*********************************************************************************************************
** Function name:       mapStepEdit
** Descriptions:        ������Ŀ���Ϊ���ĵȸ�ͼ
** input parameters:    uiX:    Ŀ�ĵغ�����
**                      uiY:    Ŀ�ĵ�������
** output parameters:   GucMapStep[][]:  �������ϵĵȸ�ֵ
** Returned value:      ��
*********************************************************************************************************/
void mapStepEdit (int8  cX, int8  cY)
{
    uint8 n         = 0;                                                /*  GmcStack[]�±�              */
    uint8 ucStep    = 1;                                                /*  �ȸ�ֵ                      */
    uint8 ucStat    = 0;                                                /*  ͳ�ƿ�ǰ���ķ�����          */
    uint8 i,j;
    
    GmcStack[n].cX  = cX;                                               /*  ���Xֵ��ջ                 */
    GmcStack[n].cY  = cY;                                               /*  ���Yֵ��ջ                 */
    n++;
    /*
     *  ��ʼ��������ȸ�ֵ
     */
    for (i = 0; i < MAZETYPE; i++) {
        for (j = 0; j < MAZETYPE; j++) {
            GucMapStep[i][j] = 0xff;
        }
    }
    /*
     *  �����ȸ�ͼ��ֱ����ջ���������ݴ������
     */
    while (n) {
        GucMapStep[cX][cY] = ucStep++;                                  /*  ����ȸ�ֵ                  */

        /*
         *  �Ե�ǰ��������ǰ���ķ���ͳ��
         */
        ucStat = 0;
        if ((GucMapBlock[cX][cY] & 0x01) &&                             /*  ǰ����·                    */
            (GucMapStep[cX][cY + 1] > (ucStep))) {                      /*  ǰ���ȸ�ֵ���ڼƻ��趨ֵ    */
            ucStat++;                                                   /*  ��ǰ����������1             */
        }
        if ((GucMapBlock[cX][cY] & 0x02) &&                             /*  �ҷ���·                    */
            (GucMapStep[cX + 1][cY] > (ucStep))) {                      /*  �ҷ��ȸ�ֵ���ڼƻ��趨ֵ    */
            ucStat++;                                                   /*  ��ǰ����������1             */
        }
        if ((GucMapBlock[cX][cY] & 0x04) &&
            (GucMapStep[cX][cY - 1] > (ucStep))) {
            ucStat++;                                                   /*  ��ǰ����������1             */
        }
        if ((GucMapBlock[cX][cY] & 0x08) &&
            (GucMapStep[cX - 1][cY] > (ucStep))) {
            ucStat++;                                                   /*  ��ǰ����������1             */
        }
        /*
         *  û�п�ǰ���ķ�������ת���������ķ�֧��
         *  ������ѡһ��ǰ������ǰ��
         */
        if (ucStat == 0) {
            n--;
            cX = GmcStack[n].cX;
            cY = GmcStack[n].cY;
            ucStep = GucMapStep[cX][cY];
        } else {
            if (ucStat > 1) {                                           /*  �ж����ǰ�����򣬱�������  */
                GmcStack[n].cX = cX;                                    /*  ������Xֵ��ջ               */
                GmcStack[n].cY = cY;                                    /*  ������Yֵ��ջ               */
                n++;
            }
            /*
             *  ����ѡ��һ����ǰ���ķ���ǰ��
             */
            if ((GucMapBlock[cX][cY] & 0x01) &&                         /*  �Ϸ���·                    */
                (GucMapStep[cX][cY + 1] > (ucStep))) {                  /*  �Ϸ��ȸ�ֵ���ڼƻ��趨ֵ    */
                cY++;                                                   /*  �޸�����                    */
                continue;
            }
            if ((GucMapBlock[cX][cY] & 0x02) &&                         /*  �ҷ���·                    */
                (GucMapStep[cX + 1][cY] > (ucStep))) {                  /*  �ҷ��ȸ�ֵ���ڼƻ��趨ֵ    */
                cX++;                                                   /*  �޸�����                    */
                continue;
            }
            if ((GucMapBlock[cX][cY] & 0x04) &&                         /*  �·���·                    */
                (GucMapStep[cX][cY - 1] > (ucStep))) {                  /*  �·��ȸ�ֵ���ڼƻ��趨ֵ    */
                cY--;                                                   /*  �޸�����                    */
                continue;
            }
            if ((GucMapBlock[cX][cY] & 0x08) &&                         /*  ����·                    */
                (GucMapStep[cX - 1][cY] > (ucStep))) {                  /*  �󷽵ȸ�ֵ���ڼƻ��趨ֵ    */
                cX--;                                                   /*  �޸�����                    */
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
     *  ���յ���ĸ�����ֱ������ȸ�ͼ
     *  ȡ����������һ������ΪĿ���
     */
    if (GucMapBlock[GucXGoal0][GucYGoal0] & 0x0c) {                     /*  �жϸ��յ������Ƿ��г���    */
        mapStepEdit(GucXGoal0,GucYGoal0);                               /*  �����ȸ�ͼ                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart]) {                /*  ������������������        */
            cXdst  = GucXGoal0;
            cYdst  = GucYGoal0;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal0][GucYGoal1] & 0x09) {                     /*  �жϸ��յ������Ƿ��г���    */
        mapStepEdit(GucXGoal0,GucYGoal1);                               /*  �����ȸ�ͼ                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart]) {                /*  ������������������        */
            cXdst  = GucXGoal0;
            cYdst  = GucYGoal1;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal1][GucYGoal0] & 0x06) {                     /*  �жϸ��յ������Ƿ��г���    */
        mapStepEdit(GucXGoal1,GucYGoal0);                               /*  �����ȸ�ͼ                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart]) {                /*  ������������������        */
            cXdst  = GucXGoal1;
            cYdst  = GucYGoal0;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    if (GucMapBlock[GucXGoal1][GucYGoal1] & 0x03) {                     /*  �жϸ��յ������Ƿ��г���    */
        mapStepEdit(GucXGoal1,GucYGoal1);                               /*  �����ȸ�ͼ                  */
        if (ucTemp > GucMapStep[GucXStart][GucYStart]) {                /*  ������������������        */
            cXdst  = GucXGoal1;
            cYdst  = GucYGoal1;
            ucTemp = GucMapStep[GucXStart][GucYStart];
        }
    }
    
    objectGoTo(cXdst,cYdst);                                            /*  ���е�ָ��Ŀ���            */
    
}

void objectGoTo(int8  cXdst, int8  cYdst)
{
    uint8 ucStep = 1;
    int8  cNBlock = 0, cDirTemp;
    int8 cX,cY;
    GucCrossroad=0;
    cX = GmcMouse.cX;
    cY = GmcMouse.cY;
    mapStepEdit(cXdst,cYdst);                                           /*  �����ȸ�ͼ                  */
    
    /*
     *  ���ݵȸ�ֵ��Ŀ����˶���ֱ���ﵽĿ�ĵ�
     */
    while ((cX != cXdst) || (cY != cYdst)) {
        
        ucStep = GucMapStep[cX][cY];
        /*
         *  ��ѡһ���ȸ�ֵ�ȵ�ǰ����ȸ�ֵС�ķ���ǰ��
         */
        if ((GucMapBlock[cX][cY] & 0x01) &&                             /*  �Ϸ���·                    */
            (GucMapStep[cX][cY + 1] < ucStep)) {                        /*  �Ϸ��ȸ�ֵ��С              */
            cDirTemp = UP;                                              /*  ��¼����                    */
            if (cDirTemp == GucMouseDir) {                              /*  ����ѡ����Ҫת��ķ���    */
                cNBlock++;                                              /*  ǰ��һ������                */
                cY++;
                if((GucMapBlock[cX][cY] & 0x0f)==0x0f)
                  GucCrossroad++;
                continue;                                               /*  ��������ѭ��                */
            }
        }
        if ((GucMapBlock[cX][cY] & 0x02) &&                             /*  �ҷ���·                    */
            (GucMapStep[cX + 1][cY] < ucStep)) {                        /*  �ҷ��ȸ�ֵ��С              */
            cDirTemp = RIGHT;                                           /*  ��¼����                    */
            if (cDirTemp == GucMouseDir) {                              /*  ����ѡ����Ҫת��ķ���    */
                cNBlock++;                                              /*  ǰ��һ������                */
                cX++;
                if((GucMapBlock[cX][cY] & 0x0f)==0x0f)
                  GucCrossroad++;
                continue;                                               /*  ��������ѭ��                */
            }
        }
        if ((GucMapBlock[cX][cY] & 0x04) &&                             /*  �·���·                    */
            (GucMapStep[cX][cY - 1] < ucStep)) {                        /*  �·��ȸ�ֵ��С              */
            cDirTemp = DOWN;                                            /*  ��¼����                    */
            if (cDirTemp == GucMouseDir) {                              /*  ����ѡ����Ҫת��ķ���    */
                cNBlock++;                                              /*  ǰ��һ������                */
                cY--;
                if((GucMapBlock[cX][cY] & 0x0f)==0x0f)
                  GucCrossroad++;
                continue;                                               /*  ��������ѭ��                */
            }
        }
        if ((GucMapBlock[cX][cY] & 0x08) &&                             /*  ����·                    */
            (GucMapStep[cX - 1][cY] < ucStep)) {                        /*  �󷽵ȸ�ֵ��С              */
            cDirTemp = LEFT;                                            /*  ��¼����                    */
            if (cDirTemp == GucMouseDir) {                              /*  ����ѡ����Ҫת��ķ���    */
                cNBlock++;                                              /*  ǰ��һ������                */
                cX--;
                if((GucMapBlock[cX][cY] & 0x0f)==0x0f)
                  GucCrossroad++;
                continue;                                               /*  ��������ѭ��                */
            }
        }
        cDirTemp = (cDirTemp + 4 - GucMouseDir)%4;                      /*  ���㷽��ƫ����              */
        GucDirTemp = cDirTemp;
        if (cNBlock) {
          if((GucCrossroad <= 1)&&(cNBlock>1))
              mouseGoahead(cNBlock);                                      /*  ǰ��cNBlock��               */
          else{
            mouseGoahead_Llow(cNBlock);
            GucCrossroad = 0;
          }
          
          
        }        
        cNBlock = 0;  
        /*  ��������                    */
        
        /*
         *  ���Ƶ�����ת��
         */
         
       switch (cDirTemp) {

        case 1:
            mouseTurnright();
            break;

        case 2:
            mouseTurnback();
            break;

        case 3:
            mouseTurnleft();
            break;

        default:
            break;
        }
      GmcMouse.cX=cX;
      GmcMouse.cY=cY;
    }
    /*
     *  �ж������Ƿ���ɣ��������ǰ��
     */
    
      if (cNBlock) {
          if((GucCrossroad <= 1)&&(cNBlock>1))
            mouseGoahead(cNBlock);                                      /*  ǰ��cNBlock��               */
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
** Descriptions:        ���ݵ��������Է���ȡ���÷������Թ����ǽ������
** input parameters:    ucDir: ���������Է���
** output parameters:   ��
** Returned value:      GucMapBlock[cX][cY] : ǽ������
*********************************************************************************************************/
uint8 mazeBlockDataGet (uint8  ucDirTemp)
{
    int8 cX     = 0,cY = 0;
    
    /*
     *  �ѵ��������Է���ת��Ϊ���Է���
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
     *  ���ݾ��Է������÷��������ڸ������
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
    
    return(GucMapBlock0[cX][cY]);                                        /*  �����Թ����ϵ�����          */
}
/*********************************************************************************************************
** Function name:       rightMethod
** Descriptions:        ���ַ�����������ǰ��
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void rightMethod (void)
{
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  ��������ұ���·            */
        (mazeBlockDataGet(MOUSERIGHT) == 0x00)) {                       /*  ��������ұ�û���߹�        */
        mouseTurnright();                                               /*  ��������ת                  */
        return;
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  �������ǰ����·            */
        (mazeBlockDataGet(MOUSEFRONT) == 0x00)) {                       /*  �������ǰ��û���߹�        */
        return;                                                         /*  ��������ת��              */
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  ������������·            */
        (mazeBlockDataGet(MOUSELEFT ) == 0x00)) {                       /*  ����������û���߹�        */
        mouseTurnleft();                                                /*  ��������ת                  */
        return;
    }
}
/*********************************************************************************************************
** Function name:       leftMethod
** Descriptions:        ���ַ������������˶�
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void leftMethod (void)
{
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  ������������·            */
        (mazeBlockDataGet(MOUSELEFT ) == 0x00)) {                       /*  ����������û���߹�        */
        mouseTurnleft();                                                /*  ��������ת                  */
        return;
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  �������ǰ����·            */
        (mazeBlockDataGet(MOUSEFRONT) == 0x00)) {                       /*  �������ǰ��û���߹�        */
        return;                                                         /*  ��������ת��              */
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  ��������ұ���·            */
        (mazeBlockDataGet(MOUSERIGHT) == 0x00)) {                       /*  ��������ұ�û���߹�        */
        mouseTurnright();                                               /*  ��������ת                  */
        return;
    }
}
/*********************************************************************************************************
** Function name:       frontRightMethod
** Descriptions:        ���ҷ���������ǰ���У��������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void frontRightMethod (void)
 {
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  �������ǰ����·            */
        (mazeBlockDataGet(MOUSEFRONT) == 0x00)) {                       /*  �������ǰ��û���߹�        */
        
        return;                                                         /*  ��������ת��              */
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  ��������ұ���·            */
        (mazeBlockDataGet(MOUSERIGHT) == 0x00)) {                       /*  ��������ұ�û���߹�        */
        mouseTurnright();                                               /*  ��������ת                  */
        return;
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  ������������·            */
        (mazeBlockDataGet(MOUSELEFT ) == 0x00)) {                       /*  ����������û���߹�        */
        mouseTurnleft();                                                /*  ��������ת                  */
        return;
    }
}
/*********************************************************************************************************
** Function name:       frontLeftMethod
** Descriptions:        ������������ǰ���У��������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void frontLeftMethod (void)
{
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&         /*  �������ǰ����·            */
        (mazeBlockDataGet(MOUSEFRONT) == 0x00)) {                       /*  �������ǰ��û���߹�        */
        return;                                                         /*  ��������ת��              */
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&         /*  ������������·            */
        (mazeBlockDataGet(MOUSELEFT ) == 0x00)) {                       /*  ����������û���߹�        */
        mouseTurnleft();                                                /*  ��������ת                  */
        return;
    }
    if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&         /*  ��������ұ���·            */
        (mazeBlockDataGet(MOUSERIGHT) == 0x00)) {                       /*  ��������ұ�û���߹�        */
        mouseTurnright();                                               /*  ��������ת                  */
        return;
    }
}

/*********************************************************************************************************
** Function name:       centralMethod
** Descriptions:        ���ķ��򣬸��ݵ�����Ŀǰ���Թ���������λ�þ���ʹ�ú�����������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void centralMethod (void)
{
    if (GmcMouse.cX & 0x08) {
        if (GmcMouse.cY & 0x08) {

            /*
             *  ��ʱ���������Թ������Ͻ�
             */ 
            switch (GucMouseDir) {
                
            case UP:                                                    /*  ��ǰ����������              */
                leftMethod();                                           /*  ���ַ���                    */
                break;

            case RIGHT:                                                 /*  ��ǰ����������              */
                rightMethod();                                          /*  ���ַ���                    */
                break;

            case DOWN:                                                  /*  ��ǰ����������              */
                frontRightMethod();                                     /*  ���ҷ���                    */
                break;

            case LEFT:                                                  /*  ��ǰ����������              */
                frontLeftMethod();                                      /*  ������                    */
                break;

            default:
                break;
            }
        } else {

            /*
             *  ��ʱ���������Թ������½�
             */    
            switch (GucMouseDir) {
                
            case UP:                                                    /*  ��ǰ����������              */
                leftMethod();//frontLeftMethod();                                      /*  ������                    */
                break;

            case RIGHT:                                                 /*  ��ǰ����������              */
                leftMethod();                                           /*  ���ַ���                    */
                break;

            case DOWN:                                                  /*  ��ǰ����������              */
                rightMethod();                                          /*  ���ַ���                    */
                break;

            case LEFT:                                                  /*  ��ǰ����������              */
                frontRightMethod();                                     /*  ���ҷ���                    */
                break;

            default:
                break;
            }
        }
    } else {
        if (GmcMouse.cY & 0x08) {

            /*
             *  ��ʱ���������Թ������Ͻ�
             */    
            switch (GucMouseDir) {
                
            case UP:                                                    /*  ��ǰ����������              */
                rightMethod();                                          /*  ���ַ���                    */
                break;

            case RIGHT:                                                 /*  ��ǰ����������              */
                frontRightMethod();                                     /*  ���ҷ���                    */
                break;

            case DOWN:                                                  /*  ��ǰ����������              */
                frontLeftMethod();                                      /*  ������                    */
                break;

            case LEFT:                                                  /*  ��ǰ����������              */
                leftMethod();                                           /*  ���ַ���                    */
                break;

            default:
                break;
            }
        } else {

            /*
             *  ��ʱ���������Թ������½�
             */    
            switch (GucMouseDir) {
                
            case UP:                                                    /*  ��ǰ����������              */
                frontRightMethod();                                     /*  ���ҷ���                    */
                break;

            case RIGHT:                                                 /*  ��ǰ����������              */
                frontLeftMethod();                                      /*  ������                    */
                break;

            case DOWN:                                                  /*  ��ǰ����������              */
                leftMethod();                                           /*  ���ַ���                    */
                break;

            case LEFT:                                                  /*  ��ǰ����������              */
                rightMethod();                                          /*  ���ַ���                    */
                break;

            default:
                break;
            }
        }
    }
}
/*********************************************************************************************************
** Function name:       crosswayCheck
** Descriptions:        ͳ��ĳ������ڻ�δ�߹���֧·��
** input parameters:    ucX����Ҫ����ĺ�����
**                      ucY����Ҫ�����������
** output parameters:   ��
** Returned value:      ucCt��δ�߹���֧·��
*********************************************************************************************************/
uint8 crosswayCheck (int8  cX, int8  cY)
{
    uint8 ucCt = 0;
    if ((GucMapBlock[cX][cY] & 0x01) &&                                 /*  ���Է����Թ��Ϸ���·      */
        (GucMapBlock0[cX][cY + 1]) == 0x00) {                            /*  ���Է����Թ��Ϸ�δ�߹�    */
        ucCt++;                                                         /*  ��ǰ����������1             */
    }
    if ((GucMapBlock[cX][cY] & 0x02) &&                                 /*  ���Է����Թ��ҷ���·      */
        (GucMapBlock0[cX + 1][cY]) == 0x00) {                            /*  ���Է����Թ��ҷ�û���߹�  */
        ucCt++;                                                         /*  ��ǰ����������1             */
    }
    if ((GucMapBlock[cX][cY] & 0x04) &&                                 /*  ���Է����Թ��·���·      */
        (GucMapBlock0[cX][cY - 1]) == 0x00) {                            /*  ���Է����Թ��·�δ�߹�    */
        ucCt++;                                                         /*  ��ǰ����������1             */
    }
    if ((GucMapBlock[cX][cY] & 0x08) &&                                 /*  ���Է����Թ�����·      */
        (GucMapBlock0[cX - 1][cY]) == 0x00) {                            /*  ���Է����Թ���δ�߹�    */
        ucCt++;                                                         /*  ��ǰ����������1             */
    }
    return ucCt;
}
/*********************************************************************************************************
** Function name:       crosswayChoice
** Descriptions:        ѡ��һ��֧·��Ϊǰ������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
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
/*********************************************************************************************************
** Function name:       __ir_Get
** Descriptions:        ��ȡE2PROM�еĺ���Ƶ��
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
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
    uint8 n          = 0;                                               /*  GmcCrossway[]�±�           */
    uint8 ucRoadStat = 0;                                               /*  ͳ��ĳһ�����ǰ����֧·��  */
    uint8 ucTemp     = 0;                                               /*  ����START״̬������ת��     */
    uint8 Start_1    = 0;
   
    mouseInit();                                                        /*  �ײ������ĳ�ʼ��            */
    zlg7289Init();                                                      /*  ��ʾģ���ʼ��              */
    __ir_Get();
    delay(10000);
    while (1) {
        switch (GucMouseTask) {                                         /*  ״̬������                  */     
        case WAIT:
            sensorDebug();
            delay(100000);
            if (startCheck() == true) {                                 /*  ��ⰴ���ȴ�����            */
              Start_1=1;
            }
            if(Start_1 && GucMouseStart)
            {
              Start_1=0; 
              GucMouseTask = START;
              zlg7289Reset();                                         /*  ��λZLG7289                 */
              while(GucMouseStart);
              delay(1000000);
              __keyIntDisable ();     
            }
            break;
            
        case START:                                                     /*  �жϵ��������ĺ�����      */
            mazeSearch();                                               /*  ��ǰ����                    */
            if (GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0x08) {         /*  �жϵ���������Ƿ���ڳ���  */
                if (MAZETYPE == 16) {                                    /*  �޸��ķ�֮һ�Թ����յ�����  */
                    GucXGoal0 = 8;
                    GucXGoal1 = 7;
                }
                GucXStart   = MAZETYPE - 1;                             /*  �޸ĵ��������ĺ�����      */
                GmcMouse.cX = MAZETYPE - 1;                             /*  �޸ĵ�����ǰλ�õĺ�����  */    
                /*
                 *  ����Ĭ�ϵ����Ϊ(0,0)��������Ҫ���Ѽ�¼��ǽ������ת������
                 */
                ucTemp = GmcMouse.cY;
                do {
                     GucMapBlock[MAZETYPE - 1][ucTemp] = GucMapBlock[0][ucTemp];
                    GucMapBlock0[MAZETYPE - 1][ucTemp] = GucMapBlock0[0][ucTemp];
                    GucMapBlock0[0][ucTemp]=0;
                    GucMapBlock1[MAZETYPE - 1][ucTemp] = GucMapBlock1[0][ucTemp];    
                    if(ucTemp>0){ GucMapBlock1[MAZETYPE - 2][ucTemp-1]=0x1d;}
                    GucMapBlock1[0 ][ucTemp+1] =0x17;
                    GucMapBlock1[1 ][ucTemp] =0x1f;
                    GucMapBlock[0 ][ucTemp] =0x10;
                    GucMapBlock[1 ][ucTemp] =0x10;
                }while (ucTemp--);
                 GucMapBlock1[0][0] =0x13;
                    GucMapBlock1[1][0] =0x1b;
                    GucMapBlock1[MAZETYPE - 2][0] =0x19;
                /*
                 *  ��OFFSHOOT[0]�б����������
                 */
                GmcCrossway[n].cX = MAZETYPE - 1;
                GmcCrossway[n].cY = 0;
                n++;
                GucMouseTask = MAZESEARCH;                              /*  ״̬ת��Ϊ��Ѱ״̬          */
            }
            if (GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0x02) {         /*  �жϵ������ұ��Ƿ���ڳ���  */
                /*
                 *  ��OFFSHOOT[0]�б����������
                 */
                GmcCrossway[n].cX = 0;
                GmcCrossway[n].cY = 0;
                n++;
                GucMouseTask = MAZESEARCH;                              /*  ״̬ת��Ϊ��Ѱ״̬          */
            }
            write_fm24LC16(&GucXStart,0x80,0xa0,1); 
            break;
            
        case MAZESEARCH:
          if (((GmcMouse.cX==GucXGoal0)&&(GmcMouse.cY==GucYGoal0))||((GmcMouse.cX==GucXGoal0)&&(GmcMouse.cY==GucYGoal1))
           ||((GmcMouse.cX==GucXGoal1)&&(GmcMouse.cY==GucYGoal0))||((GmcMouse.cX==GucXGoal1)&&(GmcMouse.cY==GucYGoal1)))
          {    
             mouseTurnback();
             objectGoTo(GucXStart,GucYStart);
             mouseTurnback_Y();  
             GucMouseTask = SPURT;
             break;
          }
          else{
            ucRoadStat = crosswayCheck(GmcMouse.cX,GmcMouse.cY);        /*  ͳ�ƿ�ǰ����֧·��          */
            if (ucRoadStat) 
            {                                                           /*  �п�ǰ������                */
                if (ucRoadStat > 1) {                                   /*  �ж�����ǰ�����򣬱�������  */
                    GmcCrossway[n].cX = GmcMouse.cX;
                    GmcCrossway[n].cY = GmcMouse.cY;
                    n++;
                }
                crosswayChoice();                                       /*  �����ַ�������ѡ��ǰ������  */
                mazeSearch();                                           /*  ǰ��һ��                    */
            }
            else if(ucRoadStat==1){
              crosswayChoice();
              mazeSearch();
            }
            //�ָ���
            //����  ����ɾ�����ж�ucRoadStat=1���������Ϊ������ж��Ѿ�������������ж�ucRoadStat=1���ظ�ִ��crosswayChoice��mazeSearch
              else 
             {                                                    /*  û�п�ǰ�����򣬻ص����֧·*/
                   
 //              while(--n){                                        //���� ����while�ж���һ����·�ڷ������������Ѿ��߹��Ǿͼ���������
   //              ucRoadStat = crosswayCheck(GmcCrossway[n].cX,GmcCrossway[n].cY);  
                 
     //            if(ucRoadStat){
       //             objectGoTo(GmcCrossway[n].cX,GmcCrossway[n].cY);
         //             if(ucRoadStat > 1){
           //             n++;
             //          }
               //     crosswayChoice();
                 //   mazeSearch();
                   // break;
//                  }
  //             }
               mouseTurnback();
               n=n-1;
               objectGoTo(GmcCrossway[n].cX,GmcCrossway[n].cY);
               
               ucRoadStat = crosswayCheck(GmcMouse.cX,GmcMouse.cY);
               if(ucRoadStat > 1){
                 GmcCrossway[n].cX=GmcMouse.cX;
                 GmcCrossway[n].cY=GmcMouse.cY;
                 n++;
               }
               crosswayChoice();
               mazeSearch();
               
               if(n == 0){                                      //���� �����ж�ջ���Ƿ�ֻʣ��������꣬�������˵�����������������
               
                    objectGoTo(GmcCrossway[0].cX,GmcCrossway[0].cY);
                    mouseTurnback();
                    GucMouseTask = SPURT;
                 
               }
               
            }//else
          }
            break;

        case SPURT:
             mouseSpurt();                                          /*  ������·�������յ�          */  
             mouseTurnback();
             objectGoTo(GucXStart,GucYStart);                      /*  �����          */     
             mouseTurnback_Y();                              /*  ���ת���ָ���������        */
            while (1) 
            {
                if (startCheck() == true)
                {
                    break;
                }
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

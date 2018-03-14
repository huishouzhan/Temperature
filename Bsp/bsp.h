/**
  *****************************************************************************
	*                                uC/OS-III
	*
  *                            ARM Cortex-M3 Port  
  *	
  * @file:    bsp.c
  * @author:  zhu
  * @version: 1.0
  * @date:    2016.07.18
  * @brief:   STM32F10x 系统设置
	*           1.中断优先级模式
	*           2.程序启动指定
	*           3.时钟计时器
	*           4.引脚初始化
*******************************************************************************
  * @attention:
  *        
  *        
  *        
  *
  *****************************************************************************
  */

#ifndef _bsp_h_
#define _bsp_h_
/* Includes ---------------------------------------*/
/* -----      -------------------------------------*/
#include "os.h" 
#include "stm32f10x.h"
#include "mb.h"

/* Defines ----------------------------------------*/
/* -----      -------------------------------------*/
#define   sysTick_1ms     SystemCoreClock/1000
#define   sysTick_5ms     SystemCoreClock/200

/* IO ---------------------------------------------*/
   /***********  IO bitBand  ***********/
#define   BITBAND(addr, bitnum)   ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define   MEM_ADDR(addr)    *((volatile CPU_INT32U *)(addr)) 
#define   BIT_ADDR(addr, bitnum)     MEM_ADDR(BITBAND(addr, bitnum)) 
   /***********  IO AddressMap  ***********/
#define   GPIOA_ODR_Addr    (GPIOA_BASE+12) 
#define   GPIOB_ODR_Addr    (GPIOB_BASE+12) 
#define   GPIOC_ODR_Addr    (GPIOC_BASE+12) 
#define   GPIOD_ODR_Addr    (GPIOD_BASE+12) 
#define   GPIOE_ODR_Addr    (GPIOE_BASE+12) 
#define   GPIOF_ODR_Addr    (GPIOF_BASE+12) 
#define   GPIOG_ODR_Addr    (GPIOG_BASE+12)   

#define   GPIOA_IDR_Addr    (GPIOA_BASE+8) 
#define   GPIOB_IDR_Addr    (GPIOB_BASE+8)
#define   GPIOC_IDR_Addr    (GPIOC_BASE+8) 
#define   GPIOD_IDR_Addr    (GPIOD_BASE+8) 
#define   GPIOE_IDR_Addr    (GPIOE_BASE+8) 
#define   GPIOF_IDR_Addr    (GPIOF_BASE+8)
#define   GPIOG_IDR_Addr    (GPIOG_BASE+8) 
 
#define   PAo(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 n<16
#define   PAi(n)   BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 
#define   PBo(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  
#define   PBi(n)   BIT_ADDR(GPIOB_IDR_Addr,n)  
#define   PCo(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  
#define   PCi(n)   BIT_ADDR(GPIOC_IDR_Addr,n)  
#define   PDo(n)   BIT_ADDR(GPIOD_ODR_Addr,n) 
#define   PDi(n)   BIT_ADDR(GPIOD_IDR_Addr,n)  
#define   PEo(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  
#define   PEi(n)   BIT_ADDR(GPIOE_IDR_Addr,n) 
#define   PFo(n)   BIT_ADDR(GPIOF_ODR_Addr,n)
#define   PFi(n)   BIT_ADDR(GPIOF_IDR_Addr,n)  
#define   PGo(n)   BIT_ADDR(GPIOG_ODR_Addr,n) 
#define   PGi(n)   BIT_ADDR(GPIOG_IDR_Addr,n) 

/* LED ---------------------------------------------*/
#define   ledOn      0
#define   ledOff     1
#define   led        PBo(9)

#define   LED       PBo(0)


/* BEEP --------------------------------------------*/
#define  beepOn     1
#define  beepOff    0

#define  beep      PBo(11)

/* USART -------------------------------------------*/
#define   usartPort1       1
#define   usartPort2       2
#define   usartPort3       3
#define   usartPort4       4

/* Motor -----------------------------------------*/
#define   runStop      0
#define   runOpen      1
#define   runClose     2

#define   doorOpen     1
#define   doorClose    2
#define   doorOpening   PBo(10)=1,PBo(1)=0;
#define   doorCloseing  PBo(10)=0,PBo(1)=1;
#define   doorStoping   PBo(10)=0,PBo(1)=0;

#define   fDoorOpenStatus   0x01
#define   fDoorCloseStatus  0x02

/* ADC  ------------------------------------------*/
#define   fdoorAdcCh0   0

/* Light Curtain ---------------------------------*/
#define  gmFeedback    PAi(12)

/* usRegHoldingBuf[] -----------------------------*/
#define  beepVal            0
#define  fdoorVal           13
#define  commuVal           15
#define  apDebugVal         16
#define  emerVal            18

/* mStatus  -------------------------------------*/
#define  mIdleStatus        1     //货道空闲状态
#define  mOutgoodsStatus    2     //货道掉货状态
#define  mOutOKStatus       3     //掉货完成状态
#define  mServoOutStatus    4     //货斗上货状态
#define  mFdoorOpenStatus   5     //开前门状态
#define  mFinishStatus      6     //客户取货完成状态
#define  mErrorStatus       7     //关门异常状态
#define  getGoodsFailed     8     //取货失败

/* Extern -----------------------------------------*/
extern uint16_t usRegInputBuf[inputRegNum];
extern uint16_t usRegHoldingBuf[holdingRegNum];
extern OS_SEM  beepSem;

/* Function ---------------------------------------*/
/* -----      -------------------------------------*/
void sysTickInit(CPU_INT32U osTick);
void nvicConfiguration(CPU_INT32U nvicGroup);
void ledConfiguration(void);
void beepConfiguration(void);
void beepAlarm(u8 alarmNum,u16 alarmTime);
u32  hexToBcd(u16 hexValue);
void motorPortConfiguration(void);
void motorRun(u8 runStatus);
void timer3Configuration(u16 arr,u16 psc);
void adcConfiguration(void);
u16  getAdcVal(u8 chx);
u16  adcFilter(u8 chx);
void usartConfiguration(u8 usartPort,u32 baudRate);
void usartPortSendData(USART_TypeDef* USARTx,u8 sendData);
void usartPortSendStr(USART_TypeDef* USARTx,char *sendStr);

void bspInit(void);





#endif

/****************************   (C) COPYRIGHT 2016 Zhu    ****************************/

                /********************  END OF FILE  ********************/



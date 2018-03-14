/**
  *****************************************************************************
	*                                uC/OS-III
	*
  *                            ARM Cortex-M3 Port  
  *	
  * @file:    app.c
  * @author:  zhu
  * @version: 1.0
  * @date:    2016.11.03
  * @brief:   
  *****************************************************************************
  * @attention:
  *        
  *        
  *        
  *
  *****************************************************************************
  */  

/* Includes ---------------------------------------*/
/* -----      -------------------------------------*/
#include "bsp.h"
#include "App.h"
#include "myiic.h"
#include <stdio.h>
#include <string.h>
#include "stmflash.h"

/************************** 任务定义 *********************/
	
/* Start Task -------------------------------------*/
#define     startTaskPrio        3          //任务优先级
#define     startStkSize         128        //任务堆栈大小	
OS_TCB      startTaskTcb;                   //任务控制块
CPU_STK     startTaskStk[startStkSize];     //任务堆栈
void startTask(void);                       //任务函数 	

/* LED Task ----------------------------------------*/
#define     ledTaskPrio     20            
#define     ledStkSize      128            
OS_TCB      ledTaskTcb;                   
CPU_STK     ledTaskStk[ledStkSize];       
void ledTask(void); 
/* Temp Task------------------------------------------*/
#define    TempTaskPrio     19
#define    TempStkSize     128
OS_TCB     TempTaskTcb;
CPU_STK    TempTaskStk[TempStkSize];
void TempTask(void);
/* 485 Task-------------------------------------------*/
#define Usart1TaskPrio      21
#define Usart1StkSize       128
OS_TCB  Usart1TaskTcb;
CPU_STK Usart1TaskStk[Usart1StkSize];
void Usart1Task(void);

OS_SEM usartSem;

#define USART_REC_LEN 100
extern u16 USART_RX_STA;
extern u8 USART_RX_BUF[USART_REC_LEN];

/* Function ---------------------------------------*/
/* -----      -------------------------------------*/
/**
  * @function: main.c.
  * @brief: 程序入口.
  * @param:  None.
  * @retval: 0.
  */
int main()
{ 
  OS_ERR err; 
  CPU_SR_ALLOC();
	
	bspInit();
	
	IIC_Init();
	
	CPU_Init();
	OSInit( (OS_ERR *)&err );
	
  OS_CRITICAL_ENTER();    //进入临界区
	//创建开始任务
	OSTaskCreate((OS_TCB 	   *)&startTaskTcb,		  //任务控制块
				       (CPU_CHAR	 *)"startTask", 		  //任务名字
							 (OS_TASK_PTR )startTask, 			  //任务函数
							 (void		   *)0,					        //传递给任务函数的参数
							 (OS_PRIO	    )startTaskPrio,     //任务优先级
							 (CPU_STK    *)&startTaskStk[0],  //任务堆栈基地址
							 (CPU_STK_SIZE)startStkSize/10,	  //任务堆栈深度限位
							 (CPU_STK_SIZE)startStkSize,		  //任务堆栈大小
							 (OS_MSG_QTY  )0,				        	//任务内部消息队列能够接收的最大消息数目,为0时禁止接收消息
							 (OS_TICK	    )0,					        //当使能时间片轮转时的时间片长度，为0时为默认长度，
							 (void   	   *)0,				        	//用户补充的存储区
							 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //任务选项
							 (OS_ERR 	   *)&err);			      	//存放该函数错误时的返回值
						 
  OS_CRITICAL_EXIT();	    //退出临界区	    
	
	OSStart( (OS_ERR *)&err );
	while(1);	
//	return 0; 
}

/**
  * @function: TaskStart.
  * @brief:  启动任务.
  * @param:  None.
  * @retval: None.
  */  
void startTask(void)
{
  OS_ERR 	err;
	
#if OS_CFG_STAT_TASK_EN > 0u
	OSStatTaskCPUUsageInit(&err);  	   //统计任务                
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN		     //如果使能了测量中断关闭时间
	CPU_IntDisMeasMaxCurReset();	
#endif
	
#if	OS_CFG_SCHED_ROUND_ROBIN_EN                 //当使用时间片轮转的时候
	OSSchedRoundRobinCfg(DEF_ENABLED,5,&err);     //使能时间片轮转调度功能,时间片长度为5个系统时钟节拍，既1*5=5ms
#endif
	/******************  创建指示灯任务  ******************/     
	OSTaskCreate((OS_TCB 	   *)&ledTaskTcb,		
				       (CPU_CHAR	 *)"ledTask", 		
							 (OS_TASK_PTR )ledTask, 			
							 (void		   *)0,					
							 (OS_PRIO	    )ledTaskPrio,     
							 (CPU_STK    *)&ledTaskStk[0],	
							 (CPU_STK_SIZE)ledStkSize/10,	
							 (CPU_STK_SIZE)ledStkSize,		
							 (OS_MSG_QTY  )0,					
							 (OS_TICK	    )0,					
							 (void   	   *)0,					
							 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
							 (OS_ERR 	    *)&err);
							 
	OSTaskCreate((OS_TCB 	   *)&TempTaskTcb,		
				       (CPU_CHAR	 *)"TempTask", 		
							 (OS_TASK_PTR )TempTask, 			
							 (void		   *)0,					
							 (OS_PRIO	    )TempTaskPrio,     
							 (CPU_STK    *)&TempTaskStk[0],	
							 (CPU_STK_SIZE)TempStkSize/10,	
							 (CPU_STK_SIZE)TempStkSize,		
							 (OS_MSG_QTY  )0,					
							 (OS_TICK	    )0,					
							 (void   	   *)0,					
							 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
							 (OS_ERR 	    *)&err);
	OSTaskCreate((OS_TCB 	   *)&Usart1TaskTcb,		
				       (CPU_CHAR	 *)"Usart1Task", 		
							 (OS_TASK_PTR )Usart1Task, 			
							 (void		   *)0,					
							 (OS_PRIO	    )Usart1TaskPrio,     
							 (CPU_STK    *)&Usart1TaskStk[0],	
							 (CPU_STK_SIZE)Usart1StkSize/10,	
							 (CPU_STK_SIZE)Usart1StkSize,		
							 (OS_MSG_QTY  )0,					
							 (OS_TICK	    )0,					
							 (void   	   *)0,					
							 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
							 (OS_ERR 	    *)&err);				 
							 
	/****************** 创建任务信号量 ******************/ 
						 

	OSSemCreate((OS_SEM   *)&usartSem,
							(CPU_CHAR *)"usartSem",
							(OS_SEM_CTR)0,
							(OS_ERR   *)&err);
  OS_TaskSuspend((OS_TCB*)&startTaskTcb,&err);		//挂起开始任务					 
}

/**
  * @function: .
  * @brief: .
  * @param: None.
  * @retval: None.
  */

#define FLASH_SAVE_ADDR  0x08100000 				//设置FLASH 保存地址(必须为偶数)
int Min, Max, calibrationval, errorval;

void Usart1Task(void)
{
	OS_ERR err;
	u8 len = 0;
	u8 buf[10];

	STMFLASH_Read(FLASH_SAVE_ADDR,(u16 *)&Min,2);
	STMFLASH_Read(FLASH_SAVE_ADDR+4,(u16 *)&Max,2);
	STMFLASH_Read(FLASH_SAVE_ADDR+8,(u16 *)&calibrationval,2);
	STMFLASH_Read(FLASH_SAVE_ADDR+12,(u16 *)&errorval,2);
//	GPIO_SetBits(GPIOA, GPIO_Pin_8);
//	usartPortSendStr(USART1, "test\r\n");
	while(1)
	{
		/* 使能接收 */
		GPIO_ResetBits(GPIOA, GPIO_Pin_8);
		/* 等待接收到数据 */
		OSSemPend(&usartSem,NULL,OS_OPT_PEND_BLOCKING,NULL,&err);
		len=USART_RX_STA&0x3FFF;
		if (strncmp((char *)USART_RX_BUF,"min=",4) == 0)
		{
			Min = (USART_RX_BUF[4]-'0') * 10 + (USART_RX_BUF[5] - '0');
			STMFLASH_Write(FLASH_SAVE_ADDR,(u16*)&Min,2);
		}
		else if (strncmp((char *)USART_RX_BUF,"max=",4) == 0)
		{
			Max = (USART_RX_BUF[4]-'0') * 10 + (USART_RX_BUF[5] - '0');
			STMFLASH_Write(FLASH_SAVE_ADDR,(u16*)&Max,2);
		}
		else if (strncmp((char *)USART_RX_BUF,"CAV=",4) == 0)
		{
			calibrationval= (USART_RX_BUF[4]-'0') * 10 + (USART_RX_BUF[5] - '0');
			STMFLASH_Write(FLASH_SAVE_ADDR,(u16*)&calibrationval,2);
		}
		else if (strncmp((char *)USART_RX_BUF,"ERR=",4) == 0)
		{
			errorval = (USART_RX_BUF[4]-'0') * 10 + (USART_RX_BUF[5] - '0');
			STMFLASH_Write(FLASH_SAVE_ADDR,(u16*)&errorval,2);
		}
		
		else if (strncmp((char *)USART_RX_BUF,"min",3) == 0)
		{
			GPIO_SetBits(GPIOA, GPIO_Pin_8);
			sprintf((char *)buf, "min=%d\r\n", Min);
			usartPortSendStr(USART1, (char *)buf);
			usartPortSendStr(USART2, (char *)buf);
		}
		else if (strncmp((char *)USART_RX_BUF,"max",3) == 0)
		{
			GPIO_SetBits(GPIOA, GPIO_Pin_8);
			sprintf((char *)buf, "max=%d\r\n", Max);
			usartPortSendStr(USART1, (char *)buf);
			usartPortSendStr(USART2, (char *)buf);
		}
		else if (strncmp((char *)USART_RX_BUF,"CAV",3) == 0)
		{
			GPIO_SetBits(GPIOA, GPIO_Pin_8);
			sprintf((char *)buf, "CAV=%d\r\n", calibrationval);
			usartPortSendStr(USART1, (char *)buf);
			usartPortSendStr(USART2, (char *)buf);
		}
		else if (strncmp((char *)USART_RX_BUF,"ERR",3) == 0)
		{
			GPIO_SetBits(GPIOA, GPIO_Pin_8);
			sprintf((char *)buf, "ERR=%d\r\n", errorval);
			usartPortSendStr(USART1, (char *)buf);
			usartPortSendStr(USART2, (char *)buf);
		}
		memset(buf,0,len);  
		USART_RX_STA = 0;
	}
}
#if 0
char buf[50];
u16 adcVal=0;
float voltage = 0;
float temperature = 0;
float temperature1 = 0;

void TempTask(void)
{
	OS_ERR err;
	u8 i = 0;
	u8 len;
	
	while (1)
	{
		adcVal = adcFilter(1);
		voltage = adcVal*3.3/4096;
		temperature = voltage * 2 / 5 * 150;
		len = sprintf(buf, "1号：%4d,%4d,%4d\r\n", adcVal, (int)(voltage*100), (int)temperature*100);
		for (i = 0; i < len; i++)
		{
			//usartPortSendData(USART2, buf[i]);
		}
		OSTimeDlyHMSM(0,0,0,50,OS_OPT_TIME_HMSM_STRICT,&err); 
		
		adcVal = adcFilter(4);
		voltage = adcVal*3.3/4096;
		temperature1 = voltage * 2 / 5 * 150;
		len = sprintf(buf, "2号：%4d,%4d,%4d\r\n", adcVal, (int)(voltage*100), (int)temperature1*100);
		for (i = 0; i < len; i++)
		{
			//usartPortSendData(USART2, buf[i]);
		}
		OSTimeDlyHMSM(0,0,0,50,OS_OPT_TIME_HMSM_STRICT,&err); 
		temperature = (temperature+temperature1)/2;
		
		if (temperature >= calibrationval-errorval)
		{
			LED = 0;
		}
		else if (temperature <= calibrationval-errorval)
		{
			LED = 1;
		}
		
		OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,&err); 
	}
}
#endif

#if 0
char buf[50];
u16 adcVal = 0;
float voltage = 0;
float temperature	= 0;
void TempTask(void)
{
	OS_ERR err;
	u8 i = 0;	
	//buf[0] = '{';
	//buf[1] = '{';
	while (1)
	{
		adcVal = adcFilter(1);
		voltage = adcVal*3.3/4096;
		temperature = voltage * 2 / 5 * 150;
		sprintf(buf, "1号：%4d,\%3.2lf,\%3.2lf\r\n", adcVal, voltage, temperature);
		for (i = 0; i < 25; i++)
		{
			usartPortSendData(USART2, buf[i]);
		}
		OSTimeDlyHMSM(0,0,0,50,OS_OPT_TIME_HMSM_STRICT,&err); 
		
		adcVal = adcFilter(4);
		voltage = adcVal*3.3/4096;
		temperature = voltage * 2 / 5 * 150;
		sprintf(buf, "2号：%4d,\%3.2lf,\%3.2lf\r\n", adcVal, voltage, temperature);
		for (i = 0; i < 25; i++)
		{
			usartPortSendData(USART2, buf[i]);
		}
		OSTimeDlyHMSM(0,0,0,50,OS_OPT_TIME_HMSM_STRICT,&err); 
		
		#if 0
		adcVal = adcFilter(1);	
		//buf[2] = (adcVal>>16)&0xff;
		//buf[3] = (adcVal>>8)&0xff;
		//buf[4] = adcVal&0xff;	
		
		voltage = adcVal*3.3/4096;      
		
		distance = -28.6*voltage+67.2;
		
		sprintf(buf, "1号：%4d,\%3.2lf,\%3.2lf\r\n", adcVal, voltage, distance);
		/*adcVal = adcFilter(4);	
		buf[5] = (adcVal>>16)&0xff;
		buf[6] = (adcVal>>8)&0xff;
		buf[7] = hexToBcd(adcVal)&0xff;	*/
		//buf[5] = '}';
		//buf[6] = '}';
		for (i = 0; i < 19; i++)
		{
			usartPortSendData(USART2, buf[i]);
		}
		#endif
		OSTimeDlyHMSM(0,0,2,0,OS_OPT_TIME_HMSM_STRICT,&err); 
	}
}
#endif

void ledTask(void)  
{
	OS_ERR err;

	while(1)
	{
		led=~led;			

		OSTimeDlyHMSM(0,0,0,500,OS_OPT_TIME_HMSM_STRICT,&err); 
	}
}

u16 val=0;
float humidity = 0;
float temperature = 0;
u8 buf[50];

void TempTask(void)
{
	OS_ERR err;
	OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,&err);
	//OS_TaskSuspend((OS_TCB*)&TempTaskTcb,&err);		//挂起开始任务				
	while (1)
	{
		/*
		humidity = Get_humidity();
		sprintf((char *)buf, "湿度：%3.2f",humidity);
		usartPortSendStr(USART2, buf);	
		usartPortSendData(USART2, '%');
		usartPortSendData(USART2, '\t');
		OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err);
		*/
		temperature = Get_temperature();
		sprintf((char *)buf, "温度：%3.2f%\r\n",temperature);
		temperature = 0;
		usartPortSendStr(USART2, buf);
		OSTimeDlyHMSM(0,0,0,300,OS_OPT_TIME_HMSM_STRICT,&err);

		if (temperature >= calibrationval)
		{
			LED = 0;
		}
		else if (temperature <= calibrationval-errorval)
		{
			LED = 1;
		}
	}
}


/****************************   (C) COPYRIGHT 2016 Zhu    ****************************/

                /********************  END OF FILE  ********************/



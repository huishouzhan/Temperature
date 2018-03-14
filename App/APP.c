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

/************************** ������ *********************/
	
/* Start Task -------------------------------------*/
#define     startTaskPrio        3          //�������ȼ�
#define     startStkSize         128        //�����ջ��С	
OS_TCB      startTaskTcb;                   //������ƿ�
CPU_STK     startTaskStk[startStkSize];     //�����ջ
void startTask(void);                       //������ 	

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
  * @brief: �������.
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
	
  OS_CRITICAL_ENTER();    //�����ٽ���
	//������ʼ����
	OSTaskCreate((OS_TCB 	   *)&startTaskTcb,		  //������ƿ�
				       (CPU_CHAR	 *)"startTask", 		  //��������
							 (OS_TASK_PTR )startTask, 			  //������
							 (void		   *)0,					        //���ݸ��������Ĳ���
							 (OS_PRIO	    )startTaskPrio,     //�������ȼ�
							 (CPU_STK    *)&startTaskStk[0],  //�����ջ����ַ
							 (CPU_STK_SIZE)startStkSize/10,	  //�����ջ�����λ
							 (CPU_STK_SIZE)startStkSize,		  //�����ջ��С
							 (OS_MSG_QTY  )0,				        	//�����ڲ���Ϣ�����ܹ����յ������Ϣ��Ŀ,Ϊ0ʱ��ֹ������Ϣ
							 (OS_TICK	    )0,					        //��ʹ��ʱ��Ƭ��תʱ��ʱ��Ƭ���ȣ�Ϊ0ʱΪĬ�ϳ��ȣ�
							 (void   	   *)0,				        	//�û�����Ĵ洢��
							 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //����ѡ��
							 (OS_ERR 	   *)&err);			      	//��Ÿú�������ʱ�ķ���ֵ
						 
  OS_CRITICAL_EXIT();	    //�˳��ٽ���	    
	
	OSStart( (OS_ERR *)&err );
	while(1);	
//	return 0; 
}

/**
  * @function: TaskStart.
  * @brief:  ��������.
  * @param:  None.
  * @retval: None.
  */  
void startTask(void)
{
  OS_ERR 	err;
	
#if OS_CFG_STAT_TASK_EN > 0u
	OSStatTaskCPUUsageInit(&err);  	   //ͳ������                
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN		     //���ʹ���˲����жϹر�ʱ��
	CPU_IntDisMeasMaxCurReset();	
#endif
	
#if	OS_CFG_SCHED_ROUND_ROBIN_EN                 //��ʹ��ʱ��Ƭ��ת��ʱ��
	OSSchedRoundRobinCfg(DEF_ENABLED,5,&err);     //ʹ��ʱ��Ƭ��ת���ȹ���,ʱ��Ƭ����Ϊ5��ϵͳʱ�ӽ��ģ���1*5=5ms
#endif
	/******************  ����ָʾ������  ******************/     
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
							 
	/****************** ���������ź��� ******************/ 
						 

	OSSemCreate((OS_SEM   *)&usartSem,
							(CPU_CHAR *)"usartSem",
							(OS_SEM_CTR)0,
							(OS_ERR   *)&err);
  OS_TaskSuspend((OS_TCB*)&startTaskTcb,&err);		//����ʼ����					 
}

/**
  * @function: .
  * @brief: .
  * @param: None.
  * @retval: None.
  */

#define FLASH_SAVE_ADDR  0x08100000 				//����FLASH �����ַ(����Ϊż��)
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
		/* ʹ�ܽ��� */
		GPIO_ResetBits(GPIOA, GPIO_Pin_8);
		/* �ȴ����յ����� */
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
		len = sprintf(buf, "1�ţ�%4d,%4d,%4d\r\n", adcVal, (int)(voltage*100), (int)temperature*100);
		for (i = 0; i < len; i++)
		{
			//usartPortSendData(USART2, buf[i]);
		}
		OSTimeDlyHMSM(0,0,0,50,OS_OPT_TIME_HMSM_STRICT,&err); 
		
		adcVal = adcFilter(4);
		voltage = adcVal*3.3/4096;
		temperature1 = voltage * 2 / 5 * 150;
		len = sprintf(buf, "2�ţ�%4d,%4d,%4d\r\n", adcVal, (int)(voltage*100), (int)temperature1*100);
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
		sprintf(buf, "1�ţ�%4d,\%3.2lf,\%3.2lf\r\n", adcVal, voltage, temperature);
		for (i = 0; i < 25; i++)
		{
			usartPortSendData(USART2, buf[i]);
		}
		OSTimeDlyHMSM(0,0,0,50,OS_OPT_TIME_HMSM_STRICT,&err); 
		
		adcVal = adcFilter(4);
		voltage = adcVal*3.3/4096;
		temperature = voltage * 2 / 5 * 150;
		sprintf(buf, "2�ţ�%4d,\%3.2lf,\%3.2lf\r\n", adcVal, voltage, temperature);
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
		
		sprintf(buf, "1�ţ�%4d,\%3.2lf,\%3.2lf\r\n", adcVal, voltage, distance);
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
	//OS_TaskSuspend((OS_TCB*)&TempTaskTcb,&err);		//����ʼ����				
	while (1)
	{
		/*
		humidity = Get_humidity();
		sprintf((char *)buf, "ʪ�ȣ�%3.2f",humidity);
		usartPortSendStr(USART2, buf);	
		usartPortSendData(USART2, '%');
		usartPortSendData(USART2, '\t');
		OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err);
		*/
		temperature = Get_temperature();
		sprintf((char *)buf, "�¶ȣ�%3.2f%\r\n",temperature);
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



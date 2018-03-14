/**
  *****************************************************************************
	*                                uC/OS-III
	*
  *                            ARM Cortex-M3 Port  
  *	
  * @file:    bsp.c
  * @author:  zhu
  * @version: 1.0
  * @date:    2016.11.03
  * @brief:   STM32F10x ϵͳ����
	*           1.�ж����ȼ�ģʽ
	*           2.��������ָ��
	*           3.ʱ�Ӽ�ʱ��
	*           4.���ų�ʼ��
*******************************************************************************
  * @attention:
  *        
  *        
  *        
  *
  *****************************************************************************
  */

/* Includes ---------------------------------------*/
/* -----      -------------------------------------*/
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_tim.h"
#include "bsp.h"

/* Variables --------------------------------------*/
/* -----      -------------------------------------*/
#define USART_REC_LEN 100
u16 USART_RX_STA=0;       //����״̬���	
u8 USART_RX_BUF[USART_REC_LEN];

/**
  * @function: LED_Configuration.
  * @brief: .
  * @param: None.
  * @retval: None.
  */  
void ledConfiguration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOB, ENABLE);//�������ʱ��ʹ��
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_9;				    //LED-->PB.9 �˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	  //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;		//IO���ٶ�Ϊ2MHz
  GPIO_Init(GPIOB, &GPIO_InitStructure);		          //��ʼ��GPIOB.9
  GPIO_SetBits(GPIOB,GPIO_Pin_0|GPIO_Pin_9);	                    //��λPB.9	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;    //light curtain
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
  * @function: BEEP_Configuration.
  * @brief: .
  * @param: None.
  * @retval: None.
  */ 
void beepConfiguration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//�������ʱ��ʹ��
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;				     //BEEP-->PB.11 �˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	   //�ٶ�Ϊ2MHz
  GPIO_Init(GPIOB, &GPIO_InitStructure);	
  GPIO_ResetBits(GPIOB,GPIO_Pin_11);
}

/**
  * @function: .
  * @brief: .
  * @param: None.
  * @retval: None.
  */ 	

/**
  * @function: .
  * @brief: .
  * @param: None.
  * @retval: None.
  */ 
u32 hexToBcd(u16 hexValue)
{
	u16 tempL=0;
	u8 tempH=0;
	
	tempH=hexValue/10000;
	tempL=((hexValue%10000)/1000<<12) + ((hexValue%1000)/100<<8) + ((hexValue%100)/10<<4) + hexValue%10;
	
	return ((tempH<<16)+tempL);
}
/**
  * @function: .
  * @brief: .
  * @param: None.
  * @retval: None.
  */ 
void motorPortConfiguration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  //�������ʱ��ʹ��
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_10;				     // �˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		   //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	     //�ٶ�Ϊ2MHz
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOB,GPIO_Pin_1 | GPIO_Pin_10);	
}

/**
  * @function: .
  * @brief: .
  * @param: None.
  * @retval: None.
  */ 
void motorRun(u8 runStatus)
{
  if(runStatus==runClose)
	{
	  PBo(10)=0;
		PBo(1)=1;
		LED=ledOff;
	}
	else if(runStatus==runOpen)
	{
	  PBo(10)=1;
		PBo(1)=0;
		LED=ledOn;
	}
	else //if(runStatus==runStop)
	{
	  PBo(10)=0;
		PBo(1)=0;
	}
}

/**
  * @function: .
  * @brief: .
  * @param: None.
  * @retval: None.
  */ 
void timer3Configuration(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʱ��ʹ��
	
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc-1; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,��������ж�

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 12;  //��ռ���ȼ�12��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���

	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIM3
}

/**
  * @function: .
  * @brief: .
  * @param: None.
  * @retval: None.
  */ 
void TIM3_IRQHandler(void)   //TIM3�ж�
{	
	OSIntEnter();    
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //���TIM3�����жϷ������
	{  
		 TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //���TIMx�����жϱ�־ 
	} 
	OSIntExit();  											 
}

/**
  * @function: .
  * @brief: .
  * @param: None.
  * @retval: None.
  */ 
void adcConfiguration(void)
{
  ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); //����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M
	                       
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4; //PA 0��Ϊģ��ͨ����������  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ������
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
 
	ADC_DeInit(ADC1);                                   //��λADC1,������ ADC1 ��ȫ���Ĵ�������Ϊȱʡֵ
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;  //ADC����ģʽ:ADC1��ADC2�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	      //ģ��ת�������ڵ�ͨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//ģ��ת�������ڵ���ת��ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//ת��������������ⲿ��������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 1;	                //˳����й���ת����ADCͨ������Ŀ
	ADC_Init(ADC1, &ADC_InitStructure);	                    //����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���   

	ADC_Cmd(ADC1, ENABLE);	                    //ʹ��ָ����ADC1
	ADC_ResetCalibration(ADC1);               	//ʹ�ܸ�λУ׼   
	while(ADC_GetResetCalibrationStatus(ADC1));	//�ȴ���λУ׼����
	ADC_StartCalibration(ADC1);	                //����ADУ׼
	while(ADC_GetCalibrationStatus(ADC1));	    //�ȴ�У׼����
}

/**
  * @function: .
  * @brief: .
  * @param: None.
  * @retval: None.
  */ 
u16 getAdcVal(u8 chx)   
{
  	//����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
	ADC_RegularChannelConfig(ADC1,chx,1,ADC_SampleTime_1Cycles5 );	//ADC1,ADCͨ��,����ʱ��Ϊ5����	  			    
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		      //ʹ��ָ����ADC1�����ת����������	
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC )); //�ȴ�ת������

	return ADC_GetConversionValue(ADC1);	          //�������һ��ADC1�������ת�����
}

/**
  * @function: .
  * @brief: .
  * @param: None.
  * @retval: None.
  */  	
u16 adcFilter(u8 chx)         
{
	u8  adcGetValCnt=0;
	u32 adcValTemp=0;
	
	for(adcGetValCnt=0;adcGetValCnt<128;adcGetValCnt++)
	   adcValTemp+=getAdcVal(chx);  
	
	return (u16)(adcValTemp>>7);
} 	


/**
  * @function: USART_Configuration.
  * @brief: .
  * @param: None.
  * @retval: None.
  */ 
void usartConfiguration(u8 usartPort,u32 baudRate)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
	
  if(usartPort1==usartPort)
	{
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
			
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;     //PA9 USART1 Tx
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�������
		GPIO_Init(GPIOA, &GPIO_InitStructure);	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;    //PA10 USART1 Rx
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //��������
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		
		USART_InitStructure.USART_BaudRate = baudRate;  //һ������Ϊ9600;
  	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //�ֳ�Ϊ8λ���ݸ�ʽ
  	USART_InitStructure.USART_StopBits = USART_StopBits_1;  //һ��ֹͣλ
  	USART_InitStructure.USART_Parity = USART_Parity_No;  //����żУ��λ
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //��Ӳ������������
  	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	 //�շ�ģʽ
    USART_Init(USART1, &USART_InitStructure); //��ʼ������

		/* 485 ʹ��*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;   
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOA, &GPIO_InitStructure); 
		
		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=9;  //��ռ���ȼ�9
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		   //�����ȼ�0
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			     //IRQͨ��ʹ��
	  NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ��� 
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  //�����ж�
		
		USART_Cmd(USART1, ENABLE);   //ʹ�ܴ��� 
	}
	else if(usartPort2==usartPort)
	{
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;     //PA2 USART2 Tx
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�������
		GPIO_Init(GPIOA, &GPIO_InitStructure);	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;    //PA3 USART2 Rx
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //��������
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		
		USART_InitStructure.USART_BaudRate = baudRate;  //һ������Ϊ9600;
  	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //�ֳ�Ϊ8λ���ݸ�ʽ
  	USART_InitStructure.USART_StopBits = USART_StopBits_1;  //һ��ֹͣλ
  	USART_InitStructure.USART_Parity = USART_Parity_No;  //����żУ��λ
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //��Ӳ������������
  	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	 //�շ�ģʽ
    USART_Init(USART2, &USART_InitStructure); //��ʼ������
		
		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=10;  //��ռ���ȼ�9
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		   //�����ȼ�0
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			     //IRQͨ��ʹ��
	  NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ��� 
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);  //�����ж�
		
		USART_Cmd(USART2, ENABLE);   //ʹ�ܴ��� 
	}
	else if(usartPort3==usartPort)
	{
//	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3 , ENABLE);
	}
	else if(usartPort4==usartPort)
	{
//	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4 , ENABLE);
	}
}
extern OS_SEM usartSem;
void USART2_IRQHandler(void)                	//����1�жϷ������
{
	OS_ERR err; 
	u8 Res;
#ifdef OS_TICKS_PER_SEC	 	//���ʱ�ӽ�����������,˵��Ҫʹ��ucosII��.
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
		{
		Res =USART_ReceiveData(USART2);//(USART1->DR);	//��ȡ���յ�������
		
		if((USART_RX_STA&0x8000)==0)//����δ���
		{
			if(USART_RX_STA&0x4000)//���յ���0x0d
			{
				if(Res!=0x0a)USART_RX_STA=0;//���մ���,���¿�ʼ
				else 
				{
					USART_RX_STA|=0x8000;	//���������
					OSSemPost(&usartSem,OS_OPT_POST_1,&err);
				}
			}
			else //��û�յ�0X0D
			{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res;
					USART_RX_STA++;
					USART_RX_BUF[USART_RX_STA&0X3FFF]=0;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
					}		 
				}
			}   		 
     } 
#ifdef OS_TICKS_PER_SEC	 	//���ʱ�ӽ�����������,˵��Ҫʹ��ucosII��.
	OSIntExit();  											 
#endif
} 
void USART1_IRQHandler(void)                	//����1�жϷ������
{
	OS_ERR err; 
	u8 Res;
#ifdef OS_TICKS_PER_SEC	 	//���ʱ�ӽ�����������,˵��Ҫʹ��ucosII��.
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
		{
		Res =USART_ReceiveData(USART1);//(USART1->DR);	//��ȡ���յ�������
		
		if((USART_RX_STA&0x8000)==0)//����δ���
		{
			if(USART_RX_STA&0x4000)//���յ���0x0d
			{
				if(Res!=0x0a)USART_RX_STA=0;//���մ���,���¿�ʼ
				else 
				{
					USART_RX_STA|=0x8000;	//���������
					OSSemPost(&usartSem,OS_OPT_POST_1,&err);
				}
			}
			else //��û�յ�0X0D
			{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res;
					USART_RX_STA++;
					USART_RX_BUF[USART_RX_STA&0X3FFF]=0;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
					}		 
				}
			}   		 
     } 
#ifdef OS_TICKS_PER_SEC	 	//���ʱ�ӽ�����������,˵��Ҫʹ��ucosII��.
	OSIntExit();  											 
#endif
} 

/**
  * @function: UsartPortSendData.
  * @brief: .
  * @param: None.
  * @retval: None.
  */ 
void usartPortSendData(USART_TypeDef* USARTx,u8 sendData)
{
	 USART_GetFlagStatus(USARTx, USART_FLAG_TC);
   USART_SendData(USARTx, sendData);
	 while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
}

/**
  * @function: UsartPortSendStr.
  * @brief: .
  * @param: None.
  * @retval: None.
  */
void usartPortSendStr(USART_TypeDef* USARTx,char *sendStr)
{
   while (0 != *sendStr)
	 {
	   usartPortSendData(USARTx, *sendStr);
	   sendStr++;
	 }
}


/**
  * @function: SysTickInit.
  * @brief: ϵͳ���ĳ�ʼ�� .
  * @param: OSTick:  SysTick_1ms or SysTick_5ms.
  * @retval: None.
  */  
void sysTickInit(CPU_INT32U osTick)
{
	SysTick_Config(osTick);			
}

/**
  * @function: NVIC_Configuration.
  * @brief: �жϷ������ȼ��趨.
  * @param: NVICGroup: NVIC_PriorityGroup_0 1 2 3 4.
  * @retval: None.
  */  
void nvicConfiguration(CPU_INT32U nvicGroup)
{
	NVIC_PriorityGroupConfig(nvicGroup); 
}

/**
  * @function: CPU_TS_TmrInit.
  * @brief: ϵͳʱ�����ʼ��.
  * @param: None.
  * @retval: None.
  */  
#if (CPU_CFG_TS_TMR_EN == DEF_ENABLED)
void  CPU_TS_TmrInit (void)
{
	
}
#endif

/**
  * @function: CPU_TS_TmrRd.
  * @brief: ʱ�������ֵ��ȡ.
  * @param: None.
  * @retval: SysTick->VAL������ֵSysTick Current Value Register.
  */  
#if (CPU_CFG_TS_TMR_EN == DEF_ENABLED)
CPU_TS_TMR  CPU_TS_TmrRd (void)
{
    return (SysTick->VAL);
}
#endif

/**
  * @function: BspInit.
  * @brief: Ӳ���ӿڳ�ʼ��.
  * @param: None.
  * @retval: None.
  */  

void bspInit(void)
{
	nvicConfiguration(NVIC_PriorityGroup_4);	  //���ȼ�����  ȫΪ��ռʽ���ȼ�
	sysTickInit(sysTick_1ms);     //ʱ��1ms
	ledConfiguration();
	usartConfiguration(usartPort2,9600);
	usartConfiguration(usartPort1,38400);
	beepConfiguration();
	//adcConfiguration();
}



/****************************   (C) COPYRIGHT 2016 Zhu    ****************************/

                /********************  END OF FILE  ********************/



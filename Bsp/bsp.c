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
u16 USART_RX_STA=0;       //接收状态标记	
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
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOB, ENABLE);//相关外设时钟使能
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_9;				    //LED-->PB.9 端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	  //推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;		//IO口速度为2MHz
  GPIO_Init(GPIOB, &GPIO_InitStructure);		          //初始化GPIOB.9
  GPIO_SetBits(GPIOB,GPIO_Pin_0|GPIO_Pin_9);	                    //置位PB.9	
	
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
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//相关外设时钟使能
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;				     //BEEP-->PB.11 端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	   //速度为2MHz
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
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  //相关外设时钟使能
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_10;				     // 端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		   //推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	     //速度为2MHz
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
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //时钟使能
	
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc-1; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //使能指定的TIM3中断,允许更新中断

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 12;  //先占优先级12级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级0级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器

	TIM_Cmd(TIM3, ENABLE);  //使能TIM3
}

/**
  * @function: .
  * @brief: .
  * @param: None.
  * @retval: None.
  */ 
void TIM3_IRQHandler(void)   //TIM3中断
{	
	OSIntEnter();    
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
	{  
		 TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //清除TIMx更新中断标志 
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
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); //设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M
	                       
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4; //PA 0作为模拟通道输入引脚  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
 
	ADC_DeInit(ADC1);                                   //复位ADC1,将外设 ADC1 的全部寄存器重设为缺省值
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;  //ADC工作模式:ADC1和ADC2工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	      //模数转换工作在单通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//模数转换工作在单次转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;	                //顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);	                    //根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器   

	ADC_Cmd(ADC1, ENABLE);	                    //使能指定的ADC1
	ADC_ResetCalibration(ADC1);               	//使能复位校准   
	while(ADC_GetResetCalibrationStatus(ADC1));	//等待复位校准结束
	ADC_StartCalibration(ADC1);	                //开启AD校准
	while(ADC_GetCalibrationStatus(ADC1));	    //等待校准结束
}

/**
  * @function: .
  * @brief: .
  * @param: None.
  * @retval: None.
  */ 
u16 getAdcVal(u8 chx)   
{
  	//设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(ADC1,chx,1,ADC_SampleTime_1Cycles5 );	//ADC1,ADC通道,采样时间为5周期	  			    
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		      //使能指定的ADC1的软件转换启动功能	
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC )); //等待转换结束

	return ADC_GetConversionValue(ADC1);	          //返回最近一次ADC1规则组的转换结果
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
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //推挽输出
		GPIO_Init(GPIOA, &GPIO_InitStructure);	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;    //PA10 USART1 Rx
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //浮空输入
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		
		USART_InitStructure.USART_BaudRate = baudRate;  //一般设置为9600;
  	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //字长为8位数据格式
  	USART_InitStructure.USART_StopBits = USART_StopBits_1;  //一个停止位
  	USART_InitStructure.USART_Parity = USART_Parity_No;  //无奇偶校验位
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //无硬件数据流控制
  	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	 //收发模式
    USART_Init(USART1, &USART_InitStructure); //初始化串口

		/* 485 使能*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;   
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOA, &GPIO_InitStructure); 
		
		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=9;  //抢占优先级9
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		   //子优先级0
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			     //IRQ通道使能
	  NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器 
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  //开启中断
		
		USART_Cmd(USART1, ENABLE);   //使能串口 
	}
	else if(usartPort2==usartPort)
	{
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;     //PA2 USART2 Tx
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //推挽输出
		GPIO_Init(GPIOA, &GPIO_InitStructure);	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;    //PA3 USART2 Rx
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //浮空输入
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		
		USART_InitStructure.USART_BaudRate = baudRate;  //一般设置为9600;
  	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //字长为8位数据格式
  	USART_InitStructure.USART_StopBits = USART_StopBits_1;  //一个停止位
  	USART_InitStructure.USART_Parity = USART_Parity_No;  //无奇偶校验位
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //无硬件数据流控制
  	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	 //收发模式
    USART_Init(USART2, &USART_InitStructure); //初始化串口
		
		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=10;  //抢占优先级9
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		   //子优先级0
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			     //IRQ通道使能
	  NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器 
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);  //开启中断
		
		USART_Cmd(USART2, ENABLE);   //使能串口 
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
void USART2_IRQHandler(void)                	//串口1中断服务程序
{
	OS_ERR err; 
	u8 Res;
#ifdef OS_TICKS_PER_SEC	 	//如果时钟节拍数定义了,说明要使用ucosII了.
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
		{
		Res =USART_ReceiveData(USART2);//(USART1->DR);	//读取接收到的数据
		
		if((USART_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART_RX_STA&0x4000)//接收到了0x0d
			{
				if(Res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
				else 
				{
					USART_RX_STA|=0x8000;	//接收完成了
					OSSemPost(&usartSem,OS_OPT_POST_1,&err);
				}
			}
			else //还没收到0X0D
			{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res;
					USART_RX_STA++;
					USART_RX_BUF[USART_RX_STA&0X3FFF]=0;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
					}		 
				}
			}   		 
     } 
#ifdef OS_TICKS_PER_SEC	 	//如果时钟节拍数定义了,说明要使用ucosII了.
	OSIntExit();  											 
#endif
} 
void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	OS_ERR err; 
	u8 Res;
#ifdef OS_TICKS_PER_SEC	 	//如果时钟节拍数定义了,说明要使用ucosII了.
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
		{
		Res =USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据
		
		if((USART_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART_RX_STA&0x4000)//接收到了0x0d
			{
				if(Res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
				else 
				{
					USART_RX_STA|=0x8000;	//接收完成了
					OSSemPost(&usartSem,OS_OPT_POST_1,&err);
				}
			}
			else //还没收到0X0D
			{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res;
					USART_RX_STA++;
					USART_RX_BUF[USART_RX_STA&0X3FFF]=0;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
					}		 
				}
			}   		 
     } 
#ifdef OS_TICKS_PER_SEC	 	//如果时钟节拍数定义了,说明要使用ucosII了.
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
  * @brief: 系统节拍初始化 .
  * @param: OSTick:  SysTick_1ms or SysTick_5ms.
  * @retval: None.
  */  
void sysTickInit(CPU_INT32U osTick)
{
	SysTick_Config(osTick);			
}

/**
  * @function: NVIC_Configuration.
  * @brief: 中断分组优先级设定.
  * @param: NVICGroup: NVIC_PriorityGroup_0 1 2 3 4.
  * @retval: None.
  */  
void nvicConfiguration(CPU_INT32U nvicGroup)
{
	NVIC_PriorityGroupConfig(nvicGroup); 
}

/**
  * @function: CPU_TS_TmrInit.
  * @brief: 系统时间戳初始化.
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
  * @brief: 时间戳计数值读取.
  * @param: None.
  * @retval: SysTick->VAL：计数值SysTick Current Value Register.
  */  
#if (CPU_CFG_TS_TMR_EN == DEF_ENABLED)
CPU_TS_TMR  CPU_TS_TmrRd (void)
{
    return (SysTick->VAL);
}
#endif

/**
  * @function: BspInit.
  * @brief: 硬件接口初始化.
  * @param: None.
  * @retval: None.
  */  

void bspInit(void)
{
	nvicConfiguration(NVIC_PriorityGroup_4);	  //优先级设置  全为抢占式优先级
	sysTickInit(sysTick_1ms);     //时基1ms
	ledConfiguration();
	usartConfiguration(usartPort2,9600);
	usartConfiguration(usartPort1,38400);
	beepConfiguration();
	//adcConfiguration();
}



/****************************   (C) COPYRIGHT 2016 Zhu    ****************************/

                /********************  END OF FILE  ********************/



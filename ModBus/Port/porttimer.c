/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: porttimer.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- STM32 includes -----------------------------------*/
#include "misc.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "bsp.h"

/* ----------------------- static functions ---------------------------------*/
static void prvvTIMERExpiredISR( void );

/* ----------------------- Start implementation -----------------------------*/
/**
  * @brief  MODBUS 计算3.5Byte时间时,默认以50us为时基 
	*         所以定时器频率应设置为20KHz
  * @param  usTim1Timerout50us 3.5Byte时间
  * @retval TRUE
  */
BOOL
xMBPortTimersInit( USHORT usTim1Timerout50us )
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	TIM_TimeBaseStructure.TIM_Period = usTim1Timerout50us;
  TIM_TimeBaseStructure.TIM_Prescaler = 3599;  //72M/3600=20KHz
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  TIM_ARRPreloadConfig(TIM4, ENABLE);  //预装载使能
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 11;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
  TIM_ClearITPendingBit(TIM4,TIM_IT_Update); //清除溢出中断标志位
  TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE); //定时器5溢出中断关闭
  TIM_Cmd(TIM4, DISABLE);  //定时器5禁能
	
  return TRUE;
}


/*inline*/
void
vMBPortTimersEnable(  )
{
    /* Enable the timer with the timeout passed to xMBPortTimersInit( ) */
  TIM_ClearITPendingBit(TIM4,TIM_IT_Update); //清除溢出中断标志位
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); //定时器溢出中断开启
  TIM_SetCounter(TIM4,0); //设定定时器5的初始值
	TIM_Cmd(TIM4, ENABLE);  //定时器5使能
}

/*inline*/
void
vMBPortTimersDisable(  )
{
    /* Disable any pending timers. */
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update); //清除溢出中断标志位
  TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE); //定时器5溢出中断关闭
  TIM_SetCounter(TIM4,0); //设定定时器5的初始值
	TIM_Cmd(TIM4, DISABLE);  //定时器5禁能
}

/* Create an ISR which is called whenever the timer has expired. This function
 * must then call pxMBPortCBTimerExpired( ) to notify the protocol stack that
 * the timer has expired.
 */
static void prvvTIMERExpiredISR( void )
{
    ( void )pxMBPortCBTimerExpired(  );
}

/**
  * @brief  定时器5中断服务函数
  * @param  None
  * @retval None
  */
void TIM4_IRQHandler(void)
{
  OSIntEnter();
  if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
  { 
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update); //清除定时器5溢出中断标志位
    prvvTIMERExpiredISR( );
  }
	OSIntExit(); 
}




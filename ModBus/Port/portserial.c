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
 * File: $Id: portserial.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- STM32 includes -----------------------------------*/
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "bsp.h"
#if 0
/* ----------------------- static functions ---------------------------------*/
static void prvvUARTTxReadyISR( void );
static void prvvUARTRxISR( void );
#endif

/* ----------------------- Start implementation -----------------------------*/
void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
  if(xRxEnable)      //接收
  {
    
     USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);    //使能接收和接收中断
     GPIO_ResetBits(GPIOA,GPIO_Pin_8);    //MAX485操作 接收模式
  }
  else
  {
     USART_ITConfig(USART1, USART_IT_RXNE, DISABLE); 
     GPIO_SetBits(GPIOA,GPIO_Pin_8);  //MAX485操作 发送模式
  }

  if(xTxEnable)     //发送
  {
     USART_ITConfig(USART1, USART_IT_TXE, ENABLE);  //使能发送完成中断
  }
  else
  {
     USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
  }
}

BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
	/**************   485RE/DE   *********************/
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;   
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
	/************************************************/
	usartConfiguration(ucPORT,ulBaudRate);
	
  return TRUE;
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
    /* Put a byte in the UARTs transmit buffer. This function is called
     * by the protocol stack if pxMBFrameCBTransmitterEmpty( ) has been
     * called. */
	usartPortSendData(USART1,ucByte);
  return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
    /* Return the byte in the UARTs receive buffer. This function is called
     * by the protocol stack after pxMBFrameCBByteReceived( ) has been called.
     */
	*pucByte = USART_ReceiveData(USART1);
  return TRUE;
}

/* Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call 
 * xMBPortSerialPutByte( ) to send the character.
 */
#if 0
static void prvvUARTTxReadyISR( void )
{
    pxMBFrameCBTransmitterEmpty(  );
}
#endif

/* Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
#if 0
static void prvvUARTRxISR( void )
{
    pxMBFrameCBByteReceived(  );
}
#endif

/**
  * @brief  USART1_IRQHandler
  * @param  None
  * @retval None
  */
#if 0
void USART1_IRQHandler(void)
{
  OSIntEnter();
 
  if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET) //发生接收中断
  {
    prvvUARTRxISR();    
    USART_ClearITPendingBit(USART1, USART_IT_RXNE);   //清除中断标志位  
  } 
  if(USART_GetITStatus(USART1, USART_IT_TXE) == SET)  //发生完成中断 USART_IT_TXE  USART_IT_TC
  {
    prvvUARTTxReadyISR(); 
    USART_ClearITPendingBit(USART1, USART_IT_TXE);    //清除中断标志 USART_IT_TXE
  }

	OSIntExit(); 
}
#endif




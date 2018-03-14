#include "myiic.h"

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//IIC���� ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/9
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
 
 void delay_us()
 {
	 int i = 0;
		for(i = 0; i < 5; i++)
	 {
			__NOP();
			__NOP();
			__NOP();
	 }
 }
//��ʼ��IIC
void IIC_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE );	
	   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOA,GPIO_Pin_1); 	//PB10,PB11 �����
	GPIO_SetBits(GPIOB,GPIO_Pin_8); 	//PB10,PB11 �����
}
//����IIC��ʼ�ź�
void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us();
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us();
	IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us();
	IIC_SCL=1; 
	delay_us();
	IIC_SDA=1;//����I2C���߽����ź�
	delay_us();							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
//	IIC_SDA=1;delay_us();	   
	IIC_SCL=1;delay_us();	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us();
	IIC_SCL=1;
	delay_us();
	IIC_SCL=0;
}
//������ACKӦ��		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us();
	IIC_SCL=1;
	delay_us();
	IIC_SCL=0;
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
		SDA_OUT(); 	    
    IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        //IIC_SDA=(txd&0x80)>>7;
		if((txd&0x80)>>7)
			IIC_SDA=1;
		else
			IIC_SDA=0;
		txd<<=1; 	  
		delay_us();   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL=1;
		delay_us(); 
		IIC_SCL=0;	
//		delay_us();
    }	
		IIC_SDA=1;
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8  IIC_Read_Byte(u8 ack)
{
	unsigned char i;
	u8 receive=0;
	SDA_IN();//SDA����Ϊ����
   for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        delay_us();
				IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
				delay_us(); 
    }					 
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}

float Get_temperature(void)
{
		OS_ERR err;
		u8 buf[3];
		u16 val = 0;
		float temperature= 0;

		IIC_Start();
		IIC_Stop();
		IIC_Send_Byte(0x03);
		if (!IIC_Wait_Ack())
		{
			//usartPortSendStr(USART2, "���յ�ACK\r\n");
		}
		else
		{
			//usartPortSendStr(USART2, "δ���յ�ACK\r\n");
		}	
		OSTimeDlyHMSM(0,0,0,400,OS_OPT_TIME_HMSM_STRICT,&err); 
		buf[0] = IIC_Read_Byte(1); 
		buf[1] = IIC_Read_Byte(1);
		buf[2] = IIC_Read_Byte(1);

		val=((buf[0]&0x3f)<<8)|buf[1];
		
		temperature =val*0.01-39.7;
		
		return temperature;
}

float Get_humidity(void)
{
	OS_ERR err;
	u8 buf[3];
	u16 val = 0;
	float humidity= 0;

	IIC_Start();
	IIC_Stop();
	IIC_Send_Byte(0x05);
	if (!IIC_Wait_Ack())
	{
	//usartPortSendStr(USART2, "���յ�ACK\r\n");
	}
	else
	{
	//usartPortSendStr(USART2, "δ���յ�ACK\r\n");
	}	
	OSTimeDlyHMSM(0,0,0,400,OS_OPT_TIME_HMSM_STRICT,&err); 
	buf[0] = IIC_Read_Byte(1); 
	buf[1] = IIC_Read_Byte(1);
	buf[2] = IIC_Read_Byte(1);

	val=((buf[0]&0x0f)<<8)|buf[1];

	humidity = -4.0000 + 0.0405*val - 2.8000E-6*val;

	return humidity;
}























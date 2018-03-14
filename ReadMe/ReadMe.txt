
B_Core  单片机引脚分布


STM32103C8         外设

  PA 0             升降器ADC
  PA 2             TxD2
  PA 3             RxD2
  PA 8             485R/D E
  PA 9             485TxD
  PA 10            485RxD
  PA 12            光幕信号反馈 FK H,32 L
  PB 11            蜂鸣器   1:open  0:close
  PB 10            motorB
  PB 1             motorA   1:ON  0:OFF
  PB 9             Test led
  PB0              灯带     1:ON  0:OFF        
  
/*******************  通信协议  ********************/

usRegHoldingBuf[0]=value  :	 0:蜂鸣器   value:鸣叫时间（ms）  				
usRegHoldingBuf[1]=value  :	 1:货道 1   value:出货数量（个）
usRegHoldingBuf[2]=value  :	 2:货道 2   value:出货数量（个）
usRegHoldingBuf[3]=value  :	 3:货道 3   value:出货数量（个）
usRegHoldingBuf[4]=value  :	 4:货道 4   value:出货数量（个）
usRegHoldingBuf[5]=value  :	 5:货道 5   value:出货数量（个）
usRegHoldingBuf[6]=value  :	 6:货道 6   value:出货数量（个）
usRegHoldingBuf[7]=value  :	 7:货道 7   value:出货数量（个）
usRegHoldingBuf[8]=value  :	 8:货道 8   value:出货数量（个）
usRegHoldingBuf[13]=value :	 13:前门    value: 1 打开   2 关闭
usRegHoldingBuf[14]=value :	 14:后门    value: 1 打开   2 关闭

usRegHoldingBuf[15]=value :	 15:协议通道   
                    value =
                           1: 设备当前处于空闲状态  （由上位机 写）
					       2：货斗上货状态          （由上位机 写）
					       3：开前门状态            （由上位机 写）
					      
						   
						   
usRegInputBuf[15]=value  :  15:协议通道 
                  value =
                           1: 设备当前处于空闲状态       
					       2：货道掉货状态
					       3：掉货完成状态
		                   4: 货斗上货状态
						   5: 开前门状态
						   6: 客户取货完成状态
						   7: 关门异常状态  
						   
						   
						   
						   
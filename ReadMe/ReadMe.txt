
B_Core  ��Ƭ�����ŷֲ�


STM32103C8         ����

  PA 0             ������ADC
  PA 2             TxD2
  PA 3             RxD2
  PA 8             485R/D E
  PA 9             485TxD
  PA 10            485RxD
  PA 12            ��Ļ�źŷ��� FK H,32 L
  PB 11            ������   1:open  0:close
  PB 10            motorB
  PB 1             motorA   1:ON  0:OFF
  PB 9             Test led
  PB0              �ƴ�     1:ON  0:OFF        
  
/*******************  ͨ��Э��  ********************/

usRegHoldingBuf[0]=value  :	 0:������   value:����ʱ�䣨ms��  				
usRegHoldingBuf[1]=value  :	 1:���� 1   value:��������������
usRegHoldingBuf[2]=value  :	 2:���� 2   value:��������������
usRegHoldingBuf[3]=value  :	 3:���� 3   value:��������������
usRegHoldingBuf[4]=value  :	 4:���� 4   value:��������������
usRegHoldingBuf[5]=value  :	 5:���� 5   value:��������������
usRegHoldingBuf[6]=value  :	 6:���� 6   value:��������������
usRegHoldingBuf[7]=value  :	 7:���� 7   value:��������������
usRegHoldingBuf[8]=value  :	 8:���� 8   value:��������������
usRegHoldingBuf[13]=value :	 13:ǰ��    value: 1 ��   2 �ر�
usRegHoldingBuf[14]=value :	 14:����    value: 1 ��   2 �ر�

usRegHoldingBuf[15]=value :	 15:Э��ͨ��   
                    value =
                           1: �豸��ǰ���ڿ���״̬  ������λ�� д��
					       2�������ϻ�״̬          ������λ�� д��
					       3����ǰ��״̬            ������λ�� д��
					      
						   
						   
usRegInputBuf[15]=value  :  15:Э��ͨ�� 
                  value =
                           1: �豸��ǰ���ڿ���״̬       
					       2����������״̬
					       3���������״̬
		                   4: �����ϻ�״̬
						   5: ��ǰ��״̬
						   6: �ͻ�ȡ�����״̬
						   7: �����쳣״̬  
						   
						   
						   
						   
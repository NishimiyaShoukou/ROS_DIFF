#include "iic.h"

void delay_us(u32 nus)
{		
//	u32 temp;	    	 
//	SysTick->LOAD=nus*fac_us; 					//ʱ�����	  		 
//	SysTick->VAL=0x00;        					//��ռ�����
//	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//��ʼ����	  
//	do
//	{
//		temp=SysTick->CTRL;
//	}while((temp&0x01)&&!(temp&(1<<16)));		//�ȴ�ʱ�䵽��   
//	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//�رռ�����
//	SysTick->VAL =0X00;      					 //��ռ�����	 
	 	volatile unsigned int num;
    volatile unsigned int t;


  
		for (num = 0; num < nus; num++)
		{
			t = 7;
			while (t != 0)
			{
				t--;
			}
		}
}

/*******************************************************************************
* �� �� ��         : IIC_Init
* ��������		   : IIC��ʼ��
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void IIC_Init(void)
{
#ifdef STDLIB
	
  GPIO_InitTypeDef  GPIO_InitStructure; 
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
#else
	GPIO_InitTypeDef GPIO_InitStruct;
	
	 /* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, IIC_SCL_PIN | IIC_SDA_PIN, GPIO_PIN_RESET);
	
	/*Configure GPIO pin : PB6*/
	GPIO_InitStruct.Pin = IIC_SCL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	/*Configure GPIO pin : PB6*/
	GPIO_InitStruct.Pin = IIC_SDA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
#endif
}

/*******************************************************************************
* �� �� ��         : SDA_OUT
* ��������		   : SDA�������	   
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void SDA_OUT(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	/*Configure GPIO pin : PB6*/
	GPIO_InitStruct.Pin = IIC_SDA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/*******************************************************************************
* �� �� ��         : SDA_IN
* ��������		   : SDA��������	   
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void SDA_IN(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	/*Configure GPIO pin : PB6*/
	GPIO_InitStruct.Pin = IIC_SDA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/*******************************************************************************
* �� �� ��         : IIC_Start
* ��������		   : ����IIC��ʼ�ź�   
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	SDA_H;	  	  
	SCL_H;
	delay_us(5);
 	SDA_L;//START:when CLK is high,DATA change form high to low 
	delay_us(6);
	SCL_L;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	

/*******************************************************************************
* �� �� ��         : IIC_Stop
* ��������		   : ����IICֹͣ�ź�   
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	SCL_L;
	SDA_L;//STOP:when CLK is high DATA change form low to high
 	SCL_H; 
	delay_us(6); 
	SDA_H;//����I2C���߽����ź�
	delay_us(6);							   	
}

/*******************************************************************************
* �� �� ��         : IIC_Wait_Ack
* ��������		   : �ȴ�Ӧ���źŵ���   
* ��    ��         : ��
* ��    ��         : 1������Ӧ��ʧ��
        			 0������Ӧ��ɹ�
*******************************************************************************/
u8 IIC_Wait_Ack(void)
{
	u8 tempTime=0;
	
	SDA_H;
	delay_us(1);
	SDA_IN();      //SDA����Ϊ����  	   
	SCL_H;
	delay_us(1);	 
	while(SDA_read)
	{
		tempTime++;
		if(tempTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	SCL_L;//ʱ�����0 	   
	return 0;  
} 

/*******************************************************************************
* �� �� ��         : IIC_Ack
* ��������		   : ����ACKӦ��  
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void IIC_Ack(void)
{
	SCL_L;
	SDA_OUT();
	SDA_L;
	delay_us(2);
	SCL_H;
	delay_us(5);
	SCL_L;
}

/*******************************************************************************
* �� �� ��         : IIC_NAck
* ��������		   : ����NACK��Ӧ��  
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/		    
void IIC_NAck(void)
{
	SCL_L;
	SDA_OUT();
	SDA_H;
	delay_us(2);
	SCL_H;
	delay_us(5);
	SCL_L;
}	

/*******************************************************************************
* �� �� ��         : IIC_Send_Byte
* ��������		   : IIC����һ���ֽ� 
* ��    ��         : txd������һ���ֽ�
* ��    ��         : ��
*******************************************************************************/		  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
		SDA_OUT(); 	    
    SCL_L;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        if((txd&0x80)>0) //0x80  1000 0000
					SDA_H;
				else
					SDA_L;
        txd<<=1; 	  
				delay_us(2);   //��TEA5767��������ʱ���Ǳ����
				SCL_H;
				delay_us(2); 
				SCL_L;	
				delay_us(2);
    }	 
} 

/*******************************************************************************
* �� �� ��         : IIC_Read_Byte
* ��������		   : IIC��һ���ֽ� 
* ��    ��         : ack=1ʱ������ACK��ack=0������nACK 
* ��    ��         : Ӧ����Ӧ��
*******************************************************************************/  
u8 IIC_Read_Byte(u8 ack)
{
		u8 i,receive=0;
		SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
		{
        SCL_L; 
        delay_us(2);
				SCL_H;
        receive<<=1;
        if(SDA_read)receive++;   
				delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}







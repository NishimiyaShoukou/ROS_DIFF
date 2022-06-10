
/*--Include-start-------------------------------------------------------------*/
#include "stm32F4xx.h"
#include "IICcom.h"

/*--Include-end---------------------------------------------------------------*/


/*******************************************************************************
* Function Name  : I2C_GPIO_Config
* Description    : Configration Simulation IIC GPIO
* Input          : None 
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_GPIO_Config(void)                                //IIC���ų�ʼ��
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
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);
	
	/*Configure GPIO pin : PB6*/
	GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	/*Configure GPIO pin : PB6*/
	GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
#endif
}

/*******************************************************************************
* Function Name  : I2C_delay
* Description    : Simulation IIC Timing series delay
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_delay(void)
{
                
   uint8_t i=5; //��������Ż��ٶ�        ����������͵�5����д��
   while(i) 
   { 
     i--; 
   }  
}

void delay5ms(void)
{
                
   int i=5000;  
   while(i) 
   { 
     i--; 
   }  
}
/*******************************************************************************
* Function Name  : I2C_Start
* Description    : Master Start Simulation IIC Communication
* Input          : None
* Output         : None
* Return         : Wheather         Start
****************************************************************************** */
uint8_t I2C_Start(void)                        //IIC��ʼ
{
	SDA_H;
	SCL_H;
	I2C_delay();
	if(!SDA_read)return FALSE;        //SDA��Ϊ�͵�ƽ������æ,�˳�
	SDA_L;
	I2C_delay();
	if(SDA_read) return FALSE;        //SDA��Ϊ�ߵ�ƽ�����߳���,�˳�
	SDA_L;
	I2C_delay();
	return TRUE;
}
/*******************************************************************************
* Function Name  : I2C_Stop
* Description    : Master Stop Simulation IIC Communication
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_Stop(void)                                //IICֹͣ
{
	SCL_L;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SDA_H;
	I2C_delay();
} 
/*******************************************************************************
* Function Name  : I2C_Ack
* Description    : Master Send Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_Ack(void)                //IIC����
{        
	SCL_L;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
}   
/*******************************************************************************
* Function Name  : I2C_NoAck
* Description    : Master Send No Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_NoAck(void)        
{        
	SCL_L;
	I2C_delay();
	SDA_H;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
} 
/*******************************************************************************
* Function Name  : I2C_WaitAck
* Description    : Master Reserive Slave Acknowledge Single
* Input          : None
* Output         : None
* Return         : Wheather         Reserive Slave Acknowledge Single
****************************************************************************** */
uint8_t I2C_WaitAck(void)          //����Ϊ:=1��ACK,=0��ACK  �ȴ�����
{
	uint8_t ucErrTime=0;
	SCL_L;
	I2C_delay();
	SDA_H;                        
	I2C_delay();
	SCL_H;
	I2C_delay();
	while(SDA_read)
	{
		SCL_L;
		I2C_delay();
		ucErrTime++;
		if(ucErrTime > 250)
		{
			return FALSE;
		}
	}
	SCL_L;
	I2C_delay();
	return TRUE;
}
/*******************************************************************************
* Function Name  : I2C_SendByte
* Description    : Master Send a Byte to Slave
* Input          : Will Send Date
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_SendByte(uint8_t SendByte) //���ݴӸ�λ����λ//��������
{
	uint8_t i=8;
	while(i--)
	{
		SCL_L;
		I2C_delay();
		
		if(SendByte&0x80)
			SDA_H;  
		else 
			SDA_L;
		
		SendByte<<=1;
		I2C_delay();
		SCL_H;
		I2C_delay();
	}
	SCL_L;
}  
/*******************************************************************************
* Function Name  : I2C_RadeByte
* Description    : Master Reserive a Byte From Slave
* Input          : None
* Output         : None
* Return         : Date From Slave 
****************************************************************************** */
unsigned char I2C_RadeByte(void)  //���ݴӸ�λ����λ//��ȡ�ֽ�
{ 
	uint8_t i=8;
	uint8_t ReceiveByte=0;

	SDA_H;                                
	while(i--)
	{
		ReceiveByte<<=1;      
		SCL_L;
		I2C_delay();
				SCL_H;
		I2C_delay();        
		if(SDA_read)
		{
			ReceiveByte|=0x01;
		}
	}
	SCL_L;
	return ReceiveByte;
} 
//ZRX          
//���ֽ�д��*******************************************

uint8_t Single_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data)                     //void 
{
	if(!I2C_Start())return FALSE;

	I2C_SendByte(SlaveAddress);   //�����豸��ַ+д�ź�//I2C_SendByte(((REG_Address & 0x0700) >>7) | SlaveAddress & 0xFFFE);//���ø���ʼ��ַ+������ַ 
	if(!I2C_WaitAck())
	{
		I2C_Stop(); return FALSE;
	}
	I2C_SendByte(REG_Address );   //���õ���ʼ��ַ      
	I2C_WaitAck();        
	I2C_SendByte(REG_data);
	I2C_WaitAck();   
	I2C_Stop(); 
	delay5ms();
	return TRUE;
}

//���ֽڶ�ȡ*****************************************
unsigned char Single_Read(unsigned char SlaveAddress,unsigned char REG_Address)
{   
	unsigned char REG_data;    
	
	if(!I2C_Start())return FALSE;
	
	I2C_SendByte(SlaveAddress); //I2C_SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//���ø���ʼ��ַ+������ַ 
	
	if(!I2C_WaitAck())
	{
		I2C_Stop();
		//test=1; 
		return FALSE;
	}
	I2C_SendByte((uint8_t) REG_Address);   //���õ���ʼ��ַ      
	I2C_WaitAck();
	I2C_Start();
	I2C_SendByte(SlaveAddress+1);
	I2C_WaitAck();

	REG_data= I2C_RadeByte();
	I2C_NoAck();
	I2C_Stop();
	
	//return TRUE;
	return REG_data;

}                                       
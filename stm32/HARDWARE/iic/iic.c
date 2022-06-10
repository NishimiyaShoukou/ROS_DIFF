#include "iic.h"

void delay_us(u32 nus)
{		
//	u32 temp;	    	 
//	SysTick->LOAD=nus*fac_us; 					//时间加载	  		 
//	SysTick->VAL=0x00;        					//清空计数器
//	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//开始倒数	  
//	do
//	{
//		temp=SysTick->CTRL;
//	}while((temp&0x01)&&!(temp&(1<<16)));		//等待时间到达   
//	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//关闭计数器
//	SysTick->VAL =0X00;      					 //清空计数器	 
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
* 函 数 名         : IIC_Init
* 函数功能		   : IIC初始化
* 输    入         : 无
* 输    出         : 无
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
* 函 数 名         : SDA_OUT
* 函数功能		   : SDA输出配置	   
* 输    入         : 无
* 输    出         : 无
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
* 函 数 名         : SDA_IN
* 函数功能		   : SDA输入配置	   
* 输    入         : 无
* 输    出         : 无
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
* 函 数 名         : IIC_Start
* 函数功能		   : 产生IIC起始信号   
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	SDA_H;	  	  
	SCL_H;
	delay_us(5);
 	SDA_L;//START:when CLK is high,DATA change form high to low 
	delay_us(6);
	SCL_L;//钳住I2C总线，准备发送或接收数据 
}	

/*******************************************************************************
* 函 数 名         : IIC_Stop
* 函数功能		   : 产生IIC停止信号   
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	SCL_L;
	SDA_L;//STOP:when CLK is high DATA change form low to high
 	SCL_H; 
	delay_us(6); 
	SDA_H;//发送I2C总线结束信号
	delay_us(6);							   	
}

/*******************************************************************************
* 函 数 名         : IIC_Wait_Ack
* 函数功能		   : 等待应答信号到来   
* 输    入         : 无
* 输    出         : 1，接收应答失败
        			 0，接收应答成功
*******************************************************************************/
u8 IIC_Wait_Ack(void)
{
	u8 tempTime=0;
	
	SDA_H;
	delay_us(1);
	SDA_IN();      //SDA设置为输入  	   
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
	SCL_L;//时钟输出0 	   
	return 0;  
} 

/*******************************************************************************
* 函 数 名         : IIC_Ack
* 函数功能		   : 产生ACK应答  
* 输    入         : 无
* 输    出         : 无
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
* 函 数 名         : IIC_NAck
* 函数功能		   : 产生NACK非应答  
* 输    入         : 无
* 输    出         : 无
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
* 函 数 名         : IIC_Send_Byte
* 函数功能		   : IIC发送一个字节 
* 输    入         : txd：发送一个字节
* 输    出         : 无
*******************************************************************************/		  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
		SDA_OUT(); 	    
    SCL_L;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        if((txd&0x80)>0) //0x80  1000 0000
					SDA_H;
				else
					SDA_L;
        txd<<=1; 	  
				delay_us(2);   //对TEA5767这三个延时都是必须的
				SCL_H;
				delay_us(2); 
				SCL_L;	
				delay_us(2);
    }	 
} 

/*******************************************************************************
* 函 数 名         : IIC_Read_Byte
* 函数功能		   : IIC读一个字节 
* 输    入         : ack=1时，发送ACK，ack=0，发送nACK 
* 输    出         : 应答或非应答
*******************************************************************************/  
u8 IIC_Read_Byte(u8 ack)
{
		u8 i,receive=0;
		SDA_IN();//SDA设置为输入
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
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}







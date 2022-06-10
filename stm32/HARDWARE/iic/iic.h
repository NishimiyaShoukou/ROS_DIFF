#ifndef _iic_H
#define _iic_H

#include "stm32f4xx.h"
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
///*  IIC_SCLʱ�Ӷ˿ڡ����Ŷ��� */
#define IIC_SCL_PORT 			GPIOB   
#define IIC_SCL_PIN 			GPIO_PIN_3
//#define IIC_SCL_PORT_RCC		RCC_APB2Periph_GPIOB

/*  IIC_SDAʱ�Ӷ˿ڡ����Ŷ��� */
#define IIC_SDA_PORT 			GPIOB  
#define IIC_SDA_PIN 			GPIO_PIN_4
//#define IIC_SDA_PORT_RCC		RCC_APB2Periph_GPIOB

////IO��������	 
//#define IIC_SCL    PBout(3) //SCL
//#define IIC_SDA    PBout(4) //SDA	 
//#define READ_SDA   PBin(4)  //����SDA
#ifdef STDLIB

#define SCL_H         GPIOB->BSRR = GPIO_Pin_6
#define SCL_L         GPIOB->BRR  = GPIO_Pin_6
   
#define SDA_H         GPIOB->BSRR = GPIO_Pin_7
#define SDA_L         GPIOB->BRR  = GPIO_Pin_7

#define SCL_read      GPIOB->IDR  & GPIO_Pin_6
#define SDA_read      GPIOB->IDR  & GPIO_Pin_7

#else

#define SCL_H         HAL_GPIO_WritePin(IIC_SCL_PORT,IIC_SCL_PIN,GPIO_PIN_SET)//GPIOB->BSRR = GPIO_PIN_6 
#define SCL_L         HAL_GPIO_WritePin(IIC_SCL_PORT,IIC_SCL_PIN,GPIO_PIN_RESET)
   
#define SDA_H         HAL_GPIO_WritePin(IIC_SDA_PORT,IIC_SDA_PIN ,GPIO_PIN_SET)
#define SDA_L         HAL_GPIO_WritePin(IIC_SDA_PORT,IIC_SDA_PIN ,GPIO_PIN_RESET)

#define SCL_read      HAL_GPIO_ReadPin(IIC_SCL_PORT,IIC_SCL_PIN)//GPIOB->IDR  & GPIO_PIN_6
#define SDA_read      HAL_GPIO_ReadPin(IIC_SDA_PORT,IIC_SDA_PIN )

#endif /*STDLIB*/

//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(u8 ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�
#define FALSE 0
#define TRUE	1

#endif

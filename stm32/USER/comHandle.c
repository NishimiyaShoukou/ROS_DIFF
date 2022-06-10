
#include "comHandle.h"
#include "usart.h"
#include "stdio.h"
#include "move.h"
float roll,gyrox,gyroz;
float a[3],w[3],angle[3],T;
union sendData
{
	short d;
	unsigned char data[2];
}leftVelNow,rightVelNow,angleNow;
union recData
{
	short d;
	unsigned char data[2];
}leftVelGet,rightVelGet,angleGet;

com_data Recdata,Sendata;
/**************************************************************************
函数功能：计算八位循环冗余校验，被usartSendData和usartReceiveOneData函数调用
入口参数：数组地址、数组大小
返回  值：无
**************************************************************************/
unsigned char getCrc8(unsigned char *ptr, unsigned short len)
{
	unsigned char crc;
	unsigned char i;
	crc = 0;
	while(len--)
	{
		crc ^= *ptr++;
		for(i = 0; i < 8; i++)
		{
			if(crc&0x01)
                crc=(crc>>1)^0x8C;
			else 
                crc >>= 1;
		}
	}
	return crc;
}

//串口字节流写入到缓冲区
uint8_t k1=0,RX_Value1[20]={0};
void UsartRxToBuf(uint8_t data)
{
	
	//上一次消息处理结束
	  RX_Value1[k1++] =data;
	//UsartRxToBuf(bt);
	  // RX_Value[k++] =USART_ReceiveData(USART1);	//读取接收到的数据
		 if(!(RX_Value1[0]==0x55))//如果帧头错误，清缓存
		 {
				k1=0;
				RX_Value1[0]=0;
		 }
		 if(k1>=10)
		 {
			 
			  
		    if(RX_Value1[1]==0xaa)
				{
					 
					leftVelGet.data[0]=RX_Value1[3];    //左轮速度
					leftVelGet.data[1]=RX_Value1[4];
//					Recdata.speed[1]=(RX_Value1[5]<<4|RX_Value1[6]);
					rightVelGet.data[0]=RX_Value1[5];
					rightVelGet.data[1]=RX_Value1[6];
					Recdata.speed[0]=leftVelGet.d;    //右轮速度
					Recdata.speed[1]=rightVelGet.d;
					
				}
				
		  	k1=0;
				RX_Value1[0]=0;
		 }
}
void UsartTx(uint8_t * data,uint8_t len)
{
	while(len--)
	{
		while((USART1->SR&0X40)==0);	
		USART1->DR = (uint8_t)*data++;
	}
}
uint8_t Tx_Buff[13];
void Usartdata_up(short leftspeed, short rightspeed,short angle,unsigned char Flag)
{
	leftVelNow.d=leftspeed;
	rightVelNow.d=rightspeed;
	angleNow.d=angle*88;
	Tx_Buff[0]=0x55;
	Tx_Buff[1]=0xaa;
	Tx_Buff[2]=5;
	Tx_Buff[3]=leftVelNow.data[0];
	
	Tx_Buff[4]=leftVelNow.data[1];
	
	Tx_Buff[5]=rightVelNow.data[0];
	
	Tx_Buff[6]=rightVelNow.data[1];
	
	
	Tx_Buff[8]=angleNow.data[1];
	Tx_Buff[7]=angleNow.data[0];
	
	Tx_Buff[9]=Flag;
	Tx_Buff[10]=0x0d;
	Tx_Buff[11]=0x0a;
	UsartTx(Tx_Buff,12);
	
}
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//中景园电子
//店铺地址：http://shop73023976.taobao.com/?spm=2013.1.0.0.M4PqC2
//
//  文 件 名   : main.c
//  版 本 号   : v2.0
//  作    者   : Evk123
//  生成日期   : 2014-0101
//  最近修改   : 
//  功能描述   : 0.69寸OLED 接口演示例程(STM32F103ZE系列IIC)
//              说明: 
//              ----------------------------------------------------------------
//              GND   电源地
//              VCC   接5V或3.3v电源
//              SCL   接PD6（SCL）
//              SDA   接PD7（SDA）            
//              ----------------------------------------------------------------
//Copyright(C) 中景园电子2014/3/16
//All rights reserved
//////////////////////////////////////////////////////////////////////////////////

#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"  	 

//OLED的显存
//存放格式如下.
//[0]0 1 2 3 ... 127	
//[1]0 1 2 3 ... 127	
//[2]0 1 2 3 ... 127	
//[3]0 1 2 3 ... 127	
//[4]0 1 2 3 ... 127	
//[5]0 1 2 3 ... 127	
//[6]0 1 2 3 ... 127	
//[7]0 1 2 3 ... 127 	
//void OLED_PORT_Init()
//{
//		GPIO_InitTypeDef GPIO_InitStructure;     //
//	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
//		GPIO_InitStructure.GPIO_Pin=OLED_SCLK|OLED_SDIN; //
//	  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
//	  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;	//
//	  GPIO_Init(OLED_PORT,&GPIO_InitStructure);
//}
///**********************************************
////IIC Start
//**********************************************/
///**********************************************
////IIC Start
//**********************************************/
//void IIC_Start()
//{

//	OLED_SCLK_Set() ;
//	OLED_SDIN_Set();
//	OLED_SDIN_Clr();
//	OLED_SCLK_Clr();
//}

///**********************************************
////IIC Stop
//**********************************************/
//void IIC_Stop()
//{
//OLED_SCLK_Set() ;
////	OLED_SCLK_Clr();
//	OLED_SDIN_Clr();
//	OLED_SDIN_Set();
//	
//}

////void IIC_Wait_Ack()
////{

////	//GPIOB->CRH &= 0XFFF0FFFF;	//设置PB12为上拉输入模式
////	//GPIOB->CRH |= 0x00080000;
//////	OLED_SDA = 1;
//////	delay_us(1);
////	//OLED_SCL = 1;
////	//delay_us(50000);
/////*	while(1)
////	{
////		if(!OLED_SDA)				//判断是否接收到OLED 应答信号
////		{
////			//GPIOB->CRH &= 0XFFF0FFFF;	//设置PB12为通用推免输出模式
////			//GPIOB->CRH |= 0x00030000;
////			return;
////		}
////	}
////*/
////	OLED_SCLK_Set() ;
////	OLED_SCLK_Clr();
////}
///**********************************************
//// IIC Write byte
//**********************************************/

//void Write_IIC_Byte(unsigned char IIC_Byte)
//{
//	unsigned char i;
//	unsigned char m,da;
//	da=IIC_Byte;
//	OLED_SCLK_Clr();
//	for(i=0;i<8;i++)		
//	{
//			m=da;
//		//	OLED_SCLK_Clr();
//		m=m&0x80;
//		if(m==0x80)
//		{OLED_SDIN_Set();}
//		else OLED_SDIN_Clr();
//			da=da<<1;
//		OLED_SCLK_Set();
//		OLED_SCLK_Clr();
//		}


//}
uint8_t Bef[3];//保存前一个数据的几个参数1.要写在第几页2.0x01要移动几位3.写什么数据
uint8_t Cur[3];//当前前一个数据1.要写在第几页2.0x01要移动几位3.写什么数据

/**********************************************
// IIC Write Command
**********************************************/
void Write_IIC_Command(unsigned char IIC_Command)
{
   IIC_Start();
   IIC_Send_Byte(0x78);            //Slave address,SA0=0
	IIC_Wait_Ack();	
   IIC_Send_Byte(0x00);			//write command
	IIC_Wait_Ack();	
   IIC_Send_Byte(IIC_Command); 
	IIC_Wait_Ack();	
   IIC_Stop();
}
///**********************************************
//// IIC Write Data
//**********************************************/
void Write_IIC_Data(unsigned char IIC_Data)
{
   IIC_Start();
   IIC_Send_Byte(0x78);			//D/C#=0; R/W#=0
	IIC_Wait_Ack();	
   IIC_Send_Byte(0x40);			//write data
	IIC_Wait_Ack();	
   IIC_Send_Byte(IIC_Data);
	IIC_Wait_Ack();	
   IIC_Stop();
}
void OLED_WR_Byte(unsigned dat,unsigned cmd)
{
	if(cmd)
			{

   Write_IIC_Data(dat);
   
		}
	else {
   Write_IIC_Command(dat);
		
	}


}


/********************************************
// fill_Picture
********************************************/
void fill_picture(unsigned char fill_Data)
{
	unsigned char m,n;
	for(m=0;m<8;m++)
	{
		OLED_WR_Byte(0xb0+m,0);		//page0-page1
		OLED_WR_Byte(0x00,0);		//low column start address
		OLED_WR_Byte(0x10,0);		//high column start address
		for(n=0;n<128;n++)
			{
				OLED_WR_Byte(fill_Data,1);
			}
	}
}


/***********************Delay****************************************/
void Delay_50ms(unsigned int Del_50ms)
{
	unsigned int m;
	for(;Del_50ms>0;Del_50ms--)
		for(m=6245;m>0;m--);
}

void Delay_1ms(unsigned int Del_1ms)
{
	unsigned char j;
	while(Del_1ms--)
	{	
		for(j=0;j<123;j++);
	}
}

//坐标设置

void OLED_Set_Pos(unsigned char x, unsigned char y) 
{ 	OLED_WR_Byte(0xb0+y,OLED_CMD);
	OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
	OLED_WR_Byte((x&0x0f),OLED_CMD); 
}   	  
//开启OLED显示    
void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
	OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}
//关闭OLED显示     
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
	OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}		   			 
//清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样!!!	  
void OLED_Clear(void)  
{  
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //设置页地址（0~7）
		OLED_WR_Byte (0x00,OLED_CMD);      //设置显示位置—列低地址
		OLED_WR_Byte (0x10,OLED_CMD);      //设置显示位置—列高地址   
		for(n=0;n<128;n++)OLED_WR_Byte(0,OLED_DATA); 
	} //更新显示
}
void OLED_On(void)  
{  
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //设置页地址（0~7）
		OLED_WR_Byte (0x00,OLED_CMD);      //设置显示位置—列低地址
		OLED_WR_Byte (0x10,OLED_CMD);      //设置显示位置—列高地址   
		for(n=0;n<128;n++)OLED_WR_Byte(1,OLED_DATA); 
	} //更新显示
}
//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:0~63
//mode:0,反白显示;1,正常显示				 
//size:选择字体 16/12 
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 Char_Size)
{      	
	unsigned char c=0,i=0;	
		c=chr-' ';//得到偏移后的值			
		if(x>Max_Column-1){x=0;y=y+2;}
		if(Char_Size ==16)
			{
			OLED_Set_Pos(x,y);	
			for(i=0;i<8;i++)
			OLED_WR_Byte(F8X16[c*16+i],OLED_DATA);
			OLED_Set_Pos(x,y+1);
			for(i=0;i<8;i++)
			OLED_WR_Byte(F8X16[c*16+i+8],OLED_DATA);
			}
			else {	
				OLED_Set_Pos(x,y);
				for(i=0;i<6;i++)
				OLED_WR_Byte(F6x8[c][i],OLED_DATA);
				
			}
}
//m^n函数
u32 oled_pow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}				  
//显示2个数字
//x,y :起点坐标	 
//len :数字的位数
//size:字体大小
//mode:模式	0,填充模式;1,叠加模式
//num:数值(0~4294967295);	 		  
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size2)
{         	
	u8 t,temp;
	u8 enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size2/2)*t,y,' ',size2);
				continue;
			}else enshow=1; 
		 	 
		}
	 	OLED_ShowChar(x+(size2/2)*t,y,temp+'0',size2); 
	}
} 
//显示一个字符号串
void OLED_ShowString(u8 x,u8 y,u8 *chr,u8 Char_Size)
{
	unsigned char j=0;
	while (chr[j]!='\0')
	{		OLED_ShowChar(x,y,chr[j],Char_Size);
			x+=8;
		if(x>120){x=0;y+=2;}
			j++;
	}
}
//显示汉字
void OLED_ShowCHinese(u8 x,u8 y,u8 no)
{      			    
	u8 t,adder=0;
	OLED_Set_Pos(x,y);	
    for(t=0;t<16;t++)
		{
				OLED_WR_Byte(Hzk[2*no][t],OLED_DATA);
				adder+=1;
     }	
		OLED_Set_Pos(x,y+1);	
    for(t=0;t<16;t++)
			{	
				OLED_WR_Byte(Hzk[2*no+1][t],OLED_DATA);
				adder+=1;
      }					
}
/***********功能描述：显示显示BMP图片128×64起始点坐标(x,y),x的范围0～127，y为页的范围0～7*****************/
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[])
{ 	
 unsigned int j=0;
 unsigned char x,y;
  
  if(y1%8==0) y=y1/8;      
  else y=y1/8+1;
	for(y=y0;y<y1;y++)
	{
		OLED_Set_Pos(x0,y);
    for(x=x0;x<x1;x++)
	    {      
	    	OLED_WR_Byte(BMP[j++],OLED_DATA);	    	
	    }
	}
} 
void Before_State_Update(uint8_t y)//根据y的值，求出前一个数据的有关参数
{
	Bef[0]=7-y/8;
	Bef[1]=7-y%8;
	Bef[2]=1<<Bef[1];
}
void Current_State_Update(uint8_t y)//根据Y值，求出当前数据的有关参数
{
	Cur[0]=7-y/8;//数据写在第几页
	Cur[1]=7-y%8;//0x01要移动的位数
	Cur[2]=1<<Cur[1];//要写什么数据
}

	
void OLED_DrawWave(uint8_t x,uint8_t y)
{

	int8_t page_sub;
	uint8_t page_buff,i,j;
	Current_State_Update(y);//根据Y值，求出当前数据的有关参数
	page_sub=Bef[0]-Cur[0];//当前值与前一个值的页数相比较
	//确定当前列，每一页应该写什么数据
	if(page_sub>0)
	{
		page_buff=Bef[0];
		OLED_Set_Pos(page_buff,x);
		Write_IIC_Data(Bef[2]-0x01);
		page_buff--;
		for(i=0;i<page_sub-1;i++)
		{
			OLED_Set_Pos(page_buff,x);
			Write_IIC_Data(0xff);
			page_buff--;
		}
		OLED_Set_Pos(page_buff,x);
		Write_IIC_Data(0xff<<Cur[1]);
	}
	else if(page_sub==0)
	{
		if(Cur[1]==Bef[1])
		{
			OLED_Set_Pos(Cur[0],x);
			Write_IIC_Data(Cur[2]);
		}
		else if(Cur[1]>Bef[1])
		{
			OLED_Set_Pos(Cur[0],x);
			Write_IIC_Data((Cur[2]-Bef[2])|Cur[2]);
		}
		else if(Cur[1]<Bef[1])
		{
			OLED_Set_Pos(Cur[0],x);
			Write_IIC_Data(Bef[2]-Cur[2]);
		}
	}
	else if(page_sub<0)
	{
		page_buff=Cur[0];
		OLED_Set_Pos(page_buff,x);
		Write_IIC_Data((Cur[2]<<1)-0x01);
		page_buff--;
		for(i=0;i<0-page_sub-1;i++)
		{
			OLED_Set_Pos(page_buff,x);
			Write_IIC_Data(0xff);
			page_buff--;
		}
		OLED_Set_Pos(page_buff,x);
		Write_IIC_Data(0xff<<(Bef[1]+1));
	}
	Before_State_Update(y);
	//把下一列，每一页的数据清除掉
	for(i=0;i<8;i++)
	{
		OLED_Set_Pos(i, x+1) ;
		for(j=0;j<1;j++)
			Write_IIC_Data(0x00);
	}
}


//初始化SSD1306					    
void OLED_Init(void)
{ 	
 
// 	  OLED_PORT_Init();
// 	GPIO_InitTypeDef  GPIO_InitStructure;
// 	
// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOG, ENABLE);	 //使能PC,D,G端口时钟

//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_3|GPIO_Pin_8;	 //PD3,PD6推挽输出  
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//速度50MHz
// 	GPIO_Init(GPIOD, &GPIO_InitStructure);	  //初始化GPIOD3,6
// 	GPIO_SetBits(GPIOD,GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_3|GPIO_Pin_8);	//PD3,PD6 输出高

// #if OLED_MODE==1
// 
// 	GPIO_InitStructure.GPIO_Pin =0xFF; //PC0~7 OUT推挽输出
// 	GPIO_Init(GPIOC, &GPIO_InitStructure);
// 	GPIO_SetBits(GPIOC,0xFF); //PC0~7输出高

// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;				 //PG13,14,15 OUT推挽输出
// 	GPIO_Init(GPIOG, &GPIO_InitStructure);
// 	GPIO_SetBits(GPIOG,GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);						 //PG13,14,15 OUT  输出高

// #else
// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;				 //PC0,1 OUT推挽输出
// 	GPIO_Init(GPIOC, &GPIO_InitStructure);
// 	GPIO_SetBits(GPIOC,GPIO_Pin_0|GPIO_Pin_1);						 //PC0,1 OUT  输出高

//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;				 //PG15 OUT推挽输出	  RST
// 	GPIO_Init(GPIOG, &GPIO_InitStructure);
// 	GPIO_SetBits(GPIOG,GPIO_Pin_15);						 //PG15 OUT  输出高


// #endif
 //  delay_ms(200);

  OLED_WR_Byte(0xAE,OLED_CMD);//--display off
	OLED_WR_Byte(0x00,OLED_CMD);//---set low column address
	OLED_WR_Byte(0x10,OLED_CMD);//---set high column address
	OLED_WR_Byte(0x40,OLED_CMD);//--set start line address  
	OLED_WR_Byte(0xB0,OLED_CMD);//--set page address
	OLED_WR_Byte(0x81,OLED_CMD); // contract control
	OLED_WR_Byte(0xFF,OLED_CMD);//--128   
	OLED_WR_Byte(0xA1,OLED_CMD);//set segment remap 
	OLED_WR_Byte(0xA6,OLED_CMD);//--normal / reverse
	OLED_WR_Byte(0xA8,OLED_CMD);//--set multiplex ratio(1 to 64)
	OLED_WR_Byte(0x3F,OLED_CMD);//--1/32 duty
	OLED_WR_Byte(0xC8,OLED_CMD);//Com scan direction
	OLED_WR_Byte(0xD3,OLED_CMD);//-set display offset
	OLED_WR_Byte(0x00,OLED_CMD);//
	
	OLED_WR_Byte(0xD5,OLED_CMD);//set osc division
	OLED_WR_Byte(0x80,OLED_CMD);//
	
	OLED_WR_Byte(0xD8,OLED_CMD);//set area color mode off
	OLED_WR_Byte(0x05,OLED_CMD);//
	
	OLED_WR_Byte(0xD9,OLED_CMD);//Set Pre-Charge Period
	OLED_WR_Byte(0xF1,OLED_CMD);//
	
	OLED_WR_Byte(0xDA,OLED_CMD);//set com pin configuartion
	OLED_WR_Byte(0x12,OLED_CMD);//
	
	OLED_WR_Byte(0xDB,OLED_CMD);//set Vcomh
	OLED_WR_Byte(0x30,OLED_CMD);//
	
	OLED_WR_Byte(0x8D,OLED_CMD);//set charge pump enable
	OLED_WR_Byte(0x14,OLED_CMD);//
	
	OLED_WR_Byte(0xAF,OLED_CMD);//--turn on oled panel
}  


void OLED_Float(unsigned char Y,unsigned char X,double real,unsigned char N) 
{
   unsigned char   i_Count=1;
   unsigned char   n[12]={0};
   long   j=1;  
   int    real_int=0;
   double decimal=0;
   unsigned int   real_decimal=0;
   if(real<0)
	 {
		 real_int=(int)(-real);
	 }
	 else
	 {
		 real_int=(int)real;
   }
	 decimal=real-real_int;
   real_decimal=decimal*1e4;
   while(real_int/10/j!=0)
   {
      j=j*10;i_Count++;  
   } 
   n[0]=(real_int/10000)%10; 
   n[1]=(real_int/1000)%10;
   n[2]=(real_int/100)%10;
   n[3]=(real_int/10)%10;
   n[4]=(real_int/1)%10; 
   n[5]='.';
   n[6]=(real_decimal/1000)%10;
   n[7]=(real_decimal/100)%10;
   n[8]=(real_decimal/10)%10;
   n[9]=real_decimal%10;
   n[6+N]='\0'; 
   for(j=0;j<10;j++) n[j]=n[j]+16+32;
	 if(real<0) 
	 {		 
		 i_Count+=1;
		 n[5-i_Count]='-';
	 }
   n[5]='.';
   n[6+N]='\0';   
   OLED_ShowString(X,Y,&n[5-i_Count],12); 
}




























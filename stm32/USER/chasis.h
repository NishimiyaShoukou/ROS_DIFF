#ifndef __chasis_H__
#define __chasis_H__


typedef enum
{
	
	START			,	//冲刺模式
	WAIT			, //等待模式
	SWAP      , //抓球模式
	PLACE     , //放球模式
	TURN      , //转向模式
	RETURN      , //返回模式
}CarStateTypedef;


void Main_Setup(void);
void Main_Loop(void);
void Main_Interrupt(void);

extern CarStateTypedef Miracle;

#endif

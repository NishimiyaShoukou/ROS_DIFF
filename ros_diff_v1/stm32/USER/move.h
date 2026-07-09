#ifndef __MOVE_H
#define __MOVE_H

#include "stm32f4xx.h"


typedef struct 
{
	 short speed[2];
	 short angel;
	
} com_data;

typedef struct
{ 
	float SetPoint;			//设定目标值
	float SetPointLast;
}Map_machine;

void Menu(void);
void Walk_run(void);
void Walk_run_interrupt(void);

void goto_1m(void);
#define LIMIT_MAX_MIN(x, max, min)	(((x) <= (min)) ? (min):(((x) >= (max)) ? (max) : (x)))
//extern  int PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY;
#endif

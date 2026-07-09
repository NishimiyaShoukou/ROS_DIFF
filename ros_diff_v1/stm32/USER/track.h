#ifndef _track_H
#define _track_H

#include "stm32f4xx.h"
enum 
{
	rin3=0,
	rin2,
	rin1,
	mid,
	lin1,
	lin2,
	lin3,
	
};
void Get_Led(void);

void track_line(void);
void track_line_back(void);
void Decision_cross(void);
extern uint8_t Led_buffer[7]; 
#endif


#ifndef __COMHANDLE_H
#define __COMHANDLE_H

#include "stm32f4xx.h"

#include "move.h"
//串口字节流写入到缓冲区
void UsartRxToBuf(uint8_t data);
void Usartdata_up(short leftspeed, short rightspeed,short angle,unsigned char Flag);
extern float a[3],w[3],angle[3],T;
extern float roll,gyrox,gyroz;
extern com_data Recdata,Sendata;
#endif /*__COMHANDLE_H*/


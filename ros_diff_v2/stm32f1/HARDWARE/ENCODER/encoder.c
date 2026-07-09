/******************************************************************************
 * @file    encoder.c
 * @brief   Quadrature encoder counter readout for both drive wheels.
 * @note    Reads TIM4 (left) and TIM2 (right) counter values once per call,
 *          returns the increments since the previous call, and clears the
 *          hardware counters. Signs are flipped on one wheel so that forward
 *          motion yields positive values on both axes.
 ******************************************************************************/
#include "encoder.h"

/********************************
 读取驱动电机编码器
 ********************************/

void get_encoder(int16_t *encoder)
{

//读取编码器计数值
	encoder[0] = TIM4->CNT;  // left wheel, PB6/PB7
	TIM4->CNT = 0;
	encoder[1] = -TIM2->CNT; // right wheel, PA0/PA1
	TIM2->CNT = 0;

}

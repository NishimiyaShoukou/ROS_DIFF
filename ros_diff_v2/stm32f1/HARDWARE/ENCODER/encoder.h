/******************************************************************************
 * @file    encoder.h
 * @brief   Quadrature encoder readout interface for both drive wheels.
 * @note    The implementation reads TIM4 (left) and TIM2 (right) and returns
 *          the increment since the previous call. Consumers typically feed the
 *          result into the speed / position estimators in ROBOT/pid.
 ******************************************************************************/
#ifndef __ENCODER_H__
#define __ENCODER_H__
#include "stm32f1xx.h"

void get_encoder(int16_t *encoder);

#endif

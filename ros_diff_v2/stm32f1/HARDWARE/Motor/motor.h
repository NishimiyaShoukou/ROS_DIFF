/******************************************************************************
 * @file    motor.h
 * @brief   Motor driver public interfaces and hardware abstractions.
 * @author  User
 * @date    2026-05-19
 * @version V1.0.0
 * @note    Part of the STM32F1 robot control project.
 ******************************************************************************/
#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "stm32f1xx.h"

void motor_set_pwm(int16_t left_pwm, int16_t right_pwm);

#endif

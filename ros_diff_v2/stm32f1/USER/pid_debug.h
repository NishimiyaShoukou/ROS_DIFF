/******************************************************************************
 * @file    pid_debug.h
 * @brief   Serial PID tuning helper for chassis debug.
 * @author  User
 * @date    2026-07-08
 * @version V1.0.0
 * @note    Part of the STM32F1 robot control project.
 ******************************************************************************/
#ifndef __PID_DEBUG_H__
#define __PID_DEBUG_H__

#include "stm32f1xx.h"

void pid_debug_rx_byte(uint8_t data);
void pid_debug_update(void);
uint8_t pid_debug_is_enabled(void);
void pid_debug_send_telemetry_20ms(void);

#endif

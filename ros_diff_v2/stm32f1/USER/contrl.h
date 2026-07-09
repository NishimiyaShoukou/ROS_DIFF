/******************************************************************************
 * @file    contrl.h
 * @brief   Robot control loop interfaces and shared declarations.
 * @author  User
 * @date    2026-05-19
 * @version V1.0.0
 * @note    Part of the STM32F1 robot control project.
 ******************************************************************************/

#ifndef __CONTRL_H__
#define __CONTRL_H__

#include "stm32f1xx.h"
#include "imu.h"

void chassis_init(void);
void chassis_set_target_speed(float left, float right);
void chassis_set_open_loop_pwm(int16_t left_pwm, int16_t right_pwm);
void chassis_disable_open_loop(void);
uint8_t chassis_is_open_loop_enabled(void);
int32_t chassis_get_left_encoder_count(void);
int32_t chassis_get_right_encoder_count(void);
void chassis_get_encoder_counts(int32_t *left, int32_t *right);
double chassis_get_position(void);
void chassis_stop(void);
float chassis_get_left_speed(void);
float chassis_get_right_speed(void);
float chassis_get_left_target_speed(void);
float chassis_get_right_target_speed(void);
float chassis_get_left_output_pwm(void);
float chassis_get_right_output_pwm(void);
float chassis_get_yaw(void);
uint8_t chassis_get_imu_data(imu_data_t *imu_data);
uint8_t chassis_init_imu(void);
uint8_t chassis_is_imu_ready(void);
uint8_t chassis_get_imu_error(void);
uint8_t chassis_get_imu_fail_count(void);
uint8_t chassis_update_imu(void);
void chassis_update_speed_loop_10ms(void);

#endif

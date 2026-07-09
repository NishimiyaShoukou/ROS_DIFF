/******************************************************************************
 * @file    kalman_filter.h
 * @brief   Lightweight Kalman filters for scalar and angle estimation.
 * @author  User
 * @date    2026-07-08
 * @version V1.0.0
 * @note    Part of the STM32F1 robot control project.
 ******************************************************************************/
#ifndef __KALMAN_FILTER_H__
#define __KALMAN_FILTER_H__

#include "stm32f1xx.h"

typedef struct
{
    float x;
    float p;
    float q;
    float r;
    float k;
} kalman_1d_t;

typedef struct
{
    float angle;
    float bias;
    float rate;
    float p[2][2];
    float q_angle;
    float q_bias;
    float r_measure;
} kalman_angle_t;

void kalman_1d_init(kalman_1d_t *kf, float init_value, float q, float r, float p);
void kalman_1d_reset(kalman_1d_t *kf, float init_value);
void kalman_1d_set_noise(kalman_1d_t *kf, float q, float r);
float kalman_1d_update(kalman_1d_t *kf, float measurement);

void kalman_angle_init(kalman_angle_t *kf, float init_angle);
void kalman_angle_reset(kalman_angle_t *kf, float init_angle);
void kalman_angle_set_noise(kalman_angle_t *kf, float q_angle, float q_bias, float r_measure);
float kalman_angle_update(kalman_angle_t *kf, float measured_angle, float measured_rate, float dt);

#endif

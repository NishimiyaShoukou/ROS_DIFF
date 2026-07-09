/******************************************************************************
 * @file    kalman_filter.c
 * @brief   Lightweight Kalman filters for scalar and angle estimation.
 * @author  User
 * @date    2026-07-08
 * @version V1.0.0
 * @note    Part of the STM32F1 robot control project.
 ******************************************************************************/
#include "kalman_filter.h"

void kalman_1d_init(kalman_1d_t *kf, float init_value, float q, float r, float p)
{
    if (kf == 0)
    {
        return;
    }

    kf->x = init_value;
    kf->p = p;
    kf->q = q;
    kf->r = r;
    kf->k = 0.0f;
}

void kalman_1d_reset(kalman_1d_t *kf, float init_value)
{
    if (kf == 0)
    {
        return;
    }

    kf->x = init_value;
    kf->p = 1.0f;
    kf->k = 0.0f;
}

void kalman_1d_set_noise(kalman_1d_t *kf, float q, float r)
{
    if (kf == 0)
    {
        return;
    }

    kf->q = q;
    kf->r = r;
}

float kalman_1d_update(kalman_1d_t *kf, float measurement)
{
    float s;

    if (kf == 0)
    {
        return measurement;
    }

    kf->p += kf->q;
    s = kf->p + kf->r;

    if (s <= 0.0f)
    {
        return kf->x;
    }

    kf->k = kf->p / s;
    kf->x += kf->k * (measurement - kf->x);
    kf->p *= (1.0f - kf->k);

    return kf->x;
}

void kalman_angle_init(kalman_angle_t *kf, float init_angle)
{
    if (kf == 0)
    {
        return;
    }

    kf->angle = init_angle;
    kf->bias = 0.0f;
    kf->rate = 0.0f;
    kf->p[0][0] = 0.0f;
    kf->p[0][1] = 0.0f;
    kf->p[1][0] = 0.0f;
    kf->p[1][1] = 0.0f;
    kf->q_angle = 0.001f;
    kf->q_bias = 0.003f;
    kf->r_measure = 0.03f;
}

void kalman_angle_reset(kalman_angle_t *kf, float init_angle)
{
    if (kf == 0)
    {
        return;
    }

    kf->angle = init_angle;
    kf->bias = 0.0f;
    kf->rate = 0.0f;
    kf->p[0][0] = 0.0f;
    kf->p[0][1] = 0.0f;
    kf->p[1][0] = 0.0f;
    kf->p[1][1] = 0.0f;
}

void kalman_angle_set_noise(kalman_angle_t *kf, float q_angle, float q_bias, float r_measure)
{
    if (kf == 0)
    {
        return;
    }

    kf->q_angle = q_angle;
    kf->q_bias = q_bias;
    kf->r_measure = r_measure;
}

float kalman_angle_update(kalman_angle_t *kf, float measured_angle, float measured_rate, float dt)
{
    float s;
    float y;
    float k0;
    float k1;
    float p00_temp;
    float p01_temp;

    if (kf == 0)
    {
        return measured_angle;
    }

    if (dt <= 0.0f)
    {
        return kf->angle;
    }

    kf->rate = measured_rate - kf->bias;
    kf->angle += dt * kf->rate;

    kf->p[0][0] += dt * (dt * kf->p[1][1] - kf->p[0][1] - kf->p[1][0] + kf->q_angle);
    kf->p[0][1] -= dt * kf->p[1][1];
    kf->p[1][0] -= dt * kf->p[1][1];
    kf->p[1][1] += kf->q_bias * dt;

    s = kf->p[0][0] + kf->r_measure;
    if (s <= 0.0f)
    {
        return kf->angle;
    }

    k0 = kf->p[0][0] / s;
    k1 = kf->p[1][0] / s;
    y = measured_angle - kf->angle;

    kf->angle += k0 * y;
    kf->bias += k1 * y;

    p00_temp = kf->p[0][0];
    p01_temp = kf->p[0][1];

    kf->p[0][0] -= k0 * p00_temp;
    kf->p[0][1] -= k0 * p01_temp;
    kf->p[1][0] -= k1 * p00_temp;
    kf->p[1][1] -= k1 * p01_temp;

    return kf->angle;
}

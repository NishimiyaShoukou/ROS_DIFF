/******************************************************************************
 * @file    imu.h
 * @brief   IMU public interfaces and data container.
 * @author  User
 * @date    2026-05-19
 * @version V1.0.0
 * @note    Part of the STM32F1 robot control project.
 ******************************************************************************/
#ifndef __IMU_H__
#define __IMU_H__

#include "stm32f1xx.h"

#define IMU_OK                 0U
#define IMU_ERR_NOT_READY      0xF0U
#define IMU_ERR_NULL_PTR       0xF1U
#define IMU_ERR_DMP_READ       0xF2U
#define IMU_ERR_ACCEL_READ     0xF3U
#define IMU_ERR_GYRO_READ      0xF4U
#define IMU_ERR_TEMP_READ      0xF5U

typedef struct
{
    float pitch;
    float roll;
    float yaw;
    short aacx;
    short aacy;
    short aacz;
    short gyrox;
    short gyroy;
    short gyroz;
    short temp;
} imu_data_t;

uint8_t imu_init(void);
void imu_reset_state(void);
uint8_t imu_is_ready(void);
uint8_t imu_get_last_error(void);
uint8_t imu_get_fail_count(void);
uint8_t imu_data_update(imu_data_t *imu_data);

#endif

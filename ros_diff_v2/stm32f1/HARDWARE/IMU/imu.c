/******************************************************************************
 * @file    imu.c
 * @brief   IMU initialization and safe data update implementation.
 * @author  User
 * @date    2026-05-19
 * @version V1.0.0
 * @note    Part of the STM32F1 robot control project.
 ******************************************************************************/
#include "imu.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#define IMU_MAX_DMP_FAIL_COUNT  5U

static uint8_t g_imu_ready = 0;
static uint8_t g_imu_last_error = IMU_ERR_NOT_READY;
static uint8_t g_imu_dmp_fail_count = 0;

static uint8_t imu_read_temperature(short *temp)
{
    uint8_t buf[2];
    short raw;
    float value;

    if (temp == 0)
    {
        return IMU_ERR_NULL_PTR;
    }

    if (MPU_Read_Len(MPU_ADDR, MPU_TEMP_OUTH_REG, 2, buf) != 0)
    {
        return IMU_ERR_TEMP_READ;
    }

    raw = (short)(((uint16_t)buf[0] << 8) | buf[1]);
    value = 36.53f + ((float)raw / 340.0f);
    *temp = (short)(value * 100.0f);

    return IMU_OK;
}

uint8_t imu_init(void)
{
    uint8_t res;

    res = mpu_dmp_init();
    if (res == IMU_OK)
    {
        g_imu_ready = 1;
        g_imu_last_error = IMU_OK;
        g_imu_dmp_fail_count = 0;
    }
    else
    {
        g_imu_ready = 0;
        g_imu_last_error = res;
        if (g_imu_dmp_fail_count < 0xFFU)
        {
            g_imu_dmp_fail_count++;
        }
    }

    return res;
}

void imu_reset_state(void)
{
    g_imu_ready = 0;
    g_imu_last_error = IMU_ERR_NOT_READY;
    g_imu_dmp_fail_count = 0;
}

uint8_t imu_is_ready(void)
{
    return g_imu_ready;
}

uint8_t imu_get_last_error(void)
{
    return g_imu_last_error;
}

uint8_t imu_get_fail_count(void)
{
    return g_imu_dmp_fail_count;
}

uint8_t imu_data_update(imu_data_t *imu_data)
{
    float pitch;
    float roll;
    float yaw;
    short temp;
    uint8_t raw_error = IMU_OK;
    uint8_t res;

    if (imu_data == 0)
    {
        g_imu_last_error = IMU_ERR_NULL_PTR;
        return IMU_ERR_NULL_PTR;
    }

    if (!g_imu_ready)
    {
        g_imu_last_error = IMU_ERR_NOT_READY;
        return IMU_ERR_NOT_READY;
    }

    res = mpu_dmp_get_data(&pitch, &roll, &yaw);
    if (res != IMU_OK)
    {
        g_imu_last_error = IMU_ERR_DMP_READ;
        if (g_imu_dmp_fail_count < 0xFFU)
        {
            g_imu_dmp_fail_count++;
        }

        if (g_imu_dmp_fail_count >= IMU_MAX_DMP_FAIL_COUNT)
        {
            (void)mpu_reset_fifo();
            g_imu_ready = 0;
        }

        return IMU_ERR_DMP_READ;
    }

    imu_data->pitch = pitch;
    imu_data->roll = roll;
    imu_data->yaw = yaw;
    g_imu_dmp_fail_count = 0;

    if (imu_read_temperature(&temp) == IMU_OK)
    {
        imu_data->temp = temp;
    }
    else
    {
        raw_error = IMU_ERR_TEMP_READ;
    }

    if (MPU_Get_Accelerometer(&imu_data->aacx, &imu_data->aacy, &imu_data->aacz) != 0)
    {
        raw_error = IMU_ERR_ACCEL_READ;
    }

    if (MPU_Get_Gyroscope(&imu_data->gyrox, &imu_data->gyroy, &imu_data->gyroz) != 0)
    {
        raw_error = IMU_ERR_GYRO_READ;
    }

    g_imu_last_error = raw_error;
    return raw_error;
}

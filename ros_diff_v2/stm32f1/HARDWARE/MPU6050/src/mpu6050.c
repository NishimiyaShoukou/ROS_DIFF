/******************************************************************************
 * @file    mpu6050.c
 * @brief   MPU6050 register access, initialization, and sensor data routines.
 * @author  User
 * @date    2026-05-19
 * @version V1.0.0
 * @note    Part of the STM32F1 robot control project.
 ******************************************************************************/

#include "mpu6050.h"

uint8_t MPU_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    uint8_t i;

    if (!I2C_Start())
    {
        I2C_Stop();
        return 1;
    }

    I2C_SendByte((addr << 1) | 0);
    if (I2C_WaitAck() == 0)
    {
        I2C_Stop();
        return 1;
    }

    I2C_SendByte(reg);
    if (I2C_WaitAck() == 0)
    {
        I2C_Stop();
        return 1;
    }

    for (i = 0; i < len; i++)
    {
        I2C_SendByte(buf[i]);
        if (I2C_WaitAck() == 0)
        {
            I2C_Stop();
            return 1;
        }
    }

    I2C_Stop();
    return 0;
}

uint8_t MPU_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (!I2C_Start())
    {
        I2C_Stop();
        return 1;
    }

    I2C_SendByte((addr << 1) | 0);
    if (I2C_WaitAck() == 0)
    {
        I2C_Stop();
        return 1;
    }

    I2C_SendByte(reg);
    if (I2C_WaitAck() == 0)
    {
        I2C_Stop();
        return 1;
    }

    if (!I2C_Start())
    {
        I2C_Stop();
        return 1;
    }

    I2C_SendByte((addr << 1) | 1);
    if (I2C_WaitAck() == 0)
    {
        I2C_Stop();
        return 1;
    }

    while (len)
    {
        *buf = I2C_RadeByte();
        len--;
        buf++;

        if (len == 0)
        {
            I2C_NoAck();
        }
        else
        {
            I2C_Ack();
        }
    }

    I2C_Stop();
    return 0;
}

uint8_t MPU_Single_Write(uint8_t reg, uint8_t data)
{
    if (!I2C_Start())
    {
        I2C_Stop();
        return 1;
    }

    I2C_SendByte((MPU_ADDR << 1) | 0);
    if (I2C_WaitAck() == 0)
    {
        I2C_Stop();
        return 1;
    }

    I2C_SendByte(reg);
    if (I2C_WaitAck() == 0)
    {
        I2C_Stop();
        return 1;
    }

    I2C_SendByte(data);
    if (I2C_WaitAck() == 0)
    {
        I2C_Stop();
        return 1;
    }

    I2C_Stop();
    return 0;
}

uint8_t MPU_Write_Byte(uint8_t reg, uint8_t data)
{
    return MPU_Single_Write(reg, data);
}

uint8_t MPU_Read_Byte(uint8_t reg)
{
    uint8_t res;

    if (!I2C_Start())
    {
        I2C_Stop();
        return 0xFF;
    }

    I2C_SendByte((MPU_ADDR << 1) | 0);
    if (I2C_WaitAck() == 0)
    {
        I2C_Stop();
        return 0xFF;
    }

    I2C_SendByte(reg);
    if (I2C_WaitAck() == 0)
    {
        I2C_Stop();
        return 0xFF;
    }

    if (!I2C_Start())
    {
        I2C_Stop();
        return 0xFF;
    }

    I2C_SendByte((MPU_ADDR << 1) | 1);
    if (I2C_WaitAck() == 0)
    {
        I2C_Stop();
        return 0xFF;
    }

    res = I2C_RadeByte();
    I2C_NoAck();
    I2C_Stop();

    return res;
}

uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
    return MPU_Single_Write(MPU_GYRO_CFG_REG, fsr << 3);
}

uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
    return MPU_Single_Write(MPU_ACCEL_CFG_REG, fsr << 3);
}

uint8_t MPU_Set_LPF(uint16_t lpf)
{
    uint8_t data;

    if (lpf >= 188)
    {
        data = 1;
    }
    else if (lpf >= 98)
    {
        data = 2;
    }
    else if (lpf >= 42)
    {
        data = 3;
    }
    else if (lpf >= 20)
    {
        data = 4;
    }
    else if (lpf >= 10)
    {
        data = 5;
    }
    else
    {
        data = 6;
    }

    return MPU_Single_Write(MPU_CFG_REG, data);
}

uint8_t MPU_Set_Rate(uint16_t rate)
{
    uint8_t data;
    uint8_t res;

    if (rate > 1000)
    {
        rate = 1000;
    }
    if (rate < 4)
    {
        rate = 4;
    }

    data = (uint8_t)(1000 / rate - 1);
    res = MPU_Single_Write(MPU_SAMPLE_RATE_REG, data);
    if (res != 0)
    {
        return res;
    }

    return MPU_Set_LPF(rate / 2);
}

short MPU_Get_Temperature(void)
{
    uint8_t buf[2];
    short raw;
    float temp;

    if (MPU_Read_Len(MPU_ADDR, MPU_TEMP_OUTH_REG, 2, buf) != 0)
    {
        return 0;
    }

    raw = (short)(((uint16_t)buf[0] << 8) | buf[1]);
    temp = 36.53f + ((float)raw / 340.0f);

    return (short)(temp * 100.0f);
}

uint8_t MPU_Get_Gyroscope(short *gx, short *gy, short *gz)
{
    uint8_t buf[6];
    uint8_t res;

    res = MPU_Read_Len(MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buf);
    if (res == 0)
    {
        *gx = (short)(((uint16_t)buf[0] << 8) | buf[1]);
        *gy = (short)(((uint16_t)buf[2] << 8) | buf[3]);
        *gz = (short)(((uint16_t)buf[4] << 8) | buf[5]);
    }

    return res;
}

uint8_t MPU_Get_Accelerometer(short *ax, short *ay, short *az)
{
    uint8_t buf[6];
    uint8_t res;

    res = MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf);
    if (res == 0)
    {
        *ax = (short)(((uint16_t)buf[0] << 8) | buf[1]);
        *ay = (short)(((uint16_t)buf[2] << 8) | buf[3]);
        *az = (short)(((uint16_t)buf[4] << 8) | buf[5]);
    }

    return res;
}

uint8_t MPU_Init(void)
{
    uint8_t res;

    I2C_GPIO_Config();

    if (MPU_Single_Write(MPU_PWR_MGMT1_REG, 0x80) != 0)
    {
        return 1;
    }
    HAL_Delay(100);

    if (MPU_Single_Write(MPU_PWR_MGMT1_REG, 0x00) != 0)
    {
        return 2;
    }
    if (MPU_Set_Gyro_Fsr(3) != 0)
    {
        return 3;
    }
    if (MPU_Set_Accel_Fsr(0) != 0)
    {
        return 4;
    }
    if (MPU_Set_Rate(50) != 0)
    {
        return 5;
    }
    if (MPU_Single_Write(MPU_INT_EN_REG, 0x00) != 0)
    {
        return 6;
    }
    if (MPU_Single_Write(MPU_USER_CTRL_REG, 0x00) != 0)
    {
        return 7;
    }
    if (MPU_Single_Write(MPU_FIFO_EN_REG, 0x00) != 0)
    {
        return 8;
    }
    if (MPU_Single_Write(MPU_INTBP_CFG_REG, 0x80) != 0)
    {
        return 9;
    }

    res = MPU_Read_Byte(MPU_DEVICE_ID_REG);
    if ((res != MPU_WHO_AM_I_ID_OLD) && (res != MPU_WHO_AM_I_ID_NEW))
    {
        return 10;
    }

    if (MPU_Single_Write(MPU_PWR_MGMT1_REG, 0x01) != 0)
    {
        return 11;
    }
    if (MPU_Single_Write(MPU_PWR_MGMT2_REG, 0x00) != 0)
    {
        return 12;
    }
    if (MPU_Set_Rate(50) != 0)
    {
        return 13;
    }

    return 0;
}

/******************************************************************************
 * @file    comHandle.h
 * @brief   Communication command interfaces and shared data types.
 * @author  User
 * @date    2026-05-19
 * @version V2.1.0
 * @note    Part of the STM32F1 robot control project.
 ******************************************************************************/

#ifndef __COMHANDLE_H
#define __COMHANDLE_H

#include "stm32f1xx.h"

#define COM_STATUS_IMU_READY  (1U << 0)
#define COM_STATUS_REMOTE     (1U << 1)
#define COM_STATUS_IMU_ERROR  (1U << 2)
#define COM_STATUS_AUTO       (1U << 3)

typedef struct
{
    short speed[2];
    short angel;
}com_data_t;
uint8_t com_take_received_data(com_data_t *out);
uint8_t com_crc8_maxim(const uint8_t *data, uint16_t len);
//把接收到的单字节写入接收缓冲区
void usart_rx_to_buf(uint8_t data);
void usart_tx(uint8_t *data, uint8_t len);
void usart_tx_isr(void);
void usart_data_up(short left_speed_scaled, short right_speed_scaled,
                   int32_t left_encoder_count, int32_t right_encoder_count,
                   short yaw_scaled, unsigned char status,
                   uint16_t battery_voltage_mv, uint8_t battery_percent,
                   uint8_t battery_state);
#endif /*__COMHANDLE_H*/

/******************************************************************************
 * @file    chassis.c
 * @brief   Chassis control, kinematics, and motor speed coordination.
 * @author  User
 * @date    2026-05-19
 * @version V2.1.0
 * @note    Part of the STM32F1 robot control project.
 ******************************************************************************/

#include "chassis.h"
#include "stm32f1xx.h"
#include "tim.h"
#include "usart.h"
#include "contrl.h"
#include "comHandle.h"
#include "robot.h"
#include "move.h"
#include "pid_debug.h"
#include "battery.h"

#define IMU_UPDATE_PERIOD_MS       20U
#define IMU_INIT_RETRY_PERIOD_MS   500U
#define BATTERY_UPDATE_PERIOD_MS   500U

CarStateTypedef Miracle = WAIT;
static volatile uint8_t g_robot_upload_due = 0U;

static uint8_t take_robot_upload_due(void)
{
    uint8_t due;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    due = g_robot_upload_due;
    g_robot_upload_due = 0U;
    __set_PRIMASK(primask);
    return due;
}

static void imu_background_task(void)
{
    static uint32_t last_init_tick = 0;
    static uint32_t last_update_tick = 0;
    static uint8_t first_init_try = 1;
    uint32_t now = HAL_GetTick();

    if (!chassis_is_imu_ready())
    {
        if (first_init_try || ((uint32_t)(now - last_init_tick) >= IMU_INIT_RETRY_PERIOD_MS))
        {
            first_init_try = 0;
            last_init_tick = now;
            (void)chassis_init_imu();
        }
        return;
    }

    if ((uint32_t)(now - last_update_tick) >= IMU_UPDATE_PERIOD_MS)
    {
        last_update_tick = now;
        (void)chassis_update_imu();
    }
}

static void battery_background_task(void)
{
    static uint32_t last_update_tick = 0U;
    static uint8_t first_update = 1U;
    static battery_data_t battery_data;
    uint32_t now = HAL_GetTick();

    if (first_update ||
        (uint32_t)(now - last_update_tick) >= BATTERY_UPDATE_PERIOD_MS)
    {
        first_update = 0U;
        last_update_tick = now;
        (void)battery_update(&battery_data);
    }
}

static void uart2_ttl_test_task(void)
{
#if UART2_TTL_TEST_ENABLE
    static uint32_t last_tick = 0;
    static const uint8_t msg[] = "$UART2,ALIVE\r\n";
    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - last_tick) >= UART2_TTL_TEST_PERIOD_MS)
    {
        last_tick = now;
        usart_tx((uint8_t *)msg, sizeof(msg) - 1U);
    }
#endif
}

void main_setup(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    HAL_Delay(100);
    chassis_init();
    robot_init(&g_robot);
    battery_init();
#if MOVE_AUTO_START_1M_TEST
    goto_1m();
#endif
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
    __HAL_UART_DISABLE_IT(&huart2, UART_IT_IDLE);
    HAL_Delay(100);
}

void main_loop(void)
{
    pid_debug_update();
    uart2_ttl_test_task();
    battery_background_task();
    if (take_robot_upload_due())
    {
        robot_upload_status_20ms();
    }
    imu_background_task();
    walk_run();
}

void Main_Interrupt(void)
{
    static uint8_t speed_loop_cnt = 0;
    static uint8_t robot_comm_cnt = 0;

    robot_update_1ms(&g_robot);

    speed_loop_cnt++;
    robot_comm_cnt++;

    if (robot_comm_cnt >= ROBOT_COMM_PERIOD_MS)
    {
        robot_update_20ms(&g_robot);
        g_robot_upload_due = 1U;
        robot_comm_cnt = 0;
    }

    if (speed_loop_cnt >= CHASSIS_SPEED_LOOP_PERIOD_MS)
    {
        chassis_update_speed_loop_10ms();
        speed_loop_cnt = 0;
    }
}

/******************************************************************************
 * @file    motor.c
 * @brief   Motor driver implementation entry point for the robot chassis.
 * @author  User
 * @date    2026-05-19
 * @version V1.0.0
 * @note    Part of the STM32F1 robot control project.
 ******************************************************************************/
#include "motor.h"

#define MOTOR_PWM_MAX  1000

static int16_t motor_limit_pwm(int16_t pwm)
{
    if (pwm > MOTOR_PWM_MAX)
    {
        return MOTOR_PWM_MAX;
    }

    if (pwm < -MOTOR_PWM_MAX)
    {
        return -MOTOR_PWM_MAX;
    }

    return pwm;
}

static void motor_set_left_pwm(int16_t pwm)
{
    pwm = motor_limit_pwm(pwm);
    TIM1->CCR1 = 0;

    if (pwm > 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
        TIM1->CCR1 = (uint16_t)pwm;
    }
    else if (pwm < 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
        TIM1->CCR1 = (uint16_t)(-pwm);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    }
}

static void motor_set_right_pwm(int16_t pwm)
{
    pwm = motor_limit_pwm(pwm);
    TIM1->CCR2 = 0;

    if (pwm > 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
        TIM1->CCR2 = (uint16_t)pwm;
    }
    else if (pwm < 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
        TIM1->CCR2 = (uint16_t)(-pwm);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    }
}

void motor_set_pwm(int16_t left_pwm, int16_t right_pwm)
{
    /*
     * PCB v2 TB6612 routing:
     * MOTOR1/left  = PWMB PA8/TIM1_CH1 + BIN1/BIN2 PB14/PB15.
     * MOTOR2/right = PWMA PA9/TIM1_CH2 + AIN2/AIN1 PB12/PB13.
     */
    motor_set_left_pwm(left_pwm);
    motor_set_right_pwm(right_pwm);
}

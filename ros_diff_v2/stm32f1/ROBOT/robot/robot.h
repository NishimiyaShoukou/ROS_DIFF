/******************************************************************************
 * @file    robot.h
 * @brief   Robot state type and top-level periodic update interfaces.
 * @author  User
 * @date    2026-05-19
 * @version V1.0.0
 * @note    Part of the STM32F1 robot control project.
 ******************************************************************************/
#ifndef __ROBOT_H__
#define __ROBOT_H__
#include "stm32f1xx.h"

typedef enum 
{
    ROBOT_MODE_WAIT = 0,
    ROBOT_MODE_AUTO,
    ROBOT_MODE_REMOTE,
    ROBOT_MODE_ERROR
}robot_mode_t;
typedef struct
{
    short speed[2];
    short angle;
} robot_cmd_t;

typedef struct
{
    robot_mode_t mode;
    robot_cmd_t cmd;
    uint32_t tick_ms;
    uint32_t last_cmd_tick;
}robot_t;


extern robot_t g_robot;
void robot_init(robot_t *robot);
void robot_setcmd(robot_t *robot, const robot_cmd_t *cmd);
void robot_enter_auto_mode(robot_t *robot);
void robot_stop(robot_t *robot);
void robot_update_1ms(robot_t *robot);
void robot_update_20ms(robot_t *robot);
void robot_upload_status_20ms(void);


#endif

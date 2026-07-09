/******************************************************************************
 * @file    robot.c
 * @brief   Robot state coordination and top-level periodic update routines.
 * @author  User
 * @date    2026-05-19
 * @version V2.1.0
 * @note    Part of the STM32F1 robot control project.
 ******************************************************************************/
#include "robot.h"


#include "chassis.h"
#include "contrl.h"
#include "comHandle.h"
#include "pid_debug.h"
#include "battery.h"

#define ROBOT_CMD_TIMEOUT_MS       300U
#define ROBOT_UPPER_SPEED_SCALE    100.0f
#define ROBOT_FEEDBACK_SPEED_SCALE 100.0f
#define ROBOT_FEEDBACK_YAW_SCALE   100.0f
#define ROBOT_LEFT_CMD_SIGN        1.0f
#define ROBOT_RIGHT_CMD_SIGN       1.0f

robot_t g_robot;

static short robot_scale_to_short(float value, float scale)
{
    float scaled = value * scale;

    if (scaled > 32767.0f)
    {
        return 32767;
    }
    if (scaled < -32768.0f)
    {
        return -32768;
    }
    scaled += (scaled >= 0.0f) ? 0.5f : -0.5f;
    return (short)scaled;
}

static void robot_setcmd_from_com(robot_t *robot, const com_data_t *cmd)
{
    robot_cmd_t robot_cmd;

    robot_cmd.speed[0] = cmd->speed[0];
    robot_cmd.speed[1] = cmd->speed[1];
    robot_cmd.angle = cmd->angel;
    robot_setcmd(robot, &robot_cmd);
}

void robot_init(robot_t *robot)
{
	robot->mode = ROBOT_MODE_WAIT;
	robot->tick_ms = 0;
	robot->last_cmd_tick = 0;
	robot->cmd.speed[0] = 0;
	robot->cmd.speed[1] = 0;
	robot->cmd.angle = 0;

}

void robot_setcmd(robot_t *robot, const robot_cmd_t *cmd)
{
	  robot->cmd = *cmd;
    robot->last_cmd_tick = robot->tick_ms;
    robot->mode = ROBOT_MODE_REMOTE;
}

void robot_enter_auto_mode(robot_t *robot)
{
    robot->cmd.speed[0] = 0;
    robot->cmd.speed[1] = 0;
    robot->cmd.angle = 0;
    chassis_stop();
    robot->mode = ROBOT_MODE_AUTO;
}

void robot_stop(robot_t *robot)
{
    robot->cmd.speed[0] = 0;
    robot->cmd.speed[1] = 0;
    robot->cmd.angle = 0;
    chassis_stop();
    robot->mode = ROBOT_MODE_WAIT;
}

void robot_update_1ms(robot_t *robot)
{
    robot->tick_ms++;
}

void robot_update_20ms(robot_t *robot)
{
    com_data_t recv_cmd;

    if (pid_debug_is_enabled())
    {
        return;
    }

    if (com_take_received_data(&recv_cmd))
    {
        robot_setcmd_from_com(robot, &recv_cmd);
    }

    if (robot->mode != ROBOT_MODE_REMOTE)
    {
        return;
    }

    if ((uint32_t)(robot->tick_ms - robot->last_cmd_tick) > ROBOT_CMD_TIMEOUT_MS)
    {
        robot_stop(robot);
        return;
    }

    chassis_set_target_speed(
        ROBOT_LEFT_CMD_SIGN * ((float)robot->cmd.speed[0] / ROBOT_UPPER_SPEED_SCALE),
        ROBOT_RIGHT_CMD_SIGN * ((float)robot->cmd.speed[1] / ROBOT_UPPER_SPEED_SCALE)
    );
}
void robot_upload_status_20ms(void)
{
    uint8_t status = 0U;
    int32_t left_encoder_count;
    int32_t right_encoder_count;

#if UART2_TTL_TEST_ENABLE
    return;
#endif

    if (pid_debug_is_enabled())
    {
        return;
    }

    if (chassis_is_imu_ready())
    {
        status |= COM_STATUS_IMU_READY;
    }
    if (g_robot.mode == ROBOT_MODE_REMOTE)
    {
        status |= COM_STATUS_REMOTE;
    }
    if (g_robot.mode == ROBOT_MODE_AUTO)
    {
        status |= COM_STATUS_AUTO;
    }
    if (chassis_get_imu_error() != IMU_OK)
    {
        status |= COM_STATUS_IMU_ERROR;
    }
    chassis_get_encoder_counts(&left_encoder_count, &right_encoder_count);
    usart_data_up(
        robot_scale_to_short(chassis_get_left_speed(), ROBOT_FEEDBACK_SPEED_SCALE),
        robot_scale_to_short(chassis_get_right_speed(), ROBOT_FEEDBACK_SPEED_SCALE),
        left_encoder_count,
        right_encoder_count,
        robot_scale_to_short(chassis_get_yaw(), ROBOT_FEEDBACK_YAW_SCALE),
        status,
        battery_get_voltage_mv(),
        battery_get_percent(),
        (uint8_t)battery_get_state());
}

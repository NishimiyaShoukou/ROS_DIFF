/******************************************************************************
 * @file    move.c
 * @brief   Encoder-based straight distance test for chassis calibration.
 ******************************************************************************/

#include "move.h"

#include <math.h>

#include "contrl.h"
#include "pid_debug.h"
#include "robot.h"

#define MOVE_ENCODER_COUNTS_PER_REV       1040.0f
#define MOVE_WHEEL_CIRCUMFERENCE_M        0.144764589f
#define MOVE_TEST_CRUISE_RPM              35.0f
#define MOVE_TEST_MIN_RPM                 9.0f
#define MOVE_TEST_STRAIGHT_KP             0.025f
#define MOVE_TEST_MAX_CORRECTION_RPM      7.0f
#define MOVE_TEST_DECEL_DISTANCE_M        0.18f
#define MOVE_TEST_STOP_TOLERANCE_M        0.004f
#define MOVE_TEST_STALL_TIMEOUT_MS        3000U
#define MOVE_TEST_TIMEOUT_MARGIN_MS       5000U
#define MOVE_TEST_PROGRESS_COUNTS         4.0f

typedef struct
{
    move_test_state_t state;
    int32_t start_left;
    int32_t start_right;
    float target_counts;
    float last_progress_counts;
    uint32_t start_tick;
    uint32_t last_progress_tick;
    uint32_t last_update_tick;
    uint32_t total_timeout_ms;
} move_test_t;

static move_test_t s_test;

static float clamp_float(float value, float low, float high)
{
    if (value < low)
    {
        return low;
    }
    if (value > high)
    {
        return high;
    }
    return value;
}

static int32_t encoder_delta(int32_t current, int32_t start)
{
    return (int32_t)((uint32_t)current - (uint32_t)start);
}

static void move_finish(move_test_state_t state)
{
    s_test.state = state;
    if (g_robot.mode == ROBOT_MODE_AUTO)
    {
        robot_stop(&g_robot);
    }
}

uint8_t move_start_distance_test(float distance_m)
{
    int32_t left;
    int32_t right;

    if (s_test.state == MOVE_TEST_RUNNING || pid_debug_is_enabled() ||
        fabsf(distance_m) < 0.05f || fabsf(distance_m) > 5.0f)
    {
        return 0U;
    }

    robot_enter_auto_mode(&g_robot);
    chassis_get_encoder_counts(&left, &right);
    s_test.start_left = left;
    s_test.start_right = right;
    s_test.target_counts =
        distance_m * MOVE_ENCODER_COUNTS_PER_REV / MOVE_WHEEL_CIRCUMFERENCE_M;
    s_test.last_progress_counts = 0.0f;
    s_test.start_tick = HAL_GetTick();
    s_test.last_progress_tick = s_test.start_tick;
    s_test.last_update_tick = s_test.start_tick;
    s_test.total_timeout_ms = (uint32_t)(
        (fabsf(distance_m) /
         (MOVE_WHEEL_CIRCUMFERENCE_M * MOVE_TEST_CRUISE_RPM / 60.0f)) *
        2000.0f) + MOVE_TEST_TIMEOUT_MARGIN_MS;
    s_test.state = MOVE_TEST_RUNNING;
    return 1U;
}

void goto_1m(void)
{
    /* Convenience wrapper is one-shot per boot. Use the generic start
     * function explicitly if a later test needs to be started again. */
    if (s_test.state == MOVE_TEST_IDLE)
    {
        (void)move_start_distance_test(1.0f);
    }
}

void move_cancel_test(void)
{
    if (s_test.state == MOVE_TEST_RUNNING)
    {
        move_finish(MOVE_TEST_ABORTED);
    }
}

move_test_state_t move_get_test_state(void)
{
    return s_test.state;
}

void walk_run(void)
{
    int32_t left;
    int32_t right;
    float direction;
    float left_progress;
    float right_progress;
    float progress;
    float remaining;
    float decel_counts;
    float tolerance_counts;
    float base_rpm;
    float correction_rpm;
    uint32_t now;
    if (s_test.state != MOVE_TEST_RUNNING)
    {
        return;
    }

    /* A valid upper-computer command changes the robot to REMOTE mode. */
    if (g_robot.mode != ROBOT_MODE_AUTO || pid_debug_is_enabled())
    {
        s_test.state = MOVE_TEST_ABORTED;
        return;
    }

    now = HAL_GetTick();
    if ((uint32_t)(now - s_test.last_update_tick) < 20U)
    {
        return;
    }
    s_test.last_update_tick = now;

    if ((uint32_t)(now - s_test.start_tick) > s_test.total_timeout_ms)
    {
        move_finish(MOVE_TEST_TIMEOUT);
        return;
    }

    chassis_get_encoder_counts(&left, &right);
    direction = (s_test.target_counts >= 0.0f) ? 1.0f : -1.0f;
    left_progress = direction * (float)encoder_delta(left, s_test.start_left);
    right_progress = direction * (float)encoder_delta(right, s_test.start_right);
    progress = 0.5f * (left_progress + right_progress);
    remaining = fabsf(s_test.target_counts) - progress;

    tolerance_counts =
        MOVE_TEST_STOP_TOLERANCE_M * MOVE_ENCODER_COUNTS_PER_REV /
        MOVE_WHEEL_CIRCUMFERENCE_M;
    if (remaining <= tolerance_counts)
    {
        move_finish(MOVE_TEST_DONE);
        return;
    }

    if (progress >= s_test.last_progress_counts + MOVE_TEST_PROGRESS_COUNTS)
    {
        s_test.last_progress_counts = progress;
        s_test.last_progress_tick = now;
    }
    else if ((uint32_t)(now - s_test.last_progress_tick) >
             MOVE_TEST_STALL_TIMEOUT_MS)
    {
        move_finish(MOVE_TEST_TIMEOUT);
        return;
    }

    decel_counts =
        MOVE_TEST_DECEL_DISTANCE_M * MOVE_ENCODER_COUNTS_PER_REV /
        MOVE_WHEEL_CIRCUMFERENCE_M;
    base_rpm = MOVE_TEST_CRUISE_RPM;
    if (remaining < decel_counts)
    {
        base_rpm = MOVE_TEST_MIN_RPM +
            (MOVE_TEST_CRUISE_RPM - MOVE_TEST_MIN_RPM) *
            (remaining / decel_counts);
    }

    correction_rpm = clamp_float(
        MOVE_TEST_STRAIGHT_KP * (left_progress - right_progress),
        -MOVE_TEST_MAX_CORRECTION_RPM,
        MOVE_TEST_MAX_CORRECTION_RPM);
    chassis_set_target_speed(
        direction * (base_rpm - correction_rpm),
        direction * (base_rpm + correction_rpm));
}

void walk_run_interrupt(void)
{
}

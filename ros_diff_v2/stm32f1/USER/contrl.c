/******************************************************************************
 * @file    contrl.c
 * @brief   Robot control loop and command coordination routines.
 * @author  User
 * @date    2026-05-19
 * @version V2.0.0
 * @note    Part of the STM32F1 robot control project.
 ******************************************************************************/

#include "contrl.h"
#include "imu.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "speed_ramp.h"
#include <stdlib.h>

#define CHASSIS_TARGET_STOP_BAND    0.2f
#define CHASSIS_SPEED_STOP_BAND     1.0f
#define CHASSIS_FEEDFORWARD_OFFSET  40.0f
#define CHASSIS_FEEDFORWARD_GAIN    2.6f
#define CHASSIS_ENCODER_LINES       13.0f
#define CHASSIS_REDUCTION_RATIO     20.0f
/* 10 ms loop, TIM encoder x4: count / (13 * 20) * 1500 = output-shaft rpm. */
#define CHASSIS_SPEED_SCALE_RPM     1500.0f
#define CHASSIS_OPEN_LOOP_TIMEOUT_MS 1000U
#define CHASSIS_LOW_TARGET_BAND     30.0f
#define CHASSIS_LOW_TARGET_PWM_MAX  250.0f
#define CHASSIS_RUNAWAY_SPEED_MIN   80.0f
#define CHASSIS_RUNAWAY_SPEED_RATIO 4.0f
#define CHASSIS_SIGN_FAULT_SPEED    40.0f
#define CHASSIS_SIGN_FAULT_LIMIT    5U
#define CHASSIS_BRAKE_SPEED_RISE    5.0f

/* Speed ramp step: max rpm change per 10ms loop (= 500 rpm/s).
 * Smaller = smoother/slower, larger = snappier. Tune to taste. */
#define CHASSIS_RAMP_STEP_PER_10MS  5.0f

typedef struct
{
    float motor_speed[2];
    float motor_target_speed[2];
    uint32_t encoder_count[2];
    float motor_out[2];
} chassis_data_t;

static imu_data_t g_imu_data =
{
    .pitch = 0, .roll = 0, .yaw = 0,
    .aacx = 0, .aacy = 0, .aacz = 0,
    .gyrox = 0, .gyroy = 0, .gyroz = 0,
    .temp = 0,
};

chassis_data_t chasis_loop =
{
    .motor_speed[0] = 0,
    .motor_speed[1] = 0,
    .motor_target_speed[0] = 0,
    .motor_target_speed[1] = 0,
    .encoder_count[0] = 0,
    .encoder_count[1] = 0,
    .motor_out[0] = 0,
    .motor_out[1] = 0,
};

static uint8_t g_open_loop_enabled = 0;
static int16_t g_open_loop_pwm[2] = {0, 0};
static uint32_t g_open_loop_tick = 0;
static uint8_t g_speed_fault_count[2] = {0, 0};
static float g_last_abs_speed[2] = {0.0f, 0.0f};
static SpeedRamp_t g_speed_ramp[2];

static void up_data_speed(void)
{
    int16_t etr[2];
    static float current_spd[2];

    get_encoder(etr);
    chasis_loop.encoder_count[0] += (int32_t)etr[0];
    chasis_loop.encoder_count[1] += (int32_t)etr[1];

    for (int i = 0; i < 2; i++)
    {
        float etrtemp;

        etrtemp = ((float)etr[i]) / (CHASSIS_ENCODER_LINES * CHASSIS_REDUCTION_RATIO);
        etrtemp = etrtemp * CHASSIS_SPEED_SCALE_RPM;
        current_spd[i] = 0.7f * current_spd[i] + 0.3f * etrtemp;
        chasis_loop.motor_speed[i] = current_spd[i];
    }
}

static float chassis_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float chassis_signf(float value)
{
    return (value >= 0.0f) ? 1.0f : -1.0f;
}

static void chassis_clear_speed_pid(uint8_t index)
{
    pid_spd[index].err[LLAST] = 0.0f;
    pid_spd[index].err[LAST] = 0.0f;
    pid_spd[index].err[NOW] = 0.0f;
    pid_spd[index].i_band = 0.0f;
    pid_spd[index].last_out = 0.0f;
    pid_spd[index].out = 0;
}

static float chassis_limit_pwm(float output, float max_output)
{
    if (output > max_output)
    {
        return max_output;
    }

    if (output < -max_output)
    {
        return -max_output;
    }

    return output;
}

static float chassis_calc_feedforward_pwm(float target_speed)
{
    float abs_target = chassis_absf(target_speed);

    if (abs_target < CHASSIS_TARGET_STOP_BAND)
    {
        return 0.0f;
    }

    return chassis_signf(target_speed) *
           (CHASSIS_FEEDFORWARD_OFFSET + CHASSIS_FEEDFORWARD_GAIN * abs_target);
}

static void chassis_clear_speed_faults(void)
{
    g_speed_fault_count[0] = 0;
    g_speed_fault_count[1] = 0;
    g_last_abs_speed[0] = 0.0f;
    g_last_abs_speed[1] = 0.0f;
}

static uint8_t chassis_check_speed_fault(float target_speed, float measured_speed,
                                         float output_pwm, uint8_t index)
{
    float abs_target = chassis_absf(target_speed);
    float abs_measured = chassis_absf(measured_speed);
    float runaway_limit = abs_target * CHASSIS_RUNAWAY_SPEED_RATIO;
    uint8_t opposite_target_speed;
    uint8_t output_pushes_measured;
    uint8_t brake_not_working;

    if (runaway_limit < CHASSIS_RUNAWAY_SPEED_MIN)
    {
        runaway_limit = CHASSIS_RUNAWAY_SPEED_MIN;
    }

    output_pushes_measured = ((output_pwm * measured_speed) > 0.0f) ? 1U : 0U;

    if ((abs_target >= CHASSIS_TARGET_STOP_BAND) &&
        (abs_measured > runaway_limit) &&
        output_pushes_measured)
    {
        return 1;
    }

    opposite_target_speed = ((abs_target >= CHASSIS_TARGET_STOP_BAND) &&
                             (abs_measured > CHASSIS_SIGN_FAULT_SPEED) &&
                             ((target_speed * measured_speed) < 0.0f)) ? 1U : 0U;
    brake_not_working = ((output_pwm * measured_speed) < 0.0f) &&
                        (abs_measured > (g_last_abs_speed[index] + CHASSIS_BRAKE_SPEED_RISE));

    if (opposite_target_speed && (output_pushes_measured || brake_not_working))
    {
        if (g_speed_fault_count[index] < CHASSIS_SIGN_FAULT_LIMIT)
        {
            g_speed_fault_count[index]++;
        }

        if (g_speed_fault_count[index] >= CHASSIS_SIGN_FAULT_LIMIT)
        {
            return 1;
        }
    }
    else
    {
        g_speed_fault_count[index] = 0;
    }

    g_last_abs_speed[index] = abs_measured;
    return 0;
}

static uint8_t chassis_is_direction_reversal(float old_target, float new_target)
{
    if ((chassis_absf(old_target) < CHASSIS_TARGET_STOP_BAND) ||
        (chassis_absf(new_target) < CHASSIS_TARGET_STOP_BAND))
    {
        return 0;
    }

    return ((old_target * new_target) < 0.0f) ? 1U : 0U;
}

static void chassis_prepare_target_change(uint8_t index, float new_target)
{
    if (chassis_is_direction_reversal(g_speed_ramp[index].target, new_target) ||
        chassis_is_direction_reversal(g_speed_ramp[index].current, new_target))
    {
        chassis_clear_speed_pid(index);
        g_speed_fault_count[index] = 0;
        g_last_abs_speed[index] = chassis_absf(chasis_loop.motor_speed[index]);
    }
}

static float chassis_apply_speed_output(float pid_output, float target_speed, uint8_t index)
{
    float limited_output;

    if (chassis_absf(target_speed) < CHASSIS_TARGET_STOP_BAND)
    {
        chassis_clear_speed_pid(index);
        return 0.0f;
    }

    pid_output += chassis_calc_feedforward_pwm(target_speed);
    limited_output = chassis_limit_pwm(pid_output, pid_spd[index].max_out);

    if (chassis_absf(target_speed) < CHASSIS_LOW_TARGET_BAND)
    {
        limited_output = chassis_limit_pwm(limited_output, CHASSIS_LOW_TARGET_PWM_MAX);
    }

    return limited_output;
}

void chassis_init(void)
{
    pid_spd[0].max_out = 300;
    pid_spd[1].max_out = 300;
    pid_spd[0].max_step = 20;
    pid_spd[1].max_step = 20;

    pid_reset(&pid_spd[0], 6.0f, 1.0f, 0.0f);
    pid_reset(&pid_spd[1], 6.0f, 1.0f, 0.0f);
    pid_reset(&pid_turn, 4.0f, 0.0f, 0.1f);
    pid_reset(&pid_move, 2.0f, 0.0f, 2.0f);

    imu_reset_state();
    SpeedRamp_Init(&g_speed_ramp[0], CHASSIS_RAMP_STEP_PER_10MS);
    SpeedRamp_Init(&g_speed_ramp[1], CHASSIS_RAMP_STEP_PER_10MS);
    chassis_stop();
}

void chassis_update_speed_loop_10ms(void)
{
    static float angle_out = 0;
    float pid_out[2];

    up_data_speed();

    if (g_open_loop_enabled)
    {
        if ((uint32_t)(HAL_GetTick() - g_open_loop_tick) > CHASSIS_OPEN_LOOP_TIMEOUT_MS)
        {
            chassis_stop();
            return;
        }

        chasis_loop.motor_target_speed[0] = 0.0f;
        chasis_loop.motor_target_speed[1] = 0.0f;
        chasis_loop.motor_out[0] = (float)g_open_loop_pwm[0];
        chasis_loop.motor_out[1] = (float)g_open_loop_pwm[1];
        motor_set_pwm(g_open_loop_pwm[0], g_open_loop_pwm[1]);
        return;
    }

    /* Apply speed ramp: user target moves toward its final value
     * at CHASSIS_RAMP_STEP_PER_10MS per loop, so the PID never sees
     * a step change. Pairs with integral separation to eliminate
     * direction-reversal wind-up. */
    chasis_loop.motor_target_speed[0] = SpeedRamp_Update(&g_speed_ramp[0]);
    chasis_loop.motor_target_speed[1] = SpeedRamp_Update(&g_speed_ramp[1]);

    pid_out[0] = incremental_pid(&pid_spd[0], chasis_loop.motor_target_speed[0],
                                 chasis_loop.motor_speed[0] + angle_out);
    pid_out[1] = incremental_pid(&pid_spd[1], chasis_loop.motor_target_speed[1],
                                 chasis_loop.motor_speed[1] - angle_out);

    chasis_loop.motor_out[0] = chassis_apply_speed_output(pid_out[0],
                                                        chasis_loop.motor_target_speed[0],
                                                        0);
    chasis_loop.motor_out[1] = chassis_apply_speed_output(pid_out[1],
                                                        chasis_loop.motor_target_speed[1],
                                                        1);

    if (chassis_check_speed_fault(chasis_loop.motor_target_speed[0],
                                  chasis_loop.motor_speed[0] + angle_out,
                                  chasis_loop.motor_out[0],
                                  0) ||
        chassis_check_speed_fault(chasis_loop.motor_target_speed[1],
                                  chasis_loop.motor_speed[1] - angle_out,
                                  chasis_loop.motor_out[1],
                                   1))
    {
        chassis_stop();
        return;
    }

    motor_set_pwm((int16_t)chasis_loop.motor_out[0], (int16_t)chasis_loop.motor_out[1]);
}

void chassis_set_target_speed(float left, float right)
{
    g_open_loop_enabled = 0;
    g_open_loop_pwm[0] = 0;
    g_open_loop_pwm[1] = 0;
    chassis_prepare_target_change(0, left);
    chassis_prepare_target_change(1, right);
    /* The 10ms loop reads the ramped value into motor_target_speed[]. */
    SpeedRamp_SetTarget(&g_speed_ramp[0], left);
    SpeedRamp_SetTarget(&g_speed_ramp[1], right);
}

void chassis_set_open_loop_pwm(int16_t left_pwm, int16_t right_pwm)
{
    g_open_loop_enabled = 1;
    g_open_loop_tick = HAL_GetTick();
    g_open_loop_pwm[0] = left_pwm;
    g_open_loop_pwm[1] = right_pwm;
    chasis_loop.motor_target_speed[0] = 0.0f;
    chasis_loop.motor_target_speed[1] = 0.0f;
    chasis_loop.motor_out[0] = (float)left_pwm;
    chasis_loop.motor_out[1] = (float)right_pwm;
    SpeedRamp_Reset(&g_speed_ramp[0], 0.0f);
    SpeedRamp_Reset(&g_speed_ramp[1], 0.0f);
    chassis_clear_speed_faults();
    chassis_clear_speed_pid(0);
    chassis_clear_speed_pid(1);
    motor_set_pwm(left_pwm, right_pwm);
}

void chassis_disable_open_loop(void)
{
    g_open_loop_enabled = 0;
    g_open_loop_pwm[0] = 0;
    g_open_loop_pwm[1] = 0;
    chassis_clear_speed_faults();
}

uint8_t chassis_is_open_loop_enabled(void)
{
    return g_open_loop_enabled;
}

int32_t chassis_get_left_encoder_count(void)
{
    return (int32_t)chasis_loop.encoder_count[0];
}

int32_t chassis_get_right_encoder_count(void)
{
    return (int32_t)chasis_loop.encoder_count[1];
}

void chassis_get_encoder_counts(int32_t *left, int32_t *right)
{
    uint32_t primask;

    if ((left == 0) || (right == 0))
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *left = (int32_t)chasis_loop.encoder_count[0];
    *right = (int32_t)chasis_loop.encoder_count[1];
    __set_PRIMASK(primask);
}

double chassis_get_position(void)
{
    return 0.5 * ((double)chassis_get_left_encoder_count() +
                  (double)chassis_get_right_encoder_count());
}

float chassis_get_left_speed(void)
{
    return chasis_loop.motor_speed[0];
}

float chassis_get_right_speed(void)
{
    return chasis_loop.motor_speed[1];
}

float chassis_get_left_target_speed(void)
{
    return chasis_loop.motor_target_speed[0];
}

float chassis_get_right_target_speed(void)
{
    return chasis_loop.motor_target_speed[1];
}

float chassis_get_left_output_pwm(void)
{
    return chasis_loop.motor_out[0];
}

float chassis_get_right_output_pwm(void)
{
    return chasis_loop.motor_out[1];
}

float chassis_get_yaw(void)
{
    return g_imu_data.yaw;
}

uint8_t chassis_get_imu_data(imu_data_t *imu_data)
{
    if (imu_data == 0)
    {
        return 0;
    }

    *imu_data = g_imu_data;
    return chassis_is_imu_ready();
}

uint8_t chassis_init_imu(void)
{
    return imu_init();
}

uint8_t chassis_is_imu_ready(void)
{
    return imu_is_ready();
}

uint8_t chassis_get_imu_error(void)
{
    return imu_get_last_error();
}

uint8_t chassis_get_imu_fail_count(void)
{
    return imu_get_fail_count();
}

uint8_t chassis_update_imu(void)
{
    return imu_data_update(&g_imu_data);
}

void chassis_stop(void)
{
    g_open_loop_enabled = 0;
    g_open_loop_pwm[0] = 0;
    g_open_loop_pwm[1] = 0;
    chasis_loop.motor_target_speed[0] = 0.0f;
    chasis_loop.motor_target_speed[1] = 0.0f;
    chasis_loop.motor_out[0] = 0.0f;
    chasis_loop.motor_out[1] = 0.0f;
    SpeedRamp_Reset(&g_speed_ramp[0], 0.0f);
    SpeedRamp_Reset(&g_speed_ramp[1], 0.0f);
    chassis_clear_speed_faults();
    chassis_clear_speed_pid(0);
    chassis_clear_speed_pid(1);
    motor_set_pwm(0, 0);
}

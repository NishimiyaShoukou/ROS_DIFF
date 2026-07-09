/******************************************************************************
 * @file    pid_debug.c
 * @brief   Serial PID tuning helper for chassis debug.
 * @author  User
 * @date    2026-07-08
 * @version V1.0.0
 * @note    Part of the STM32F1 robot control project.
 ******************************************************************************/
#include "pid_debug.h"
#include "comHandle.h"
#include "contrl.h"
#include "pid.h"

#define PID_DEBUG_LINE_MAX  80U
#define PID_DEBUG_TX_MAX    160U
#define PID_DEBUG_QUEUE_DEPTH 4U
#define PID_DEBUG_TELEMETRY_PERIOD_MS 20U
#define PID_DEBUG_SCALE_PID 1000.0f
#define PID_DEBUG_SCALE_SPD 100.0f
#define PID_DEBUG_IMU_PERIOD_MS 100U
#define PID_DEBUG_PWM_MAX 1000

static volatile uint8_t g_pid_debug_enabled = 0;
static volatile uint8_t g_pid_debug_imu_enabled = 0;
static char g_pid_debug_line[PID_DEBUG_LINE_MAX];
static char g_pid_debug_pending_lines[PID_DEBUG_QUEUE_DEPTH][PID_DEBUG_LINE_MAX];
static uint8_t g_pid_debug_index = 0;
static volatile uint8_t g_pid_debug_queue_head = 0;
static volatile uint8_t g_pid_debug_queue_tail = 0;

static uint8_t pid_debug_starts_with(const char *str, const char *prefix)
{
    while (*prefix != '\0')
    {
        if (*str != *prefix)
        {
            return 0;
        }
        str++;
        prefix++;
    }

    return 1;
}

static uint8_t pid_debug_parse_int(const char **str, int32_t *out)
{
    int32_t sign = 1;
    int32_t value = 0;
    uint8_t has_digit = 0;

    while ((**str == ' ') || (**str == ','))
    {
        (*str)++;
    }

    if (**str == '-')
    {
        sign = -1;
        (*str)++;
    }
    else if (**str == '+')
    {
        (*str)++;
    }

    while ((**str >= '0') && (**str <= '9'))
    {
        value = value * 10 + (**str - '0');
        (*str)++;
        has_digit = 1;
    }

    if (!has_digit)
    {
        return 0;
    }

    *out = value * sign;
    return 1;
}

static void pid_debug_append_char(char *buf, uint8_t *idx, char ch)
{
    if (*idx < (PID_DEBUG_TX_MAX - 1U))
    {
        buf[*idx] = ch;
        (*idx)++;
        buf[*idx] = '\0';
    }
}

static void pid_debug_append_str(char *buf, uint8_t *idx, const char *str)
{
    while (*str != '\0')
    {
        pid_debug_append_char(buf, idx, *str);
        str++;
    }
}

static void pid_debug_append_int(char *buf, uint8_t *idx, int32_t value)
{
    char temp[12];
    uint8_t temp_idx = 0;
    uint32_t abs_value;

    if (value < 0)
    {
        pid_debug_append_char(buf, idx, '-');
        abs_value = (uint32_t)(-value);
    }
    else
    {
        abs_value = (uint32_t)value;
    }

    do
    {
        temp[temp_idx++] = (char)('0' + (abs_value % 10U));
        abs_value /= 10U;
    } while ((abs_value > 0U) && (temp_idx < sizeof(temp)));

    while (temp_idx > 0U)
    {
        pid_debug_append_char(buf, idx, temp[--temp_idx]);
    }
}

static int32_t pid_debug_scale_float(float value, float scale)
{
    if (value >= 0.0f)
    {
        return (int32_t)(value * scale + 0.5f);
    }

    return (int32_t)(value * scale - 0.5f);
}

static void pid_debug_send_line(const char *line)
{
    uint8_t len = 0;

    while ((line[len] != '\0') && (len < 250U))
    {
        len++;
    }

    usart_tx((uint8_t *)line, len);
}

static void pid_debug_send_pair_ack(const char *command,
                                    int32_t left, int32_t right)
{
    char tx[PID_DEBUG_TX_MAX];
    uint8_t idx = 0;

    pid_debug_append_str(tx, &idx, "$ACK,");
    pid_debug_append_str(tx, &idx, command);
    pid_debug_append_char(tx, &idx, ',');
    pid_debug_append_int(tx, &idx, left);
    pid_debug_append_char(tx, &idx, ',');
    pid_debug_append_int(tx, &idx, right);
    pid_debug_append_char(tx, &idx, '\n');
    usart_tx((uint8_t *)tx, idx);
}

static void pid_debug_copy_line(char *dst, const char *src)
{
    uint8_t i = 0;

    while ((src[i] != '\0') && (i < (PID_DEBUG_LINE_MAX - 1U)))
    {
        dst[i] = src[i];
        i++;
    }
    dst[i] = '\0';
}

static void pid_debug_clear_pid_state(pid_t *pid)
{
    pid->err[LLAST] = 0.0f;
    pid->err[LAST] = 0.0f;
    pid->err[NOW] = 0.0f;
    pid->i_band = 0.0f;
    pid->last_out = 0.0f;
    pid->out = 0;
}

static void pid_debug_apply_pid(pid_t *pid, int32_t kp, int32_t ki, int32_t kd,
                                int32_t max_out, int32_t max_step)
{
    pid_reset(pid, (float)kp / PID_DEBUG_SCALE_PID,
                   (float)ki / PID_DEBUG_SCALE_PID,
                   (float)kd / PID_DEBUG_SCALE_PID);
    pid->max_out = (float)max_out;
    pid->max_step = (float)max_step;
    pid_debug_clear_pid_state(pid);
}

static int16_t pid_debug_limit_pwm(int32_t pwm)
{
    if (pwm > PID_DEBUG_PWM_MAX)
    {
        return PID_DEBUG_PWM_MAX;
    }

    if (pwm < -PID_DEBUG_PWM_MAX)
    {
        return -PID_DEBUG_PWM_MAX;
    }

    return (int16_t)pwm;
}

static void pid_debug_handle_pid_command(const char *line)
{
    const char *p = line + 4;
    int32_t index;
    int32_t kp;
    int32_t ki;
    int32_t kd;
    int32_t max_out;
    int32_t max_step;

    if (!pid_debug_parse_int(&p, &index) ||
        !pid_debug_parse_int(&p, &kp) ||
        !pid_debug_parse_int(&p, &ki) ||
        !pid_debug_parse_int(&p, &kd) ||
        !pid_debug_parse_int(&p, &max_out) ||
        !pid_debug_parse_int(&p, &max_step))
    {
        pid_debug_send_line("$ACK,PID,ERR\n");
        return;
    }

    if ((index == 0) || (index == 2))
    {
        pid_debug_apply_pid(&pid_spd[0], kp, ki, kd, max_out, max_step);
    }

    if ((index == 1) || (index == 2))
    {
        pid_debug_apply_pid(&pid_spd[1], kp, ki, kd, max_out, max_step);
    }

    g_pid_debug_enabled = 1;
    pid_debug_send_line("$ACK,PID,OK\n");
}

static void pid_debug_handle_speed_command(const char *line)
{
    const char *p = line + 4;
    int32_t left;
    int32_t right;

    if (!pid_debug_parse_int(&p, &left) ||
        !pid_debug_parse_int(&p, &right))
    {
        pid_debug_send_line("$ACK,SPD,ERR\n");
        return;
    }

    g_pid_debug_enabled = 1;
    chassis_set_target_speed((float)left / PID_DEBUG_SCALE_SPD,
                             (float)right / PID_DEBUG_SCALE_SPD);
    pid_debug_send_pair_ack("SPD", left, right);
}

static void pid_debug_handle_motor_command(const char *line)
{
    const char *p = line + 4;
    int32_t left;
    int32_t right;

    if (!pid_debug_parse_int(&p, &left) ||
        !pid_debug_parse_int(&p, &right))
    {
        pid_debug_send_line("$ACK,MOT,ERR\n");
        return;
    }

    g_pid_debug_enabled = 1;
    chassis_set_open_loop_pwm(pid_debug_limit_pwm(left),
                              pid_debug_limit_pwm(right));
    pid_debug_send_pair_ack("MOT",
                            pid_debug_limit_pwm(left),
                            pid_debug_limit_pwm(right));
}

static void pid_debug_handle_debug_command(const char *line)
{
    const char *p = line + 4;
    int32_t enable;

    if (!pid_debug_parse_int(&p, &enable))
    {
        pid_debug_send_line("$ACK,DBG,ERR\n");
        return;
    }

    g_pid_debug_enabled = (enable != 0) ? 1U : 0U;
    if (!g_pid_debug_enabled)
    {
        g_pid_debug_imu_enabled = 0U;
        chassis_stop();
    }
    pid_debug_send_line("$ACK,DBG,OK\n");
}

static void pid_debug_handle_imu_command(const char *line)
{
    const char *p = line + 4;
    int32_t enable;

    if (!pid_debug_parse_int(&p, &enable))
    {
        pid_debug_send_line("$ACK,IMU,ERR\n");
        return;
    }

    g_pid_debug_imu_enabled = (enable != 0) ? 1U : 0U;
    if (g_pid_debug_imu_enabled)
    {
        g_pid_debug_enabled = 1U;
    }
    pid_debug_send_line("$ACK,IMU,OK\n");
}

static void pid_debug_handle_line(const char *line)
{
    if (line[0] != '$')
    {
        return;
    }

    if (pid_debug_starts_with(line, "$DBG"))
    {
        pid_debug_handle_debug_command(line);
    }
    else if (pid_debug_starts_with(line, "$IMU"))
    {
        pid_debug_handle_imu_command(line);
    }
    else if (pid_debug_starts_with(line, "$PID"))
    {
        pid_debug_handle_pid_command(line);
    }
    else if (pid_debug_starts_with(line, "$SPD"))
    {
        pid_debug_handle_speed_command(line);
    }
    else if (pid_debug_starts_with(line, "$MOT"))
    {
        pid_debug_handle_motor_command(line);
    }
    else if (pid_debug_starts_with(line, "$STOP"))
    {
        chassis_stop();
        g_pid_debug_enabled = 1;
        pid_debug_send_line("$ACK,STOP,OK\n");
    }
    else if (pid_debug_starts_with(line, "$GET"))
    {
        pid_debug_send_line("$ACK,GET,OK\n");
        pid_debug_send_telemetry_20ms();
    }
}

void pid_debug_rx_byte(uint8_t data)
{
    if (data == '\r')
    {
        return;
    }

    if (data == '\n')
    {
        uint8_t next_head;

        g_pid_debug_line[g_pid_debug_index] = '\0';
        next_head = (uint8_t)((g_pid_debug_queue_head + 1U) %
                              PID_DEBUG_QUEUE_DEPTH);
        if (next_head != g_pid_debug_queue_tail)
        {
            pid_debug_copy_line(
                g_pid_debug_pending_lines[g_pid_debug_queue_head],
                g_pid_debug_line);
            g_pid_debug_queue_head = next_head;
        }
        g_pid_debug_index = 0;
        return;
    }

    if ((data < 0x20U) || (data > 0x7EU))
    {
        return;
    }

    if (g_pid_debug_index < (PID_DEBUG_LINE_MAX - 1U))
    {
        g_pid_debug_line[g_pid_debug_index++] = (char)data;
    }
    else
    {
        g_pid_debug_index = 0;
    }
}

void pid_debug_update(void)
{
    char line[PID_DEBUG_LINE_MAX];
    uint32_t primask;
    uint32_t now;
    static uint32_t last_telemetry_tick = 0U;
    uint8_t has_line = 0U;

    primask = __get_PRIMASK();
    __disable_irq();
    if (g_pid_debug_queue_tail != g_pid_debug_queue_head)
    {
        pid_debug_copy_line(
            line,
            g_pid_debug_pending_lines[g_pid_debug_queue_tail]);
        g_pid_debug_queue_tail =
            (uint8_t)((g_pid_debug_queue_tail + 1U) %
                      PID_DEBUG_QUEUE_DEPTH);
        has_line = 1U;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }

    if (has_line)
    {
        pid_debug_handle_line(line);
    }

    now = HAL_GetTick();
    if (g_pid_debug_enabled &&
        ((uint32_t)(now - last_telemetry_tick) >=
         PID_DEBUG_TELEMETRY_PERIOD_MS))
    {
        last_telemetry_tick = now;
        pid_debug_send_telemetry_20ms();
    }
}

uint8_t pid_debug_is_enabled(void)
{
    return g_pid_debug_enabled;
}

static void pid_debug_send_imu_telemetry(void)
{
    char tx[PID_DEBUG_TX_MAX];
    uint8_t idx = 0;
    imu_data_t imu_data;
    uint8_t imu_ready;

    imu_ready = chassis_get_imu_data(&imu_data);

    pid_debug_append_str(tx, &idx, "$IMU,");
    pid_debug_append_int(tx, &idx, (int32_t)HAL_GetTick());
    pid_debug_append_char(tx, &idx, ',');
    pid_debug_append_int(tx, &idx, imu_ready);
    pid_debug_append_char(tx, &idx, ',');
    pid_debug_append_int(tx, &idx, chassis_get_imu_error());
    pid_debug_append_char(tx, &idx, ',');
    pid_debug_append_int(tx, &idx, chassis_get_imu_fail_count());
    pid_debug_append_char(tx, &idx, ',');
    pid_debug_append_int(tx, &idx, pid_debug_scale_float(imu_data.pitch, 100.0f));
    pid_debug_append_char(tx, &idx, ',');
    pid_debug_append_int(tx, &idx, pid_debug_scale_float(imu_data.roll, 100.0f));
    pid_debug_append_char(tx, &idx, ',');
    pid_debug_append_int(tx, &idx, pid_debug_scale_float(imu_data.yaw, 100.0f));
    pid_debug_append_char(tx, &idx, ',');
    pid_debug_append_int(tx, &idx, imu_data.aacx);
    pid_debug_append_char(tx, &idx, ',');
    pid_debug_append_int(tx, &idx, imu_data.aacy);
    pid_debug_append_char(tx, &idx, ',');
    pid_debug_append_int(tx, &idx, imu_data.aacz);
    pid_debug_append_char(tx, &idx, ',');
    pid_debug_append_int(tx, &idx, imu_data.gyrox);
    pid_debug_append_char(tx, &idx, ',');
    pid_debug_append_int(tx, &idx, imu_data.gyroy);
    pid_debug_append_char(tx, &idx, ',');
    pid_debug_append_int(tx, &idx, imu_data.gyroz);
    pid_debug_append_char(tx, &idx, ',');
    pid_debug_append_int(tx, &idx, imu_data.temp);
    pid_debug_append_char(tx, &idx, '\n');

    usart_tx((uint8_t *)tx, idx);
}

void pid_debug_send_telemetry_20ms(void)
{
    char tx[PID_DEBUG_TX_MAX];
    uint8_t idx = 0;
    static uint32_t last_imu_tick = 0;
    uint32_t now;

    if (!g_pid_debug_enabled)
    {
        return;
    }

    now = HAL_GetTick();

    pid_debug_append_str(tx, &idx, "$TEL,");
    pid_debug_append_int(tx, &idx, (int32_t)now);
    pid_debug_append_char(tx, &idx, ',');
    pid_debug_append_int(tx, &idx, pid_debug_scale_float(chassis_get_left_target_speed(), PID_DEBUG_SCALE_SPD));
    pid_debug_append_char(tx, &idx, ',');
    pid_debug_append_int(tx, &idx, pid_debug_scale_float(chassis_get_right_target_speed(), PID_DEBUG_SCALE_SPD));
    pid_debug_append_char(tx, &idx, ',');
    pid_debug_append_int(tx, &idx, pid_debug_scale_float(chassis_get_left_speed(), PID_DEBUG_SCALE_SPD));
    pid_debug_append_char(tx, &idx, ',');
    pid_debug_append_int(tx, &idx, pid_debug_scale_float(chassis_get_right_speed(), PID_DEBUG_SCALE_SPD));
    pid_debug_append_char(tx, &idx, ',');
    pid_debug_append_int(tx, &idx, (int32_t)chassis_get_left_output_pwm());
    pid_debug_append_char(tx, &idx, ',');
    pid_debug_append_int(tx, &idx, (int32_t)chassis_get_right_output_pwm());
    pid_debug_append_char(tx, &idx, ',');
    pid_debug_append_int(tx, &idx, pid_debug_scale_float(pid_spd[0].kp, PID_DEBUG_SCALE_PID));
    pid_debug_append_char(tx, &idx, ',');
    pid_debug_append_int(tx, &idx, pid_debug_scale_float(pid_spd[0].ki, PID_DEBUG_SCALE_PID));
    pid_debug_append_char(tx, &idx, ',');
    pid_debug_append_int(tx, &idx, pid_debug_scale_float(pid_spd[0].kd, PID_DEBUG_SCALE_PID));
    pid_debug_append_char(tx, &idx, ',');
    pid_debug_append_int(tx, &idx, pid_debug_scale_float(pid_spd[1].kp, PID_DEBUG_SCALE_PID));
    pid_debug_append_char(tx, &idx, ',');
    pid_debug_append_int(tx, &idx, pid_debug_scale_float(pid_spd[1].ki, PID_DEBUG_SCALE_PID));
    pid_debug_append_char(tx, &idx, ',');
    pid_debug_append_int(tx, &idx, pid_debug_scale_float(pid_spd[1].kd, PID_DEBUG_SCALE_PID));
    pid_debug_append_char(tx, &idx, '\n');

    usart_tx((uint8_t *)tx, idx);

    if (g_pid_debug_imu_enabled &&
        ((uint32_t)(now - last_imu_tick) >= PID_DEBUG_IMU_PERIOD_MS))
    {
        last_imu_tick = now;
        pid_debug_send_imu_telemetry();
    }
}

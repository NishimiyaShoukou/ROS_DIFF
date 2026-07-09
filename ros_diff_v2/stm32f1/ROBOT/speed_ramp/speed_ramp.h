/******************************************************************************
 * @file    speed_ramp.h
 * @brief   Speed ramp / slew-rate limiter for smooth target transitions.
 * @author  User
 * @date    2026-07-08
 * @version V1.0.0
 * @note    Call SpeedRamp_Update() at a fixed rate (e.g. 10 ms in this
 *          project). `step` is interpreted as "max change per Update() call",
 *          i.e. rpm per 10 ms in this project. Always treated as positive.
 ******************************************************************************/

#ifndef SPEED_RAMP_H
#define SPEED_RAMP_H

typedef struct
{
    float current;   /* Ramp output, what the caller should use as the
                        effective target this tick. */
    float target;    /* User-requested value the ramp is moving toward. */
    float step;      /* Max change per Update() call. Always >= 0. */
} SpeedRamp_t;

/* Initialize ramp at zero with the given step magnitude (negative input is
 * auto-absolute, so a wrong sign does not silently disable the ramp). */
void SpeedRamp_Init(SpeedRamp_t *ramp, float step);

/* Set the new target. The ramp moves toward it on subsequent Update() calls. */
void SpeedRamp_SetTarget(SpeedRamp_t *ramp, float target);

/* Advance the ramp one tick toward target. Returns the current (ramped)
 * value, which is also stored in ramp->current. */
float SpeedRamp_Update(SpeedRamp_t *ramp);

/* Snap both current and target to the same value (e.g. on emergency stop
 * or when entering open-loop PWM mode). */
void SpeedRamp_Reset(SpeedRamp_t *ramp, float value);

#endif /* SPEED_RAMP_H */

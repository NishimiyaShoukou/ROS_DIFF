/******************************************************************************
 * @file    speed_ramp.c
 * @brief   Speed ramp / slew-rate limiter implementation.
 * @author  User
 * @date    2026-07-08
 * @version V1.0.0
 ******************************************************************************/

#include "speed_ramp.h"
#include <math.h>

void SpeedRamp_Init(SpeedRamp_t *ramp, float step)
{
    ramp->current = 0.0f;
    ramp->target  = 0.0f;
    ramp->step    = fabsf(step);
}

void SpeedRamp_SetTarget(SpeedRamp_t *ramp, float target)
{
    ramp->target = target;
}

float SpeedRamp_Update(SpeedRamp_t *ramp)
{
    if (ramp->current < ramp->target)
    {
        ramp->current += ramp->step;
        if (ramp->current > ramp->target)
        {
            ramp->current = ramp->target;
        }
    }
    else if (ramp->current > ramp->target)
    {
        ramp->current -= ramp->step;
        if (ramp->current < ramp->target)
        {
            ramp->current = ramp->target;
        }
    }

    return ramp->current;
}

void SpeedRamp_Reset(SpeedRamp_t *ramp, float value)
{
    ramp->current = value;
    ramp->target  = value;
}

/******************************************************************************
 * @file    move.h
 * @brief   Standalone chassis distance-test interface.
 ******************************************************************************/

#ifndef __MOVE_H
#define __MOVE_H

#include "stm32f1xx.h"

/*
 * Set to 1 only for a clear-floor, STM32-only bench test build.
 * Do not run base_controller during this test: valid ROS command frames
 * intentionally take control and abort AUTO mode.
 */
#define MOVE_AUTO_START_1M_TEST  0U

typedef enum
{
    MOVE_TEST_IDLE = 0,
    MOVE_TEST_RUNNING,
    MOVE_TEST_DONE,
    MOVE_TEST_ABORTED,
    MOVE_TEST_TIMEOUT
} move_test_state_t;

void walk_run(void);
void walk_run_interrupt(void);
void goto_1m(void);
uint8_t move_start_distance_test(float distance_m);
void move_cancel_test(void);
move_test_state_t move_get_test_state(void);

#endif

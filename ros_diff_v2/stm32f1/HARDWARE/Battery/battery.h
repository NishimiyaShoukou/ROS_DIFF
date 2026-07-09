/******************************************************************************
 * @file    battery.h
 * @brief   Battery voltage measurement via ADC voltage divider interface.
 * @author  User
 * @date    2026-07-08
 * @version V1.0.0
 * @note    Part of the STM32F1 robot control project.
 *
 *  Hardware: 47kΩ + 10kΩ voltage divider (ratio = 5.7)
 *  2S LiPo:  6.0V - 8.4V  → ADC input: 1.05V - 1.47V
 *  3S LiPo:  9.0V - 12.6V → ADC input: 1.58V - 2.21V
 *  Never connect the battery directly to PB1. The divider is mandatory.
 ******************************************************************************/
#ifndef __BATTERY_H__
#define __BATTERY_H__

#include "stm32f1xx.h"

/*==============================================================================
 * Configuration - Must match your hardware
 *============================================================================*/

/* Voltage divider resistors (Ω) */
#define BATTERY_R1_OHM         47000.0f
#define BATTERY_R2_OHM         10000.0f

/* Number of cells: 2 = 2S LiPo, 3 = 3S LiPo */
#ifndef BATTERY_CELL_COUNT
#define BATTERY_CELL_COUNT     2
#endif

/* ADC reference voltage (mV) */
#define BATTERY_VREF_MV        3300.0f

/* Adjust after comparing telemetry with a trusted multimeter. */
#ifndef BATTERY_CALIBRATION_SCALE
#define BATTERY_CALIBRATION_SCALE  1.0f
#endif

/* ADC resolution (12-bit) */
#define BATTERY_ADC_MAX        4095U

/* Number of samples for averaging filter */
#define BATTERY_SAMPLES        8U

/*==============================================================================
 * Battery Thresholds (mV) - 2S/3S LiPo typical values
 *============================================================================*/
#if (BATTERY_CELL_COUNT == 2)

    #define BATTERY_MIN_MV         6000U       /* 2S empty: 3.0V/cell */
    #define BATTERY_MAX_MV         8400U       /* 2S full: 4.2V/cell */
    #define BATTERY_LOW_MV         6800U       /* 2S low:  3.40V/cell */
    #define BATTERY_CRITICAL_MV    6400U       /* 2S critical: 3.20V/cell */
    #define BATTERY_OVERVOLTAGE_MV 8600U       /* 2S: 4.30V/cell */

#elif (BATTERY_CELL_COUNT == 3)

    #define BATTERY_MIN_MV         9000U       /* 3S empty: 3.0V/cell */
    #define BATTERY_MAX_MV         12600U      /* 3S full: 4.2V/cell */
    #define BATTERY_LOW_MV         10200U      /* 3S low:  3.40V/cell */
    #define BATTERY_CRITICAL_MV    9600U       /* 3S critical: 3.20V/cell */
    #define BATTERY_OVERVOLTAGE_MV 12900U      /* 3S: 4.30V/cell */

#else
    #error "BATTERY_CELL_COUNT must be 2 or 3"
#endif

#define BATTERY_DIVIDER_RATIO      ((BATTERY_R1_OHM + BATTERY_R2_OHM) / BATTERY_R2_OHM)
#define BATTERY_PRESENT_MIN_MV     1000U

/*==============================================================================
 * Status codes
 *============================================================================*/
typedef enum {
    BATTERY_OK = 0,
    BATTERY_ERR_NULL_PTR,       /* NULL pointer passed */
    BATTERY_ERR_OVERVOLTAGE,
    BATTERY_ERR_ADC_TIMEOUT,    /* ADC conversion timeout */
    BATTERY_ERR_NOT_INIT,       /* Module not initialized */
    BATTERY_ERR_CALIBRATION
} battery_status_t;

/*==============================================================================
 * Battery state
 *============================================================================*/
typedef enum {
    BATTERY_STATE_NORMAL = 0,
    BATTERY_STATE_LOW,
    BATTERY_STATE_CRITICAL,
    BATTERY_STATE_OVERVOLTAGE,
    BATTERY_STATE_UNAVAILABLE
} battery_state_t;

/*==============================================================================
 * Data structure
 *============================================================================*/
typedef struct {
    uint16_t        raw_adc;        /* Raw ADC value */
    uint16_t        voltage_mv;     /* Battery voltage in mV */
    uint8_t         percent;        /* Battery percentage 0-100% */
    battery_state_t state;          /* Battery state */
    battery_status_t last_status;   /* Last operation status */
} battery_data_t;

/*==============================================================================
 * Public functions
 *============================================================================*/
void        battery_init(void);
battery_status_t battery_update(battery_data_t *bat_data);
battery_status_t battery_get_last_status(void);

uint16_t    battery_get_voltage_mv(void);
uint8_t     battery_get_percent(void);
battery_state_t battery_get_state(void);
uint8_t     battery_is_low(void);
uint8_t     battery_is_critical(void);
uint8_t     battery_is_overvoltage(void);

#endif /* __BATTERY_H__ */

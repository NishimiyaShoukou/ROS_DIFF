/******************************************************************************
 * @file    battery.c
 * @brief   Battery voltage measurement implementation.
 * @author  User
 * @date    2026-07-08
 * @version V1.0.0
 * @note    Part of the STM32F1 robot control project.
 *
 *  Features:
 *    - 2S/3S LiPo battery support (configured via BATTERY_CELL_COUNT)
 *    - ADC calibration and averaging filter
 *    - Low/Critical battery warnings
 ******************************************************************************/
#include "battery.h"
#include "adc.h"

/*==============================================================================
 * External dependencies
 *============================================================================*/
extern ADC_HandleTypeDef hadc1;

/*==============================================================================
 * Private variables
 *============================================================================*/
static uint16_t s_voltage_mv = 0;
static uint8_t  s_percent = 0;
static battery_state_t s_state = BATTERY_STATE_UNAVAILABLE;
static battery_status_t s_last_status = BATTERY_ERR_NOT_INIT;
static uint8_t s_init_ok = 0;

/*==============================================================================
 * Private function prototypes
 *============================================================================*/
static battery_status_t read_adc_with_filter(uint16_t *raw_adc);
static uint16_t raw_to_voltage_mv(uint16_t raw_adc);
static uint8_t  voltage_to_percent(uint16_t voltage_mv);
static battery_state_t voltage_to_state(uint16_t voltage_mv);

/*==============================================================================
 * Public functions
 *============================================================================*/

/**
 * @brief  Initialize battery monitor
 * @note   Call once at system startup after HAL_Init()
 */
void battery_init(void)
{
    s_voltage_mv = 0;
    s_percent = 0;
    s_state = BATTERY_STATE_UNAVAILABLE;
    s_init_ok = 0;

    if (HAL_ADCEx_Calibration_Start(&hadc1) == HAL_OK) {
        s_last_status = BATTERY_OK;
        s_init_ok = 1;
    } else {
        s_last_status = BATTERY_ERR_CALIBRATION;
    }
}

/**
 * @brief  Update battery data
 * @param  bat_data: Pointer to battery_data_t structure
 * @retval battery_status_t: Operation status
 */
battery_status_t battery_update(battery_data_t *bat_data)
{
    uint16_t raw_adc;
    uint16_t voltage_mv;

    if (bat_data == NULL) {
        s_last_status = BATTERY_ERR_NULL_PTR;
        return BATTERY_ERR_NULL_PTR;
    }

    if (!s_init_ok) {
        s_last_status = BATTERY_ERR_NOT_INIT;
        return BATTERY_ERR_NOT_INIT;
    }

    s_last_status = read_adc_with_filter(&raw_adc);
    if (s_last_status != BATTERY_OK) {
        s_state = BATTERY_STATE_UNAVAILABLE;
        bat_data->raw_adc = 0;
        bat_data->voltage_mv = s_voltage_mv;
        bat_data->percent = s_percent;
        bat_data->state = s_state;
        bat_data->last_status = s_last_status;
        return s_last_status;
    }

    voltage_mv = raw_to_voltage_mv(raw_adc);

    s_voltage_mv = voltage_mv;
    s_percent = voltage_to_percent(voltage_mv);
    s_state = voltage_to_state(voltage_mv);
    s_last_status = (s_state == BATTERY_STATE_OVERVOLTAGE) ?
        BATTERY_ERR_OVERVOLTAGE : BATTERY_OK;

    /* Fill output structure */
    bat_data->raw_adc = raw_adc;
    bat_data->voltage_mv = voltage_mv;
    bat_data->percent = s_percent;
    bat_data->state = s_state;
    bat_data->last_status = s_last_status;

    return s_last_status;
}

/**
 * @brief  Get last operation status
 * @retval battery_status_t: Last status code
 */
battery_status_t battery_get_last_status(void)
{
    return s_last_status;
}

/**
 * @brief  Get current battery voltage in mV
 * @retval uint16_t: Voltage in millivolts
 */
uint16_t battery_get_voltage_mv(void)
{
    return s_voltage_mv;
}

/**
 * @brief  Get estimated battery percentage (0-100)
 * @retval uint8_t: Battery percentage
 */
uint8_t battery_get_percent(void)
{
    return s_percent;
}

/**
 * @brief  Get current battery state
 * @retval battery_state_t: Current state
 */
battery_state_t battery_get_state(void)
{
    return s_state;
}

/**
 * @brief  Check if battery is in low state
 * @retval uint8_t: 1 if low, 0 otherwise
 */
uint8_t battery_is_low(void)
{
    return (s_state == BATTERY_STATE_LOW) ? 1 : 0;
}

/**
 * @brief  Check if battery is in critical state
 * @retval uint8_t: 1 if critical, 0 otherwise
 */
uint8_t battery_is_critical(void)
{
    return (s_state == BATTERY_STATE_CRITICAL) ? 1 : 0;
}

/**
 * @brief  Check if overvoltage protection was triggered
 * @retval uint8_t: 1 if overvoltage, 0 otherwise
 */
uint8_t battery_is_overvoltage(void)
{
    return (s_state == BATTERY_STATE_OVERVOLTAGE) ? 1U : 0U;
}

/*==============================================================================
 * Private functions
 *============================================================================*/

/**
 * @brief  Read ADC with moving average filter
 * @retval uint16_t: Filtered ADC value
 */
static battery_status_t read_adc_with_filter(uint16_t *raw_adc)
{
    uint32_t sum = 0;
    uint8_t i;
    HAL_StatusTypeDef hal_status;

    if (raw_adc == NULL) {
        return BATTERY_ERR_NULL_PTR;
    }

    for (i = 0; i < BATTERY_SAMPLES; i++) {
        hal_status = HAL_ADC_Start(&hadc1);
        if (hal_status != HAL_OK) {
            (void)HAL_ADC_Stop(&hadc1);
            return BATTERY_ERR_ADC_TIMEOUT;
        }
        hal_status = HAL_ADC_PollForConversion(&hadc1, 2);
        if (hal_status != HAL_OK) {
            (void)HAL_ADC_Stop(&hadc1);
            return BATTERY_ERR_ADC_TIMEOUT;
        }
        sum += HAL_ADC_GetValue(&hadc1);
        (void)HAL_ADC_Stop(&hadc1);
    }

    *raw_adc = (uint16_t)(sum / BATTERY_SAMPLES);
    return BATTERY_OK;
}

/**
 * @brief  Convert raw ADC value to battery voltage (mV)
 * @param  raw_adc: Raw ADC value (0-4095)
 * @retval uint16_t: Battery voltage in millivolts
 */
static uint16_t raw_to_voltage_mv(uint16_t raw_adc)
{
    float divider_ratio = BATTERY_DIVIDER_RATIO;
    float vout_mv;
    float vin_mv;

    /* ADC output voltage */
    vout_mv = ((float)raw_adc / (float)BATTERY_ADC_MAX) * BATTERY_VREF_MV;

    /* Restore original voltage using divider ratio */
    vin_mv = vout_mv * divider_ratio * BATTERY_CALIBRATION_SCALE;

    return (uint16_t)(vin_mv + 0.5f);
}

/**
 * @brief  Convert battery voltage to percentage
 * @param  voltage_mv: Battery voltage in mV
 * @retval uint8_t: Estimated percentage (0-100)
 */
static uint8_t voltage_to_percent(uint16_t voltage_mv)
{
    int16_t percent;

    if (voltage_mv <= BATTERY_MIN_MV) {
        return 0;
    }
    if (voltage_mv >= BATTERY_MAX_MV) {
        return 100;
    }

    percent = (int16_t)(
        ((float)(voltage_mv - BATTERY_MIN_MV) / (float)(BATTERY_MAX_MV - BATTERY_MIN_MV)) * 100.0f
    );

    if (percent < 0)   percent = 0;
    if (percent > 100) percent = 100;

    return (uint8_t)percent;
}

/**
 * @brief  Determine battery state from voltage
 * @param  voltage_mv: Battery voltage in mV
 * @retval battery_state_t: Battery state
 */
static battery_state_t voltage_to_state(uint16_t voltage_mv)
{
    if (voltage_mv < BATTERY_PRESENT_MIN_MV) {
        return BATTERY_STATE_UNAVAILABLE;
    }
    if (voltage_mv > BATTERY_OVERVOLTAGE_MV) {
        return BATTERY_STATE_OVERVOLTAGE;
    }
    if (voltage_mv <= BATTERY_CRITICAL_MV) {
        return BATTERY_STATE_CRITICAL;
    }
    if (voltage_mv <= BATTERY_LOW_MV) {
        return BATTERY_STATE_LOW;
    }
    return BATTERY_STATE_NORMAL;
}

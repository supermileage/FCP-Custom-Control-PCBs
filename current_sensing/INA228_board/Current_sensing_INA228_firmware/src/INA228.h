#pragma once

#include "stm32l4xx_hal.h"
#include "C:\Users\anhph\FCP-Custom-Control-PCBs\current_sensing\INA228_board\Current_sensing_INA228_firmware\Core\Inc\main.h"
#include "stm32l4xx_hal_gpio.h"
#include "stm32l4xx_hal_i2c.h"
#include "stm32l4xx_hal_usart.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// INA228 register map (8-bit register addresses)
#define INA228_REG_CONFIG        0x00
#define INA228_REG_ADC_CONFIG    0x01
#define INA228_REG_SHUNT_CAL     0x02
#define INA228_REG_CURRENT       0x07

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint8_t  addr7;          // 7-bit I2C address (e.g. 0x40)
    float    shunt_ohms;      // shunt resistance (ohms)
    float    max_current_a;   // expected max current (A)

    // computed calibration
    float    current_lsb;     // A/LSB
    uint16_t shunt_cal;       // SHUNT_CAL register
    bool     adc_range_high;  // ADCRANGE setting
} INA228_Handle_t;

// Basic init (stores params, does not touch hardware)
void INA228_Init(INA228_Handle_t *dev,
                 I2C_HandleTypeDef *hi2c,
                 uint8_t addr7,
                 float shunt_ohms,
                 float max_current_a);

// Writes CONFIG, ADC_CONFIG, SHUNT_CAL
bool INA228_Begin(INA228_Handle_t *dev,
                  uint16_t config,
                  uint16_t adc_config,
                  bool adc_range_high);

// Reads CURRENT register and converts to amps
bool INA228_ReadCurrentA(INA228_Handle_t *dev, float *current_a);

#ifdef __cplusplus
}
#endif
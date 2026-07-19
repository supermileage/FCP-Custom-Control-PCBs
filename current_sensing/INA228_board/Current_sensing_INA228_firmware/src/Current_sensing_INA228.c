#include "ina228.h"

static void INA228_ComputeCalibration(INA228_Handle_t *dev)
{
    // CURRENT_LSB = Imax / 2^19  (matches your earlier logic)
    dev->current_lsb = dev->max_current_a / (float)(1UL << 19);

    // SHUNT_CAL = 13107.2e6 * CURRENT_LSB * Rshunt
    // (this constant matches the formula you were using)
    const float k = 1.31072e10f;
    float cal = k * dev->current_lsb * dev->shunt_ohms;

    // If ADCRANGE = 1 (high range), multiply by 4 (same as your earlier logic)
    if (dev->adc_range_high) cal *= 4.0f;

    if (cal < 0.0f) cal = 0.0f;
    if (cal > 65535.0f) cal = 65535.0f;

    dev->shunt_cal = (uint16_t)(cal + 0.5f);
}

static bool INA228_WriteReg16(INA228_Handle_t *dev, uint8_t reg, uint16_t value)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(value >> 8);
    buf[1] = (uint8_t)(value & 0xFF);

    uint16_t addr8 = (uint16_t)(dev->addr7 << 1);

    return (HAL_I2C_Mem_Write(dev->hi2c,
                              addr8,
                              reg,
                              I2C_MEMADD_SIZE_8BIT,
                              buf,
                              2,
                              100) == HAL_OK);
}

static bool INA228_ReadReg24(INA228_Handle_t *dev, uint8_t reg, uint32_t *value)
{
    uint8_t buf[3] = {0};
    uint16_t addr8 = (uint16_t)(dev->addr7 << 1);

    if (HAL_I2C_Mem_Read(dev->hi2c,
                         addr8,
                         reg,
                         I2C_MEMADD_SIZE_8BIT,
                         buf,
                         3,
                         100) != HAL_OK)
    {
        return false;
    }

    *value = ((uint32_t)buf[0] << 16) |
             ((uint32_t)buf[1] << 8)  |
             ((uint32_t)buf[2]);
    return true;
}

void INA228_Init(INA228_Handle_t *dev,
                 I2C_HandleTypeDef *hi2c,
                 uint8_t addr7,
                 float shunt_ohms,
                 float max_current_a)
{
    dev->hi2c = hi2c;
    dev->addr7 = addr7;
    dev->shunt_ohms = shunt_ohms;
    dev->max_current_a = max_current_a;

    dev->current_lsb = 0.0f;
    dev->shunt_cal = 0;
    dev->adc_range_high = false;
}

bool INA228_Begin(INA228_Handle_t *dev,
                  uint16_t config,
                  uint16_t adc_config,
                  bool adc_range_high)
{
    dev->adc_range_high = adc_range_high;

    INA228_ComputeCalibration(dev);

    // Your earlier assumption: ADCRANGE bit is bit 4 in ADC_CONFIG.
    // Keep consistent with your code.
    if (dev->adc_range_high) adc_config |=  (1u << 4);
    else                     adc_config &= ~(1u << 4);

    if (!INA228_WriteReg16(dev, INA228_REG_CONFIG, config)) return false;
    if (!INA228_WriteReg16(dev, INA228_REG_ADC_CONFIG, adc_config)) return false;
    if (!INA228_WriteReg16(dev, INA228_REG_SHUNT_CAL, dev->shunt_cal)) return false;

    return true;
}

bool INA228_ReadCurrentA(INA228_Handle_t *dev, float *current_a)
{
    uint32_t raw24 = 0;
    if (!INA228_ReadReg24(dev, INA228_REG_CURRENT, &raw24)) return false;

    // CURRENT is 20-bit two's complement in bits [23:4]
    uint32_t raw20 = raw24 >> 4;

    int32_t s20 = (int32_t)raw20;
    if (s20 & 0x80000) {
        // sign extend 20-bit -> 32-bit
        s20 |= (int32_t)0xFFF00000;
    }

    *current_a = (float)s20 * dev->current_lsb;
    return true;
}
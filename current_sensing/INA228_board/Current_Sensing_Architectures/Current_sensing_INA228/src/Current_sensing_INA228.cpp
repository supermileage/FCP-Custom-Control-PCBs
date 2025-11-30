// INA228.cpp
#include "INA228.h"

// Constructor
INA228::INA228(uint8_t i2cAddr, float shuntOhms, float maxCurrentA, TwoWire &wirePort)
    : wire(wirePort),
      addr(i2cAddr),
      shunt(shuntOhms),
      maxCurrent(maxCurrentA),
      currentLSB(0.0f),
      shuntCal(0),
      adcRangeHigh(true) // default to high range (±163.84mV)
{
}

// Compute CURRENT_LSB and SHUNT_CAL based on datasheet formulas
void INA228::computeCalibration()
{
    // CURRENT_LSB = Imax / 2^19  (20-bit signed, magnitude 2^19)
    currentLSB = maxCurrent / (float)(1 << 19); // A per LSB

    // Datasheet formula (approx):
    // SHUNT_CAL = 13107.2 * 10^6 * CURRENT_LSB * Rshunt
    // If ADCRANGE = 1 (high range), multiply by 4.
    const float k = 1.31072e10f; // 13107.2 * 10^6
    float cal = k * currentLSB * shunt;

    if (adcRangeHigh)
    {
        cal *= 4.0f;
    }

    // Clamp & store as uint16_t
    if (cal > 65535.0f)
        cal = 65535.0f;
    else if (cal < 0.0f)
        cal = 0.0f;

    shuntCal = (uint16_t)(cal + 0.5f); // round to nearest
}

// Write a 16-bit register
bool INA228::writeReg16(uint8_t reg, uint16_t value)
{
    wire.beginTransmission(addr);
    wire.write(reg);
    wire.write((uint8_t)(value >> 8));    // MSB
    wire.write((uint8_t)(value & 0xFF));  // LSB
    return (wire.endTransmission() == 0);
}

// Read a 24-bit register (e.g., CURRENT)
bool INA228::readReg24(uint8_t reg, uint32_t &value)
{
    // Write register address
    wire.beginTransmission(addr);
    wire.write(reg);
    if (wire.endTransmission(false) != 0) // repeated start
    {
        return false;
    }

    // Request 3 bytes
    uint8_t count = wire.requestFrom(addr, (uint8_t)3);
    if (count != 3)
    {
        return false;
    }

    uint32_t v = 0;
    v |= (uint32_t)wire.read() << 16; // MSB
    v |= (uint32_t)wire.read() << 8;  // MID
    v |= (uint32_t)wire.read();       // LSB

    value = v;
    return true;
}

// Initialize device
bool INA228::begin(uint16_t config, uint16_t adcConfig, bool adcrangeHigh_)
{
    adcRangeHigh = adcrangeHigh_;

    // Compute CURRENT_LSB & SHUNT_CAL based on given shunt and Imax
    computeCalibration();

    // Optionally tweak adcConfig to ensure ADCRANGE matches 'adcRangeHigh'
    // ADCRANGE is bit 4 in ADC_CONFIG (check datasheet); here we enforce it.
    if (adcRangeHigh)
        adcConfig |= (1 << 4);
    else
        adcConfig &= ~(1 << 4);

    // Write CONFIG
    if (!writeReg16(REG_CONFIG, config))
        return false;

    // Write ADC_CONFIG
    if (!writeReg16(REG_ADC_CONFIG, adcConfig))
        return false;

    // Write SHUNT_CAL
    if (!writeReg16(REG_SHUNT_CAL, shuntCal))
        return false;

    return true;
}

// Read current in amps
bool INA228::readCurrent(float &currentA)
{
    uint32_t raw24 = 0;
    if (!readReg24(REG_CURRENT, raw24))
        return false;

    // CURRENT is 20-bit two's complement in bits [23:4]
    uint32_t raw20 = raw24 >> 4; // drop lower 4 bits

    int32_t s20 = (int32_t)raw20;
    if (s20 & 0x80000) // bit 19 is sign
    {
        s20 |= 0xFFF00000; // sign-extend into 32 bits
    }

    currentA = (float)s20 * currentLSB;
    return true;
}

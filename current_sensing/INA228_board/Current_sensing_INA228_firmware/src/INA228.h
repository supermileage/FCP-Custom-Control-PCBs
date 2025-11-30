// INA228.h
#pragma once

#include "Wire.h"

class INA228
{
public:
    // Default 7-bit address (depends on A0/A1 wiring – adjust as needed)
    static constexpr uint8_t DEFAULT_ADDR = 0x40;

    // Constructor:
    //  - i2cAddr: INA228 I2C address (7-bit)
    //  - shuntOhms: shunt resistor in ohms
    //  - maxCurrentA: maximum expected current in amps (used to compute CURRENT_LSB & SHUNT_CAL)
    INA228(uint8_t i2cAddr,
           float shuntOhms,
           float maxCurrentA,
           TwoWire &wirePort = Wire);

    // Initialize device: config registers, compute & write SHUNT_CAL
    bool begin(uint16_t config = 0x0000, uint16_t adcConfig = 0xFB68, bool adcrangeHigh = true);

    // Read current in amps (bidirectional)
    bool readCurrent(float &currentA);

    // (Optional to implement later)
    // bool readBusVoltage(float &voltageV);
    // bool readShuntVoltage(float &vshunt);

private:
    // I2C & device parameters
    TwoWire &wire;
    uint8_t addr;
    float shunt;
    float maxCurrent;
    float currentLSB;    // A/LSB
    uint16_t shuntCal;   // value written to SHUNT_CAL register
    bool adcRangeHigh;   // ADCRANGE bit (true = ±40.96mV, false = ±163.84mV)

    // --- Register addresses (from datasheet) ---
    static constexpr uint8_t REG_CONFIG      = 0x00;
    static constexpr uint8_t REG_ADC_CONFIG  = 0x01;
    static constexpr uint8_t REG_SHUNT_CAL   = 0x02;
    static constexpr uint8_t REG_CURRENT     = 0x07;
    // static constexpr uint8_t REG_VSHUNT      = 0x04;
    // static constexpr uint8_t REG_VBUS        = 0x05;

    // Internal helpers
    void computeCalibration();
    bool writeReg16(uint8_t reg, uint16_t value);
    bool readReg24(uint8_t reg, uint32_t &value);
};

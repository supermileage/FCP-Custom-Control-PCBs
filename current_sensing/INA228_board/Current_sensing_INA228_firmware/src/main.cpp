// main.cpp
#include <Arduino.h>
#include "INA228.h"

// Example: A0 = GND, A1 = GND → address 0x40
// Shunt = 10 Ω, Imax = 30 A
INA228 ina(INA228::DEFAULT_ADDR, 10.0f, 30.0f);

void setup()
{
    Serial.begin(9600);
    delay(1000);

    Wire.begin();

    // CONFIG = 0x0000 (defaults)
    // ADC_CONFIG = 0xFB68 (example from TI: continuous, averaging, etc.)
    // adcrangeHigh = true (±163.84mV)
    bool ok = ina.begin(0x0000, 0xFB68, false);

    if (!ok)
    {
        Serial.println("INA228 init failed!");
    }
    else
    {
        Serial.println("INA228 init OK, SHUNT_CAL programmed.");
    }
}

void loop()
{
    float currentA = 0.0f;

    if (ina.readCurrent(currentA))
    {
        Serial.println("Current: " + String(currentA, 4) + " A");
    }
    else
    {
        Serial.println("Failed to read current");
    }

    delay(500);
}

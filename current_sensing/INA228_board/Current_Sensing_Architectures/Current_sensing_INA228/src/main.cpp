// main.cpp
#include "Particle.h"
#include "INA228.h"

SYSTEM_MODE(AUTOMATIC);
SYSTEM_THREAD(ENABLED);

SerialLogHandler logHandler(LOG_LEVEL_INFO);

// Example: A0 = GND, A1 = GND → address 0x40
// Shunt = 2.5 mΩ, Imax = 30 A
INA228 ina(INA228::DEFAULT_ADDR, 0.0025f, 30.0f);

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Wire.begin();

    // CONFIG = 0x0000 (defaults)
    // ADC_CONFIG = 0xFB68 (example from TI: continuous, averaging, etc.)
    // adcrangeHigh = true (±163.84mV)
    bool ok = ina.begin(0x0000, 0xFB68, true);

    if (!ok)
    {
        Log.error("INA228 init failed!");
    }
    else
    {
        Log.info("INA228 init OK, SHUNT_CAL programmed.");
    }
}

void loop()
{
    float currentA = 0.0f;

    if (ina.readCurrent(currentA))
    {
        Log.info("Current: %0.4f A", currentA);
    }
    else
    {
        Log.error("Failed to read current");
    }

    delay(500);
}

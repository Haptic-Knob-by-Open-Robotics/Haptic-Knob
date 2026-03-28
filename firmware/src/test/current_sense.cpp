#include <Arduino.h>
#include <SimpleFOC.h>
#include "../app/Config.h"

InlineCurrentSense current_sense = InlineCurrentSense(SHUNT_RESISTOR_OHM, CURRENT_SENSE_AMP_GAIN, PIN_I_A, PIN_I_B, PIN_I_C);
BLDCDriver6PWM driver = BLDCDriver6PWM(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL, PIN_EN);

void setup()
{
    Serial.begin(115200);
    delay(3000);

    Serial.println("\n=== CURRENT SENSE TEST ===\n");

    // Initialize driver
    Serial.print("Driver init... ");
    driver.voltage_power_supply = 12;
    driver.init();
    Serial.println("done");

    // Initialize current sense
    Serial.print("Current sense init... ");
    current_sense.linkDriver(&driver);

    if (current_sense.init())
    {
        Serial.println("SUCCESS");
    }
    else
    {
        Serial.println("FAILED");
        Serial.println("Check:");
        Serial.println("  - ADC pin connections");
        Serial.println("  - Shunt resistor values");
        Serial.println("  - Op-amp gain");
        while (1)
            ;
    }

    Serial.println("\nCalibrating...");
    current_sense.driverAlign(12); // just the voltage
    Serial.println("Calibration done");

    Serial.println("\n=== Reading Current ===");
    Serial.println("Phase A | Phase B | Phase C");
    Serial.println("--------|---------|--------");
}

void loop()
{
    // Read phase currents
    PhaseCurrent_s currents = current_sense.getPhaseCurrents();

    // Print currents
    Serial.print(currents.a, 3);
    Serial.print(" A | ");
    Serial.print(currents.b, 3);
    Serial.print(" A | ");
    Serial.print(currents.c, 3);
    Serial.println(" A");

    delay(500);
}
#include <Arduino.h>
#include <SimpleFOC.h>
#include "app/Config.h"
#include "drivers/ModifiedMagneticSensorMT6701SSI.h"
#include "drivers/SpiBus.h"

SpiBus spiBus(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI);
ModifiedMagneticSensorMT6701SSI encoder(PIN_ENC_CS);
InlineCurrentSense current_sense(SHUNT_RESISTOR_OHM, CURRENT_SENSE_AMP_GAIN, PIN_I_A, PIN_I_B, PIN_I_C);
BLDCDriver6PWM driver(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL);
BLDCMotor motor(POLE_PAIRS);

static float clampf(float x, float lo, float hi)
{
    if (x < lo)
        return lo;
    if (x > hi)
        return hi;
    return x;
}

void setup()
{
    Serial.begin(115200);
    delay(1500);
    Serial.println("=== Diode Mode Haptic Knob ===");

    spiBus.init();
    encoder.init(spiBus.bus());
    Serial.println("Encoder initialized!");

    driver.voltage_power_supply = VOLTAGE_SUPPLY_V;
    driver.voltage_limit = DRIVER_VOLTAGE_LIMIT_V;
    driver.pwm_frequency = 30000;
    driver.dead_zone = 0.05f;
    if (!driver.init())
    {
        Serial.println("Driver FAILED");
        while (1)
            ;
    }
    Serial.println("Driver initialized");

    current_sense.linkDriver(&driver);
    if (!current_sense.init())
    {
        Serial.println("Current sense FAILED");
        while (1)
            ;
    }
    current_sense.driverAlign(1.0f); // low alignment voltage
    Serial.println("Current sense ready");

    motor.linkDriver(&driver);
    motor.linkSensor(&encoder);
    motor.linkCurrentSense(&current_sense);

    motor.controller = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::voltage; // no PI tuning needed
    motor.voltage_limit = DRIVER_VOLTAGE_LIMIT_V;
    motor.voltage_sensor_align = 1.0f;
    motor.LPF_velocity.Tf = 0.15f; // smooth velocity estimate

    motor.init();
    motor.initFOC();
    motor.target = 0.0f;

    Serial.println("Ready - diode mode active (CW free, CCW blocked)");
}

void loop()
{
    motor.loopFOC();
    motor.move(3.0f);
    delay(10);
}
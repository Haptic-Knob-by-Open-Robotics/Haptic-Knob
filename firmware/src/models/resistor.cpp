#include <Arduino.h>
#include <SimpleFOC.h>
#include "app/Config.h"
#include "drivers/ModifiedMagneticSensorMT6701SSI.h"
#include "drivers/SpiBus.h"

SpiBus spiBus(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI);
ModifiedMagneticSensorMT6701SSI encoder(PIN_ENC_CS);
InlineCurrentSense current_sense(SHUNT_RESISTOR, AMP_GAIN, PIN_I_A, PIN_I_B, PIN_I_C);
BLDCDriver6PWM driver(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL);
BLDCMotor motor(POLE_PAIRS);

void getData()
{
    float angleDeg = encoder.angleDegWrapped();
    float angleRad = encoder.getAngle();
    float velocity = encoder.getVelocity();
    PhaseCurrent_s currents = current_sense.getPhaseCurrents();

    Serial.printf("Angle: %7.2f°  Velocity: %6.2f rad/s  Ia: %7.3fA  Ib: %7.3fA  Ic: %7.3fA\n",
                  angleDeg, velocity, currents.a, currents.b, currents.c);
}

void driverSetup()
{
    // ESP32 PWM Frequency
    driver.pwm_frequency = 30000;

    // Driver Deadzone, safety feature
    driver.dead_zone = 0.05;

    // power supply voltage [V]
    driver.voltage_power_supply = VOLTAGE_SUPPLY;

    // Max DC voltage allowed
    driver.voltage_limit = VOLTAGE_LIMIT;

    driver.init();
    Serial.print("Driver init ");
    if (driver.init())
        Serial.println("success!");
    else
    {
        Serial.println("failed!");
        return;
    }

    driver.enable();
}

void currentSenseSetup()
{
    // Current
    current_sense.linkDriver(&driver);

    // init current sense
    if (current_sense.init())
        Serial.println("Current sense init success!");
    else
    {
        Serial.println("Current sense init failed!");
        while (1)
            ;
    }

    Serial.println("Calibrating...");
    current_sense.driverAlign(VOLTAGE_LIMIT);
    Serial.println("Calibration done\n");
}

void motorSetup()
{
    motor.linkDriver(&driver);
    motor.linkSensor(&encoder);
    motor.controller = MotionControlType::velocity_openloop;
    motor.voltage_limit = VOLTAGE_LIMIT; // keep low for testing
    motor.init();
}

void setup()
{
    Serial.begin(115200);
    delay(3000);
    Serial.println("=== Haptic Knob -- Resistor Mode ===");

    // SPI Initialization
    spiBus.init();
    encoder.init(spiBus.bus());
    Serial.println("Encoder initialized!");

    // Driver Configuration
    driverSetup();

    // Current Sense Configuration
    currentSenseSetup();

    // Motor Configuration
    motorSetup();
}

void loop()
{
    encoder.update();
    float operatorGain = 1;
    float resistance = 50.0f;
    float motorAnguVel = encoder.getVelocity();
    float angleRad = encoder.getSensorAngle();
    float torqueSetpoint = operatorGain * resistance * motorAnguVel;
    float Kt = 0.035;
    float desiredCurrent = torqueSetpoint / Kt;

    static uint32_t start = millis();
    static uint32_t last_ms = 0;
    uint32_t elapsed = (millis() - start) % 6000;

    if (millis() - last_ms > 100)
    {
        last_ms = millis();
        getData();
    }
}
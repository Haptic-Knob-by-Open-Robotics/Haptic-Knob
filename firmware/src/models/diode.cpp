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

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

// Model Paramaters
static constexpr float CCW_THRESHOLD = 0.1f;   // rad/s

static constexpr float TORQUE_CONST = 0.035f;   // N*m/A

// Virtual diode damping [N*m / (rad/s)]
static constexpr float B_virtual = 0.02f;

static constexpr float MAX_TORQUE  = 0.12f;    // N*m
static constexpr float MAX_CURRENT = 2.0f;     // A

void setup()
{
    Serial.begin(115200);
    delay(1500);
    Serial.println("Diode Mode Haptic Knob");

    // 1. Encoder
    spiBus.init();
    encoder.init(spiBus.bus());
    encoder.update();
    Serial.println("Encoder initialized");

    // 2. Driver
    driver.voltage_power_supply = VOLTAGE_SUPPLY;
    driver.voltage_limit = VOLTAGE_LIMIT;
    driver.pwm_frequency = 30000;
    driver.dead_zone = 0.05f;

    if (!driver.init())
    {
        Serial.println("Driver FAILED");
        while (1) {}
    }
    driver.enable();
    Serial.println("Driver initialized");

    // 3. Current sense
    current_sense.linkDriver(&driver);
    if (!current_sense.init())
    {
        Serial.println("Current sense FAILED");
        while (1) {}
    }
    Serial.println("Current sense initialized");

    // 4. Motor
    motor.linkDriver(&driver);
    motor.linkSensor(&encoder);
    motor.linkCurrentSense(&current_sense);

    motor.controller = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::foc_current;

    motor.voltage_limit = VOLTAGE_LIMIT;
    motor.current_limit = MAX_CURRENT;
    motor.voltage_sensor_align = 1.0f;

    motor.LPF_velocity.Tf = 0.05f;

    // current loop tuning
    // motor.PID_current_q.P = ...;
    // motor.PID_current_q.I = ...;
    // motor.LPF_current_q.Tf = ...;

    motor.init();

    if (!motor.initFOC())
    {
        Serial.println("FOC FAILED");
        while (1) {}
    }

    motor.target = 0.0f;

    Serial.println("Diode mode ready");
}

void loop()
{
    // 1. Update sensor
    encoder.update();

    // 2. Run electrical FOC loop
    motor.loopFOC();

    // 3. Read shaft state
    const float omega = motor.shaftVelocity();
    const float angleDeg = encoder.angleDegWrapped();

    // 4. Diode logic
    float tau_des = 0.0f;

    if (omega > CCW_THRESHOLD)
    {
        tau_des = -B_virtual * omega;
        tau_des = clampf(tau_des, -MAX_TORQUE, 0.0f);
    }

    // 5. Convert torque to q-axis current
    float iq_des = tau_des / KT;
    iq_des = clampf(iq_des, -MAX_CURRENT, MAX_CURRENT);

    // 6. Apply current command
    motor.move(iq_des);

    // 7. Telemetry
    static uint32_t last_ms = 0;
    if (millis() - last_ms > 100)
    {
        last_ms = millis();
        Serial.printf(
            "Angle: %6.1f deg | Vel: %7.3f rad/s | Tau: %7.4f N*m | Iq_des: %6.3f A\n",
            angleDeg, omega, tau_des, iq_des);
    }
}
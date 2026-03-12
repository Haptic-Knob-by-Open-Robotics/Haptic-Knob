#include <Arduino.h>
#include <SimpleFOC.h>
#include "app/Config.h"
#include "drivers/ModifiedMagneticSensorMT6701SSI.h"
#include "drivers/SpiBus.h"

SpiBus spiBus(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI);
ModifiedMagneticSensorMT6701SSI encoder(PIN_ENC_CS);
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

// ── Diode mode parameters ──────────────────────────────────────────────────
// CCW rotation (negative omega) is blocked, CW rotation is free.
// Resistance only kicks in once velocity crosses CCW_THRESHOLD.
static constexpr float CCW_THRESHOLD = 0.1f; // rad/s — how fast CCW before resistance starts
static constexpr float DIODE_R_FEEL = 2.0f;   // V/(rad/s) — proportional resistance strength

void setup()
{
    Serial.begin(115200);
    delay(1500);
    Serial.println("=== Diode Mode Haptic Knob ===");

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
        while (1)
            ;
    }
    driver.enable();
    Serial.println("Driver initialized");

    // 3. Motor
    motor.linkDriver(&driver);
    motor.linkSensor(&encoder);

    motor.controller = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::voltage; // no current PID needed
    motor.voltage_limit = VOLTAGE_LIMIT;
    motor.voltage_sensor_align = 1.0f; // safe for 5V supply

    motor.LPF_velocity.Tf = 0.05f; // smooth velocity, low enough to stay responsive

    motor.init();
    if (!motor.initFOC())
    {
        Serial.println("FOC FAILED");
        while (1)
            ;
    }
    motor.target = 0.0f;

    Serial.println("Ready — CW free, CCW blocked");
}

void loop()
{
    // 1. Encoder first so loopFOC gets fresh position data
    encoder.update();

    // 2. FOC electrical loop
    motor.loopFOC();

    // 3. Read velocity
    const float omega = motor.shaftVelocity();
    const float angleDeg = encoder.angleDegWrapped();

    // 4. Diode logic — only resist CCW (negative omega) past threshold
    float v_des = 0.0f;
    if (omega > CCW_THRESHOLD)
    {
        // Proportional resistance: pushes back harder the faster you go CCW
        v_des = clampf(-DIODE_R_FEEL * omega, -VOLTAGE_LIMIT, 0.0f);
        // Clamp upper bound to 0 so we never accidentally assist CW motion
    }

    // 5. Apply voltage command
    motor.move(v_des);

    // 6. Telemetry at 10 Hz
    static uint32_t last_ms = 0;
    if (millis() - last_ms > 100)
    {
        last_ms = millis();
        Serial.printf(
            "Angle: %6.1f deg | Vel: %7.3f rad/s | Vdes: %6.3f V\n",
            angleDeg, omega, v_des);
    }
}
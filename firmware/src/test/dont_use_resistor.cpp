#include <Arduino.h>
#include <SPI.h>

#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>

#include "../app/Config.h"
#include "../drivers/ModifiedMagneticSensorMT6701SSI.h"

// -------------------- SimpleFOC objects --------------------
BLDCMotor motor(POLE_PAIRS);
BLDCDriver6PWM driver(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL, PIN_EN);

// Sensor + current sense
ModifiedMagneticSensorMT6701SSI encoder(PIN_ENC_CS);
InlineCurrentSense current_sense(SHUNT_RESISTOR, AMP_GAIN, PIN_I_A, PIN_I_B, PIN_I_C);

// Optional runtime tuning via serial
Commander command(Serial);
float R_FEEL_RUNTIME = R_FEEL;
float IQ_MAX_RUNTIME = IQ_MAX;

void doRfeel(char *cmd) { command.scalar(&R_FEEL_RUNTIME, cmd); }
void doIqMax(char *cmd) { command.scalar(&IQ_MAX_RUNTIME, cmd); }
void doCurrentLimit(char *cmd) { command.scalar(&motor.current_limit, cmd); }
void doVoltageLimit(char *cmd) { command.scalar(&motor.voltage_limit, cmd); }

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

    Serial.println("\n=== HAPTIC KNOB: MT6701 + Current Sense + SimpleFOC Closed Loop ===");
    Serial.println("Model: Resistor feel -> iq_des = R_FEEL * omega\n");

    // 1) SPI pins for MT6701 (CLK, MISO, MOSI = -1, CS)
    SPI.begin(PIN_ENC_CLK, PIN_ENC_MISO, -1, PIN_ENC_CS);

    // 2) Init sensor
    encoder.init(&SPI);
    Serial.println("Encoder initialized");

    // 3) Init 6PWM driver
    driver.voltage_power_supply = SUPPLY_VOLTAGE;
    driver.voltage_limit = VOLTAGE_LIMIT;
    driver.pwm_frequency = 30000;
    driver.dead_zone = 0.05f;

    if (!driver.init())
    {
        Serial.println("Driver init FAILED");
        while (1)
            delay(1000);
    }
    Serial.println("Driver initialized");

    // 4) Init current sense + align
    current_sense.linkDriver(&driver);

    if (!current_sense.init())
    {
        Serial.println("Current sense init FAILED (check ADC pins/wiring/gain/shunt)");
        while (1)
            delay(1000);
    }
    Serial.println("Current sense initialized");

    Serial.println("Aligning current sense...");
    current_sense.driverAlign(SUPPLY_VOLTAGE);
    Serial.println("Current sense aligned");

    // 5) Link motor to driver, sensor, current sense
    motor.linkDriver(&driver);
    motor.linkSensor(&encoder);
    motor.linkCurrentSense(&current_sense);

    // 6) Closed-loop torque using FOC current (SimpleFOC runs the inner PI/PID)
    motor.controller = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::foc_current;

    motor.voltage_limit = VOLTAGE_LIMIT;
    motor.current_limit = CURRENT_LIMIT;

    // 7) Init motor + FOC
    motor.init();
    motor.initFOC();

    // Start with no torque
    motor.target = 0.0f;

    // Serial commands
    command.add('R', doRfeel, "R_FEEL (resistance strength)");
    command.add('M', doIqMax, "IQ_MAX clamp (A)");
    command.add('I', doCurrentLimit, "motor.current_limit (A)");
    command.add('V', doVoltageLimit, "motor.voltage_limit (V)");

    Serial.println("\nCommands:");
    Serial.println("  R <val>  (increase resistance strength)");
    Serial.println("  M <amps> (iq clamp)");
    Serial.println("  I <amps> (current limit)");
    Serial.println("  V <volts>(voltage limit)\n");
    Serial.println("Turn the knob: faster turn => stronger pushback.\n");
}

void loop()
{
    // 1) Run FOC: reads encoder+currents, runs inner current PI/PID, updates PWM
    motor.loopFOC();

    // 2) Use SimpleFOC velocity estimate as omega (rad/s)
    const float omega = motor.shaftVelocity();

    // 3) Outer haptic model (resistor feel)
    float iq_des = R_FEEL_RUNTIME * omega;
    iq_des = clampf(iq_des, -IQ_MAX_RUNTIME, +IQ_MAX_RUNTIME);

    // 4) Send torque command (Iq target) live
    motor.move(iq_des);

    // 5) Debug print (10 Hz)
    static uint32_t last_ms = 0;
    if (millis() - last_ms > 100)
    {
        last_ms = millis();

        PhaseCurrent_s ph = current_sense.getPhaseCurrents();
        Serial.print("ang: ");
        Serial.print(motor.shaftAngle(), 3);
        Serial.print("\tom: ");
        Serial.print(omega, 3);
        Serial.print("\tiq: ");
        Serial.print(iq_des, 3);
        Serial.print("\tIa: ");
        Serial.print(ph.a, 3);
        Serial.print("\tIb: ");
        Serial.print(ph.b, 3);
        Serial.print("\tIc: ");
        Serial.print(ph.c, 3);
        Serial.println();
    }

    command.run();
}
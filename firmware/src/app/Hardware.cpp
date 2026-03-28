#include "app/Hardware.h"
#include <Arduino.h>
#include <SimpleFOC.h>
#include "app/Config.h"
#include "drivers/SpiBus.h"
#include "drivers/ModifiedMagneticSensorMT6701SSI.h"
#include "drivers/Mcp3204.h"

/*
    Hardware.cpp

    This file implements the real hardware subsystem for the haptic knob.

    It creates the actual hardware objects used by the system:
      - SPI bus
      - MT6701 encoder
      - current sense
      - 6-PWM BLDC driver
      - BLDC motor object

    It also performs all bring-up and configuration needed to make the
    motor-control stack run:
      - SPI / encoder initialization
      - driver initialization
      - current-sense initialization and alignment
      - motor setup
      - FOC initialization

    At runtime, the MotorControlTask will call into this file to:
      - update the encoder / FOC control step
      - read measured motor state
      - apply current or voltage commands

    Important:
    This file should NOT contain resistor/capacitor/inductor/diode equations.
    Those belong in HapticModels.cpp.
*/

static bool setupCurrentSense();
static bool setupDriver();
static bool setupExternalAdc();
static bool setupMotor();

static SpiBus spiBus(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI);
static ModifiedMagneticSensorMT6701SSI encoder(PIN_ENC_CS);
static InlineCurrentSense current_sense(SHUNT_RESISTOR_OHM, CURRENT_SENSE_AMP_GAIN, PIN_I_A, PIN_I_B, PIN_I_C);
static BLDCDriver6PWM driver(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL);
static BLDCMotor motor(POLE_PAIRS);

bool initHardware()
{
    // TODO:
    // 1. init SPI bus
    // 2. init encoder
    // 3. init external ADC
    // 4. init BLDC driver
    // 5. init motor object
    // 6. init FOC
    Serial.println("Starting hardware initialization...");

    spiBus.begin();
    encoder.init(&spiBus);

    if (!setupExternalAdc())
        return false;

    if (!setupDriver())
        return false;

    if (!setupCurrentSense())
        return false;

    if (!setupMotor())
        return false;

    Serial.println("Hardware initialization complete.");
    return true;
}

// Current Sense Setup
static bool setupCurrentSense()
{
    current_sense.linkDriver(&driver);

    Serial.print("Initializing current sense  ");
    if (!current_sense.init())
    {
        Serial.println("FAILED");
        return false;
    }

    Serial.print("Aligning current sense w driver  ");
    if (!current_sense.driverAlign(DRIVER_VOLTAGE_LIMIT_V))
    {
        Serial.println("FAILED");
        return false;
    }

    Serial.println("SUCCESSFUL current sense setup");
    return true;
}

void updateHardwareControlStep()
{
    // TODO:
    // - update encoder
    // - optionally sample external ADC / current sensing
    // - run motor.loopFOC()
    encoder.update();
    motor.loopFOC();
}

void applyMotorCurrent(float iq_cmd)
{
    // TODO: send q-axis current command
    motor.move(iq_cmd);
}

void stopMotor()
{
    // TODO: send zero output
    motor.move(0.0f);
}

float getMotorAngleRad()
{
    // TODO: return encoder angle in radians
    return encoder.getAngle();
}

float getMotorAngleDegWrapped()
{
    // TODO: return wrapped angle in degrees
    return encoder.angleDegWrapped();
}

float getMotorVelocityRad()
{
    // TODO: return shaft velocity
    return encoder.getVelocity();
}

float getMeasuredIq()
{
    // TODO: return measured q-axis current
    DQCurrent_s dq = current_sense.getFOCCurrentsDQ(encoder.getAngle());
    return dq.q;
}

PhaseCurrent_s getPhaseCurrents()
{
    // TODO: if using external ADC, return converted phase currents here
    return current_sense.getPhaseCurrents();
}

void setCurrentPidGains(float qp, float qi, float qd,
                        float dp, float di, float dd)
{
    // TODO: update motor PID_current_q / PID_current_d values
    motor.PID_current_q.P = qp;
    motor.PID_current_q.I = qi;
    motor.PID_current_q.D = qd;

    motor.PID_current_d.P = dp;
    motor.PID_current_d.I = di;
    motor.PID_current_d.D = dd;
}

static bool setupDriver()
{
    // TODO: set PWM frequency, dead zone, voltage supply, voltage limit
    // TODO: init driver
    driver.pwm_frequency = 30e3;
    driver.dead_zone = 0.05f;
    driver.voltage_power_supply = VOLTAGE_SUPPLY_V;
    driver.voltage_limit = DRIVER_VOLTAGE_LIMIT_V;

    Serial.print("Initializing motor driver");
    if (!driver.init())
    {
        Serial.println("FAILED");
        return false;
    }

    driver.enable();
    Serial.println("SUCCESSFUL driver setup");

    return true;
}

static bool setupExternalAdc()
{
    // TODO: init MCP3204 and any current-sense calibration layer
    return true;
}

static bool setupMotor()
{
    // TODO:
    // - link driver
    motor.linkDriver(&driver);
    // - link encoder
    motor.linkSensor(&encoder);
    // -link currentSense
    motor.linkCurrentSense(&current_sense);
    // - configure torque control mode
    motor.controller = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::foc_current;
    // - configure limits
    motor.voltage_limit = DRIVER_VOLTAGE_LIMIT_V;
    motor.current_limit = DRIVER_CURRENT_LIMIT_A;

    // default PID gains
    motor.PID_current_q.P = 10.0f;
    motor.PID_current_q.I = 150.0f;
    motor.PID_current_q.D = 0.0001f;
    motor.PID_current_q.limit = DRIVER_CURRENT_LIMIT_A;

    motor.PID_current_d.P = 10.0f;
    motor.PID_current_d.I = 150.0f;
    motor.PID_current_d.D = 0.0001f;
    motor.PID_current_d.limit = DRIVER_CURRENT_LIMIT_A;

    // filter measured current noise a bit
    motor.LPF_current_q.Tf = 0.002f;
    motor.LPF_current_d.Tf = 0.002f;
    // - init motor
    Serial.print("Initializing motor object  ");
    if (!motor.init())
    {
        Serial.println("FAILED");
        return false;
    }
    // - initFOC
    Serial.print("Initializing FOC  ");
    if (!motor.initFOC())
    {
        Serial.println("FAILED");
        return false;
    }

    Serial.println("SUCCESSFUL Motor & FOC initialization");
    return true;
}

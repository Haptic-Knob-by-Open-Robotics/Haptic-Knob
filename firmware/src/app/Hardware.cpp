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


static bool setupDriver();
static bool setupExternalAdc();
static bool setupMotor();

bool initHardware()
{
    // TODO:
    // 1. init SPI bus
    // 2. init encoder
    // 3. init external ADC
    // 4. init BLDC driver
    // 5. init motor object
    // 6. init FOC

    SpiBus spiBus(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI);
    ModifiedMagneticSensorMT6701SSI encoder(PIN_ENC_CS);
    InlineCurrentSense current_sense(SHUNT_RESISTOR, AMP_GAIN, PIN_I_A, PIN_I_B, PIN_I_C);
    BLDCDriver6PWM driver(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL);
    BLDCMotor motor(POLE_PAIRS);

    return true;
}
// Driver Setup
bool driverSetup()
{
    driver.pwm_frequency = 30e3;
    driver.dead_zone = 0.05f;
    driver.voltage_power_supply = VOLTAGE_SUPPLY;
    driver.voltage_limit = VOLTAGE_LIMIT;

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


// Current Sense Setup
bool currentSenseSetup()
{
    current_sense.linkDriver(&driver);

    Serial.print("Initializing current sense  ");
    if (!current_sense.init())
    {
        Serial.println("FAILED");
        return false;
    }

    Serial.print("Aligning current sense w driver  ");
    if (!current_sense.driverAlign(VOLTAGE_LIMIT))
    {
        Serial.println("FAILED");
        return false;
    }

    Serial.println("SUCCESSFUL current sense setup");
    return true;
}

// Motor & FOC Setup
bool motorSetup()
{
    motor.linkDriver(&driver);
    motor.linkSensor(&encoder);
    motor.linkCurrentSense(&current_sense);

    motor.controller = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::foc_current;

    motor.voltage_limit = VOLTAGE_LIMIT;
    motor.current_limit = MAX_CURRENT;

    float PID_Constants[6];
    askUserForPIDGains(PID_Constants);

    motor.PID_current_q.P = PID_Constants[0];
    motor.PID_current_q.I = PID_Constants[1];
    motor.PID_current_q.D = PID_Constants[2];
    motor.PID_current_q.limit = VOLTAGE_LIMIT;

    motor.PID_current_d.P = PID_Constants[3];
    motor.PID_current_d.I = PID_Constants[4];
    motor.PID_current_d.D = PID_Constants[5];
    motor.PID_current_d.limit = VOLTAGE_LIMIT;

    motor.LPF_current_q.Tf = 0.05f;
    motor.LPF_current_d.Tf = 0.05f;

    Serial.print("Initializing motor object  ");
    if (!motor.init())
    {
        Serial.println("FAILED");
        return false;
    }

    Serial.print("Initializing FOC  ");
    if (!motor.initFOC())
    {
        Serial.println("FAILED");
        return false;
    }

    Serial.println("SUCCESSFUL Motor & FOC initialization");
    return true;
}

void updateHardwareControlStep()
{
    // TODO:
    // - update encoder
    // - optionally sample external ADC / current sensing
    // - run motor.loopFOC()
}

void applyMotorCurrent(float iq_cmd)
{
    // TODO: send q-axis current command
}

void applyMotorVoltage(float v_cmd)
{
    // TODO: send voltage command
}

void stopMotor()
{
    // TODO: send zero output
}

float getMotorAngleRad()
{
    // TODO: return encoder angle in radians
    return 0.0f;
}

float getMotorAngleDegWrapped()
{
    // TODO: return wrapped angle in degrees
    return 0.0f;
}

float getMotorVelocityRad()
{
    // TODO: return shaft velocity
    return 0.0f;
}

float getMeasuredIq()
{
    // TODO: return measured q-axis current
    return 0.0f;
}

PhaseCurrent_s getPhaseCurrents()
{
    // TODO: if using external ADC, return converted phase currents here
    PhaseCurrent_s currents{};
    return currents;
}

void setCurrentPidGains(float qp, float qi, float qd,
                        float dp, float di, float dd)
{
    // TODO: update motor PID_current_q / PID_current_d values
}

static bool setupDriver()
{
    // TODO: set PWM frequency, dead zone, voltage supply, voltage limit
    // TODO: init driver
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
    // - link encoder
    // - configure torque control mode
    // - configure limits
    // - init motor
    // - initFOC
    return true;
}

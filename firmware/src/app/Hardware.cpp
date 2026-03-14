
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

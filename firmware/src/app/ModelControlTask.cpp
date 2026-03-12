#include "ModelControlTask.h"

#include <Arduino.h>
#include <SimpleFOC.h>
#include "control/MotorSystem.h"
#include "app/SharedState.h"

// Want motor control and model calculations to be seperated
// This file orchestrates the model calculations

// Constants
static constexpr float TORQUE_CONST = 0.035f;   // N*m/A
static constexpr float MAX_TORQUE   = 0.12f;
static constexpr float MAX_CURRENT  = 2.0f;

// Mode Selection
enum HapticMode
{
    MODE_RESISTOR,
    MODE_CAPACITOR,
    MODE_INDUCTOR,
    MODE_RLC
};

static HapticMode activeMode = MODE_RESISTOR;
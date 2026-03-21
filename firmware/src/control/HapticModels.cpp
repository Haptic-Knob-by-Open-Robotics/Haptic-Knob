#include "control/HapticModels.h"
#include <Arduino.h>

/*
    This file implements the virtual haptic behaviors for the knob.

    Each model converts measured shaft motion into a motor command:

      - Resistor:
          torque opposes velocity

      - Capacitor:
          torque behaves like a virtual spring/damper relative to an origin

      - Inductor:
          torque opposes acceleration / changes in motion

      - Diode:
          motion is allowed more easily in one direction and resisted in the other

    This layer should only do math and command generation.
    It should not call encoder.update(), motor.loopFOC(), or motor.move().

    In other words:
      - Hardware.cpp handles the real motor hardware
      - HapticModels.cpp decides what command should be sent
*/



/*
    Do NOT:
    - call encoder.update()
    - call motor.loopFOC()
    - call motor.move()
*/

static float clampf(float val, float minVal, float maxVal)
{
    if (val < minVal) return minVal;
    if (val > maxVal) return maxVal;
    return val;
}

void computeResistorCommand(const MeasuredState& measured,
                            const RuntimeConfig& config,
                            HapticCommand& command)
{
    // TODO:
    // 1. compute torque opposing velocity
    // 2. convert torque -> iq_cmd
    // 3. clamp current
    // 4. set use_voltage_mode = false
}

void computeCapacitorCommand(const MeasuredState& measured,
                             const RuntimeConfig& config,
                             HapticCommand& command)
{
    // TODO:
    // 1. compute displacement from theta_origin
    // 2. compute spring/damper torque
    // 3. convert torque -> iq_cmd
    // 4. clamp current
    // 5. set use_voltage_mode = false

    if (fabsf(omega) < 0.15f)
        omega = 0.0f;
    // virtual capacitor model
    float displacement = theta - theta_origin;

    float torqueCmd = -K_virtual * displacement - B_virtual * omega;
    torqueCmd = clampf(torqueCmd, -MAX_TORQUE, MAX_TORQUE);

    float iqCmd = clampf(torqueCmd / TORQUE_CONST, -MAX_CURRENT, MAX_CURRENT);
}

void computeInductorCommand(const MeasuredState& measured,
                            const RuntimeConfig& config,
                            HapticCommand& command)
{
    // TODO:
    // 1. use measured acceleration
    // 2. compute inertial / inductive torque
    // 3. convert torque -> iq_cmd
    // 4. clamp current
    // 5. set use_voltage_mode = false
}

void computeDiodeCommand(const MeasuredState& measured,
                         const RuntimeConfig& config,
                         HapticCommand& command)
{
    // TODO:
    // 1. apply asymmetric direction logic
    // 2. compute voltage command
    // 3. clamp voltage
    // 4. set use_voltage_mode = true
}

void computeActiveModelCommand(const MeasuredState& measured,
                               const RuntimeConfig& config,
                               HapticCommand& command)
{
    // TODO:
    // switch(config.active_mode)
    // {
    //   case HapticMode::Resistor:  ...
    //   case HapticMode::Capacitor: ...
    //   case HapticMode::Inductor:  ...
    //   case HapticMode::Diode:     ...
    // }
}

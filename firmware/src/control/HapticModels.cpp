#include "control/HapticModels.h"
#include "app/SharedState.h"
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

static constexpr float TORQUE_CONST = 0.035f;   // N*m/A
static constexpr float MAX_TORQUE   = 0.12f;
static constexpr float MAX_CURRENT  = 2.0f;


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
    float torqueCmd = -config.resistance_gain * measured.velocity_rad_s;
    torqueCmd = clampf(torqueCmd, -MAX_TORQUE, MAX_TORQUE);
    // 2. convert torque -> iq_cmd
    float iqCmd = torqueCmd / TORQUE_CONST;
    // 3. clamp current
    iqCmd = clampf(iqCmd, -MAX_CURRENT, MAX_CURRENT);
    // 4. set use_voltage_mode = false
    command.use_voltage_mode = false;
    command.iq_cmd = iqCmd;
    command.v_cmd = 0.0f;
    command.last_update_us = micros();
}

// Capacitor model parameters
static float K_virtual = 0.6f;   // stiffness
static float B_virtual = 0.03f;  // damping

static float theta_origin = 0.0f; // unwrapped equilibrium position

void computeCapacitorCommand(const MeasuredState& measured,
                             const RuntimeConfig& config,
                             HapticCommand& command)
{
    // TODO:
    // 1. compute displacement from theta_origin
    // 2. compute spring/damper torque
    // 3. convert torque -> iq_cmd
    // 4. clamp current

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
    float alpha = measured.acceleration_rad_s2;
    float omega = measured.velocity_rad_s;
    // Deadband
    if (fabsf(alpha) < ALPHA_DEADBAND)
    {
        alpha = 0.0f;
    }

    if (fabsf(omega) < OMEGA_DEADBAND)
    {
        omega = 0.0f;
    }
    // 2. compute inertial / inductive torque
    float torqueCmd = -config.virtual_inductance *alpha - INDUCTOR_DAMPING * omega;
    torqueCmd = clampf(torqueCmd, -MAX_TORQUE, MAX_TORQUE);
    // 3. convert torque -> iq_cmd
    float iqCmd = torqueCmd / TORQUE_CONST;
    // 4. clamp current
    iqCmd = clampf(iqCmd, -MAX_CURRENT, MAX_CURRENT);

    if (fabsf(iqCmd) < IQ_DEADBAND)
    {
        iqCmd = 0.0f;
    }
    // 5. set use_voltage_mode = false
    command.use_voltage_mode = false;
    command.iq_cmd = iqCmd;
    command.v_cmd = 0.0f;
    command.last_update_us = micros();

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

void computeRLCCommand(const MeasuredState& measured,
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
    switch(config.active_mode)
    {
        case HapticMode::Resistor:
            computeResistorCommand(measured, config, command);
            break;
        case HapticMode::Capacitor:
            computeCapacitorCommand(measured, config, command);
            break;
        case HapticMode::Inductor:
            computeInductorCommand(measured, config, command);
            break;
        case HapticMode::Diode:
            computeDiodeCommand(measured, config, command);
            break;
        case HapticMode::RLC:
            computeRLCCommand(measured, config, command);
            break;
        default:
            break;
    }
}

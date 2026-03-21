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
    torqueCmd = clampf(torqueCmd, -config.MAX_TORQUE, config.MAX_TORQUE);
    // 2. convert torque -> iq_cmd
    float iqCmd = torqueCmd / config.TORQUE_CONST;
    // 3. clamp current
    iqCmd = clampf(iqCmd, -config.MAX_CURRENT, config.MAX_CURRENT);
    command.iq_cmd = iqCmd;
    command.last_update_us = micros();
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

    // Read measured state
    float theta = measured.angle_rad;
    float omega = measured.velocity_rad_s;

    // Velocity deadband
    if (fabsf(omega) < 0.15f)
    {
        omega = 0.0f;
    }

    // Compute displacement from virtual equilibrium
    float displacement = theta - config.theta_origin;

    // Spring-damper torque
    float torqueCmd = -config.k_virtual * displacement
                      -config.b_virtual * omega;

    // Clamp torque to actuator limit
    torqueCmd = clampf(torqueCmd, -config.MAX_TORQUE, config.MAX_TORQUE);

    // Convert torque to q-axis current
    float iqCmd = torqueCmd / config.TORQUE_CONST;

    // Clamp current
    iqCmd = clampf(iqCmd, -config.MAX_CURRENT, config.MAX_CURRENT);

    command.iq_cmd = iqCmd;
    command.last_update_us = micros();
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
    if (fabsf(alpha) < config.ALPHA_DEADBAND)
    {
        alpha = 0.0f;
    }

    if (fabsf(omega) < config.OMEGA_DEADBAND)
    {
        omega = 0.0f;
    }
    // 2. compute inertial / inductive torque
    float torqueCmd = -config.virtual_inductance *alpha - config.INDUCTOR_DAMPING * omega;
    torqueCmd = clampf(torqueCmd, -config.MAX_TORQUE, config.MAX_TORQUE);
    // 3. convert torque -> iq_cmd
    float iqCmd = torqueCmd / config.TORQUE_CONST;
    // 4. clamp current
    iqCmd = clampf(iqCmd, -config.MAX_CURRENT, config.MAX_CURRENT);

    if (fabsf(iqCmd) < config.IQ_DEADBAND)
    {
        iqCmd = 0.0f;
    }
    command.iq_cmd = iqCmd;
    command.last_update_us = micros();

}

void computeDiodeCommand(const MeasuredState& measured,
                         const RuntimeConfig& config,
                         HapticCommand& command)
{
    // TODO:
    // 1. apply asymmetric direction logic
    float torqueCmd = 0.0f;

    if (measured.velocity_rad_s > config.diode_threshold)
    {
        torqueCmd = -config.diode_gain * measured.velocity_rad_s;
    }
    // 2. compue voltage command
    float iqCmd = torqueCmd / config.TORQUE_CONST;

    // 3. clamp c
    iqCmd = clampf(iqCmd, -config.MAX_CURRENT, config.MAX_CURRENT);
    command.iq_cmd = iqCmd;
    command.last_update_us = micros();
}

void computeRLCCommand(const MeasuredState& measured,
                         const RuntimeConfig& config,
                         HapticCommand& command)
{
    // TODO:
    // 1. apply asymmetric direction logic
    // 2. compute voltage command
    // 3. clamp voltage

    // Virtual circuit states
    static float i_virtual  = 0.0f;   // virtual circuit current
    static float vc_virtual = 0.0f;   // virtual capacitor voltage
    static uint32_t lastModelUpdateUs = 0;

    // Compute dt from measured timestamp
    uint32_t nowUs = measured.last_update_us;
    float dt = 0.001f; //  default

    if (lastModelUpdateUs != 0 && nowUs > lastModelUpdateUs)
    {
        dt = (nowUs - lastModelUpdateUs) * 1e-6f;
    }
    lastModelUpdateUs = nowUs;

    // Guard dt against bad jumps
    dt = clampf(dt, 0.0001f, 0.005f);

    // Read velocity input
    float omega = measured.velocity_rad_s;
    if (fabsf(omega) < config.OMEGA_DEADBAND)
    {
        omega = 0.0f;
    }

    // Map shaft velocity to virtual source voltage
    float vin = config.INPUT_GAIN * omega;

    // Read virtual RLC parameters from config
    float R = config.virtual_resistance;     
    float L = config.virtual_inductance;   
    float C = config.virtual_capacitance;

    // Prevent divide-by-zero / unstable params
    R = fmaxf(R, 0.0001f);
    L = fmaxf(L, 0.0001f);

    //    Virtual series RLC state equations
    //    di/dt  = (vin - R*i - vc) / L
    //    dvc/dt = i / C
    float di_dt  = (vin - R * i_virtual - vc_virtual) / L;
    float dvc_dt = i_virtual / C;

    i_virtual  += di_dt * dt;
    vc_virtual += dvc_dt * dt;

    // Clamp virtual states for robustness
    i_virtual  = clampf(i_virtual,  -config.MAX_STATE_ABS, config.MAX_STATE_ABS);
    vc_virtual = clampf(vc_virtual, -config.MAX_STATE_ABS, config.MAX_STATE_ABS);

    // Map virtual current to motor torque
    float torqueCmd = -config.TORQUE_GAIN * i_virtual;
    torqueCmd = clampf(torqueCmd, -config.MAX_TORQUE, config.MAX_TORQUE);

    // Convert torque to q-axis current
    float iqCmd = torqueCmd / config.TORQUE_CONST;
    iqCmd = clampf(iqCmd, -config.MAX_CURRENT, config.MAX_CURRENT);

    // Fill output command
    command.iq_cmd = iqCmd;
    command.last_update_us = micros();
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

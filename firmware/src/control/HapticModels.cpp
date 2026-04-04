#include "control/HapticModels.h"

#include <Arduino.h>
#include <SimpleFOC.h>

#include <cmath>

#include "app/Config.h"

LowPassFilter velocityFilter(0.03f);

namespace
{
    float clampf(float value, float minValue, float maxValue)
    {
        if (value < minValue)
        {
            return minValue;
        }
        if (value > maxValue)
        {
            return maxValue;
        }
        return value;
    }
}

void computeResistorCommand(const MeasuredState &measured,
                            const RuntimeConfig &config,
                            HapticCommand &command)
{
    float torqueCmd = -config.resistance_gain * measured.velocity_rad_s;
    torqueCmd = clampf(torqueCmd, -MAX_TORQUE, MAX_TORQUE);

    float iqCmd = torqueCmd / TORQUE_CONST;
    iqCmd = clampf(iqCmd, -MAX_CURRENT, MAX_CURRENT);

    command.iq_cmd = iqCmd;
    command.last_update_us = micros();
}

void computeCapacitorCommand(const MeasuredState &measured,
                             const RuntimeConfig &config,
                             HapticCommand &command)
{
    static float filteredOmega = 0.0f;
    float theta = measured.angle_rad;
    float omega = velocityFilter(measured.velocity_rad_s);

    if (fabsf(omega) < 0.15f)
    {
        omega = 0.0f;
    }

    const float displacement = theta - config.theta_origin;
    float torqueCmd = -config.k_virtual * displacement - config.b_virtual * omega;
    torqueCmd = clampf(torqueCmd, -MAX_TORQUE, MAX_TORQUE);

    float iqCmd = torqueCmd / TORQUE_CONST;
    iqCmd = clampf(iqCmd, -MAX_CURRENT, MAX_CURRENT);

    command.iq_cmd = iqCmd;
    command.last_update_us = micros();
}

void computeInductorCommand(const MeasuredState &measured,
                            const RuntimeConfig &config,
                            HapticCommand &command)
{
    float alpha = measured.acceleration_rad_s2;
    float omega = measured.velocity_rad_s;

    if (fabsf(alpha) < config.alpha_deadband)
    {
        alpha = 0.0f;
    }

    if (fabsf(omega) < config.omega_deadband)
    {
        omega = 0.0f;
    }

    float torqueCmd = -config.virtual_inductance * alpha - config.inductor_damping * omega;
    torqueCmd = clampf(torqueCmd, -MAX_TORQUE, MAX_TORQUE);

    float iqCmd = torqueCmd / TORQUE_CONST;
    iqCmd = clampf(iqCmd, -MAX_CURRENT, MAX_CURRENT);

    if (fabsf(iqCmd) < config.iq_deadband)
    {
        iqCmd = 0.0f;
    }

    command.iq_cmd = iqCmd;
    command.last_update_us = micros();
}

void computeDiodeCommand(const MeasuredState &measured,
                         const RuntimeConfig &config,
                         HapticCommand &command)
{
    float torqueCmd = 0.0f;

    if (measured.velocity_rad_s > config.diode_threshold)
    {
        torqueCmd = -config.diode_gain * measured.velocity_rad_s;
    }

    torqueCmd = clampf(torqueCmd, -MAX_TORQUE, MAX_TORQUE);

    float iqCmd = torqueCmd / TORQUE_CONST;
    iqCmd = clampf(iqCmd, -MAX_CURRENT, MAX_CURRENT);

    command.iq_cmd = iqCmd;
    command.last_update_us = micros();
}

void computeRLCCommand(const MeasuredState &measured,
                       const RuntimeConfig &config,
                       HapticCommand &command)
{
    static float iVirtual = 0.0f;
    static float vcVirtual = 0.0f;
    static uint32_t lastModelUpdateUs = 0;

    const uint32_t nowUs = measured.last_update_us;
    float dt = 0.001f;

    if (lastModelUpdateUs != 0 && nowUs > lastModelUpdateUs)
    {
        dt = (nowUs - lastModelUpdateUs) * 1e-6f;
    }
    lastModelUpdateUs = nowUs;

    dt = clampf(dt, 0.0001f, 0.005f);

    float omega = measured.velocity_rad_s;
    if (fabsf(omega) < config.omega_deadband)
    {
        omega = 0.0f;
    }

    const float vin = config.input_gain * omega;

    float R = config.virtual_resistance;
    float L = config.virtual_inductance;
    float C = config.virtual_capacitance;

    R = fmaxf(R, 0.0001f);
    L = fmaxf(L, 0.0001f);
    C = fmaxf(C, 0.0001f);

    const float diDt = (vin - R * iVirtual - vcVirtual) / L;
    const float dvcDt = iVirtual / C;

    iVirtual += diDt * dt;
    vcVirtual += dvcDt * dt;

    iVirtual = clampf(iVirtual, -config.max_state_abs, config.max_state_abs);
    vcVirtual = clampf(vcVirtual, -config.max_state_abs, config.max_state_abs);

    float torqueCmd = -config.torque_gain * iVirtual;
    torqueCmd = clampf(torqueCmd, -MAX_TORQUE, MAX_TORQUE);

    float iqCmd = torqueCmd / TORQUE_CONST;
    iqCmd = clampf(iqCmd, -MAX_CURRENT, MAX_CURRENT);

    command.iq_cmd = iqCmd;
    command.last_update_us = micros();
}

void computeActiveModelCommand(const MeasuredState &measured,
                               const RuntimeConfig &config,
                               HapticCommand &command)
{
    switch (config.active_mode)
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
        command.iq_cmd = 0.0f;
        command.last_update_us = micros();
        break;
    }
}
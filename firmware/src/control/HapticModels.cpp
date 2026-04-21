#include "control/HapticModels.h"

#include <Arduino.h>
#include <SimpleFOC.h>

#include <cmath>

#include "app/Config.h"

namespace
{
    constexpr float VELOCITY_FILTER_TF_S = 0.03f;
    constexpr float ACCEL_FILTER_TF_S = 0.015f;
    constexpr float DEFAULT_VELOCITY_DEADBAND = 0.15f;
    constexpr float DEFAULT_ACCEL_DEADBAND = 8.0f;
    constexpr float DEFAULT_CURRENT_DEADBAND = 0.03f;
    constexpr float DEFAULT_INDUCTOR_DAMPING = 0.02f;
    constexpr float RLC_IDLE_DECAY_PER_UPDATE = 0.98f;

    struct ModelRuntimeState
    {
        HapticMode activeMode = HapticMode::Resistor;
        bool initialized = false;
        float filteredVelocity = 0.0f;
        float previousFilteredVelocity = 0.0f;
        float filteredAcceleration = 0.0f;
        float virtualCurrent = 0.0f;
        float virtualCapVoltage = 0.0f;
        uint32_t lastModelUpdateUs = 0;
    };

    ModelRuntimeState g_model_state;

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

    float applyLowPass(float previousValue, float sample, float dt, float timeConstant)
    {
        const float alpha = dt / (timeConstant + dt);
        return previousValue + alpha * (sample - previousValue);
    }

    float applyDeadband(float value, float threshold)
    {
        if (fabsf(value) < threshold)
        {
            return 0.0f;
        }
        return value;
    }

    float getModelDt(uint32_t measurementTimestampUs)
    {
        float dt = MODEL_CONTROL_PERIOD_MS * 1e-3f;

        if (g_model_state.initialized &&
            g_model_state.lastModelUpdateUs != 0 &&
            measurementTimestampUs > g_model_state.lastModelUpdateUs)
        {
            dt = (measurementTimestampUs - g_model_state.lastModelUpdateUs) * 1e-6f;
        }

        g_model_state.lastModelUpdateUs = measurementTimestampUs;
        return clampf(dt, 0.0001f, 0.01f);
    }

    void resetDynamicModelState(HapticMode activeMode)
    {
        g_model_state.activeMode = activeMode;
        g_model_state.filteredVelocity = 0.0f;
        g_model_state.previousFilteredVelocity = 0.0f;
        g_model_state.filteredAcceleration = 0.0f;
        g_model_state.virtualCurrent = 0.0f;
        g_model_state.virtualCapVoltage = 0.0f;
        g_model_state.lastModelUpdateUs = 0;
        g_model_state.initialized = true;
    }

    void syncModelState(const MeasuredState &measured, HapticMode activeMode)
    {
        if (!g_model_state.initialized || g_model_state.activeMode != activeMode)
        {
            resetDynamicModelState(activeMode);
            g_model_state.filteredVelocity = measured.velocity_rad_s;
            g_model_state.previousFilteredVelocity = measured.velocity_rad_s;
        }
    }

    float getFilteredVelocity(const MeasuredState &measured,
                              const RuntimeConfig &config,
                              float dt)
    {
        g_model_state.filteredVelocity = applyLowPass(
            g_model_state.filteredVelocity,
            measured.velocity_rad_s,
            dt,
            VELOCITY_FILTER_TF_S);

        const float velocityDeadband =
            config.omega_deadband > 0.0f ? config.omega_deadband : DEFAULT_VELOCITY_DEADBAND;
        return applyDeadband(g_model_state.filteredVelocity, velocityDeadband);
    }

    float getFilteredAcceleration(float filteredVelocity,
                                  const RuntimeConfig &config,
                                  float dt)
    {
        const float rawAcceleration =
            (filteredVelocity - g_model_state.previousFilteredVelocity) / dt;
        g_model_state.previousFilteredVelocity = filteredVelocity;

        g_model_state.filteredAcceleration = applyLowPass(
            g_model_state.filteredAcceleration,
            rawAcceleration,
            dt,
            ACCEL_FILTER_TF_S);

        const float accelerationDeadband =
            config.alpha_deadband > 0.0f ? config.alpha_deadband : DEFAULT_ACCEL_DEADBAND;
        return applyDeadband(g_model_state.filteredAcceleration, accelerationDeadband);
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
    syncModelState(measured, HapticMode::Capacitor);
    const float dt = getModelDt(measured.last_update_us);
    float theta = measured.angle_rad;
    float omega = getFilteredVelocity(measured, config, dt);

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
    syncModelState(measured, HapticMode::Inductor);
    const float dt = getModelDt(measured.last_update_us);
    const float omega = getFilteredVelocity(measured, config, dt);
    const float alpha = getFilteredAcceleration(omega, config, dt);
    const float damping =
        config.inductor_damping > 0.0f ? config.inductor_damping : DEFAULT_INDUCTOR_DAMPING;

    float torqueCmd = -config.virtual_inductance * alpha - damping * omega;
    torqueCmd = clampf(torqueCmd, -MAX_TORQUE, MAX_TORQUE);

    float iqCmd = torqueCmd / TORQUE_CONST;
    iqCmd = clampf(iqCmd, -MAX_CURRENT, MAX_CURRENT);

    const float currentDeadband =
        config.iq_deadband > 0.0f ? config.iq_deadband : DEFAULT_CURRENT_DEADBAND;
    if (fabsf(iqCmd) < currentDeadband)
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
    syncModelState(measured, HapticMode::Diode);
    const float dt = getModelDt(measured.last_update_us);
    const float omega = getFilteredVelocity(measured, config, dt);
    const float absOmega = fabsf(omega);
    float torqueCmd = 0.0f;

    if (absOmega > config.diode_threshold)
    {
        const float conductionVelocity = absOmega - config.diode_threshold;
        torqueCmd = -config.diode_gain * conductionVelocity * (omega > 0.0f ? 1.0f : -1.0f);
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
    syncModelState(measured, HapticMode::RLC);
    const float dt = getModelDt(measured.last_update_us);
    const float omega = getFilteredVelocity(measured, config, dt);

    const float vin = config.input_gain * omega;

    float R = config.virtual_resistance;
    float L = config.virtual_inductance;
    float C = config.virtual_capacitance;

    R = fmaxf(R, 0.0001f);
    L = fmaxf(L, 0.0001f);
    C = fmaxf(C, 0.0001f);

    const float diDt =
        (vin - R * g_model_state.virtualCurrent - g_model_state.virtualCapVoltage) / L;
    const float dvcDt = g_model_state.virtualCurrent / C;

    g_model_state.virtualCurrent += diDt * dt;
    g_model_state.virtualCapVoltage += dvcDt * dt;

    if (omega == 0.0f)
    {
        g_model_state.virtualCurrent *= RLC_IDLE_DECAY_PER_UPDATE;
        g_model_state.virtualCapVoltage *= RLC_IDLE_DECAY_PER_UPDATE;
    }

    g_model_state.virtualCurrent = clampf(
        g_model_state.virtualCurrent, -config.max_state_abs, config.max_state_abs);
    g_model_state.virtualCapVoltage = clampf(
        g_model_state.virtualCapVoltage, -config.max_state_abs, config.max_state_abs);

    float torqueCmd = -config.torque_gain * g_model_state.virtualCurrent;
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

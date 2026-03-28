#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "app/Faults.h"

enum class HapticMode : uint8_t
{
    Resistor = 0,
    Capacitor,
    Inductor,
    Diode,
    RLC
};

struct MeasuredState
{
    float angle_rad = 0.0f;
    float angle_deg_wrapped = 0.0f;
    float velocity_rad_s = 0.0f;
    float acceleration_rad_s2 = 0.0f;

    float iq_meas = 0.0f;
    float ia = 0.0f;
    float ib = 0.0f;
    float ic = 0.0f;

    uint32_t last_update_us = 0;
};

struct HapticCommand
{
    float iq_cmd = 0.0f;
    uint32_t last_update_us = 0;
};

struct PIDGains
{
    float P = 0.0f;
    float I = 0.0f;
    float D = 0.0f;
};

struct CurrentLoopPID
{
    PIDGains q;
    PIDGains d;
};

struct RuntimeConfig
{
    HapticMode active_mode = HapticMode::Resistor;

    CurrentLoopPID resistor_pid = {
        {10.0f, 150.0f, 0.0f},
        {10.0f, 150.0f, 0.0f}};

    CurrentLoopPID capacitor_pid = {
        {3.0f, 300.0f, 0.0f},
        {3.0f, 300.0f, 0.0f}};

    CurrentLoopPID inductor_pid = {
        {3.0f, 300.0f, 0.0f},
        {3.0f, 300.0f, 0.0f}};

    CurrentLoopPID diode_pid = {
        {3.0f, 300.0f, 0.0f},
        {3.0f, 300.0f, 0.0f}};

    CurrentLoopPID rlc_pid = {
        {3.0f, 300.0f, 0.0f},
        {3.0f, 300.0f, 0.0f}};

    float resistance_gain = 0.001f;

    float k_virtual = 0.6f;
    float b_virtual = 0.03f;
    float theta_origin = 0.0f;

    float virtual_inductance = 0.020f;
    float alpha_deadband = 0.0f;
    float omega_deadband = 0.0f;
    float iq_deadband = 0.0f;
    float inductor_damping = 0.0f;

    float diode_threshold = 0.1f;
    float diode_gain = 2.0f;

    float max_state_abs = 20.0f;
    float input_gain = 1.0f;
    float torque_gain = 0.020f;

    float virtual_resistance = 0.5f;
    float virtual_capacitance = 0.30f;
};

struct SystemState
{
    bool control_enabled = false;
    bool trial_configured = false;
    bool trial_active = false;

    bool fault_latched = false;
    uint32_t fault_bits = 0;

    uint32_t control_last_heartbeat_us = 0;
    uint32_t telemetry_last_heartbeat_us = 0;
};

extern MeasuredState g_measured_state;
extern HapticCommand g_haptic_command;
extern RuntimeConfig g_runtime_config;
extern SystemState g_system_state;
extern SemaphoreHandle_t g_state_mutex;

bool initSharedState();

bool readMeasuredState(MeasuredState &out);
bool writeMeasuredState(const MeasuredState &in);

bool readHapticCommand(HapticCommand &out);
bool writeHapticCommand(const HapticCommand &in);

bool readRuntimeConfig(RuntimeConfig &out);
bool writeRuntimeConfig(const RuntimeConfig &in);

bool readSystemState(SystemState &out);
bool writeSystemState(const SystemState &in);

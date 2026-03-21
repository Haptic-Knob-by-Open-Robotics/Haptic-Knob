#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "app/Faults.h"

/*
    This file defines the shared data used by the FreeRTOS tasks.

    Since the firmware is split into multiple tasks, we need one central
    place to store:
      - the latest measured motor/shaft state
      - the latest command requested by the haptic model
      - the current configuration / active mode / gains
      - fault flags if something goes wrong

    Typical data stored here:
      - measured angle
      - measured velocity
      - measured acceleration
      - measured q-axis current / phase currents
      - desired iq command
      - desired voltage command
      - active mode (R/C/L/diode)
      - gains / thresholds / origin
      - faulted state

    Access to this data should be protected with a mutex so tasks do not
    overwrite each other.
*/


enum class HapticMode : uint8_t {
    Resistor = 0, 
    Capacitor, 
    Inductor, Diode, RLC
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

struct RuntimeConfig {
  
    HapticMode active_mode = HapticMode::Resistor;

    // Global params
    static constexpr float TORQUE_CONST = 0.035f;   // N*m/A
    static constexpr float MAX_TORQUE   = 0.12f;
    static constexpr float MAX_CURRENT  = 2.0f;
    
    // Resistor params
    float resistance_gain = 0.001f;

    // Capacitor / spring-damper params 
    float k_virtual = 0.6f;
    float b_virtual = 0.03f;
    float theta_origin = 0.0f;

    // Inductor params
    float virtual_inductance = 0.020f;
    float ALPHA_DEADBAND = 0.0f;
    float OMEGA_DEADBAND = 0.0f;
    float IQ_DEADBAND = 0.0f;
    float INDUCTOR_DAMPING = 0.0f;


    // Diode params
    float diode_threshold = 0.1f;
    float diode_gain = 2.0f;

    // RLC params
    //float OMEGA_DEADBAND = 0.15f;
    float MAX_STATE_ABS = 20.0f;
    float INPUT_GAIN = 1.0f;
    float TORQUE_GAIN = 0.020f;

    float virtual_resistance =0.5f;
    float virtual_capacitance = 0.30f;
};

struct SystemState{

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
extern HapticCommand g_haptic_command;
extern RuntimeConfig g_runtime_config; 
extern SystemState g_system_state; 
extern SemaphoreHandle_t g_state_mutex; 

void initSharedState(); 
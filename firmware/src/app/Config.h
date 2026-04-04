#pragma once
#include <cstdint>

// ============================================================
// PWM / Driver Pins
// ============================================================
constexpr int PIN_UL = 4;
constexpr int PIN_UH = 5;
constexpr int PIN_VL = 6;
constexpr int PIN_VH = 7;
constexpr int PIN_WL = 15;
constexpr int PIN_WH = 16;
constexpr int PIN_EN = 17;

// ============================================================
// Current Sense Pins
// ============================================================
constexpr int ADC_CH_U = 0;
constexpr int ADC_CH_V = 1;
constexpr int ADC_CH_W = 2;

// ============================================================
// SPI Pins
// ============================================================
// Used by External ADC
constexpr int PIN_SPI_MISO_1 = 11;
constexpr int PIN_SPI_MOSI_1 = 12;
constexpr int PIN_SPI_CLK_1 = 10;

// Used by Encoder
constexpr int PIN_SPI_MISO_2 = 42;
constexpr int PIN_SPI_MOSI_2 = -1; // Not used
constexpr int PIN_SPI_CLK_2 = 41;

// ============================================================
// Sensor Chip Select Pins
// ============================================================
constexpr int PIN_ENC_CSN = 40;
constexpr int PIN_ADC_CSN = 13;

// ============================================================
// Limits
// ============================================================
constexpr float VOLTAGE_SUPPLY_V = 5.0f;
constexpr float DRIVER_VOLTAGE_LIMIT_V = 3.0f;
constexpr float DRIVER_CURRENT_LIMIT_A = 0.8f;
constexpr float IQ_MAX_A = 2.0f;
constexpr float MAX_TORQUE = 0.12f;
constexpr float MAX_CURRENT = 2.0f;

// ============================================================
// Current Sense Constants
// ============================================================
constexpr float CURRENT_SENSE_SHUNT_RESISTOR_OHM = 0.0107f;
constexpr float CURRENT_SENSE_AMP_GAIN = 50.0f;
constexpr float CURRENT_SENSE_VREF = 3.3f;

// ============================================================
// Motor Constants
// ============================================================
constexpr int POLE_PAIRS = 7;
constexpr float DEFAULT_RESISTANCE_GAIN = 5.0f;
constexpr float TORQUE_CONST = 0.035f; // N*m/A

// ============================================================
// RTOS Periods
// ============================================================
constexpr uint32_t MOTOR_CONTROL_PERIOD_MS = 1; // 1 kHz
constexpr uint32_t MODEL_CONTROL_PERIOD_MS = 5; // 200 Hz
constexpr uint32_t TELEMETRY_PERIOD_MS = 20;    // 50 Hz
constexpr uint32_t WATCHDOG_PERIOD_MS = 50;     // 20 Hz

// ============================================================
// RTOS Stack Sizes
// ============================================================
constexpr uint32_t MOTOR_CONTROL_STACK_SIZE = 4096;
constexpr uint32_t MODEL_CONTROL_STACK_SIZE = 4096;
constexpr uint32_t TELEMETRY_STACK_SIZE = 4096;
constexpr uint32_t WATCHDOG_STACK_SIZE = 2048;

// ============================================================
// RTOS Priorities
// ============================================================
constexpr UBaseType_t MOTOR_CONTROL_PRIORITY = 5;
constexpr UBaseType_t MODEL_CONTROL_PRIORITY = 4;
constexpr UBaseType_t WATCHDOG_PRIORITY = 3;
constexpr UBaseType_t TELEMETRY_PRIORITY = 2;
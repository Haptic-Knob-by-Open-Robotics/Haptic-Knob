/*
    SharedState.cpp

    This file allocates the global shared-state structures and creates the
    synchronization primitives used by the tasks.

    In practice, this file provides:
      - the actual shared measurement/config/command objects
      - the mutex used to safely read/write them
      - read write helper functions

    This file does not contain control logic itself.
    It only provides the shared memory layer that lets tasks communicate safely.
*/
#include "SharedState.h"

SystemState g_system_state;
MeasuredState g_measured_state;
HapticCommand g_haptic_command;
RuntimeConfig g_runtime_config;
SemaphoreHandle_t g_state_mutex = nullptr;

bool initSharedState()
{
    g_state_mutex = xSemaphoreCreateMutex();
    return (g_state_mutex != nullptr);
}

bool readMeasuredState(MeasuredState &out)
{
    if (g_state_mutex == nullptr)
        return false;

    if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) != pdTRUE)
        return false;

    out = g_measured_state;

    xSemaphoreGive(g_state_mutex);
    return true;
}

bool writeMeasuredState(const MeasuredState &in)
{
    if (g_state_mutex == nullptr)
        return false;

    if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) != pdTRUE)
        return false;

    g_measured_state = in;

    xSemaphoreGive(g_state_mutex);
    return true;
}

bool readHapticCommand(HapticCommand &out)
{
    if (g_state_mutex == nullptr)
        return false;

    if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) != pdTRUE)
        return false;

    out = g_haptic_command;

    xSemaphoreGive(g_state_mutex);
    return true;
}

bool writeHapticCommand(const HapticCommand &in)
{
    if (g_state_mutex == nullptr)
        return false;

    if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) != pdTRUE)
        return false;

    g_haptic_command = in;

    xSemaphoreGive(g_state_mutex);
    return true;
}

bool readRuntimeConfig(RuntimeConfig &out)
{
    if (g_state_mutex == nullptr)
        return false;

    if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) != pdTRUE)
        return false;

    out = g_runtime_config;

    xSemaphoreGive(g_state_mutex);
    return true;
}

bool writeRuntimeConfig(const RuntimeConfig &in)
{
    if (g_state_mutex == nullptr)
        return false;

    if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) != pdTRUE)
        return false;

    g_runtime_config = in;

    xSemaphoreGive(g_state_mutex);
    return true;
}

bool readSystemState(SystemState &out)
{
    if (g_state_mutex == nullptr)
        return false;

    if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) != pdTRUE)
        return false;

    out = g_system_state;

    xSemaphoreGive(g_state_mutex);
    return true;
}

bool writeSystemState(const SystemState &in)
{
    if (g_state_mutex == nullptr)
        return false;

    if (xSemaphoreTake(g_state_mutex, portMAX_DELAY) != pdTRUE)
        return false;

    g_system_state = in;

    xSemaphoreGive(g_state_mutex);
    return true;
}
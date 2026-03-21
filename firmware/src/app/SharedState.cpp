/*
    SharedState.cpp

    This file allocates the global shared-state structures and creates the
    synchronization primitives used by the tasks.

    In practice, this file provides:
      - the actual shared measurement/config/command objects
      - the mutex used to safely read/write them

    This file does not contain control logic itself.
    It only provides the shared memory layer that lets tasks communicate safely.
*/

MeasuredState g_measured_state;
HapticCommand g_haptic_command;
RuntimeConfig g_runtime_config;

void initSharedState()
{
    // TODO: create the mutex
    // g_state_mutex = xSemaphoreCreateMutex();

    // TODO: initialize any runtime defaults if needed
}
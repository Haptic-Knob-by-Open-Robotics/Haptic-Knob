#include "app/WatchdogTask.h"

#include <Arduino.h>
#include <cmath>
#include <cstdint>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app/Config.h"
#include "app/Faults.h"
#include "app/Hardware.h"
#include "app/SharedState.h"

namespace
{
constexpr uint32_t CONTROL_HEARTBEAT_TIMEOUT_US = 100000;
constexpr uint32_t TELEMETRY_HEARTBEAT_TIMEOUT_US = 500000;
constexpr uint32_t MEASURED_STATE_TIMEOUT_US = 100000;
constexpr uint32_t COMMAND_TIMEOUT_US = 100000;

constexpr uint32_t WATCHDOG_STARTUP_GRACE_MS = 250;

TaskHandle_t g_watchdog_task_handle = nullptr;

bool isFinite(float value)
{
    return std::isfinite(value);
}

bool isCommandValid(const HapticCommand &command)
{
    return isFinite(command.iq_cmd) && (std::fabs(command.iq_cmd) <= IQ_MAX_A);
}

bool isMeasurementValid(const MeasuredState &measured)
{
    return isFinite(measured.angle_rad) &&
           isFinite(measured.angle_deg_wrapped) &&
           isFinite(measured.velocity_rad_s) &&
           isFinite(measured.acceleration_rad_s2) &&
           isFinite(measured.iq_meas) &&
           isFinite(measured.ia) &&
           isFinite(measured.ib) &&
           isFinite(measured.ic);
}

bool isTimestampStale(uint32_t nowUs, uint32_t lastUpdateUs, uint32_t timeoutUs)
{
    if (lastUpdateUs == 0)
    {
        return true;
    }

    return static_cast<uint32_t>(nowUs - lastUpdateUs) > timeoutUs;
}

uint32_t buildFaultBits(uint32_t nowUs,
                        const SystemState &systemState,
                        const MeasuredState &measured,
                        const HapticCommand &command)
{
    uint32_t faultBits = 0;

    if (isTimestampStale(nowUs, systemState.control_last_heartbeat_us, CONTROL_HEARTBEAT_TIMEOUT_US))
    {
        faultBits |= FaultBits::ControlHeartbeatStale;
    }

    if (isTimestampStale(nowUs, systemState.telemetry_last_heartbeat_us, TELEMETRY_HEARTBEAT_TIMEOUT_US))
    {
        faultBits |= FaultBits::TelemetryHeartbeatStale;
    }

    if (isTimestampStale(nowUs, measured.last_update_us, MEASURED_STATE_TIMEOUT_US))
    {
        faultBits |= FaultBits::MeasuredStateStale;
    }

    if (isTimestampStale(nowUs, command.last_update_us, COMMAND_TIMEOUT_US))
    {
        faultBits |= FaultBits::ModelCommandStale;
    }

    if (!isMeasurementValid(measured))
    {
        faultBits |= FaultBits::InvalidMeasurement;
    }

    if (!isCommandValid(command))
    {
        faultBits |= FaultBits::InvalidCommand;
    }

    if (std::fabs(measured.iq_meas) > MAX_CURRENT ||
        std::fabs(measured.ia) > MAX_CURRENT ||
        std::fabs(measured.ib) > MAX_CURRENT ||
        std::fabs(measured.ic) > MAX_CURRENT)
    {
        faultBits |= FaultBits::OverCurrent;
    }

    return faultBits;
}

void updateFaultState(uint32_t faultBits)
{
    SystemState systemState{};
    if (!readSystemState(systemState))
    {
        return;
    }

    systemState.fault_bits = faultBits;
    if (faultBits != 0)
    {
        systemState.fault_latched = true;
        systemState.control_enabled = false;
    }

    writeSystemState(systemState);
}

void printNewFaults(uint32_t previousFaultBits, uint32_t currentFaultBits)
{
    const uint32_t newFaults = currentFaultBits & ~previousFaultBits;
    if (newFaults == 0)
    {
        return;
    }

    Serial.printf("WATCHDOG fault latched: 0x%08" PRIx32 "\n", currentFaultBits);
}
} // namespace

void WatchdogTask(void *pvParameters)
{
    (void)pvParameters;

    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t periodTicks =
        pdMS_TO_TICKS(WATCHDOG_PERIOD_MS) > 0 ? pdMS_TO_TICKS(WATCHDOG_PERIOD_MS) : 1;

    vTaskDelay(pdMS_TO_TICKS(WATCHDOG_STARTUP_GRACE_MS));
    lastWakeTime = xTaskGetTickCount();

    uint32_t previousFaultBits = 0;

    for (;;)
    {
        SystemState systemState{};
        MeasuredState measured{};
        HapticCommand command{};

        const bool haveSystem = readSystemState(systemState);
        const bool haveMeasured = readMeasuredState(measured);
        const bool haveCommand = readHapticCommand(command);

        uint32_t faultBits = FaultBits::InvalidMeasurement | FaultBits::InvalidCommand;
        if (haveSystem && haveMeasured && haveCommand)
        {
            const uint32_t nowUs = micros();
            faultBits = buildFaultBits(nowUs, systemState, measured, command);
        }

        if (faultBits != 0)
        {
            stopMotor();
        }

        updateFaultState(faultBits);
        printNewFaults(previousFaultBits, faultBits);
        previousFaultBits = faultBits;

        vTaskDelayUntil(&lastWakeTime, periodTicks);
    }
}

void startWatchdogTask()
{
    const BaseType_t ok = xTaskCreatePinnedToCore(
        WatchdogTask,
        "WatchdogTask",
        WATCHDOG_STACK_SIZE,
        nullptr,
        WATCHDOG_PRIORITY,
        &g_watchdog_task_handle,
        1);

    if (ok != pdPASS)
    {
        Serial.println("Failed to create WatchdogTask");
        while (true)
        {
            delay(1000);
        }
    }
}

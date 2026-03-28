/*
    This file defines the fault states used by the firmware.

    Examples:
      - stale sensor data
      - stale model command
      - overcurrent
      - invalid command values
      - initialization failure

    Keeping fault definitions here makes it easier to expand the system safely
    as the firmware becomes more complex.
*/

#pragma once

namespace FaultBits
{
    constexpr uint32_t ControlHeartbeatStale = 1u << 0;
    constexpr uint32_t MeasuredStateStale    = 1u << 1;
    constexpr uint32_t ModelCommandStale     = 1u << 2;
    constexpr uint32_t InvalidCommand        = 1u << 3;
    constexpr uint32_t InvalidMeasurement    = 1u << 4;
    constexpr uint32_t OverCurrent           = 1u << 5;
    constexpr uint32_t TelemetryHeartbeatStale = 1u << 6;
}

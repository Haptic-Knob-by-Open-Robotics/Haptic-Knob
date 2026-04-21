#pragma once

/*
    TelemetryTask.h

    This file declares the telemetry / serial command task.

    This task is responsible for user interaction and debugging output.
*/

#include <Arduino.h>
#include "app/SharedState.h"

void TelemetryTask(void *pvParameters);
void startTelemetryTask();
static void processIncomingSerialCommands();
void handleCommandLine(String &line);
void sendTelemetryLine(MeasuredState &measured, HapticCommand &haptic, RuntimeConfig &config, SystemState &state);
void sendHelpMenu();
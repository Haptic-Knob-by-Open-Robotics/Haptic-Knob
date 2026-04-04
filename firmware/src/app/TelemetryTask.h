#pragma once

/*
    TelemetryTask.h

    This file declares the telemetry / serial command task.

    This task is responsible for user interaction and debugging output.
*/

void TelemetryTask(void *pvParameters);
void startTelemetryTask();
static void processIncomingSerialCommands();
void handleCommandLine();
void sendTelemetryLine();
void sendHelpMenu();
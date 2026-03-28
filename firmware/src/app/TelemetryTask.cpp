/*
    TelemetryTask.cpp

    This file implements the telemetry task.

    Main responsibilities:
      - read commands from Serial / Python GUI
      - parse mode changes, gain updates, PID updates, zeroing commands, etc.
      - update SharedState configuration values
      - periodically print useful telemetry for debugging and visualization

    This task should be the main place where Serial I/O happens so the
    motor-control loop stays cleaner and more deterministic.
*/

#include "app/TelemetryTask.h"
#include <Arduino.h>
#include "app/SharedState.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app/config.h"

void TelemetryTask(void *pvParameters)
{
  (void)pvParameters;
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(TELEMETRY_PERIOD_MS);

  while (1)
  {
    MeasuredState measured_state{};
    HapticCommand haptic_command{};
    RuntimeConfig runtime_config{};
    SystemState system_state{};

    readMeasuredState(measured_state);
    readHapticCommand(haptic_command);
    readRuntimeConfig(runtime_config);
    readSystemState(system_state);

    system_state.telemetry_last_heartbeat_us = micros();
    writeSystemState(system_state);
  }
}

void sendTelemetryLine(MeasuredState &measured,
                       HapticCommand &haptic,
                       RuntimeConfig &config,
                       SystemState &state)
{

  Serial.print(static_cast<uint8_t>(config.active_mode));
  Serial.print(",");

  Serial.print(measured.angle_rad);
  Serial.print(",");
  Serial.print(measured.angle_deg_wrapped);
  Serial.print(",");
  Serial.print(measured.velocity_rad_s);
  Serial.print(",");
  Serial.print(measured.acceleration_rad_s2);
  Serial.print(",");

  Serial.print(measured.iq_meas);
  Serial.print(",");
  Serial.print(measured.ia);
  Serial.print(",");
  Serial.print(measured.ib);
  Serial.print(",");
  Serial.print(measured.ic);
  Serial.print(",");
  Serial.print(measured.last_update_us);
  Serial.print(",");

  Serial.print(haptic.iq_cmd);
  Serial.print(",");
  Serial.print(haptic.last_update_us);
  Serial.print(",");

  Serial.print(state.control_enabled);
  Serial.print(",");
  Serial.print(state.trial_configured);
  Serial.print(",");
  Serial.print(state.trial_active);
  Serial.print(",");

  Serial.print(state.fault_latched);
  Serial.print(",");
  Serial.print(state.fault_bits);
  Serial.print(",");

  Serial.print(state.control_last_heartbeat_us);
  Serial.print(",");
  Serial.print(state.telemetry_last_heartbeat_us);
  Serial.print(",");

  Serial.println();
}

static void processIncomingSerialCommands()
{
  static String lineBuffer = "";

  while (Serial.available() > 0)
  {
    char c = static_cast<char>(Serial.read());

    if (c == '\r')
      continue;

    if (c == '\n')
    {
      lineBuffer.trim();

      if (lineBuffer.length() > 0)
      {
        handleCommandLine(lineBuffer);
      }

      lineBuffer = "";
      continue;
    }

    if (lineBuffer.length() < 128)
    {
      lineBuffer += c;
    }
    else
    {
      // buffer overflow
      lineBuffer = "";
      Serial.println("ERR, Command_Too_Long");
    }
  }
}

void sendHelpMenu()
{
}
// namespace
// {
//   constexpr uint32_t TELEMETRY_TASK_PERIOD_MS = 20;
//   constexpr uint32_t TELEMETRY_TASK_STACK_SIZE = 4096;
//   constexpr UBaseType_t TELEMETRY_TASK_PRIORITY = 2;

//   TaskHandle_t g_telemetry_task_handle = nullptr;

//   const char *modeToString(HapticMode mode)
//   {
//     switch (mode)
//     {
//     case HapticMode::Resistor:
//       return "resistor";
//     case HapticMode::Capacitor:
//       return "capacitor";
//     case HapticMode::Inductor:
//       return "inductor";
//     case HapticMode::Diode:
//       return "diode";
//     case HapticMode::RLC:
//       return "rlc";
//     default:
//       return "unknown";
//     }
//   }

//   void printHelp()
//   {
//     Serial.println();
//     Serial.println("Commands:");
//     Serial.println("  help");
//     Serial.println("  status");
//     Serial.println("  enable");
//     Serial.println("  disable");
//     Serial.println("  zero");
//     Serial.println("  mode resistor");
//     Serial.println("  mode capacitor");
//     Serial.println("  mode inductor");
//     Serial.println("  mode diode");
//     Serial.println("  mode rlc");
//     Serial.println();
//   }

//   bool parseMode(const char *token, HapticMode &mode)
//   {
//     if (strcmp(token, "resistor") == 0 || strcmp(token, "r") == 0)
//     {
//       mode = HapticMode::Resistor;
//       return true;
//     }
//     if (strcmp(token, "capacitor") == 0 || strcmp(token, "c") == 0)
//     {
//       mode = HapticMode::Capacitor;
//       return true;
//     }
//     if (strcmp(token, "inductor") == 0 || strcmp(token, "l") == 0)
//     {
//       mode = HapticMode::Inductor;
//       return true;
//     }
//     if (strcmp(token, "diode") == 0 || strcmp(token, "d") == 0)
//     {
//       mode = HapticMode::Diode;
//       return true;
//     }
//     if (strcmp(token, "rlc") == 0)
//     {
//       mode = HapticMode::RLC;
//       return true;
//     }
//     return false;
//   }

//   void updateTelemetryHeartbeat()
//   {
//     SystemState systemState{};
//     if (readSystemState(systemState))
//     {
//       systemState.telemetry_last_heartbeat_us = micros();
//       writeSystemState(systemState);
//     }
//   }

//   void printStatus()
//   {
//     SystemState systemState{};
//     RuntimeConfig config{};
//     MeasuredState measured{};
//     HapticCommand command{};

//     readSystemState(systemState);
//     readRuntimeConfig(config);
//     readMeasuredState(measured);
//     readHapticCommand(command);

//     Serial.printf(
//         "mode=%s enabled=%d fault=%d angle=%.4f vel=%.4f accel=%.4f iq_cmd=%.4f iq_meas=%.4f\n",
//         modeToString(config.active_mode),
//         systemState.control_enabled ? 1 : 0,
//         systemState.fault_latched ? 1 : 0,
//         measured.angle_rad,
//         measured.velocity_rad_s,
//         measured.acceleration_rad_s2,
//         command.iq_cmd,
//         measured.iq_meas);
//   }

//   void handleLine(char *line)
//   {
//     char *context = nullptr;
//     char *cmd = strtok_r(line, " \t\r\n", &context);
//     if (cmd == nullptr)
//     {
//       return;
//     }

//     if (strcmp(cmd, "help") == 0)
//     {
//       printHelp();
//       return;
//     }

//     if (strcmp(cmd, "status") == 0)
//     {
//       printStatus();
//       return;
//     }

//     if (strcmp(cmd, "enable") == 0)
//     {
//       SystemState systemState{};
//       readSystemState(systemState);
//       systemState.control_enabled = true;
//       writeSystemState(systemState);
//       Serial.println("control enabled");
//       return;
//     }

//     if (strcmp(cmd, "disable") == 0)
//     {
//       SystemState systemState{};
//       readSystemState(systemState);
//       systemState.control_enabled = false;
//       writeSystemState(systemState);
//       Serial.println("control disabled");
//       return;
//     }

//     if (strcmp(cmd, "zero") == 0)
//     {
//       RuntimeConfig config{};
//       MeasuredState measured{};

//       readRuntimeConfig(config);
//       readMeasuredState(measured);

//       config.theta_origin = measured.angle_rad;
//       writeRuntimeConfig(config);

//       Serial.println("theta origin set to current angle");
//       return;
//     }

//     if (strcmp(cmd, "mode") == 0)
//     {
//       char *modeToken = strtok_r(nullptr, " \t\r\n", &context);
//       if (modeToken == nullptr)
//       {
//         Serial.println("missing mode name");
//         return;
//       }

//       HapticMode newMode{};
//       if (!parseMode(modeToken, newMode))
//       {
//         Serial.println("invalid mode");
//         return;
//       }

//       RuntimeConfig config{};
//       readRuntimeConfig(config);
//       config.active_mode = newMode;
//       writeRuntimeConfig(config);

//       Serial.print("mode set to ");
//       Serial.println(modeToString(newMode));
//       return;
//     }

//     Serial.println("unknown command");
//     printHelp();
//   }
// } // namespace

// void TelemetryTask(void *pvParameters)
// {
//   (void)pvParameters;

//   TickType_t lastWakeTime = xTaskGetTickCount();
//   const TickType_t periodTicks =
//       pdMS_TO_TICKS(TELEMETRY_TASK_PERIOD_MS) > 0 ? pdMS_TO_TICKS(TELEMETRY_TASK_PERIOD_MS) : 1;

//   char buffer[96];
//   size_t length = 0;

//   printHelp();

//   for (;;)
//   {
//     while (Serial.available() > 0)
//     {
//       const int ch = Serial.read();
//       if (ch < 0)
//       {
//         break;
//       }

//       if (ch == '\r' || ch == '\n')
//       {
//         if (length > 0)
//         {
//           buffer[length] = '\0';
//           handleLine(buffer);
//           length = 0;
//         }
//         continue;
//       }

//       if (length + 1 < sizeof(buffer))
//       {
//         buffer[length++] = static_cast<char>(ch);
//       }
//     }

//     updateTelemetryHeartbeat();
//     vTaskDelayUntil(&lastWakeTime, periodTicks);
//   }
// }

// void startTelemetryTask()
// {
//   const BaseType_t ok = xTaskCreatePinnedToCore(
//       TelemetryTask,
//       "TelemetryTask",
//       TELEMETRY_TASK_STACK_SIZE,
//       nullptr,
//       TELEMETRY_TASK_PRIORITY,
//       &g_telemetry_task_handle,
//       1);

//   if (ok != pdPASS)
//   {
//     Serial.println("Failed to create TelemetryTask");
//     while (true)
//     {
//       delay(1000);
//     }
//   }
// }
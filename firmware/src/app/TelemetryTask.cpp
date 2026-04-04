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
#include <cstring>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app/Config.h"

void TelemetryTask(void *pvParameters)
{
  (void)pvParameters;
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(TELEMETRY_PERIOD_MS);

  sendHelpMenu();

  while (1)
  {
    // 1. Parse any pending commands from the PC GUI (non-blocking)
    processIncomingSerialCommands();

    // 2. Read all shared state
    MeasuredState measured_state{};
    HapticCommand haptic_command{};
    RuntimeConfig runtime_config{};
    SystemState system_state{};

    readMeasuredState(measured_state);
    readHapticCommand(haptic_command);
    readRuntimeConfig(runtime_config);
    readSystemState(system_state);

    // 3. Emit telemetry line — "T," prefix required by the Python GUI parser
    Serial.print("T,");
    sendTelemetryLine(measured_state, haptic_command, runtime_config, system_state);

    // 4. Update our own heartbeat
    system_state.telemetry_last_heartbeat_us = micros();
    writeSystemState(system_state);

    // 5. Pace at TELEMETRY_PERIOD_MS (50 Hz)
    vTaskDelayUntil(&lastWakeTime, period);
  }
}

// Emits one CSV telemetry line to Serial (without the leading "T," prefix —
// the task loop prints that before calling this).
//
// Field order matches what haptic_knob_tuner.py expects (indices 1-19):
//   [1]  active_mode        [2]  angle_rad         [3]  angle_deg_wrapped
//   [4]  velocity_rad_s     [5]  acceleration_rad_s2
//   [6]  iq_meas  [7] ia  [8] ib  [9] ic  [10] meas_last_update_us
//   [11] iq_cmd   [12] cmd_last_update_us
//   [13] control_enabled  [14] trial_configured  [15] trial_active
//   [16] fault_latched    [17] fault_bits
//   [18] ctrl_heartbeat_us  [19] tel_heartbeat_us
void sendTelemetryLine(MeasuredState &measured,
                       HapticCommand &haptic,
                       RuntimeConfig &config,
                       SystemState &state)
{
  // Field order matches haptic_knob_tuner.py indices [1]-[19]:
  // [1]  mode
  // [2-5]  angle_rad, angle_deg, vel, accel
  // [6-10] iq_meas, ia, ib, ic, meas_ts
  // [11-12] iq_cmd, cmd_ts
  // [13-15] ctrl_en, trial_cfg, trial_act
  // [16-17] fault_lat, fault_bits
  // [18-19] ctrl_hb, tel_hb
  Serial.printf("%u,%.4f,%.4f,%.4f,%.4f,"
                "%.4f,%.4f,%.4f,%.4f,%lu,"
                "%.4f,%lu,"
                "%d,%d,%d,"
                "%d,%lx,"
                "%lu,%lu\n",
                (uint8_t)config.active_mode,
                measured.angle_rad, measured.angle_deg_wrapped,
                measured.velocity_rad_s, measured.acceleration_rad_s2,
                measured.iq_meas, measured.ia, measured.ib, measured.ic,
                (unsigned long)measured.last_update_us,
                haptic.iq_cmd, (unsigned long)haptic.last_update_us,
                (int)state.control_enabled, (int)state.trial_configured, (int)state.trial_active,
                (int)state.fault_latched, (unsigned long)state.fault_bits,
                (unsigned long)state.control_last_heartbeat_us,
                (unsigned long)state.telemetry_last_heartbeat_us);
}

// Reads all available bytes from Serial into a line buffer (non-blocking).
// When a newline is received, dispatches the complete line to handleCommandLine().
// Buffer is capped at 128 chars — oversized commands are dropped with an ERR.
static void processIncomingSerialCommands()
{
  static String lineBuffer = "";

  while (Serial.available() > 0)
  {
    char c = static_cast<char>(Serial.read());

    if (c == '\r')
      continue; // ignore carriage returns (Windows line endings)

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

// Parses and executes a single command line received from the Python GUI.
//
// Every command either updates RuntimeConfig (model params, PID gains, mode,
// theta origin) or SystemState (enable/disable control). After a successful
// write the firmware replies "ACK,<CMD>". On bad input it replies "ERR,<reason>".
//
// Supported commands (see haptic_knob_tuner.py for full protocol):
//   SET_MODE,<0-4>                               — switch active HapticMode
//   SET_ENABLE,<0|1>                             — enable / disable FOC control
//   ZERO                                         — set theta_origin to current angle
//   SET_RES,<resistance_gain>                    — resistor model gain
//   SET_CAP,<K>,<B>                              — capacitor stiffness + damping
//   SET_IND,<L>,<damp>,<adb>,<odb>,<iqdb>        — inductor model params
//   SET_DIODE,<threshold>,<gain>                 — diode model params
//   SET_RLC,<R>,<L>,<C>,<input_gain>,<torque_gain> — RLC model params
//   SET_PID,<mode>,<QP>,<QI>,<QD>,<DP>,<DI>,<DD>  — current loop PID per mode
void handleCommandLine(String &line)
{
  // Read current config/state — we'll modify and write back only what changed
  RuntimeConfig config{};
  SystemState state{};
  MeasuredState measured{};
  readRuntimeConfig(config);
  readSystemState(state);

  if (line.startsWith("SET_MODE,"))
  {
    // TODO: parse mode index (0-4), validate range,
    //       set config.active_mode = static_cast<HapticMode>(idx),
    //       writeRuntimeConfig(config), Serial.println("ACK,SET_MODE")
  }
  else if (line.startsWith("SET_ENABLE,"))
  {
    // TODO: parse 0|1,
    //       set state.control_enabled = (val != 0),
    //       writeSystemState(state), Serial.println("ACK,SET_ENABLE")
  }
  else if (line == "ZERO")
  {
    // TODO: readMeasuredState(measured),
    //       set config.theta_origin = measured.angle_rad,
    //       writeRuntimeConfig(config), Serial.println("ACK,ZERO")
  }
  else if (line.startsWith("SET_RES,"))
  {
    // TODO: parse resistance_gain float from line.substring(8),
    //       set config.resistance_gain, writeRuntimeConfig(config),
    //       Serial.println("ACK,SET_RES")
  }
  else if (line.startsWith("SET_CAP,"))
  {
    // TODO: sscanf two floats (k_virtual, b_virtual) from line.c_str()+8,
    //       set config.k_virtual and config.b_virtual, writeRuntimeConfig(config),
    //       Serial.println("ACK,SET_CAP")
  }
  else if (line.startsWith("SET_IND,"))
  {
    // TODO: sscanf five floats (L, damping, alpha_deadband, omega_deadband, iq_deadband),
    //       set the five config fields, writeRuntimeConfig(config),
    //       Serial.println("ACK,SET_IND")
  }
  else if (line.startsWith("SET_DIODE,"))
  {
    // TODO: sscanf two floats (diode_threshold, diode_gain) from line.c_str()+10,
    //       set config fields, writeRuntimeConfig(config),
    //       Serial.println("ACK,SET_DIODE")
  }
  else if (line.startsWith("SET_RLC,"))
  {
    float R, L, C, ig, tg;
    int parsed = sscanf(line.c_str() + 8, "%f,%f,%f,%f,%f", &R, &L, &C, &ig, &tg);
    if (parsed == 5)
    {
      config.virtual_resistance = R;
      config.virtual_inductance = L;
      config.virtual_capacitance = C;
      config.input_gain = ig;
      config.torque_gain = tg;
      writeRuntimeConfig(config);
      Serial.println("ACK,SET_RLC");
    }
    else
    {
      Serial.println("ERR,SET_RLC expects 5 floats: R,L,C,input_gain,torque_gain");
    }
  }

  else if (line.startsWith("SET_PID,"))
  {
    int modeIdx;
    float qP, qI, qD, dP, dI, dD;
    int parsed = sscanf(line.c_str() + 8, "%d,%f,%f,%f,%f,%f,%f",
                        &modeIdx, &qP, &qI, &qD, &dP, &dI, &dD);
    if (parsed == 7 && modeIdx >= 0 && modeIdx <= 4)
    {
      CurrentLoopPID gains = {{qP, qI, qD}, {dP, dI, dD}};
      switch (modeIdx)
      {
      case 0:
        config.resistor_pid = gains;
        break;
      case 1:
        config.capacitor_pid = gains;
        break;
      case 2:
        config.inductor_pid = gains;
        break;
      case 3:
        config.diode_pid = gains;
        break;
      case 4:
        config.rlc_pid = gains;
        break;
      }
      writeRuntimeConfig(config);
      Serial.println("ACK,SET_PID");
    }
    else
    {
      Serial.println("ERR,SET_PID expects mode_idx(0-4),QP,QI,QD,DP,DI,DD");
    }
  }
  else
  {
    Serial.print("ERR,unknown command: ");
    Serial.println(line);
  }
}

void sendHelpMenu()
{
  Serial.println("Commands:");
  Serial.println("  SET_MODE,<0-4>");
  Serial.println("  SET_ENABLE,<0|1>");
  Serial.println("  ZERO");
  Serial.println("  SET_RES,<resistance_gain>");
  Serial.println("  SET_CAP,<K>,<B>");
  Serial.println("  SET_IND,<L>,<damping>,<alpha_db>,<omega_db>,<iq_db>");
  Serial.println("  SET_DIODE,<threshold>,<gain>");
  Serial.println("  SET_RLC,<R>,<L>,<C>,<input_gain>,<torque_gain>");
  Serial.println("  SET_PID,<mode_idx>,<QP>,<QI>,<QD>,<DP>,<DI>,<DD>");
}
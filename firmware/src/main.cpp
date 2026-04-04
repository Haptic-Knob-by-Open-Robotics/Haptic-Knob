#include <Arduino.h>
#include "app/Hardware.h"
#include "app/SharedState.h"
#include "app/ControlTask.h"
#include "app/TelemetryTask.h"
#include "app/WatchdogTask.h"
/*
  Overall program flow:

  1. boot ESP32 and start serial for debug output
  2. Initialize shared state, mutexes and runtime defaults
  3. Initialize the real motor hardware stack:
      - SPI bus
      - encoder
      - driver
      - motor
      - ADC or any ohter peripherals we use
  4. Create FreeRTOS tasks
  5. Let tasks run forever

  Runtime flow:
  MAIN FOR NOW:
  - ControlTask: The main real-time task. It runs the fast motor/control loop every cycle and runs
                  the slower haptic model update every N cycles using a deterministic counter
  - TelemetryTask: Handles user input through serial, collects runtime configuration, prints debug/telemtry data,
                  and enables control when the system is ready to start the trial

  - WatchdogTask: Monitors task health, stale data, timing faults, and safety conditions.

  SUB:
  - MotorControlTask reads real hardware state and applies commands
  - ModelControlTask computes what the knob should feel like
  - TelemetryTask handles serial commands and debug pritns
  - WatchdogTask checks for stale data / fault conditions
*/

void setup()
{

  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("==== HAPTIC KNOB BOOT ====");

  // Initialize shared software state first. This should set up mutexes, default setpoints, timestamps,
  // fault falgs, and any shared runtime variables that tasks use.
  if (!initSharedState())
  {
    Serial.println("Shared state initialization failed");
    while (true)
    {
      delay(1000);
    }
  }

  // Initialize all real hardware before starting any tasks, we only want the tasks to start running once the
  // hardware stack is fully ready
  if (!initHardware())
  {
    Serial.println("Hardware initilization failed");
    while (true)
    {
      delay(1000);
    }
  }

  // Start telemtry task
  // startTelemetryTask();

  // Create the main deterministic control task, note that we give this the highest priority since it owns the
  // real time motor/control path
  startControlTask();

  // Start the watchdog/safety monitoring task
  // startWatchdogTask();

  Serial.println("System startup complete!");
}

void loop()
{
  // FreeRTOS tasks own the runtime behavior now so just block forever to avoid wasting CPU time here
  Serial.println("inside loop");

  vTaskDelay(portMAX_DELAY);
}

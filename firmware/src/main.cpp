#include <Arduino.h>
#include "app/Hardware.h"
#include "app/SharedState.h"
#include "app/MotorControlTask.h"
#include "app/ModelControlTask.h"
#include "app/TelemetryTask.h"
#include "app/WatchdogTask.h"
/*
  Overall program flow:

  1. boot ESP32 and start serial
  2. Initialize shared state / mutex / runtime defaults 
  3. Initialize the real motor hardware stack
  4. Create FreeRTOS tasks
  5. Let tasks run forever 

  Runtime flow: 
  - MotorControlTask reads real hardware state and applies commands 
  - ModelControlTask computes what the knob should feel like 
  - TelemetryTask handles serial commands and debug pritns 
  - WatchdogTask checks for stale data / fault conditions 
*/

void setup()
{

  Serial.begin(115200); 
  delay(1500); 

  // TODO: initialize shared state, config defualts, and mutex 
  initSharedState(); 

  if (!initHardware())
  {
    Serial.println("Hardware initilization failed");
    while (true){}
  }

  // TODO: Create the Core tasks 

  Serial.println("System startup complete!");
}

void loop()
{
  // FreeRTOS tasks own the runtime behavior now 
  vTaskDelay(portMAX_DELAY);
}
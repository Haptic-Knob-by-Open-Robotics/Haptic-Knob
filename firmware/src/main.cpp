#include <Arduino.h>
#include "app/Config.h"

#include "drivers/SsiEncoder.h"
#include "drivers/MotorDriver.h"

#include "drivers/SpiBus.h"
#include "drivers/AdcSpi.h"
#include "app/SharedState.h"
#include "app/ControlTask.h"
#include "app/TelemetryTask.h"
#include "app/WatchdogTask.h"


// GLobal Driver INstances. We construct these once and reuse across the firmware
SsiEncoder encoder(PIN_ENC_CS, PIN_ENC_CLK, PIN_ENC_MISO);
MotorDriver MtrDrv(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL, PIN_EN);


// ====================== HELPER FUNCTIONS =========================

static bool initDrivers(){

  // Encoder init
  if (!encoder.init()) {
    Serial.println("Encoder init failed!");
    return false 
  }
  Serial.println("Encoder initialized");

  // Motor driver init
  if(!MtrDrv.init(VLIM, VSUP)) {
    Serial.println("Motor Driver init failed!");
    return false;
  }

  // Safety default: 
  // Note in our final firmware design we may want to defer enable() until watchdog is armed and tasks are live 
  MtrDrv.enable();
  Serial.println("Motor Driver initialized");

  return true;
}

static bool initFirmware() {

  // Current bring up we're just initilaizing encoder and motor driver

  // For our final implementation (when we have impelmented RTOS + ADC): 
  // need to intialized shared SPI bus (for encoder + ADC)
  // need to iniailzise shared state + fault flags 
  // initialize control blocks (Estimator/Kalman, Our Setpoint MOdel, PID)
  return initDrivers(); 

}


static void startTasks() {
  // Create controlTask (highest priority)
  // Create WatchdogTask (monitors heartbeat and disables motor on stall)
  // Create TelemetryTask (low rate logging, low priority)
}

static void enterSafeState() {
  // this will get called by WatchdogTask if there is a stall
  // Disable motor driver or we can command zero output 
  MtrDrv.setPWM(0.0f, 0.0f, 0.0f);
  Serial.println("Entering SAFE STATE. Halting.");
  while (1) delay(1000);
}


// Arduino Entry points
void setup() {
  Serial.begin(115200);
  while (!Serial) {

  }

  if (!initFirmware()) {
    enterSafeState();
  }

  startTasks(); 
  // note after takss start loop() should idle forever
}

void loop() {

  // Note in our final fimrware, ControlTask will run this logic at a fixed tick rate 
  
  encoder.update();
  Serial.print("Angle: ");
  Serial.print(encoder.angleDegWrapped(), 2);
  Serial.print("\tVel (rad/s): ");
  Serial.println(encoder.velocityRadS(), 4);

  // open loop motor control
  MtrDrv.setPWM(0.2f, 0.0f, 0.0f);
  delay(5);
  MtrDrv.setPWM(0.0f, 0.0f, 0.0f);
  delay(95);
}

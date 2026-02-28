// #include <Arduino.h>
// #include <SimpleFOC.h>

// #include "app/Config.h"

// // Drivers / modules
// #include "drivers/SpiBus.h"
// #include "drivers/SsiEncoder.h"
// #include "drivers/AdcSpi.h"
// #include "drivers/MotorDriver.h"

// // App layer
// #include "app/SharedState.h"
// #include "app/ControlTask.h"
// #include "app/TelemetryTask.h"
// #include "app/WatchdogTask.h"

// #include "drivers/MagneticSensorMT6701SSI.h"



// // ===================== GLOBAL INSTANCES ==========================

// // Shared SPI bus (encoder + ADC). Exact pins/frequency/mode will come from config.h
// SpiBus spiBus; 

// // MT6701 encoder (SSI). This driver reads the raw angle over SSI
// SsiEncoder encoder(PIN_ENC_CS, PIN_ENC_CLK, PIN_ENC_MISO);

// // ADC current sense (SPI). Implement later keep as stub if needed 
// AdcSpi adc(PIN_ADC_CS); 

// // Motor driver wrapper for our 6-PWM bridge (uses MCPWM under the hood)
// MotorDriver MtrDrv(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL, PIN_EN);

// // Shared state for telemetry + watchdog heartbeat 
// SharedState gShared; 

// // ====================== SIMPLEFOC OBJECTS ========================

// // Motor has 4 pole paris 
// BLDCMotor motor(4); 

// // 6-PWM driver object SimpleFOC will use
// BLDCDriver6PWM focDriver(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL, PIN_EN);

// // Note SimpleFOC needs a Sensor to call sensor->getAngle(). Since we're using MT6701 over SSI, 
// // we will provide a small wrapper class that will expose the encoder through the simpleFOC sensor interface.
// MT6701Sensor focSensor(&encoder); 


// // ====================== HELPER FUNCTIONS =========================

// static bool initDrivers(){
  
//   // SPI bus (shared)
//   if (!spiBus.init(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, SPI_HZ)) {
//     Serial.println("SPI bus init failed!");
//     return false;
//   }
//   Serial.println("SPI bus initialized");


//   // Encoder init (MT6701 SSI)
//   if (!encoder.init()) {
//     Serial.println("Encoder init failed!");
//     return false 
//   }
//   Serial.println("Encoder initialized");

//   // ADC (current sensing) 
//   if (!adc.init(&spiBus)) {
//     Serial.println("ADC init failed!");
//     return false; 
//   }
//   Serial.println("ADC initialized");

//   // Motor driver wrapper 
//   if(!MtrDrv.init(VLIM, VSUP)) {
//     Serial.println("Motor Driver init failed!");
//     return false;
//   }
//   Serial.println("Motor Driver wrapper initialized");

//   return true;
// }


// static bool initSimpleFOC() {

//   // Configure SimpleFOC driver 
//   focDriver.pwm_frequency = 30000;
//   focDriver.dead_zone = 0.05f;
//   focDriver.voltage_power_supply = VSUP;   // e.g., 9V supply (from your diagram)
//   focDriver.voltage_limit = VLIM;          // safety clamp
//   if (!focDriver.init()) {
//     Serial.println("SimpleFOC driver init failed!");
//     return false;
//   }

//   // Link motor to driver + sensor 
//   motor.linkDriver(&focDriver); 
//   motor.linkSensor(&focSensor);

//   // Choose initial model (rn set as torque-voltage)
//   motor.controller = MotionControlType::torque;
//   motor.torque_controller = TorqueControlType::voltage; 

//   // Safety limits
//   motor.voltage_limit = VLIM;
//   motor.current_limit = I_LIM;  // only used if/when current sense is enabled

//   // Init motor + FOC
//   motor.init();

//   // If sensor direction is wrong, initFOC may fail or behave badly.
//   // You can later use motor.initFOC() with an offset calibration routine.
//   motor.initFOC();

//   Serial.println("SimpleFOC initialized");
//   return true;

// }
// static bool initFirmware() {

//   // Current bring up we're just initilaizing encoder and motor driver

//   // For our final implementation (when we have impelmented RTOS + ADC): 
//   // need to intialized shared SPI bus (for encoder + ADC)
//   // need to iniailzise shared state + fault flags 
//   // initialize control blocks (Estimator/Kalman, Our Setpoint MOdel, PID)
//   if (!initDrivers()) return false; 
//   if (!initSimpleFOC()) return false; 

//   return true; 
// }


// static void startTasks() {
//   // Create controlTask (highest priority)
//   // Create WatchdogTask (monitors heartbeat and disables motor on stall)
//   // Create TelemetryTask (low rate logging, low priority)
// }

// static void enterSafeState() {
//   // Force motor safe state and halt.
//   // In final firmware, WatchdogTask will call a similar path.
//   motor.disable();
//   focDriver.disable();

//   Serial.println("Entering SAFE STATE. Halting.");
//   while (1) delay(1000);
// }


// // Arduino Entry points
// void setup() {
//   Serial.begin(115200);
//   while (!Serial) {
//     delay(10);
//   }

//   if (!initFirmware()) {
//     enterSafeState();
//   }

//   startTasks(); 
//   // note after takss start loop() should idle forever
// }

// void loop() {

//   delay(1000); 
// }

#include <Arduino.h>
#include "app/Config.h"
#include "drivers/SsiEncoder.h"

SsiEncoder encoder(PIN_ENC_CS, PIN_ENC_CLK, PIN_ENC_MISO);

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("=== MT6701 Encoder Test ===");

  if (!encoder.init()) {
    Serial.println("ERROR: Encoder initialization failed!");
    while (1) {
      delay(1000);
      Serial.println("Encoder init failed - halted");
    }
  }

  Serial.println("Encoder initialized successfully!");
  Serial.println("Starting readings...\n");
  delay(500);
}

void loop() {
  encoder.update();

  float angleDeg = encoder.angleDegWrapped();
  float angleRad = encoder.angleRad();
  float velocity = encoder.velocityRadS();

  Serial.print("Angle: ");
  Serial.print(angleDeg, 2);
  Serial.print("° (");
  Serial.print(angleRad, 4);
  Serial.print(" rad)\t");
  
  Serial.print("Velocity: ");
  Serial.print(velocity, 4);
  Serial.println(" rad/s");

  delay(100);  // 10Hz update rate
}
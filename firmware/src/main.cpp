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
#include <SimpleFOC.h>
#include "app/Config.h"
#include "drivers/ModifiedMagneticSensorMT6701SSI.h"
#include "drivers/SpiBus.h"

SpiBus spiBus(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI);
ModifiedMagneticSensorMT6701SSI encoder(PIN_ENC_CS);
InlineCurrentSense current_sense(SHUNT_RESISTOR, AMP_GAIN, PIN_I_A, PIN_I_B, PIN_I_C);
BLDCDriver6PWM driver(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL);
BLDCMotor motor(12);

void setup()
{
  Serial.begin(115200);
  delay(3000);
  Serial.println("=== Encoder + Current Sense Test ===");
  Serial.println(analogRead(PIN_I_A));
  Serial.println(analogRead(PIN_I_B));
  Serial.println(analogRead(PIN_I_C));

  // SPI bus
  spiBus.init();
  encoder.init(spiBus.bus());
  Serial.println("Encoder initialized!");

  // Driver
  Serial.print("Driver init... ");
  driver.voltage_power_supply = VSUP;
  driver.pwm_frequency = 30000;
  driver.dead_zone = 0.05f;
  if (!driver.init())
  {
    Serial.println("FAILED - halting");
    while (1)
      ;
  }
  Serial.println("done");

  // Current sense
  Serial.print("Current sense init... ");
  current_sense.linkDriver(&driver);
  if (current_sense.init())
  {
    Serial.println("SUCCESS");
  }
  else
  {
    Serial.println("FAILED - check ADC pins, shunt resistor, op-amp gain");
    while (1)
      ;
  }

  Serial.println("Calibrating...");
  current_sense.driverAlign(VSUP);
  Serial.println("Calibration done\n");

  motor.linkDriver(&driver);
  motor.linkSensor(&encoder);
  motor.controller = MotionControlType::velocity_openloop;
  motor.voltage_limit = 3.0f; // keep low for testing
  motor.init();
}

void loop()
{
  encoder.update();

  float angleDeg = encoder.angleDegWrapped();
  float angleRad = encoder.getAngle();
  float velocity = encoder.getVelocity();
  PhaseCurrent_s currents = current_sense.getPhaseCurrents();

  Serial.print("Angle: ");
  Serial.print(angleDeg, 2);
  Serial.print("° (");
  Serial.print(angleRad, 4);
  Serial.print(" rad)  Vel: ");
  Serial.print(velocity, 4);
  Serial.print(" rad/s  |  I_a: ");
  Serial.print(currents.a, 3);
  Serial.print(" A  I_b: ");
  Serial.print(currents.b, 3);
  Serial.print(" A  I_c: ");
  Serial.print(currents.c, 3);
  Serial.println(" A");

  delay(100);

  static uint32_t start = millis();
  uint32_t elapsed = (millis() - start) % 6000;

  if (elapsed < 3000)
  {
    motor.move(10.0f); // CW ~10 rad/s
  }
  else
  {
    motor.move(-10.0f); // CCW
  }

  motor.loopFOC();
}
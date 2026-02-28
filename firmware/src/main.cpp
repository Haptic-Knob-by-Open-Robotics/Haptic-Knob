#include <Arduino.h>
#include <SPI.h>

#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>

#include "app/Config.h"
#include "drivers/MagneticSensorMT6701SSI.h"

// Hardware constants 
static constexpr float SHUNT_RESISTOR = 0.012f;  // 12 mΩ
static constexpr float AMP_GAIN       = 50.0f;

// SimpleFOC objects
BLDCMotor motor(4); // 4 pole pairs (from your earlier bring-up)

// 6-PWM driver (same pins as your test)
BLDCDriver6PWM driver(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL, PIN_EN);

// MT6701 SSI sensor (your working class)
MagneticSensorMT6701SSI encoder(PIN_ENC_CS);

// Inline current sense using 3 phase current pins
InlineCurrentSense current_sense(SHUNT_RESISTOR, AMP_GAIN, PIN_I_A, PIN_I_B, PIN_I_C);

// Optional: SimpleFOC commander for runtime tuning (nice for quick testing)
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }
void doVoltageLimit(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }
void doCurrentLimit(char* cmd) { command.scalar(&motor.current_limit, cmd); }

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("\n=== INTEGRATED FOC CURRENT TORQUE TEST ===");

  // 1) SPI + encoder init (from your encoder test)
  SPI.begin(PIN_ENC_CLK, PIN_ENC_MISO, -1, PIN_ENC_CS);
  encoder.init();
  Serial.println("Encoder initialized");

  // 2) Driver init (from your current test)
  driver.voltage_power_supply = 12.0f;    // your supply
  driver.voltage_limit        = 6.0f;     // safety clamp (tune)
  driver.pwm_frequency        = 30000;    // your prior setting
  driver.dead_zone            = 0.05f;

  if (!driver.init()) {
    Serial.println("Driver init FAILED");
    while (1) delay(1000);
  }
  Serial.println("Driver initialized");

  // 3) Current sense init + alignment (from your current test)
  current_sense.linkDriver(&driver);

  if (!current_sense.init()) {
    Serial.println("Current sense init FAILED");
    while (1) delay(1000);
  }
  Serial.println("Current sense initialized");

  Serial.println("Calibrating current sense (driverAlign)...");
  current_sense.driverAlign(12.0f);  // alignment voltage (you used 12)
  Serial.println("Current sense calibration done");

  // 4) Link everything to motor
  motor.linkDriver(&driver);
  motor.linkSensor(&encoder);
  motor.linkCurrentSense(&current_sense);

  // 5) Choose control mode: torque via FOC current
  motor.controller       = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::foc_current; // <-- closed-loop current torque

  // Safety limits
  motor.voltage_limit = 6.0f;     // keep conservative at first
  motor.current_limit = 1.0f;     // start low (amps), raise gradually

  // 6) Motor init + FOC init
  motor.init();

  // If direction/offset are wrong, initFOC can behave badly.
  // You may need to flip sensor direction depending on mounting:
  // encoder.direction = Direction::CW; or Direction::CCW (if supported by your sensor class)
  motor.initFOC();

  Serial.println("Motor + FOC initialized");

  // 7) Start with zero torque command
  motor.target = 0.0f;

  // Optional runtime tuning
  command.add('T', doTarget, "target (Iq A)"); // in foc_current torque mode, target is Iq (A)
  command.add('V', doVoltageLimit, "voltage_limit");
  command.add('I', doCurrentLimit, "current_limit");

  Serial.println("\nCommands:");
  Serial.println("  T <amps>   (sets target Iq current in A)");
  Serial.println("  I <amps>   (sets motor.current_limit)");
  Serial.println("  V <volts>  (sets motor.voltage_limit)");
  Serial.println("\nStart by trying: T 0.2  (small torque)\n");
}

void loop() {
  // FOC loop must run fast and continuously
  motor.loopFOC();

  // Apply commanded torque/current
  motor.move();

  // Optional: serial commander (lets you type commands in Serial Monitor)
  command.run();

  // Low-rate debug print (don’t spam)
  static uint32_t last_ms = 0;
  if (millis() - last_ms > 200) {
    last_ms = millis();

    // Read currents for visibility (phase)
    PhaseCurrent_s ph = current_sense.getPhaseCurrents();

    Serial.print("angle(rad): "); Serial.print(motor.shaftAngle(), 3);
    Serial.print("\tvel(rad/s): "); Serial.print(motor.shaftVelocity(), 3);
    Serial.print("\tIq_target(A): "); Serial.print(motor.target, 3);

    Serial.print("\tIa: "); Serial.print(ph.a, 3);
    Serial.print("\tIb: "); Serial.print(ph.b, 3);
    Serial.print("\tIc: "); Serial.print(ph.c, 3);

    Serial.println();
  }
}
#include <Arduino.h>
#include "app/Config.h"
#include "drivers/SsiEncoder.h"


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
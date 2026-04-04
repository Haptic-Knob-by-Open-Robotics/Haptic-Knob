#include <Arduino.h>
#include <SimpleFOC.h>
#include "app/Config.h"
#include "drivers/ModifiedMagneticSensorMT6701SSI.h"
#include "drivers/SpiBus.h"

SpiBus spiBus(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI);
ModifiedMagneticSensorMT6701SSI encoder(PIN_ENC_CSN);
InlineCurrentSense current_sense(CURRENT_SENSE_SHUNT_RESISTOR_OHM, CURRENT_SENSE_AMP_GAIN, PIN_I_A, PIN_I_B, PIN_I_C);
BLDCDriver6PWM driver(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL);
BLDCMotor motor(POLE_PAIRS);

float target_current = 0.3f;

void printData()
{
  float angleDeg = encoder.getSensorAngle();
  float angleRad = encoder.getAngle();
  float velocity = encoder.getVelocity();
  PhaseCurrent_s currents = current_sense.getPhaseCurrents();

  Serial.printf("Angle: %7.2f°  Velocity: %6.2f rad/s  Ia: %7.3fA  Ib: %7.3fA  Ic: %7.3fA\n",
                angleDeg, velocity, currents.a, currents.b, currents.c);
  // Serial.print("Angle: ");
  // Serial.print(angleDeg, 2);
  // Serial.print("°\tVelocity: ");
  // Serial.print(velocity, 2);
  // Serial.print(" rad/s\tIa: ");
  // Serial.print(currents.a, 3);
  // Serial.print("A\tIb: ");
  // Serial.print(currents.b, 3);
  // Serial.print("A\tIc: ");
  // Serial.print(currents.c, 3);
  // Serial.print("A");
  // Serial.println();
}

void setup()
{
  Serial.begin(115200);
  delay(3000);
  Serial.println("=== Closed-Loop Torque Control Test ===");

  // SPI bus
  spiBus.init();
  encoder.init(spiBus.bus());
  Serial.println("Encoder initialized!");

  // Driver
  Serial.print("Driver init... ");
  driver.voltage_power_supply = VOLTAGE_SUPPLY_V;
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

  //   Serial.println("Calibrating...");
  //   current_sense.driverAlign(DRIVER_VOLTAGE_LIMIT_V);
  //   Serial.println("Calibration done\n");

  motor.linkDriver(&driver);
  motor.linkSensor(&encoder);
  motor.linkCurrentSense(&current_sense);
  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::torque;

  motor.voltage_limit = DRIVER_VOLTAGE_LIMIT_V; // keep low for testing
  motor.current_limit = 1.0f;

  // current loop tuning (conservative starting values)
  motor.PID_current_q.P = 3.0f;
  motor.PID_current_q.I = 300.0f;
  motor.PID_current_q.D = 0.0f;

  motor.PID_current_d.P = 3.0f;
  motor.PID_current_d.I = 300.0f;
  motor.PID_current_d.D = 0.0f;

  motor.LPF_current_q.Tf = 0.01f;
  motor.LPF_current_d.Tf = 0.01f;

  motor.useMonitoring(Serial);
  Serial.print("Motor init");
  motor.init();
  Serial.print("Done");

  Serial.print("FOC init... ");
  if (!motor.initFOC())
  {
    Serial.println("FAILED");
    while (1)
    {
    }
  }

  motor.target = target_current;

  Serial.println("Done");

  Serial.println("Starting torque test");
}

void loop()
{
  motor.loopFOC();

  motor.move();
  motor.monitor();
  // command.run();
  // static uint32_t last_print = 0;
  // static uint32_t last_switch = 0;
  // uint32_t now = millis();

  // // alternate direction every 3 seconds
  // if (now - last_switch > 3000)
  // {
  //   last_switch = now;
  //   target_current = -target_current;
  // }

  // if (now - last_print > 100)
  // {
  //   last_print = now;
  //   printData();
  // }
}

#include <Arduino.h>
#include <SimpleFOC.h>
#include "app/Config.h"
#include "drivers/ModifiedMagneticSensorMT6701SSI.h"
#include "drivers/SpiBus.h"

SpiBus spiBus(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI);
ModifiedMagneticSensorMT6701SSI encoder(PIN_ENC_CS);
InlineCurrentSense current_sense(SHUNT_RESISTOR_OHM, CURRENT_SENSE_AMP_GAIN, PIN_I_A, PIN_I_B, PIN_I_C);
BLDCDriver6PWM driver(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL);
BLDCMotor motor(POLE_PAIRS);

float target_angle = 0.0f;

void printData()
{
  Serial.printf(
      "Target: %7.3f rad | Angle: %7.3f rad | Angle: %7.2f deg | Vel: %7.3f rad/s\n",
      target_angle,
      motor.shaft_angle,
      motor.shaft_angle * 180.0f / _PI,
      motor.shaft_velocity);
}

void setup()
{
  Serial.begin(115200);
  delay(3000);
  SimpleFOCDebug::enable(&Serial);

  Serial.println("=== FOC Position Hold Test ===");

  spiBus.init();
  encoder.init(spiBus.bus());
  Serial.println("Encoder initialized");

  driver.voltage_power_supply = VOLTAGE_SUPPLY_V;
  driver.voltage_limit = DRIVER_VOLTAGE_LIMIT_V;
  driver.pwm_frequency = 30000;
  driver.dead_zone = 0.05f;

  Serial.print("Driver init... ");
  if (!driver.init())
  {
    Serial.println("FAILED");
    while (1)
    {
    }
  }
  Serial.println("done");

  motor.linkDriver(&driver);
  motor.linkSensor(&encoder);

  // Use angle control
  motor.controller = MotionControlType::angle;
  motor.torque_controller = TorqueControlType::voltage;

  motor.voltage_limit = DRIVER_VOLTAGE_LIMIT_V;
  motor.velocity_limit = 20.0f;
  motor.voltage_sensor_align = 4.0f;

  // Position loop tuning
  motor.P_angle.P = 20.0f;
  motor.P_angle.output_ramp = 1000.0f;
  motor.P_angle.limit = 30.0f;

  motor.PID_velocity.P = 0.8f;
  motor.PID_velocity.I = 10.0f;
  motor.PID_velocity.D = 0.0f;
  motor.PID_velocity.output_ramp = 1000.0f;
  motor.PID_velocity.limit = 4.0f;

  motor.LPF_velocity.Tf = 0.01f;

  Serial.print("Motor init... ");
  motor.init();
  Serial.println("done");

  Serial.print("FOC init... ");
  if (!motor.initFOC())
  {
    Serial.println("FAILED");
    while (1)
    {
    }
  }
  Serial.println("done");

  delay(200);

  // Hold whatever angle the motor is currently at
  target_angle = motor.shaft_angle + 0.5f;

  Serial.print("Target angle set to: ");
  Serial.println(target_angle, 4);

  Serial.println("Starting position hold test");
}

void loop()
{
  motor.loopFOC();
  motor.move(target_angle);

  static uint32_t last_print = 0;
  if (millis() - last_print > 100)
  {
    last_print = millis();
    printData();
  }
}
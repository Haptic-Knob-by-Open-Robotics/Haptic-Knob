#include <Arduino.h>
#include <SimpleFOC.h>
#include "app/Config.h"
#include "drivers/ModifiedMagneticSensorMT6701SSI.h"
#include "drivers/SpiBus.h"

SpiBus spiBus(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI);
ModifiedMagneticSensorMT6701SSI encoder(PIN_ENC_CS);
InlineCurrentSense current_sense(SHUNT_RESISTOR, AMP_GAIN, PIN_I_A, PIN_I_B, PIN_I_C);
BLDCDriver6PWM driver(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL);
BLDCMotor motor(4);

float target_voltage_torque = 0.2f;

void printData()
{
//   encoder.update();

//   Serial.printf(
//       "Angle: %7.2f deg | Vel: %7.3f rad/s | ShaftAngle: %7.3f rad\n",
//       encoder.angleDegWrapped(),
//       encoder.getVelocity(),
//       motor.shaft_angle
//   );
Serial.printf(
      "Angle: %7.2f deg | Vel: %7.3f rad/s | ShaftAngle: %7.3f rad\n",
      motor.shaft_angle * 180.0f / _PI,
      motor.shaft_velocity,
      motor.shaft_angle
  );
}

void setup()
{
  Serial.begin(115200);
  delay(3000);
  SimpleFOCDebug::enable(&Serial);
  Serial.println("=== FOC Voltage Torque Test ===");

  spiBus.init();
  encoder.init(spiBus.bus());
  Serial.println("Encoder initialized");

  driver.voltage_power_supply = SUPPLY_VOLTAGE;
  driver.voltage_limit = VOLTAGE_LIMIT;
  driver.pwm_frequency = 30000;
  driver.dead_zone = 0.05f;

  Serial.print("Driver init... ");
  if (!driver.init())
  {
    Serial.println("FAILED");
    while (1) {}
  }
  Serial.println("done");

  motor.linkDriver(&driver);
  motor.linkSensor(&encoder);

  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;

  motor.voltage_limit = VOLTAGE_LIMIT;
  motor.velocity_limit = 20.0f;
  motor.voltage_sensor_align = 4.0f;


  
  Serial.print("Motor init... ");
  motor.init();
  Serial.println("done");

  motor.sensor_direction = Direction::CCW;

  Serial.print("FOC init... ");
  if (!motor.initFOC())
  {
    Serial.println("FAILED");
    while (1) {}
  }
  Serial.println("done");

  Serial.println("Starting test");
}

void loop()
{
//   motor.loopFOC();
//   motor.move(target_voltage_torque);

//   static uint32_t last_print = 0;
//   static uint32_t last_flip = 0;
//   uint32_t now = millis();

//   if (now - last_flip > 3000)
//   {
//     last_flip = now;
//     target_voltage_torque = -target_voltage_torque;
//   }

//   if (now - last_print > 100)
//   {
//     last_print = now;
//     printData();
//   }
    motor.loopFOC();
    motor.move(1.0f);

//   motor.move(target_voltage_torque);

  static uint32_t last_print = 0;
  if (millis() - last_print > 100)
  {
    last_print = millis();
    printData();
  }
}
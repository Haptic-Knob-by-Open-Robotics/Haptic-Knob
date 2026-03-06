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

void getData()
{
  float angleDeg = encoder.angleDegWrapped();
  float angleRad = encoder.getAngle();
  float velocity = encoder.getVelocity();
  PhaseCurrent_s currents = current_sense.getPhaseCurrents();

  Serial.printf("Angle: %7.2f°  Velocity: %6.2f rad/s  Ia: %7.3fA  Ib: %7.3fA  Ic: %7.3fA\n",
                angleDeg, velocity, currents.a, currents.b, currents.c);
}

void setup()
{
  Serial.begin(115200);
  delay(3000);
  Serial.println("=== Encoder + Current Sense Test ===");

  // SPI bus
  spiBus.init();
  encoder.init(spiBus.bus());
  Serial.println("Encoder initialized!");

  // Driver
  Serial.print("Driver init... ");
  driver.voltage_power_supply = SUPPLY_VOLTAGE;
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
  current_sense.driverAlign(VOLTAGE_LIMIT);
  Serial.println("Calibration done\n");

  motor.linkDriver(&driver);
  motor.linkSensor(&encoder);
  motor.controller = MotionControlType::velocity_openloop;
  motor.voltage_limit = VOLTAGE_LIMIT; // keep low for testing
  motor.init();
}

void loop()
{
  encoder.update();
  float angleRad = encoder.getSensorAngle();

  static uint32_t start = millis();
  static uint32_t last_ms = 0;
  uint32_t elapsed = (millis() - start) % 6000;

  if (millis() - last_ms > 100)
  {
    last_ms = millis();
    getData();
  }

  // if (elapsed < 3000)
  // {
  //   motor.move(-10.0f); // CW ~10 rad/s
  // }
  // else
  // {
  //   motor.move(10.0f); // CCW
  // }

  // motor.loopFOC();
}

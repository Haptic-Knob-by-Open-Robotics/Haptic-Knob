#include <Arduino.h>
#include <SimpleFOC.h>
#include "app/Config.h"
#include "drivers/ModifiedMagneticSensorMT6701SSI.h"
#include "drivers/SpiBus.h"

SpiBus spiBus(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI);
ModifiedMagneticSensorMT6701SSI encoder(PIN_ENC_CSN);
BLDCDriver6PWM driver(PIN_VH, PIN_VL, PIN_UH, PIN_UL, PIN_WH, PIN_WL, PIN_EN);
BLDCMotor motor(POLE_PAIRS);

float target_angle = 0.0f;
bool position_set = false;

void setup()
{
  Serial.begin(115200);
  delay(3000);
  SimpleFOCDebug::enable(&Serial);

  spiBus.init();
  encoder.init(spiBus.bus());

  driver.voltage_power_supply = 12.0f;
  driver.voltage_limit = 5.0f;
  driver.pwm_frequency = 30000;
  driver.dead_zone = 0.05f;

  if (!driver.init())
  {
    Serial.println("driver init failed");
    while (1)
    {
    }
  }

  motor.linkDriver(&driver);
  motor.linkSensor(&encoder);

  motor.controller = MotionControlType::angle;
  motor.torque_controller = TorqueControlType::voltage;

  motor.voltage_limit = 5.0f;
  motor.velocity_limit = 20.0f;
  motor.voltage_sensor_align = 4.0f;

  motor.P_angle.P = 8.0f;
  motor.P_angle.I = 0.0f;
  motor.P_angle.D = 0.1f;
  motor.P_angle.output_ramp = 10000.0f;
  motor.P_angle.limit = 20.0f;

  motor.PID_velocity.P = 0.5f;
  motor.PID_velocity.I = 2.0f;
  motor.PID_velocity.D = 0.0f;
  motor.PID_velocity.output_ramp = 10000.0f;
  motor.PID_velocity.limit = 5.0f;

  // moderate filtering
  motor.LPF_velocity.Tf = 0.01f;

  motor.init();

  if (!motor.initFOC())
  {
    Serial.println("FOC init failed");
    while (1)
    {
    }
  }

  Serial.println("FOC initialized - motor should hold position firmly");
}

void loop()
{
  motor.loopFOC();

  // Set target after startup
  if (!position_set && millis() > 1000)
  {
    target_angle = motor.shaft_angle;
    position_set = true;
    Serial.printf("\nLocked to position: %.3f rad (%.1f deg)\n",
                  target_angle, target_angle * 57.2958f);
  }

  motor.move(target_angle);

  static uint32_t last_print = 0;
  if (millis() - last_print > 200)
  {
    last_print = millis();
    float error = target_angle - motor.shaft_angle;
    Serial.printf("err=%+.4f | vel=%+6.2f | Vq=%+.2f\n",
                  error, motor.shaft_velocity, motor.voltage.q);
  }

  static float max_error_seen = 0;
  static uint32_t error_time = 0;

  float abs_error = abs(motor.shaft_angle - target_angle);

  if (abs_error > 1.0f)
  {
    if (error_time == 0)
    {
      error_time = millis();
    }
    else if (millis() - error_time > 2000)
    {
      // error has been > 1 rad for 2 seconds then runaway
      Serial.printf("\n!!! RUNAWAY: Error stuck at %.2f rad for 2s !!!\n", abs_error);
      Serial.println("Try changing sensor_direction from CCW to CW (or vice versa)");
      motor.disable();
      while (1)
      {
        delay(1000);
      }
    }
  }
  else
  {
    error_time = 0; // Reset timer if error goes back down
  }

  if (abs_error > max_error_seen)
  {
    max_error_seen = abs_error;
  }
}
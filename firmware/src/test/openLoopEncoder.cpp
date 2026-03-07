#include <Arduino.h>
#include <SimpleFOC.h>
#include "app/Config.h"
#include "drivers/ModifiedMagneticSensorMT6701SSI.h"
#include "drivers/SpiBus.h"

BLDCMotor motor(4);
SpiBus spiBus(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI);
BLDCDriver6PWM driver(PIN_VH, PIN_VL, PIN_UH, PIN_UL, PIN_WH, PIN_WL, PIN_EN);
ModifiedMagneticSensorMT6701SSI encoder(PIN_ENC_CS);

unsigned long lastPrintUs = 0;
const unsigned long printPeriodUs = 10000;   // 100 Hz

void setup() {
    Serial.begin(115200);
    delay(3000);

    Serial.println("\nTEST WITH SWAPPED PHASES A-B\n");

    spiBus.init();
    encoder.init(spiBus.bus());
    Serial.println("Encoder initialized");

    motor.linkSensor(&encoder);

    driver.pwm_frequency = 30000;
    driver.voltage_power_supply = 12.0f;
    driver.init();
    motor.linkDriver(&driver);

    motor.controller = MotionControlType::velocity_openloop;
    motor.voltage_limit = 2.0f;

    motor.init();

    motor.target = 2.0f;

    Serial.println("OPEN LOOP TEST FIXED AT T2");
    Serial.println("time_s,angle_rad,angle_deg,velocity_rad_s");
}

void loop() {

    motor.loopFOC();
    motor.move();

    encoder.update();

    float angleRad = encoder.getSensorAngle();
    float angleDeg = angleRad * 180.0f / PI;
    float velocity = encoder.getVelocity();

    unsigned long now = micros();
    if (now - lastPrintUs >= printPeriodUs) {

        lastPrintUs = now;

        float t = now * 1e-6f;

        Serial.print(t,6);
        Serial.print(",");
        Serial.print(angleRad,6);
        Serial.print(",");
        Serial.print(angleDeg,3);
        Serial.print(",");
        Serial.println(velocity,6);
    }
}
#include <Arduino.h>
#include <SimpleFOC.h>
#include "app/Config.h"
#include "drivers/ModifiedMagneticSensorMT6701SSI.h"
#include "drivers/SpiBus.h"

BLDCMotor motor(4);
SpiBus spiBus(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI);
BLDCDriver6PWM driver(PIN_VH, PIN_VL, PIN_UH, PIN_UL, PIN_WH, PIN_WL, PIN_EN);  // swapped A/B
ModifiedMagneticSensorMT6701SSI encoder(PIN_ENC_CS);

unsigned long lastPrintUs = 0;
const unsigned long printPeriodUs = 10000;   // 100 Hz

float prevWrappedRad = 0.0f;
float unwrappedRad = 0.0f;
bool unwrapInitialized = false;

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
    Serial.println("time_s,target_rad_s,wrapped_deg,wrapped_rad,unwrapped_deg,unwrapped_rad,velocity_rad_s");
}

void loop() {
    motor.loopFOC();
    motor.move();

    encoder.update();

    float wrappedDeg = encoder.angleDegWrapped();
    float wrappedRad = wrappedDeg * PI / 180.0f;

    if (!unwrapInitialized) {
        prevWrappedRad = wrappedRad;
        unwrappedRad = wrappedRad;
        unwrapInitialized = true;
    } else {
        float delta = wrappedRad - prevWrappedRad;

        if (delta > PI) {
            delta -= 2.0f * PI;
        } else if (delta < -PI) {
            delta += 2.0f * PI;
        }

        unwrappedRad += delta;
        prevWrappedRad = wrappedRad;
    }

    float unwrappedDeg = unwrappedRad * 180.0f / PI;
    float velocity = encoder.getVelocity();

    unsigned long now = micros();
    if (now - lastPrintUs >= printPeriodUs) {
        lastPrintUs = now;

        float t = now * 1e-6f;

        Serial.print(t, 6);
        Serial.print(",");
        Serial.print(motor.target, 4);
        Serial.print(",");
        Serial.print(wrappedDeg, 3);
        Serial.print(",");
        Serial.print(wrappedRad, 6);
        Serial.print(",");
        Serial.print(unwrappedDeg, 3);
        Serial.print(",");
        Serial.print(unwrappedRad, 6);
        Serial.print(",");
        Serial.println(velocity, 6);
    }
}
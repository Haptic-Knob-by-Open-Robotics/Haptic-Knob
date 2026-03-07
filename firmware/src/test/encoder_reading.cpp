#include <Arduino.h>
#include <SimpleFOC.h>
#include "app/Config.h"
#include "drivers/ModifiedMagneticSensorMT6701SSI.h"
#include "drivers/SpiBus.h"

SpiBus spiBus(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI);
ModifiedMagneticSensorMT6701SSI encoder(PIN_ENC_CS);

unsigned long lastPrintUs = 0;
const unsigned long printPeriodUs = 5000;   // 200 Hz

void setup() {
    Serial.begin(115200);
    delay(2000);

    spiBus.init();
    encoder.init(spiBus.bus());
    Serial.println("Encoder initialized!");


    // CSV header
    Serial.println("time_s,angle_deg,angle_rad,velocity_rad_s");
}

void loop() {
    encoder.update();

    unsigned long now = micros();
    if (now - lastPrintUs >= printPeriodUs) {
        lastPrintUs = now;

        float t = now * 1e-6f;
        float angleRad = encoder.getAngle();
        float angleDeg = angleRad * 180.0f / PI;
        float velocity = encoder.getVelocity();

        Serial.print(t, 6);
        Serial.print(",");
        Serial.print(angleDeg, 3);
        Serial.print(",");
        Serial.print(angleRad, 6);
        Serial.print(",");
        Serial.println(velocity, 6);
    }
}
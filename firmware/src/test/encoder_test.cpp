    // #include <Arduino.h>
    // #include <SPI.h>
    // #include <SimpleFOC.h>
    // #include <SimpleFOCDrivers.h>
    // #include "drivers/ModifiedMagneticSensorMT6701SSI.h"
    // #include "app/Config.h"
    // #include "drivers/SpiBus.h"
    // #include <math.h>

    // // #define SENSOR1_CS 5 // some digital pin that you're using as the nCS pin

    // //4 pole pairs, 8 poles total
    // // BLDCMotor motor = BLDCMotor(4); // set up for 4 pole pairs
    // // BLDCDriver6PWM( int phA_h, int phA_l, int phB_h, int phB_l, int phC_h, int phC_l, int en)
    // // BLDCDriver6PWM driver = BLDCDriver6PWM(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL, PIN_EN);

    // ModifiedMagneticSensorMT6701SSI encoder(PIN_ENC_CS); // Setup encoder 
    // SpiBus spiBus(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI);

    // void setup() 
    // {   
    //     Serial.begin(115200);

    //     spiBus.init();
    //     encoder.init(spiBus.bus());
    //     Serial.println("Encoder initialized");
    //     encoder.init();

    //     Serial.println("Sensor initialized");

    //     // motor.linkSensor(&encoder);

    //     Serial.println("Motor initialized");

    //     delay(1000);
    // }

    // void loop() 
    // {
    //     encoder.update(); // Update sensor values
    
    //     float angle = fmod(encoder.getAngle() * (180.0 / M_PI), 360.0);
    //     if (angle < 0) angle += 360.0;

    //     Serial.print("Angle: ");
    //     Serial.print(angle, 2); // Print angle with 2 decimal places
    //     Serial.print("\t");
    //     Serial.print("Velocity: ");
    //     Serial.println(encoder.getVelocity());

        
    //     delay(100);
    // }
#include <Arduino.h>
#include <SimpleFOC.h>
#include "app/Config.h"
#include "drivers/ModifiedMagneticSensorMT6701SSI.h"
#include "drivers/SpiBus.h"

SpiBus spiBus(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI);
ModifiedMagneticSensorMT6701SSI encoder(PIN_ENC_CS);

unsigned long lastPrintMs = 0;

void setup() {
    Serial.begin(115200);
    delay(2000);

    spiBus.init();
    encoder.init(spiBus.bus());

    Serial.println("Encoder raw test");
}

void loop() {
    encoder.update();

    if (millis() - lastPrintMs >= 50) {   // 20 Hz so you can read it clearly
        lastPrintMs = millis();

        // Serial.print("wrapped_deg = ");
        // Serial.print(encoder.angleDegWrapped(), 3);

        Serial.print(" | angle_rad = ");
        Serial.print(encoder.getSensorAngle(), 6);

        // Serial.print(" | vel = ");
        // Serial.println(encoder.getVelocity(), 6);
    }
}
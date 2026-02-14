    #include <Arduino.h>
    #include <SPI.h>
    #include <SimpleFOC.h>
    #include <SimpleFOCDrivers.h>
    #include "../headers/MagneticSensorMT6701SSI.h"
    #include "../headers/constants.h"
    #include <math.h>

    // #define SENSOR1_CS 5 // some digital pin that you're using as the nCS pin

    //4 pole pairs, 8 poles total
    // BLDCMotor motor = BLDCMotor(4); // set up for 4 pole pairs
    // BLDCDriver6PWM( int phA_h, int phA_l, int phB_h, int phB_l, int phC_h, int phC_l, int en)
    // BLDCDriver6PWM driver = BLDCDriver6PWM(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL, PIN_EN);

    MagneticSensorMT6701SSI encoder(PIN_ENC_CS); // Setup encoder 

    void setup() 
    {   
        Serial.begin(115200);

        SPI.begin(PIN_ENC_CLK, PIN_ENC_MISO, -1, PIN_ENC_CS); //(clk, miso, mosi, cs);

        encoder.init();

        Serial.println("Sensor initialized");

        // motor.linkSensor(&encoder);

        Serial.println("Motor initialized");

        delay(1000);
    }

    void loop() 
    {
        encoder.update(); // Update sensor values
    
        float angle = fmod(encoder.getAngle() * (180.0 / M_PI), 360.0);
        if (angle < 0) angle += 360.0;

        Serial.print("Angle: ");
        Serial.print(angle, 2); // Print angle with 2 decimal places
        Serial.print("\t");
        Serial.print("Velocity: ");
        Serial.println(encoder.getVelocity());

        
        delay(100);
    }
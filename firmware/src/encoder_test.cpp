#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "../headers/MagneticSensorMT6701SSI.h"
#include "../headers/constants.h"

#define SENSOR1_CS 5 // some digital pin that you're using as the nCS pin

//4 pole pairs, 8 poles total
BLDCMotor motor = BLDCMotor(4); // set up for 4 pole pairs
// BLDCDriver6PWM( int phA_h, int phA_l, int phB_h, int phB_l, int phC_h, int phC_l, int en)
BLDCDriver6PWM driver = BLDCDriver6PWM(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL, PIN_EN);

MagneticSensorMT6701SSI sensor1(SENSOR1_CS); // Setup encoder 

void setup() 
{
    Serial.begin(115200);
    //SimpleFOCDebug(&Serial);
    sensor1.init();
    Serial.println("Sensor initialized");
    delay(1000);
}

void loop() 
{
    sensor1.update(); // Update sensor values

    Serial.print(sensor1.getAngle());// print angle value of encoder 

    Serial.print("\t");

    Serial.print(sensor1.getVelocity()); // pritn velocity of sensor
}
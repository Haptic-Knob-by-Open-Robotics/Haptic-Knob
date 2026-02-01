#include <Arduino.h>
#include <SimpleFOC.h>
#include "../headers/constants.h"


//4 pole pairs, 8 poles total
BLDCMotor motor = BLDCMotor(4); // set up for 4 pole pairs
// BLDCDriver6PWM( int phA_h, int phA_l, int phB_h, int phB_l, int phC_h, int phC_l, int en)
BLDCDriver6PWM driver = BLDCDriver6PWM(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL, PIN_EN); 


#define SENSOR1_CS 5 // some digital pin that you're using as the nCS pin
MagneticSensorMT6701SSI sensor1(SENSOR1_CS);


void setup() {
    sensor1.init();
}

void loop() 
{

}
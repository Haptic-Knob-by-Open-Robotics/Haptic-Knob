#include <SimpleFOC.h>
#include "../headers/constants.h"


BLDCDriver6PWM driver = (PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL, PIN_EN);

void setup() {
    // use monitoring with seriol 
    Serial.begin(115200);

    
    // enable more verbose output for debugging 
    SimpleFOCDebug::enable(&Serial); 

    // Driver Voltages
    driver.voltage_limit = 5;
    driver.voltage_power_supply = 3.3;
    
    if (!driver.init()){
        Serial.println("Initialize the driver now!");
        return;
    }

    // enable the driver 
    driver.enable();
    Serial.println("Driver is ready to go!");

    _delay(1000);
}

void loop() {
    // setting pwm (three diff phases)
    driver.setPwm(3,4,5);
}
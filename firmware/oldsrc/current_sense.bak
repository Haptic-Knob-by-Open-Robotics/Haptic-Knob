#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "../headers/MagneticSensorMT6701SSI.h"
#include "../headers/constants.h"

// InlineCurrentSensor constructor
//  - shunt_resistor  - shunt resistor value
//  - gain  - current-sense op-amp gain
//  - phA   - A phase adc pin
//  - phB   - B phase adc pin
//  - phC   - C phase adc pin (optional)
// Make sure to change the values to match ours 
InlineCurrentSense current_sense  = InlineCurrentSense(0.01, 20, A0, A1, A2);


// initialise the current sense 
if (current_sense.init()) Serial.println("Current sense init success!");
else{
    Serial.println("Current sense init failed!");
    return; 
}

// Now we can start measuring the currents!
// Enable debugging 
Serial.begin(115200); // to output the debug information to the serial
SimpleFOCDebug::enable(&Serial);

void setup(){

    //link current sense
    current_sense.linkDriver(&driver);

}
#include <Arduino.h>
#include <SimpleFOC.h>
#include "../drivers/ModifiedMagneticSensorMT6701SSI.h"
#include "../app/config.h"

BLDCMotor motor = BLDCMotor(4);
BLDCDriver6PWM driver = BLDCDriver6PWM(PIN_VH, PIN_VL, PIN_UH, PIN_UL, PIN_WH, PIN_WL, PIN_EN);  // Swapped A and B

void setup() {
    Serial.begin(115200);

    driver.voltage_power_supply = 12;
    driver.voltage_limit =  12;
    driver.dead_zone = 0.02f;

      if (!driver.init()){
        Serial.println("Driver init failed!");
        return;
    }

    // enable driver
    driver.enable();
    Serial.println("Driver ready!");
    _delay(1000);

}

void loop() {
    // setting pwm
    // phase A: 3V
    // phase B: 6V
    // phase C: 5V
    driver.setPwm(3,6,5);
}
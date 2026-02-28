
#include <Arduino.h>
#include <SimpleFOC.h>
#include "drivers/MagneticSensorMT6701SSI.h"
#include "../app/config.h"

BLDCMotor motor = BLDCMotor(7);  

BLDCDriver6PWM driver = BLDCDriver6PWM(
    PIN_VH, PIN_VL,
    PIN_UH, PIN_UL,
    PIN_WH, PIN_WL,
    PIN_EN
);

MagneticSensorMT6701SSI encoder(PIN_ENC_CS);

void setup() {
    Serial.begin(115200);
    delay(3000);
    
    
    encoder.init();
    motor.linkSensor(&encoder);
    
    driver.pwm_frequency = 30000;
    driver.voltage_power_supply = 12;
    driver.init();
    motor.linkDriver(&driver);
    
    motor.useMonitoring(Serial);
    motor.init();
    Serial.println("\n\n=== TEST COMPLETE ===");
}

void loop() {
    // Empty
}
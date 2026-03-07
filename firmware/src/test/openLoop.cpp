#include <Arduino.h>
#include <SimpleFOC.h>
#include "app/Config.h"
#include "drivers/ModifiedMagneticSensorMT6701SSI.h"
#include "drivers/SpiBus.h"

BLDCMotor motor = BLDCMotor(4);
BLDCDriver6PWM driver = BLDCDriver6PWM(PIN_VH, PIN_VL, PIN_UH, PIN_UL, PIN_WH, PIN_WL, PIN_EN);  // Swapped A and B
ModifiedMagneticSensorMT6701SSI encoder(PIN_ENC_CS);
Commander command = Commander(Serial);

void doTarget(char* cmd) { 
    command.scalar(&motor.target, cmd); 
}

void setup() {
    Serial.begin(115200);
    delay(3000);
    
    Serial.println("\nTEST WITH SWAPPED PHASES A-B\n");
    
    encoder.init();
    motor.linkSensor(&encoder);
    
    driver.pwm_frequency = 30000;
    driver.voltage_power_supply = 12;
    driver.init();
    motor.linkDriver(&driver);
    
    motor.controller = MotionControlType::velocity_openloop;
    motor.voltage_limit = 2.0;
    
    motor.init();
    
    command.add('T', doTarget, "velocity");
    
    Serial.println("OPEN LOOP TEST");
    Serial.println("T2");
}

void loop() {
    motor.loopFOC();
    motor.move();
    command.run();
}
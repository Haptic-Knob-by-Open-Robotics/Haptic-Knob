#include <Arduino.h>
#include <SimpleFOC.h>
#include "../headers/MagneticSensorMT6701SSI.h"
#include "../headers/constants.h"

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
    
    Serial.println("\nZERO ANGLE SEARCH\n");
    
    encoder.init();
    motor.linkSensor(&encoder);
    
    driver.pwm_frequency = 30000;
    driver.voltage_power_supply = 12;
    driver.init();
    motor.linkDriver(&driver);
    
    motor.controller = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::voltage;
    motor.voltage_limit = 3.0;
    motor.sensor_direction = Direction::CCW;
    
    motor.init();
    
    Serial.println("Searching 0 to 6.28 in 0.2 steps");
    
    for(float zero = 0; zero < 6.28; zero += 0.2) {
        motor.zero_electric_angle = zero;
        motor.target = 1.0;
        
        Serial.print("Testing zero = ");
        Serial.print(zero, 2);
        
        // Measure current draw
        delay(200);  // stabilization
        
        // Check if position holds
        float start = encoder.getAngle();
        for(int i = 0; i < 100; i++) {
            motor.loopFOC();
            motor.move();
            delay(5);
        }
        float end = encoder.getAngle();
        float drift = abs(end - start);
        
        Serial.print(" | Drift: ");
        Serial.print(drift, 3);
        
        if(drift < 0.1) {
            Serial.println("EXCELLENT");
        } else if(drift < 0.3) {
            Serial.println("GOOD");
        } else if(drift < 1.0) {
            Serial.println("OK");
        } else {
            Serial.println("Poor");
        }
        
        motor.target = 0;
        delay(300);
    }
    
    Serial.println("\nDONE");
}

void loop() {
    // Empty
}

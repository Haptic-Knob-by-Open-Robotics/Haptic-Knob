#include <Arduino.h>
#include <SimpleFOC.h>
#include "../headers/MagneticSensorMT6701SSI.h"
#include "../headers/constants.h"

BLDCMotor motor = BLDCMotor(7);  // 7 pole pairs 

BLDCDriver6PWM driver = BLDCDriver6PWM(
    PIN_VH, PIN_VL,
    PIN_UH, PIN_UL,
    PIN_WH, PIN_WL,
    PIN_EN
);

MagneticSensorMT6701SSI encoder(PIN_ENC_CS);
Commander command = Commander(Serial);

void doTarget(char* cmd) { 
    command.scalar(&motor.target, cmd);
    Serial.print("Target: ");
    Serial.print(motor.target, 2);
    Serial.println(" V");
}

void setup() {
    Serial.begin(115200);
    delay(3000);
    
    Serial.println("\nFOC VOLTAGE CONTROL\n");
    
    encoder.init();
    motor.linkSensor(&encoder);
    
    driver.pwm_frequency = 30000;
    driver.dead_zone = 0.05;
    driver.voltage_power_supply = 12;
    driver.voltage_limit = 12;
    driver.init();
    motor.linkDriver(&driver);
    
    motor.useMonitoring(Serial);
    motor.init();
    
    // Auto calibrate zero angle
    Serial.println("Calibrating zero angle...");
    motor.controller = MotionControlType::velocity_openloop;
    motor.voltage_limit = 2.0;
    motor.target = 2.0;
    
    for(int i = 0; i < 150; i++) {
        motor.move();
        delay(10);
    }
    
    float shaft_angle = motor.shaft_angle;
    motor.target = 0;
    for(int i = 0; i < 50; i++) {
        motor.move();
        delay(10);
    }
    
    float zero_elec = _normalizeAngle(_electricalAngle(shaft_angle, motor.pole_pairs));
    
    // Configure FOC voltage mode
    motor.controller = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::voltage;
    motor.voltage_limit = 3.0;
    
    motor.zero_electric_angle = zero_elec;
    motor.sensor_direction = Direction::CCW;  // CCW 
    
    Serial.print("Zero angle: ");
    Serial.println(zero_elec, 2);
    Serial.println("Sensor direction: CCW");
    Serial.println("Pole pairs: 7");
    
    command.add('T', doTarget, "voltage");
    
    Serial.println("\n=== MOTOR READY ===");
    Serial.println("Commands:");
    Serial.println("  T0.5  - Apply 0.5V");
    Serial.println("  T1    - Apply 1V");
    Serial.println("  T2    - Apply 2V");
    Serial.println("  T0    - Release");
}

void loop() {
    motor.loopFOC();
    motor.move();
    command.run();
    
    static unsigned long last = 0;
    if(millis() - last > 1000) {
        Serial.print("Target: ");
        Serial.print(motor.target, 2);
        Serial.print(" V | Angle: ");
        Serial.print(encoder.getAngle(), 2);
        Serial.print(" | Velocity: ");
        Serial.println(encoder.getVelocity(), 2);
        last = millis();
    }
}
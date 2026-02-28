// #include <Arduino.h>
// #include <SimpleFOC.h>
// #include "../headers/MagneticSensorMT6701SSI.h"
// #include "../headers/constants.h"

// BLDCMotor motor = BLDCMotor(4);  
// BLDCDriver6PWM driver = BLDCDriver6PWM(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL, PIN_EN);
// MagneticSensorMT6701SSI encoder(PIN_ENC_CS);
// Commander command = Commander(Serial);

// void doTarget(char* cmd) { 
//     command.scalar(&motor.target, cmd); 
// }

// void setup() {
//     Serial.begin(115200);
//     delay(3000);
    
//     Serial.println("\nPOSITION CONTROL MODE\n");
    
//     // 1. Encoder init
//     Serial.print("Encoder init... ");
//     encoder.init();qw
//     motor.linkSensor(&encoder);
//     Serial.println("done");
    
//     // 2. Driver init
//     Serial.print("Driver init... ");
//     driver.pwm_frequency = 30000;
//     driver.dead_zone = 0.05;
//     driver.voltage_power_supply = 12;
//     driver.voltage_limit = 12;
//     driver.init();
//     motor.linkDriver(&driver);
//     Serial.println("done");
    
//     // 3. POSITION CONTROL configuration
//     motor.controller = MotionControlType::angle;  // Position/angle control
//     motor.torque_controller = TorqueControlType::voltage;
    
//     // Position PID tuning
//     motor.P_angle.P = 20;      // Proportional gain
//     motor.P_angle.I = 0;       // Integral gain (usually 0 for position)
//     motor.P_angle.D = 1;       // Derivative gain (damping)
//     motor.P_angle.output_ramp = 1000;  // Max change rate
    
//     // Velocity PID (inner loop)
//     motor.PID_velocity.P = 0.2;
//     motor.PID_velocity.I = 20;
//     motor.PID_velocity.D = 0;
//     motor.LPF_velocity.Tf = 0.01;
    
//     // Limits
//     motor.voltage_limit = 3.0;
//     motor.velocity_limit = 20;  // Max velocity in rad/s
    
//     motor.useMonitoring(Serial);
    
//     // 4. Motor init
//     Serial.print("Motor init... ");
//     motor.init();
//     Serial.println("done");
    
//     // 5. FOC init manual settings to avoid hang
//     Serial.print("FOC setup... ");
//     motor.zero_electric_angle = 0;
//     motor.sensor_direction = Direction::CW;  // Change to CCW if needed
//     Serial.println("done");
    
//     command.add('T', doTarget, "target angle");
    
//     Serial.println("\n=== MOTOR READY ===");
//     Serial.println("Commands (angles in radians):");
//     Serial.println("  T0     - Go to 0° (home)");
//     Serial.println("  T1.57  - Go to 90° (π/2)");
//     Serial.println("  T3.14  - Go to 180° (π)");
//     Serial.println("  T6.28  - Go to 360° (2π)");
//     Serial.println("  T-1.57 - Go to -90°");
//     Serial.println("\nMotor will hold position");
// }

// void loop() {
//     motor.loopFOC();
//     motor.move();
//     command.run();
// }

#include <Arduino.h>
#include <SimpleFOC.h>
#include "../headers/MagneticSensorMT6701SSI.h"
#include "../headers/constants.h"

BLDCMotor motor = BLDCMotor(7);
BLDCDriver6PWM driver = BLDCDriver6PWM(PIN_VH, PIN_VL,PIN_UH, PIN_UL, PIN_WH, PIN_WL, PIN_EN);
MagneticSensorMT6701SSI encoder(PIN_ENC_CS);

void setup() {
    Serial.begin(115200);
    delay(3000);
    
    Serial.println("\n=== ZERO ANGLE FINDER ===\n");
    
    encoder.init();
    motor.linkSensor(&encoder);
    
    driver.pwm_frequency = 30000;
    driver.voltage_power_supply = 12;
    driver.init();
    motor.linkDriver(&driver);
    
    motor.controller = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::voltage;
    motor.voltage_limit = 1.5;
    
    motor.init();
    
    Serial.println("Testing different zero angles...");
    Serial.println("LISTEN/WATCH - smooth = correct angle\n");
    
    // Test different pole pairs
    int pole_pairs[] = {4, 7, 11, 14};
    
    for(int pp_idx = 0; pp_idx < 4; pp_idx++) {
        motor.pole_pairs = pole_pairs[pp_idx];
        
        Serial.print("\n=== TESTING ");
        Serial.print(motor.pole_pairs);
        Serial.println(" POLE PAIRS ===\n");
        
        // Test CW direction with different zero angles
        motor.sensor_direction = Direction::CW;
        
        for(float zero_angle = 0; zero_angle < 6.28; zero_angle += 0.785) {  // 45° steps
            motor.zero_electric_angle = zero_angle;
            motor.target = 1.0;
            
            Serial.print("PP=");
            Serial.print(motor.pole_pairs);
            Serial.print(", DIR=CW, ZERO=");
            Serial.print(zero_angle, 2);
            Serial.print(" ... ");
            
            // Run for 1 second
            for(int i = 0; i < 100; i++) {
                motor.loopFOC();
                motor.move();
                delay(10);
            }
            
            Serial.println("(did it run smooth?)");
            
            motor.target = 0;
            delay(500);
        }
        
        // Test CCW direction
        motor.sensor_direction = Direction::CCW;
        
        for(float zero_angle = 0; zero_angle < 6.28; zero_angle += 0.785) {
            motor.zero_electric_angle = zero_angle;
            motor.target = 1.0;
            
            Serial.print("PP=");
            Serial.print(motor.pole_pairs);
            Serial.print(", DIR=CCW, ZERO=");
            Serial.print(zero_angle, 2);
            Serial.print(" ... ");
            
            for(int i = 0; i < 100; i++) {
                motor.loopFOC();
                motor.move();
                delay(10);
            }
            
            Serial.println("(did it run smooth?)");
            
            motor.target = 0;
            delay(500);
        }
    }
    
    Serial.println("\n\n=== TEST COMPLETE ===");
    Serial.println("Which configuration ran smoothly without vibration?");
    Serial.println("watch the pole_pairs, direction, and zero_angle values.");
}

void loop() {
    // Empty
}

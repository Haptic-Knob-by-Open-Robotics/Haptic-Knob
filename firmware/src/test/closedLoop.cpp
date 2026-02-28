#include <Arduino.h>
#include <SimpleFOC.h>
#include "../drivers/newMagneticSensorMT6701SSI.h"
#include "../app/config.h"

// 
static constexpr float SHUNT_RESISTOR = 0.012f;  // 12 mΩ
static constexpr float AMP_GAIN       = 50.0f;
//

BLDCMotor motor = BLDCMotor(4);  
//7 pole pairs worked better in reality for unknown reasons

BLDCDriver6PWM driver = BLDCDriver6PWM(
    PIN_VH, PIN_VL,
    PIN_UH, PIN_UL,
    PIN_WH, PIN_WL,
    PIN_EN
);
//flipped due to hardware wiring (U and V swapped)

 MT6701SensorCustom encoder(PIN_ENC_CS);

//inline current senssing
InlineCurrentSense current_sense(
    SHUNT_RESISTOR,
    AMP_GAIN,
    PIN_I_A, PIN_I_B, PIN_I_C
);



void setup() {
    Serial.begin(115200);
    delay(3000);
    Serial.println("\n=== SIMPLE CLOSED-LOOP FOC TEST (VOLTS TORQUE) ===")
    
    //encoder init
    encoder.init();
    motor.linkSensor(&encoder);
    Serial.println("Encoder initialized")

    //driver init
    driver.pwm_frequency = 30000;
    driver.voltage_power_supply = 12;
    driver.init();
    motor.linkDriver(&driver);

    //current sense init
    current_sense.linkDriver(&driver);
      if (!current_sense.init()) {
        Serial.println("Current sense init FAILED");
        while (1) {
            Serial.println("HALT: current sense init failed");
            delay(1000);
        }
    }
    Serial.println("Current sense initialized");

    // set torque control mode
    motor.torque_controller = TorqueControlType::foc_current;
    //motion control mode
    motor.controller = MotionControlType::torque;

    //foc current control parameters
    motor.PID_current_q.P  = 5;
    motor.PID_current_q.I  = 300;
    motor.PID_current_q.D  = 0;

    motor.PID_current_d.P  = 5;
    motor.PID_current_d.I  = 300;
    motor.PID_current_d.D  = 0;

    motor.LPF_current_q.Tf = 0.01;
    motor.LPF_current_d.Tf = 0.01;

    motor.useMonitoring(Serial);

    //motor init
    motor.init();
    Serial.println("\n\n=== TEST COMPLETE ===");

    command.add('T', doTarget, "target current (Iq A)");

    Serial.println("Motor ready.");
    Serial.println("Set the target current using serial terminal, e.g.:");
    Serial.println("  T 0.1");
}

void loop() {
    motor.loopFOC();
    motor.move();
    command.run();
    
}
#include <Arduino.h>
#include <SimpleFOC.h>

// Pin declarations
#define PIN_UH 25
#define PIN_UL 33
#define PIN_VH 27
#define PIN_VL 26
#define PIN_WH 12
#define PIN_WL 14
#define PIN_EN 32

//4 pole pairs, 8 poles total
BLDCMotor motor = BLDCMotor(4); // set up for 4 pole pairs
// BLDCDriver6PWM( int phA_h, int phA_l, int phB_h, int phB_l, int phC_h, int phC_l, int en)
BLDCDriver6PWM driver = BLDCDriver6PWM(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL, PIN_EN); 

//instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }
void doLimitCurrent(char* cmd) { command.scalar(&motor.current_limit, cmd); }

// Max Voltage: 6-8V (7.4V nominasl, use 6V for safety)
// 5-10% PWM Duty Cycle (to be safe)

void setup() {
  
  // PWM frequency
  driver.pwm_frequency = 30000;

  // dead_zone [0,1] - default 0.02 - 2%
  driver.dead_zone = 0.05; // default set up
  // dead_time = 1/driver.pwm_frequency*driver.dead_zone;

  // Driver Voltages
  driver.voltage_limit = 5;
  driver.voltage_power_supply = 3.3;
   
  // limiting voltage 
  motor.voltage_limit = 3;   // Volts
  // or current  - if phase resistance provided
  motor.current_limit = 0.5; // Amps

  //linking motor to driver

  Serial.print("Driver init ");
  // init driver
  if (driver.init())  Serial.println("success!");
  else{
    Serial.println("failed!");
    return;
}
    motor.linkDriver(&driver);
   motor.controller = MotionControlType::velocity_openloop;
    
  motor.init();
  motor.initFOC();


  command.add('T', doTarget, "target velocity");
  command.add('C', doLimitCurrent, "current limit");
  _delay(1000);


}
  
float target_voltage = 2;

void loop() {
  // put your main code here, to run repeatedly:
  
  //FOC algorithm execution
  motor.loopFOC();
  motor.move(target_voltage);
  command.run();
}
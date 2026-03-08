#include <Arduino.h>
#include <SimpleFOC.h>
#include "app/Config.h"
#include "drivers/ModifiedMagneticSensorMT6701SSI.h"
#include "drivers/SpiBus.h"

// ================== HIGH LEVEL SUMMARY ===============
// 
// In this file we're going to implement the resistor mode for the haptic knob.
// The idea of the workflow is:
// 1. Read the shaft angle / velocity from the magnetic encoder
// 2. Then use the measured shaft velocity to compute a resistive toruqe by doing: 
//        tau = -B * omega
//    where tau is the torque we want the motor to apply back to the user 
//    B is the resistance we're using for our model (this constant is set by us for now)
//    omega is the measured angular velocity from the user turning the knob
//  
// 3. Afterwards, convert that desired torque inot a desired motor current (this is the current set point)
//        Iq_desired = tau / kt (the motor torque is proportional to torque producing current. Kt is torque constant retrieved from motor data sheet)
// 4. Run the FOC current loop so the motor's REAL current (measured by the current sensor) matches the current we want (which is essentially the torque feedback)
// 5. The motor then should generate the corresponding torque, and the user feels resistance when turning the knob 


// =================== HARDWARE OBJECTS =============

// Shared SPI bus used to talk to the magnetic encoder 
SpiBus spiBus(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI); 

// Encoder object 
ModifiedMagneticSensorMT6701SSI encoder(PIN_ENC_CS);

// Current sense object: This measures the motor phase currents (recall using 3 phase motor) using our shunt resistors & amplifiers. 
// These measured currents will later allow the current control loop to do the magic!
InlineCurrentSense current_sense(SHUNT_RESISTOR, AMP_GAIN, PIN_I_A, PIN_I_B, PIN_I_C); 


// 6PWM BLDC driver object 
// This represents the inverter / gate-driving state that will produce the three motor phase voltages using PWM 
BLDCDriver6PWM driver(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL); 

// Motor object
// This i sthe high level motor model which will be used by the SimpleFOC library 
BLDCMotor motor(POLE_PAIRS); 


// ================== CONTROL PARAMETERS ==============

// The virtual damping constant for our resistor mode. Bigger -> harder to turn 
static constexpr float RESISTANCE = 0.020f; 

// Motor torque constant Kt
static constexpr float TORQUE_CONST = 0.035f; 

// Safety limits 
// we need to prevent the system from sending too much torque/currnet or else andys wrist could potentially break
static constexpr float MAX_TORQUE = 0.12f;
static constexpr float MAX_CURRENT = 2.50f;

// How often to print debug info 
static constexpr uint32_t PRINT_PERIOD_MS = 100; 
// need to track last print 
static uint32_t lastPrint = 0; 



// ================ HELPERS ============

// Clamp the torque/corrent commands so that they stay in safe range 
static float clampf(float val, float min, float max) {
    if (val < min) val = min; 
    if (val > max) val = max; 
    return val; 
}

// Debug print function 
// We can observe : current shaft angle, shaft velocity, commanded torque, commanded current, measured phase currents 
void printDebugData(float torqueCmd, float iqCmd) {
    // Read current measured mechanical state from our encoder 
    float angleDeg = encoder.angleDegWrapped(); 
    float angleRad = encoder.getAngle(); 
    float velocity = encoder.getVelocity(); 

    // Read phase currents from the current sense hardware
    PhaseCurrent_s currents = current_sense.getPhaseCurrents(); 
    Serial.printf(
        "Angle: %7.2f deg | AngleRad: %7.3f | Vel: %7.3f rad/s | "
        "TorqueCmd: %7.4f N*m | IqCmd: %6.3f A | "
        "Ia: %6.3f A | Ib: %6.3f A | Ic: %6.3f A\n",
        angleDeg,
        angleRad,
        velocity,
        torqueCmd,
        iqCmd,
        currents.a,
        currents.b,
        currents.c
    );
    
}


// ====================== DRIVER SETUP ====================

// This configures the PWM inverter / motor driver stage
// If you refer to the full block diagram of the ssystem, this is the block that takes control commands form 
// the ESP32 and turns them into PWM switching signals for the motor phases 
bool driverSetup() {

    // Set PWM frequency for the inverter 
    // Note that a higher PWM frequency generally gives smoother operation and keeps switching noise above audible range, 
    // but it increases switching losses 
    driver.pwm_frequency = 30000; // 30 khz

    // Dead-zone
    driver.dead_zone = 0.05f; 

    // Supply voltage available to the driver 
    driver.voltage_power_supply = VOLTAGE_SUPPLY; 

    // Maximum voltage the controller is allowed to apply 
    driver.voltage_limit = VOLTAGE_LIMIT; 

    Serial.print("Initializing motor driver  "); 

    bool success = driver.init();
    if (!success) {
        Serial.println("FAILED initializing driver"); 
        return false; 
    }

    // enable the driver so PWM outputs can actually drive the inverter 
    driver.enable(); 

    Serial.println("SUCCESSFUL driver setup");
    return true; 
}


// ======================== CURRENT SENSE SETUP =================

// Remember we need this because torque is proportional to current, so if we want accurate torque control, we need accurate current feedback 
bool currentSenseSetup() {

    // Tell the current-sense object which driver it belongs to. This helps SIMPLEFOC understand the phase relationship
    current_sense.linkDriver(&driver); 

    Serial.print("Initializing current sense  "); 
    bool success = current_sense.init(); 
    if (!success){
        Serial.println("Failed initializing current sense");
        return false; 
    }

    // Align the current sense system with the driver (this helps determine correct phase/sign conventions)
    Serial.print("Aligning current sense w driver");
    int align_ok = current_sense.driverAlign(VOLTAGE_LIMIT); 
    if(!align_ok){
        Serial.println("FAILED");
        return false; 
    }

    Serial.println("SUCCESSFUL current sense setup");
    return true; 
}


// ========================= MOTOR / FOC SETUP ================

// This function will configure the motor object and also tell SimpleFOC which driver to use
// which encoder to use, which current sensor to use and what control mode we want. 
bool motorSetup() {

    // Connect motor object to the hardware pieces
    motor.linkDriver(&driver);
    motor.linkSensor(&encoder);
    motor.linkCurrentSense(&current_sense); 

    // High-level motion mode: torque control. We set it to torque since our model computes desired torqu
    motor.controller = MotionControlType::torque; 

    // Low-level torque implementaiton: current controlled FOC (toruqe is propotional to q-axis current so we set this to foc_current)
    motor.torque_controller = TorqueControlType::foc_current; 

    // Safety limits 
    motor.voltage_limit = VOLTAGE_LIMIT; 
    motor.current_limit = MAX_CURRENT; 

    // Current loop PID tuning
    // Note that these values are just put in place for now to replace foc default values as simplefoc default values
    // are meant to be replaced and fine tuned so change these as you wish. 
    // note that q-axis current -> torque producing current
    //           d-axis current -> usually kept near zero for normal FOC operations 
    motor.PID_current_q.P = 0.5f;
    motor.PID_current_q.I = 5.0f;
    motor.PID_current_q.D = 0.0f;
    motor.PID_current_q.limit = VOLTAGE_LIMIT;

    motor.PID_current_d.P = 0.5f;
    motor.PID_current_d.I = 5.0f;
    motor.PID_current_d.D = 0.0f;
    motor.PID_current_d.limit = VOLTAGE_LIMIT;

    

    // Low-pass filters on measured currents (this helps reduce the noise in the current loop)
    motor.LPF_current_q.Tf = 0.002f; // change these as u wish 
    motor.LPF_current_d.Tf = 0.002f; 

    Serial.print("Initializing motor object  ");
    bool ok = motor.init();
    if (!ok) {
        Serial.println("FAILED");
        return false;
    }

    // initFOC performs electrical angle alignment and starts the
    // closed-loop FOC control structure.
    Serial.print("Initializing FOC  ");
    ok = motor.initFOC();
    if (!ok) {
        Serial.println("FAILED");
        return false;
    }
    Serial.println("SUCCESSFUL Motor & FOC initialization");

    return true;
}   


// ========================== SETUP =================

// setup() runs once at startup. This is where we bring the whole system online in the right order
// 1. Start serial output
// 2. Start SPI bus
// 3. Start encoder
// 4. Start motor driver 
// 5. Start current sense
// 6. Start motor + FOC 
void setup() {
    Serial.begin(115200); 

    // Small delay so serial monitor has time to connect 
    delay(2000); 

    Serial.println();
    Serial.println("==============================================");
    Serial.println("Haptic Knob - Resistor Mode");
    Serial.println("==============================================");


    Serial.println("Starting system setup");
    // 1. Initialize SPI bus and encoder 
    spiBus.init(); 
    encoder.init(spiBus.bus()); 

    // do an update so initial values become available 
    encoder.update(); 

    // 2. Initialize motor driver
    if (!driverSetup()) {
        Serial.println("Driver setup failed. Halting.");
        while (true) { }
    }

    // 3. Initialize current-sense path 
    if (!currentSenseSetup()) {
        Serial.println("Current sense setup failed. Halting.");
        while (true) { }
    }
    
    // 4. Initialize motor + FOC control 
    if (!motorSetup()) {
        Serial.println("Motor setup failed. Halting.");
        while (true) { }
    }

    Serial.println("System ready.");
    Serial.println();
}


// =================== MAIN LOOP ====================
//loop() runs over and over until we stop 
// This is the real-time control loop of the haptic system 
void loop() {

    // 1. Update encoder measurements 
    // This reads the shaft position and velocity from the encoder. We need to contiously update this 
    // as we need fresh measurements every loop as the use could be turning the loop continously
    encoder.update(); 

    // 2. Run low-level FOC control update 
    // This is the inner electrical control loop. This part reads motor electrical state, uses current feedback
    // updates voltage / modulation commands, and eventually produces PWM duty cycles for the inverter 
    motor.loopFOC(); 

    // 3. Read measured mechanical velocity. Note that positive velocity is in one direction and negative velocity is in the other direction 
    float omega = encoder.getVelocity(); 

    // 4. Compute the virtual torque from resistor model 
    float torqueCmd = -RESISTANCE * omega; // note negative is there since it has to be in the opposite direction of the velocity 

    // Clamp the torque command 
    torqueCmd = clampf(torqueCmd, -MAX_TORQUE, MAX_TORQUE); 

    // 5. Convert desired torque into desired q-axis current 
    float iqCmd = torqueCmd / TORQUE_CONST; 

    // clamp current command 
    iqCmd = clampf(iqCmd, -MAX_CURRENT, MAX_CURRENT);

    // 6. Send desired current to motor controller 
    //  Note that Since we configured:
    //   motor.controller = torque
    //   motor.torque_controller = foc_current
    // SimpleFOC interprets this command as the torque-producing current target, and the current loop will try to make actual current match it.
    // so desired torque -> desired current -> current PID -> voltage command -> PWM -> motor phases -> actual torque feedback 
    motor.move(iqCmd); 

    // 7. Telemetry output 
    uint32_t now = millis(); 
    if (now - lastPrint >= PRINT_PERIOD_MS){
        lastPrint = now; 
        printDebugData(torqueCmd, iqCmd); 
    }

}
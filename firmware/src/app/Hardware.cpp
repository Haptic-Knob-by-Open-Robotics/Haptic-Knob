#include "app/Hardware.h"

#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>

#include "app/Config.h"
#include "drivers/ModifiedMagneticSensorMT6701SSI.h"

namespace
{

bool setupCurrentSense();
bool setupDriver();
bool setupMotor();

// ========== GLOBAL HARDWARE OBJECTS ==========
// These are the core hardware/control objects used by the system.
//
// encoder:
//   Magnetic shaft encoder used to measure the motor/knob angle.
//
// current_sense:
//   Reads motor phase currents through the inline current sensing
//   hardware so FOC current control can work.
//
// driver:
//   6-PWM BLDC gate driver interface. This is what actually outputs
//   the PWM signals to the motor driver hardware.
//
// motor:
//   SimpleFOC motor object that performs the control-side logic
//   (FOC, torque/current control, velocity estimation, etc.).
//
// These are placed in an anonymous namespace so they stay local
// to this translation unit (this .cpp file).
ModifiedMagneticSensorMT6701SSI encoder(PIN_ENC_CS);

InlineCurrentSense current_sense(
    SHUNT_RESISTOR_OHM,
    CURRENT_SENSE_AMP_GAIN,
    PIN_I_A,
    PIN_I_B,
    PIN_I_C);

BLDCDriver6PWM driver(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL);

BLDCMotor motor(POLE_PAIRS);
}

// ========= TOP-LEVEL HARDWARE INIT =========
// This function is called once during startup.
//
// High-level flow:
//   1. start SPI bus
//   2. initialize encoder
//   3. initialize motor driver
//   4. initialize current sensing
//   5. initialize motor + FOC
//
// If any required step fails, we return false so the rest of the
// system knows hardware is not safe/ready to use.
//
bool initHardware()
{
    Serial.println("Starting hardware initialization...");

    // Start the SPI bus using the board-specific pin mapping from Config.h.
    //
    // We pass the encoder chip-select pin as the SS argument so the SPI
    // peripheral is configured consistently with our setup.
    SPI.begin(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_ENC_CS);

    // Initialize the encoder and attach it to the SPI bus we just started.
    // After this, the encoder object can begin reading shaft angle data.
    encoder.init(&SPI);

    // Bring up the power driver first.
    // If this fails, nothing else should continue.
    if (!setupDriver())
    {
        return false;
    }

    // Then initialize current sensing.
    // This must work properly for FOC current control to behave correctly.
    if (!setupCurrentSense())
    {
        return false;
    }

    // Finally initialize the motor object itself, including FOC startup.
    if (!setupMotor())
    {
        return false;
    }

    Serial.println("Hardware initialization complete.");
    return true;
}

// ============ FAST HARDWARE CONTROL-STEP UPDATE ===========
// This function is meant to be called from the fast control loop.
// What happens here:
//
// 1. encoder.update()
//    Refresh the encoder reading so the control loop has the most
//    recent shaft position.
//
// 2. motor.loopFOC()
//    Run the low-level FOC update using the latest sensor/current data.
//
// This is intentionally small because it belongs in the time-sensitive
// control loop.
//
void updateHardwareControlStep()
{
    encoder.update();
    motor.loopFOC();
}

// =========== MOTOR COMMAND / SAFE STOP HELPERS =========
// Apply a q-axis current request to the motor.
// Since the motor has already been configured for torque control
// using foc_current mode, motor.move(iq_cmd) here means:
// try to produce this torque-producing current
void applyMotorCurrent(float iq_cmd)
{
    motor.move(iq_cmd);
}

// Safe stop helper used when control is disabled or a fault occurs.
//
// Here we simply command zero current.
void stopMotor()
{
    motor.move(0.0f);
}

// ======== MOTOR / SENSOR STATE ACCESSORS =========
// These helper functions provide a clean interface to the rest
// of the application so higher-level code does not need to reach
// directly into the raw hardware objects.

// Return the current shaft angle in radians.
float getMotorAngleRad()
{
    return encoder.getAngle();
}

// Return the shaft angle in wrapped degrees.
//
// This is often more convenient for display/telemetry/debugging
// because it stays in a bounded angle range rather than growing forever.
float getMotorAngleDegWrapped()
{
    return encoder.angleDegWrapped();
}

// Return the motor shaft velocity in rad/s.
//
// This value is estimated/maintained by the SimpleFOC motor object.
float getMotorVelocityRad()
{
    return motor.shaftVelocity();
}

// Return the measured q-axis current.
//
// q-axis current is especially important because in FOC torque control,
// it is the current component most directly related to produced torque.
float getMeasuredIq()
{
    DQCurrent_s dq = current_sense.getFOCCurrents(encoder.getAngle());
    return dq.q;
}

// Return the three measured phase currents.
//
// This is useful for debugging, telemetry, and checking whether
// the current sensing hardware is behaving properly.
PhaseCurrent_s getPhaseCurrents()
{
    return current_sense.getPhaseCurrents();
}

// ========  RUNTIME PID GAIN UPDATE HELPER ==========
//
// Allows telemetry/debug code to tune the q-axis and d-axis current
// controller gains at runtime without recompiling.
//
// q-axis controller:
//   Controls torque-producing current
//
// d-axis controller:
//   Controls the orthogonal current component that is generally
//   regulated toward zero in many FOC setups
//
void setCurrentPidGains(float qp, float qi, float qd,
                        float dp, float di, float dd)
{
    motor.PID_current_q.P = qp;
    motor.PID_current_q.I = qi;
    motor.PID_current_q.D = qd;

    motor.PID_current_d.P = dp;
    motor.PID_current_d.I = di;
    motor.PID_current_d.D = dd;
}

namespace
{

// ========== CURRENT SENSE INITIALIZATION =========
//
// This function sets up inline current sensing and aligns it with
// the motor driver.
//
// Why this matters:
// FOC current control depends on accurate current measurements.
// If current sense is not initialized or aligned correctly,
// torque/current control will not behave properly.
//
bool setupCurrentSense()
{
    // Tell the current sense object which driver it is associated with.
    // This lets SimpleFOC coordinate current sensing with the driver state.
    current_sense.linkDriver(&driver);

    Serial.print("Initializing current sense... ");
    if (!current_sense.init())
    {
        Serial.println("FAILED");
        return false;
    }

    // Align the sensed current channels with the driver/phases.
    //
    // This step helps SimpleFOC determine whether the current sensing
    // orientation/sign mapping is correct for the configured hardware.
    Serial.print("Aligning current sense... ");
    if (!current_sense.driverAlign(DRIVER_VOLTAGE_LIMIT_V))
    {
        Serial.println("FAILED");
        return false;
    }

    Serial.println("OK");
    return true;
}

// =========  DRIVER INITIALIZATION =========
//
// This configures the 6-PWM BLDC driver object.
//
// Key settings:
// pwm_frequency:
//   PWM switching frequency for the motor phases.
//
// dead_zone:
//   Dead time / dead zone to reduce shoot-through risk in complementary
//   switching situations.
//
// voltage_power_supply:
//   Actual DC supply voltage available to the driver.
//
// voltage_limit:
//   Software-imposed voltage limit used by the control library.
//
bool setupDriver()
{
    // Set PWM switching frequency.
    // 30 kHz is high enough to move switching noise well above
    // much of the audible range and is a common choice.
    driver.pwm_frequency = 30000;

    // Small dead zone to help avoid overlap between high-side and low-side
    // switching transitions.
    driver.dead_zone = 0.05f;

    // Supply voltage feeding the motor driver stage.
    driver.voltage_power_supply = VOLTAGE_SUPPLY_V;

    // Software voltage limit for the driver.
    driver.voltage_limit = DRIVER_VOLTAGE_LIMIT_V;

    Serial.print("Initializing motor driver... ");
    if (!driver.init())
    {
        Serial.println("FAILED");
        return false;
    }

    // Enable the driver after successful initialization.
    driver.enable();

    Serial.println("OK");
    return true;
}

// ========= MOTOR / FOC INITIALIZATION ==========
//
// This function links together the motor, driver, encoder,
// and current sensing objects, then configures the control mode
// and controller gains before starting FOC.
//
// High-level flow:
//
//   1. link hardware objects to motor
//   2. choose torque/current control mode
//   3. set safety/control limits
//   4. configure current-loop PID gains + LPFs
//   5. initialize motor object
//   6. initialize FOC
//
bool setupMotor()
{
    // Link the motor object to the already-created driver, encoder,
    // and current sensing blocks.
    motor.linkDriver(&driver);
    motor.linkSensor(&encoder);
    motor.linkCurrentSense(&current_sense);

    // We want torque control at the high level.
    //
    // Combined with foc_current below, this means our commands are
    // essentially current-based torque requests.
    motor.controller = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::foc_current;

    // Apply software safety/control limits.
    motor.voltage_limit = DRIVER_VOLTAGE_LIMIT_V;
    motor.current_limit = DRIVER_CURRENT_LIMIT_A;

    // q-axis current controller gains
    // The q-axis current loop is the main torque-producing loop.
    motor.PID_current_q.P = 10.0f;
    motor.PID_current_q.I = 150.0f;
    motor.PID_current_q.D = 0.0001f;
    motor.PID_current_q.limit = DRIVER_CURRENT_LIMIT_A;


    // d-axis current controller gains
    // The d-axis loop typically regulates the orthogonal current component.
    // In many cases we try to keep this component near zero.
    motor.PID_current_d.P = 10.0f;
    motor.PID_current_d.I = 150.0f;
    motor.PID_current_d.D = 0.0001f;
    motor.PID_current_d.limit = DRIVER_CURRENT_LIMIT_A;

    // Low-pass filtering of measured currents
    // These filters help smooth noisy current measurements before they
    // are used by the current controllers.
    motor.LPF_current_q.Tf = 0.002f;
    motor.LPF_current_d.Tf = 0.002f;

    Serial.print("Initializing motor... ");
    if (!motor.init())
    {
        Serial.println("FAILED");
        return false;
    }

    // Initialize FOC after the motor object and all linked hardware
    // have been configured.
    //
    // This step typically performs the control-side startup/alignment
    // required before normal closed-loop operation.
    Serial.print("Initializing FOC... ");
    if (!motor.initFOC())
    {
        Serial.println("FAILED");
        return false;
    }

    Serial.println("OK");
    return true;
}
}
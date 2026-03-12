#include <Arduino.h>
#include <SimpleFOC.h>
#include "app/Config.h"
#include "control/MotorSystem.h"
#include "drivers/ModifiedMagneticSensorMT6701SSI.h"
#include "drivers/SpiBus.h"

// Hardware instantiations
SpiBus spiBus(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI);
ModifiedMagneticSensorMT6701SSI encoder(PIN_ENC_CS);
InlineCurrentSense current_sense(SHUNT_RESISTOR, AMP_GAIN, PIN_I_A, PIN_I_B, PIN_I_C);
BLDCDriver6PWM driver(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL);
BLDCMotor motor(POLE_PAIRS);

static constexpr float MAX_CURRENT = 2.0f;

// Helpers
String readLineFromSerial()
{
    String input = "";
    while (true)
    {
        while (Serial.available() == 0)
        {
            delay(10);
        }
        char c = Serial.read();
        if (c == '\r')
            continue;
        if (c == '\n')
        {
            if (input.length() > 0)
                return input;
        }
        else
            input += c;
    }
}

void askUserForPIDGains(float *PID_Consts)
{
    Serial.println();
    Serial.println("Enter 6 PID values in one line:");
    Serial.println("Q_P Q_I Q_D D_P D_I D_D");
    Serial.println("Example: 5.0 200.0 0.0001 5.0 200.0 0.0001");
    Serial.println();

    String line = readLineFromSerial();
    line.trim();

    int parsed = sscanf(line.c_str(), "%f %f %f %f %f %f",
                        &PID_Consts[0], &PID_Consts[1], &PID_Consts[2],
                        &PID_Consts[3], &PID_Consts[4], &PID_Consts[5]);

    if (parsed != 6)
    {
        Serial.println("Invalid input. Using defaults.");
        PID_Consts[0] = 5.0f;
        PID_Consts[1] = 200.0f;
        PID_Consts[2] = 0.0001f;
        PID_Consts[3] = 5.0f;
        PID_Consts[4] = 200.0f;
        PID_Consts[5] = 0.0001f;
    }

    Serial.printf("Q-axis: P=%.4f, I=%.4f, D=%.4f\n", PID_Consts[0], PID_Consts[1], PID_Consts[2]);
    Serial.printf("D-axis: P=%.4f, I=%.4f, D=%.4f\n", PID_Consts[3], PID_Consts[4], PID_Consts[5]);
}

// ====================== DRIVER SETUP ====================
bool driverSetup()
{
    driver.pwm_frequency = 30e3;
    driver.dead_zone = 0.05f;
    driver.voltage_power_supply = VOLTAGE_SUPPLY;
    driver.voltage_limit = VOLTAGE_LIMIT;
    Serial.print("Initializing motor driver  ");
    if (!driver.init())
    {
        Serial.println("FAILED");
        return false;
    }
    driver.enable();
    Serial.println("SUCCESSFUL driver setup");
    return true;
}

// ======================== CURRENT SENSE SETUP =================
bool currentSenseSetup()
{
    current_sense.linkDriver(&driver);
    Serial.print("Initializing current sense  ");
    if (!current_sense.init())
    {
        Serial.println("FAILED");
        return false;
    }
    Serial.print("Aligning current sense w driver");
    if (!current_sense.driverAlign(VOLTAGE_LIMIT))
    {
        Serial.println("FAILED");
        return false;
    }
    Serial.println("SUCCESSFUL current sense setup");
    return true;
}


// ========================= MOTOR / FOC SETUP ================
bool motorSetup()
{
    // Connect motor object to the hardware pieces
    motor.linkDriver(&driver);
    motor.linkSensor(&encoder);
    motor.linkCurrentSense(&current_sense);

    // High-level motion mode: torque control. We set it to torque since our model computes desired torque
    motor.controller = MotionControlType::torque;

    // Low-level torque implementaiton: current controlled FOC (toruqe is propotional to q-axis current so we set this to foc_current)
    motor.torque_controller = TorqueControlType::foc_current;

    // Safety Limits
    motor.voltage_limit = VOLTAGE_LIMIT;
    motor.current_limit = MAX_CURRENT;

    // Current loop PID tuning
    // note that q-axis current -> torque producing current
    //           d-axis current -> usually kept near zero for normal FOC operations
    float PID_Constants[6];
    askUserForPIDGains(PID_Constants);

    // Q-Axis
    motor.PID_current_q.P = PID_Constants[0];
    motor.PID_current_q.I = PID_Constants[1];
    motor.PID_current_q.D = PID_Constants[2];
    motor.PID_current_q.limit = VOLTAGE_LIMIT;

    // D-Axis
    motor.PID_current_d.P = PID_Constants[3];
    motor.PID_current_d.I = PID_Constants[4];
    motor.PID_current_d.D = PID_Constants[5];
    motor.PID_current_d.limit = VOLTAGE_LIMIT;

    // Low-pass filters on measured currents (this helps reduce the noise in the current loop)
    motor.LPF_current_q.Tf = 0.05f; // change these as u wish
    motor.LPF_current_d.Tf = 0.05f;

    // initFOC performs electrical angle alignment and starts the
    // closed-loop FOC control structure.
    Serial.print("Initializing motor object  ");
    if (!motor.init())
    {
        Serial.println("FAILED");
        return false;
    }
    Serial.print("Initializing FOC  ");
    if (!motor.initFOC())
    {
        Serial.println("FAILED");
        return false;
    }
    Serial.println("SUCCESSFUL Motor & FOC initialization");
    return true;
}

// API Functions

bool initMotorSystem()
{
    spiBus.init();
    encoder.init(spiBus.bus());
    encoder.update();

    if (!driverSetup())
        return false;

    if (!currentSenseSetup())
        return false;

    if (!motorSetup())
        return false;

    return true;
}

void updateMotorControlStep()
{
    encoder.update();
    motor.loopFOC();
}

void applyMotorCurrent(float iqCmd)
{
    motor.move(iqCmd);
}

void stopMotor()
{
    motor.move(0.0f);
}

float getMotorAngleRad()
{
    return encoder.getAngle();
}

float getMotorAngleDegWrapped()
{
    return encoder.angleDegWrapped();
}

float getMotorVelocityRad()
{
    return encoder.getVelocity();
}

float getMeasuredIq()
{
    return motor.current.q;
}

PhaseCurrent_s getPhaseCurrents()
{
    return current_sense.getPhaseCurrents();
}

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
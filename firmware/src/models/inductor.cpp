#include <Arduino.h>
#include <SimpleFOC.h>
#include "app/Config.h"
#include "drivers/ModifiedMagneticSensorMT6701SSI.h"
#include "drivers/SpiBus.h"

// Inductor model

SpiBus spiBus(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI);
ModifiedMagneticSensorMT6701SSI encoder(PIN_ENC_CS);
InlineCurrentSense current_sense(SHUNT_RESISTOR, AMP_GAIN, PIN_I_A, PIN_I_B, PIN_I_C);
BLDCDriver6PWM driver(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL);
BLDCMotor motor(POLE_PAIRS);

// Parameters

// Virtual inductance / inertia gain. Bigger -> stronger resistance/feedback torque to acceleration.
static constexpr float VIRTUAL_INDUCTANCE = 0.020f;

// Motor torque constant Kt
static constexpr float TORQUE_CONST = 0.035f;

// Safety limits
static constexpr float MAX_TORQUE = 5.0f;
static constexpr float MAX_CURRENT = 2.0f;

// Acceleration estimation is noisy, so smooth it with a first-order filter.
static constexpr float ALPHA_FILTER_TF = 0.015f;

// Deadbands used to suppress encoder noise around zero motion.
static constexpr float OMEGA_DEADBAND = 0.15f;  // rad/s
static constexpr float ALPHA_DEADBAND = 8.0f;   // rad/s^2
static constexpr float IQ_DEADBAND = 0.03f;     // A

// How often to print debug info
static constexpr uint32_t PRINT_PERIOD_MS = 100;

static float prevOmega = 0.0f;
static float filteredAlpha = 0.0f;
static uint32_t lastLoopUs = 0;
static uint32_t lastPrint = 0;

// Helper Functions

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
        {
            continue;
        }
        if (c == '\n')
        {
            if (input.length() > 0)
            {
                return input;
            }
        }
        else
        {
            input += c;
        }
    }
}

void askUserForPIDGains(float *pidConstants)
{
    Serial.println();
    Serial.println("Enter 6 PID values in one line:");
    Serial.println("Q_P Q_I Q_D D_P D_I D_D");
    Serial.println("Example: 2.0 200.0 0.0 2.0 200.0 0.0");
    Serial.println();

    String line = readLineFromSerial();
    line.trim();

    int parsed = sscanf(line.c_str(), "%f %f %f %f %f %f",
                        &pidConstants[0], &pidConstants[1], &pidConstants[2],
                        &pidConstants[3], &pidConstants[4], &pidConstants[5]);

    if (parsed != 6)
    {
        Serial.println("Invalid input. Using default PID values.");
        pidConstants[0] = 2.0f;
        pidConstants[1] = 200.0f;
        pidConstants[2] = 0.0f;
        pidConstants[3] = 2.0f;
        pidConstants[4] = 200.0f;
        pidConstants[5] = 0.0f;
    }

    Serial.printf("Q-axis: P=%.4f, I=%.4f, D=%.4f\n",
                  pidConstants[0], pidConstants[1], pidConstants[2]);
    Serial.printf("D-axis: P=%.4f, I=%.4f, D=%.4f\n",
                  pidConstants[3], pidConstants[4], pidConstants[5]);
}

static float clampf(float val, float minVal, float maxVal)
{
    if (val < minVal)
    {
        return minVal;
    }
    if (val > maxVal)
    {
        return maxVal;
    }
    return val;
}

void printDebugData(float omega, float rawAlpha, float torqueCmd, float iqCmd)
{
    float angleDeg = encoder.angleDegWrapped();
    float angleRad = encoder.getAngle();
    PhaseCurrent_s currents = current_sense.getPhaseCurrents();

    Serial.printf(
        "Angle: %7.2f deg | AngleRad: %7.3f | Vel: %7.3f rad/s | "
        "AlphaRaw: %8.3f | AlphaFilt: %8.3f | "
        "TorqueCmd: %7.4f N*m | IqCmd: %6.3f A | "
        "Ia: %6.3f A | Ib: %6.3f A | Ic: %6.3f A\n",
        angleDeg,
        angleRad,
        omega,
        rawAlpha,
        filteredAlpha,
        torqueCmd,
        iqCmd,
        currents.a,
        currents.b,
        currents.c);
}

// DRIVER SETUP

bool driverSetup()
{
    driver.pwm_frequency = 30000;
    driver.dead_zone = 0.05f;
    driver.voltage_power_supply = VOLTAGE_SUPPLY;
    driver.voltage_limit = VOLTAGE_LIMIT;

    Serial.print("Initializing motor driver  ");

    bool success = driver.init();
    if (!success)
    {
        Serial.println("FAILED initializing driver");
        return false;
    }

    driver.enable();

    Serial.println("SUCCESSFUL driver setup");
    return true;
}

// CURRENT SENSE SETUP

bool currentSenseSetup()
{
    current_sense.linkDriver(&driver);

    Serial.print("Initializing current sense  ");
    bool success = current_sense.init();
    if (!success)
    {
        Serial.println("Failed initializing current sense");
        return false;
    }

    Serial.print("Aligning current sense w driver");
    int alignOk = current_sense.driverAlign(VOLTAGE_LIMIT);
    if (!alignOk)
    {
        Serial.println("FAILED");
        return false;
    }

    Serial.println("SUCCESSFUL current sense setup");
    return true;
}

// motor setup

bool motorSetup()
{
    motor.linkDriver(&driver);
    motor.linkSensor(&encoder);
    motor.linkCurrentSense(&current_sense);

    motor.controller = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::foc_current;

    motor.voltage_limit = VOLTAGE_LIMIT;
    motor.current_limit = MAX_CURRENT;

    float pidConstants[6];
    askUserForPIDGains(pidConstants);

    motor.PID_current_q.P = pidConstants[0];
    motor.PID_current_q.I = pidConstants[1];
    motor.PID_current_q.D = pidConstants[2];
    motor.PID_current_q.limit = VOLTAGE_LIMIT;

    motor.PID_current_d.P = pidConstants[3];
    motor.PID_current_d.I = pidConstants[4];
    motor.PID_current_d.D = pidConstants[5];
    motor.PID_current_d.limit = VOLTAGE_LIMIT;

    motor.LPF_current_q.Tf = 0.002f;
    motor.LPF_current_d.Tf = 0.002f;

    Serial.print("Initializing motor object  ");
    bool ok = motor.init();
    if (!ok)
    {
        Serial.println("FAILED");
        return false;
    }

    Serial.print("Initializing FOC  ");
    ok = motor.initFOC();
    if (!ok)
    {
        Serial.println("FAILED");
        return false;
    }
    Serial.println("SUCCESSFUL Motor & FOC initialization");

    return true;
}

// setup

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println();
    Serial.println("==============================================");
    Serial.println("Haptic Knob - Inductor Mode");
    Serial.println("==============================================");

    Serial.println("Starting system setup");

    spiBus.init();
    encoder.init(spiBus.bus());
    encoder.update();

    prevOmega = encoder.getVelocity();
    filteredAlpha = 0.0f;
    lastLoopUs = micros();

    if (!driverSetup())
    {
        Serial.println("Driver setup failed. Halting.");
        while (true)
        {
        }
    }

    if (!currentSenseSetup())
    {
        Serial.println("Current sense setup failed. Halting.");
        while (true)
        {
        }
    }

    if (!motorSetup())
    {
        Serial.println("Motor setup failed. Halting.");
        while (true)
        {
        }
    }

    Serial.println("System ready.");
    Serial.println();
}

// MAIN LOOP

void loop()
{
    encoder.update();
    motor.loopFOC();

    uint32_t nowUs = micros();
    float dt = (nowUs - lastLoopUs) * 1e-6f;
    lastLoopUs = nowUs;

    dt = clampf(dt, 0.0001f, 0.01f);
    // getting alpha
    float omega = encoder.getVelocity();
    if (fabsf(omega) < OMEGA_DEADBAND)
    {
        omega = 0.0f;
    }

    float rawAlpha = (omega - prevOmega) / dt;
    prevOmega = omega;

    // first order filter
    float alphaFilterGain = dt / (ALPHA_FILTER_TF + dt);
    filteredAlpha += alphaFilterGain * (rawAlpha - filteredAlpha);
    if (fabsf(filteredAlpha) < ALPHA_DEADBAND)
    {
        filteredAlpha = 0.0f;
    }

    // torque formula
    float torqueCmd = -VIRTUAL_INDUCTANCE * filteredAlpha;
    torqueCmd = clampf(torqueCmd, -MAX_TORQUE, MAX_TORQUE);

    float iqCmd = torqueCmd / TORQUE_CONST;
    iqCmd = clampf(iqCmd, -MAX_CURRENT, MAX_CURRENT);
    if (fabsf(iqCmd) < IQ_DEADBAND)
    {
        iqCmd = 0.0f;
    }

    motor.move(iqCmd);

    uint32_t nowMs = millis();
    if (nowMs - lastPrint >= PRINT_PERIOD_MS)
    {
        lastPrint = nowMs;
        printDebugData(omega, rawAlpha, torqueCmd, iqCmd);
    }
}

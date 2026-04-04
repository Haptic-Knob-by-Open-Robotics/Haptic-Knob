#include <Arduino.h>
#include <SimpleFOC.h>
#include "app/Config.h"
#include "drivers/ModifiedMagneticSensorMT6701SSI.h"
#include "drivers/SpiBus.h"

SpiBus spiBus(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI);
ModifiedMagneticSensorMT6701SSI encoder(PIN_ENC_CSN);
InlineCurrentSense current_sense(CURRENT_SENSE_SHUNT_RESISTOR_OHM, CURRENT_SENSE_AMP_GAIN, PIN_I_A, PIN_I_B, PIN_I_C);
BLDCDriver6PWM driver(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL);
BLDCMotor motor(POLE_PAIRS);

// Model Parameters
static constexpr float TORQUE_CONST = 0.035f; // N*m/A
static constexpr float MAX_TORQUE = 0.12f;
static constexpr float MAX_CURRENT = 2.0f;

static float K_virtual = 0.6f;  // stiffness
static float B_virtual = 0.03f; // damping

static float theta_origin = 0.0f; // unwrapped equilibrium position

// Filters
LowPassFilter velocityFilter(0.03f);

static constexpr uint32_t PRINT_PERIOD_MS = 20;
static uint32_t lastPrint = 0;

// Helper functions
static float clampf(float val, float minVal, float maxVal)
{
    if (val < minVal)
        return minVal;
    if (val > maxVal)
        return maxVal;
    return val;
}

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
        {
            input += c;
        }
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

// Serial Commands
void handleIncomingCommands()
{
    static String cmdBuf = "";

    while (Serial.available() > 0)
    {
        char c = (char)Serial.read();

        if (c == '\r')
            continue;

        if (c == '\n')
        {
            cmdBuf.trim();

            if (cmdBuf.startsWith("PID:"))
            {
                float qp, qi, qd, dp, di, dd;
                int parsed = sscanf(cmdBuf.c_str() + 4, "%f,%f,%f,%f,%f,%f",
                                    &qp, &qi, &qd, &dp, &di, &dd);
                if (parsed == 6)
                {
                    motor.PID_current_q.P = qp;
                    motor.PID_current_q.I = qi;
                    motor.PID_current_q.D = qd;

                    motor.PID_current_d.P = dp;
                    motor.PID_current_d.I = di;
                    motor.PID_current_d.D = dd;

                    Serial.printf("Q-axis: P=%.4f, I=%.4f, D=%.4f\n", qp, qi, qd);
                    Serial.printf("D-axis: P=%.4f, I=%.4f, D=%.4f\n", dp, di, dd);
                }
                else
                {
                    Serial.println(">> Bad PID command");
                }
            }
            else if (cmdBuf.startsWith("CAP:"))
            {
                float k, b;
                int parsed = sscanf(cmdBuf.c_str() + 4, "%f,%f", &k, &b);

                if (parsed == 2 && k > 0.0f && b >= 0.0f)
                {
                    K_virtual = k;
                    B_virtual = b;
                    Serial.printf(">> Capacitor updated: K=%.4f B=%.4f\n",
                                  K_virtual, B_virtual);
                }
                else
                {
                    Serial.println(">> Bad CAP command. Use CAP:K,B");
                }
            }
            else if (cmdBuf == "ZERO")
            {
                encoder.update();
                theta_origin = encoder.getAngle();
                Serial.printf(">> Origin reset to %.4f rad\n", theta_origin);
            }

            cmdBuf = "";
        }
        else
        {
            cmdBuf += c;
        }
    }
}

// Driver Setup
bool driverSetup()
{
    driver.pwm_frequency = 30e3;
    driver.dead_zone = 0.05f;
    driver.voltage_power_supply = VOLTAGE_SUPPLY_V;
    driver.voltage_limit = DRIVER_VOLTAGE_LIMIT_V;

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

// Current Sense Setup
bool currentSenseSetup()
{
    current_sense.linkDriver(&driver);

    Serial.print("Initializing current sense  ");
    if (!current_sense.init())
    {
        Serial.println("FAILED");
        return false;
    }

    Serial.print("Aligning current sense w driver  ");
    if (!current_sense.driverAlign(DRIVER_VOLTAGE_LIMIT_V))
    {
        Serial.println("FAILED");
        return false;
    }

    Serial.println("SUCCESSFUL current sense setup");
    return true;
}

// Motor & FOC Setup
bool motorSetup()
{
    motor.linkDriver(&driver);
    motor.linkSensor(&encoder);
    motor.linkCurrentSense(&current_sense);

    motor.controller = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::foc_current;

    motor.voltage_limit = DRIVER_VOLTAGE_LIMIT_V;
    motor.current_limit = MAX_CURRENT;

    float PID_Constants[6];
    askUserForPIDGains(PID_Constants);

    motor.PID_current_q.P = PID_Constants[0];
    motor.PID_current_q.I = PID_Constants[1];
    motor.PID_current_q.D = PID_Constants[2];
    motor.PID_current_q.limit = DRIVER_VOLTAGE_LIMIT_V;

    motor.PID_current_d.P = PID_Constants[3];
    motor.PID_current_d.I = PID_Constants[4];
    motor.PID_current_d.D = PID_Constants[5];
    motor.PID_current_d.limit = DRIVER_VOLTAGE_LIMIT_V;

    motor.LPF_current_q.Tf = 0.05f;
    motor.LPF_current_d.Tf = 0.05f;

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

// Setup
void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println();
    Serial.println("==============================================");
    Serial.println("Haptic Knob - Capacitor Mode");
    Serial.println("==============================================");

    spiBus.init();
    encoder.init(spiBus.bus());
    encoder.update();

    if (!driverSetup())
    {
        while (true)
        {
        }
    }

    if (!currentSenseSetup())
    {
        while (true)
        {
        }
    }

    if (!motorSetup())
    {
        while (true)
        {
        }
    }

    theta_origin = encoder.getAngle();

    Serial.println("System ready.");
    Serial.printf("Initial capacitor params: K=%.4f B=%.4f\n", K_virtual, B_virtual);
    Serial.printf("Origin angle: %.4f rad\n", theta_origin);
    Serial.println("Commands:");
    Serial.println("  CAP:K,B");
    Serial.println("  PID:QP,QI,QD,DP,DI,DD");
    Serial.println("  ZERO");
    Serial.println();
}

// Loop
void loop()
{
    handleIncomingCommands();

    encoder.update();
    motor.loopFOC();

    // read and filter knob motion
    float theta = encoder.getAngle(); // unwrapped angle
    float rawVel = encoder.getVelocity();
    float omega = velocityFilter(rawVel);

    if (fabsf(omega) < 0.15f)
        omega = 0.0f;

    // virtual capacitor model
    float displacement = theta - theta_origin;

    float torqueCmd = -K_virtual * displacement - B_virtual * omega;
    torqueCmd = clampf(torqueCmd, -MAX_TORQUE, MAX_TORQUE);

    float iqCmd = clampf(torqueCmd / TORQUE_CONST, -MAX_CURRENT, MAX_CURRENT);

    motor.move(iqCmd);

    // telemetry
    uint32_t nowMs = millis();
    if (nowMs - lastPrint >= PRINT_PERIOD_MS)
    {
        lastPrint = nowMs;

        PhaseCurrent_s currents = current_sense.getPhaseCurrents();

        Serial.printf(
            "AngleDeg:%7.2f | AngleRad:%7.3f | Origin:%7.3f | Disp:%7.3f | Vel:%7.3f | "
            "K:%6.3f | B:%6.3f | "
            "TorqueCmd:%7.4f | IqCmd:%6.3f | IqMeas:%6.3f | "
            "Ia:%6.3f | Ib:%6.3f | Ic:%6.3f\n",
            encoder.angleDegWrapped(),
            theta,
            theta_origin,
            displacement,
            omega,
            K_virtual, B_virtual,
            torqueCmd,
            iqCmd,
            motor.current.q,
            currents.a, currents.b, currents.c);
    }
}

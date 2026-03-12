#include <Arduino.h>
#include <SimpleFOC.h>
#include "app/Config.h"
#include "drivers/ModifiedMagneticSensorMT6701SSI.h"
#include "drivers/SpiBus.h"

// =================== HARDWARE OBJECTS =============
SpiBus spiBus(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI);
ModifiedMagneticSensorMT6701SSI encoder(PIN_ENC_CS);
InlineCurrentSense current_sense(SHUNT_RESISTOR, AMP_GAIN, PIN_I_A, PIN_I_B, PIN_I_C);
BLDCDriver6PWM driver(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL);
BLDCMotor motor(POLE_PAIRS);

// ================== CONTROL PARAMETERS ==============

// Base resistance value — do not change this at runtime, use GAIN_MULT instead
static constexpr float RESISTANCE_BASE = 1.0f;

// Gain multiplier applied on top of RESISTANCE_BASE.
// This can be updated live over serial with the command: G:<value>
// Effective resistance = RESISTANCE_BASE * gainMult
// e.g. G:5.0 -> resistance = 5.0, G:0.5 -> resistance = 0.5
static float gainMult = 1.0f;

// Motor torque constant Kt (from datasheet)
static constexpr float TORQUE_CONST = 0.035f;

// Safety limits
static constexpr float MAX_TORQUE = 5.0f;
static constexpr float MAX_CURRENT = 2.0f;

// Low-pass filter on velocity to remove encoder noise spikes
// Tf = time constant. Try 0.05 to start, increase if still spikey
LowPassFilter velocityFilter(0.05f);

// Telemetry rate — 20ms = 50Hz for smooth plotting
static constexpr uint32_t PRINT_PERIOD_MS = 20;
static uint32_t lastPrint = 0;

// ================ SERIAL COMMAND PROTOCOL ================
//
// The Python GUI can send two types of commands mid-run:
//
//   G:<value>\n
//       Sets the gain multiplier.
//       Example: "G:3.5\n"  →  gainMult = 3.5
//
//   PID:<QP>,<QI>,<QD>,<DP>,<DI>,<DD>\n
//       Updates all 6 PID constants live.
//       Example: "PID:2.0,200.0,0.0,2.0,200.0,0.0\n"
//
// Commands are parsed non-blocking in loop() from the serial input buffer.

// ================ HELPERS ============

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
    Serial.println("Example: 2.0 200.0 0.0 2.0 200.0 0.0");
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

static float clampf(float val, float min, float max)
{
    if (val < min)
        val = min;
    if (val > max)
        val = max;
    return val;
}

// ================== LIVE SERIAL COMMAND PARSER ==================
//
// Called every loop() iteration. Reads any available serial bytes into
// a static buffer. When a full line (terminated by \n) is received,
// it parses and applies the command immediately — no blocking.

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

            // ── G:<value>  →  update gain multiplier ──────────────────
            if (cmdBuf.startsWith("G:"))
            {
                float val = cmdBuf.substring(2).toFloat();
                if (val > 0.0f)
                {
                    gainMult = val;
                    Serial.printf(">> Gain updated: %.4f  (effective resistance: %.4f)\n",
                                  gainMult, RESISTANCE_BASE * gainMult);
                }
                else
                {
                    Serial.println(">> Bad G command — value must be > 0");
                }
            }

            // ── PID:<QP>,<QI>,<QD>,<DP>,<DI>,<DD>  →  update PID gains ──
            else if (cmdBuf.startsWith("PID:"))
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

                    // Echo back so the GUI can update its display
                    Serial.printf("Q-axis: P=%.4f, I=%.4f, D=%.4f\n", qp, qi, qd);
                    Serial.printf("D-axis: P=%.4f, I=%.4f, D=%.4f\n", dp, di, dd);
                }
                else
                {
                    Serial.println(">> Bad PID command — expected PID:QP,QI,QD,DP,DI,DD");
                }
            }

            cmdBuf = "";
        }
        else
        {
            cmdBuf += c;
        }
    }
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
    motor.linkDriver(&driver);
    motor.linkSensor(&encoder);
    motor.linkCurrentSense(&current_sense);
    motor.controller = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::foc_current;
    motor.voltage_limit = VOLTAGE_LIMIT;
    motor.current_limit = MAX_CURRENT;

    float PID_Constants[6];
    askUserForPIDGains(PID_Constants);

    motor.PID_current_q.P = PID_Constants[0];
    motor.PID_current_q.I = PID_Constants[1];
    motor.PID_current_q.D = PID_Constants[2];
    motor.PID_current_q.limit = VOLTAGE_LIMIT;

    motor.PID_current_d.P = PID_Constants[3];
    motor.PID_current_d.I = PID_Constants[4];
    motor.PID_current_d.D = PID_Constants[5];
    motor.PID_current_d.limit = VOLTAGE_LIMIT;

    motor.LPF_current_q.Tf = 0.002f;
    motor.LPF_current_d.Tf = 0.002f;

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

// ========================== SETUP =================
void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println();
    Serial.println("==============================================");
    Serial.println("Haptic Knob - Resistor Mode");
    Serial.println("==============================================");
    Serial.println("Starting system setup");

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

    Serial.println("System ready.");
    Serial.printf("Gain multiplier: %.4f  |  Effective resistance: %.4f\n",
                  gainMult, RESISTANCE_BASE * gainMult);
    Serial.println("Commands: G:<value>   PID:<QP>,<QI>,<QD>,<DP>,<DI>,<DD>");
    Serial.println();
}

// =================== MAIN LOOP ====================
void loop()
{
    // 1. Check for any incoming serial commands (non-blocking)
    handleIncomingCommands();

    // 2. Update encoder
    encoder.update();

    // 3. Run FOC inner loop
    motor.loopFOC();

    // 4. Compute resistive torque using effective resistance
    float omega = velocityFilter(encoder.getVelocity());
    float effectiveResistance = RESISTANCE_BASE * gainMult;
    float torqueCmd = clampf(-effectiveResistance * omega, -MAX_TORQUE, MAX_TORQUE);

    // 5. Convert to current command
    float iqCmd = clampf(torqueCmd / TORQUE_CONST, -MAX_CURRENT, MAX_CURRENT);

    // 6. Apply
    motor.move(iqCmd);

    // 7. Telemetry at 50Hz
    uint32_t now = millis();
    if (now - lastPrint >= PRINT_PERIOD_MS)
    {
        lastPrint = now;
        PhaseCurrent_s currents = current_sense.getPhaseCurrents();
        Serial.printf(
            "Angle: %7.2f deg | AngleRad: %7.3f | Vel: %7.3f rad/s | "
            "TorqueCmd: %7.4f N*m | IqCmd: %6.3f A | "
            "Ia: %6.3f A | Ib: %6.3f A | Ic: %6.3f A | IqMeas: %6.3f A | Gain: %.3f\n",
            encoder.angleDegWrapped(),
            encoder.getAngle(),
            omega,
            torqueCmd,
            iqCmd,
            currents.a, currents.b, currents.c,
            motor.current.q,
            gainMult);
    }
}
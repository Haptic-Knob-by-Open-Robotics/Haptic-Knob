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
SpiBus spiBus(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI);
ModifiedMagneticSensorMT6701SSI encoder(PIN_ENC_CS);
InlineCurrentSense current_sense(SHUNT_RESISTOR, AMP_GAIN, PIN_I_A, PIN_I_B, PIN_I_C);
BLDCDriver6PWM driver(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL);
BLDCMotor motor(POLE_PAIRS);

// ================== CONTROL PARAMETERS ==============

// Base resistance value
static constexpr float RESISTANCE_BASE = 1.0f;

// Effective resistance = RESISTANCE_BASE * gainMult
static float gainMult = 0.001f;

// Motor torque constant Kt (from datasheet)
static constexpr float TORQUE_CONST = 0.035f;

// Safety limits
// we need to prevent the system from sending too much torque/currnet or else andys wrist could potentially break
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

// ========================== SETUP =================

// setup() runs once at startup. This is where we bring the whole system online in the right order
// 1. Start serial output
// 2. Start SPI bus
// 3. Start encoder
// 4. Start motor driver
// 5. Start current sense
// 6. Start motor + FOC
void setup()
{
    Serial.begin(115200);

    // Small delay so serial monitor has time to connect
    delay(2000);

    Serial.println();
    Serial.println("==============================================");
    Serial.println("Haptic Knob - Resistor Mode");
    Serial.println("==============================================");
    Serial.println("Starting system setup");

    // Initialize SPI bus and encoder
    spiBus.init();
    encoder.init(spiBus.bus());
    encoder.update();

    // Initialize motor driver
    if (!driverSetup())
    {
        while (true)
        {
        }
    }

    // Initialize current-sense
    if (!currentSenseSetup())
    {
        while (true)
        {
        }
    }

    // Initialize motor + FOC control
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
// loop() runs over and over until we stop
// This is the real-time control loop of the haptic system

void loop()
{
    // Check for any incoming serial commands
    handleIncomingCommands();

    // Update encoder
    encoder.update();

    // Run low-level FOC control update
    // This is the inner electrical control loop. This part reads motor electrical state, uses current feedback
    // updates voltage / modulation commands, and eventually produces PWM duty cycles for the inverter
    motor.loopFOC();

    // Compute resistive torque using effective resistance
    // filter idle garbage values
    float rawVel = encoder.getVelocity();
    float filteredVel = velocityFilter(rawVel);

    float omega = 0;
    if (filteredVel > 0.25f || filteredVel < -0.25f)
        omega = filteredVel;
        
    float effectiveResistance = RESISTANCE_BASE * gainMult;
    float torqueCmd = clampf(-effectiveResistance * omega, -MAX_TORQUE, MAX_TORQUE);

    // Convert to current command
    float iqCmd = clampf(torqueCmd / TORQUE_CONST, -MAX_CURRENT, MAX_CURRENT);

    //  Send desired current to motor controller
    //  Note that Since we configured:
    //   motor.controller = torque
    //   motor.torque_controller = foc_current
    // SimpleFOC interprets this command as the torque-producing current target, and the current loop will try to make actual current match it.
    // so desired torque -> desired current -> current PID -> voltage command -> PWM -> motor phases -> actual torque feedback
    motor.move(iqCmd);

    // Telemetry
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
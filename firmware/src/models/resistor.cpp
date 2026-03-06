#include <Arduino.h>
#include <SimpleFOC.h>
#include "app/Config.h"
#include "drivers/ModifiedMagneticSensorMT6701SSI.h"
#include "drivers/SpiBus.h"

SpiBus spiBus(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI);
ModifiedMagneticSensorMT6701SSI encoder(PIN_ENC_CS);
InlineCurrentSense current_sense(SHUNT_RESISTOR, AMP_GAIN, PIN_I_A, PIN_I_B, PIN_I_C);
BLDCDriver6PWM driver(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL);
BLDCMotor motor(POLE_PAIRS);

void getData()
{
    float angleDeg = encoder.angleDegWrapped();
    float angleRad = encoder.getAngle();
    float velocity = encoder.getVelocity();
    PhaseCurrent_s currents = current_sense.getPhaseCurrents();

    Serial.printf("Angle: %7.2f°  Velocity: %6.2f rad/s  Ia: %7.3fA  Ib: %7.3fA  Ic: %7.3fA\n",
                  angleDeg, velocity, currents.a, currents.b, currents.c);
}

void setup()
{
    Serial.begin(115200);
    delay(3000);
    Serial.println("=== Haptic Knob -- Resistor Mode ===");

    // SPI Initialization
    spiBus.init();
    encoder.init(spiBus.bus());
    Serial.println("Encoder initialized!");

    // Driver Configuration
    driver.pwm_frequency = 30000;
    driver.dead_zone = 0.05;
}
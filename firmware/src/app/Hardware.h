#include <SimpleFOC.h>

/*
    Hardware.h

    This file declares the hardware-facing API for the haptic knob system.

    It owns the one-and-only access point to the real motor hardware stack:
      - SPI bus
      - magnetic encoder
      - current sense
      - BLDC driver
      - BLDC motor / SimpleFOC integration

    Other parts of the firmware should not directly instantiate or manage
    these hardware objects themselves. Instead, they should call the helper
    functions declared here.

    Responsibilities:
      - initialize the physical motor/sensor system
      - expose one control-step update function for the fast motor loop
      - expose helper functions to apply current commands
      - expose helper functions to read angle, velocity, and measured currents
*/

bool initHardware(); 


/*
    Called by the fast motor task.
    Expected to do things like:
    - encoder.update()
    - ADC update / current sampling if needed
    - motor.loopFOC()
*/
void updateHardwareControlStep();

/*
    Command application helpers.
    Use current mode for R/C/L if you keep foc_current.
    Use voltage mode for diode if that is your chosen implementation.
*/
void applyMotorCurrent(float iq_cmd);
void stopMotor();


/*
    Read back the latest motor/sensor state.
*/
float getMotorAngleRad();
float getMotorAngleDegWrapped();
float getMotorVelocityRad();
float getMeasuredIq();
PhaseCurrent_s getPhaseCurrents();

/*
    Runtime tuning helpers.
*/
void setCurrentPidGains(float qp, float qi, float qd,
                        float dp, float di, float dd);
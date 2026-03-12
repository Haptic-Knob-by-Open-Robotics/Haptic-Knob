#pragma once

#include <Arduino.h>
#include <SimpleFOC.h>
#include "drivers/ModifiedMagneticSensorMT6701SSI.h"
#include "drivers/SpiBus.h"

// Motor System API Functions

// Full hardware + FOC initialization
bool initMotorSystem();

// Run one update motor step
void updateMotorControlStep();

// Apply q-axis current command [A]
void applyMotorCurrent(float iqCmd);

// Safe stop
void stopMotor();

// Access State functions

float getMotorAngleRad();
float getMotorAngleDegWrapped();
float getMotorVelocityRad();
float getMeasuredIq();
PhaseCurrent_s getPhaseCurrents();

// Tuning functions

void setCurrentPidGains(float qp, float qi, float qd,
                        float dp, float di, float dd);

void askUserForPIDGains(float *PID_Consts);


extern SpiBus spiBus;
extern ModifiedMagneticSensorMT6701SSI encoder;
extern InlineCurrentSense current_sense;
extern BLDCDriver6PWM driver;
extern BLDCMotor motor;
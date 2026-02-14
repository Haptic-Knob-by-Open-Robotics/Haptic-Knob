#pragma once
#include <SimpleFOC.h>
#include "drivers/SsiEncoder.h"


/*
  MT6701Sensor (why this exists)

  SimpleFOC's FOC algorithm needs a rotor angle sensor. It only talks to sensors
  through the SimpleFOC `Sensor` interface (getAngle()/getVelocity()).

  Our MT6701 is read over SSI using our own driver class `SsiEncoder`.
  SimpleFOC doesn't know what `SsiEncoder` is, so this class is a small "adapter"
  that converts:

      SimpleFOC Sensor API  <->  our SsiEncoder API

  In practice, SimpleFOC calls getAngle() frequently inside motor.loopFOC().
  We respond by updating the SSI read and returning the angle in *radians*.
*/
class MT6701Sensor : public Sensor {
public: 

// Wrap an existing encoder driver 
explicit MT6701Sensor(SsiEncoder& encoder); 

// Called by SimpleFOC internally (expects radians)
float getAngle() override; 

// Still need to figure out what to do for this if we want to return derivative estimate or 0 and let SimpleFOC estimate
float getVelocity() override; 

  // Optional init hook (kept for Sensor interface completeness).
  void init() override; 

private: 
    SsiEncoder& encoder_; // our raw SSI encoder reader 

    //used only if we compute a simple velocity esitmate (derivate of angle) 
  float    prev_angle_rad_ = 0.0f;
  uint32_t prev_time_us_   = 0;
  float    velocity_rad_s_ = 0.0f;
};
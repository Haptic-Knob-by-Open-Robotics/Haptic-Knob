#include "drivers/MT6701Sensor.h"
#include <Arduino.h>

MT6701Sensor::MT6701Sensor(SsiEncoder* enc) : enc_(enc) {}

void MT6701Sensor::init() {
  // TODO: If encoder needs special init beyond enc_->init(), do it here.
  // For now, nothing.
}

float MT6701Sensor::getAngle() {
  // SimpleFOC expects radians.
  // Ensure update() is called here (or in ControlTask) so angle is fresh.
  enc_->update();
  return enc_->angleRadWrapped();
}

float MT6701Sensor::getVelocity() {
  // TODO:
  // Option A: return encoder-derived velocity (derivative)
  // Option B: return 0 and let SimpleFOC estimate
  //
  // Placeholder derivative:
  uint32_t now = micros();
  float a = getAngle();
  if (last_us_ != 0) {
    float dt = (now - last_us_) * 1e-6f;
    if (dt > 1e-6f) vel_ = (a - last_angle_) / dt;
  }
  last_us_ = now;
  last_angle_ = a;
  return vel_;
}

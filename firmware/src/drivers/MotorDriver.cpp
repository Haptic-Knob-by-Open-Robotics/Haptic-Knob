#include "MotorDriver.h"

MotorDriver::MotorDriver(int uh, int ul, int vh, int vl, int wh, int wl, int en)
: _driver(uh, ul, vh, vl, wh, wl, en) {}

bool MotorDriver::init(float v_supply, float v_limit){
    _driver.voltage_limit = v_limit;
    _driver.voltage_power_supply = v_supply;

    _ok = _driver.init();
    return _ok;
}

void MotorDriver::enable() {
  if (!_ok) return;
  _driver.enable();
}

void MotorDriver::disable() {
  if (!_ok) return;
  _driver.disable();
}

void MotorDriver::setPWM(float ua, float ub, float uc) {
  if (!_ok) return;
  _driver.setPwm(ua, ub, uc);
}

BLDCDriver6PWM& MotorDriver::raw() {
  return _driver;
}
#pragma once

#include <Arduino.h>
#include <SPI.h>
#include "MagneticSensorMT6701SSI.h"

class SsiEncoder {
public:
  SsiEncoder(int csPin, int clkPin, int misoPin);

  bool init();       // call once
  void update();     // call often

  float angleRad();
  float velocityRadS();
  float angleDegWrapped();
  uint16_t getRawAngle();  // Add this

private:
  int _cs, _clk, _miso;
  MagneticSensorMT6701SSI _sensor;
  bool _ok = false;
};
#pragma once

#include <Arduino.h>
#include <SPI.h>
// #include <SimpleFOC.h>
#include "drivers/newMagneticSensorMT6701SSI.h"

class SsiEncoder
{
public:
  SsiEncoder(int csPin, int clkPin, int misoPin);

  // call once in setup()
  bool init();

  // call repeatedly (ControlTask loop or loop())
  void update();

  // values from encoder
  float angleRad();
  float velocityRadS();

  // convenience (optional)
  float angleDegWrapped();

private:
  int _cs, _clk, _miso;

  MT6701SensorCustom _sensor;
  bool _ok = false;
};

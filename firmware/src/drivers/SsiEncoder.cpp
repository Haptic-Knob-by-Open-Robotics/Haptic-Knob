#include "SsiEncoder.h"
#include <math.h>

SsiEncoder::SsiEncoder(int csPin, int clkPin, int misoPin)
    : _cs(csPin), _clk(clkPin), _miso(misoPin), _sensor(csPin) {}

bool SsiEncoder::init()
{
  SPI.begin(_clk, _miso, -1, _cs); // for now its here, ideally it should be on SpiBus codes

  // initializing the encoder
  _sensor.init(&SPI);

  // checking if encoder is initialized
  _sensor.update();
  float a = _sensor.getAngle();
  _ok = !isnan(a);

  return _ok;
}

void SsiEncoder::update()
{
  if (!_ok)
    return;
  _sensor.update();
}

float SsiEncoder::angleRad()
{
  return _sensor.getAngle();
}

float SsiEncoder::velocityRadS()
{
  return _sensor.getVelocity();
}

float SsiEncoder::angleDegWrapped()
{
  float deg = _sensor.getAngle() * (180.0f / (float)M_PI);
  deg = fmodf(deg, 360.0f);
  if (deg < 0)
    deg += 360.0f;
  return deg;
}

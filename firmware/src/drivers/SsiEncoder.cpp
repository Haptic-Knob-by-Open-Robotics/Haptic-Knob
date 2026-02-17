#include "SsiEncoder.h"
#include <math.h>

SsiEncoder::SsiEncoder(int csPin, int clkPin, int misoPin)
: _cs(csPin), _clk(clkPin), _miso(misoPin), _sensor(csPin) {}

bool SsiEncoder::init() {
  
  // Initialize SPI
  SPI.begin(_clk, _miso, -1, _cs);
  
  _sensor.init(&SPI);

  // test read
  _sensor.update();
  float a = _sensor.getAngle();
  _ok = !isnan(a);

  return _ok;
}

void SsiEncoder::update() {
  _sensor.update();
}

float SsiEncoder::angleRad() {
  return _sensor.getAngle();
}

float SsiEncoder::velocityRadS() {
  return _sensor.getVelocity();
}

uint16_t SsiEncoder::getRawAngle() {
  // Get raw 14-bit value directly
  digitalWrite(_cs, LOW);
  delayMicroseconds(1);
  uint16_t value = SPI.transfer16(0x0000);
  digitalWrite(_cs, HIGH);
  
  return (value >> 1) & 0x3FFF;  // Shift and mask to get 14 bits
}

float SsiEncoder::angleDegWrapped() {
  // Use the formula from the image: θ = (Σ D<i>•2^i / 16384) • 360°
  uint16_t raw = getRawAngle();
  float deg = (raw / 16384.0f) * 360.0f;
  return deg;
}
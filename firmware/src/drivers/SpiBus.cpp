#include "SpiBus.h"

SpiBus ::SpiBus(int clk, int miso, int mosi)
    : _clk(clk), _miso(miso), _mosi(mosi) {}

void SpiBus::init()
{
    SPI.begin(_clk, _miso, _mosi, -1); // no CS here, each device owns its own CS
}
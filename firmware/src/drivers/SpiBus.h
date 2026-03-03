#pragma once
#include <SPI.h>

class SpiBus
{
public:
    SpiBus(int clk, int miso, int mosi = -1);
    void init();
    SPIClass *bus() { return &SPI; }

private:
    int _clk, _miso, _mosi;
};
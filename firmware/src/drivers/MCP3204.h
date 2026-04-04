#pragma once
#include <SPI.h>
#include <SimpleFOC.h> // for CurrentSense, PhaseCurrent_s

class MCP3204CurrentSense : public CurrentSense
{
public:
    // cs_pin: the chip-select GPIO for the MCP3204 (NCS)
    // vref: reference voltage (determines counts→volts conversion)
    // shunt_ohm, amp_gain: same as InlineCurrentSense constructor
    // ch_u, ch_v, ch_w: MCP3204 channel numbers for each phase
    MCP3204CurrentSense(SPIClass *spi, int csn_pin, float vref,
                        float shunt_ohm, float amp_gain,
                        int ch_u, int ch_v, int ch_w);

    int init() override;

    PhaseCurrent_s getPhaseCurrents() override;

private:
    uint16_t readChannel(int channel); // raw SPI read, returns 0-4095
    float getVoltage(uint16_t raw_ADC);
    void calibrateOffsets();

    SPIClass *_spi;
    int _csn_pin;
    float _vref;
    float _volts_to_amps;
    int _ch_u, _ch_v, _ch_w;
};

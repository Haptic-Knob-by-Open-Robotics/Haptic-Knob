#include "MCP3204.h"

/*
note that inlineCurrentSense from simplefoc library assumes that we are using
stm32 and wont recognize the external adc, hence we will be handling
that ourselves and just pass the currents to the foc library via
PhaseCurrent_s InlineCurrentSense (hence the include simplefoc.h above)

https://ww1.microchip.com/downloads/en/DeviceDoc/21298e.pdf
1. init spibus
2. this guy is CSN so pull cs to low to start reading/transmit data
3. MOSI send start bit 1, sgl/diff, channels
4. read MISO, 1st bit is null, the rest 11 bit is data, MSB.
5. convert bits into voltage,
*/

// MCP3204 constructor
//  - shunt_resistor  - shunt resistor value
//  - gain  - current-sense op-amp gain
//  - ch_u   - A phase adc pin
//  - phB   - B phase adc pin
//  - phC   - C phase adc pin
MCP3204CurrentSense::MCP3204CurrentSense(SPIClass *spi, int csn_pin, float vref,
                                         float shunt_ohm, float amp_gain,
                                         int ch_u, int ch_v, int ch_w)
{
    _spi = spi;
    _ch_u = ch_u;
    _ch_v = ch_v;
    _ch_w = ch_w;
    _csn_pin = csn_pin;
    _vref = vref;
    _volts_to_amps = 1.0f / shunt_ohm / amp_gain;

    gain_a = gain_b = gain_c = _volts_to_amps;

    // Tell the base class that there are no analog ADC pins — this sensor
    // reads currents over SPI (MCP3204 channels), not directly via MCU ADC pins.
    // Without this, driverAlign() sees garbage pin values and makes wrong
    // phase-swap decisions that corrupt the offset and gain arrays.
    pinA = _NC;
    pinB = _NC;
    pinC = _NC;
};

// Inline sensor init function
int MCP3204CurrentSense::init()
{
    pinMode(_csn_pin, OUTPUT);
    digitalWrite(_csn_pin, HIGH);

    // set the center pwm (0 voltage vector)
    if (driver_type == DriverType::BLDC)
        static_cast<BLDCDriver *>(driver)->setPwm(driver->voltage_limit / 2, driver->voltage_limit / 2, driver->voltage_limit / 2);

    // calibrate zero offsets
    calibrateOffsets();

    // set zero voltage to all phases
    if (driver_type == DriverType::BLDC)
        static_cast<BLDCDriver *>(driver)->setPwm(0, 0, 0);

    // set the initialized flag
    initialized = true;

    // return success
    return 1;
}

uint16_t MCP3204CurrentSense::readChannel(int channel)
{
    _spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(_csn_pin, LOW);
    // MCP3204 Din  : [start, SGL, D2, D1, D0, x x x x]
    // MCP3204 Dout : [-------- HI-Z --------, null , B11, B10, B9, ... , B0]

    // Byte 1: [x x x x x START SGL D2]
    //   0x06 = 0b00000110 — start bit at bit2, SGL at bit1
    //   D2 = channel >> 2, always 0 for MCP3204 (only 4 channels, max ch=3=0b011)
    _spi->transfer(0x06); // D2 always 0 (channels 0-3 never set bit2)

    // Byte 2: [D1 D0 x x x x x x]
    //   D1 at bit7, D0 at bit6
    uint8_t hi = _spi->transfer((channel & 0x03) << 6);

    // Byte 3: clock out 8 more bits to receive remaining data bits on MISO
    uint8_t lo = _spi->transfer(0x00);

    digitalWrite(_csn_pin, HIGH);
    _spi->endTransaction();

    return ((hi & 0x0F) << 8) | lo; // 12-bit result
}

float MCP3204CurrentSense::getVoltage(uint16_t raw_ADC)
{
    float voltage = raw_ADC * (_vref / 4096.0f);
    return voltage;
}

// Function finding zero offsets of the ADC
void MCP3204CurrentSense::calibrateOffsets()
{
    const int calibration_rounds = 1000;

    // find adc offset = zero current voltage
    offset_ia = 0;
    offset_ib = 0;
    offset_ic = 0;
    // read the adc voltage 1000 times ( arbitrary number )
    for (int i = 0; i < calibration_rounds; i++)
    {
        offset_ia += getVoltage(readChannel(_ch_u));
        offset_ib += getVoltage(readChannel(_ch_v));
        offset_ic += getVoltage(readChannel(_ch_w));
        _delay(1);
    }
    // calculate the mean offsets
    offset_ia = offset_ia / calibration_rounds;
    offset_ib = offset_ib / calibration_rounds;
    offset_ic = offset_ic / calibration_rounds;
}

// read all three phase currents (if possible 2 or 3)
PhaseCurrent_s MCP3204CurrentSense::getPhaseCurrents()
{
    PhaseCurrent_s current;
    current.a = (getVoltage(readChannel(_ch_u)) - offset_ia) * gain_a; // amps
    current.b = (getVoltage(readChannel(_ch_v)) - offset_ib) * gain_b; // amps
    current.c = (getVoltage(readChannel(_ch_w)) - offset_ic) * gain_c; // amps
    return current;
}

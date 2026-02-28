#ifndef __MAGNETIC_SENSOR_MT6701_SSI_H__
#define __MAGNETIC_SENSOR_MT6701_SSI_H__

#include "Arduino.h"
#include "SPI.h"
#include "common/base_classes/Sensor.h"

// MT6701 Magnetic Hall Encoder Datasheet: https://www.novosns.com/enfiles/MT6701_Rev.1.8.pdf

// MT6701 sends 14 bit of angle data. Therefore, the number of distinct angle per revolution is 2^14 = 16384,
// this is count per revolution, the resolution is 360deg/16384 = 0.0219 deg
#define MT6701_CPR 16384.0f

// According to the datasheet, the encoder sends the MSB first
#define MT6701_BITORDER MSBFIRST

// Bit shift, according to the datasheet, the first bit is the actual angle
#define MT6701_DATA_POS 0

// From the datasheet, the MT6701 data transfer starts when CSN is pulled to logic 'Low'. see page 25 of the datasheet.
// The MT6701 transfers data on the rising edge of CLK, and the data transfer finally stops when CSN is pulled to logic 'High'.
// Use SPI mode 1, capture on falling edge. First bit is not valid data, so have to read 25 bits to get a full SSI frame.
// SSI frame is 1 bit ignore, 14 bits angle, 4 bit status and 6 bit CRC.
// 1Mhz is the spi clock frequency
static SPISettings MT6701SSISettings(1000000, MT6701_BITORDER, SPI_MODE1); // @suppress("Invalid arguments")

class MT6701SensorCustom : public Sensor
{
public:
    MT6701SensorCustom(int nCS = -1, SPISettings settings = MT6701SSISettings);
    virtual ~MT6701SensorCustom();

    virtual void init(SPIClass *_spi = &SPI);

    float getSensorAngle() override; // angle in radians, return current value

protected:
    uint16_t readRawAngleSSI();

    SPISettings settings;
    SPIClass *spi;
    int nCS = -1;
};

#endif
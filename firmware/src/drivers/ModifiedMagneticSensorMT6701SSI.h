#ifndef __MODIFIED_MAGNETIC_SENSOR_MT6701_SSI_H__
#define __MODIFIED_MAGNETIC_SENSOR_MT6701_SSI_H__
/**
 * @file ModifiedMagneticSensorMT6701SSI.cpp
 *
 * Modified version of the MT6701 SSI magnetic sensor driver from SimpleFOCDrivers.
 * Original: https://github.com/simplefoc/Arduino-FOC-drivers
 *
 * Modifications by Andy Setiawan:
 *  - Corrected SPI read to use 3-byte (24-bit) transfer to capture the full
 *    MT6701 SSI frame: [14-bit angle][4-bit status][6-bit CRC]
 *  - Fixed bit extraction shift (>> 10) to correctly align the 14-bit angle
 *    from the 24-bit frame per the MT6701 datasheet (Rev 1.8, page 25)
 *  - Removed beginTransaction/endTransaction to maintain custom pin mapping
 *    on ESP32-S3 (SPI.begin() with custom pins is called externally)
 *  - SPI bus initialization delegated to caller (SsiEncoder or main)
 *
 * Reference: https://www.novosns.com/enfiles/MT6701_Rev.1.8.pdf
 *
 * Revision Date: March 1st 2026
 */

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
// 1Mhz is the spi clock frequency
static SPISettings MT6701SSISettings(1000000, MT6701_BITORDER, SPI_MODE1); // @suppress("Invalid arguments")

class ModifiedMagneticSensorMT6701SSI : public Sensor
{
public:
    ModifiedMagneticSensorMT6701SSI(int nCS = -1, SPISettings settings = MT6701SSISettings);
    virtual ~ModifiedMagneticSensorMT6701SSI();

    virtual void init(SPIClass *_spi = &SPI);

    float getSensorAngle() override; // angle in radians, return current value
    float angleDegWrapped();         // wrapped 0-360 deg

protected:
    uint16_t readRawAngleSSI();

    SPISettings settings;
    SPIClass *spi;
    int nCS = -1;
};

#endif
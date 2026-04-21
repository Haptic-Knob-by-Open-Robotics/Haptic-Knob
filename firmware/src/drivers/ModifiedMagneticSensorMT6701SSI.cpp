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

#include "ModifiedMagneticSensorMT6701SSI.h"
#include "common/foc_utils.h"
#include "common/time_utils.h"

ModifiedMagneticSensorMT6701SSI ::ModifiedMagneticSensorMT6701SSI(int nCS, SPISettings settings) : settings(settings), nCS(nCS)
{
}

ModifiedMagneticSensorMT6701SSI ::~ModifiedMagneticSensorMT6701SSI()
{
}

void ModifiedMagneticSensorMT6701SSI ::init(SPIClass *_spi)
{
    this->spi = _spi;
    if (nCS >= 0)
    {
        pinMode(nCS, OUTPUT);
        digitalWrite(nCS, HIGH);
    }

    // Apply SPI settings (mode, speed, bit order) once here.
    // beginTransaction() is intentionally NOT used in readRawAngleSSI()
    // because on ESP32-S3 it reassigns the SPI bus to default GPIO pins,
    // overriding the custom pins set by spiENC.begin().
    spi->setDataMode(settings._dataMode);
    spi->setFrequency(settings._clock);
    spi->setBitOrder(settings._bitOrder);

    this->Sensor::init();
}

uint16_t ModifiedMagneticSensorMT6701SSI ::readRawAngleSSI()
{

    // Begin SSI frame: sensor active LOW
    if (nCS >= 0)
        digitalWrite(nCS, LOW);

    // The MT6701 SSI frame is 25 bits:
    //   [1-bit invalid/marker][14-bit angle D13:D0][4-bit status][6-bit CRC]
    //
    // We read 3 bytes (24 bits). With SPI_MODE2 (CLK idles high, capture on
    // falling edge), the first bit clocked in is the invalid marker bit, so
    // the 14 angle bits land in positions [22:9] of the 24-bit frame.
    //
    // Each spi->transfer(0x00) clocks out 0x00 on MOSI to generate 8 clock
    // pulses and captures 8 bits from MISO.
    uint16_t b0 = spi->transfer(0x00); // bits [23:16]: invalid + D13:D7
    uint16_t b1 = spi->transfer(0x00); // bits [15:8]:  D6:D0 + MG3:MG1
    uint16_t b2 = spi->transfer(0x00); // bits [7:0]:   MG0 + CRC[5:0] (ignored)

    // End the SSI frame
    if (nCS >= 0)
        digitalWrite(nCS, HIGH);

    // Combine the three bytes into one 24-bit value:
    // frame = b0 b1 b2
    // b0 becomes frame[23:16]
    // b1 becomes frame[15:8]
    // b2 becomes frame[7:0]
    uint32_t frame = (((uint32_t)b0 << 16) | (uint32_t)b1 << 8 | (uint32_t)b2);

    // Angle bits D13:D0 sit at frame[22:9] (leading invalid bit at [23]).
    // Shift right by 9 and mask to 14 bits.
    uint16_t angle = (frame >> 9) & 0x3FFF;

    return angle;
};

// check 40us delay between each read?
float ModifiedMagneticSensorMT6701SSI ::getSensorAngle()
{
    float angle_data = readRawAngleSSI();
    angle_data = (angle_data / (float)MT6701_CPR) * _2PI;
    // return the shaft angle
    return angle_data;
}

float ModifiedMagneticSensorMT6701SSI ::angleDegWrapped()
{
    float deg = readRawAngleSSI();
    deg = (deg / (float)MT6701_CPR) * 360.0f;
    deg = fmodf(deg, 360.0f);
    if (deg < 0)
        deg += 360.0f;
    return deg;
}
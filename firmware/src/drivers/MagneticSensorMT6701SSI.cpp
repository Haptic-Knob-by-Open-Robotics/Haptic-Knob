#include "./MagneticSensorMT6701SSI.h"
#include "common/foc_utils.h"
#include "common/time_utils.h"

MagneticSensorMT6701SSI::MagneticSensorMT6701SSI(int nCS, SPISettings settings) : settings(settings), nCS(nCS) {

}

MagneticSensorMT6701SSI::~MagneticSensorMT6701SSI() {

}

void MagneticSensorMT6701SSI::init(SPIClass* _spi) {
    this->spi=_spi;
    if (nCS >= 0) {
        pinMode(nCS, OUTPUT);
        digitalWrite(nCS, HIGH);
    }

    this->Sensor::init();
}

// check 40us delay between each read?
float MagneticSensorMT6701SSI::getSensorAngle() {
    float angle_data = readRawAngleSSI();
    angle_data = ( angle_data / (float)MT6701_CPR ) * _2PI;
    // return the shaft angle
    return angle_data;
}

uint16_t MagneticSensorMT6701SSI::readRawAngleSSI() {

    // Begin SSI frame: sensor active LOW
    if (nCS >= 0) digitalWrite(nCS, LOW);

    // The MT6701 outputs a 24-bit frame:
    // [14-bit angle][4-bit status][6-bit CRC]  = 24 bits = 3 bytes.
    //
    // Each spi->transfer(0x00) does two things at once:
    //   1) Sends 0x00 on MOSI just to generate 8 clock pulses (dummy byte)
    //   2) Receives 8 bits from the sensor on MISO/DO (this is what we store)
    //
    // Because we are MSB-first, the first byte we receive contains the earliest
    // (most significant) bits of the frame.
    uint16_t b0 = spi->transfer(0x00); // receive first 8 angle bits D13:D6
    uint16_t b1 = spi->transfer(0x00); // receive the rest 6 angle bits D5:D0 + 2 bit magnetic field status MG3:MG2
    uint16_t b2 = spi->transfer(0x00); // receive the rest 2 magnetic field status MG1:MG0 and the rest crc code
    
    // End the SSI frame
    if (nCS >= 0) digitalWrite(nCS, HIGH);

    // Combine the three bytes into one 24-bit value:
    // frame = b0 b1 b2
    // b0 becomes frame[23:16]
    // b1 becomes frame[15:8]   
    // b2 becomes frame[7:0]
    uint32_t frame = ( ((uint32_t)b0 << 16) | (uint32_t)b1 << 8 | (uint32_t)b2 ); 

    // extract 14 bits angle data D13:D0
    uint16_t angle = (frame >> 10) & 0x3FFF;

    return angle;
};
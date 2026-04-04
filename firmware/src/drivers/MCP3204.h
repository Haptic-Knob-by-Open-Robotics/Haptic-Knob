#pragma once

#include <Arduino.h>
#include <SPI.h>

#include <cstdint>

class MCP3204
{
public:
    enum class SingleEndedChannel : uint8_t
    {
        CH0 = 0,
        CH1 = 1,
        CH2 = 2,
        CH3 = 3
    };

    enum class DifferentialChannel : uint8_t
    {
        CH0_MINUS_CH1 = 0,
        CH1_MINUS_CH0 = 1,
        CH2_MINUS_CH3 = 2,
        CH3_MINUS_CH2 = 3
    };

    static constexpr uint16_t MAX_CODE = 4095;
    static constexpr uint8_t CHANNEL_COUNT = 4;

    explicit MCP3204(
        int csPin = -1,
        SPISettings settings = SPISettings(1000000, MSBFIRST, SPI_MODE0));

    void init(SPIClass *spi = &SPI);
    bool isInitialized() const;

    uint16_t readSingleEnded(SingleEndedChannel channel) const;
    uint16_t readSingleEnded(uint8_t channel) const;

    uint16_t readDifferential(DifferentialChannel channel) const;
    uint16_t readDifferential(uint8_t channel) const;

    bool readSingleEnded(SingleEndedChannel channel, uint16_t &out) const;
    bool readSingleEnded(uint8_t channel, uint16_t &out) const;

    bool readDifferential(DifferentialChannel channel, uint16_t &out) const;
    bool readDifferential(uint8_t channel, uint16_t &out) const;

    float readSingleEndedVoltage(SingleEndedChannel channel, float vRef) const;
    float readSingleEndedVoltage(uint8_t channel, float vRef) const;

    float codeToVoltage(uint16_t code, float vRef) const;

private:
    bool isValidChannel(uint8_t channel) const;
    bool readRaw(uint8_t muxSelection, bool singleEnded, uint16_t &out) const;

    SPIClass *_spi;
    SPISettings _settings;
    int _csPin;
    bool _initialized;
};

#include "drivers/MCP3204.h"

#include <algorithm>

namespace
{
constexpr uint8_t MCP3204_SINGLE_ENDED_START = 0x06;
constexpr uint8_t MCP3204_DIFFERENTIAL_START = 0x04;
}

MCP3204::MCP3204(int csPin, SPISettings settings)
    : _spi(nullptr),
      _settings(settings),
      _csPin(csPin),
      _initialized(false)
{
}

void MCP3204::init(SPIClass *spi)
{
    _spi = spi;

    if (_csPin >= 0)
    {
        pinMode(_csPin, OUTPUT);
        digitalWrite(_csPin, HIGH);
    }

    _initialized = (_spi != nullptr) && (_csPin >= 0);
}

bool MCP3204::isInitialized() const
{
    return _initialized;
}

uint16_t MCP3204::readSingleEnded(SingleEndedChannel channel) const
{
    return readSingleEnded(static_cast<uint8_t>(channel));
}

uint16_t MCP3204::readSingleEnded(uint8_t channel) const
{
    uint16_t code = 0;
    readSingleEnded(channel, code);
    return code;
}

uint16_t MCP3204::readDifferential(DifferentialChannel channel) const
{
    return readDifferential(static_cast<uint8_t>(channel));
}

uint16_t MCP3204::readDifferential(uint8_t channel) const
{
    uint16_t code = 0;
    readDifferential(channel, code);
    return code;
}

bool MCP3204::readSingleEnded(SingleEndedChannel channel, uint16_t &out) const
{
    return readSingleEnded(static_cast<uint8_t>(channel), out);
}

bool MCP3204::readSingleEnded(uint8_t channel, uint16_t &out) const
{
    if (!isValidChannel(channel))
    {
        out = 0;
        return false;
    }

    return readRaw(channel, true, out);
}

bool MCP3204::readDifferential(DifferentialChannel channel, uint16_t &out) const
{
    return readDifferential(static_cast<uint8_t>(channel), out);
}

bool MCP3204::readDifferential(uint8_t channel, uint16_t &out) const
{
    if (!isValidChannel(channel))
    {
        out = 0;
        return false;
    }

    return readRaw(channel, false, out);
}

float MCP3204::readSingleEndedVoltage(SingleEndedChannel channel, float vRef) const
{
    return readSingleEndedVoltage(static_cast<uint8_t>(channel), vRef);
}

float MCP3204::readSingleEndedVoltage(uint8_t channel, float vRef) const
{
    uint16_t code = 0;
    if (!readSingleEnded(channel, code))
    {
        return 0.0f;
    }

    return codeToVoltage(code, vRef);
}

float MCP3204::codeToVoltage(uint16_t code, float vRef) const
{
    if (vRef <= 0.0f)
    {
        return 0.0f;
    }

    const uint16_t clampedCode = std::min<uint16_t>(code, MAX_CODE);
    return (static_cast<float>(clampedCode) * vRef) / static_cast<float>(MAX_CODE);
}

bool MCP3204::isValidChannel(uint8_t channel) const
{
    return channel < CHANNEL_COUNT;
}

bool MCP3204::readRaw(uint8_t muxSelection, bool singleEnded, uint16_t &out) const
{
    if (!_initialized || _spi == nullptr || !isValidChannel(muxSelection))
    {
        out = 0;
        return false;
    }

    const uint8_t commandHigh = singleEnded ? MCP3204_SINGLE_ENDED_START : MCP3204_DIFFERENTIAL_START;
    const uint8_t commandLow = static_cast<uint8_t>(muxSelection << 6);

    _spi->beginTransaction(_settings);
    digitalWrite(_csPin, LOW);

    _spi->transfer(commandHigh);
    const uint8_t upper = _spi->transfer(commandLow);
    const uint8_t lower = _spi->transfer(0x00);

    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();

    out = static_cast<uint16_t>(((upper & 0x0F) << 8) | lower);
    return true;
}

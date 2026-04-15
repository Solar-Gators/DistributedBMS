#pragma once

#include <array>
#include <cstdint>

namespace CanFrames {

constexpr uint16_t BMS_CAN_ID = 0x07;

enum MessageType : uint8_t {
    HIGH_TEMP = 0,
    VOLTAGE_EXTREMES = 1,
    AVERAGES = 2
};

struct Frame {
    std::array<uint8_t, 8> data{};
    uint8_t dlc = 8;
};

Frame encodeHighTemp(float highTemp, uint8_t highIndex);
Frame encodeVoltageExtremes(uint16_t highV, uint16_t lowV, uint8_t lowIdx, uint8_t highIdx);
Frame encodeAverages(float avgTemp, uint16_t avgVoltage, uint8_t numCells);

uint8_t getType(const uint8_t* data);
bool decodeHighTemp(const uint8_t* data, float& temp, uint8_t& idx);
bool decodeVoltageExtremes(const uint8_t* data, uint16_t& highV, uint16_t& lowV, uint8_t& lowIdx,
                            uint8_t& highIdx, uint8_t& faults);
bool decodeAverages(const uint8_t* data, float& avgTemp, uint16_t& avgVoltage, uint8_t& numCells);

}  // namespace CanFrames

/*
 * CanFrames.hpp
 *
 *  Created on: Oct 27, 2025
 *      Author: samrb
 */

#ifndef INC_CANFRAMES_HPP_
#define INC_CANFRAMES_HPP_

#pragma once
#include <cstdint>
#include <array>
#include "CanFrames.hpp"
#include "CanDriver.hpp"

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

// Encode
Frame encodeHighTemp(float highTemp, uint8_t highIndex);
Frame encodeVoltageExtremes(uint16_t highV, uint16_t lowV,
                            uint8_t lowIdx, uint8_t highIdx);
Frame encodeAverages(float avgTemp, uint16_t avgVoltage, uint8_t numCells);

// Decode
uint8_t getType(const uint8_t* data);
bool decodeHighTemp(const uint8_t* data, float& temp, uint8_t& idx);
bool decodeVoltageExtremes(const uint8_t* data, uint16_t& highV,
                           uint16_t& lowV, uint8_t& lowIdx, uint8_t& highIdx);
bool decodeAverages(const uint8_t* data, float& avgTemp,
                    uint16_t& avgVoltage, uint8_t& numCells);

} // namespace CanFrames




#endif /* INC_CANFRAMES_HPP_ */

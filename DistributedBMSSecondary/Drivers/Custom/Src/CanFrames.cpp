#include "CanFrames.hpp"
#include <cstring>

namespace CanFrames {

// Encode
Frame encodeHighTemp(float highTemp, uint8_t highIndex) {
    Frame f{};
    f.data[0] = HIGH_TEMP;
    std::memcpy(&f.data[1], &highTemp, 4);
    f.data[5] = highIndex;
    return f;
}

Frame encodeVoltageExtremes(uint16_t highV, uint16_t lowV,
                            uint8_t lowIdx, uint8_t highIdx) {
    Frame f{};
    f.data[0] = VOLTAGE_EXTREMES;
    f.data[1] = highV & 0xFF;
    f.data[2] = highV >> 8;
    f.data[3] = lowV & 0xFF;
    f.data[4] = lowV >> 8;
    f.data[5] = lowIdx;
    f.data[6] = highIdx;
    return f;
}

Frame encodeAverages(float avgTemp, uint16_t avgVoltage, uint8_t numCells) {
    Frame f{};
    f.data[0] = AVERAGES;
    std::memcpy(&f.data[1], &avgTemp, 4);
    f.data[5] = avgVoltage & 0xFF;
    f.data[6] = avgVoltage >> 8;
    f.data[7] = numCells;
    return f;
}

// Decode
uint8_t getType(const uint8_t* data) {
    return data[0];
}

bool decodeHighTemp(const uint8_t* data, float& temp, uint8_t& idx) {
    if (data[0] != HIGH_TEMP) return false;
    std::memcpy(&temp, &data[1], 4);
    idx = data[5];
    return true;
}

bool decodeVoltageExtremes(const uint8_t* data, uint16_t& highV,
                           uint16_t& lowV, uint8_t& lowIdx, uint8_t& highIdx) {
    if (data[0] != VOLTAGE_EXTREMES) return false;
    highV = (data[2] << 8) | data[1];
    lowV  = (data[4] << 8) | data[3];
    lowIdx = data[5];
    highIdx = data[6];
    return true;
}

bool decodeAverages(const uint8_t* data, float& avgTemp,
                    uint16_t& avgVoltage, uint8_t& numCells) {
    if (data[0] != AVERAGES) return false;
    std::memcpy(&avgTemp, &data[1], 4);
    avgVoltage = (data[6] << 8) | data[5];
    numCells = data[7];
    return true;
}

} // namespace CanFrames

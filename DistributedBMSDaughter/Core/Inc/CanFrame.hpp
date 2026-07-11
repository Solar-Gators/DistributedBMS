#pragma once

#include "BMS.hpp"
#include <array>
#include <cstdint>
#include <cstring>

namespace CanFrames {

struct Frame8 {
    std::array<uint8_t, 8> bytes{};
    uint8_t dlc = 8;
};

inline Frame8 make_high_temp(const BMS::Results& r) {
    Frame8 f{};
    f.bytes[0] = 0;
    std::memcpy(&f.bytes[1], &r.high_C, 4);
    f.bytes[5] = r.high_temp_idx;
    /* Filtered low temp (deci-°C); 0x7FFF = no valid low in range. */
    int16_t low_x10 = 0x7FFF;
    if (r.low_C >= 10.0f && r.low_C <= 60.0f) {
        low_x10 = static_cast<int16_t>(r.low_C * 10.0f);
    }
    f.bytes[6] = static_cast<uint8_t>(low_x10 & 0xFFu);
    f.bytes[7] = static_cast<uint8_t>((low_x10 >> 8) & 0xFFu);
    return f;
}

inline Frame8 make_voltage_extremes(const BMS::Results& r) {
    Frame8 f{};
    f.bytes[0] = 1;
    f.bytes[1] = r.high_cell_mV & 0xFF;
    f.bytes[2] = r.high_cell_mV >> 8;
    f.bytes[3] = r.low_cell_mV & 0xFF;
    f.bytes[4] = r.low_cell_mV >> 8;
    f.bytes[5] = r.low_cell_phys_idx;
    f.bytes[6] = r.high_cell_phys_idx;
    f.bytes[7] = r.faults;
    return f;
}

inline Frame8 make_average_stats(const BMS::Results& r) {
    Frame8 f{};
    f.bytes[0] = 2;
    std::memcpy(&f.bytes[1], &r.avg_C, 4);
    f.bytes[5] = r.avg_cell_mV & 0xFF;
    f.bytes[6] = r.avg_cell_mV >> 8;
    f.bytes[7] = r.num_cells;
    return f;
}

}  // namespace CanFrames

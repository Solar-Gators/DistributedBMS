// CanFrames.hpp

#include "BMS.hpp"
#include <array>
#include <cstdint>
#include <cstring>
#include <bit>

namespace CanFrames {

struct Frame8 {
    std::array<uint8_t,8> bytes{};
    uint8_t dlc = 8;
};

inline Frame8 make_high_temp(const BMS::Results& r) {
    Frame8 f{};
    f.bytes[0] = 0; // type
    std::memcpy(&f.bytes[1], &r.high_C, 4);  // pack float
    f.bytes[5] = r.high_temp_idx;
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

} // namespace CanFrames

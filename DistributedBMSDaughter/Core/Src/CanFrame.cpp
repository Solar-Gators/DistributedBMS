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

// ****************** START OF CODE BY JACK ******************
inline Frame8 make_voltage_messages(const BMS::Results&r)
{
	Frame8 f{}; // Create empty 8-byte CAN frame
	    f.bytes[0] = 3; // Tells that cell ID is 3 (Cell 0).
	    f.bytes[1] = r.cell_voltages_mV[0] & 0xFF; // Defines cell 0 voltage low byte.
	    f.bytes[2] = r.cell_voltages_mV[0] >> 8; // Defines cell 0 voltage high byte.
	    std::memcpy(&f.bytes[3], &r.ntc_C[0], 4); // Defines Cell 0 temperature (float, bytes 3-6)
	    return f; // Returns the frame.
}

inline Frame8 make_voltage_message2 (const BMS::Results&r)
{
	Frame8 f{};
	    f.bytes[0] = 4;
	    f.bytes[1] = r.cell_voltages_mV[1] & 0xFF;
	    f.bytes[2] = r.cell_voltages_mV[1] >> 8;
	    std::memcpy(&f.bytes[3], &r.ntc_C[1], 4);
	    return f;
}
inline Frame8 make_voltage_message3 (const BMS::Results&r)
{
	Frame8 f{};
	    f.bytes[0] = 5;
	    f.bytes[1] = r.cell_voltages_mV[2] & 0xFF;
	    f.bytes[2] = r.cell_voltages_mV[2] >> 8;
	    std::memcpy(&f.bytes[3], &r.ntc_C[2], 4);
	    return f;
}
inline Frame8 make_voltage_message4 (const BMS::Results&r)
{
	Frame8 f{};
	    f.bytes[0] = 6;
	    f.bytes[1] = r.cell_voltages_mV[3] & 0xFF;
	    f.bytes[2] = r.cell_voltages_mV[3] >> 8;
	    std::memcpy(&f.bytes[3], &r.ntc_C[3], 4);
	    return f;
}
inline Frame8 make_voltage_message5 (const BMS::Results&r)
{
	Frame8 f{};
	    f.bytes[0] = 7;
	    f.bytes[1] = r.cell_voltages_mV[4] & 0xFF;
	    f.bytes[2] = r.cell_voltages_mV[4] >> 8;
	    std::memcpy(&f.bytes[3], &r.ntc_C[4], 4);
	    return f;
}
inline Frame8 make_voltage_message6 (const BMS::Results&r)
{
	Frame8 f{};
	    f.bytes[0] = 8;
	    f.bytes[1] = r.cell_voltages_mV[5] & 0xFF;
	    f.bytes[2] = r.cell_voltages_mV[5] >> 8;
	    std::memcpy(&f.bytes[3], &r.ntc_C[5], 4);
	    return f;

}
//  ****************** END OF CODE BY JACK ******************


} // namespace CanFrames

/*
 * BmsFleet.hpp
 *
 *  Created on: Oct 27, 2025
 *      Author: samrb
 */

#ifndef INC_BMSFLEET_HPP_
#define INC_BMSFLEET_HPP_

#pragma once
#include <cstdint>
#include <array>
#include "CanFrames.hpp"
#include "CanBus.hpp"
#include "CanDriver.hpp"

namespace BmsFleetCfg {
    constexpr uint8_t  MAX_MODULES = 8;
    constexpr uint32_t STALE_MS    = 1500;
}

struct ModuleData {
    float high_C = -1000.f;
    uint8_t high_temp_idx = 0;
    uint16_t high_mV = 0, low_mV = 0;
    uint8_t low_idx = 0, high_idx = 0;
    float avg_C = 0.f;
    uint16_t avg_cell_mV = 0;
    uint8_t num_cells = 0;
    uint32_t last_ms = 0;
    bool got_type0 = false, got_type1 = false, got_type2 = false;

    void clear();
    bool online(uint32_t now_ms, uint32_t stale_ms = BmsFleetCfg::STALE_MS) const;
};

struct IdMapEntry {
    uint16_t can_id = 0;
    uint8_t index = 0;
    bool used = false;
};

class BmsFleet {
public:
    BmsFleet();

    bool register_node(uint16_t can_id, uint8_t idx);
    void handle(const CANDriver::CANFrame& msg, uint32_t now_ms);

    ModuleData& module(uint8_t idx);
    const ModuleData& module(uint8_t idx) const;

    int hottest_module(uint32_t now_ms, float* out_temp = nullptr) const;
    int lowest_cell_module(uint32_t now_ms, uint16_t* out_mV = nullptr) const;
    bool has_any_data(uint8_t idx) const;

private:
    std::array<ModuleData, BmsFleetCfg::MAX_MODULES> modules_;
    std::array<IdMapEntry, BmsFleetCfg::MAX_MODULES> idmap_;

    int find_module_index(uint16_t can_id) const;
};




#endif /* INC_BMSFLEET_HPP_ */

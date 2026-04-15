#pragma once

#include <array>
#include <cstdint>
#include "PrimaryV2Contract.hpp"

/** Matches legacy Primary MCU `FleetSummaryData` / `ModuleSummaryData` for BmsManager. */
namespace PrimaryBmsFleetCfg {
constexpr uint32_t STALE_MS = PrimaryV2Contract::DAUGHTER_STALE_TIMEOUT_MS;
constexpr uint8_t MAX_MODULES = 8;
}

struct FleetSummaryData {
    uint16_t total_voltage_mV = 0;
    uint16_t highest_cell_mV = 0;
    uint16_t lowest_cell_mV = 0;

    float highest_temp_C = -1000.0f;

    uint8_t highest_cell_idx = 0xFF;
    uint8_t lowest_cell_idx = 0xFF;
    uint8_t highest_temp_idx = 0xFF;

    uint32_t last_update_ms = 0;

    bool is_online(uint32_t now_ms, uint32_t stale_ms = PrimaryBmsFleetCfg::STALE_MS) const {
        return (now_ms - last_update_ms) <= stale_ms;
    }

    void clear() {
        total_voltage_mV = 0;
        highest_cell_mV = 0;
        lowest_cell_mV = 0;
        highest_temp_C = -1000.0f;
        highest_cell_idx = 0xFF;
        lowest_cell_idx = 0xFF;
        highest_temp_idx = 0xFF;
        last_update_ms = 0;
    }
};

struct ModuleSummaryData {
    bool valid = false;
    uint8_t module_idx = 0xFF;

    float high_temp_C = -1000.0f;
    uint8_t high_temp_cell = 0;

    uint16_t high_mV = 0;
    uint16_t low_mV = 0;

    uint8_t low_idx = 0;
    uint8_t high_idx = 0;

    float avg_temp_C = 0.0f;
    uint16_t avg_cell_mV = 0;

    uint8_t num_cells = 0;
    uint32_t age_ms = 0;

    uint32_t last_update_ms = 0;
};

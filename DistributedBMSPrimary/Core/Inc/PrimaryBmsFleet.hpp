/*
 * PrimaryBmsFleet.hpp
 *
 *  Created on: Nov 7, 2025
 *      Author: samrb
 *
 * Fleet data structure for Primary MCU.
 * Stores aggregated fleet summary data received from Secondary MCU via UART.
 */

#ifndef INC_PRIMARYBMSFLEET_HPP_
#define INC_PRIMARYBMSFLEET_HPP_

#pragma once

#include <cstdint>
#include <array>
#include "../../../DistributedBMSCommon/Inc/UartFleetTypes.hpp"

namespace PrimaryBmsFleetCfg {
    constexpr uint32_t STALE_MS   = 2000;  // Data stale after 2 seconds
    constexpr uint8_t  MAX_MODULES = 8;
}

/**
 * Fleet summary data received from Secondary MCU
 *
 * NOTE:
 * This struct represents the *decoded state*, not the UART wire format.
 * UART payloads are parsed explicitly and copied into this structure.
 */
struct FleetSummaryData {
    // Voltage summary
    uint16_t total_voltage_mV  = 0;
    uint16_t highest_cell_mV   = 0;
    uint16_t lowest_cell_mV    = 0;

    // Temperature summary
    float    highest_temp_C    = -1000.0f;

    // Cell indices
    uint8_t  highest_cell_idx  = 0xFF;
    uint8_t  lowest_cell_idx   = 0xFF;
    uint8_t  highest_temp_idx  = 0xFF;

    // Timestamps
    uint32_t last_update_ms = 0;

    /**
     * Check if data is fresh (not stale)
     */
    bool is_online(uint32_t now_ms,
                   uint32_t stale_ms = PrimaryBmsFleetCfg::STALE_MS) const
    {
        return (now_ms - last_update_ms) <= stale_ms;
    }

    /**
     * Clear/reset all data
     */
    void clear() {
        total_voltage_mV = 0;
        highest_cell_mV = 0;
        lowest_cell_mV = 0;
        highest_temp_C = -1000.0f;

        highest_cell_idx = 0xFF;
        lowest_cell_idx  = 0xFF;
        highest_temp_idx = 0xFF;

        last_update_ms = 0;
    }
};

/**
 * Per-module summary data
 */
struct ModuleSummaryData {
    bool     valid = false;

    uint8_t  module_idx = 0xFF;

    float    high_temp_C = -1000.0f;
    uint8_t  high_temp_cell = 0;

    uint16_t high_mV = 0;
    uint16_t low_mV  = 0;

    uint8_t  low_idx  = 0;
    uint8_t  high_idx = 0;

    float    avg_temp_C = 0.0f;
    uint16_t avg_cell_mV = 0;

    uint8_t  num_cells = 0;
    uint32_t age_ms = 0;

    uint32_t last_update_ms = 0;
};

/**
 * Heartbeat data from secondary MCU
 */
struct HeartbeatData {
    bool     valid = false;
    uint32_t counter = 0;
    uint32_t last_update_ms = 0;
};

/**
 * Primary BMS Fleet Manager
 *
 * Owns decoded UART state and performs validation, aging, and aggregation.
 */
class PrimaryBmsFleet {
public:
    PrimaryBmsFleet();

    // UART update handlers (payload-only, no framing)
    bool update_from_uart_payload(const uint8_t* payload,
                                  uint16_t len,
                                  uint32_t now_ms);

    bool update_module_summary(const uint8_t* payload,
                               uint16_t len,
                               uint32_t now_ms);

    bool update_heartbeat(const uint8_t* payload,
                           uint16_t len,
                           uint32_t now_ms);

    // Derived summary from modules
    bool update_summary_from_modules(uint32_t now_ms);

    // Accessors
    const FleetSummaryData& summary() const { return summary_; }
    FleetSummaryData& summary() { return summary_; }

    const ModuleSummaryData& module(uint8_t idx) const { return modules_[idx]; }
    bool module_valid(uint8_t idx) const { return modules_[idx].valid; }

    const HeartbeatData& heartbeat() const { return heartbeat_; }
    bool heartbeat_valid() const { return heartbeat_.valid; }

    bool has_data(uint32_t now_ms) const {
        return summary_.is_online(now_ms);
    }

private:
    FleetSummaryData summary_{};
    std::array<ModuleSummaryData, PrimaryBmsFleetCfg::MAX_MODULES> modules_{};
    HeartbeatData heartbeat_{};
};

#endif /* INC_PRIMARYBMSFLEET_HPP_ */

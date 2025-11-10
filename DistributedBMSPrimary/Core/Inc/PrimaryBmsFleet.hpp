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

namespace PrimaryBmsFleetCfg {
    constexpr uint32_t STALE_MS = 2000;  // Consider data stale after 2 seconds
}

/**
 * Fleet summary data received from Secondary MCU
 */
struct FleetSummaryData {
    // Hottest module information
    uint8_t  hottest_module_idx = 0xFF;  // 0xFF = none
    float    hottest_temp_C = -1000.0f;  // Temperature in °C
    
    // Lowest cell information
    uint8_t  lowest_cell_module_idx = 0xFF;  // Module with lowest cell
    uint16_t lowest_cell_mV = 0;  // Lowest cell voltage in mV
    
    // Fleet status
    uint8_t  num_online_modules = 0;  // Number of online modules
    
    // Timestamps
    uint32_t last_update_ms = 0;  // When this data was last updated
    uint32_t secondary_timestamp_ms = 0;  // Timestamp from secondary MCU
    
    /**
     * Check if data is fresh (not stale)
     */
    bool is_online(uint32_t now_ms, uint32_t stale_ms = PrimaryBmsFleetCfg::STALE_MS) const {
        return (now_ms - last_update_ms) <= stale_ms;
    }
    
    /**
     * Calculate latency (time between secondary MCU timestamp and our reception)
     */
    uint32_t latency_ms(uint32_t now_ms) const {
        if (secondary_timestamp_ms == 0) return 0xFFFFFFFF;
        if (now_ms < secondary_timestamp_ms) return 0;  // Clock skew
        return now_ms - secondary_timestamp_ms;
    }
    
    /**
     * Clear/reset all data
     */
    void clear() {
        hottest_module_idx = 0xFF;
        hottest_temp_C = -1000.0f;
        lowest_cell_module_idx = 0xFF;
        lowest_cell_mV = 0;
        num_online_modules = 0;
        last_update_ms = 0;
        secondary_timestamp_ms = 0;
    }
};

/**
 * Primary BMS Fleet Manager
 * Manages fleet summary data received from Secondary MCU
 */
class PrimaryBmsFleet {
public:
    PrimaryBmsFleet();
    
    /**
     * Update fleet summary from received UART payload
     * @param payload Pointer to FleetSummaryPayload data
     * @param len Length of payload
     * @param now_ms Current timestamp
     * @return true if successfully parsed and updated
     */
    bool update_from_uart_payload(const uint8_t* payload, uint16_t len, uint32_t now_ms);
    
    /**
     * Get current fleet summary data
     */
    const FleetSummaryData& summary() const { return summary_; }
    FleetSummaryData& summary() { return summary_; }
    
    /**
     * Check if we have valid data
     */
    bool has_data(uint32_t now_ms) const {
        return summary_.is_online(now_ms);
    }
    
private:
    FleetSummaryData summary_;
    
    /**
     * Convert temperature from °C*10 (int16_t) to float °C
     */
    static float from_cdeg10(int16_t c_x10) {
        return (float)c_x10 / 10.0f;
    }
};

#endif /* INC_PRIMARYBMSFLEET_HPP_ */


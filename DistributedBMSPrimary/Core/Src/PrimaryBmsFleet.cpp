/*
 * PrimaryBmsFleet.cpp
 *
 *  Created on: Nov 7, 2025
 *      Author: samrb
 */

#include "PrimaryBmsFleet.hpp"
#include <cstring>

// UART payload structure (must match UartFleetPack.hpp from Secondary MCU)
typedef struct __attribute__((packed)) {
    uint8_t  type;           // = 0x10 (UART_FLEET_SUMMARY)
    uint8_t  hottest_idx;    // 0..MAX_MODULES-1, 0xFF if none
    int16_t  hottest_c_x10;  // °C*10
    uint8_t  lowest_idx;     // module that has the lowest cell
    uint16_t lowest_mV;      // mV (cell)
    uint8_t  num_online;     // how many modules online
    uint32_t now_ms;         // sender time (for latency checks)
} FleetSummaryPayload;

PrimaryBmsFleet::PrimaryBmsFleet() {
    summary_.clear();
}

bool PrimaryBmsFleet::update_from_uart_payload(const uint8_t* payload, uint16_t len, uint32_t now_ms) {
    // Validate payload
    if (!payload || len < sizeof(FleetSummaryPayload)) {
        return false;
    }
    
    const FleetSummaryPayload* p = (const FleetSummaryPayload*)payload;
    
    // Validate message type
    if (p->type != 0x10) {  // UART_FLEET_SUMMARY
        return false;
    }
    
    // Update fleet summary data
    summary_.hottest_module_idx = p->hottest_idx;
    summary_.hottest_temp_C = from_cdeg10(p->hottest_c_x10);
    summary_.lowest_cell_module_idx = p->lowest_idx;
    summary_.lowest_cell_mV = p->lowest_mV;
    summary_.num_online_modules = p->num_online;
    summary_.secondary_timestamp_ms = p->now_ms;
    summary_.last_update_ms = now_ms;
    
    return true;
}


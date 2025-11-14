/*
 * PrimaryBmsFleet.cpp
 *
 *  Created on: Nov 7, 2025
 *      Author: samrb
 */

#include "PrimaryBmsFleet.hpp"
#include <cstring>



PrimaryBmsFleet::PrimaryBmsFleet() {
    summary_.clear();
}

bool PrimaryBmsFleet::update_from_uart_payload(const uint8_t* payload, uint16_t len, uint32_t now_ms) {
    if (!payload || len < sizeof(UartFleetSummaryPayload)) {
        return false;
    }

    UartFleetSummaryPayload p;
    memcpy(&p, payload, sizeof(p));

    if (p.type != UART_FLEET_SUMMARY) {
        return false;
    }

    summary_.hottest_module_idx     = p.hottest_idx;
    summary_.hottest_temp_C         = uart_from_cdeg10(p.hottest_c_x10);
    summary_.lowest_cell_module_idx = p.lowest_idx;
    summary_.lowest_cell_mV         = p.lowest_mV;
    summary_.num_online_modules     = p.num_online;
    summary_.secondary_timestamp_ms = p.now_ms;
    summary_.last_update_ms         = now_ms;

    return true;
}

bool PrimaryBmsFleet::update_module_summary(const uint8_t* payload, uint16_t len, uint32_t now_ms) {
    if (!payload || len < sizeof(UartModuleSummaryPayload)) {
        return false;
    }

    UartModuleSummaryPayload p;
    memcpy(&p, payload, sizeof(p));

    if (p.type != UART_MODULE_SUMMARY) {
        return false;
    }

    if (p.module_idx >= PrimaryBmsFleetCfg::MAX_MODULES) {
        return false;
    }

    ModuleSummaryData& M = modules_[p.module_idx];
    M.valid           = true;
    M.module_idx      = p.module_idx;
    M.high_temp_C     = uart_from_cdeg10(p.high_c_x10);
    M.high_temp_cell  = p.high_temp_cell;
    M.high_mV         = p.high_mV;
    M.low_mV          = p.low_mV;
    M.low_idx         = p.low_idx;
    M.high_idx        = p.high_idx;
    M.avg_temp_C      = uart_from_cdeg10(p.avg_c_x10);
    M.avg_cell_mV     = p.avg_cell_mV;
    M.num_cells       = p.num_cells;
    M.age_ms          = p.age_ms;
    M.last_update_ms  = now_ms;

    return true;
}

bool PrimaryBmsFleet::update_heartbeat(const uint8_t* payload, uint16_t len, uint32_t now_ms) {
    if (!payload || len < sizeof(UartHeartbeatPayload)) {
        return false;
    }

    UartHeartbeatPayload p;
    memcpy(&p, payload, sizeof(p));

    if (p.type != UART_HEARTBEAT) {
        return false;
    }

    heartbeat_.valid = true;
    heartbeat_.counter = ((uint32_t)p.counter_msb << 16) |
                         ((uint32_t)p.counter_mid << 8)  |
                         ((uint32_t)p.counter_lsb);
    heartbeat_.last_update_ms = now_ms;

    return true;
}

bool PrimaryBmsFleet::update_summary_from_modules(uint32_t now_ms) {
    float hottest_temp = -1000.0f;
    uint8_t hottest_idx = 0xFF;

    uint16_t lowest_mV = 0xFFFF;
    uint8_t lowest_idx = 0xFF;

    uint16_t highest_mV = 0;           // <-- NEW
    uint8_t highest_idx = 0xFF;        // <-- NEW

    uint8_t num_online = 0;

    for (uint8_t i = 0; i < PrimaryBmsFleetCfg::MAX_MODULES; i++) {
        const ModuleSummaryData& M = modules_[i];
        if (!M.valid) continue;

        num_online++;

        if (M.high_temp_C > hottest_temp) {
            hottest_temp = M.high_temp_C;
            hottest_idx = i;
        }

        if (M.low_mV < lowest_mV) {
            lowest_mV = M.low_mV;
            lowest_idx = i;
        }

        // NEW: track highest cell mV -------------------------
        if (M.high_mV > highest_mV) {   // <-- NEW
            highest_mV = M.high_mV;    // <-- NEW
            highest_idx = i;           // <-- NEW
        }
        // ----------------------------------------------------
    }

    if (num_online == 0)
        return false;

    summary_.hottest_module_idx       = hottest_idx;
    summary_.hottest_temp_C           = hottest_temp;

    summary_.lowest_cell_module_idx   = lowest_idx;
    summary_.lowest_cell_mV           = lowest_mV;

    summary_.highest_cell_module_idx  = highest_idx;   // <-- NEW
    summary_.highest_cell_mV          = highest_mV;    // <-- NEW

    summary_.num_online_modules       = num_online;
    summary_.secondary_timestamp_ms   = now_ms;
    summary_.last_update_ms           = now_ms;

    return true;
}



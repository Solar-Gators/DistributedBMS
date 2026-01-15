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

bool PrimaryBmsFleet::update_from_uart_payload(const uint8_t* payload,
                                               uint16_t len,
                                               uint32_t now_ms)
{
    constexpr uint16_t EXPECTED_LEN = 12;

    if (!payload || len != EXPECTED_LEN) {
        return false;
    }

    size_t offset = 0;

    // ---- Message type ----
    uint8_t type = payload[offset++];
    if (type != UART_FLEET_SUMMARY) {
        return false;
    }

    auto get_u16 = [&](uint16_t& v) {
        v  = payload[offset++];
        v |= static_cast<uint16_t>(payload[offset++]) << 8;
    };

    auto get_u8 = [&](uint8_t& v) {
        v = payload[offset++];
    };

    uint16_t totalVoltage;
    uint16_t highestVoltage;
    uint16_t lowestVoltage;
    uint16_t highestTemp;
    uint8_t  highVoltageID;
    uint8_t  lowVoltageID;
    uint8_t  highTempID;

    get_u16(totalVoltage);
    get_u16(highestVoltage);
    get_u16(lowestVoltage);
    get_u16(highestTemp);
    get_u8(highVoltageID);
    get_u8(lowVoltageID);
    get_u8(highTempID);

    // ---- Store decoded values ----
    summary_.total_voltage_mV   = totalVoltage;
    summary_.highest_cell_mV    = highestVoltage;
    summary_.lowest_cell_mV     = lowestVoltage;
    summary_.highest_temp_C     = highestTemp;

    summary_.highest_cell_idx   = highVoltageID;
    summary_.lowest_cell_idx    = lowVoltageID;
    summary_.highest_temp_idx   = highTempID;

    summary_.last_update_ms = now_ms;

    return true;
}


bool PrimaryBmsFleet::update_module_summary(const uint8_t* payload,
                                            uint16_t len,
                                            uint32_t now_ms)
{
    if (!payload || len < 1) {
        return false;
    }

    size_t offset = 0;

    // ---- Message type ----
    uint8_t type = payload[offset++];
    if (type != UART_MODULE_SUMMARY) {
        return false;
    }

    // ---- Minimum payload length check ----
    // type (1)
    // module_idx (1)
    // high_c_x10 (2)
    // high_temp_cell (1)
    // high_mV (2)
    // low_mV (2)
    // low_idx (1)
    // high_idx (1)
    // avg_c_x10 (2)
    // avg_cell_mV (2)
    // num_cells (1)
    // age_ms (4)
    constexpr uint16_t MIN_LEN = 22;

    if (len < MIN_LEN) {
        return false;
    }

    auto get_u8 = [&](uint8_t& v) {
        v = payload[offset++];
    };

    auto get_u16 = [&](uint16_t& v) {
        v = payload[offset++];
        v |= static_cast<uint16_t>(payload[offset++]) << 8;
    };

    auto get_u32 = [&](uint32_t& v) {
        v  = payload[offset++];
        v |= static_cast<uint32_t>(payload[offset++]) << 8;
        v |= static_cast<uint32_t>(payload[offset++]) << 16;
        v |= static_cast<uint32_t>(payload[offset++]) << 24;
    };

    uint8_t  module_idx;
    uint16_t high_c_x10;
    uint8_t  high_temp_cell;
    uint16_t high_mV;
    uint16_t low_mV;
    uint8_t  low_idx;
    uint8_t  high_idx;
    uint16_t avg_c_x10;
    uint16_t avg_cell_mV;
    uint8_t  num_cells;
    uint32_t age_ms;

    get_u8(module_idx);

    if (module_idx >= PrimaryBmsFleetCfg::MAX_MODULES) {
        return false;
    }

    get_u16(high_c_x10);
    get_u8(high_temp_cell);
    get_u16(high_mV);
    get_u16(low_mV);
    get_u8(low_idx);
    get_u8(high_idx);
    get_u16(avg_c_x10);
    get_u16(avg_cell_mV);
    get_u8(num_cells);
    get_u32(age_ms);

    // ---- Update module state ----
    ModuleSummaryData& M = modules_[module_idx];

    M.valid            = true;
    M.module_idx       = module_idx;
    M.high_temp_C      = uart_from_cdeg10(high_c_x10);
    M.high_temp_cell   = high_temp_cell;
    M.high_mV          = high_mV;
    M.low_mV           = low_mV;
    M.low_idx          = low_idx;
    M.high_idx         = high_idx;
    M.avg_temp_C       = uart_from_cdeg10(avg_c_x10);
    M.avg_cell_mV      = avg_cell_mV;
    M.num_cells        = num_cells;
    M.age_ms           = age_ms;
    M.last_update_ms   = now_ms;

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

    summary_.highest_temp_idx  = hottest_idx;
    summary_.highest_temp_C    = hottest_temp;

    summary_.lowest_cell_idx   = lowest_idx;
    summary_.lowest_cell_mV    = lowest_mV;

    summary_.highest_cell_idx  = highest_idx;   // <-- NEW
    summary_.highest_cell_mV   = highest_mV;    // <-- NEW


    summary_.last_update_ms    = now_ms;

    return true;
}



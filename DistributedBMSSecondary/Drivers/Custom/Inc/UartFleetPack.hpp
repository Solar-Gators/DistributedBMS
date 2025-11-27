/*
 * UartFleetPack.cpp
 *
 *  Created on: Nov 7, 2025
 *      Author: samrb
 */

#ifndef INC_UARTFLEETPACK_CPP_
#define INC_UARTFLEETPACK_CPP_


// UartFleetPack.hpp
#pragma once
#include <stdint.h>
#include <string.h>
#include "BmsFleet.hpp"
#include "UartFramer.hpp"
#include "../../../DistributedBMSCommon/Inc/UartFleetTypes.hpp"


// Reuse your existing encoder from earlier:
// size_t uart_encode_frame(const uint8_t* payload, uint16_t payload_len,
//                          uint8_t* out_buf, size_t out_max);

// -----------------------------
// Builders (encode into UART frame)
// -----------------------------

static inline size_t uart_make_fleet_summary(const BmsFleet& F,
                                             uint32_t now_ms,
                                             uint8_t* out_frame,
                                             size_t out_max)
{
    UartFleetSummaryPayload p;
    memset(&p, 0, sizeof(p));
    p.type = UART_FLEET_SUMMARY;

    // hottest module
    float hottest = -1000.0f;
    int  hidx = F.hottest_module(now_ms, &hottest);
    p.hottest_idx   = (hidx >= 0) ? (uint8_t)hidx : 0xFF;
    p.hottest_c_x10 = uart_to_cdeg10(hottest);

    // lowest cell module
    uint16_t low_mV = 0xFFFF;
    int lidx = F.lowest_cell_module(now_ms, &low_mV);
    p.lowest_idx = (lidx >= 0) ? (uint8_t)lidx : 0xFF;
    p.lowest_mV  = (lidx >= 0) ? low_mV : 0;

    // count online
    uint8_t online = 0;
    for (uint8_t i = 0; i < BmsFleetCfg::MAX_MODULES; ++i) {
        if (F.module(i).online(now_ms)) ++online;
    }
    p.num_online = online;
    p.now_ms     = now_ms;

    static_assert(sizeof(UartFleetSummaryPayload) == 12, "Unexpected fleet summary payload size");

    return uart_encode_frame((const uint8_t*)&p, (uint16_t)sizeof(p), out_frame, out_max);
}

static inline size_t uart_make_module_summary(const BmsFleet& F,
                                              uint8_t module_idx,
                                              uint32_t now_ms,
                                              uint8_t* out_frame,
                                              size_t out_max)
{
    if (module_idx >= BmsFleetCfg::MAX_MODULES) return 0;

    const ModuleData& M = F.module(module_idx);
    UartModuleSummaryPayload p;

    p.type           = UART_MODULE_SUMMARY;
    p.module_idx     = module_idx;
    p.high_c_x10     = uart_to_cdeg10(M.high_C);
    p.high_temp_cell = M.high_temp_idx;
    p.high_mV        = M.high_mV;
    p.low_mV         = M.low_mV;
    p.low_idx        = M.low_idx;
    p.high_idx       = M.high_idx;
    p.avg_c_x10      = uart_to_cdeg10(M.avg_C);
    p.avg_cell_mV    = M.avg_cell_mV;
    p.num_cells      = M.num_cells;

    uint32_t age = (now_ms >= M.last_ms) ? (now_ms - M.last_ms) : 0;
    if (age > 0xFFFFu) age = 0xFFFFu;
    p.age_ms = (uint16_t)age;

    static_assert(sizeof(UartModuleSummaryPayload) == 18, "Unexpected module summary payload size");

    return uart_encode_frame((const uint8_t*)&p, (uint16_t)sizeof(p), out_frame, out_max);
}

// Optional: a tiny heartbeat if you want a periodic link check (4 bytes payload)
static inline size_t uart_make_heartbeat(uint32_t counter,
                                         uint8_t* out_frame,
                                         size_t out_max)
{
    UartHeartbeatPayload payload;
    payload.type        = UART_HEARTBEAT;
    payload.counter_lsb = (uint8_t)(counter & 0xFF);
    payload.counter_mid = (uint8_t)((counter >> 8) & 0xFF);
    payload.counter_msb = (uint8_t)((counter >> 16) & 0xFF);

    static_assert(sizeof(UartHeartbeatPayload) == 4, "Unexpected heartbeat payload size");

    return uart_encode_frame((const uint8_t*)&payload, (uint16_t)sizeof(payload), out_frame, out_max);
}



#endif /* INC_UARTFLEETPACK_CPP_ */

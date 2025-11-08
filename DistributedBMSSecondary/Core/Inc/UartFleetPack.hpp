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


// Reuse your existing encoder from earlier:
// size_t uart_encode_frame(const uint8_t* payload, uint16_t payload_len,
//                          uint8_t* out_buf, size_t out_max);

enum : uint8_t {
    UART_FLEET_SUMMARY  = 0x10,
    UART_MODULE_SUMMARY = 0x11,
    UART_HEARTBEAT      = 0x12
};

// Ensure tight packing (no padding)
#if defined(_MSC_VER)
  #pragma pack(push,1)
  #define PACKED
#else
  #define PACKED __attribute__((packed))
#endif

// °C sent as (degC * 10) int16_t
static inline int16_t to_cdeg10(float c)
{
    // clamp to a sane range to avoid overflow
    if (c >  3276.7f) c =  3276.7f;
    if (c < -3276.8f) c = -3276.8f;
    int32_t v = (int32_t)(c * 10.0f + (c >= 0 ? 0.5f : -0.5f));
    if (v >  32767) v =  32767;
    if (v < -32768) v = -32768;
    return (int16_t)v;
}

// -----------------------------
// Payload layouts
// -----------------------------

// Fleet-wide quick summary (11 bytes total)
typedef struct PACKED {
    uint8_t  type;           // = UART_FLEET_SUMMARY
    uint8_t  hottest_idx;    // 0..MAX_MODULES-1, 0xFF if none
    int16_t  hottest_c_x10;  // °C*10
    uint8_t  lowest_idx;     // module that has the lowest cell
    uint16_t lowest_mV;      // mV (cell)
    uint8_t  num_online;     // how many modules online
    uint32_t now_ms;         // sender time (for latency checks)
} FleetSummaryPayload;

// Per-module snapshot (20 bytes total)
typedef struct PACKED {
    uint8_t  type;           // = UART_MODULE_SUMMARY
    uint8_t  module_idx;
    int16_t  high_c_x10;     // highest temperature in module
    uint8_t  high_temp_cell; // index of hottest cell
    uint16_t high_mV;
    uint16_t low_mV;
    uint8_t  low_idx;
    uint8_t  high_idx;
    int16_t  avg_c_x10;      // average temp
    uint16_t avg_cell_mV;    // average cell voltage
    uint8_t  num_cells;      // 3..5
    uint16_t age_ms;         // age of data (now_ms - last_ms), saturated
} ModuleSummaryPayload;

#if defined(_MSC_VER)
  #pragma pack(pop)
#endif
#undef PACKED

// -----------------------------
// Builders (encode into UART frame)
// -----------------------------

static inline size_t uart_make_fleet_summary(const BmsFleet& F,
                                             uint32_t now_ms,
                                             uint8_t* out_frame,
                                             size_t out_max)
{
    FleetSummaryPayload p;
    memset(&p, 0, sizeof(p));
    p.type = UART_FLEET_SUMMARY;

    // hottest module
    float hottest = -1000.0f;
    int  hidx = F.hottest_module(now_ms, &hottest);
    p.hottest_idx  = (hidx >= 0) ? (uint8_t)hidx : 0xFF;
    p.hottest_c_x10 = to_cdeg10(hottest);

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
    ModuleSummaryPayload p;

    p.type           = UART_MODULE_SUMMARY;
    p.module_idx     = module_idx;
    p.high_c_x10     = to_cdeg10(M.high_C);
    p.high_temp_cell = M.high_temp_idx;
    p.high_mV        = M.high_mV;
    p.low_mV         = M.low_mV;
    p.low_idx        = M.low_idx;
    p.high_idx       = M.high_idx;
    p.avg_c_x10      = to_cdeg10(M.avg_C);
    p.avg_cell_mV    = M.avg_cell_mV;
    p.num_cells      = M.num_cells;

    uint32_t age = (now_ms >= M.last_ms) ? (now_ms - M.last_ms) : 0;
    if (age > 0xFFFFu) age = 0xFFFFu;
    p.age_ms = (uint16_t)age;

    return uart_encode_frame((const uint8_t*)&p, (uint16_t)sizeof(p), out_frame, out_max);
}

// Optional: a tiny heartbeat if you want a periodic link check (4 bytes payload)
static inline size_t uart_make_heartbeat(uint32_t counter,
                                         uint8_t* out_frame,
                                         size_t out_max)
{
    uint8_t payload[1 + 3];
    payload[0] = UART_HEARTBEAT;
    payload[1] = (uint8_t)(counter & 0xFF);
    payload[2] = (uint8_t)((counter >> 8) & 0xFF);
    payload[3] = (uint8_t)((counter >> 16) & 0xFF);
    return uart_encode_frame(payload, (uint16_t)sizeof(payload), out_frame, out_max);
}



#endif /* INC_UARTFLEETPACK_CPP_ */

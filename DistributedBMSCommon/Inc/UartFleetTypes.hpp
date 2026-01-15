#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UART_FLEET_SUMMARY   ((uint8_t)0x11)
#define UART_MODULE_SUMMARY  ((uint8_t)0x10)
#define UART_HEARTBEAT       ((uint8_t)0x12)

#pragma pack(push, 1)

typedef struct {
    uint8_t  type;           // = UART_FLEET_SUMMARY
    uint8_t  hottest_idx;    // 0..MAX_MODULES-1, 0xFF if none
    int16_t  hottest_c_x10;  // °C*10
    uint8_t  lowest_idx;     // module that has the lowest cell
    uint16_t lowest_mV;      // mV (cell)
    uint8_t  num_online;     // how many modules online
    uint32_t now_ms;         // sender time (for latency checks)
} UartFleetSummaryPayload;

// Per-module snapshot (18 bytes total)
typedef struct {
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
} UartModuleSummaryPayload;

typedef struct {
    uint8_t type;            // = UART_HEARTBEAT
    uint8_t counter_lsb;
    uint8_t counter_mid;
    uint8_t counter_msb;
} UartHeartbeatPayload;

#pragma pack(pop)

static inline int16_t uart_to_cdeg10(float c)
{
    if (c >  3276.7f) c =  3276.7f;
    if (c < -3276.8f) c = -3276.8f;
    int32_t v = (int32_t)(c * 10.0f + (c >= 0 ? 0.5f : -0.5f));
    if (v >  32767) v =  32767;
    if (v < -32768) v = -32768;
    return (int16_t)v;
}

static inline float uart_from_cdeg10(int16_t c_x10)
{
    return (float)c_x10 / 10.0f;
}

#ifdef __cplusplus
}
#endif

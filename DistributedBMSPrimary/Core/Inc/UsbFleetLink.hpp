#pragma once

#include <cstdint>
#include <cstddef>
#include <cmath>   // for std::lroundf

#include "PrimaryBmsFleet.hpp"

// Pull in CDC_Transmit_FS from ST USB stack
extern "C" {
#include "usbd_cdc_if.h"
}

class UsbFleetLink {
public:
    static constexpr uint8_t SOF0 = 0xA5;
    static constexpr uint8_t SOF1 = 0x5A;

    enum PacketType : uint8_t {
        USB_PKT_FLEET_SUMMARY  = 0x01,
        USB_PKT_MODULE_SUMMARY = 0x02,
        USB_PKT_HEARTBEAT      = 0x03,
    };

    static constexpr std::size_t MAX_FRAME_SIZE = 64; // full-speed CDC EP size

    UsbFleetLink();

    // High-level API: feed in data from PrimaryBmsFleet structs
    bool sendFleetSummary(const FleetSummaryData& summary, uint32_t now_ms);
    bool sendModuleSummary(const ModuleSummaryData& module, uint32_t now_ms);
    bool sendHeartbeat(const HeartbeatData& hb, uint32_t now_ms);

private:
    uint8_t frame_buf_[MAX_FRAME_SIZE];

    bool sendFrame(PacketType type, const void* payload, uint16_t payload_len);
    static uint16_t crc16_ccitt(const uint8_t* data, uint16_t len);
};


// ---------------- On-the-wire payload structs ----------------

#pragma pack(push, 1)

// Fleet summary payload
struct UsbFleetSummaryPayload {
    uint32_t now_ms;                // primary tick at send time
    uint32_t secondary_timestamp_ms; // as reported by secondary (for latency)

    uint8_t  num_online_modules;

    uint8_t  hottest_module_idx;
    int16_t  hottest_c_x10;         // hottest_temp_C * 10

    uint8_t  lowest_cell_module_idx;
    uint16_t lowest_cell_mV;

    uint8_t  highest_cell_module_idx;
    uint16_t highest_cell_mV;
};

// Per-module summary payload
struct UsbModuleSummaryPayload {
    uint32_t now_ms;         // primary send time
    uint8_t  module_idx;

    int16_t  high_c_x10;     // high_temp_C * 10
    uint8_t  high_temp_cell;

    uint16_t high_mV;
    uint16_t low_mV;

    uint8_t  low_idx;
    uint8_t  high_idx;

    int16_t  avg_c_x10;      // avg_temp_C * 10
    uint16_t avg_cell_mV;

    uint8_t  num_cells;

    uint16_t age_ms;         // from ModuleSummaryData
};

// Heartbeat payload
struct UsbHeartbeatPayload {
    uint32_t counter;        // HeartbeatData.counter
    uint32_t now_ms;         // primary send time
};

#pragma pack(pop)

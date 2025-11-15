#include "UsbFleetLink.hpp"
#include <cstring> // memcpy

UsbFleetLink::UsbFleetLink() {
    // nothing to init yet
}

// ---------------- Public API ----------------

bool UsbFleetLink::sendFleetSummary(const FleetSummaryData& summary,
                                    uint32_t now_ms)
{
    UsbFleetSummaryPayload p{};
    p.now_ms                = now_ms;
    p.secondary_timestamp_ms = summary.secondary_timestamp_ms;

    p.num_online_modules    = summary.num_online_modules;

    p.hottest_module_idx    = summary.hottest_module_idx;
    p.hottest_c_x10         = static_cast<int16_t>(
        std::lroundf(summary.hottest_temp_C * 10.0f));

    p.lowest_cell_module_idx = summary.lowest_cell_module_idx;
    p.lowest_cell_mV         = summary.lowest_cell_mV;

    p.highest_cell_module_idx = summary.highest_cell_module_idx;
    p.highest_cell_mV         = summary.highest_cell_mV;

    return sendFrame(USB_PKT_FLEET_SUMMARY, &p,
                     static_cast<uint16_t>(sizeof(p)));
}

bool UsbFleetLink::sendModuleSummary(const ModuleSummaryData& M,
                                     uint32_t now_ms)
{
    if (!M.valid) {
        return false;  // nothing to send
    }

    UsbModuleSummaryPayload p{};
    p.now_ms       = now_ms;
    p.module_idx   = M.module_idx;

    p.high_c_x10    = static_cast<int16_t>(std::lroundf(M.high_temp_C * 10.0f));
    p.high_temp_cell = M.high_temp_cell;

    p.high_mV      = M.high_mV;
    p.low_mV       = M.low_mV;

    p.low_idx      = M.low_idx;
    p.high_idx     = M.high_idx;

    p.avg_c_x10    = static_cast<int16_t>(std::lroundf(M.avg_temp_C * 10.0f));
    p.avg_cell_mV  = M.avg_cell_mV;

    p.num_cells    = M.num_cells;
    p.age_ms       = M.age_ms;

    return sendFrame(USB_PKT_MODULE_SUMMARY, &p,
                     static_cast<uint16_t>(sizeof(p)));
}

bool UsbFleetLink::sendHeartbeat(const HeartbeatData& hb, uint32_t now_ms)
{
    if (!hb.valid) {
        // you can decide to send even if invalid; this is conservative
        return false;
    }

    UsbHeartbeatPayload p{};
    p.counter = hb.counter;
    p.now_ms  = now_ms;

    return sendFrame(USB_PKT_HEARTBEAT, &p,
                     static_cast<uint16_t>(sizeof(p)));
}

// ---------------- Frame builder ----------------

bool UsbFleetLink::sendFrame(PacketType type,
                             const void* payload,
                             uint16_t payload_len)
{
    constexpr uint16_t HEADER_LEN = 5; // SOF0, SOF1, TYPE, LEN_L, LEN_H
    constexpr uint16_t CRC_LEN    = 2; // CRC-16

    const uint16_t total_len = HEADER_LEN + payload_len + CRC_LEN;
    if (total_len > MAX_FRAME_SIZE) {
        return false;  // too big for our USB EP
    }

    uint8_t* p = frame_buf_;

    // Header
    *p++ = SOF0;
    *p++ = SOF1;
    *p++ = static_cast<uint8_t>(type);
    *p++ = static_cast<uint8_t>(payload_len & 0xFF);
    *p++ = static_cast<uint8_t>((payload_len >> 8) & 0xFF);

    // Payload
    if (payload_len > 0 && payload != nullptr) {
        std::memcpy(p, payload, payload_len);
        p += payload_len;
    }

    // CRC over header + payload (not including CRC bytes)
    const uint16_t crc = crc16_ccitt(frame_buf_, HEADER_LEN + payload_len);
    *p++ = static_cast<uint8_t>(crc & 0xFF);
    *p++ = static_cast<uint8_t>((crc >> 8) & 0xFF);

    const uint16_t frame_len =
        static_cast<uint16_t>(p - frame_buf_);

    // Send via USB CDC
    // NOTE: CDC_Transmit_FS may return USBD_BUSY; this example just fails fast.
    if (CDC_Transmit_FS(frame_buf_, frame_len) != USBD_OK) {
        return false;
    }

    return true;
}

// ---------------- CRC-16/CCITT-FALSE ----------------

// Polynomial 0x1021, init 0xFFFF, no reflect, no final xor
uint16_t UsbFleetLink::crc16_ccitt(const uint8_t* data, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    while (len--) {
        crc ^= static_cast<uint16_t>(*data++) << 8;
        for (uint8_t i = 0; i < 8; ++i) {
            if (crc & 0x8000) {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

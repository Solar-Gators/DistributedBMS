#include "BmsFleet.hpp"

#include "UartCrc16.hpp"

#include <algorithm>
#include <cmath>

#include "cmsis_os.h"

namespace {

bool isPlausibleCell_mV(uint16_t mv)
{
    return mv >= kFleetMinPlausibleCell_mV && mv <= kFleetMaxPlausibleCell_mV;
}

bool isPlausibleAvgVoltage(float v)
{
    return v >= static_cast<float>(kFleetMinPlausibleCell_mV) &&
           v <= static_cast<float>(kFleetMaxPlausibleCell_mV);
}

bool isPlausibleTemp_C(float c)
{
    return std::isfinite(c) && c >= kFleetMinPlausibleTemp_C && c <= kFleetMaxPlausibleTemp_C;
}

bool useFleetPlausibilityFilter()
{
    return kFilterImplausibleFleetReadings;
}

}  // namespace

void ModuleData::clear() {
    highTemp = -1000.0f;
    lowTemp = 1000.0f;
    avgTemp = 0.0f;
    highTempID = 0;

    highVoltage = 0;
    lowVoltage = 0;
    lowVoltageID = 0;
    highVoltageID = 0;
    avgVoltage = 0.0f;

    num_cells = 0;
    faults = 0;
    last_ms = 0;
    online = false;
}

BmsFleet::BmsFleet() {
    for (auto& m : modules_) {
        m.clear();
    }
    for (auto& e : idmap_) {
        e = {};
    }
}

void BmsFleet::online(uint32_t now_ms) {
    for (auto& m : modules_) {
        if (now_ms - m.last_ms > STALE_MS) {
            m.online = false;
        }
    }
}

bool BmsFleet::isOnline(uint8_t index) {
    return modules_[index].online;
}

bool BmsFleet::registerDaughter(uint16_t can_id, uint8_t index) {
    for (auto& e : idmap_) {
        if (!e.used) {
            e.used = true;
            e.can_id = can_id;
            e.index = index;
            return true;
        }
    }
    return false;
}

ModuleData& BmsFleet::module(uint8_t id) {
    return modules_[id];
}

int BmsFleet::daughterSlotForCanId_(const CanBus::Frame& msg) const {
    if (msg.extended) {
        return -1;
    }
    const uint16_t sid = static_cast<uint16_t>(msg.id & 0x7FFu);

    bool any_registered = false;
    for (const auto& e : idmap_) {
        if (!e.used) {
            continue;
        }
        any_registered = true;
        if (e.can_id == sid && e.index < MAX_MODULES) {
            return static_cast<int>(e.index);
        }
    }

    (void)any_registered;
    /* DeviceConfig: CAN_ID = 0x100 + DEVICE_NUMBER (slots 0..7). */
    if (sid >= 0x100u && sid < (0x100u + MAX_MODULES)) {
        return static_cast<int>(sid - 0x100u);
    }
    return -1;
}

int BmsFleet::slotForStdCanId(uint16_t can_id) const {
    CanBus::Frame msg{};
    msg.id = static_cast<uint32_t>(can_id) & 0x7FFu;
    msg.extended = false;
    return daughterSlotForCanId_(msg);
}

void BmsFleet::handleMessage(const CanBus::Frame& msg, uint32_t now_ms) {
    if (msg.rtr || msg.len == 0) {
        return;
    }

    const int slot = daughterSlotForCanId_(msg);
    if (slot < 0 || slot >= static_cast<int>(MAX_MODULES)) {
        return;
    }

    ModuleData& M = modules_[static_cast<size_t>(slot)];
    const uint8_t* data = msg.data.data();

    /* Any data frame on a known daughter ID counts as online (CAN heartbeat). */
    M.last_ms = now_ms;
    M.online = true;

    bool decoded = false;
    switch (CanFrames::getType(data)) {
    case CanFrames::AVERAGES: {
        float avg_temp = 0.0f;
        uint16_t avg_voltage = 0;
        uint8_t num_cells = 0;
        if (CanFrames::decodeAverages(data, avg_temp, avg_voltage, num_cells)) {
            M.avgTemp = avg_temp;
            M.avgVoltage = static_cast<float>(avg_voltage);
            M.num_cells = num_cells;
            decoded = true;
        }
        break;
    }
    case CanFrames::VOLTAGE_EXTREMES: {
        uint16_t high_voltage = 0;
        uint16_t low_voltage = 0;
        uint8_t high_index = 0;
        uint8_t low_index = 0;
        uint8_t faults = 0;
        if (CanFrames::decodeVoltageExtremes(data, high_voltage, low_voltage, low_index, high_index,
                                             faults)) {
            M.highVoltage = high_voltage;
            M.lowVoltage = low_voltage;
            M.lowVoltageID = low_index;
            M.highVoltageID = high_index;
            M.faults = faults;
            decoded = true;
        }
        break;
    }
    case CanFrames::HIGH_TEMP: {
        float high_temp = 0.0f;
        float low_temp = 1000.0f;
        uint8_t high_index = 0;
        if (CanFrames::decodeHighTemp(data, high_temp, high_index, low_temp)) {
            M.highTemp = high_temp;
            M.lowTemp = low_temp;
            M.highTempID = high_index;
            decoded = true;
        }
        break;
    }
    case 3:{
    	break;
    }
    default:
        break;
    }

    if (decoded) {
        ++decode_ok_count_;
    } else {
        ++decode_fail_count_;
    }
}

void BmsFleet::applySymmetricTempMirrors_() {
    if (!kMirrorDb4TempsFromDb1) {
        return;
    }

    const size_t src = static_cast<size_t>(kSymmetricTempSourceSlot);
    const size_t dst = static_cast<size_t>(kSymmetricTempDestSlot);
    if (src >= modules_.size() || dst >= modules_.size()) {
        return;
    }

    const ModuleData& from = modules_[src];
    ModuleData& to = modules_[dst];
    if (!from.online || !to.online) {
        return;
    }

    to.avgTemp = from.avgTemp;
    to.highTemp = from.highTemp;
    to.lowTemp = from.lowTemp;
    to.highTempID = from.highTempID;
}

void BmsFleet::processModules() {
    FleetData fleet{};
    fleet.totalVoltage = 0;
    fleet.highestVoltage = 0;
    fleet.lowestVoltage = 0xFFFF;
    fleet.highestTemp = -1000.0f;

    uint8_t cell_offset = 0;

    const uint32_t now_ms = osKernelGetTickCount();
    online(now_ms);
    applySymmetricTempMirrors_();

    for (size_t i = 0; i < modules_.size(); i++) {
        auto& m = modules_[i];

        if (!m.online) {
            cell_offset += m.num_cells;
            continue;
        }

        const bool filter = useFleetPlausibilityFilter();
        const bool avg_ok = !filter || isPlausibleAvgVoltage(m.avgVoltage);
        const bool high_ok = !filter || isPlausibleCell_mV(m.highVoltage);
        const bool low_ok = !filter || isPlausibleCell_mV(m.lowVoltage);
        const bool temp_ok = !filter || isPlausibleTemp_C(m.highTemp);

        if (avg_ok) {
            fleet.totalVoltage += static_cast<uint32_t>(m.avgVoltage * static_cast<float>(m.num_cells));
        }

        if (high_ok && m.highVoltage > fleet.highestVoltage) {
            fleet.highestVoltage = m.highVoltage;
            fleet.highVoltageID = static_cast<uint8_t>(cell_offset + m.highVoltageID);
        }

        if (low_ok && m.lowVoltage < fleet.lowestVoltage) {
            fleet.lowestVoltage = m.lowVoltage;
            fleet.lowVoltageID = static_cast<uint8_t>(cell_offset + m.lowVoltageID);
        }

        if (temp_ok && m.highTemp > fleet.highestTemp) {
            fleet.highestTemp = m.highTemp;
            fleet.highTempID = static_cast<uint8_t>(cell_offset + m.highTempID);
        }

        cell_offset += m.num_cells;
    }

    fleet_ = fleet;
    refreshSummaryCache(now_ms);
}

void BmsFleet::refreshSummaryCache(uint32_t now_ms) {
    bool any_online = false;
    for (const auto& m : modules_) {
        if (m.online) {
            any_online = true;
            break;
        }
    }

    if (!any_online) {
        summary_cache_.clear();
        for (auto& s : module_snapshots_) {
            s = ModuleSummaryData{};
        }
        return;
    }

    const FleetData& f = fleet_;
    summary_cache_.total_voltage_mV =
        static_cast<uint16_t>(std::min<uint32_t>(f.totalVoltage, 0xFFFFu));
    summary_cache_.highest_cell_mV = f.highestVoltage;
    summary_cache_.lowest_cell_mV =
        (f.lowestVoltage == 0xFFFFu) ? f.highestVoltage : f.lowestVoltage;
    summary_cache_.highest_temp_C = f.highestTemp;
    summary_cache_.highest_cell_idx = f.highVoltageID;
    summary_cache_.lowest_cell_idx = f.lowVoltageID;
    summary_cache_.highest_temp_idx = f.highTempID;
    /* Fleet freshness = latest daughter RX among modules still marked online (not aggregation time). */
    uint32_t newest_rx_ms = 0;
    for (const auto& m : modules_) {
        if (m.online && m.last_ms > newest_rx_ms) {
            newest_rx_ms = m.last_ms;
        }
    }
    summary_cache_.last_update_ms = newest_rx_ms;

    for (size_t i = 0; i < modules_.size(); ++i) {
        const auto& src = modules_[i];
        ModuleSummaryData& dst = module_snapshots_[i];
        dst.valid = src.online;
        dst.module_idx = static_cast<uint8_t>(i);
        dst.high_temp_C = src.highTemp;
        dst.low_temp_C = src.lowTemp;
        dst.high_temp_cell = src.highTempID;
        dst.high_mV = src.highVoltage;
        dst.low_mV = src.lowVoltage;
        dst.low_idx = src.lowVoltageID;
        dst.high_idx = src.highVoltageID;
        dst.avg_temp_C = src.avgTemp;
        dst.avg_cell_mV = static_cast<uint16_t>(std::max(0.0f, std::min(src.avgVoltage, 65535.0f)));
        dst.num_cells = src.num_cells;
        dst.age_ms = src.online ? (now_ms - src.last_ms) : 0;
        dst.last_update_ms = src.last_ms;
    }
}

size_t BmsFleet::packFleetData(uint8_t* out) const {
    size_t offset = 0;
    const FleetData& f = fleet_;

    out[offset++] = 0xA5;
    out[offset++] = 0x5A;

    const size_t length_offset = offset;
    out[offset++] = 0x00;
    out[offset++] = 0x00;

    const size_t payload_start = offset;

    out[offset++] = UART_FLEET_SUMMARY;

    auto put_u16 = [&](uint16_t v) {
        out[offset++] = static_cast<uint8_t>(v & 0xFF);
        out[offset++] = static_cast<uint8_t>((v >> 8) & 0xFF);
    };

    auto put_s16 = [&](int16_t v) {
        const uint16_t uv = static_cast<uint16_t>(v);
        out[offset++] = static_cast<uint8_t>(uv & 0xFF);
        out[offset++] = static_cast<uint8_t>((uv >> 8) & 0xFF);
    };

    auto put_u8 = [&](uint8_t v) { out[offset++] = v; };

    const int16_t highest_c_x10 =
        static_cast<int16_t>(std::round(static_cast<double>(f.highestTemp * 10.0f)));

    put_u16(static_cast<uint16_t>(f.totalVoltage));
    put_u16(f.highestVoltage);
    put_u16(f.lowestVoltage);
    put_s16(highest_c_x10);

    put_u8(f.highVoltageID);
    put_u8(f.lowVoltageID);
    put_u8(f.highTempID);

    const uint16_t payload_len = static_cast<uint16_t>(offset - payload_start);
    out[length_offset + 0] = static_cast<uint8_t>(payload_len & 0xFF);
    out[length_offset + 1] = static_cast<uint8_t>((payload_len >> 8) & 0xFF);

    const uint8_t* crc_data = &out[length_offset];
    const size_t crc_data_len = 2U + payload_len;
    const uint16_t crc = UartCrc16::calculate(crc_data, crc_data_len);

    out[offset++] = static_cast<uint8_t>(crc & 0xFF);
    out[offset++] = static_cast<uint8_t>((crc >> 8) & 0xFF);

    return offset;
}

size_t BmsFleet::packModuleData(uint8_t* out, uint8_t module_idx) const {
    if (module_idx >= modules_.size()) {
        return 0;
    }

    const ModuleData& m = modules_[module_idx];

    size_t offset = 0;

    out[offset++] = 0xA5;
    out[offset++] = 0x5A;

    const size_t length_offset = offset;
    out[offset++] = 0x00;
    out[offset++] = 0x00;

    const size_t payload_start = offset;

    out[offset++] = UART_MODULE_SUMMARY;
    out[offset++] = module_idx;

    auto put_u8 = [&](uint8_t v) { out[offset++] = v; };
    auto put_u16 = [&](uint16_t v) {
        out[offset++] = static_cast<uint8_t>(v & 0xFF);
        out[offset++] = static_cast<uint8_t>((v >> 8) & 0xFF);
    };
    auto put_u32 = [&](uint32_t v) {
        out[offset++] = static_cast<uint8_t>(v & 0xFF);
        out[offset++] = static_cast<uint8_t>((v >> 8) & 0xFF);
        out[offset++] = static_cast<uint8_t>((v >> 16) & 0xFF);
        out[offset++] = static_cast<uint8_t>((v >> 24) & 0xFF);
    };

    const int16_t high_c_x10 = static_cast<int16_t>(m.highTemp * 10.0f);
    const int16_t avg_c_x10 = static_cast<int16_t>(m.avgTemp * 10.0f);

    put_u16(static_cast<uint16_t>(high_c_x10));
    put_u8(m.highTempID);

    put_u16(m.highVoltage);
    put_u16(m.lowVoltage);
    put_u8(m.lowVoltageID);
    put_u8(m.highVoltageID);

    put_u16(static_cast<uint16_t>(avg_c_x10));
    put_u16(static_cast<uint16_t>(m.avgVoltage));

    put_u8(m.num_cells);

    const uint32_t age_ms = osKernelGetTickCount() - m.last_ms;
    put_u32(age_ms);

    const uint16_t payload_len = static_cast<uint16_t>(offset - payload_start);
    out[length_offset + 0] = static_cast<uint8_t>(payload_len & 0xFF);
    out[length_offset + 1] = static_cast<uint8_t>((payload_len >> 8) & 0xFF);

    const uint8_t* crc_data = &out[length_offset];
    const size_t crc_data_len = 2U + payload_len;
    const uint16_t crc = UartCrc16::calculate(crc_data, crc_data_len);

    out[offset++] = static_cast<uint8_t>(crc & 0xFF);
    out[offset++] = static_cast<uint8_t>((crc >> 8) & 0xFF);

    return offset;
}

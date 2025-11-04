#include "BmsFleet.hpp"
#include <cstring>

// -------------------------------------------------------------
// ModuleData helpers
// -------------------------------------------------------------
void ModuleData::clear() {
    high_C = -1000.f; high_temp_idx = 0;
    high_mV = low_mV = 0; low_idx = high_idx = 0;
    avg_C = 0.f; avg_cell_mV = 0; num_cells = 0;
    last_ms = 0; got_type0 = got_type1 = got_type2 = false;
}

bool ModuleData::online(uint32_t now_ms, uint32_t stale_ms) const {
    return (now_ms - last_ms) <= stale_ms;
}

// -------------------------------------------------------------
// BmsFleet implementation
// -------------------------------------------------------------
BmsFleet::BmsFleet() {
    for (auto& m : modules_) m.clear();
    for (auto& e : idmap_)   e = {};
}

bool BmsFleet::register_node(uint16_t can_id, uint8_t idx) {
    if (idx >= BmsFleetCfg::MAX_MODULES) return false;

    // Update existing entry
    for (auto& e : idmap_) {
        if (e.used && e.can_id == can_id) {
            e.index = idx;
            return true;
        }
    }

    // Add new
    for (auto& e : idmap_) {
        if (!e.used) {
            e.used = true;
            e.can_id = can_id;
            e.index = idx;
            return true;
        }
    }
    return false; // map full
}

int BmsFleet::find_module_index(uint16_t can_id) const {
    for (const auto& e : idmap_) {
        if (e.used && e.can_id == can_id) return e.index;
    }
    return -1;
}

void BmsFleet::handle(const CANDriver::CANFrame& msg, uint32_t now_ms) {
    int mi = find_module_index((uint16_t)msg.can_id);
    if (mi < 0) return; // not registered

    ModuleData& M = modules_[mi];
    const uint8_t* b = msg.data;
    const uint8_t  n = msg.len;

    if (n < 1) return;

    switch (CanFrames::getType(b)) {
    case CanFrames::HIGH_TEMP: {
        float t; uint8_t idx;
        if (CanFrames::decodeHighTemp(b, t, idx)) {
            M.high_C = t;
            M.high_temp_idx = idx;
            M.last_ms = now_ms;
            M.got_type0 = true;
        }
    } break;

    case CanFrames::VOLTAGE_EXTREMES: {
        uint16_t hv, lv; uint8_t li, hi;
        if (CanFrames::decodeVoltageExtremes(b, hv, lv, li, hi)) {
            M.high_mV = hv; M.low_mV = lv;
            M.low_idx = li; M.high_idx = hi;
            M.last_ms = now_ms;
            M.got_type1 = true;
        }
    } break;

    case CanFrames::AVERAGES: {
        float at; uint16_t av; uint8_t nc;
        if (CanFrames::decodeAverages(b, at, av, nc)) {
            M.avg_C = at; M.avg_cell_mV = av; M.num_cells = nc;
            M.last_ms = now_ms;
            M.got_type2 = true;
        }
    } break;

    default:
        break;
    }
}

ModuleData& BmsFleet::module(uint8_t idx) { return modules_[idx]; }
const ModuleData& BmsFleet::module(uint8_t idx) const { return modules_[idx]; }

int BmsFleet::hottest_module(uint32_t now_ms, float* out_temp) const {
    int best = -1; float maxT = -1000.f;
    for (uint8_t i = 0; i < BmsFleetCfg::MAX_MODULES; ++i) {
        const auto& M = modules_[i];
        if (!M.online(now_ms)) continue;
        if (M.high_C > maxT) { maxT = M.high_C; best = i; }
    }
    if (out_temp && best >= 0) *out_temp = maxT;
    return best;
}

int BmsFleet::lowest_cell_module(uint32_t now_ms, uint16_t* out_mV) const {
    int best = -1; uint16_t minV = 0xFFFF;
    for (uint8_t i = 0; i < BmsFleetCfg::MAX_MODULES; ++i) {
        const auto& M = modules_[i];
        if (!M.online(now_ms)) continue;
        if (M.low_mV < minV) { minV = M.low_mV; best = i; }
    }
    if (out_mV && best >= 0) *out_mV = minV;
    return best;
}

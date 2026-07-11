/*
 * BMS.cpp
 *
 *  Created on: Oct 27, 2025
 *      Author: samrb
 */
#include "BMS.hpp"
#include <algorithm>

namespace {

/** Plausible cell readings for avg / min / max (outliers excluded). */
constexpr uint16_t kMinPlausibleCell_mV = 2000u;
constexpr uint16_t kMaxPlausibleCell_mV = 5000u;

/** Plausible NTC readings for avg / min / max (outliers excluded). */
constexpr float kMinPlausibleTemp_C = -40.0f;
constexpr float kMaxPlausibleTemp_C = 85.0f;

bool isPlausibleCell_mV(uint16_t mv)
{
    return mv >= kMinPlausibleCell_mV && mv <= kMaxPlausibleCell_mV;
}

bool isPlausibleTemp_C(float c)
{
    return std::isfinite(c) && c >= kMinPlausibleTemp_C && c <= kMaxPlausibleTemp_C;
}

}  // namespace

BMS::BMS(uint8_t num_cells, ThermParams tp) : tp_(tp) {
    set_num_cells(num_cells);
    clear();
}

void BMS::set_num_cells(uint8_t n) {
    num_cells_ = (n < 3) ? 3 : (n > 6 ? 6 : n);

    if (num_cells_ == 3) {
        cell_phys_map_ = {0,1,4,255,255,255};
    } else if (num_cells_ == 4) {
        cell_phys_map_ = {0,1,2,4,255,255};
    } else if (num_cells_ == 5) {
        cell_phys_map_ = {0,1,2,3,4,255};
    } else { // 5
        cell_phys_map_ = {0,1,2,3,4,5};
    }

    for (uint8_t i = 0; i < 6; ++i) {
        // Temp sensors are conceptually independent of cell count.
        // Default: treat all 6 as enabled; compute_temps() will ignore invalid readings (NaN).
        temp_use_[i] = true;
    }
}

uint8_t BMS::num_cells() const { return num_cells_; }

void BMS::set_cell_mV(const std::array<uint16_t,6>& mV) {
    cell_mV_ = mV;
}

void BMS::set_ntc_volts(const std::array<float,6>& v) {
    ntc_V_ = v;
    have_ntc_volts_ = true;
}

void BMS::set_ntc_counts(const std::array<uint16_t,6>& counts) {
    ntc_counts_ = counts;
    have_ntc_counts_ = true;
}

void BMS::setFaults(uint8_t faults){
	faults_ = faults;
}

void BMS::update() {
    compute_temps();
    compute_cell_stats();
    res_.faults = faults_;
}

const BMS::Results& BMS::results() const { return res_; }

uint16_t BMS::average_cell_mV()  const { return res_.avg_cell_mV; }
uint16_t BMS::high_cell_mV()     const { return res_.high_cell_mV; }
uint16_t BMS::low_cell_mV()      const { return res_.low_cell_mV; }
uint8_t  BMS::high_cell_index()  const { return res_.high_cell_phys_idx; }
uint8_t  BMS::low_cell_index()   const { return res_.low_cell_phys_idx; }
float    BMS::average_temp_C()   const { return res_.avg_C; }
float    BMS::high_temp_C()      const { return res_.high_C; }
float    BMS::low_temp_C()       const { return res_.low_C; }
uint8_t  BMS::high_temp_index()  const { return res_.high_temp_idx; }
uint8_t  BMS::low_temp_index()   const { return res_.low_temp_idx; }

void BMS::clear() {
    res_ = Results{};
    res_.num_cells = num_cells_;
}

bool BMS::is_valid_slot(uint8_t idx) { return idx != 255; }

void BMS::compute_cell_stats() {
    uint32_t sum = 0;
    bool first = true;
    uint16_t vmin = 0;
    uint16_t vmax = 0;
    uint8_t imin = 0;
    uint8_t imax = 0;
    uint8_t used = 0;

    for (uint8_t k = 0; k < 6u; ++k) {
        const uint8_t phys = cell_phys_map_[k];
        if (!is_valid_slot(phys)) {
            continue;
        }

        const uint16_t v = cell_mV_[phys];
        if (!isPlausibleCell_mV(v)) {
            continue;
        }

        sum += v;
        ++used;

        if (first) {
            vmin = vmax = v;
            imin = imax = phys;
            first = false;
        } else {
            if (v < vmin) {
                vmin = v;
                imin = phys;
            }
            if (v > vmax) {
                vmax = v;
                imax = phys;
            }
        }
    }

    if (used == 0) {
        res_.avg_cell_mV = 0;
        res_.high_cell_mV = 0;
        res_.low_cell_mV = 0;
        res_.high_cell_phys_idx = 0;
        res_.low_cell_phys_idx = 0;
    } else {
        res_.avg_cell_mV = static_cast<uint16_t>(sum / used);
        res_.high_cell_mV = vmax;
        res_.low_cell_mV = vmin;
        res_.high_cell_phys_idx = imax;
        res_.low_cell_phys_idx = imin;
    }
}

void BMS::compute_temps() {
    if (!have_ntc_counts_) {
        res_.avg_C = 0.f;
        res_.high_C = -1000.f;
        res_.low_C = 1000.f;
        res_.high_temp_idx = 0;
        res_.low_temp_idx = 0;
        res_.ntc_C = {};
        return;
    }

    float sum = 0.f;
    uint8_t used = 0;
    res_.high_C = -1000.f;
    res_.low_C = 1000.f;
    res_.high_temp_idx = 0;
    res_.low_temp_idx = 0;

    for (uint8_t i = 0; i < 6; ++i) {
        res_.ntc_C[i] = ntc_to_C(ntc_counts_[i]);
    }

    for (uint8_t i = 0; i < 6; ++i) {
        if (!temp_use_[i]) {
            continue;
        }
        const float t = res_.ntc_C[i];
        if (!isPlausibleTemp_C(t)) {
            continue;
        }

        sum += t;
        ++used;
        if (t > res_.high_C) {
            res_.high_C = t;
            res_.high_temp_idx = i;
        }
        if (t < res_.low_C) {
            res_.low_C = t;
            res_.low_temp_idx = i;
        }
    }

    if (used == 0) {
        res_.avg_C = NAN;
        res_.high_C = -1000.f;
        res_.low_C = 1000.f;
    } else {
        res_.avg_C = sum / static_cast<float>(used);
    }
}

float BMS::ntc_to_C(uint16_t adc) const {
    const float adc_max = tp_.adc_fs;
    if (adc == 0u || adc >= static_cast<uint16_t>(adc_max)) {
        return NAN;
    }

    // Divider: NTC on top, rfix (10k) to GND — see build_guide.md
    const float r_bottom_ohm = tp_.rfix_k * 1000.0f;
    const float R_ntc =
        r_bottom_ohm * (adc_max / static_cast<float>(adc) - 1.0f);

    const float lnR = logf(R_ntc);
    const float invT =
        tp_.A + tp_.B * lnR + tp_.C * lnR * lnR * lnR;

    const float T_K = 1.0f / invT;
    return T_K - 273.15f;
}





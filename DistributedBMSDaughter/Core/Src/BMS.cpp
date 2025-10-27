/*
 * BMS.cpp
 *
 *  Created on: Oct 27, 2025
 *      Author: samrb
 */
#include "BMS.hpp"
#include <algorithm>

BMS::BMS(uint8_t num_cells, ThermParams tp) : tp_(tp) {
    set_num_cells(num_cells);
    clear();
}

void BMS::set_num_cells(uint8_t n) {
    num_cells_ = (n < 3) ? 3 : (n > 5 ? 5 : n);

    if (num_cells_ == 3) {
        cell_phys_map_ = {0,1,4,255,255};
    } else if (num_cells_ == 4) {
        cell_phys_map_ = {0,1,2,4,255};
    } else { // 5
        cell_phys_map_ = {0,1,2,3,4};
    }

    for (uint8_t i=0;i<5;++i)
        temp_use_[i] = (i < num_cells_);
}

uint8_t BMS::num_cells() const { return num_cells_; }

void BMS::set_cell_mV(const std::array<uint16_t,5>& mV) {
    cell_mV_ = mV;
}

void BMS::set_ntc_volts(const std::array<float,5>& v) {
    ntc_V_ = v;
    have_ntc_volts_ = true;
}

void BMS::set_ntc_counts(const std::array<uint16_t,5>& counts) {
    for (size_t i=0;i<5;++i)
        ntc_V_[i] = tp_.vref * (float(counts[i]) / tp_.adc_fs);
    have_ntc_volts_ = true;
}

void BMS::update() {
    compute_cell_stats();
    compute_temps();
}

const BMS::Results& BMS::results() const { return res_; }

uint16_t BMS::average_cell_mV()  const { return res_.avg_cell_mV; }
uint16_t BMS::high_cell_mV()     const { return res_.high_cell_mV; }
uint16_t BMS::low_cell_mV()      const { return res_.low_cell_mV; }
uint8_t  BMS::high_cell_index()  const { return res_.high_cell_phys_idx; }
uint8_t  BMS::low_cell_index()   const { return res_.low_cell_phys_idx; }
float    BMS::average_temp_C()   const { return res_.avg_C; }
float    BMS::high_temp_C()      const { return res_.high_C; }
uint8_t  BMS::high_temp_index()  const { return res_.high_temp_idx; }

void BMS::clear() {
    res_ = Results{};
    res_.num_cells = num_cells_;
}

bool BMS::is_valid_slot(uint8_t idx) { return idx != 255; }

void BMS::compute_cell_stats() {
    uint32_t sum = 0;
    bool first = true;
    uint16_t vmin = 0, vmax = 0;
    uint8_t imin = 0, imax = 0;
    uint8_t used = 0;

    for (uint8_t k=0; k<5; ++k) {
        uint8_t phys = cell_phys_map_[k];
        if (!is_valid_slot(phys)) continue;

        uint16_t v = cell_mV_[phys];
        sum += v;
        ++used;

        if (first) { vmin=vmax=v; imin=imax=phys; first=false; }
        else {
            if (v < vmin) { vmin = v; imin = phys; }
            if (v > vmax) { vmax = v; imax = phys; }
        }
    }

    if (used == 0) {
        res_.avg_cell_mV = res_.high_cell_mV = res_.low_cell_mV = 0;
        res_.high_cell_phys_idx = res_.low_cell_phys_idx = 0;
    } else {
        res_.avg_cell_mV = static_cast<uint16_t>(sum / used);
        res_.high_cell_mV = vmax;
        res_.low_cell_mV = vmin;
        res_.high_cell_phys_idx = imax;
        res_.low_cell_phys_idx = imin;
    }
}

void BMS::compute_temps() {
    if (!have_ntc_volts_) {
        res_.avg_C = 0.f;
        res_.high_C = -1000.f;
        res_.high_temp_idx = 0;
        res_.ntc_C = {};
        return;
    }

    float sum = 0.f;
    res_.high_C = -1000.f;
    res_.high_temp_idx = 0;

    for (uint8_t i=0;i<5;++i)
        res_.ntc_C[i] = ntc_to_C(ntc_V_[i]);

    for (uint8_t i=0;i<num_cells_; ++i) {
        float T = res_.ntc_C[i];
        sum += T;
        if (T > res_.high_C) {
            res_.high_C = T;
            res_.high_temp_idx = i;
        }
    }

    res_.avg_C = sum / float(num_cells_);
}

float BMS::ntc_to_C(float v_out) const {
    if (v_out < 0.001f) return 999.f;

    // R_therm (kΩ) = (33 / Vout) - 10, using your divider model
    float r_therm = (33.0f / v_out) - tp_.rfix_k;
    float lnR = std::log(r_therm);

    float invT = tp_.A + tp_.B*lnR + tp_.C*lnR*lnR*lnR;
    return 1.0f / invT - 273.15f;
}





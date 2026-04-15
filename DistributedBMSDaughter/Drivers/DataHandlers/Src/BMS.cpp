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

    // CODE BY JACK
    copy_voltages_to_results();
    // END OF CODE BY JACK

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
    if (!have_ntc_counts_) {
        res_.avg_C = 0.f;
        res_.high_C = -1000.f;
        res_.high_temp_idx = 0;
        res_.ntc_C = {};
        return;
    }

    float sum = 0.f;
    uint8_t used = 0;
    res_.high_C = -1000.f;
    res_.high_temp_idx = 0;

    for (uint8_t i = 0; i < 6; ++i) {
        res_.ntc_C[i] = ntc_to_C(ntc_counts_[i]);
    }

    for (uint8_t i = 0; i < 6; ++i) {
        if (!temp_use_[i]) continue;
        float T = res_.ntc_C[i];
        if (!std::isfinite(T)) continue;
        sum += T;
        ++used;
        if (T > res_.high_C) {
            res_.high_C = T;
            res_.high_temp_idx = i;
        }
    }

    res_.avg_C = (used > 0) ? (sum / float(used)) : NAN;
}

float BMS::ntc_to_C(uint16_t adc) const {
    // 12-bit ADC range: 0 to 4095
    if (adc == 0 || adc >= 4095) {
        return NAN;  // invalid / out of range
    }

    // Divider: NTC on top, 10k on bottom
    // R_ntc = R_bottom * (4095/adc - 1)
    constexpr float R_BOTTOM = 10000.0f;
    float R_ntc = R_BOTTOM * (4095.0f / static_cast<float>(adc) - 1.0f);

    // Steinhart-Hart coefficients
    constexpr float A = 0.0008929776265041391f;
    constexpr float B = 0.000250374123231943f;
    constexpr float C = 1.980949711932094e-07f;

    float lnR = logf(R_ntc);

    // 1/T = A + B*ln(R) + C*(ln(R))^3
    float invT = A + B * lnR + C * lnR * lnR * lnR;

    float T_K = 1.0f / invT;
    return T_K - 273.15f;
}

// CODE BY JACK
void BMS::copy_voltages_to_results()
{
    for (uint8_t i = 0; i < 6; ++i)
    {
        res_.cell_voltages_mV[i] = cell_mV_[i];
    }

}
// END OF CODE BY JACK



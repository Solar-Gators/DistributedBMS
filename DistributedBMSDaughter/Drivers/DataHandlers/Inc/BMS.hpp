/*
 * BMS.hpp
 *
 *  Created on: Oct 27, 2025
 *      Author: samrb
 */

#ifndef SRC_BMS_HPP_
#define SRC_BMS_HPP_


#pragma once
#include <array>
#include <cstdint>
#include <cmath>

class BMS {
public:
	struct ThermParams {
	    float A, B, C;
	    float rfix_k;
	    float vref;
	    float adc_fs;

	    ThermParams()
	        : A(0.002687481f),
	          B(0.0002829040f),
	          C(0.000001183565f),
	          rfix_k(10.0f),
	          vref(3.3f),
	          adc_fs(4096.0f) {}
	};


    struct Results {
        uint16_t avg_cell_mV   = 0;
        uint16_t high_cell_mV  = 0;
        uint16_t low_cell_mV   = 0;
        uint8_t  high_cell_phys_idx = 0;
        uint8_t  low_cell_phys_idx  = 0;

        std::array<float,5> ntc_C{};
        float avg_C = 0.0f;
        float high_C = -1000.0f;
        uint8_t high_temp_idx = 0;

        uint8_t num_cells = 0;
        uint8_t faults = 0;
    };

    explicit BMS(uint8_t num_cells = 4, ThermParams tp = ThermParams{});


    void set_num_cells(uint8_t n);
    uint8_t num_cells() const;

    void set_cell_mV(const std::array<uint16_t,5>& mV);
    void set_ntc_volts(const std::array<float,5>& v);
    void set_ntc_counts(const std::array<uint16_t,5>& counts);

    void setFaults(uint8_t faults);

    void update();
    const Results& results() const;

    uint16_t average_cell_mV()  const;
    uint16_t high_cell_mV()     const;
    uint16_t low_cell_mV()      const;
    uint8_t  high_cell_index()  const;
    uint8_t  low_cell_index()   const;
    float    average_temp_C()   const;
    float    high_temp_C()      const;
    uint8_t  high_temp_index()  const;

    void clear();

private:
    ThermParams tp_;
    uint8_t num_cells_ = 4;
    uint8_t faults_ = 0;
    std::array<uint8_t,5> cell_phys_map_{{0,1,2,4,255}};
    std::array<bool,5> temp_use_{{true,true,true,true,true}};
    std::array<uint16_t,5> cell_mV_{};
    std::array<float,5> ntc_V_{};
    bool have_ntc_volts_ = false;
    Results res_;

    static bool is_valid_slot(uint8_t idx);
    void compute_cell_stats();
    void compute_temps();
    float ntc_to_C(float v_out) const;

};



#endif /* SRC_BMS_HPP_ */

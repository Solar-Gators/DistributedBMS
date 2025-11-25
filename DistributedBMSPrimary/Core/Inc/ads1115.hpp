/*
 * ads1115.hpp
 *
 *  Created on: Nov 24, 2025
 *      Author: samrb
 */

#ifndef INC_ADS1115_HPP_
#define INC_ADS1115_HPP_

#pragma once

// Adjust include to your STM32 family
extern "C" {
#include "stm32l5xx_hal.h"
}

#include <cstdint>

class ADS1115 {
public:
    // 7-bit addresses, depending on how ADDR pin is wired
    enum class Addr7 : uint8_t {
        GND = 0x48,
        VDD = 0x49,
        SDA = 0x4A,
        SCL = 0x4B
    };

    // Programmable gain amplifier (PGA) full-scale ranges
    // These map to the PGA bits [11:9] in the config register.
    enum class Pga : uint8_t {
        FS_6_144V = 0, // ±6.144 V
        FS_4_096V = 1, // ±4.096 V
        FS_2_048V = 2, // ±2.048 V (default)
        FS_1_024V = 3, // ±1.024 V
        FS_0_512V = 4, // ±0.512 V
        FS_0_256V = 5  // ±0.256 V (also codes 6,7)
    };

    // Data rate (samples per second)
    enum class DataRate : uint8_t {
        SPS_8   = 0,
        SPS_16  = 1,
        SPS_32  = 2,
        SPS_64  = 3,
        SPS_128 = 4,  // default
        SPS_250 = 5,
        SPS_475 = 6,
        SPS_860 = 7
    };

    // Constructor: addr7 is the 7-bit address; we shift << 1 for HAL.
    ADS1115(I2C_HandleTypeDef *hi2c,
            Addr7 addr7   = Addr7::GND,
            Pga pga       = Pga::FS_4_096V,
            DataRate dr   = DataRate::SPS_128);

    // Optional init: currently nothing mandatory, but good for future extension
    HAL_StatusTypeDef init();

    // Read a single-ended channel (0–3), returns voltage in 'voltage'.
    // Blocking single-shot conversion, uses OS bit and polls with a timeout.
    HAL_StatusTypeDef readSingleEnded(uint8_t channel, float &voltage);

    // Optionally read raw code if you want to handle conversion yourself.
    HAL_StatusTypeDef readSingleEndedRaw(uint8_t channel, int16_t &raw);

    // Change PGA / data rate at runtime
    void setPga(Pga pga);
    void setDataRate(DataRate dr);

private:
    I2C_HandleTypeDef *hi2c_;
    uint8_t addr8_;       // 8-bit address for HAL (7-bit << 1)

    Pga pga_;
    DataRate dr_;
    float full_scale_V_;  // corresponding full-scale voltage

    // Register pointers
    enum : uint8_t {
        REG_CONVERSION = 0x00,
        REG_CONFIG     = 0x01,
        REG_LO_THRESH  = 0x02,
        REG_HI_THRESH  = 0x03
    };

    // Low-level helpers
    HAL_StatusTypeDef writeReg16(uint8_t reg, uint16_t value);
    HAL_StatusTypeDef readReg16(uint8_t reg, uint16_t &value);

    // Helpers
    float fullScaleFromPga(Pga pga) const;
    uint16_t buildConfigWord(uint8_t mux_bits) const;
};




#endif /* INC_ADS1115_HPP_ */

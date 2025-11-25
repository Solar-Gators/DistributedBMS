/*
 * ads1115.cpp
 *
 *  Created on: Nov 24, 2025
 *      Author: samrb
 */


#include "ads1115.hpp"
#include <cmath>

// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------
ADS1115::ADS1115(I2C_HandleTypeDef *hi2c,
                 Addr7 addr7,
                 Pga pga,
                 DataRate dr)
    : hi2c_(hi2c),
      addr8_(static_cast<uint8_t>(static_cast<uint8_t>(addr7) << 1)),
      pga_(pga),
      dr_(dr),
      full_scale_V_(fullScaleFromPga(pga))
{}

// -----------------------------------------------------------------------------
// fullScaleFromPga: returns the full-scale range in volts for the given PGA
// -----------------------------------------------------------------------------
float ADS1115::fullScaleFromPga(Pga pga) const
{
    switch (pga) {
    case Pga::FS_6_144V: return 6.144f;
    case Pga::FS_4_096V: return 4.096f;
    case Pga::FS_2_048V: return 2.048f;
    case Pga::FS_1_024V: return 1.024f;
    case Pga::FS_0_512V: return 0.512f;
    case Pga::FS_0_256V: return 0.256f;
    default:             return 4.096f;
    }
}

// -----------------------------------------------------------------------------
// Public: init (currently no mandatory configuration; comparator disabled
// in every single-shot config word we build anyway)
// -----------------------------------------------------------------------------
HAL_StatusTypeDef ADS1115::init()
{
    // Could pre-write LO/HI threshold or default config here if desired.
    return HAL_OK;
}

// -----------------------------------------------------------------------------
// Low-level 16-bit register write/read (MSB first)
// -----------------------------------------------------------------------------
HAL_StatusTypeDef ADS1115::writeReg16(uint8_t reg, uint16_t value)
{
    uint8_t buf[2];
    buf[0] = static_cast<uint8_t>((value >> 8) & 0xFF); // MSB
    buf[1] = static_cast<uint8_t>(value & 0xFF);        // LSB

    return HAL_I2C_Mem_Write(hi2c_, addr8_, reg,
                             I2C_MEMADD_SIZE_8BIT,
                             buf, 2,
                             HAL_MAX_DELAY);
}

HAL_StatusTypeDef ADS1115::readReg16(uint8_t reg, uint16_t &value)
{
    uint8_t buf[2] = {0, 0};

    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(hi2c_, addr8_, reg,
                                            I2C_MEMADD_SIZE_8BIT,
                                            buf, 2,
                                            HAL_MAX_DELAY);
    if (st != HAL_OK) {
        return st;
    }

    value = (static_cast<uint16_t>(buf[0]) << 8) | buf[1];
    return HAL_OK;
}

// -----------------------------------------------------------------------------
// Build config word for a single-shot conversion on a specific MUX setting
//
// Config register bits:
// [15]    OS      : 1 = start single conversion
// [14:12] MUX     : input selection
// [11:9]  PGA     : full-scale range
// [8]     MODE    : 1 = single-shot
// [7:5]   DR      : data rate
// [4]     COMP_MODE : 0 = traditional
// [3]     COMP_POL  : 0 = active low
// [2]     COMP_LAT  : 0 = non-latching
// [1:0]   COMP_QUE  : 3 = disable comparator
// -----------------------------------------------------------------------------
uint16_t ADS1115::buildConfigWord(uint8_t mux_bits) const
{
    // OS = 1 (start conversion)
    uint16_t config = 0;
    config |= (1u << 15);

    // MUX bits [14:12]
    config |= (static_cast<uint16_t>(mux_bits & 0x07) << 12);

    // PGA bits [11:9]
    uint8_t pga_code = 0;
    switch (pga_) {
    case Pga::FS_6_144V: pga_code = 0; break;
    case Pga::FS_4_096V: pga_code = 1; break;
    case Pga::FS_2_048V: pga_code = 2; break;
    case Pga::FS_1_024V: pga_code = 3; break;
    case Pga::FS_0_512V: pga_code = 4; break;
    case Pga::FS_0_256V: pga_code = 5; break; // 5,6,7 all 0.256V
    }
    config |= (static_cast<uint16_t>(pga_code & 0x07) << 9);

    // MODE = 1 (single-shot)
    config |= (1u << 8);

    // DR bits [7:5]
    uint8_t dr_code = static_cast<uint8_t>(dr_) & 0x07;
    config |= (static_cast<uint16_t>(dr_code) << 5);

    // Comparator settings: disabled
    // COMP_MODE[4] = 0, COMP_POL[3] = 0, COMP_LAT[2] = 0, COMP_QUE[1:0] = 3
    config |= 0x0003; // COMP_QUE = 3 → disable comparator

    return config;
}

// -----------------------------------------------------------------------------
// Public: readSingleEndedRaw
// Single-ended channels map to MUX codes:
//  AIN0: 100 (4), AIN1: 101 (5), AIN2: 110 (6), AIN3: 111 (7)
// -----------------------------------------------------------------------------
HAL_StatusTypeDef ADS1115::readSingleEndedRaw(uint8_t channel, int16_t &raw)
{
    if (channel > 3) {
        return HAL_ERROR;
    }

    uint8_t mux_bits = 0;

    // Single-ended: AINx vs GND
    switch (channel) {
    case 0: mux_bits = 0b100; break; // AIN0-GND
    case 1: mux_bits = 0b101; break; // AIN1-GND
    case 2: mux_bits = 0b110; break; // AIN2-GND
    case 3: mux_bits = 0b111; break; // AIN3-GND
    default: mux_bits = 0b100; break;
    }

    uint16_t config = buildConfigWord(mux_bits);

    // Write config to start conversion
    HAL_StatusTypeDef st = writeReg16(REG_CONFIG, config);
    if (st != HAL_OK) {
        return st;
    }

    // Wait for conversion to complete by polling OS bit
    // (OS bit goes high when conversion finished)
    const uint32_t start = HAL_GetTick();
    const uint32_t timeout_ms = 50; // plenty for all DR settings

    while (true) {
        uint16_t cfg_read = 0;
        st = readReg16(REG_CONFIG, cfg_read);
        if (st != HAL_OK) {
            return st;
        }

        if (cfg_read & (1u << 15)) {
            // OS = 1 again → conversion complete
            break;
        }

        if ((HAL_GetTick() - start) > timeout_ms) {
            return HAL_TIMEOUT;
        }
    }

    // Read conversion result
    uint16_t conv_u16 = 0;
    st = readReg16(REG_CONVERSION, conv_u16);
    if (st != HAL_OK) {
        return st;
    }

    raw = static_cast<int16_t>(conv_u16);
    return HAL_OK;
}

// -----------------------------------------------------------------------------
// Public: readSingleEnded (returns voltage)
// -----------------------------------------------------------------------------
HAL_StatusTypeDef ADS1115::readSingleEnded(uint8_t channel, float &voltage)
{
    int16_t raw = 0;
    HAL_StatusTypeDef st = readSingleEndedRaw(channel, raw);
    if (st != HAL_OK) {
        voltage = 0.0f;
        return st;
    }

    // Convert to voltage:
    // LSB = full_scale / 32768 (since range is ±FS, 16-bit signed)
    const float lsb = full_scale_V_ / 32768.0f;
    float v = static_cast<float>(raw) * lsb;

    // For single-ended mode, negative readings are usually noise / offset.
    if (v < 0.0f) {
        v = 0.0f;
    }

    voltage = v;
    return HAL_OK;
}

// -----------------------------------------------------------------------------
// Setters: update PGA / DR at runtime
// -----------------------------------------------------------------------------
void ADS1115::setPga(Pga pga)
{
    pga_ = pga;
    full_scale_V_ = fullScaleFromPga(pga_);
}

void ADS1115::setDataRate(DataRate dr)
{
    dr_ = dr;
}



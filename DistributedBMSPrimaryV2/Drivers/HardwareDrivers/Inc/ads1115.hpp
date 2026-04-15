#pragma once

extern "C" {
#include "stm32g4xx_hal.h"
}

#include <cstdint>

class ADS1115 {
public:
    // 7-bit I2C addresses (ADDR pin wiring); HAL uses addr << 1.
    enum class Addr7 : uint8_t {
        GND = 0x48,
        VDD = 0x49,
        SDA = 0x4A,
        SCL = 0x4B
    };

    // PGA full-scale ranges → bits [11:9] in config register
    enum class Pga : uint8_t {
        FS_6_144V = 0,
        FS_4_096V = 1,
        FS_2_048V = 2,
        FS_1_024V = 3,
        FS_0_512V = 4,
        FS_0_256V = 5
    };

    enum class DataRate : uint8_t {
        SPS_8 = 0,
        SPS_16 = 1,
        SPS_32 = 2,
        SPS_64 = 3,
        SPS_128 = 4,
        SPS_250 = 5,
        SPS_475 = 6,
        SPS_860 = 7
    };

    ADS1115(I2C_HandleTypeDef* hi2c, Addr7 addr7 = Addr7::GND, Pga pga = Pga::FS_4_096V,
            DataRate dr = DataRate::SPS_128);

    HAL_StatusTypeDef init();
    HAL_StatusTypeDef readSingleEnded(uint8_t channel, float& voltage);
    HAL_StatusTypeDef readSingleEndedRaw(uint8_t channel, int16_t& raw);

    void setPga(Pga pga);
    void setDataRate(DataRate dr);

private:
    I2C_HandleTypeDef* hi2c_;
    uint8_t addr8_;
    Pga pga_;
    DataRate dr_;
    float full_scale_V_;

    enum : uint8_t {
        REG_CONVERSION = 0x00,
        REG_CONFIG = 0x01,
        REG_LO_THRESH = 0x02,
        REG_HI_THRESH = 0x03
    };

    HAL_StatusTypeDef writeReg16(uint8_t reg, uint16_t value);
    HAL_StatusTypeDef readReg16(uint8_t reg, uint16_t& value);
    float fullScaleFromPga(Pga pga) const;
    uint16_t buildConfigWord(uint8_t mux_bits) const;
};

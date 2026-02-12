/*
 * lsm6dso32.hpp
 *
 *  Created on: Nov 23, 2025
 *      Author: samrb
 */

#ifndef INC_LSM6DSO32_HPP_
#define INC_LSM6DSO32_HPP_



#pragma once

// Adjust this include to your MCU family, e.g. "stm32l4xx_hal.h", "stm32f4xx_hal.h", etc.
extern "C" {
#include "stm32l5xx_hal.h"
}

#include <cstdint>

class LSM6DSO32 {
public:
    // I2C 7-bit base addresses (before HAL's <<1 shift)
    static constexpr uint8_t I2C_ADDR_SA0_LOW  = 0x6A; // SA0/SDO = GND
    static constexpr uint8_t I2C_ADDR_SA0_HIGH = 0x6B; // SA0/SDO = VDD

    // Output Data Rate code (ODR bits [7:4] in CTRL1_XL / CTRL2_G)
    enum class Odr : uint8_t {
        PowerDown   = 0x0,
        Hz_12_5     = 0x1,
        Hz_26       = 0x2,
        Hz_52       = 0x3,
        Hz_104      = 0x4,
        Hz_208      = 0x5,
        Hz_416      = 0x6,
        Hz_833      = 0x7,
        Hz_1k66     = 0x8,
        Hz_3k33     = 0x9,
        Hz_6k66     = 0xA
    };

    // Accelerometer full-scale configuration (FS_XL[1:0] in CTRL1_XL)
    // 00: ±4 g, 01: ±32 g, 10: ±8 g, 11: ±16 g :contentReference[oaicite:4]{index=4}
    enum class AccelFs : uint8_t {
        FS_4G   = 0b00,
        FS_32G  = 0b01,
        FS_8G   = 0b10,
        FS_16G  = 0b11
    };

    // Gyroscope full-scale configuration (FS_G[1:0] + FS_125 in CTRL2_G)
    // FS_G[1:0]: 00: ±250 dps, 01: ±500, 10: ±1000, 11: ±2000; FS_125 bit gives ±125 dps :contentReference[oaicite:5]{index=5}
    enum class GyroFs : uint8_t {
        FS_250DPS  = 0,
        FS_500DPS  = 1,
        FS_1000DPS = 2,
        FS_2000DPS = 3,
        FS_125DPS  = 4  // special case: uses FS_125 bit
    };

    struct RawData {
        int16_t temp_raw;
        int16_t gx_raw;
        int16_t gy_raw;
        int16_t gz_raw;
        int16_t ax_raw;
        int16_t ay_raw;
        int16_t az_raw;
    };

    struct ScaledData {
        float temp_degC;
        float gx_dps;
        float gy_dps;
        float gz_dps;
        float ax_mps2;
        float ay_mps2;
        float az_mps2;
    };

    // i2c_addr should be HAL-style 8-bit address (7-bit << 1), so default is 0x6A << 1
    LSM6DSO32(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr = (I2C_ADDR_SA0_LOW << 1));

    // Initialize sensor: sets BDU, IF_INC, accel+gyro ODR/FS.
    HAL_StatusTypeDef init(Odr accel_odr = Odr::Hz_104,
                           AccelFs accel_fs = AccelFs::FS_4G,
                           Odr gyro_odr = Odr::Hz_104,
                           GyroFs gyro_fs = GyroFs::FS_2000DPS);

    // Read WHO_AM_I register (should be 0x6C)
    HAL_StatusTypeDef readWhoAmI(uint8_t &who_am_i);

    // Read raw registers OUT_TEMP_L through OUTZ_H_A (14 bytes)
    HAL_StatusTypeDef readRaw(RawData &raw);

    // Read and convert to physical units
    HAL_StatusTypeDef readScaled(ScaledData &scaled);

private:
    I2C_HandleTypeDef *hi2c_;
    uint8_t addr_; // HAL 8-bit I2C address (7-bit << 1)

    AccelFs accel_fs_;
    GyroFs  gyro_fs_;

    // Low-level register access
    HAL_StatusTypeDef writeReg(uint8_t reg, uint8_t value);
    HAL_StatusTypeDef readRegs(uint8_t start_reg, uint8_t *buf, uint16_t len);

    // Sensitivity helpers (from datasheet) :contentReference[oaicite:6]{index=6}
    float accelSensitivity_g_per_lsb(AccelFs fs) const;
    float gyroSensitivity_dps_per_lsb(GyroFs fs) const;
};


#endif /* INC_LSM6DSO32_HPP_ */

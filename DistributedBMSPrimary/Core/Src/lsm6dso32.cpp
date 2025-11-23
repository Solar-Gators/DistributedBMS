/*
 * lsm6dso32.cpp
 *
 *  Created on: Nov 23, 2025
 *      Author: samrb
 */




#include "lsm6dso32.hpp"

// -----------------------------------------------------------------------------
// Register map (subset) :contentReference[oaicite:7]{index=7}
// -----------------------------------------------------------------------------
static constexpr uint8_t REG_PIN_CTRL     = 0x02;
static constexpr uint8_t REG_WHO_AM_I     = 0x0F;
static constexpr uint8_t REG_CTRL1_XL     = 0x10;
static constexpr uint8_t REG_CTRL2_G      = 0x11;
static constexpr uint8_t REG_CTRL3_C      = 0x12;
static constexpr uint8_t REG_STATUS_REG   = 0x1E;
static constexpr uint8_t REG_OUT_TEMP_L   = 0x20; // then H, Gx, Gy, Gz, Ax, Ay, Az...
// 0x20..0x2D: TEMP_L/H, OUTX/Y/Z_G (22-27h), OUTX/Y/Z_A (28-2Dh) :contentReference[oaicite:8]{index=8}

static constexpr uint8_t WHO_AM_I_EXPECTED = 0x6C;

// CTRL3_C bits: BOOT(7) BDU(6) H_LACTIVE(5) PP_OD(4) SIM(3) IF_INC(2) 0(1) SW_RESET(0)
// We want BDU=1, IF_INC=1, SW_RESET=0 => 0b0100'0100 = 0x44 :contentReference[oaicite:9]{index=9}

// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------
LSM6DSO32::LSM6DSO32(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr)
    : hi2c_(hi2c),
      addr_(i2c_addr),
      accel_fs_(AccelFs::FS_4G),
      gyro_fs_(GyroFs::FS_2000DPS)
{}

// -----------------------------------------------------------------------------
// Low-level I2C helpers
// -----------------------------------------------------------------------------
HAL_StatusTypeDef LSM6DSO32::writeReg(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(hi2c_, addr_, reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &value, 1,
                             HAL_MAX_DELAY);
}

HAL_StatusTypeDef LSM6DSO32::readRegs(uint8_t start_reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(hi2c_, addr_, start_reg,
                            I2C_MEMADD_SIZE_8BIT,
                            buf, len,
                            HAL_MAX_DELAY);
}

// -----------------------------------------------------------------------------
// WHO_AM_I
// -----------------------------------------------------------------------------
HAL_StatusTypeDef LSM6DSO32::readWhoAmI(uint8_t &who_am_i)
{
    return readRegs(REG_WHO_AM_I, &who_am_i, 1);
}

// -----------------------------------------------------------------------------
// Init
// -----------------------------------------------------------------------------
HAL_StatusTypeDef LSM6DSO32::init(Odr accel_odr, AccelFs accel_fs,
                                  Odr gyro_odr, GyroFs gyro_fs)
{
    HAL_StatusTypeDef status;

    // 1) Optionally check WHO_AM_I
    uint8_t who = 0;
    status = readWhoAmI(who);
    if (status != HAL_OK) {
        return status;
    }
    // You can uncomment this if you want to enforce the ID:
    // if (who != WHO_AM_I_EXPECTED) return HAL_ERROR;

    // 2) CTRL3_C: BDU=1, IF_INC=1, SW_RESET=0
    status = writeReg(REG_CTRL3_C, 0x44);
    if (status != HAL_OK) {
        return status;
    }

    // A short delay after configuration
    HAL_Delay(10);

    // 3) Configure accelerometer CTRL1_XL:
    // [7:4] ODR_XL, [3:2] FS_XL, [1] LPF2_EN (0=first stage), [0] 0
    uint8_t odr_xl_bits = static_cast<uint8_t>(accel_odr) & 0x0F;
    uint8_t fs_xl_bits  = static_cast<uint8_t>(accel_fs) & 0x03;
    uint8_t ctrl1_xl    = (odr_xl_bits << 4) | (fs_xl_bits << 2);
    status = writeReg(REG_CTRL1_XL, ctrl1_xl);
    if (status != HAL_OK) {
        return status;
    }

    // 4) Configure gyroscope CTRL2_G:
    // [7:4] ODR_G, [3:2] FS_G, [1] FS_125, [0] 0
    uint8_t odr_g_bits = static_cast<uint8_t>(gyro_odr) & 0x0F;
    uint8_t fs_g_bits  = 0;
    uint8_t fs_125_bit = 0;

    switch (gyro_fs) {
    case GyroFs::FS_125DPS:
        fs_g_bits  = 0;      // FS_G = 00
        fs_125_bit = 1;      // FS_125 = 1
        break;
    case GyroFs::FS_250DPS:
        fs_g_bits  = 0;
        fs_125_bit = 0;
        break;
    case GyroFs::FS_500DPS:
        fs_g_bits  = 1;
        fs_125_bit = 0;
        break;
    case GyroFs::FS_1000DPS:
        fs_g_bits  = 2;
        fs_125_bit = 0;
        break;
    case GyroFs::FS_2000DPS:
    default:
        fs_g_bits  = 3;
        fs_125_bit = 0;
        break;
    }

    uint8_t ctrl2_g = (odr_g_bits << 4)
                    | ((fs_g_bits & 0x03) << 2)
                    | ((fs_125_bit & 0x01) << 1);
    status = writeReg(REG_CTRL2_G, ctrl2_g);
    if (status != HAL_OK) {
        return status;
    }

    accel_fs_ = accel_fs;
    gyro_fs_  = gyro_fs;

    // Give sensor some time to start
    HAL_Delay(10);

    return HAL_OK;
}

// -----------------------------------------------------------------------------
// Read raw block: TEMP, Gx, Gy, Gz, Ax, Ay, Az
// -----------------------------------------------------------------------------
HAL_StatusTypeDef LSM6DSO32::readRaw(RawData &raw)
{
    uint8_t buf[14] = {0};

    HAL_StatusTypeDef status = readRegs(REG_OUT_TEMP_L, buf, sizeof(buf));
    if (status != HAL_OK) {
        return status;
    }

    // Little-endian 16-bit signed values
    raw.temp_raw = static_cast<int16_t>((uint16_t(buf[1]) << 8) | buf[0]);
    raw.gx_raw   = static_cast<int16_t>((uint16_t(buf[3]) << 8) | buf[2]);
    raw.gy_raw   = static_cast<int16_t>((uint16_t(buf[5]) << 8) | buf[4]);
    raw.gz_raw   = static_cast<int16_t>((uint16_t(buf[7]) << 8) | buf[6]);
    raw.ax_raw   = static_cast<int16_t>((uint16_t(buf[9]) << 8) | buf[8]);
    raw.ay_raw   = static_cast<int16_t>((uint16_t(buf[11]) << 8) | buf[10]);
    raw.az_raw   = static_cast<int16_t>((uint16_t(buf[13]) << 8) | buf[12]);

    return HAL_OK;
}

// -----------------------------------------------------------------------------
// Read and scale to physical units
// -----------------------------------------------------------------------------
HAL_StatusTypeDef LSM6DSO32::readScaled(ScaledData &scaled)
{
    RawData r{};
    HAL_StatusTypeDef status = readRaw(r);
    if (status != HAL_OK) {
        return status;
    }

    // Temperature: T[°C] = 25 + TEMP_OUT / 256 :contentReference[oaicite:10]{index=10}
    scaled.temp_degC = 25.0f + (static_cast<float>(r.temp_raw) / 256.0f);

    // Sensitivities
    const float a_sens_g   = accelSensitivity_g_per_lsb(accel_fs_);
    const float g_sens_dps = gyroSensitivity_dps_per_lsb(gyro_fs_);

    // Accel: convert LSB -> g -> m/s^2
    const float g_to_mps2 = 9.80665f;

    scaled.ax_mps2 = static_cast<float>(r.ax_raw) * a_sens_g * g_to_mps2/2;
    scaled.ay_mps2 = static_cast<float>(r.ay_raw) * a_sens_g * g_to_mps2/2;
    scaled.az_mps2 = static_cast<float>(r.az_raw) * a_sens_g * g_to_mps2/2;

    // Gyro: LSB -> dps
    scaled.gx_dps = static_cast<float>(r.gx_raw) * g_sens_dps;
    scaled.gy_dps = static_cast<float>(r.gy_raw) * g_sens_dps;
    scaled.gz_dps = static_cast<float>(r.gz_raw) * g_sens_dps;

    return HAL_OK;
}

// -----------------------------------------------------------------------------
// Sensitivity helpers (datasheet mechanical characteristics) :contentReference[oaicite:11]{index=11}
// -----------------------------------------------------------------------------
// LA_So (accel) :
//   ±4 g  -> 0.122 mg/LSB
//   ±8 g  -> 0.244 mg/LSB
//   ±16 g -> 0.488 mg/LSB
//   ±32 g -> 0.976 mg/LSB
float LSM6DSO32::accelSensitivity_g_per_lsb(AccelFs fs) const
{
    float mg_per_lsb = 0.122f; // default FS_4G

    switch (fs) {
    case AccelFs::FS_4G:   mg_per_lsb = 0.122f; break;
    case AccelFs::FS_8G:   mg_per_lsb = 0.244f; break;
    case AccelFs::FS_16G:  mg_per_lsb = 0.488f; break;
    case AccelFs::FS_32G:  mg_per_lsb = 0.976f; break;
    default:               mg_per_lsb = 0.122f; break;
    }

    return mg_per_lsb * 1.0e-3f; // convert mg -> g
}

// G_So (gyro):
//   ±125 dps  ->  4.375 mdps/LSB
//   ±250 dps  ->  8.75  mdps/LSB
//   ±500 dps  -> 17.50  mdps/LSB
//   ±1000 dps -> 35.0   mdps/LSB
//   ±2000 dps -> 70.0   mdps/LSB
float LSM6DSO32::gyroSensitivity_dps_per_lsb(GyroFs fs) const
{
    float mdps_per_lsb = 8.75f; // default to 250 dps

    switch (fs) {
    case GyroFs::FS_125DPS:  mdps_per_lsb = 4.375f; break;
    case GyroFs::FS_250DPS:  mdps_per_lsb = 8.75f;  break;
    case GyroFs::FS_500DPS:  mdps_per_lsb = 17.50f; break;
    case GyroFs::FS_1000DPS: mdps_per_lsb = 35.0f;  break;
    case GyroFs::FS_2000DPS: mdps_per_lsb = 70.0f;  break;
    default:                 mdps_per_lsb = 8.75f;  break;
    }

    return mdps_per_lsb * 1.0e-3f; // convert mdps -> dps
}

#include "ads1115.hpp"

ADS1115::ADS1115(I2C_HandleTypeDef* hi2c, Addr7 addr7, Pga pga, DataRate dr)
    : hi2c_(hi2c),
      addr8_(static_cast<uint8_t>(static_cast<uint8_t>(addr7) << 1)),
      pga_(pga),
      dr_(dr),
      full_scale_V_(fullScaleFromPga(pga)) {}

float ADS1115::fullScaleFromPga(Pga pga) const {
    switch (pga) {
    case Pga::FS_6_144V:
        return 6.144f;
    case Pga::FS_4_096V:
        return 4.096f;
    case Pga::FS_2_048V:
        return 2.048f;
    case Pga::FS_1_024V:
        return 1.024f;
    case Pga::FS_0_512V:
        return 0.512f;
    case Pga::FS_0_256V:
        return 0.256f;
    default:
        return 4.096f;
    }
}

HAL_StatusTypeDef ADS1115::init() {
    return HAL_OK;
}

HAL_StatusTypeDef ADS1115::writeReg16(uint8_t reg, uint16_t value) {
    uint8_t buf[2];
    buf[0] = static_cast<uint8_t>((value >> 8) & 0xFF);
    buf[1] = static_cast<uint8_t>(value & 0xFF);
    return HAL_I2C_Mem_Write(hi2c_, addr8_, reg, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY);
}

HAL_StatusTypeDef ADS1115::readReg16(uint8_t reg, uint16_t& value) {
    uint8_t buf[2]{};
    const HAL_StatusTypeDef st =
        HAL_I2C_Mem_Read(hi2c_, addr8_, reg, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY);
    if (st != HAL_OK) {
        return st;
    }
    value = static_cast<uint16_t>((static_cast<uint16_t>(buf[0]) << 8) | buf[1]);
    return HAL_OK;
}

uint16_t ADS1115::buildConfigWord(uint8_t mux_bits) const {
    uint16_t config = 0;
    config |= (1u << 15);
    config |= (static_cast<uint16_t>(mux_bits & 0x07) << 12);

    uint8_t pga_code = 0;
    switch (pga_) {
    case Pga::FS_6_144V:
        pga_code = 0;
        break;
    case Pga::FS_4_096V:
        pga_code = 1;
        break;
    case Pga::FS_2_048V:
        pga_code = 2;
        break;
    case Pga::FS_1_024V:
        pga_code = 3;
        break;
    case Pga::FS_0_512V:
        pga_code = 4;
        break;
    case Pga::FS_0_256V:
        pga_code = 5;
        break;
    }
    config |= (static_cast<uint16_t>(pga_code & 0x07) << 9);
    config |= (1u << 8);
    const uint8_t dr_code = static_cast<uint8_t>(dr_) & 0x07;
    config |= (static_cast<uint16_t>(dr_code) << 5);
    config |= 0x0003;
    return config;
}

HAL_StatusTypeDef ADS1115::readSingleEndedRaw(uint8_t channel, int16_t& raw) {
    if (channel > 3) {
        return HAL_ERROR;
    }
    uint8_t mux_bits = 0;
    switch (channel) {
    case 0:
        mux_bits = 0b100;
        break;
    case 1:
        mux_bits = 0b101;
        break;
    case 2:
        mux_bits = 0b110;
        break;
    case 3:
        mux_bits = 0b111;
        break;
    default:
        mux_bits = 0b100;
        break;
    }
    const uint16_t config = buildConfigWord(mux_bits);
    HAL_StatusTypeDef st = writeReg16(REG_CONFIG, config);
    if (st != HAL_OK) {
        return st;
    }
    const uint32_t start = HAL_GetTick();
    const uint32_t timeout_ms = 50;
    while (true) {
        uint16_t cfg_read = 0;
        st = readReg16(REG_CONFIG, cfg_read);
        if (st != HAL_OK) {
            return st;
        }
        if (cfg_read & (1u << 15)) {
            break;
        }
        if ((HAL_GetTick() - start) > timeout_ms) {
            return HAL_TIMEOUT;
        }
    }
    uint16_t conv_u16 = 0;
    st = readReg16(REG_CONVERSION, conv_u16);
    if (st != HAL_OK) {
        return st;
    }
    raw = static_cast<int16_t>(conv_u16);
    return HAL_OK;
}

HAL_StatusTypeDef ADS1115::readSingleEnded(uint8_t channel, float& voltage) {
    int16_t raw = 0;
    const HAL_StatusTypeDef st = readSingleEndedRaw(channel, raw);
    if (st != HAL_OK) {
        voltage = 0.0f;
        return st;
    }
    const float lsb = full_scale_V_ / 32768.0f;
    float v = static_cast<float>(raw) * lsb;
    if (v < 0.0f) {
        v = 0.0f;
    }
    voltage = v;
    return HAL_OK;
}

void ADS1115::setPga(Pga pga) {
    pga_ = pga;
    full_scale_V_ = fullScaleFromPga(pga_);
}

void ADS1115::setDataRate(DataRate dr) {
    dr_ = dr;
}

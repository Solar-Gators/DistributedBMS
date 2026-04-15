#include "ina226.hpp"

#include <cmath>

INA226::INA226(I2C_HandleTypeDef* hi2c, uint8_t addr_7bit)
    // STM32 HAL uses 8-bit I2C address format (7-bit device address shifted left by 1).
    : hi2c_(hi2c), addr_(static_cast<uint8_t>(addr_7bit << 1)) {}

HAL_StatusTypeDef INA226::writeReg16(uint8_t reg, uint16_t value) {
    // INA226 register transfers are big-endian: MSB first, then LSB.
    uint8_t buf[2];
    buf[0] = static_cast<uint8_t>((value >> 8) & 0xFF);
    buf[1] = static_cast<uint8_t>(value & 0xFF);
    return HAL_I2C_Mem_Write(hi2c_, addr_, reg, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY);
}

HAL_StatusTypeDef INA226::readReg16(uint8_t reg, uint16_t& value) {
    uint8_t buf[2]{};
    const HAL_StatusTypeDef st =
        HAL_I2C_Mem_Read(hi2c_, addr_, reg, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY);
    if (st != HAL_OK) {
        return st;
    }
    // Reassemble big-endian register bytes to host uint16_t.
    value = static_cast<uint16_t>((static_cast<uint16_t>(buf[0]) << 8) | buf[1]);
    return HAL_OK;
}

uint16_t INA226::computeCalibration(float shunt_res_ohm, float max_current_A) {
    if (shunt_res_ohm <= 0.0f || max_current_A <= 0.0f) {
        current_lsb_A_ = 0.0f;
        power_lsb_W_ = 0.0f;
        return 0;
    }
    // INA226 current register is signed 16-bit.
    // Choose current_LSB so max expected current maps near full-scale: I_LSB = I_max / 32768.
    current_lsb_A_ = max_current_A / 32768.0f;
    // Calibration equation from INA226 datasheet:
    // CAL = 0.00512 / (current_LSB * Rshunt).
    const float cal_f = 0.00512f / (current_lsb_A_ * shunt_res_ohm);
    const uint16_t cal_reg = static_cast<uint16_t>(std::round(cal_f));
    // Power register LSB is fixed at 25 * current_LSB.
    power_lsb_W_ = 25.0f * current_lsb_A_;
    return cal_reg;
}

HAL_StatusTypeDef INA226::init(float shunt_res_ohm, float max_current_A, uint16_t config) {
    // Program operating mode/averaging/conversion times first, then write CAL so
    // CURRENT/POWER registers use the intended scaling constants.
    shunt_res_ohm_ = shunt_res_ohm;
    HAL_StatusTypeDef st = writeConfig(config);
    if (st != HAL_OK) {
        return st;
    }
    const uint16_t cal = computeCalibration(shunt_res_ohm, max_current_A);
    st = writeCalibration(cal);
    if (st != HAL_OK) {
        return st;
    }
    HAL_Delay(2);
    return HAL_OK;
}

HAL_StatusTypeDef INA226::writeConfig(uint16_t config) {
    return writeReg16(REG_CONFIG, config);
}

HAL_StatusTypeDef INA226::writeCalibration(uint16_t cal) {
    return writeReg16(REG_CALIB, cal);
}

HAL_StatusTypeDef INA226::readShuntVoltage(float& volts) {
    uint16_t raw_u16 = 0;
    const HAL_StatusTypeDef st = readReg16(REG_SHUNT_V, raw_u16);
    if (st != HAL_OK) {
        return st;
    }
    // Shunt-voltage register is signed; LSB = 2.5 uV.
    const int16_t raw = static_cast<int16_t>(raw_u16);
    volts = static_cast<float>(raw) * 2.5e-6f;
    return HAL_OK;
}

HAL_StatusTypeDef INA226::readBusVoltage(float& volts) {
    uint16_t raw = 0;
    HAL_StatusTypeDef st = readReg16(REG_BUS_V, raw);
    if (st != HAL_OK) return st;

    // Bus-voltage register LSB = 1.25 mV.
    volts = static_cast<float>(raw) * 1.25e-3f;
    return HAL_OK;
}

HAL_StatusTypeDef INA226::readCurrent(float& amps) {
    if (current_lsb_A_ <= 0.0f) {
        amps = 0.0f;
        return HAL_ERROR;
    }
    uint16_t raw_u16 = 0;
    const HAL_StatusTypeDef st = readReg16(REG_CURRENT, raw_u16);
    if (st != HAL_OK) {
        return st;
    }
    // CURRENT register is signed 16-bit and requires prior CAL programming.
    const int16_t raw = static_cast<int16_t>(raw_u16);
    amps = static_cast<float>(raw) * current_lsb_A_;
    return HAL_OK;
}

HAL_StatusTypeDef INA226::readPower(float& watts) {
    if (power_lsb_W_ <= 0.0f) {
        watts = 0.0f;
        return HAL_ERROR;
    }
    uint16_t raw = 0;
    const HAL_StatusTypeDef st = readReg16(REG_POWER, raw);
    if (st != HAL_OK) {
        return st;
    }
    // POWER register is unsigned and scaled by power_LSB = 25 * current_LSB.
    watts = static_cast<float>(raw) * power_lsb_W_;
    return HAL_OK;
}

HAL_StatusTypeDef INA226::readMeasurement(Measurement& m) {
    // Read sequence mirrors INA226 measurement chain and returns first failing status.
    HAL_StatusTypeDef st = readShuntVoltage(m.shunt_V);
    if (st != HAL_OK) {
        return st;
    }
    st = readBusVoltage(m.bus_V);
    if (st != HAL_OK) {
        return st;
    }
    st = readCurrent(m.current_A);
    if (st != HAL_OK) {
        return st;
    }
    st = readPower(m.power_W);
    return st;
}

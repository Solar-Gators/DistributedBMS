#include "ina226.hpp"



#include <algorithm>

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

    current_lsb_A_ = 0.0f;

    power_lsb_W_ = 0.0f;

    cal_reg_ = 0;



    if (shunt_res_ohm <= 0.0f || max_current_A <= 0.0f) {

        return 0;

    }



    // Clamp expected current to what the shunt ADC can represent (datasheet +/-81.92 mV).

    const float fs_current_A = fullScaleCurrent_A(shunt_res_ohm);

    const float i_max_A = std::min(max_current_A, fs_current_A);



    // Eq. 2: Current_LSB = I_max / 2^15 (highest resolution).

    const float requested_lsb_A = i_max_A / 32768.0f;



    // Eq. 1: CAL = 0.00512 / (Current_LSB * Rshunt). CAL is 16-bit; keep within range.

    const float cal_f = kCalConstant / (requested_lsb_A * shunt_res_ohm);

    uint16_t cal_reg = static_cast<uint16_t>(std::round(cal_f));

    if (cal_reg < 1u) {

        cal_reg = 1u;

    }

    if (cal_reg > 32767u) {

        cal_reg = 32767u;

    }



    // Effective LSB after CAL quantization (datasheet Eq. 3 + register scaling).

    current_lsb_A_ = kCalConstant / (static_cast<float>(cal_reg) * shunt_res_ohm);

    power_lsb_W_ = 25.0f * current_lsb_A_;

    cal_reg_ = cal_reg;

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

    if (cal == 0u) {

        return HAL_ERROR;

    }

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

    // Shunt-voltage register is signed; LSB = 2.5 uV (datasheet §7.1.2).

    const int16_t raw = static_cast<int16_t>(raw_u16);

    volts = static_cast<float>(raw) * kShuntVoltageLsb_V;

    return HAL_OK;

}



HAL_StatusTypeDef INA226::readBusVoltage(float& volts) {

    uint16_t raw = 0;

    const HAL_StatusTypeDef st = readReg16(REG_BUS_V, raw);

    if (st != HAL_OK) {

        return st;

    }

    // Bus voltage: bits [15:3], LSB = 1.25 mV (datasheet §7.1.3).

    const uint16_t v_raw = raw >> 3;

    volts = static_cast<float>(v_raw) * kBusVoltageLsb_V;

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

    // CURRENT register is signed 16-bit; LSB = programmed Current_LSB (datasheet §7.1.5).

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

    // POWER register is unsigned; LSB = 25 * Current_LSB (datasheet §7.1.4).

    watts = static_cast<float>(raw) * power_lsb_W_;

    return HAL_OK;

}



HAL_StatusTypeDef INA226::readManufacturerId(uint16_t& manuf_id) {

    return readReg16(REG_MANUF_ID, manuf_id);

}



HAL_StatusTypeDef INA226::probe(uint16_t& manuf_id) {

    manuf_id = 0;

    return readManufacturerId(manuf_id);

}



HAL_StatusTypeDef INA226::readMeasurement(Measurement& m) {

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



#pragma once

extern "C" {
#include "stm32g4xx_hal.h"
}

#include <cstdint>

class INA226 {
public:
    // 7-bit base addresses (before HAL shift)
    static constexpr uint8_t I2C_ADDR_DEVICE1 = 0x40;   // A0/A1 = GND, see datasheet
    static constexpr uint8_t I2C_ADDR_DEVICE2 = 0x44;   // A1=Vcc, A0=GND

    struct Measurement {
        float shunt_V = 0.0f;
        float bus_V = 0.0f;
        float current_A = 0.0f;
        float power_W = 0.0f;
    };

    INA226(I2C_HandleTypeDef* hi2c, uint8_t addr_7bit);

    HAL_StatusTypeDef init(float shunt_res_ohm, float max_current_A, uint16_t config = 0x4127);

    HAL_StatusTypeDef readShuntVoltage(float& volts);
    HAL_StatusTypeDef readBusVoltage(float& volts);
    HAL_StatusTypeDef readCurrent(float& amps);
    HAL_StatusTypeDef readPower(float& watts);
    HAL_StatusTypeDef readMeasurement(Measurement& m);

    HAL_StatusTypeDef writeConfig(uint16_t config);
    HAL_StatusTypeDef writeCalibration(uint16_t cal);

private:
    I2C_HandleTypeDef* hi2c_;
    uint8_t addr_;

    float shunt_res_ohm_ = 0.0f;
    float current_lsb_A_ = 0.0f;
    float power_lsb_W_ = 0.0f;

    enum : uint8_t {
        REG_CONFIG = 0x00,
        REG_SHUNT_V = 0x01,
        REG_BUS_V = 0x02,
        REG_POWER = 0x03,
        REG_CURRENT = 0x04,
        REG_CALIB = 0x05,
        REG_MASK_ENABLE = 0x06,
        REG_ALERT_LIMIT = 0x07,
        REG_MANUF_ID = 0xFE,
        REG_DIE_ID = 0xFF
    };

    HAL_StatusTypeDef writeReg16(uint8_t reg, uint16_t value);
    HAL_StatusTypeDef readReg16(uint8_t reg, uint16_t& value);
    uint16_t computeCalibration(float shunt_res_ohm, float max_current_A);
};

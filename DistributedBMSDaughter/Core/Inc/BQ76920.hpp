#ifndef BQ76920_HPP
#define BQ76920_HPP

#include "stm32l4xx_hal.h"
#include <stdint.h>

class BQ76920 {
public:
    // Simple error codes
    enum class Error {
        OK = 0,
        HAL_ERROR,
        BAD_ARG
    };

    // Constructor
    BQ76920(I2C_HandleTypeDef* hi2c, uint8_t i2c_addr_7bit);

    // Init device
    Error init();

    // Control
    Error enableADC(bool enable);
    Error setFETs(bool chg_on, bool dsg_on);

    // Measurements
    Error readCellVoltage(uint8_t cell, uint16_t* mv_out);
    Error readPackVoltage(uint16_t* mv_out);
    Error readTemperature(uint16_t* temp_out); // raw ADC value

    // Status
    Error readSysStatus(uint8_t* status);
    Error clearSysStatus(uint8_t mask);

private:
    I2C_HandleTypeDef* hi2c_;
    uint8_t addr7_;

    // Low level helpers
    Error writeReg(uint8_t reg, uint8_t data);
    Error readReg(uint8_t reg, uint8_t* data);
    Error readRegs(uint8_t reg, uint8_t* buf, uint8_t len);

    // Conversion helpers
    uint16_t countsToMillivolts(uint16_t counts);
    uint16_t packCountsToMillivolts(uint16_t counts);
};

#endif // BQ76920_HPP

#include "BQ76920.hpp"

// BQ76920 register map (simplified for example)
#define SYS_STAT_REG     0x00
#define SYS_CTRL1_REG    0x04
#define SYS_CTRL2_REG    0x05
#define CELL_VOLTAGE_1   0x0C   // start of cell voltage registers (2 bytes each)
#define PACK_VOLTAGE_H   0x2A
#define TEMP_SENSOR_REG  0x2E

// Voltage LSBs from datasheet
#define CELL_VOLTAGE_LSB_mV   0.382   // ~382 µV per bit
#define PACK_VOLTAGE_LSB_mV   1.0     // 1 mV per bit

// Constructor
BQ76920::BQ76920(I2C_HandleTypeDef* hi2c, uint8_t i2c_addr_7bit)
    : hi2c_(hi2c), addr7_(i2c_addr_7bit << 1) {}  // HAL expects 8-bit addr

// Init device (basic ADC + FET enable)
BQ76920::Error BQ76920::init() {
    // Enable ADC
    return enableADC(true);
}

// Enable/disable ADC
BQ76920::Error BQ76920::enableADC(bool enable) {
    return writeReg(SYS_CTRL1_REG, enable ? 0x01 : 0x00);
}

// Control FETs
BQ76920::Error BQ76920::setFETs(bool chg_on, bool dsg_on) {
    uint8_t val = 0;
    if (chg_on) val |= 0x02;
    if (dsg_on) val |= 0x01;
    return writeReg(SYS_CTRL2_REG, val);
}

// Read a single cell voltage
BQ76920::Error BQ76920::readCellVoltage(uint8_t cell, uint16_t* mv_out) {
    if (cell < 1 || cell > 5) return Error::BAD_ARG;

    uint8_t buf[2];
    Error err = readRegs(CELL_VOLTAGE_1 + (2 * (cell - 1)), buf, 2);
    if (err != Error::OK) return err;

    uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
    *mv_out = countsToMillivolts(raw);
    return Error::OK;
}

// Read pack voltage
BQ76920::Error BQ76920::readPackVoltage(uint16_t* mv_out) {
    uint8_t buf[2];
    Error err = readRegs(PACK_VOLTAGE_H, buf, 2);
    if (err != Error::OK) return err;

    uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
    *mv_out = packCountsToMillivolts(raw);
    return Error::OK;
}

// Read temperature (raw ADC)
BQ76920::Error BQ76920::readTemperature(uint16_t* temp_out) {
    uint8_t buf[2];
    Error err = readRegs(TEMP_SENSOR_REG, buf, 2);
    if (err != Error::OK) return err;

    *temp_out = ((uint16_t)buf[0] << 8) | buf[1];
    return Error::OK;
}

// Status register
BQ76920::Error BQ76920::readSysStatus(uint8_t* status) {
    return readReg(SYS_STAT_REG, status);
}

BQ76920::Error BQ76920::clearSysStatus(uint8_t mask) {
    return writeReg(SYS_STAT_REG, mask);
}

// Low-level I2C
BQ76920::Error BQ76920::writeReg(uint8_t reg, uint8_t data) {
    if (HAL_I2C_Mem_Write(hi2c_, addr7_, reg, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK)
        return Error::HAL_ERROR;
    return Error::OK;
}

BQ76920::Error BQ76920::readReg(uint8_t reg, uint8_t* data) {
    if (HAL_I2C_Mem_Read(hi2c_, addr7_, reg, 1, data, 1, HAL_MAX_DELAY) != HAL_OK)
        return Error::HAL_ERROR;
    return Error::OK;
}

BQ76920::Error BQ76920::readRegs(uint8_t reg, uint8_t* buf, uint8_t len) {
    if (HAL_I2C_Mem_Read(hi2c_, addr7_, reg, 1, buf, len, HAL_MAX_DELAY) != HAL_OK)
        return Error::HAL_ERROR;
    return Error::OK;
}

// Conversion helpers
uint16_t BQ76920::countsToMillivolts(uint16_t counts) {
    return (uint16_t)(counts * CELL_VOLTAGE_LSB_mV);
}

uint16_t BQ76920::packCountsToMillivolts(uint16_t counts) {
    return (uint16_t)(counts * PACK_VOLTAGE_LSB_mV);
}
/*
BQ76920::Error BQ76920::selfTest() {
    uint8_t val;

    // 1. Try reading a known register (SYS_STAT should be readable)
    Error err = readReg(SYS_STAT_REG, &val);
    if (err != Error::OK) return err;

    // 2. Try writing & reading back SYS_CTRL1 (ADC enable/disable)
    err = writeReg(SYS_CTRL1_REG, 0x01);  // enable ADC
    if (err != Error::OK) return err;

    HAL_Delay(10);  // small delay for effect

    err = readReg(SYS_CTRL1_REG, &val);
    if (err != Error::OK) return err;
    if ((val & 0x01) == 0) return Error::VERIFY_FAIL;

    // 3. Try toggling FET control bits in SYS_CTRL2
    err = writeReg(SYS_CTRL2_REG, 0x03);  // enable both FETs
    if (err != Error::OK) return err;

    err = readReg(SYS_CTRL2_REG, &val);
    if (err != Error::OK) return err;
    if ((val & 0x03) != 0x03) return Error::VERIFY_FAIL;

    // 4. Check pack voltage register returns a sane value
    uint16_t pack_mv;
    err = readPackVoltage(&pack_mv);
    if (err != Error::OK) return err;
    if (pack_mv < 1000 || pack_mv > 25000) {
        // out of expected range for a 3-5s pack
        return Error::VERIFY_FAIL;
    }

    return Error::OK;
}
*/

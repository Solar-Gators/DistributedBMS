#pragma once

#include "i2c_api.hpp"

#include <array>

// BQ76925PWR driver — TI BQ76925 3–6 cell AFE (no internal ADC).
// Cell voltages are read via VCOUT -> host ADC; this driver does I2C config only.
// See TI datasheet SLUSAM9E (sections 8.5.1, 8.6) for addressing and register map.

#define BQ76925_ONE_BYTE 1
#define BQ76925_TWO_BYTES 2

// Series‑cell count on the new daughterboard (VC1..VC6)
#define BQ76925_CELL_COUNT 6

class BQ76925PWR : public I2CDevice
{
   public:
    BQ76925PWR(I2C_HandleTypeDef *h) : I2CDevice(h, 0), rawHandle_(h){};

    /**
     * @brief Device‑specific initialization sequence.
     *
     * Configure the BQ76925 according to the datasheet (reference design)
     * for gain/offset, ADC, cell sense, and protection behavior.
     */
    HAL_StatusTypeDef init();

    /**
     * @brief Configure VCOUT to output a specific cell voltage.
     *
     * cellIndex: 0 -> VC1, 1 -> VC2, ..., 5 -> VC6.
     * VCOUT_SEL is set to '01' (VCn) as per Table 7/8 in the datasheet.
     */
    HAL_StatusTypeDef setCellForVCOUT(uint8_t cellIndex);

    // Optional helpers for future expansion – left unimplemented for now.
    HAL_StatusTypeDef getBAT(uint16_t *data);
    HAL_StatusTypeDef getDieTemp(uint16_t *data);

   private:
    // Default I2C_GROUP_ADDR from datasheet is 0b0100 (section 8.5.1.1).
    static constexpr uint8_t I2C_GROUP_ADDR = 0b0100;

    // NOTE: BQ76925 uses a combined 7‑bit I2C address:
    // ADDRESS[6:0] = (I2C_GROUP_ADDR[3:0] << 3) + REG_ADDR[4:0]
    // We implement this in read/write helpers instead of using the base
    // I2CDevice::readN/writeN (which assume a standard device+register scheme).

    enum class registers : uint8_t
    {
        STATUS    = 0x00,
        CELL_CTL  = 0x01,
        BAL_CTL   = 0x02,
        CONFIG_1  = 0x03,
        CONFIG_2  = 0x04,
        POWER_CTL = 0x05,
        CHIP_ID   = 0x07,
    };

    uint8_t status_ = 0;
    I2C_HandleTypeDef *rawHandle_;

    // Low-level helpers that implement the combined-address I2C scheme.
    HAL_StatusTypeDef readReg(uint8_t reg, uint8_t *buf, size_t len);
    HAL_StatusTypeDef writeReg(uint8_t reg, const uint8_t *buf, size_t len);
};


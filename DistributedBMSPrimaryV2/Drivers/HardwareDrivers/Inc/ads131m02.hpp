#pragma once

extern "C" {
#include "stm32g4xx_hal.h"
}

#include <cstdint>

/**
 * ADS131M02 (IRUKT / WQFN) - 2-channel 24-bit delta-sigma ADC, SPI slave.
 *
 * SPI: TI specifies SPI mode 1 (CPOL=0, CPHA=1). On STM32 HAL use
 * SPI_POLARITY_LOW + SPI_PHASE_2EDGE (sample on falling edge).
 * MSB first, 8-bit SPI transfers; each device word is 24 bits (3 bytes).
 *
 * Default frame: 4 words (12 bytes) with input CRC disabled - command + zeros.
 * Response: status + CH0 + CH1 + output CRC word (see TI SBAS853).
 *
 * nCS must be GPIO (software NSS); drive low for the whole 12-byte transaction.
 *
 * Hardware: provide CLKIN per datasheet (e.g. 8.192 MHz crystal); tie SYNC/RESET high if unused.
 */
class Ads131m02 {
public:
    struct Pins {
        GPIO_TypeDef* ncs_port;
        uint16_t ncs_pin;
    };

    enum class Reg : uint8_t {
        ID = 0x00,
        STATUS = 0x01,
        MODE = 0x02,
        CLOCK = 0x03,
        GAIN = 0x04,
        CFG = 0x06,
        CH0_CFG = 0x09,
        CH1_CFG = 0x0E,
        REGMAP_CRC = 0x3E
    };

    enum class Osr : uint8_t {
        OSR_128 = 0,
        OSR_256 = 1,
        OSR_512 = 2,
        OSR_1024 = 3,
        OSR_2048 = 4,
        OSR_4096 = 5,
        OSR_8192 = 6,
        OSR_16384 = 7
    };

    enum class PgaGain : uint8_t {
        G1 = 0,
        G2 = 1,
        G4 = 2,
        G8 = 3,
        G16 = 4,
        G32 = 5,
        G64 = 6,
        G128 = 7
    };

    struct Config {
        Osr osr = Osr::OSR_1024;
        PgaGain gain_ch0 = PgaGain::G1;
        PgaGain gain_ch1 = PgaGain::G1;
        bool enable_ch0 = true;
        bool enable_ch1 = true;
    };

    Ads131m02(SPI_HandleTypeDef* hspi, const Pins& pins);

    HAL_StatusTypeDef reset();

    /** Default OSR/gains; use init(cfg) to customize. */
    HAL_StatusTypeDef init();
    HAL_StatusTypeDef init(const Config& cfg);

    HAL_StatusTypeDef readRegister(Reg addr, uint16_t& value);
    HAL_StatusTypeDef writeRegister(Reg addr, uint16_t value);

    HAL_StatusTypeDef readConversion(uint16_t& status, int32_t& ch0, int32_t& ch1);

    HAL_StatusTypeDef flushFifo();

    HAL_StatusTypeDef transferFrame(const uint8_t tx[12], uint8_t rx[12]);

    static void packCmd16ToWord24(uint16_t cmd16, uint8_t out3[3]);

    static uint16_t unpackWord24ToU16(const uint8_t w3[3]);
    static int32_t unpackWord24Signed(const uint8_t w3[3]);

    /** Differential voltage (V) from 24-bit code; gain divisor 1..128 (TI SBAS853 eq. 10). */
    static float voltsFromCode(int32_t code, unsigned pga_gain_divisor);
    static float voltsFromCode(int32_t code, PgaGain pga);

    /** ID register reset is 22xxh (bits 15:8 = 0x22); bits 7:0 are device-specific. */
    static constexpr uint16_t kIdRegisterHighMask = 0xFF00;
    static constexpr uint16_t kIdRegisterHighExpected = 0x2200;
    static constexpr uint16_t kIdChanCountMask = 0x0F00;
    static constexpr uint16_t kIdTwoChannels = 0x0200;

private:
    static uint16_t cmdReset();
    static uint16_t cmdRreg(uint8_t addr, uint8_t numRegsMinus1);
    static uint16_t cmdWreg(uint8_t addr, uint8_t numRegsMinus1);

    void csLow() const;
    void csHigh() const;

    HAL_StatusTypeDef applyConfig(const Config& cfg);
    /** NULL command frames (12 zero bytes) to align SPI pipeline after reset. */
    HAL_StatusTypeDef nullFrames(unsigned count);

    SPI_HandleTypeDef* hspi_;
    Pins pins_;
};

#include "ads131m02.hpp"

#include <cstring>

namespace {

constexpr uint32_t kSpiTimeout = 50;

void packWord24(uint16_t cmd16, uint8_t out3[3]) {
    // ADS131M02 defaults to 24-bit SPI words; 16-bit command/register payload is left-justified
    // in the word and padded with zeros in the lowest byte (datasheet Section 8.5.1.8).
    out3[0] = static_cast<uint8_t>((cmd16 >> 8) & 0xFF);
    out3[1] = static_cast<uint8_t>(cmd16 & 0xFF);
    out3[2] = 0;
}

void frameFromCmdAndZeros(uint16_t cmd16, uint8_t out12[12]) {
    packWord24(cmd16, out12);
    std::memset(out12 + 3, 0, 9);
}

}  // namespace

Ads131m02::Ads131m02(SPI_HandleTypeDef* hspi, const Pins& pins) : hspi_(hspi), pins_(pins) {}

void Ads131m02::csLow() const {
    HAL_GPIO_WritePin(pins_.ncs_port, pins_.ncs_pin, GPIO_PIN_RESET);
}

void Ads131m02::csHigh() const {
    HAL_GPIO_WritePin(pins_.ncs_port, pins_.ncs_pin, GPIO_PIN_SET);
}

void Ads131m02::packCmd16ToWord24(uint16_t cmd16, uint8_t out3[3]) {
    packWord24(cmd16, out3);
}

uint16_t Ads131m02::unpackWord24ToU16(const uint8_t w3[3]) {
    return static_cast<uint16_t>((static_cast<uint16_t>(w3[0]) << 8) | w3[1]);
}

int32_t Ads131m02::unpackWord24Signed(const uint8_t w3[3]) {
    // ADC channel data are 24-bit two's-complement codes (Section 8.5.1.9, Table 8-10).
    // Sign-extend to 32-bit for normal C++ arithmetic.
    uint32_t u = (static_cast<uint32_t>(w3[0]) << 16) | (static_cast<uint32_t>(w3[1]) << 8) |
                  static_cast<uint32_t>(w3[2]);
    if (u & 0x800000u) {
        u |= 0xFF000000u;
    }
    return static_cast<int32_t>(u);
}

float Ads131m02::voltsFromCode(int32_t code, unsigned pga_gain_divisor) {
    // Code-to-voltage mapping follows ADS131M0x transfer function:
    // Vin = code * (Vref / gain) / 2^23, with internal Vref = 2.4 V.
    // Using 2^24 here with signed code magnitude yields equivalent LSB scaling.
    unsigned g = (pga_gain_divisor == 0) ? 1u : pga_gain_divisor;
    const float lsb = (2.4f / static_cast<float>(g)) / 16777216.f;
    return static_cast<float>(code) * lsb;
}

float Ads131m02::voltsFromCode(int32_t code, PgaGain pga) {
    static const unsigned kGain[] = {1u, 2u, 4u, 8u, 16u, 32u, 64u, 128u};
    return voltsFromCode(code, kGain[static_cast<unsigned>(pga) & 7u]);
}

uint16_t Ads131m02::cmdReset() {
    return 0x0011;
}

uint16_t Ads131m02::cmdRreg(uint8_t addr, uint8_t numRegsMinus1) {
    const uint16_t a = static_cast<uint16_t>(addr & 0x1Fu);
    const uint16_t n = static_cast<uint16_t>(numRegsMinus1 & 0x7Fu);
    return static_cast<uint16_t>(0xA000u | (a << 7) | n);
}

uint16_t Ads131m02::cmdWreg(uint8_t addr, uint8_t numRegsMinus1) {
    const uint16_t a = static_cast<uint16_t>(addr & 0x1Fu);
    const uint16_t n = static_cast<uint16_t>(numRegsMinus1 & 0x7Fu);
    return static_cast<uint16_t>(0x6000u | (a << 7) | n);
}

HAL_StatusTypeDef Ads131m02::transferFrame(const uint8_t tx[12], uint8_t rx[12]) {
    // One communication unit is a complete SPI frame with CS held low for all words.
    // Raising CS resets the interface state machine (Section 8.5.1.1 and 8.5.1.7).
    csLow();
    const HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(hspi_, const_cast<uint8_t*>(tx), rx, 12, kSpiTimeout);
    csHigh();
    return st;
}

HAL_StatusTypeDef Ads131m02::applyConfig(const Config& cfg) {
    // Program CLOCK and GAIN registers using R/W register commands
    // (Section 8.5.1.10, Table 8-11 command definitions).
    uint16_t clock = 0x030Eu;
    const uint8_t osr = static_cast<uint8_t>(cfg.osr) & 7u;
    clock = static_cast<uint16_t>((clock & static_cast<uint16_t>(~(7u << 2))) | (static_cast<uint16_t>(osr) << 2));
    if (!cfg.enable_ch0) {
        clock = static_cast<uint16_t>(clock & static_cast<uint16_t>(~(1u << 8)));
    }
    if (!cfg.enable_ch1) {
        clock = static_cast<uint16_t>(clock & static_cast<uint16_t>(~(1u << 9)));
    }
    HAL_StatusTypeDef st = writeRegister(Reg::CLOCK, clock);
    if (st != HAL_OK) {
        return st;
    }
    const uint16_t gain = static_cast<uint16_t>(
        (static_cast<uint16_t>(static_cast<uint8_t>(cfg.gain_ch1) & 7u) << 4) |
        (static_cast<uint16_t>(static_cast<uint8_t>(cfg.gain_ch0) & 7u)));
    return writeRegister(Reg::GAIN, gain);
}

HAL_StatusTypeDef Ads131m02::nullFrames(unsigned count) {
    // NULL command frame (all zeros) returns STATUS + conversion data from prior pipeline slot.
    // Useful to advance/clear stale responses after reset or command sequences.
    uint8_t tx[12]{};
    uint8_t rx[12];
    for (unsigned i = 0; i < count; ++i) {
        const HAL_StatusTypeDef st = transferFrame(tx, rx);
        if (st != HAL_OK) {
            return st;
        }
    }
    return HAL_OK;
}

HAL_StatusTypeDef Ads131m02::reset() {
    // Software reset command = 0x0011 (Section 8.4.1.3 RESET Command, Table 8-11).
    // Command is latched only after the complete frame clocks out.
    uint8_t tx[12];
    frameFromCmdAndZeros(cmdReset(), tx);
    uint8_t rx[12];
    const HAL_StatusTypeDef st = transferFrame(tx, rx);
    if (st != HAL_OK) {
        return st;
    }
    HAL_Delay(1);
    return HAL_OK;
}

HAL_StatusTypeDef Ads131m02::init() {
    return init(Config{});
}

HAL_StatusTypeDef Ads131m02::init(const Config& cfg) {
    HAL_StatusTypeDef st = reset();
    if (st != HAL_OK) {
        return st;
    }
    // After reset, drain delayed command response/data pipeline before first RREG.
    // Datasheet notes command response appears in following frame and FIFO can hold stale packets
    // (Sections 8.5.1.7 and 8.5.1.9.1).
    st = nullFrames(2);
    if (st != HAL_OK) {
        return st;
    }
    uint16_t id = 0;
    st = readRegister(Reg::ID, id);
    if (st != HAL_OK) {
        return st;
    }
    if ((id & kIdRegisterHighMask) != kIdRegisterHighExpected) {
        return HAL_ERROR;
    }
    st = applyConfig(cfg);
    if (st != HAL_OK) {
        return st;
    }
    return flushFifo();
}

HAL_StatusTypeDef Ads131m02::readRegister(Reg addr, uint16_t& value) {
    // RREG response is returned in the following frame, not the same frame as the command.
    // See Section 8.5.1.7 and RREG command description in Section 8.5.1.10 / Table 8-11.
    uint8_t tx[12];
    frameFromCmdAndZeros(cmdRreg(static_cast<uint8_t>(addr), 0), tx);
    uint8_t rx[12];
    HAL_StatusTypeDef st = transferFrame(tx, rx);
    if (st != HAL_OK) {
        return st;
    }
    uint8_t nullTx[12]{};
    uint8_t nullRx[12];
    st = transferFrame(nullTx, nullRx);
    if (st != HAL_OK) {
        return st;
    }
    value = unpackWord24ToU16(nullRx);
    return HAL_OK;
}

HAL_StatusTypeDef Ads131m02::writeRegister(Reg addr, uint16_t value) {
    // WREG frame starts with command word, then register payload word(s).
    // For one register write, payload is placed in word 2 of this 4-word frame.
    uint8_t tx[12]{};
    packWord24(cmdWreg(static_cast<uint8_t>(addr), 0), tx);
    packWord24(value, tx + 3);
    uint8_t rx[12];
    return transferFrame(tx, rx);
}

HAL_StatusTypeDef Ads131m02::readConversion(uint16_t& status, int32_t& ch0, int32_t& ch1) {
    // NULL frame readout format at default word size:
    // word0 = STATUS response, word1 = CH0, word2 = CH1, word3 = CRC/unused.
    // See Sections 8.5.1.7 and 8.5.1.9.
    uint8_t tx[12]{};
    uint8_t rx[12];
    const HAL_StatusTypeDef st = transferFrame(tx, rx);
    if (st != HAL_OK) {
        return st;
    }
    status = unpackWord24ToU16(rx);
    ch0 = unpackWord24Signed(rx + 3);
    ch1 = unpackWord24Signed(rx + 6);
    return HAL_OK;
}

HAL_StatusTypeDef Ads131m02::flushFifo() {
    // FIFO can hold two packets; reading two frames realigns to freshest sample stream.
    // See "Collecting Data for the First Time or After a Pause" (Section 8.5.1.9.1).
    uint16_t s0{};
    int32_t a0{};
    int32_t b0{};
    HAL_StatusTypeDef st = readConversion(s0, a0, b0);
    if (st != HAL_OK) {
        return st;
    }
    uint16_t s1{};
    int32_t a1{};
    int32_t b1{};
    return readConversion(s1, a1, b1);
}

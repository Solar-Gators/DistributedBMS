#include "ads1115.hpp"

ADS1115::ADS1115(I2C_HandleTypeDef* hi2c, Addr7 addr7, Pga pga, DataRate dr)
    : hi2c_(hi2c),
      // STM32 HAL expects the device address in 8-bit form (7-bit address left-shifted by 1).
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
    // ADS1115 powers up with usable defaults; this driver configures per-conversion in buildConfigWord().
    // Keep init as a no-op hook in case future board bring-up needs explicit startup configuration.
    return HAL_OK;
}

HAL_StatusTypeDef ADS1115::writeReg16(uint8_t reg, uint16_t value) {

    // ADS1115 registers are transferred MSB first, then LSB (datasheet I2C register access section).
    uint8_t buf[2];
    buf[0] = static_cast<uint8_t>((value >> 8) & 0xFF);
    buf[1] = static_cast<uint8_t>(value & 0xFF);

    // Write 16-bit register payload via pointer-register addressing.
    return HAL_I2C_Mem_Write(hi2c_, addr8_, reg, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY);
}

HAL_StatusTypeDef ADS1115::readReg16(uint8_t reg, uint16_t& value) {
    
    uint8_t buf[2]{};
    // Read two bytes (MSB first) from selected register.
    const HAL_StatusTypeDef st =
        HAL_I2C_Mem_Read(hi2c_, addr8_, reg, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY);
    if (st != HAL_OK) {
        return st;
    }
    // Reassemble big-endian register bytes into host uint16_t.
    value = static_cast<uint16_t>((static_cast<uint16_t>(buf[0]) << 8) | buf[1]);
    return HAL_OK;
}

uint16_t ADS1115::buildConfigWord(uint8_t mux_bits) const {
    // TI ADS1115 datasheet:
    // - Section 8.1.3 "Config Register (P[1:0] = 01b)"
    // - Table 8-3 "Config Register Field Descriptions"
    // This function composes the 16-bit Config register image written before a single-shot conversion.
    uint16_t config = 0;

    // Bit 15 OS = 1: start a single conversion when in power-down/single-shot mode.
    // Ref: Table 8-3, field OS[15].
    config |= (1u << 15);

    // Bits 14:12 MUX: select input channel combination.
    // For this driver's single-ended path, caller passes 100b..111b (AIN0..AIN3 vs GND).
    // Ref: Section 8.1.3 (ADS1115 config layout), Table 8-3 field MUX[14:12].
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

    // Bits 11:9 PGA: full-scale range selection (maps enum to ADS1115 PGA code).
    // Ref: Table 8-3 field PGA[11:9].
    config |= (static_cast<uint16_t>(pga_code & 0x07) << 9);

    // Bit 8 MODE = 1: power-down single-shot mode (required for OS-triggered conversions).
    // Ref: Table 8-3 field MODE[8].
    config |= (1u << 8);

    // Bits 7:5 DR: data rate selection (SPS), sourced from driver DataRate enum.
    // Ref: Table 8-3 field DR[7:5].
    const uint8_t dr_code = static_cast<uint8_t>(dr_) & 0x07;
    config |= (static_cast<uint16_t>(dr_code) << 5);

    // Bits 1:0 COMP_QUE = 11b: disable comparator and free ALERT/RDY pin.
    // (Bits 4:2 left at 0 by default since comparator is disabled.)
    // Ref: Table 8-3 fields COMP_MODE[4], COMP_POL[3], COMP_LAT[2], COMP_QUE[1:0].
    config |= 0x0003;
    return config;
}

HAL_StatusTypeDef ADS1115::readSingleEndedRaw(uint8_t channel, int16_t& raw) {
    // TI ADS1115 datasheet references used in this function:
    // - Section 8.1.3, Table 8-3: Config register fields (MUX/OS/MODE/DR).
    // - Section 8.1.2, Table 8-2: Conversion register format (16-bit two's complement).
    // - Section 7.5.3: Register read/write transaction flow over I2C.
    if (channel > 3) {
        return HAL_ERROR;
    }

    // Map logical single-ended channel to MUX[14:12] code per ADS1115 Config register.
    // 100b: AIN0-GND, 101b: AIN1-GND, 110b: AIN2-GND, 111b: AIN3-GND (Table 8-3).
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

    // Build config with:
    // - OS=1 (start single conversion),
    // - MODE=1 (single-shot),
    // - selected MUX/PGA/DR,
    // - comparator disabled (COMP_QUE=11b).
    // See buildConfigWord() comments for field-by-field mapping (Section 8.1.3 / Table 8-3).
    const uint16_t config = buildConfigWord(mux_bits);
    HAL_StatusTypeDef st = writeReg16(REG_CONFIG, config);
    if (st != HAL_OK) {
        return st;
    }

    // Poll OS[15] until conversion completes.
    // In single-shot mode, OS reads 0 while converting and returns 1 when conversion data is ready.
    // Ref: Table 8-3, OS[15] behavior.
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

    // Read the completed conversion result from Conversion register (P[1:0]=00b).
    // Data is 16-bit two's complement, MSB first on I2C.
    // Refs: Section 8.1.2 / Table 8-2 and Section 7.5.3.
    uint16_t conv_u16 = 0;
    st = readReg16(REG_CONVERSION, conv_u16);
    if (st != HAL_OK) {
        return st;
    }
    // Bit-cast preserving raw ADC code bits; caller interprets sign/magnitude as int16_t.
    raw = static_cast<int16_t>(conv_u16);
    return HAL_OK;
}

HAL_StatusTypeDef ADS1115::readSingleEnded(uint8_t channel, float& voltage) {
    // High-level helper: performs single-ended conversion then scales raw code to volts.
    // Scale factor is LSB = FSR / 2^15 for ADS1115 16-bit bipolar two's complement coding.
    // Ref: datasheet transfer function/equation in "Device Functional Modes" (single-shot conversions).
    int16_t raw = 0;
    const HAL_StatusTypeDef st = readSingleEndedRaw(channel, raw);
    if (st != HAL_OK) {
        voltage = 0.0f;
        return st;
    }

    // Convert code to volts using currently selected full-scale range (PGA setting).
    const float lsb = full_scale_V_ / 32768.0f;
    float v = static_cast<float>(raw) * lsb;

    // Single-ended channels (AINx-GND) should not report negative physical voltage;
    // clamp small negative codes (noise/offset) to 0 V for caller convenience.
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

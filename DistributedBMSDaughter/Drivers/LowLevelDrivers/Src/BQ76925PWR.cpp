#include "BQ76925PWR.hpp"

#include <cstdint>

#define TRY_BQ76925(x)         \
    do                         \
    {                          \
        if ((x) != HAL_OK)     \
            return HAL_ERROR;  \
    } while (0)

HAL_StatusTypeDef BQ76925PWR::readReg(uint8_t reg, uint8_t *buf, size_t len)
{
    // ADDRESS[6:0] = (I2C_GROUP_ADDR[3:0] << 3) + REG_ADDR[4:0]
    const uint8_t reg_addr = static_cast<uint8_t>(reg);
    const uint8_t addr7    = static_cast<uint8_t>((I2C_GROUP_ADDR << 3) | reg_addr);
    const uint16_t addr8   = static_cast<uint16_t>(addr7 << 1);

    return HAL_I2C_Master_Receive(rawHandle_, addr8, buf, static_cast<uint16_t>(len), HAL_MAX_DELAY);
}

HAL_StatusTypeDef BQ76925PWR::writeReg(uint8_t reg, const uint8_t *buf, size_t len)
{
    const uint8_t reg_addr = static_cast<uint8_t>(reg);
    const uint8_t addr7    = static_cast<uint8_t>((I2C_GROUP_ADDR << 3) | reg_addr);
    const uint16_t addr8   = static_cast<uint16_t>(addr7 << 1);

    return HAL_I2C_Master_Transmit(
        rawHandle_, addr8, const_cast<uint8_t *>(buf), static_cast<uint16_t>(len), HAL_MAX_DELAY);
}

HAL_StatusTypeDef BQ76925PWR::init()
{
    // Basic bring-up:
    //  - Enable reference and cell amplifier
    //  - Select REF_SEL = 1 (3.0 V reference, VCOUT gain = 0.6)

    // POWER_CTL: D0 = REF_EN, D2 = VC_AMP_EN
    uint8_t power_ctl = 0;
    power_ctl |= (1u << 0);  // REF_EN
    power_ctl |= (1u << 2);  // VC_AMP_EN
    TRY_BQ76925(writeReg(static_cast<uint8_t>(registers::POWER_CTL), &power_ctl, BQ76925_ONE_BYTE));

    // CONFIG_2: D0 = REF_SEL (1 -> 3.0 V, VCOUT gain = 0.6)
    uint8_t config2 = 0x01;
    TRY_BQ76925(writeReg(static_cast<uint8_t>(registers::CONFIG_2), &config2, BQ76925_ONE_BYTE));

    // Optional: read STATUS to confirm communication and latch POR state.
    uint8_t dummy{};
    TRY_BQ76925(readReg(static_cast<uint8_t>(registers::STATUS), &dummy, BQ76925_ONE_BYTE));

    status_ = dummy;
    return HAL_OK;
}

HAL_StatusTypeDef BQ76925PWR::setCellForVCOUT(uint8_t cellIndex)
{
    if (cellIndex >= BQ76925_CELL_COUNT)
    {
        return HAL_ERROR;
    }

    // CELL_CTL (Table 6): D5:D4 = VCOUT_SEL, D2:D0 = CELL_SEL. D7 must be 0.
    // VCOUT_SEL = 01b -> VCOUT = VCn (Table 7). CELL_SEL 000..101 = VC1..VC6 (Table 8).
    const uint8_t cell_sel = static_cast<uint8_t>(cellIndex & 0x07u);
    uint8_t cell_ctl      = 0;
    cell_ctl |= (1 << 5);           // VCOUT_SEL = 01 (D4=1, D5=0)
    cell_ctl |= cell_sel;            // CELL_SEL in D2:D0

    TRY_BQ76925(writeReg(static_cast<uint8_t>(registers::CELL_CTL), &cell_ctl, BQ76925_ONE_BYTE));
    return HAL_OK;
}

HAL_StatusTypeDef BQ76925PWR::getBAT(uint16_t *data)
{
    (void)data;
    return HAL_ERROR;
}

HAL_StatusTypeDef BQ76925PWR::getDieTemp(uint16_t *data)
{
    (void)data;
    return HAL_ERROR;
}


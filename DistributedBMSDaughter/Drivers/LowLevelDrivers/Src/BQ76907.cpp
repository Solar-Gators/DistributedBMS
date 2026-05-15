#include "BQ76907.hpp"

HAL_StatusTypeDef BQ76907::init()
{
    return HAL_OK;
}

HAL_StatusTypeDef BQ76907::readDirect16(uint8_t command, uint16_t& value)
{
    uint8_t raw[2]{};
    HAL_StatusTypeDef st = readN(command, raw, 2);
    if (st != HAL_OK)
    {
        return st;
    }

    // BQ76907 direct command payloads are little-endian.
    value = static_cast<uint16_t>((static_cast<uint16_t>(raw[1]) << 8u) | raw[0]);
    return HAL_OK;
}

HAL_StatusTypeDef BQ76907::writeDirect16(uint8_t command, uint16_t value)
{
    uint8_t raw[2]{};
    // Subcommands are written as little-endian words.
    raw[0] = static_cast<uint8_t>(value & 0x00FFu);
    raw[1] = static_cast<uint8_t>((value >> 8u) & 0x00FFu);
    return writeN(command, raw, 2);
}

HAL_StatusTypeDef BQ76907::readCellVoltage(uint8_t cell_1_based, uint16_t& millivolts)
{
    if (cell_1_based == 0u || cell_1_based > MAX_CELLS)
    {
        return HAL_ERROR;
    }

    const uint8_t command = static_cast<uint8_t>(
        static_cast<uint8_t>(DirectCommand::CELL1_VOLTAGE) + static_cast<uint8_t>(2u * (cell_1_based - 1u)));
    return readDirect16(command, millivolts);
}

HAL_StatusTypeDef BQ76907::readStackVoltage(uint16_t& millivolts)
{
    return readDirect16(static_cast<uint8_t>(DirectCommand::STACK_VOLTAGE), millivolts);
}

HAL_StatusTypeDef BQ76907::readInternalTempDeciC(int16_t& deci_c)
{
    uint16_t raw = 0;
    HAL_StatusTypeDef st = readDirect16(static_cast<uint8_t>(DirectCommand::INT_TEMPERATURE), raw);
    if (st != HAL_OK)
    {
        return st;
    }
    deci_c = static_cast<int16_t>(raw);
    return HAL_OK;
}

HAL_StatusTypeDef BQ76907::readCurrentMilliAmps(int16_t& milliamps)
{
    uint16_t raw = 0;
    HAL_StatusTypeDef st = readDirect16(static_cast<uint8_t>(DirectCommand::CURRENT_CC2), raw);
    if (st != HAL_OK)
    {
        return st;
    }
    milliamps = static_cast<int16_t>(raw);
    return HAL_OK;
}

HAL_StatusTypeDef BQ76907::readTelemetry(uint8_t cell_count, Telemetry& out)
{
    if (cell_count == 0u || cell_count > MAX_CELLS)
    {
        return HAL_ERROR;
    }

    out = Telemetry{};
    out.cell_count = cell_count;

    for (uint8_t i = 0; i < cell_count; ++i)
    {
        HAL_StatusTypeDef st = readCellVoltage(static_cast<uint8_t>(i + 1u), out.cell_mV[i]);
        if (st != HAL_OK)
        {
            return st;
        }
    }

    HAL_StatusTypeDef st = readStackVoltage(out.stack_mV);
    if (st != HAL_OK)
    {
        return st;
    }

    st = readInternalTempDeciC(out.internal_temp_dC);
    if (st != HAL_OK)
    {
        return st;
    }

    return readCurrentMilliAmps(out.current_mA);
}

HAL_StatusTypeDef BQ76907::executeSubcommand(uint16_t subcommand)
{
    return writeDirect16(static_cast<uint8_t>(DirectCommand::SUBCOMMAND_LO), subcommand);
}

HAL_StatusTypeDef BQ76907::enterConfigUpdate()
{
    return executeSubcommand(static_cast<uint16_t>(Subcommand::SET_CFGUPDATE));
}

HAL_StatusTypeDef BQ76907::exitConfigUpdate()
{
    return executeSubcommand(static_cast<uint16_t>(Subcommand::EXIT_CFGUPDATE));
}

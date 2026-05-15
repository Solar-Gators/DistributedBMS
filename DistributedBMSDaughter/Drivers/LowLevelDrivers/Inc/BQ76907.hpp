#pragma once

#include "i2c_api.hpp"

#include <array>
#include <cstdint>

class BQ76907 : public I2CDevice
{
   public:
    static constexpr uint8_t DEFAULT_I2C_ADDR_8BIT = (0x08u << 1u);
    static constexpr uint8_t MAX_CELLS = 7u;

    struct Telemetry
    {
        std::array<uint16_t, MAX_CELLS> cell_mV{};
        uint8_t cell_count = 0;
        uint16_t stack_mV = 0;
        int16_t internal_temp_dC = 0;  // 0.1 C units
        int16_t current_mA = 0;
    };

    explicit BQ76907(I2C_HandleTypeDef* h, uint8_t addr_8bit = DEFAULT_I2C_ADDR_8BIT)
        : I2CDevice(h, addr_8bit){};

    HAL_StatusTypeDef init();

    HAL_StatusTypeDef readCellVoltage(uint8_t cell_1_based, uint16_t& millivolts);
    HAL_StatusTypeDef readStackVoltage(uint16_t& millivolts);
    HAL_StatusTypeDef readInternalTempDeciC(int16_t& deci_c);
    HAL_StatusTypeDef readCurrentMilliAmps(int16_t& milliamps);
    HAL_StatusTypeDef readTelemetry(uint8_t cell_count, Telemetry& out);

    HAL_StatusTypeDef executeSubcommand(uint16_t subcommand);
    HAL_StatusTypeDef enterConfigUpdate();
    HAL_StatusTypeDef exitConfigUpdate();

   private:
    enum class DirectCommand : uint8_t
    {
        CELL1_VOLTAGE = 0x14,  // Cell n = CELL1_VOLTAGE + 2*(n-1)
        STACK_VOLTAGE = 0x26,
        INT_TEMPERATURE = 0x28,
        CURRENT_CC2 = 0x3A,
        SUBCOMMAND_LO = 0x3E
    };

    enum class Subcommand : uint16_t
    {
        SET_CFGUPDATE = 0x0090,
        EXIT_CFGUPDATE = 0x0092
    };

    HAL_StatusTypeDef readDirect16(uint8_t command, uint16_t& value);
    HAL_StatusTypeDef writeDirect16(uint8_t command, uint16_t value);
};

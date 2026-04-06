#pragma once

#include <array>
#include <cstdint>

/**
 * Vehicle-facing BMS CAN protocol (classic 8-byte frames).
 * Wire format only — no HAL; transport uses CanBus::sendStd / read().
 */

namespace BmsCanProtocol {

/** Classic CAN frame for encode/decode (not CAN FD). */
struct BmsCanFrame {
    uint32_t id = 0;
    bool extended = false;
    uint8_t dlc = 0;
    std::array<uint8_t, 8> data{};
};

// ========== CAN ID Allocation ==========

enum CanId : uint16_t {
    BMS_STATUS = 0x040,
    BMS_BATTERY_VOLTAGE = 0x041,
    BMS_BATTERY_TEMPERATURE = 0x042,
    BMS_BATTERY_CURRENT = 0x043,

    BMS_HEARTBEAT = 0x180,
    BMS_PACK_STATUS = 0x181,
    BMS_TEMPERATURE = 0x182,
    BMS_CELL_VOLTAGES = 0x183,

    BMS_FAULT_STATUS = 0x190,
    BMS_WARNING_STATUS = 0x191,
    BMS_STATE_CHANGE = 0x192,

    BMS_COMMAND = 0x1A0,
    BMS_CONFIG_REQUEST = 0x1A1,

    BMS_CONFIG_RESPONSE = 0x1B0,

    BMS_DETAILED_STATUS = 0x1C0,
    BMS_CELL_DETAILS = 0x1C1,
};

enum BmsStatusFaultCode : uint16_t {
    BMS_FAULT_NONE = 0x0000,
    BMS_FAULT_OVERVOLTAGE = 0x0001,
    BMS_FAULT_UNDERVOLTAGE = 0x0002,
    BMS_FAULT_CELL_IMBALANCE = 0x0004,
    BMS_FAULT_OVERTEMPERATURE = 0x0008,
    BMS_FAULT_UNDERTEMPERATURE = 0x0010,
    BMS_FAULT_BATTERY_OVERCURRENT = 0x0020,
};

struct BmsStatusMsg {
    uint16_t bms_faults;
    uint8_t contactors_state;
    uint8_t daughter_board_status;
    uint8_t reserved[4];
};

struct BatteryVoltageMsg {
    uint16_t total_voltage_x100;
    uint16_t highest_cell_mV;
    uint16_t lowest_cell_mV;
    uint8_t highest_cell_idx;
    uint8_t lowest_cell_idx;
};

struct BatteryTemperatureMsg {
    int16_t high_temp_C_x10;
    uint8_t high_temp_idx;
    int16_t avg_temp_C_x10;
    uint8_t reserved[3];
};

struct BatteryCurrentMsg {
    float current_A;
    uint8_t reserved[4];
};

struct HeartbeatMsg {
    uint8_t node_id;
    uint8_t state;
    uint8_t fault_count;
    uint8_t warning_count;
    uint16_t uptime_s;
    uint16_t sequence;
};

struct PackStatusMsg {
    uint16_t pack_voltage_mV;
    int16_t pack_current_mA;
    uint16_t soc_percent_x10;
    uint16_t soh_percent_x10;
    uint8_t contactor_state;
    uint8_t reserved;
};

struct TemperatureMsg {
    int16_t highest_temp_C_x10;
    int16_t lowest_temp_C_x10;
    int16_t avg_temp_C_x10;
    uint8_t highest_temp_idx;
    uint8_t lowest_temp_idx;
    uint8_t num_sensors;
    uint8_t reserved;
};

struct CellVoltagesMsg {
    uint16_t highest_cell_mV;
    uint16_t lowest_cell_mV;
    uint16_t avg_cell_mV;
    uint8_t highest_cell_idx;
    uint8_t lowest_cell_idx;
    uint8_t imbalance_mV;
    uint8_t reserved;
};

struct FaultStatusMsg {
    uint16_t active_faults;
    uint16_t fault_count;
    uint8_t critical_fault;
    uint8_t fault_code;
    uint16_t fault_timestamp_s;
    uint8_t reserved[2];
};

struct WarningStatusMsg {
    uint16_t active_warnings;
    uint8_t warning_count;
    uint8_t reserved;
    uint32_t reserved2;
};

struct StateChangeMsg {
    uint8_t old_state;
    uint8_t new_state;
    uint16_t state_duration_ms;
    uint32_t timestamp_ms;
};

enum CommandType : uint8_t {
    CMD_NONE = 0x00,
    CMD_CLOSE_CONTACTORS = 0x01,
    CMD_OPEN_CONTACTORS = 0x02,
    CMD_CLEAR_FAULTS = 0x03,
    CMD_EMERGENCY_SHUTDOWN = 0x04,
    CMD_REQUEST_STATUS = 0x05,
    CMD_SET_CHARGE_LIMIT = 0x10,
    CMD_SET_DISCHARGE_LIMIT = 0x11,
};

struct CommandMsg {
    uint8_t command_type;
    uint8_t reserved;
    uint16_t parameter;
    uint32_t reserved2;
};

enum ConfigRequestType : uint8_t {
    CFG_REQ_NONE = 0x00,
    CFG_REQ_VOLTAGE_LIMITS = 0x01,
    CFG_REQ_TEMP_LIMITS = 0x02,
    CFG_REQ_CURRENT_LIMITS = 0x03,
    CFG_REQ_ALL = 0xFF,
};

struct ConfigRequestMsg {
    uint8_t request_type;
    uint8_t reserved[7];
};

struct ConfigResponseMsg {
    uint8_t response_type;
    uint8_t reserved;
    uint16_t cell_overvoltage_mV;
    uint16_t cell_undervoltage_mV;
    uint16_t cell_imbalance_mV;
    uint8_t reserved2;
};

class MessageEncoder {
public:
    static BmsCanFrame encodeBmsStatus(const BmsStatusMsg& msg);
    static BmsCanFrame encodeBatteryVoltage(const BatteryVoltageMsg& msg);
    static BmsCanFrame encodeBatteryTemperature(const BatteryTemperatureMsg& msg);
    static BmsCanFrame encodeBatteryCurrent(const BatteryCurrentMsg& msg);
    static BmsCanFrame encodeHeartbeat(const HeartbeatMsg& msg);
    static BmsCanFrame encodePackStatus(const PackStatusMsg& msg);
    static BmsCanFrame encodeTemperature(const TemperatureMsg& msg);
    static BmsCanFrame encodeCellVoltages(const CellVoltagesMsg& msg);
    static BmsCanFrame encodeFaultStatus(const FaultStatusMsg& msg);
    static BmsCanFrame encodeWarningStatus(const WarningStatusMsg& msg);
    static BmsCanFrame encodeStateChange(const StateChangeMsg& msg);
    static BmsCanFrame encodeConfigResponse(const ConfigResponseMsg& msg);

private:
    static void packUint16(uint8_t* buf, uint16_t value);
    static void packInt16(uint8_t* buf, int16_t value);
    static void packUint32(uint8_t* buf, uint32_t value);
};

class MessageDecoder {
public:
    static bool decodeCommand(const BmsCanFrame& frame, CommandMsg& msg);
    static bool decodeConfigRequest(const BmsCanFrame& frame, ConfigRequestMsg& msg);
    static CanId getMessageType(uint16_t can_id);

private:
    static uint16_t unpackUint16(const uint8_t* buf);
    static int16_t unpackInt16(const uint8_t* buf);
    static uint32_t unpackUint32(const uint8_t* buf);
};

}  // namespace BmsCanProtocol

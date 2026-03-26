#pragma once

#include <cstdint>
#include <array>
#include "CanFdBus.hpp"
#include "BmsManager.hpp"

/**
 * BMS CAN Protocol Application Layer
 * 
 * This module defines the CAN message protocol for communication between
 * the Primary BMS MCU and the vehicle's main controller.
 * 
 * Protocol Design:
 * - Custom protocol optimized for BMS data
 * - Supports both classic CAN (8 bytes) and CAN FD (up to 64 bytes)
 * - Periodic status messages (heartbeat, pack status)
 * - Event-driven messages (faults, warnings)
 * - Command/response messages (control, configuration)
 */

namespace BmsCanProtocol {

// ========== CAN ID Allocation ==========
// ID-based: each CAN ID maps to a single message format.
// Vehicle specification IDs (e.g. 0x040) + Primary BMS node IDs (0x180+).

enum CanId : uint16_t {
    // Vehicle / specification IDs (ID-based, not device-based)
    BMS_STATUS           = 0x040,  // BMS Status: faults, contactors, daughter boards (From: BMS, To: Rear VCU, Telemetry, Steeringwheel)
    BMS_BATTERY_VOLTAGE  = 0x041,  // Battery Voltage: pack + high/low cell voltages (From: BMS, To: Telemetry, Steering wheel)
    BMS_BATTERY_TEMPERATURE = 0x042,  // Battery Temperature: high/avg temp (From: BMS, To: Telemetry, Steering wheel)
    BMS_BATTERY_CURRENT     = 0x043,  // Battery Current: float in A (From: BMS, To: Telemetry, Steering wheel)

    // Primary BMS node IDs (0x180 - 0x1FF)
    // Periodic Status Messages (10ms - 100ms)
    BMS_HEARTBEAT        = 0x180,  // Heartbeat/status (100ms)
    BMS_PACK_STATUS      = 0x181,  // Pack voltage, current, SOC (50ms)
    BMS_TEMPERATURE     = 0x182,  // Temperature summary (100ms)
    BMS_CELL_VOLTAGES   = 0x183,  // Cell voltage extremes (100ms)
    
    // Event-Driven Messages (on change)
    BMS_FAULT_STATUS    = 0x190,  // Active faults (on change)
    BMS_WARNING_STATUS   = 0x191,  // Warnings (on change)
    BMS_STATE_CHANGE     = 0x192,  // State machine transitions
    
    // Command Messages (from vehicle to BMS)
    BMS_COMMAND          = 0x1A0,  // Control commands
    BMS_CONFIG_REQUEST   = 0x1A1,  // Configuration read request
    
    // Response Messages (to vehicle commands)
    BMS_CONFIG_RESPONSE  = 0x1B0,  // Configuration data response
    
    // Extended Data (CAN FD, optional)
    BMS_DETAILED_STATUS  = 0x1C0,  // Extended status (CAN FD, 32 bytes)
    BMS_CELL_DETAILS     = 0x1C1,  // Per-cell data (CAN FD, 64 bytes)
};

// ========== Message Types ==========

// BMS_STATUS (0x040) - 8 bytes
// From: BMS, To: Rear VCU, Telemetry, Steeringwheel
// Spec fault codes: NONE=0, OVERVOLTAGE=0x0001, UNDERVOLTAGE=0x0002, CELL_IMBALANCE=0x0004,
//   OVERTEMPERATURE=0x0008, UNDERTEMPERATURE=0x0010, BATTERY_OVERCURRENT=0x0020
enum BmsStatusFaultCode : uint16_t {
    BMS_FAULT_NONE              = 0x0000,
    BMS_FAULT_OVERVOLTAGE       = 0x0001,
    BMS_FAULT_UNDERVOLTAGE     = 0x0002,
    BMS_FAULT_CELL_IMBALANCE   = 0x0004,
    BMS_FAULT_OVERTEMPERATURE  = 0x0008,
    BMS_FAULT_UNDERTEMPERATURE = 0x0010,
    BMS_FAULT_BATTERY_OVERCURRENT = 0x0020,
};
struct BmsStatusMsg {
    uint16_t bms_faults;           // BMS Faults (Bytes 0-1, MSB first)
    uint8_t  contactors_state;     // 0=open, 1=closed
    uint8_t  daughter_board_status; // Bitmap: bit N = module N online (from Secondary)
    uint8_t  reserved[4];          // Bytes 4-7 reserved
};

// Battery Voltage (0x041) - 8 bytes
// From: BMS, To: Telemetry, Steering wheel
// Byte 0-1: Total pack voltage (UInt16, voltage * 100, MSB first)
// Byte 2-3: Highest cell voltage (UInt16, mV, MSB first)
// Byte 4:   Highest cell index
// Byte 5-6: Lowest cell voltage (UInt16, mV, MSB first)
// Byte 7:   Lowest cell index
struct BatteryVoltageMsg {
    uint16_t total_voltage_x100;   // Pack voltage * 100 (e.g. 90.67V -> 9067)
    uint16_t highest_cell_mV;      // Highest cell voltage in mV
    uint16_t lowest_cell_mV;       // Lowest cell voltage in mV
    uint8_t  highest_cell_idx;     // Index of highest cell
    uint8_t  lowest_cell_idx;      // Index of lowest cell
};

// Battery Temperature (0x042) - 8 bytes
// From: BMS, To: Telemetry, Steering wheel
// Byte 0-1: High Temp (UInt16, temp*10, MSB first, e.g. 456 = 45.6°C)
// Byte 2:   High Temp Index (which module)
// Byte 3-4: Avg Temp (UInt16, temp*10, MSB first)
// Byte 5-7: Reserved
struct BatteryTemperatureMsg {
    int16_t  high_temp_C_x10;      // Highest temperature (°C * 10)
    uint8_t  high_temp_idx;        // Module index with highest temp
    int16_t  avg_temp_C_x10;       // Average temperature (°C * 10)
    uint8_t  reserved[3];         // Bytes 5-7 reserved
};

// Battery Current (0x043) - 8 bytes
// From: BMS, To: Telemetry, Steering wheel
// Byte 0-3: Battery current (float, IEEE 754, big-endian MSB first)
// Byte 4-7: Reserved
struct BatteryCurrentMsg {
    float current_A;               // Battery current in amperes (positive = discharge)
    uint8_t reserved[4];          // Bytes 4-7 reserved
};

// BMS_HEARTBEAT (0x180) - 8 bytes
struct HeartbeatMsg {
    uint8_t  node_id;           // BMS node ID (0x01)
    uint8_t  state;              // BmsManager::BmsState
    uint8_t  fault_count;        // Number of active faults
    uint8_t  warning_count;      // Number of active warnings
    uint16_t uptime_s;           // System uptime in seconds
    uint16_t sequence;           // Sequence counter
};

// BMS_PACK_STATUS (0x181) - 8 bytes
struct PackStatusMsg {
    uint16_t pack_voltage_mV;    // Pack voltage in mV
    int16_t  pack_current_mA;     // Pack current in mA (signed, discharge positive)
    uint16_t soc_percent_x10;     // State of charge (0-1000 = 0-100.0%)
    uint16_t soh_percent_x10;     // State of health (0-1000 = 0-100.0%)
    uint8_t  contactor_state;     // 0=open, 1=closed
    uint8_t  reserved;
};

// BMS_TEMPERATURE (0x182) - 8 bytes
struct TemperatureMsg {
    int16_t  highest_temp_C_x10;  // Highest temperature (°C * 10)
    int16_t  lowest_temp_C_x10;   // Lowest temperature (°C * 10)
    int16_t  avg_temp_C_x10;      // Average temperature (°C * 10)
    uint8_t  highest_temp_idx;     // Cell/module index with highest temp
    uint8_t  lowest_temp_idx;      // Cell/module index with lowest temp
    uint8_t  num_sensors;         // Number of temperature sensors
    uint8_t  reserved;
};

// BMS_CELL_VOLTAGES (0x183) - 8 bytes
struct CellVoltagesMsg {
    uint16_t highest_cell_mV;     // Highest cell voltage
    uint16_t lowest_cell_mV;      // Lowest cell voltage
    uint16_t avg_cell_mV;         // Average cell voltage
    uint8_t  highest_cell_idx;    // Cell index with highest voltage
    uint8_t  lowest_cell_idx;     // Cell index with lowest voltage
    uint8_t  imbalance_mV;        // Voltage imbalance (max - min)
    uint8_t  reserved;
};

// BMS_FAULT_STATUS (0x190) - 8 bytes
struct FaultStatusMsg {
    uint16_t active_faults;       // Bitfield of active faults (BmsManager::FaultType)
    uint16_t fault_count;         // Total number of faults (including latched)
    uint8_t  critical_fault;      // 1 if critical fault present
    uint8_t  fault_code;          // Most significant fault code
    uint16_t fault_timestamp_s;   // Timestamp when fault occurred (seconds since boot)
    uint8_t  reserved[2];
};

// BMS_WARNING_STATUS (0x191) - 8 bytes
struct WarningStatusMsg {
    uint16_t active_warnings;     // Bitfield of active warnings
    uint8_t  warning_count;       // Number of active warnings
    uint8_t  reserved;
    uint32_t reserved2;
};

// BMS_STATE_CHANGE (0x192) - 8 bytes
struct StateChangeMsg {
    uint8_t  old_state;            // Previous state
    uint8_t  new_state;            // New state (BmsManager::BmsState)
    uint16_t state_duration_ms;   // Duration in previous state (ms)
    uint32_t timestamp_ms;         // Timestamp of state change
};

// BMS_COMMAND (0x1A0) - 8 bytes
enum CommandType : uint8_t {
    CMD_NONE              = 0x00,
    CMD_CLOSE_CONTACTORS  = 0x01,
    CMD_OPEN_CONTACTORS   = 0x02,
    CMD_CLEAR_FAULTS      = 0x03,
    CMD_EMERGENCY_SHUTDOWN = 0x04,
    CMD_REQUEST_STATUS    = 0x05,
    CMD_SET_CHARGE_LIMIT  = 0x10,  // Set charge current limit
    CMD_SET_DISCHARGE_LIMIT = 0x11, // Set discharge current limit
};

struct CommandMsg {
    uint8_t  command_type;        // CommandType enum
    uint8_t  reserved;
    uint16_t parameter;            // Command-specific parameter
    uint32_t reserved2;
};

// BMS_CONFIG_REQUEST (0x1A1) - 8 bytes
enum ConfigRequestType : uint8_t {
    CFG_REQ_NONE          = 0x00,
    CFG_REQ_VOLTAGE_LIMITS = 0x01,
    CFG_REQ_TEMP_LIMITS   = 0x02,
    CFG_REQ_CURRENT_LIMITS = 0x03,
    CFG_REQ_ALL           = 0xFF,
};

struct ConfigRequestMsg {
    uint8_t  request_type;        // ConfigRequestType
    uint8_t  reserved[7];
};

// BMS_CONFIG_RESPONSE (0x1B0) - 8 bytes (or CAN FD for extended)
struct ConfigResponseMsg {
    uint8_t  response_type;       // ConfigRequestType
    uint8_t  reserved;
    // Voltage limits (mV)
    uint16_t cell_overvoltage_mV;
    uint16_t cell_undervoltage_mV;
    uint16_t cell_imbalance_mV;
    uint8_t  reserved2;
};

// ========== Message Encoder/Decoder ==========

class MessageEncoder {
public:
    // Encode messages to CAN frame (ID-based)
    static CanFdFrame encodeBmsStatus(const BmsStatusMsg& msg);
    static CanFdFrame encodeBatteryVoltage(const BatteryVoltageMsg& msg);
    static CanFdFrame encodeBatteryTemperature(const BatteryTemperatureMsg& msg);
    static CanFdFrame encodeBatteryCurrent(const BatteryCurrentMsg& msg);
    static CanFdFrame encodeHeartbeat(const HeartbeatMsg& msg);
    static CanFdFrame encodePackStatus(const PackStatusMsg& msg);
    static CanFdFrame encodeTemperature(const TemperatureMsg& msg);
    static CanFdFrame encodeCellVoltages(const CellVoltagesMsg& msg);
    static CanFdFrame encodeFaultStatus(const FaultStatusMsg& msg);
    static CanFdFrame encodeWarningStatus(const WarningStatusMsg& msg);
    static CanFdFrame encodeStateChange(const StateChangeMsg& msg);
    static CanFdFrame encodeConfigResponse(const ConfigResponseMsg& msg);
    
private:
    static void packUint16(uint8_t* buf, uint16_t value);
    static void packInt16(uint8_t* buf, int16_t value);
    static void packUint32(uint8_t* buf, uint32_t value);
};

class MessageDecoder {
public:
    // Decode CAN frame to message structures
    static bool decodeCommand(const CanFdFrame& frame, CommandMsg& msg);
    static bool decodeConfigRequest(const CanFdFrame& frame, ConfigRequestMsg& msg);
    
    // Helper to identify message type from CAN ID
    static CanId getMessageType(uint16_t can_id);
    
private:
    static uint16_t unpackUint16(const uint8_t* buf);
    static int16_t unpackInt16(const uint8_t* buf);
    static uint32_t unpackUint32(const uint8_t* buf);
};

} // namespace BmsCanProtocol

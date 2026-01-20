#include "BmsCanProtocol.hpp"
#include <cstring>

namespace BmsCanProtocol {

// ========== Message Encoder ==========

CanFdFrame MessageEncoder::encodeHeartbeat(const HeartbeatMsg& msg)
{
    CanFdFrame frame{};
    frame.extended = false;
    frame.id = BMS_HEARTBEAT;
    frame.fd_frame = false;  // Classic CAN
    frame.dlc = 8;
    
    frame.data[0] = msg.node_id;
    frame.data[1] = msg.state;
    frame.data[2] = msg.fault_count;
    frame.data[3] = msg.warning_count;
    packUint16(&frame.data[4], msg.uptime_s);
    packUint16(&frame.data[6], msg.sequence);
    
    return frame;
}

CanFdFrame MessageEncoder::encodePackStatus(const PackStatusMsg& msg)
{
    CanFdFrame frame{};
    frame.extended = false;
    frame.id = BMS_PACK_STATUS;
    frame.fd_frame = false;  // Classic CAN
    frame.dlc = 8;
    
    packUint16(&frame.data[0], msg.pack_voltage_mV);
    packInt16(&frame.data[2], msg.pack_current_mA);
    packUint16(&frame.data[4], msg.soc_percent_x10);
    packUint16(&frame.data[6], msg.soh_percent_x10);
    
    return frame;
}

CanFdFrame MessageEncoder::encodeTemperature(const TemperatureMsg& msg)
{
    CanFdFrame frame{};
    frame.extended = false;
    frame.id = BMS_TEMPERATURE;
    frame.fd_frame = false;  // Classic CAN
    frame.dlc = 8;
    
    packInt16(&frame.data[0], msg.highest_temp_C_x10);
    packInt16(&frame.data[2], msg.lowest_temp_C_x10);
    packInt16(&frame.data[4], msg.avg_temp_C_x10);
    frame.data[6] = msg.highest_temp_idx;
    frame.data[7] = msg.lowest_temp_idx;
    
    return frame;
}

CanFdFrame MessageEncoder::encodeCellVoltages(const CellVoltagesMsg& msg)
{
    CanFdFrame frame{};
    frame.extended = false;
    frame.id = BMS_CELL_VOLTAGES;
    frame.fd_frame = false;  // Classic CAN
    frame.dlc = 8;
    
    packUint16(&frame.data[0], msg.highest_cell_mV);
    packUint16(&frame.data[2], msg.lowest_cell_mV);
    packUint16(&frame.data[4], msg.avg_cell_mV);
    frame.data[6] = msg.highest_cell_idx;
    frame.data[7] = msg.lowest_cell_idx;
    
    return frame;
}

CanFdFrame MessageEncoder::encodeFaultStatus(const FaultStatusMsg& msg)
{
    CanFdFrame frame{};
    frame.extended = false;
    frame.id = BMS_FAULT_STATUS;
    frame.fd_frame = false;  // Classic CAN
    frame.dlc = 8;
    
    packUint16(&frame.data[0], msg.active_faults);
    packUint16(&frame.data[2], msg.fault_count);
    frame.data[4] = msg.critical_fault;
    frame.data[5] = msg.fault_code;
    packUint16(&frame.data[6], msg.fault_timestamp_s);
    
    return frame;
}

// Helper functions
void MessageEncoder::packUint16(uint8_t* buf, uint16_t value)
{
    buf[0] = static_cast<uint8_t>(value & 0xFF);
    buf[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

void MessageEncoder::packInt16(uint8_t* buf, int16_t value)
{
    packUint16(buf, static_cast<uint16_t>(value));
}

void MessageEncoder::packUint32(uint8_t* buf, uint32_t value)
{
    buf[0] = static_cast<uint8_t>(value & 0xFF);
    buf[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    buf[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
    buf[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

// ========== Message Decoder ==========

bool MessageDecoder::decodeCommand(const CanFdFrame& frame, CommandMsg& msg)
{
    if (frame.id != BMS_COMMAND || frame.dlc < 1) {
        return false;
    }
    
    msg.command_type = frame.data[0];
    msg.parameter = unpackUint16(&frame.data[2]);
    
    return true;
}

bool MessageDecoder::decodeConfigRequest(const CanFdFrame& frame, ConfigRequestMsg& msg)
{
    if (frame.id != BMS_CONFIG_REQUEST || frame.dlc < 1) {
        return false;
    }
    
    msg.request_type = frame.data[0];
    
    return true;
}

CanId MessageDecoder::getMessageType(uint16_t can_id)
{
    if (can_id >= BMS_HEARTBEAT && can_id <= BMS_CELL_VOLTAGES) {
        return static_cast<CanId>(can_id);
    }
    if (can_id >= BMS_FAULT_STATUS && can_id <= BMS_STATE_CHANGE) {
        return static_cast<CanId>(can_id);
    }
    if (can_id == BMS_COMMAND) return BMS_COMMAND;
    if (can_id == BMS_CONFIG_REQUEST) return BMS_CONFIG_REQUEST;
    if (can_id == BMS_CONFIG_RESPONSE) return BMS_CONFIG_RESPONSE;
    
    return BMS_HEARTBEAT;  // Default
}

uint16_t MessageDecoder::unpackUint16(const uint8_t* buf)
{
    return static_cast<uint16_t>(buf[0]) | 
           (static_cast<uint16_t>(buf[1]) << 8);
}

int16_t MessageDecoder::unpackInt16(const uint8_t* buf)
{
    return static_cast<int16_t>(unpackUint16(buf));
}

uint32_t MessageDecoder::unpackUint32(const uint8_t* buf)
{
    return static_cast<uint32_t>(buf[0]) |
           (static_cast<uint32_t>(buf[1]) << 8) |
           (static_cast<uint32_t>(buf[2]) << 16) |
           (static_cast<uint32_t>(buf[3]) << 24);
}

} // namespace BmsCanProtocol

#include "BmsCanInterface.hpp"
#include "BmsCanProtocol.hpp"
#include <cstring>

BmsCanInterface::BmsCanInterface(CanFdBus& can_bus, BmsManager& bms_manager)
    : can_bus_(can_bus)
    , bms_manager_(bms_manager)
    , config_{}
    , last_heartbeat_ms_(0)
    , last_pack_status_ms_(0)
    , system_start_ms_(0)
    , sequence_counter_(0)
{
}

void BmsCanInterface::init(const Config& config)
{
    config_ = config;
    system_start_ms_ = HAL_GetTick();
    sequence_counter_ = 0;
    last_heartbeat_ms_ = 0;
    last_pack_status_ms_ = 0;
}

void BmsCanInterface::update(uint32_t now_ms)
{
    // Process any received messages
    processReceivedMessages(now_ms);
    
    // Send periodic messages
    updatePeriodicTransmission(now_ms);
}

void BmsCanInterface::sendHeartbeat()
{
    BmsCanProtocol::HeartbeatMsg msg = createHeartbeatMsg(HAL_GetTick());
    CanFdFrame frame = BmsCanProtocol::MessageEncoder::encodeHeartbeat(msg);
    
    CanFdBus::Result result = can_bus_.sendStd(frame.id, frame.data.data(), frame.dlc);
    if (result == CanFdBus::Result::Ok) {
        tx_ok_count_++;
    } else {
        tx_error_count_++;
    }
}

void BmsCanInterface::sendPackStatus()
{
    BmsCanProtocol::PackStatusMsg msg = createPackStatusMsg();
    CanFdFrame frame = BmsCanProtocol::MessageEncoder::encodePackStatus(msg);
    
    CanFdBus::Result result = can_bus_.sendStd(frame.id, frame.data.data(), frame.dlc);
    if (result == CanFdBus::Result::Ok) {
        tx_ok_count_++;
    } else {
        tx_error_count_++;
    }
}

void BmsCanInterface::sendTemperature()
{
    BmsCanProtocol::TemperatureMsg msg = createTemperatureMsg();
    CanFdFrame frame = BmsCanProtocol::MessageEncoder::encodeTemperature(msg);
    
    CanFdBus::Result result = can_bus_.sendStd(frame.id, frame.data.data(), frame.dlc);
    if (result == CanFdBus::Result::Ok) {
        tx_ok_count_++;
    } else {
        tx_error_count_++;
    }
}

void BmsCanInterface::sendCellVoltages()
{
    BmsCanProtocol::CellVoltagesMsg msg = createCellVoltagesMsg();
    CanFdFrame frame = BmsCanProtocol::MessageEncoder::encodeCellVoltages(msg);
    
    CanFdBus::Result result = can_bus_.sendStd(frame.id, frame.data.data(), frame.dlc);
    if (result == CanFdBus::Result::Ok) {
        tx_ok_count_++;
    } else {
        tx_error_count_++;
    }
}

void BmsCanInterface::sendFaultStatus()
{
    BmsCanProtocol::FaultStatusMsg msg = createFaultStatusMsg();
    CanFdFrame frame = BmsCanProtocol::MessageEncoder::encodeFaultStatus(msg);
    
    CanFdBus::Result result = can_bus_.sendStd(frame.id, frame.data.data(), frame.dlc);
    if (result == CanFdBus::Result::Ok) {
        tx_ok_count_++;
    } else {
        tx_error_count_++;
    }
}

void BmsCanInterface::setConfig(const Config& config)
{
    config_ = config;
}

const BmsCanInterface::Config& BmsCanInterface::getConfig() const
{
    return config_;
}

// ========== Private Methods ==========

void BmsCanInterface::updatePeriodicTransmission(uint32_t now_ms)
{
    // Send heartbeat
    if ((now_ms - last_heartbeat_ms_) >= config_.heartbeat_period_ms) {
        sendHeartbeat();
        last_heartbeat_ms_ = now_ms;
    }
    
    // Send pack status
    if ((now_ms - last_pack_status_ms_) >= config_.pack_status_period_ms) {
        sendPackStatus();
        last_pack_status_ms_ = now_ms;
    }
}

void BmsCanInterface::processReceivedMessages(uint32_t now_ms)
{
    (void)now_ms;  // Unused for now
    
    // Process any received CAN messages
    CanFdFrame frame;
    while (can_bus_.read(frame)) {
        // Check if it's a command message
        if (frame.id == BmsCanProtocol::BMS_COMMAND) {
            BmsCanProtocol::CommandMsg cmd;
            if (BmsCanProtocol::MessageDecoder::decodeCommand(frame, cmd)) {
                handleCommand(cmd);
                rx_command_count_++;
            }
        }
    }
}

void BmsCanInterface::handleCommand(const BmsCanProtocol::CommandMsg& cmd)
{
    switch (cmd.command_type) {
        case BmsCanProtocol::CMD_CLOSE_CONTACTORS:
            bms_manager_.requestContactorsClose();
            break;
            
        case BmsCanProtocol::CMD_OPEN_CONTACTORS:
            bms_manager_.requestContactorsOpen();
            break;
            
        case BmsCanProtocol::CMD_CLEAR_FAULTS:
            bms_manager_.clearFaults();
            break;
            
        case BmsCanProtocol::CMD_EMERGENCY_SHUTDOWN:
            bms_manager_.requestShutdown();
            break;
            
        default:
            // Unknown command - ignore
            break;
    }
}

BmsCanProtocol::HeartbeatMsg BmsCanInterface::createHeartbeatMsg(uint32_t now_ms)
{
    BmsCanProtocol::HeartbeatMsg msg{};
    msg.node_id = config_.node_id;
    msg.state = static_cast<uint8_t>(bms_manager_.getState());
    
    uint16_t faults = bms_manager_.getActiveFaults();
    msg.fault_count = 0;
    for (int i = 0; i < 16; i++) {
        if (faults & (1 << i)) {
            msg.fault_count++;
        }
    }
    
    msg.warning_count = 0;  // TODO: implement warnings
    msg.uptime_s = (now_ms - system_start_ms_) / 1000;
    msg.sequence = sequence_counter_++;
    
    return msg;
}

BmsCanProtocol::PackStatusMsg BmsCanInterface::createPackStatusMsg()
{
    BmsCanProtocol::PackStatusMsg msg{};
    
    // Get pack voltage from BMS manager
    float pack_voltage_V = bms_manager_.getPackVoltage_V();
    msg.pack_voltage_mV = static_cast<uint16_t>(pack_voltage_V * 1000.0f);
    
    // Get pack current from BMS manager
    float pack_current_A = bms_manager_.getBatteryCurrent_A();
    msg.pack_current_mA = static_cast<int16_t>(pack_current_A * 1000.0f);
    
    // SOC/SOH - placeholder for now
    msg.soc_percent_x10 = 0;  // TODO: implement SOC calculation
    msg.soh_percent_x10 = 1000;  // Assume 100% health for now
    
    // Contactor state
    msg.contactor_state = bms_manager_.areContactorsClosed() ? 1 : 0;
    
    return msg;
}

BmsCanProtocol::TemperatureMsg BmsCanInterface::createTemperatureMsg()
{
    BmsCanProtocol::TemperatureMsg msg{};
    
    const auto& summary = bms_manager_.getFleetSummary();
    
    msg.highest_temp_C_x10 = static_cast<int16_t>(summary.highest_temp_C * 10.0f);
    msg.lowest_temp_C_x10 = static_cast<int16_t>(summary.lowest_temp_C * 10.0f);
    
    // Calculate average (simplified - just use highest/lowest average)
    float avg_temp = (summary.highest_temp_C + summary.lowest_temp_C) / 2.0f;
    msg.avg_temp_C_x10 = static_cast<int16_t>(avg_temp * 10.0f);
    
    msg.highest_temp_idx = summary.highest_temp_idx;
    msg.lowest_temp_idx = summary.lowest_temp_idx;
    msg.num_sensors = summary.num_modules;  // Approximate
    
    return msg;
}

BmsCanProtocol::CellVoltagesMsg BmsCanInterface::createCellVoltagesMsg()
{
    BmsCanProtocol::CellVoltagesMsg msg{};
    
    const auto& summary = bms_manager_.getFleetSummary();
    
    msg.highest_cell_mV = summary.highest_cell_mV;
    msg.lowest_cell_mV = summary.lowest_cell_mV;
    
    // Calculate average cell voltage
    uint16_t avg_cell_mV = summary.total_voltage_mV / summary.num_modules;
    msg.avg_cell_mV = avg_cell_mV;
    
    msg.highest_cell_idx = summary.highest_cell_idx;
    msg.lowest_cell_idx = summary.lowest_cell_idx;
    msg.imbalance_mV = static_cast<uint8_t>(summary.highest_cell_mV - summary.lowest_cell_mV);
    
    return msg;
}

BmsCanProtocol::FaultStatusMsg BmsCanInterface::createFaultStatusMsg()
{
    BmsCanProtocol::FaultStatusMsg msg{};
    
    uint16_t faults = bms_manager_.getActiveFaults();
    msg.active_faults = faults;
    
    // Count number of faults
    msg.fault_count = 0;
    for (int i = 0; i < 16; i++) {
        if (faults & (1 << i)) {
            msg.fault_count++;
        }
    }
    
    msg.critical_fault = bms_manager_.hasCriticalFault() ? 1 : 0;
    
    // Find most significant fault code
    msg.fault_code = 0;
    for (int i = 15; i >= 0; i--) {
        if (faults & (1 << i)) {
            msg.fault_code = static_cast<uint8_t>(i);
            break;
        }
    }
    
    uint32_t now_ms = HAL_GetTick();
    msg.fault_timestamp_s = (now_ms - system_start_ms_) / 1000;
    
    return msg;
}

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
    last_bms_status_ms_ = 0;
    last_battery_voltage_ms_ = 0;
    last_battery_temperature_ms_ = 0;
    last_battery_current_ms_ = 0;
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

void BmsCanInterface::sendBmsStatus()
{
    BmsCanProtocol::BmsStatusMsg msg = createBmsStatusMsg(HAL_GetTick());
    CanFdFrame frame = BmsCanProtocol::MessageEncoder::encodeBmsStatus(msg);
    
    CanFdBus::Result result = can_bus_.sendStd(frame.id, frame.data.data(), frame.dlc);
    if (result == CanFdBus::Result::Ok) {
        tx_ok_count_++;
    } else {
        tx_error_count_++;
    }
}

void BmsCanInterface::sendBatteryVoltage()
{
    BmsCanProtocol::BatteryVoltageMsg msg = createBatteryVoltageMsg();
    CanFdFrame frame = BmsCanProtocol::MessageEncoder::encodeBatteryVoltage(msg);
    
    CanFdBus::Result result = can_bus_.sendStd(frame.id, frame.data.data(), frame.dlc);
    if (result == CanFdBus::Result::Ok) {
        tx_ok_count_++;
    } else {
        tx_error_count_++;
    }
}

void BmsCanInterface::sendBatteryTemperature()
{
    BmsCanProtocol::BatteryTemperatureMsg msg = createBatteryTemperatureMsg();
    CanFdFrame frame = BmsCanProtocol::MessageEncoder::encodeBatteryTemperature(msg);
    
    CanFdBus::Result result = can_bus_.sendStd(frame.id, frame.data.data(), frame.dlc);
    if (result == CanFdBus::Result::Ok) {
        tx_ok_count_++;
    } else {
        tx_error_count_++;
    }
}

void BmsCanInterface::sendBatteryCurrent()
{
    BmsCanProtocol::BatteryCurrentMsg msg = createBatteryCurrentMsg();
    CanFdFrame frame = BmsCanProtocol::MessageEncoder::encodeBatteryCurrent(msg);
    
    CanFdBus::Result result = can_bus_.sendStd(frame.id, frame.data.data(), frame.dlc);
    if (result == CanFdBus::Result::Ok) {
        tx_ok_count_++;
    } else {
        tx_error_count_++;
    }
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
    // Send BMS Status (0x040) - spec message to Rear VCU, Telemetry, Steeringwheel
    if ((now_ms - last_bms_status_ms_) >= config_.bms_status_period_ms) {
        sendBmsStatus();
        last_bms_status_ms_ = now_ms;
    }
    // Send Battery Voltage (0x041) - pack + high/low cell voltages
    if ((now_ms - last_battery_voltage_ms_) >= config_.battery_voltage_period_ms) {
        sendBatteryVoltage();
        last_battery_voltage_ms_ = now_ms;
    }
    // Send Battery Temperature (0x042) - high temp, high temp index, avg temp
    if ((now_ms - last_battery_temperature_ms_) >= config_.battery_temperature_period_ms) {
        sendBatteryTemperature();
        last_battery_temperature_ms_ = now_ms;
    }
    // Send Battery Current (0x043) - float current in A (bytes 0-3, MSB first)
    if ((now_ms - last_battery_current_ms_) >= config_.battery_current_period_ms) {
        sendBatteryCurrent();
        last_battery_current_ms_ = now_ms;
    }
}

void BmsCanInterface::processReceivedMessages(uint32_t now_ms)
{
    (void)now_ms;  // Unused for now
    
    // Process any received CAN messages
    CanFdFrame frame;
    while (can_bus_.read(frame)) {
        // Update debug snapshot for inspection via debugger
        rx_debug_.last_id = frame.id;
        rx_debug_.last_dlc = frame.dlc;
        for (uint8_t i = 0; i < frame.dlc && i < 8; ++i) {
            rx_debug_.last_data[i] = frame.data[i];
        }
        rx_debug_.rx_count++;

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

BmsCanProtocol::BmsStatusMsg BmsCanInterface::createBmsStatusMsg(uint32_t now_ms)
{
    BmsCanProtocol::BmsStatusMsg msg{};
    // Map BmsManager faults to spec fault codes (mask to 0x003F - spec defines 6 faults)
    uint16_t faults = bms_manager_.getActiveFaults() & 0x003Fu;
    msg.bms_faults = faults;
    msg.contactors_state = bms_manager_.areContactorsClosed() ? 1u : 0u;
    msg.daughter_board_status = bms_manager_.getDaughterBoardStatusBitmap(now_ms);
    msg.reserved[0] = 0;
    msg.reserved[1] = 0;
    msg.reserved[2] = 0;
    msg.reserved[3] = 0;
    return msg;
}

BmsCanProtocol::BatteryVoltageMsg BmsCanInterface::createBatteryVoltageMsg()
{
    BmsCanProtocol::BatteryVoltageMsg msg{};

    // Total pack voltage: use BmsManager pack voltage (V) -> scale by 100
    float pack_voltage_V = bms_manager_.getPackVoltage_V();
    if (pack_voltage_V < 0.0f) {
        pack_voltage_V = 0.0f;
    }
    msg.total_voltage_x100 = static_cast<uint16_t>(pack_voltage_V * 100.0f);

    // High/low cell voltages and indices from fleet summary
    const auto& summary = bms_manager_.getFleetSummary();
    msg.highest_cell_mV  = summary.highest_cell_mV;
    msg.lowest_cell_mV   = summary.lowest_cell_mV;
    msg.highest_cell_idx = summary.highest_cell_idx;
    msg.lowest_cell_idx  = summary.lowest_cell_idx;

    return msg;
}

BmsCanProtocol::BatteryTemperatureMsg BmsCanInterface::createBatteryTemperatureMsg()
{
    BmsCanProtocol::BatteryTemperatureMsg msg{};

    const auto& summary = bms_manager_.getFleetSummary();

    // High temp (temp*10, e.g. 45.6°C -> 456)
    msg.high_temp_C_x10 = static_cast<int16_t>(summary.highest_temp_C * 10.0f);
    msg.high_temp_idx   = summary.highest_temp_idx;

    // Avg temp: FleetSummaryData has only highest; use it as placeholder until avg is available
    msg.avg_temp_C_x10 = msg.high_temp_C_x10;

    return msg;
}

BmsCanProtocol::BatteryCurrentMsg BmsCanInterface::createBatteryCurrentMsg()
{
    BmsCanProtocol::BatteryCurrentMsg msg{};
    msg.current_A = bms_manager_.getBatteryCurrent_A();
    return msg;
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
    
    // lowest_temp_C not available in FleetSummaryData - use highest_temp_C as placeholder
    // TODO: Add lowest_temp_C to FleetSummaryData if needed
    msg.lowest_temp_C_x10 = msg.highest_temp_C_x10;
    
    // Average is same as highest since we only have one temperature value
    msg.avg_temp_C_x10 = msg.highest_temp_C_x10;
    
    msg.highest_temp_idx = summary.highest_temp_idx;
    msg.lowest_temp_idx = summary.highest_temp_idx;  // Same as highest since we only have one temp
    msg.num_sensors = 1;  // Only have highest temp data available
    
    return msg;
}

BmsCanProtocol::CellVoltagesMsg BmsCanInterface::createCellVoltagesMsg()
{
    BmsCanProtocol::CellVoltagesMsg msg{};
    
    const auto& summary = bms_manager_.getFleetSummary();
    
    msg.highest_cell_mV = summary.highest_cell_mV;
    msg.lowest_cell_mV = summary.lowest_cell_mV;
    
    // Calculate average cell voltage as midpoint between highest and lowest
    // (num_modules not available in FleetSummaryData)
    msg.avg_cell_mV = (summary.highest_cell_mV + summary.lowest_cell_mV) / 2;
    
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

#include "BmsCanInterface.hpp"

#include "FleetSummary.hpp"

#include <algorithm>
#include <cstdint>

namespace {

CanFdFrame frameFromCanBus(const CanBus::Frame& f) {
    CanFdFrame wire{};
    wire.extended = f.extended;
    wire.id = f.extended ? (f.id & 0x1FFFFFFFu) : (f.id & 0x7FFu);
    wire.rtr = f.rtr;
    wire.fd_frame = f.fd;
    const uint8_t n = static_cast<uint8_t>(std::min<uint8_t>(static_cast<uint8_t>(f.len), 8));
    wire.dlc = n;
    for (uint8_t i = 0; i < n; ++i) {
        wire.data[i] = f.data[i];
    }
    return wire;
}

}  // namespace

BmsCanInterface::BmsCanInterface(CanBus& can_bus, BmsManager& bms_manager)
    : can_bus_(can_bus)
    , bms_manager_(bms_manager) {}

void BmsCanInterface::init(const Config& config) {
    config_ = config;
    system_start_ms_ = HAL_GetTick();
    sequence_counter_ = 0;
    last_bms_status_ms_ = 0;
    last_battery_voltage_ms_ = 0;
    last_battery_temperature_ms_ = 0;
    last_battery_current_ms_ = 0;
    last_tx_ms_ = 0;
    last_faults_ = bms_manager_.getActiveFaults();
    last_state_ = bms_manager_.getState();
    state_entry_time_ms_ = system_start_ms_;
}

void BmsCanInterface::update(uint32_t now_ms) {
    processReceivedMessages(now_ms);
    updatePeriodicTransmission(now_ms);
    updateEventDrivenTransmission(now_ms);
}

void BmsCanInterface::sendWire(const CanFdFrame& fr) {
    const uint16_t sid = static_cast<uint16_t>(fr.id & 0x7FFu);
    const CanBus::Result r = can_bus_.sendStd(sid, fr.data.data(), fr.dlc, false);
    if (r == CanBus::Result::Ok) {
        ++tx_ok_count_;
        last_tx_ms_ = HAL_GetTick();
    } else {
        ++tx_error_count_;
    }
}

bool BmsCanInterface::canTransmit(uint32_t now_ms) {
    return (now_ms - last_tx_ms_) >= config_.min_message_interval_ms;
}

void BmsCanInterface::sendBmsStatus() {
    sendWire(BmsCanProtocol::MessageEncoder::encodeBmsStatus(createBmsStatusMsg(HAL_GetTick())));
}

void BmsCanInterface::sendBatteryVoltage() {
    sendWire(BmsCanProtocol::MessageEncoder::encodeBatteryVoltage(createBatteryVoltageMsg()));
}

void BmsCanInterface::sendBatteryTemperature() {
    sendWire(BmsCanProtocol::MessageEncoder::encodeBatteryTemperature(createBatteryTemperatureMsg()));
}

void BmsCanInterface::sendBatteryCurrent() {
    sendWire(BmsCanProtocol::MessageEncoder::encodeBatteryCurrent(createBatteryCurrentMsg()));
}

void BmsCanInterface::sendHeartbeat() {
    sendWire(BmsCanProtocol::MessageEncoder::encodeHeartbeat(createHeartbeatMsg(HAL_GetTick())));
}

void BmsCanInterface::sendPackStatus() {
    sendWire(BmsCanProtocol::MessageEncoder::encodePackStatus(createPackStatusMsg()));
}

void BmsCanInterface::sendTemperature() {
    sendWire(BmsCanProtocol::MessageEncoder::encodeTemperature(createTemperatureMsg()));
}

void BmsCanInterface::sendCellVoltages() {
    sendWire(BmsCanProtocol::MessageEncoder::encodeCellVoltages(createCellVoltagesMsg()));
}

void BmsCanInterface::sendFaultStatus() {
    sendWire(BmsCanProtocol::MessageEncoder::encodeFaultStatus(createFaultStatusMsg()));
}

void BmsCanInterface::sendStateChange(BmsManager::BmsState old_state, BmsManager::BmsState new_state) {
    const uint32_t now = HAL_GetTick();
    const uint32_t duration = now - state_entry_time_ms_;
    sendWire(BmsCanProtocol::MessageEncoder::encodeStateChange(
        createStateChangeMsg(old_state, new_state, duration)));
}

void BmsCanInterface::setConfig(const Config& config) {
    config_ = config;
}

const BmsCanInterface::Config& BmsCanInterface::getConfig() const {
    return config_;
}

void BmsCanInterface::updatePeriodicTransmission(uint32_t now_ms) {
    // Vehicle spec IDs 0x040–0x043 only (same periodic set as DistributedBMSPrimary).
    if ((now_ms - last_bms_status_ms_) >= config_.bms_status_period_ms) {
        sendBmsStatus();
        last_bms_status_ms_ = now_ms;
    }
    if ((now_ms - last_battery_voltage_ms_) >= config_.battery_voltage_period_ms) {
        sendBatteryVoltage();
        last_battery_voltage_ms_ = now_ms;
    }
    if ((now_ms - last_battery_temperature_ms_) >= config_.battery_temperature_period_ms) {
        sendBatteryTemperature();
        last_battery_temperature_ms_ = now_ms;
    }
    if ((now_ms - last_battery_current_ms_) >= config_.battery_current_period_ms) {
        sendBatteryCurrent();
        last_battery_current_ms_ = now_ms;
    }
}

void BmsCanInterface::updateEventDrivenTransmission(uint32_t now_ms) {
    if (config_.enable_fault_messages) {
        checkAndSendFaults(now_ms);
    }
    if (config_.enable_state_change_messages) {
        checkAndSendStateChange(now_ms);
    }
}

void BmsCanInterface::checkAndSendFaults(uint32_t now_ms) {
    const uint16_t faults = bms_manager_.getActiveFaults();
    if (faults == last_faults_) {
        return;
    }
    if (!canTransmit(now_ms)) {
        return;
    }
    sendFaultStatus();
    last_faults_ = faults;
}

void BmsCanInterface::checkAndSendStateChange(uint32_t now_ms) {
    const BmsManager::BmsState s = bms_manager_.getState();
    if (s == last_state_) {
        return;
    }
    if (!canTransmit(now_ms)) {
        return;
    }
    sendStateChange(last_state_, s);
    last_state_ = s;
    state_entry_time_ms_ = now_ms;
}

void BmsCanInterface::processReceivedMessages(uint32_t now_ms) {
    (void)now_ms;

    CanBus::Frame f{};
    while (can_bus_.read(f)) {
        if (f.rtr) {
            continue;
        }

        const CanFdFrame wire = frameFromCanBus(f);
        rx_debug_.last_id = wire.id;
        rx_debug_.last_dlc = wire.dlc;
        for (uint8_t i = 0; i < wire.dlc && i < 8; ++i) {
            rx_debug_.last_data[i] = wire.data[i];
        }
        ++rx_debug_.rx_count;

        if (wire.id == BmsCanProtocol::BMS_COMMAND) {
            BmsCanProtocol::CommandMsg cmd{};
            if (BmsCanProtocol::MessageDecoder::decodeCommand(wire, cmd)) {
                handleCommand(cmd);
                ++rx_command_count_;
            }
        } else if (wire.id == BmsCanProtocol::BMS_CONFIG_REQUEST) {
            BmsCanProtocol::ConfigRequestMsg req{};
            if (BmsCanProtocol::MessageDecoder::decodeConfigRequest(wire, req)) {
                handleConfigRequest(req);
            }
        }
    }
}

void BmsCanInterface::handleCommand(const BmsCanProtocol::CommandMsg& cmd) {
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
        break;
    }
}

void BmsCanInterface::handleConfigRequest(const BmsCanProtocol::ConfigRequestMsg& req) {
    const BmsManager::Config& c = bms_manager_.getConfig();
    BmsCanProtocol::ConfigResponseMsg resp{};
    resp.response_type = req.request_type;
    resp.reserved = 0;
    resp.cell_overvoltage_mV = c.cell_overvoltage_mV;
    resp.cell_undervoltage_mV = c.cell_undervoltage_mV;
    resp.cell_imbalance_mV = c.cell_imbalance_mV;
    resp.reserved2 = 0;
    if (req.request_type == BmsCanProtocol::CFG_REQ_NONE) {
        return;
    }
    sendWire(BmsCanProtocol::MessageEncoder::encodeConfigResponse(resp));
}

BmsCanProtocol::BmsStatusMsg BmsCanInterface::createBmsStatusMsg(uint32_t now_ms) {
    BmsCanProtocol::BmsStatusMsg msg{};
    const uint16_t faults = bms_manager_.getActiveFaults() & 0x003Fu;
    msg.bms_faults = faults;
    msg.contactors_state = bms_manager_.areContactorsClosed() ? 1u : 0u;
    msg.daughter_board_status = bms_manager_.getDaughterBoardStatusBitmap(now_ms);
    msg.reserved[0] = 0;
    msg.reserved[1] = 0;
    msg.reserved[2] = 0;
    msg.reserved[3] = 0;
    return msg;
}

BmsCanProtocol::BatteryVoltageMsg BmsCanInterface::createBatteryVoltageMsg() {
    BmsCanProtocol::BatteryVoltageMsg msg{};
    float pack_voltage_V = bms_manager_.getPackVoltage_V();
    if (pack_voltage_V < 0.0f) {
        pack_voltage_V = 0.0f;
    }
    msg.total_voltage_x100 = static_cast<uint16_t>(pack_voltage_V * 100.0f);
    const FleetSummaryData summary = bms_manager_.getFleetSummary();
    msg.highest_cell_mV = summary.highest_cell_mV;
    msg.lowest_cell_mV = summary.lowest_cell_mV;
    msg.highest_cell_idx = summary.highest_cell_idx;
    msg.lowest_cell_idx = summary.lowest_cell_idx;
    return msg;
}

BmsCanProtocol::BatteryTemperatureMsg BmsCanInterface::createBatteryTemperatureMsg() {
    BmsCanProtocol::BatteryTemperatureMsg msg{};
    const FleetSummaryData summary = bms_manager_.getFleetSummary();
    msg.high_temp_C_x10 = static_cast<int16_t>(summary.highest_temp_C * 10.0f);
    msg.high_temp_idx = summary.highest_temp_idx;
    msg.avg_temp_C_x10 = msg.high_temp_C_x10;
    return msg;
}

BmsCanProtocol::BatteryCurrentMsg BmsCanInterface::createBatteryCurrentMsg() {
    BmsCanProtocol::BatteryCurrentMsg msg{};
    msg.current_A = bms_manager_.getBatteryCurrent_A();
    return msg;
}

BmsCanProtocol::HeartbeatMsg BmsCanInterface::createHeartbeatMsg(uint32_t now_ms) {
    BmsCanProtocol::HeartbeatMsg msg{};
    msg.node_id = config_.node_id;
    msg.state = static_cast<uint8_t>(bms_manager_.getState());
    const uint16_t faults = bms_manager_.getActiveFaults();
    msg.fault_count = 0;
    for (int i = 0; i < 16; i++) {
        if (faults & (1 << i)) {
            ++msg.fault_count;
        }
    }
    msg.warning_count = 0;
    msg.uptime_s = static_cast<uint16_t>((now_ms - system_start_ms_) / 1000u);
    msg.sequence = sequence_counter_++;
    return msg;
}

BmsCanProtocol::PackStatusMsg BmsCanInterface::createPackStatusMsg() {
    BmsCanProtocol::PackStatusMsg msg{};
    const float pack_voltage_V = bms_manager_.getPackVoltage_V();
    msg.pack_voltage_mV = static_cast<uint16_t>(pack_voltage_V * 1000.0f);
    const float pack_current_A = bms_manager_.getBatteryCurrent_A();
    msg.pack_current_mA = static_cast<int16_t>(pack_current_A * 1000.0f);
    msg.soc_percent_x10 = static_cast<uint16_t>(bms_manager_.getStateOfCharge() * 10.0f);
    msg.soh_percent_x10 = 1000;
    msg.contactor_state = bms_manager_.areContactorsClosed() ? 1 : 0;
    msg.reserved = 0;
    return msg;
}

BmsCanProtocol::TemperatureMsg BmsCanInterface::createTemperatureMsg() {
    BmsCanProtocol::TemperatureMsg msg{};
    const FleetSummaryData summary = bms_manager_.getFleetSummary();
    msg.highest_temp_C_x10 = static_cast<int16_t>(summary.highest_temp_C * 10.0f);
    msg.lowest_temp_C_x10 = msg.highest_temp_C_x10;
    msg.avg_temp_C_x10 = msg.highest_temp_C_x10;
    msg.highest_temp_idx = summary.highest_temp_idx;
    msg.lowest_temp_idx = summary.highest_temp_idx;
    msg.num_sensors = 0;
    msg.reserved = 0;
    return msg;
}

BmsCanProtocol::CellVoltagesMsg BmsCanInterface::createCellVoltagesMsg() {
    BmsCanProtocol::CellVoltagesMsg msg{};
    const FleetSummaryData summary = bms_manager_.getFleetSummary();
    msg.highest_cell_mV = summary.highest_cell_mV;
    msg.lowest_cell_mV = summary.lowest_cell_mV;
    msg.avg_cell_mV = static_cast<uint16_t>((summary.highest_cell_mV + summary.lowest_cell_mV) / 2);
    msg.highest_cell_idx = summary.highest_cell_idx;
    msg.lowest_cell_idx = summary.lowest_cell_idx;
    const uint16_t imb = summary.highest_cell_mV - summary.lowest_cell_mV;
    msg.imbalance_mV = static_cast<uint8_t>(imb > 255u ? 255u : imb);
    msg.reserved = 0;
    return msg;
}

BmsCanProtocol::FaultStatusMsg BmsCanInterface::createFaultStatusMsg() {
    BmsCanProtocol::FaultStatusMsg msg{};
    const uint16_t faults = bms_manager_.getActiveFaults();
    msg.active_faults = faults;
    msg.fault_count = 0;
    for (int i = 0; i < 16; i++) {
        if (faults & (1 << i)) {
            ++msg.fault_count;
        }
    }
    msg.critical_fault = bms_manager_.hasCriticalFault() ? 1 : 0;
    msg.fault_code = 0;
    for (int i = 15; i >= 0; i--) {
        if (faults & (1 << i)) {
            msg.fault_code = static_cast<uint8_t>(i);
            break;
        }
    }
    const uint32_t now_ms = HAL_GetTick();
    msg.fault_timestamp_s = static_cast<uint16_t>((now_ms - system_start_ms_) / 1000u);
    msg.reserved[0] = 0;
    msg.reserved[1] = 0;
    return msg;
}

BmsCanProtocol::StateChangeMsg BmsCanInterface::createStateChangeMsg(BmsManager::BmsState old_state,
                                                                     BmsManager::BmsState new_state,
                                                                     uint32_t duration_ms) {
    BmsCanProtocol::StateChangeMsg msg{};
    msg.old_state = static_cast<uint8_t>(old_state);
    msg.new_state = static_cast<uint8_t>(new_state);
    msg.state_duration_ms = static_cast<uint16_t>(duration_ms > 0xFFFFu ? 0xFFFFu : duration_ms);
    msg.timestamp_ms = HAL_GetTick();
    return msg;
}

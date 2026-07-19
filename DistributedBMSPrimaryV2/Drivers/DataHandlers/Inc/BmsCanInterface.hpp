#pragma once

#include "BmsCanProtocol.hpp"
#include "BmsManager.hpp"
#include "CanBus.hpp"
#include "CanFdFrame.hpp"
#include "FleetSummary.hpp"

#include <cstdint>

/**
 * BMS CAN Interface - Application Layer
 *
 * Vehicle TX: periodic 0x040–0x043 (pack) + 0x044+N (per-module).
 * Vehicle RX: kill 0x010, debug 0x0A1, commands 0x1A0, config request 0x1A1.
 */
class BmsCanInterface {
public:
    struct RxDebug {
        uint32_t last_id = 0;
        uint8_t last_dlc = 0;
        uint8_t last_data[8]{};
        uint32_t rx_count = 0;
    };

    struct Config {
        uint32_t bms_status_period_ms = 50;
        uint32_t battery_voltage_period_ms = 50;
        uint32_t battery_temperature_period_ms = 50;
        uint32_t battery_current_period_ms = 50;
        uint32_t module_status_period_ms = 50;
        /** How many module frames to send (IDs 0x044 .. 0x044+count-1). */
        uint8_t module_count = PrimaryBmsFleetCfg::MAX_MODULES;

        // Legacy / unused by periodic TX
        uint32_t heartbeat_period_ms = 100;
        uint32_t pack_status_period_ms = 50;
        uint32_t temperature_period_ms = 100;
        uint32_t cell_voltages_period_ms = 100;

        bool enable_fault_messages = false;
        bool enable_warning_messages = false;
        bool enable_state_change_messages = false;

        uint8_t node_id = 0x01;
        uint32_t min_message_interval_ms = 10;
    };

    BmsCanInterface(CanBus& can_bus, BmsManager& bms_manager);

    void init(const Config& config);
    void update(uint32_t now_ms);

    void sendBmsStatus();
    void sendBatteryVoltage();
    void sendBatteryTemperature();
    void sendBatteryCurrent();
    void sendModuleStatus(uint8_t module_index);
    void sendHeartbeat();
    void sendPackStatus();
    void sendTemperature();
    void sendCellVoltages();
    void sendFaultStatus();
    void sendStateChange(BmsManager::BmsState old_state, BmsManager::BmsState new_state);

    void setConfig(const Config& config);
    const Config& getConfig() const;

    uint32_t txOkCount() const { return tx_ok_count_; }
    uint32_t txErrorCount() const { return tx_error_count_; }
    uint32_t rxCommandCount() const { return rx_command_count_; }
    const RxDebug& rxDebug() const { return rx_debug_; }

private:
    void updatePeriodicTransmission(uint32_t now_ms);
    void updateEventDrivenTransmission(uint32_t now_ms);
    void checkAndSendFaults(uint32_t now_ms);
    void checkAndSendStateChange(uint32_t now_ms);

    void processReceivedMessages(uint32_t now_ms);
    void handleCommand(const BmsCanProtocol::CommandMsg& cmd);
    void handleKillSwitch(const CanFdFrame& frame);
    void handleConfigRequest(const BmsCanProtocol::ConfigRequestMsg& req);
    void handleDebugMode(const BmsCanProtocol::DebugModeMsg& msg);

    void sendWire(const CanFdFrame& fr);

    BmsCanProtocol::BmsStatusMsg createBmsStatusMsg(uint32_t now_ms);
    BmsCanProtocol::BatteryVoltageMsg createBatteryVoltageMsg();
    BmsCanProtocol::BatteryTemperatureMsg createBatteryTemperatureMsg();
    BmsCanProtocol::BatteryCurrentMsg createBatteryCurrentMsg();
    BmsCanProtocol::ModuleStatusMsg createModuleStatusMsg(uint8_t module_index);
    BmsCanProtocol::HeartbeatMsg createHeartbeatMsg(uint32_t now_ms);
    BmsCanProtocol::PackStatusMsg createPackStatusMsg();
    BmsCanProtocol::TemperatureMsg createTemperatureMsg();
    BmsCanProtocol::CellVoltagesMsg createCellVoltagesMsg();
    BmsCanProtocol::FaultStatusMsg createFaultStatusMsg();
    BmsCanProtocol::StateChangeMsg createStateChangeMsg(
        BmsManager::BmsState old_state,
        BmsManager::BmsState new_state,
        uint32_t duration_ms);

    bool canTransmit(uint32_t now_ms);

    CanBus& can_bus_;
    BmsManager& bms_manager_;
    Config config_;

    RxDebug rx_debug_{};

    uint32_t last_bms_status_ms_ = 0;
    uint32_t last_battery_voltage_ms_ = 0;
    uint32_t last_battery_temperature_ms_ = 0;
    uint32_t last_battery_current_ms_ = 0;
    uint32_t last_module_status_ms_ = 0;
    uint32_t last_tx_ms_ = 0;

    uint16_t last_faults_ = 0;
    BmsManager::BmsState last_state_ = BmsManager::BmsState::INIT;
    uint32_t state_entry_time_ms_ = 0;

    uint32_t tx_ok_count_ = 0;
    uint32_t tx_error_count_ = 0;
    uint32_t rx_command_count_ = 0;
    uint32_t rx_kill_count_ = 0;

    uint32_t system_start_ms_ = 0;
    uint16_t sequence_counter_ = 0;
};

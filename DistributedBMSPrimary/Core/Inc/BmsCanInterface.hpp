#pragma once

#include "CanFdBus.hpp"
#include "BmsManager.hpp"
#include "BmsCanProtocol.hpp"
#include <cstdint>

/**
 * BMS CAN Interface - Application Layer
 * 
 * High-level interface for BMS CAN communication.
 * Handles:
 * - Periodic message transmission
 * - Event-driven message transmission
 * - Command reception and processing
 * - Message rate limiting
 * - Integration with BmsManager
 */
class BmsCanInterface {
public:
    struct Config {
        // Transmission rates (ms)
        uint32_t heartbeat_period_ms = 100;      // Heartbeat every 100ms
        uint32_t pack_status_period_ms = 50;     // Pack status every 50ms
        uint32_t temperature_period_ms = 100;     // Temperature every 100ms
        uint32_t cell_voltages_period_ms = 100;  // Cell voltages every 100ms
        
        // Event-driven message settings
        bool enable_fault_messages = true;
        bool enable_warning_messages = true;
        bool enable_state_change_messages = true;
        
        // Node identification
        uint8_t node_id = 0x01;
        
        // Rate limiting
        uint32_t min_message_interval_ms = 10;    // Minimum time between messages
    };

    BmsCanInterface(CanFdBus& can_bus, BmsManager& bms_manager);
    
    // Initialization
    void init(const Config& config);
    
    // Main update function - call regularly from main loop
    void update(uint32_t now_ms);
    
    // Force transmission (for testing/debugging)
    void sendHeartbeat();
    void sendPackStatus();
    void sendTemperature();
    void sendCellVoltages();
    void sendFaultStatus();
    void sendStateChange(BmsManager::BmsState old_state, BmsManager::BmsState new_state);
    
    // Configuration
    void setConfig(const Config& config);
    const Config& getConfig() const;
    
    // Statistics
    uint32_t txOkCount() const { return tx_ok_count_; }
    uint32_t txErrorCount() const { return tx_error_count_; }
    uint32_t rxCommandCount() const { return rx_command_count_; }
    
private:
    // Periodic transmission handlers
    void updatePeriodicTransmission(uint32_t now_ms);
    
    // Event-driven transmission handlers
    void updateEventDrivenTransmission(uint32_t now_ms);
    void checkAndSendFaults(uint32_t now_ms);
    void checkAndSendStateChange(uint32_t now_ms);
    
    // Command processing
    void processReceivedMessages(uint32_t now_ms);
    void handleCommand(const BmsCanProtocol::CommandMsg& cmd);
    void handleConfigRequest(const BmsCanProtocol::ConfigRequestMsg& req);
    
    // Message creation helpers
    BmsCanProtocol::HeartbeatMsg createHeartbeatMsg(uint32_t now_ms);
    BmsCanProtocol::PackStatusMsg createPackStatusMsg();
    BmsCanProtocol::TemperatureMsg createTemperatureMsg();
    BmsCanProtocol::CellVoltagesMsg createCellVoltagesMsg();
    BmsCanProtocol::FaultStatusMsg createFaultStatusMsg();
    BmsCanProtocol::StateChangeMsg createStateChangeMsg(
        BmsManager::BmsState old_state, 
        BmsManager::BmsState new_state,
        uint32_t duration_ms);
    
    // Rate limiting
    bool canTransmit(uint32_t now_ms);
    
    // Member variables
    CanFdBus& can_bus_;
    BmsManager& bms_manager_;
    Config config_;
    
    // Timing
    uint32_t last_heartbeat_ms_ = 0;
    uint32_t last_pack_status_ms_ = 0;
    uint32_t last_temperature_ms_ = 0;
    uint32_t last_cell_voltages_ms_ = 0;
    uint32_t last_tx_ms_ = 0;
    
    // State tracking for event-driven messages
    uint16_t last_faults_ = 0;
    BmsManager::BmsState last_state_ = BmsManager::BmsState::INIT;
    uint32_t state_entry_time_ms_ = 0;
    
    // Statistics
    uint32_t tx_ok_count_ = 0;
    uint32_t tx_error_count_ = 0;
    uint32_t rx_command_count_ = 0;
    
    // System uptime
    uint32_t system_start_ms_ = 0;
    uint16_t sequence_counter_ = 0;
};

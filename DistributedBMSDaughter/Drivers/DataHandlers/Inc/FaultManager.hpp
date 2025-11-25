/*
 * FaultManager.hpp
 *
 * Created on: Oct 27, 2025
 *      Author: samrb
 */

 #ifndef INC_FAULTMANAGER_HPP_
 #define INC_FAULTMANAGER_HPP_
 
 #pragma once
 #include <cstdint>
 #include <array>
 
 class FaultManager {
 public:
    enum class FaultType : uint8_t {
        NONE = 0,
        COMMUNICATION_TIMEOUT = 1,
        BQ76920_COMM_ERROR = 2,
        ADC_READ_ERROR = 3,
        CAN_TRANSMIT_ERROR = 4,
        CAN_RECEIVE_ERROR = 5,
        EEPROM_WRITE_ERROR = 6,
        WATCHDOG_TIMEOUT = 7,
        TEMPERATURE_SENSOR_FAIL = 8,
        SPI_COMM_ERROR = 9,
        I2C_COMM_ERROR = 10,
        DMA_ERROR = 11,
        CLOCK_ERROR = 12,
        MEMORY_ERROR = 13,
        RESTART_ATTEMPTED = 14,
        MAX_FAULTS = 32
    };
 
     enum class FaultSeverity : uint8_t {
         INFO = 0,      // Non-critical, just logging
         WARNING = 1,   // Monitor but continue operation
         ERROR = 2,     // System degraded but functional
         CRITICAL = 3   // System non-functional
     };
 
     struct FaultInfo {
         FaultType type;
         FaultSeverity severity;
         uint32_t timestamp_ms;
         uint32_t count;
         bool active;
     };
 
     FaultManager();
     
     // Fault management
     void setFault(FaultType type, bool active, FaultSeverity severity = FaultSeverity::ERROR);
     void clearFault(FaultType type);
     bool hasFault(FaultType type) const;
     bool hasActiveFaults() const;
     bool hasCriticalFaults() const;
     
     // Fault information
     uint32_t getFaultMask() const;
     FaultInfo getFaultInfo(FaultType type) const;
     uint8_t getActiveFaultCount() const;
     
     // System status - focused on hardware functionality
     bool isSystemFunctional() const;
     bool canReadVoltages() const;
     bool canReadTemperatures() const;
     bool canTransmitCAN() const;
     
    // Update and maintenance
    void update(uint32_t current_time_ms);
    void clearAllFaults();
    
    // Restart functionality
    bool shouldRestart() const;
    void attemptRestart();
    void setRestartEnabled(bool enabled);
    void setRestartDelay(uint32_t delay_ms);
    void setMaxRestartAttempts(uint8_t max_attempts);
    uint8_t getRestartAttemptCount() const;
    
    // Communication timeout tracking
    void updateCommunicationTime(uint32_t current_time_ms);
    void setCommunicationTimeout(uint32_t timeout_ms);
    bool hasCommunicationTimeout() const;
    
    // Fault logging
    void logFaultEvent(FaultType type, bool active, uint32_t timestamp_ms);
 
private:
    static constexpr uint8_t MAX_FAULT_TYPES = 32;
    static constexpr uint32_t FAULT_DEBOUNCE_MS = 100;
    static constexpr uint32_t FAULT_LOG_SIZE = 64;
    static constexpr uint32_t DEFAULT_RESTART_DELAY_MS = 5000;  // 5 seconds
    static constexpr uint8_t DEFAULT_MAX_RESTART_ATTEMPTS = 3;
    static constexpr uint32_t DEFAULT_COMM_TIMEOUT_MS = 10000;  // 10 seconds
    
    std::array<FaultInfo, MAX_FAULT_TYPES> faults_;
    uint32_t fault_mask_;
    uint32_t last_update_ms_;
    
    // Restart functionality
    bool restart_enabled_;
    uint32_t restart_delay_ms_;
    uint8_t max_restart_attempts_;
    uint8_t restart_attempt_count_;
    uint32_t last_restart_time_ms_;
    bool restart_pending_;
    
    // Communication timeout tracking
    uint32_t comm_timeout_ms_;
    uint32_t last_comm_time_ms_;
    
    // Fault logging
    struct FaultLogEntry {
        FaultType type;
        bool active;
        uint32_t timestamp_ms;
    };
    std::array<FaultLogEntry, FAULT_LOG_SIZE> fault_log_;
    uint8_t log_head_;
    uint8_t log_count_;
    
    void addToLog(FaultType type, bool active, uint32_t timestamp_ms);
    bool isFaultDebounced(FaultType type, uint32_t current_time_ms) const;
 };
 
 #endif /* INC_FAULTMANAGER_HPP_ */

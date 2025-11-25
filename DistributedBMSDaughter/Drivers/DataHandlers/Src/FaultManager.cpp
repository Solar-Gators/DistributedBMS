/*
 * FaultManager.cpp
 *
 * Created on: Oct 27, 2025
 *      Author: samrb
 */

 #include "FaultManager.hpp"
 #include <algorithm>
 
FaultManager::FaultManager() 
    : fault_mask_(0)
    , last_update_ms_(0)
    , restart_enabled_(true)
    , restart_delay_ms_(DEFAULT_RESTART_DELAY_MS)
    , max_restart_attempts_(DEFAULT_MAX_RESTART_ATTEMPTS)
    , restart_attempt_count_(0)
    , last_restart_time_ms_(0)
    , restart_pending_(false)
    , comm_timeout_ms_(DEFAULT_COMM_TIMEOUT_MS)
    , last_comm_time_ms_(0)
    , log_head_(0)
    , log_count_(0) {
    
    // Initialize all faults as inactive
    for (auto& fault : faults_) {
        fault = {FaultType::NONE, FaultSeverity::INFO, 0, 0, false};
    }
}
 
 void FaultManager::setFault(FaultType type, bool active, FaultSeverity severity) {
     if (type >= FaultType::MAX_FAULTS) return;
     
     uint8_t index = static_cast<uint8_t>(type);
     FaultInfo& fault = faults_[index];
     
     // Only update if state actually changed
     if (fault.active != active) {
         fault.active = active;
         fault.type = type;
         fault.severity = severity;
         fault.timestamp_ms = last_update_ms_;
         
         if (active) {
             fault.count++;
             fault_mask_ |= (1U << index);
         } else {
             fault_mask_ &= ~(1U << index);
         }
         
         // Log the fault event
         logFaultEvent(type, active, last_update_ms_);
     }
 }
 
 void FaultManager::clearFault(FaultType type) {
     setFault(type, false, FaultSeverity::INFO);
 }
 
 bool FaultManager::hasFault(FaultType type) const {
     if (type >= FaultType::MAX_FAULTS) return false;
     uint8_t index = static_cast<uint8_t>(type);
     return faults_[index].active;
 }
 
 bool FaultManager::hasActiveFaults() const {
     return fault_mask_ != 0;
 }
 
 bool FaultManager::hasCriticalFaults() const {
     for (const auto& fault : faults_) {
         if (fault.active && fault.severity == FaultSeverity::CRITICAL) {
             return true;
         }
     }
     return false;
 }
 
 uint32_t FaultManager::getFaultMask() const {
     return fault_mask_;
 }
 
 FaultManager::FaultInfo FaultManager::getFaultInfo(FaultType type) const {
     if (type >= FaultType::MAX_FAULTS) {
         return {FaultType::NONE, FaultSeverity::INFO, 0, 0, false};
     }
     return faults_[static_cast<uint8_t>(type)];
 }
 
 uint8_t FaultManager::getActiveFaultCount() const {
     uint8_t count = 0;
     for (uint8_t i = 0; i < MAX_FAULT_TYPES; ++i) {
         if (fault_mask_ & (1U << i)) {
             count++;
         }
     }
     return count;
 }
 
 bool FaultManager::isSystemFunctional() const {
     return !hasCriticalFaults() && 
            !hasFault(FaultType::BQ76920_COMM_ERROR) &&
            !hasFault(FaultType::CAN_TRANSMIT_ERROR);
 }
 
 bool FaultManager::canReadVoltages() const {
     return !hasFault(FaultType::BQ76920_COMM_ERROR) &&
            !hasFault(FaultType::I2C_COMM_ERROR);
 }
 
 bool FaultManager::canReadTemperatures() const {
     return !hasFault(FaultType::ADC_READ_ERROR) &&
            !hasFault(FaultType::DMA_ERROR) &&
            !hasFault(FaultType::TEMPERATURE_SENSOR_FAIL);
 }
 
 bool FaultManager::canTransmitCAN() const {
     return !hasFault(FaultType::CAN_TRANSMIT_ERROR);
 }
 
void FaultManager::update(uint32_t current_time_ms) {
    last_update_ms_ = current_time_ms;
    
    // Check for communication timeouts
    if (last_comm_time_ms_ > 0 && 
        (current_time_ms - last_comm_time_ms_) > comm_timeout_ms_) {
        setFault(FaultType::COMMUNICATION_TIMEOUT, true, FaultSeverity::CRITICAL);
    } else {
        clearFault(FaultType::COMMUNICATION_TIMEOUT);
    }
    
    // Handle restart logic - now includes communication timeouts
    bool should_trigger_restart = false;
    
    if (restart_enabled_ && !restart_pending_) {
        // Check for critical faults OR communication timeout
        if (hasCriticalFaults() || hasCommunicationTimeout()) {
            should_trigger_restart = true;
        }
        
        // Check if enough time has passed since last restart attempt
        if (should_trigger_restart && 
            current_time_ms - last_restart_time_ms_ >= restart_delay_ms_) {
            // Check if we haven't exceeded max restart attempts
            if (restart_attempt_count_ < max_restart_attempts_) {
                restart_pending_ = true;
            }
        }
    }
}
 
 void FaultManager::clearAllFaults() {
     fault_mask_ = 0;
     for (auto& fault : faults_) {
         fault.active = false;
     }
 }
 
 void FaultManager::logFaultEvent(FaultType type, bool active, uint32_t timestamp_ms) {
     addToLog(type, active, timestamp_ms);
 }
 
 void FaultManager::addToLog(FaultType type, bool active, uint32_t timestamp_ms) {
     fault_log_[log_head_] = {type, active, timestamp_ms};
     log_head_ = (log_head_ + 1) % FAULT_LOG_SIZE;
     if (log_count_ < FAULT_LOG_SIZE) {
         log_count_++;
     }
 }
 
bool FaultManager::isFaultDebounced(FaultType type, uint32_t current_time_ms) const {
    if (type >= FaultType::MAX_FAULTS) return false;
    
    const FaultInfo& fault = faults_[static_cast<uint8_t>(type)];
    return (current_time_ms - fault.timestamp_ms) >= FAULT_DEBOUNCE_MS;
}

// Restart functionality implementation
bool FaultManager::shouldRestart() const {
    return restart_pending_;
}

void FaultManager::attemptRestart() {
    if (restart_pending_) {

        last_restart_time_ms_ = last_update_ms_;
        restart_pending_ = false;
        
        // Log the restart attempt
        setFault(FaultType::RESTART_ATTEMPTED, true, FaultSeverity::INFO);
        
        // Clear all faults to give the system a fresh start
        clearAllFaults();
        
        // Reset restart attempt count if we've had a successful period
        // This could be enhanced with more sophisticated logic
    }
}

void FaultManager::setRestartEnabled(bool enabled) {
    restart_enabled_ = enabled;
    if (!enabled) {
        restart_pending_ = false;
    }
}

void FaultManager::setRestartDelay(uint32_t delay_ms) {
    restart_delay_ms_ = delay_ms;
}

void FaultManager::setMaxRestartAttempts(uint8_t max_attempts) {
    max_restart_attempts_ = max_attempts;
}

uint8_t FaultManager::getRestartAttemptCount() const {
    return restart_attempt_count_;
}

// Communication timeout functionality implementation
void FaultManager::updateCommunicationTime(uint32_t current_time_ms) {
    last_comm_time_ms_ = current_time_ms;
}

void FaultManager::setCommunicationTimeout(uint32_t timeout_ms) {
    comm_timeout_ms_ = timeout_ms;
}

bool FaultManager::hasCommunicationTimeout() const {
    return hasFault(FaultType::COMMUNICATION_TIMEOUT);
}

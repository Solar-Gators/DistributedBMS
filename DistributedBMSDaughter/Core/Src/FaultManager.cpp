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
     // This would be implemented based on your specific timeout requirements
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
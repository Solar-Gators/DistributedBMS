# Fault Management System Usage Guide

## Overview

The Fault Management System is a comprehensive monitoring and diagnostic framework designed to track system health, detect failures, and provide recovery mechanisms. It focuses exclusively on hardware functionality and communication status, without making battery management decisions.

## Key Features

- **Hardware-Focused Monitoring**: Tracks system functionality, not battery management
- **Multi-Level Fault Severity**: INFO, WARNING, ERROR, and CRITICAL levels
- **Automatic Recovery**: Clears faults when conditions improve
- **Fault Logging**: Maintains history of fault events for debugging
- **System Status Assessment**: Provides overall system health indicators
- **Debouncing**: Prevents false fault triggers from transient conditions

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                Fault Management System                     │
│                                                             │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐     │
│  │   Sensors   │    │ Communication│    │   System    │     │
│  │ Monitoring  │    │ Monitoring   │    │ Monitoring  │     │
│  └──────┬──────┘    └──────┬──────┘    └──────┬──────┘     │
│         │                  │                  │            │
│         └──────────────────┼──────────────────┘            │
│                            │                               │
│  ┌─────────────────────────▼─────────────────────────┐     │
│  │              Fault Detection                       │     │
│  │  • Range checking                                  │     │
│  │  • Communication timeouts                          │     │
│  │  • Hardware failures                               │     │
│  │  • System errors                                   │     │
│  └─────────────────────────┬─────────────────────────┘     │
│                            │                               │
│  ┌─────────────────────────▼─────────────────────────┐     │
│  │              Fault Processing                      │     │
│  │  • Severity assessment                            │     │
│  │  • Debouncing                                     │     │
│  │  • Logging                                        │     │
│  │  • Recovery management                            │     │
│  └─────────────────────────┬─────────────────────────┘     │
│                            │                               │
│  ┌─────────────────────────▼─────────────────────────┐     │
│  │              System Status                         │     │
│  │  • Health indicators                               │     │
│  │  • Capability assessment                           │     │
│  │  • Fault reporting                                 │     │
│  └─────────────────────────┘                         │
└─────────────────────────────────────────────────────────────┘
```

## Fault Types

### Communication Faults

#### BQ76920_COMM_ERROR
**Purpose**: Monitors communication with BQ76920 battery monitor IC
**Severity**: ERROR
**Triggers**:
- I2C communication failures
- Register read/write errors
- Device not responding

**Example Usage**:
```cpp
// Set fault when communication fails
if (bq.getBAT(&packVoltage) != HAL_OK) {
    faultManager.setFault(FaultManager::FaultType::BQ76920_COMM_ERROR, true);
}

// Clear fault when communication succeeds
if (bq.getVC(cellVoltages) == HAL_OK) {
    faultManager.clearFault(FaultManager::FaultType::BQ76920_COMM_ERROR);
}
```

#### CAN_TRANSMIT_ERROR
**Purpose**: Monitors CAN bus transmission failures
**Severity**: ERROR
**Triggers**:
- CAN transmission failures
- Bus off conditions
- Transmission timeouts

**Example Usage**:
```cpp
// Monitor CAN transmission
if (can.sendStd(0x101, data, 8) != CanBus::Result::Ok) {
    faultManager.setFault(FaultManager::FaultType::CAN_TRANSMIT_ERROR, true);
} else {
    faultManager.clearFault(FaultManager::FaultType::CAN_TRANSMIT_ERROR);
}
```

#### CAN_RECEIVE_ERROR
**Purpose**: Monitors CAN bus reception issues
**Severity**: WARNING
**Triggers**:
- Receive buffer overflows
- Message corruption
- Reception timeouts

**Example Usage**:
```cpp
// Check for reception errors
if (can.rx_dropped() > 0) {
    faultManager.setFault(FaultManager::FaultType::CAN_RECEIVE_ERROR, true);
}
```

### Sensor Faults

#### ADC_READ_ERROR
**Purpose**: Monitors ADC reading failures
**Severity**: ERROR
**Triggers**:
- ADC values stuck at 0 or maximum
- ADC conversion failures
- Invalid ADC readings

**Example Usage**:
```cpp
// Check ADC readings
for (uint8_t i = 0; i < 5; i++) {
    if (adc_buf[i] == 0 || adc_buf[i] == 4095) {
        faultManager.setFault(FaultManager::FaultType::ADC_READ_ERROR, true);
        break;
    }
}
```

#### TEMPERATURE_SENSOR_FAIL
**Purpose**: Monitors temperature sensor failures
**Severity**: WARNING
**Triggers**:
- Temperature sensor not responding
- Unrealistic temperature readings
- Sensor calibration failures

**Example Usage**:
```cpp
// Check temperature sensor health
if (temperature < -50.0f || temperature > 100.0f) {
    faultManager.setFault(FaultManager::FaultType::TEMPERATURE_SENSOR_FAIL, true);
}
```

### System Faults

#### COMMUNICATION_TIMEOUT
**Purpose**: Monitors communication timeouts
**Severity**: ERROR
**Triggers**:
- No communication for extended period
- Device not responding
- Communication protocol violations

**Example Usage**:
```cpp
// Check communication timeout
uint32_t last_comm_time = getLastCommunicationTime();
if ((HAL_GetTick() - last_comm_time) > 5000) { // 5 second timeout
    faultManager.setFault(FaultManager::FaultType::COMMUNICATION_TIMEOUT, true);
}
```

#### DMA_ERROR
**Purpose**: Monitors DMA operation failures
**Severity**: CRITICAL
**Triggers**:
- DMA transfer failures
- DMA timeout conditions
- DMA configuration errors

**Example Usage**:
```cpp
// Check DMA completion
if (!TempDMAComplete && (HAL_GetTick() - dma_start_time) > 100) {
    faultManager.setFault(FaultManager::FaultType::DMA_ERROR, true);
}
```

#### MEMORY_ERROR
**Purpose**: Monitors memory operation failures
**Severity**: CRITICAL
**Triggers**:
- Memory allocation failures
- Stack overflow conditions
- Memory corruption detection

**Example Usage**:
```cpp
// Check memory usage
if (getFreeHeapSize() < 1024) { // Less than 1KB free
    faultManager.setFault(FaultManager::FaultType::MEMORY_ERROR, true);
}
```

## Fault Severity Levels

### INFO Level
**Purpose**: Non-critical events for logging and monitoring
**Characteristics**:
- System continues normal operation
- No immediate action required
- Useful for trend analysis
- Logged for diagnostic purposes

**Example**:
```cpp
// Log system startup
faultManager.setFault(FaultManager::FaultType::NONE, false, 
                     FaultManager::FaultSeverity::INFO);
```

### WARNING Level
**Purpose**: Monitor but continue operation
**Characteristics**:
- System degraded but functional
- Increased monitoring recommended
- Preventive maintenance may be needed
- Trend analysis important

**Example**:
```cpp
// Temperature sensor showing signs of failure
faultManager.setFault(FaultManager::FaultType::TEMPERATURE_SENSOR_FAIL, true,
                     FaultManager::FaultSeverity::WARNING);
```

### ERROR Level
**Purpose**: System degraded but functional
**Characteristics**:
- Reduced performance or capability
- Immediate attention recommended
- May affect data quality
- Recovery procedures may be needed

**Example**:
```cpp
// Communication failure
faultManager.setFault(FaultManager::FaultType::BQ76920_COMM_ERROR, true,
                     FaultManager::FaultSeverity::ERROR);
```

### CRITICAL Level
**Purpose**: System non-functional
**Characteristics**:
- Immediate attention required
- System may be unsafe
- Emergency procedures may be needed
- Complete system failure possible

**Example**:
```cpp
// DMA failure - system cannot function
faultManager.setFault(FaultManager::FaultType::DMA_ERROR, true,
                     FaultManager::FaultSeverity::CRITICAL);
```

## Basic Usage Patterns

### 1. Initialization

```cpp
// Create fault manager instance
FaultManager faultManager;

// Initialize in main loop
faultManager.update(HAL_GetTick());
```

### 2. Setting Faults

```cpp
// Set a fault with default severity (ERROR)
faultManager.setFault(FaultManager::FaultType::BQ76920_COMM_ERROR, true);

// Set a fault with specific severity
faultManager.setFault(FaultManager::FaultType::ADC_READ_ERROR, true,
                     FaultManager::FaultSeverity::WARNING);

// Set multiple faults
faultManager.setFault(FaultManager::FaultType::CAN_TRANSMIT_ERROR, true);
faultManager.setFault(FaultManager::FaultType::CAN_RECEIVE_ERROR, true);
```

### 3. Clearing Faults

```cpp
// Clear a specific fault
faultManager.clearFault(FaultManager::FaultType::BQ76920_COMM_ERROR);

// Clear all faults
faultManager.clearAllFaults();
```

### 4. Checking Fault Status

```cpp
// Check if a specific fault is active
if (faultManager.hasFault(FaultManager::FaultType::BQ76920_COMM_ERROR)) {
    // Handle BQ76920 communication error
}

// Check if any faults are active
if (faultManager.hasActiveFaults()) {
    // Handle system faults
}

// Check if critical faults are active
if (faultManager.hasCriticalFaults()) {
    // Handle critical system failure
}
```

## System Status Assessment

### System Functionality Checks

#### isSystemFunctional()
**Purpose**: Determines if system can operate normally
**Returns**: True if system is functional

```cpp
if (faultManager.isSystemFunctional()) {
    // System is healthy, proceed with normal operation
    processData();
    transmitData();
} else {
    // System has issues, implement degraded operation
    handleSystemDegradation();
}
```

#### canReadVoltages()
**Purpose**: Checks if voltage reading is possible
**Returns**: True if voltage reading is functional

```cpp
if (faultManager.canReadVoltages()) {
    // Safe to read voltages
    bq.getVC(cellVoltages);
    bq.getBAT(&packVoltage);
} else {
    // Voltage reading not available
    useLastKnownVoltages();
}
```

#### canReadTemperatures()
**Purpose**: Checks if temperature reading is possible
**Returns**: True if temperature reading is functional

```cpp
if (faultManager.canReadTemperatures()) {
    // Safe to read temperatures
    readTemperatureSensors();
} else {
    // Temperature reading not available
    useEstimatedTemperatures();
}
```

#### canTransmitCAN()
**Purpose**: Checks if CAN transmission is possible
**Returns**: True if CAN transmission is functional

```cpp
if (faultManager.canTransmitCAN()) {
    // Safe to transmit CAN messages
    can.sendStd(0x101, data, 8);
} else {
    // CAN transmission not available
    storeDataForLaterTransmission();
}
```

## Advanced Usage Patterns

### 1. Fault-Based State Machine

```cpp
enum class SystemState {
    NORMAL,
    DEGRADED,
    EMERGENCY,
    SHUTDOWN
};

SystemState determineSystemState() {
    if (faultManager.hasCriticalFaults()) {
        return SystemState::EMERGENCY;
    } else if (faultManager.hasActiveFaults()) {
        return SystemState::DEGRADED;
    } else if (faultManager.isSystemFunctional()) {
        return SystemState::NORMAL;
    } else {
        return SystemState::SHUTDOWN;
    }
}

void handleSystemState(SystemState state) {
    switch (state) {
        case SystemState::NORMAL:
            // Full functionality
            break;
        case SystemState::DEGRADED:
            // Reduced functionality
            break;
        case SystemState::EMERGENCY:
            // Emergency procedures
            break;
        case SystemState::SHUTDOWN:
            // Safe shutdown
            break;
    }
}
```

### 2. Fault Recovery Strategies

#### Automatic Recovery
```cpp
// Attempt automatic recovery
void attemptRecovery(FaultManager::FaultType faultType) {
    switch (faultType) {
        case FaultManager::FaultType::BQ76920_COMM_ERROR:
            // Retry communication
            if (bq.init() == HAL_OK) {
                faultManager.clearFault(faultType);
            }
            break;
            
        case FaultManager::FaultType::CAN_TRANSMIT_ERROR:
            // Reset CAN controller
            HAL_CAN_ResetError(&hcan1);
            if (can.start()) {
                faultManager.clearFault(faultType);
            }
            break;
            
        case FaultManager::FaultType::ADC_READ_ERROR:
            // Recalibrate ADC
            HAL_ADC_Calibration_Start(&hadc1);
            faultManager.clearFault(faultType);
            break;
    }
}
```

#### Manual Recovery
```cpp
// Manual recovery procedures
void performManualRecovery() {
    // Clear all faults
    faultManager.clearAllFaults();
    
    // Reinitialize systems
    bq.init();
    can.start();
    
    // Verify recovery
    if (faultManager.isSystemFunctional()) {
        printf("System recovered successfully\n");
    } else {
        printf("Recovery failed\n");
    }
}
```

### 3. Fault Logging and Analysis

#### Logging Fault Events
```cpp
// Log fault events for analysis
void logFaultEvent(FaultManager::FaultType type, bool active) {
    printf("Fault Event: Type=%d, Active=%s, Time=%d\n",
           static_cast<int>(type),
           active ? "TRUE" : "FALSE",
           HAL_GetTick());
}
```

#### Fault Statistics
```cpp
// Get fault statistics
void printFaultStatistics() {
    printf("Active Faults: %d\n", faultManager.getActiveFaultCount());
    printf("Fault Mask: 0x%08X\n", faultManager.getFaultMask());
    
    // Check each fault type
    for (int i = 1; i < static_cast<int>(FaultManager::FaultType::MAX_FAULTS); i++) {
        auto faultType = static_cast<FaultManager::FaultType>(i);
        if (faultManager.hasFault(faultType)) {
            auto info = faultManager.getFaultInfo(faultType);
            printf("Fault %d: Count=%d, Severity=%d\n",
                   i, info.count, static_cast<int>(info.severity));
        }
    }
}
```

## Integration Examples

### 1. Main Loop Integration

```cpp
int main(void) {
    // Initialize system
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_CAN1_Init();
    MX_I2C2_Init();
    MX_ADC1_Init();
    
    // Initialize fault manager
    FaultManager faultManager;
    
    // Initialize other systems
    BQ7692000PW bq(&hi2c2);
    CanBus can(hcan1);
    BMS bms(4);
    
    while (1) {
        // Update fault manager
        faultManager.update(HAL_GetTick());
        
        // Check system health
        if (!faultManager.isSystemFunctional()) {
            handleSystemFailure();
            continue;
        }
        
        // Perform operations based on system capabilities
        if (faultManager.canReadVoltages()) {
            readVoltages(bq, faultManager);
        }
        
        if (faultManager.canReadTemperatures()) {
            readTemperatures(faultManager);
        }
        
        if (faultManager.canTransmitCAN()) {
            transmitData(can, faultManager);
        }
        
        // Update system status LEDs
        updateStatusLEDs(faultManager);
        
        HAL_Delay(250);
    }
}
```

### 2. Error Handling Integration

```cpp
void readVoltages(BQ7692000PW& bq, FaultManager& faultManager) {
    uint16_t packVoltage;
    std::array<uint16_t, 5> cellVoltages;
    
    // Read pack voltage
    if (bq.getBAT(&packVoltage) != HAL_OK) {
        faultManager.setFault(FaultManager::FaultType::BQ76920_COMM_ERROR, true);
        return;
    } else {
        faultManager.clearFault(FaultManager::FaultType::BQ76920_COMM_ERROR);
    }
    
    // Read cell voltages
    if (bq.getVC(cellVoltages) != HAL_OK) {
        faultManager.setFault(FaultManager::FaultType::BQ76920_COMM_ERROR, true);
        return;
    } else {
        faultManager.clearFault(FaultManager::FaultType::BQ76920_COMM_ERROR);
    }
    
    // Process voltage data
    processVoltageData(cellVoltages, packVoltage);
}
```

### 3. Status LED Integration

```cpp
void updateStatusLEDs(FaultManager& faultManager) {
    // Fault LED - Red when faults are active
    if (faultManager.hasActiveFaults()) {
        HAL_GPIO_WritePin(GPIOB, Fault_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOB, Fault_Pin, GPIO_PIN_RESET);
    }
    
    // OK LED - Green when system is functional
    if (faultManager.isSystemFunctional()) {
        HAL_GPIO_TogglePin(GPIOB, OK_Pin);
    } else {
        HAL_GPIO_WritePin(GPIOB, OK_Pin, GPIO_PIN_RESET);
    }
    
    // Error LED - Yellow for warnings
    if (faultManager.hasFault(FaultManager::FaultType::TEMPERATURE_SENSOR_FAIL)) {
        HAL_GPIO_WritePin(GPIOB, ERROR_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOB, ERROR_Pin, GPIO_PIN_RESET);
    }
}
```

## Best Practices

### 1. Fault Management Strategy

#### Proactive Monitoring
```cpp
// Monitor system health continuously
void monitorSystemHealth() {
    // Check communication health
    if (!isCommunicationHealthy()) {
        faultManager.setFault(FaultManager::FaultType::COMMUNICATION_TIMEOUT, true);
    }
    
    // Check sensor health
    if (!areSensorsHealthy()) {
        faultManager.setFault(FaultManager::FaultType::TEMPERATURE_SENSOR_FAIL, true);
    }
    
    // Check memory health
    if (getFreeHeapSize() < 1024) {
        faultManager.setFault(FaultManager::FaultType::MEMORY_ERROR, true);
    }
}
```

#### Reactive Response
```cpp
// Respond to fault conditions
void respondToFaults() {
    if (faultManager.hasCriticalFaults()) {
        // Implement emergency procedures
        emergencyShutdown();
    } else if (faultManager.hasActiveFaults()) {
        // Implement degraded operation
        degradedOperation();
    }
}
```

### 2. Fault Prevention

#### Input Validation
```cpp
// Validate inputs before processing
bool validateInputs(const std::array<uint16_t, 5>& voltages) {
    for (auto voltage : voltages) {
        if (voltage == 0 || voltage > 5000) {
            faultManager.setFault(FaultManager::FaultType::ADC_READ_ERROR, true);
            return false;
        }
    }
    return true;
}
```

#### Timeout Protection
```cpp
// Implement timeout protection
bool readWithTimeout(I2C_HandleTypeDef* hi2c, uint8_t* data, uint32_t timeout) {
    uint32_t start_time = HAL_GetTick();
    
    while (HAL_I2C_IsDeviceReady(hi2c, 0x08, 1, timeout) != HAL_OK) {
        if ((HAL_GetTick() - start_time) > timeout) {
            faultManager.setFault(FaultManager::FaultType::BQ76920_COMM_ERROR, true);
            return false;
        }
    }
    
    return true;
}
```

### 3. Debugging and Diagnostics

#### Fault Logging
```cpp
// Log fault events for debugging
void logFaultEvent(FaultManager::FaultType type, bool active) {
    static uint32_t last_log_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    // Prevent excessive logging
    if ((current_time - last_log_time) > 1000) { // 1 second minimum
        printf("Fault: Type=%d, Active=%s, Time=%d\n",
               static_cast<int>(type),
               active ? "TRUE" : "FALSE",
               current_time);
        last_log_time = current_time;
    }
}
```

#### System Diagnostics
```cpp
// Perform system diagnostics
void performDiagnostics() {
    printf("=== System Diagnostics ===\n");
    printf("System Functional: %s\n", 
           faultManager.isSystemFunctional() ? "YES" : "NO");
    printf("Can Read Voltages: %s\n", 
           faultManager.canReadVoltages() ? "YES" : "NO");
    printf("Can Read Temperatures: %s\n", 
           faultManager.canReadTemperatures() ? "YES" : "NO");
    printf("Can Transmit CAN: %s\n", 
           faultManager.canTransmitCAN() ? "YES" : "NO");
    printf("Active Faults: %d\n", faultManager.getActiveFaultCount());
    printf("Fault Mask: 0x%08X\n", faultManager.getFaultMask());
    printf("========================\n");
}
```

## Troubleshooting

### Common Issues

#### 1. False Fault Triggers
**Symptoms**: Faults being set when system is actually working
**Causes**:
- Insufficient debouncing
- Overly sensitive thresholds
- Environmental interference

**Solutions**:
```cpp
// Implement proper debouncing
bool isFaultDebounced(FaultManager::FaultType type, uint32_t current_time) {
    auto info = faultManager.getFaultInfo(type);
    return (current_time - info.timestamp_ms) >= 100; // 100ms debounce
}
```

#### 2. Faults Not Clearing
**Symptoms**: Faults remain active even when conditions improve
**Causes**:
- Missing fault clearing logic
- Incorrect recovery procedures
- Persistent hardware issues

**Solutions**:
```cpp
// Ensure proper fault clearing
void clearFaultsWhenConditionsImprove() {
    // Clear communication fault when communication succeeds
    if (bq.getBAT(&packVoltage) == HAL_OK) {
        faultManager.clearFault(FaultManager::FaultType::BQ76920_COMM_ERROR);
    }
    
    // Clear ADC fault when readings are valid
    if (areADCReadingsValid()) {
        faultManager.clearFault(FaultManager::FaultType::ADC_READ_ERROR);
    }
}
```

#### 3. System Status Incorrect
**Symptoms**: System status functions return incorrect values
**Causes**:
- Incorrect fault logic
- Missing fault conditions
- Fault severity misconfiguration

**Solutions**:
```cpp
// Verify system status logic
void verifySystemStatus() {
    bool expected_functional = !faultManager.hasCriticalFaults() && 
                              !faultManager.hasFault(FaultManager::FaultType::BQ76920_COMM_ERROR);
    
    if (faultManager.isSystemFunctional() != expected_functional) {
        printf("System status mismatch detected!\n");
        // Investigate fault configuration
    }
}
```

### Debugging Tools

#### 1. Fault Monitor
```cpp
// Monitor fault changes
void monitorFaultChanges() {
    static uint32_t last_fault_mask = 0;
    uint32_t current_fault_mask = faultManager.getFaultMask();
    
    if (current_fault_mask != last_fault_mask) {
        printf("Fault mask changed: 0x%08X -> 0x%08X\n", 
               last_fault_mask, current_fault_mask);
        last_fault_mask = current_fault_mask;
    }
}
```

#### 2. Performance Monitoring
```cpp
// Monitor fault management performance
void monitorFaultPerformance() {
    static uint32_t last_check_time = 0;
    uint32_t current_time = HAL_GetTick();
    uint32_t check_interval = current_time - last_check_time;
    
    if (check_interval > 300) { // Should be ~250ms
        printf("Fault check interval too long: %dms\n", check_interval);
    }
    
    last_check_time = current_time;
}
```

## Safety Considerations

### Critical Fault Handling
```cpp
// Handle critical faults immediately
void handleCriticalFaults() {
    if (faultManager.hasCriticalFaults()) {
        // Immediate safety actions
        emergencyShutdown();
        
        // Log critical fault
        printf("CRITICAL FAULT DETECTED!\n");
        
        // Notify external systems
        notifyExternalSystems();
    }
}
```

### Fault Recovery Safety
```cpp
// Safe fault recovery
void safeFaultRecovery() {
    // Only attempt recovery if safe to do so
    if (isSafeToRecover()) {
        attemptRecovery();
    } else {
        // Wait for safe conditions
        waitForSafeConditions();
    }
}
```

This comprehensive fault management system usage guide provides everything needed to effectively implement and use the fault management system in your Distributed BMS project. The system focuses on hardware functionality monitoring while providing robust error detection, recovery mechanisms, and diagnostic capabilities.

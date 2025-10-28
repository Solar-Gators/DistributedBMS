# Data Validation System Guide

## Overview

The Data Validation System provides comprehensive validation of sensor data to ensure data integrity, detect sensor failures, and maintain system reliability. It focuses on validating the quality and consistency of sensor readings before they are processed or transmitted.

## Key Features

- **Range Validation**: Ensures all sensor readings are within expected ranges
- **Outlier Detection**: Identifies readings that deviate significantly from recent values
- **Rate Limiting**: Detects unrealistic rate of change in sensor values
- **Stuck Value Detection**: Identifies sensors that are not responding
- **Quality Scoring**: Provides confidence metrics for data reliability
- **Historical Analysis**: Uses recent data for trend analysis and outlier detection

## Architecture

```
Raw Sensor Data
       │
       ▼
┌─────────────────┐
│   Range Check   │ ──► Pass ──► Outlier Detection ──► Pass ──► Rate Check
│                 │ ──► Fail ──► Set Fault ──────────► Fail ──► Set Fault
└─────────────────┘
       │
       ▼
┌─────────────────┐
│ Consistency     │ ──► Pass ──► Quality Assessment ──► Valid Data
│ Check          │ ──► Fail ──► Set Fault ──────────► Invalid Data
└─────────────────┘
```

## Configuration

### ValidationConfig Structure

The validation system uses a configuration structure to define validation parameters:

```cpp
struct ValidationConfig {
    // Voltage limits (mV)
    uint16_t min_cell_voltage_mV = 2000;  // 2.0V minimum
    uint16_t max_cell_voltage_mV = 4500;  // 4.5V maximum
    float max_voltage_deviation_percent = 10.0f;  // 10% max deviation
    
    // Temperature limits (°C)
    float min_temperature_C = -40.0f;     // -40°C minimum
    float max_temperature_C = 85.0f;      // +85°C maximum
    float max_temp_rate_C_per_sec = 5.0f; // 5°C/second max rate
    
    // ADC limits
    uint16_t min_adc_value = 100;         // Avoid dead zones
    uint16_t max_adc_value = 3995;        // Avoid saturation
    uint16_t adc_noise_threshold = 50;     // Max jitter threshold
    
    // Outlier detection
    uint8_t outlier_window_size = 5;      // Historical window size
    float outlier_threshold_sigma = 2.0f;  // Sigma threshold for outliers
    
    // Quality thresholds
    uint8_t min_quality_score = 70;       // Minimum acceptable quality
    float min_confidence_score = 0.8f;     // Minimum confidence level
};
```

### Configuration Examples

#### Conservative Configuration (High Safety)
```cpp
ValidationConfig conservative_config;
conservative_config.min_cell_voltage_mV = 2500;  // 2.5V minimum
conservative_config.max_cell_voltage_mV = 4200;  // 4.2V maximum
conservative_config.max_voltage_deviation_percent = 5.0f;  // 5% max deviation
conservative_config.max_temp_rate_C_per_sec = 2.0f;  // 2°C/second max rate
conservative_config.min_quality_score = 90;  // 90% minimum quality
```

#### Permissive Configuration (Development/Testing)
```cpp
ValidationConfig permissive_config;
permissive_config.min_cell_voltage_mV = 1500;  // 1.5V minimum
permissive_config.max_cell_voltage_mV = 5000;  // 5.0V maximum
permissive_config.max_voltage_deviation_percent = 20.0f;  // 20% max deviation
permissive_config.max_temp_rate_C_per_sec = 10.0f;  // 10°C/second max rate
permissive_config.min_quality_score = 50;  // 50% minimum quality
```

## Validation Types

### 1. Cell Voltage Validation

**Purpose**: Ensures cell voltages are within safe operating ranges and detect sensor failures.

**Validation Criteria**:
- **Range Check**: Voltages must be between `min_cell_voltage_mV` and `max_cell_voltage_mV`
- **Outlier Detection**: Voltages that deviate more than `max_voltage_deviation_percent` from recent values
- **Stuck Value Detection**: Voltages that don't change over time
- **Consistency Check**: All cells should be within reasonable range of each other

**Example Usage**:
```cpp
DataValidator validator;
std::array<uint16_t, 5> cellVoltages = {3700, 3650, 3720, 3680, 0};

auto result = validator.validateCellVoltages(cellVoltages, HAL_GetTick());

if (result.is_valid) {
    // Process valid voltage data
    uint8_t quality = result.quality_score;
    float confidence = result.confidence_score;
} else {
    // Handle validation failure
    switch (result.error_code) {
        case ValidationError::OUT_OF_RANGE:
            // Voltage outside safe range
            break;
        case ValidationError::OUTLIER_DETECTED:
            // Unusual voltage reading
            break;
        case ValidationError::STUCK_VALUE:
            // Sensor not responding
            break;
    }
}
```

### 2. Temperature Validation

**Purpose**: Validates temperature readings for accuracy and detects sensor failures.

**Validation Criteria**:
- **Range Check**: Temperatures must be between `min_temperature_C` and `max_temperature_C`
- **Rate Limiting**: Temperature change rate must be less than `max_temp_rate_C_per_sec`
- **Outlier Detection**: Temperatures that deviate significantly from recent values
- **Sensor Correlation**: Multiple temperature sensors should show consistent trends

**Example Usage**:
```cpp
std::array<float, 5> temperatures = {25.5f, 26.1f, 25.8f, 26.0f, 0.0f};

auto result = validator.validateTemperatures(temperatures, HAL_GetTick());

if (result.is_valid) {
    // Process valid temperature data
} else {
    // Handle temperature validation failure
    if (result.error_code == ValidationError::RATE_TOO_HIGH) {
        // Temperature changing too rapidly - possible sensor failure
    }
}
```

### 3. ADC Validation

**Purpose**: Validates raw ADC readings for sensor health and signal integrity.

**Validation Criteria**:
- **Range Check**: ADC values must be between `min_adc_value` and `max_adc_value`
- **Noise Detection**: ADC jitter must be less than `adc_noise_threshold`
- **Stuck Value Detection**: ADC values that don't change over time
- **Dead Zone Detection**: Values stuck at 0 or maximum (4095)

**Example Usage**:
```cpp
std::array<uint16_t, 5> adcValues = {2048, 2000, 2100, 2050, 0};

auto result = validator.validateADCReadings(adcValues, HAL_GetTick());

if (result.is_valid) {
    // Process valid ADC data
} else {
    // Handle ADC validation failure
    if (result.error_code == ValidationError::NOISE_EXCESSIVE) {
        // ADC showing excessive noise - possible connection issue
    }
}
```

### 4. Combined Validation

**Purpose**: Validates all sensor data together for overall system health assessment.

**Example Usage**:
```cpp
auto combined_result = validator.validateAllData(
    cellVoltages, 
    temperatures, 
    adcValues, 
    HAL_GetTick()
);

if (combined_result.is_valid) {
    // All data is valid, proceed with processing
    uint8_t overall_quality = combined_result.quality_score;
} else {
    // Some data failed validation
    // Check individual validation results for specific issues
}
```

## Quality Scoring

### Quality Score Calculation

The validation system provides quality scores (0-100) based on:

1. **Validation Success**: Base score from successful validation
2. **Confidence Level**: Based on consistency and reliability
3. **Historical Performance**: Trend analysis of recent readings
4. **Sensor Health**: Long-term sensor performance

### Quality Score Interpretation

| Score Range | Interpretation | Action Required |
|-------------|----------------|-----------------|
| 90-100 | Excellent | Normal operation |
| 80-89 | Good | Monitor trends |
| 70-79 | Acceptable | Increased monitoring |
| 60-69 | Poor | Investigate issues |
| 0-59 | Unacceptable | Immediate attention |

### Example Quality Assessment
```cpp
uint8_t quality = validator.calculateOverallQuality(
    voltage_result, 
    temp_result, 
    adc_result
);

if (quality >= 90) {
    // High quality data - proceed with confidence
} else if (quality >= 70) {
    // Acceptable quality - proceed with caution
} else {
    // Poor quality - investigate or reject data
}
```

## Error Handling

### Validation Error Types

```cpp
enum class ValidationError : uint8_t {
    NONE = 0,              // No error
    OUT_OF_RANGE = 1,      // Value outside expected range
    OUTLIER_DETECTED = 2,  // Unusual value detected
    NOISE_EXCESSIVE = 3,   // Excessive signal noise
    RATE_TOO_HIGH = 4,     // Change rate too high
    STUCK_VALUE = 5,       // Value not changing
    INCONSISTENT_DATA = 6, // Data inconsistency
    SENSOR_FAILURE = 7,    // Sensor malfunction
    CALIBRATION_ERROR = 8  // Calibration issue
};
```

### Error Response Strategies

#### 1. Immediate Rejection
```cpp
if (!result.is_valid) {
    // Reject data immediately
    faultManager.setFault(FaultManager::FaultType::ADC_READ_ERROR, true);
    return; // Skip processing
}
```

#### 2. Graceful Degradation
```cpp
if (result.quality_score < 70) {
    // Use data but with reduced confidence
    // Flag for increased monitoring
    faultManager.setFault(FaultManager::FaultType::TEMPERATURE_SENSOR_FAIL, true);
}
```

#### 3. Retry Logic
```cpp
if (result.error_code == ValidationError::NOISE_EXCESSIVE) {
    // Retry reading after short delay
    HAL_Delay(10);
    // Re-read sensor
}
```

## Integration with Fault Management

### Automatic Fault Setting

The validation system automatically integrates with the fault management system:

```cpp
// In main loop
auto validation_result = validator.validateAllData(voltages, temps, adc, timestamp);

if (!validation_result.is_valid) {
    // Set appropriate fault based on error type
    switch (validation_result.error_code) {
        case ValidationError::OUT_OF_RANGE:
            faultManager.setFault(FaultManager::FaultType::ADC_READ_ERROR, true);
            break;
        case ValidationError::STUCK_VALUE:
            faultManager.setFault(FaultManager::FaultType::TEMPERATURE_SENSOR_FAIL, true);
            break;
        // ... other cases
    }
} else {
    // Clear validation-related faults
    faultManager.clearFault(FaultManager::FaultType::ADC_READ_ERROR);
    faultManager.clearFault(FaultManager::FaultType::TEMPERATURE_SENSOR_FAIL);
}
```

## Historical Data Management

### Data History

The validation system maintains a rolling history of recent readings for outlier detection:

```cpp
// Update history after successful validation
validator.updateHistory(HAL_GetTick());

// Clear history if needed (e.g., after sensor replacement)
validator.clearHistory();
```

### Historical Analysis

The system uses historical data for:
- **Trend Analysis**: Detect gradual sensor drift
- **Outlier Detection**: Compare current readings to recent history
- **Pattern Recognition**: Identify recurring issues
- **Calibration Validation**: Verify sensor calibration over time

## Best Practices

### 1. Configuration Management

- **Start Conservative**: Use conservative validation parameters initially
- **Adjust Based on Experience**: Modify parameters based on actual system behavior
- **Document Changes**: Keep track of configuration changes and their effects
- **Test Thoroughly**: Validate configuration changes in test environment

### 2. Error Handling

- **Fail Safe**: Always err on the side of caution
- **Log Everything**: Maintain detailed logs of validation failures
- **Monitor Trends**: Watch for patterns in validation failures
- **Regular Review**: Periodically review validation parameters

### 3. Performance Considerations

- **Minimize Overhead**: Validation should not significantly impact system performance
- **Efficient Algorithms**: Use efficient algorithms for outlier detection
- **Memory Management**: Limit historical data storage
- **CPU Usage**: Monitor CPU usage of validation routines

### 4. Maintenance

- **Regular Calibration**: Periodically calibrate sensors
- **Sensor Replacement**: Replace sensors showing consistent validation failures
- **Parameter Updates**: Update validation parameters as system ages
- **Documentation**: Maintain documentation of validation behavior

## Troubleshooting

### Common Issues

#### 1. False Positives
**Symptoms**: Valid data being rejected
**Causes**: 
- Overly conservative validation parameters
- Insufficient historical data
- Environmental factors not accounted for

**Solutions**:
- Adjust validation thresholds
- Increase historical window size
- Add environmental compensation

#### 2. False Negatives
**Symptoms**: Invalid data passing validation
**Causes**:
- Overly permissive validation parameters
- Insufficient validation criteria
- Sensor failure not detected

**Solutions**:
- Tighten validation parameters
- Add additional validation criteria
- Improve sensor health monitoring

#### 3. Performance Issues
**Symptoms**: System slowdown due to validation
**Causes**:
- Complex validation algorithms
- Excessive historical data storage
- Frequent validation calls

**Solutions**:
- Optimize validation algorithms
- Reduce historical data storage
- Reduce validation frequency

### Debugging Tools

#### 1. Validation Logging
```cpp
// Enable detailed validation logging
if (!result.is_valid) {
    printf("Validation failed: Error=%d, Quality=%d, Confidence=%.2f\n",
           static_cast<int>(result.error_code),
           result.quality_score,
           result.confidence_score);
}
```

#### 2. Historical Data Analysis
```cpp
// Analyze historical data for patterns
validator.clearHistory(); // Reset history
// Collect data over time
// Analyze patterns in validation results
```

#### 3. Configuration Testing
```cpp
// Test different configurations
ValidationConfig test_config = validator.getConfig();
test_config.min_quality_score = 50; // More permissive
validator.setConfig(test_config);
// Test with problematic data
```

## Advanced Features

### Custom Validation Rules

You can extend the validation system with custom rules:

```cpp
// Custom validation function
bool customVoltageValidation(const std::array<uint16_t, 5>& voltages) {
    // Check for specific patterns
    // Return true if validation passes
    return true;
}

// Use in validation pipeline
if (customVoltageValidation(cellVoltages)) {
    auto result = validator.validateCellVoltages(cellVoltages, timestamp);
    // Process result
}
```

### Adaptive Thresholds

Implement adaptive thresholds based on system behavior:

```cpp
// Adjust thresholds based on environmental conditions
if (ambient_temp > 40.0f) {
    config.max_temp_rate_C_per_sec = 8.0f; // Allow higher rate in hot environment
} else {
    config.max_temp_rate_C_per_sec = 5.0f; // Normal rate
}
validator.setConfig(config);
```

This comprehensive data validation system ensures reliable sensor data and helps maintain system integrity in the distributed BMS architecture.

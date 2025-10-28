/*
 * DataValidator.cpp
 *
 * Created on: Oct 27, 2025
 *      Author: samrb
 */

 #include "DataValidator.hpp"
 #include <algorithm>
 #include <numeric>
 
 DataValidator::DataValidator() 
     : history_head_(0)
     , history_count_(0) {
     // Use default config
 }
 
 DataValidator::DataValidator(const ValidationConfig& config) 
     : config_(config)
     , history_head_(0)
     , history_count_(0) {
 }
 
 void DataValidator::setConfig(const ValidationConfig& config) {
     config_ = config;
 }
 
 const DataValidator::ValidationConfig& DataValidator::getConfig() const {
     return config_;
 }
 
 DataValidator::ValidationResult DataValidator::validateCellVoltages(
     const std::array<uint16_t, 5>& voltages_mV, uint32_t timestamp_ms) {
     
     ValidationResult result;
     result.validation_timestamp = timestamp_ms;
     
     // Check range for each cell
     for (size_t i = 0; i < 5; ++i) {
         if (voltages_mV[i] == 0) continue; // Skip unused cells
         
         if (voltages_mV[i] < config_.min_cell_voltage_mV || 
             voltages_mV[i] > config_.max_cell_voltage_mV) {
             result.is_valid = false;
             result.error_code = ValidationError::OUT_OF_RANGE;
             result.confidence_score = 0.0f;
             result.quality_score = 0;
             return result;
         }
     }
     
     // Check for outliers using historical data
     if (history_count_ > 0) {
         for (size_t i = 0; i < 5; ++i) {
             if (voltages_mV[i] == 0) continue;
             
             if (isOutlier(voltages_mV[i], history_[history_head_].voltages_mV)) {
                 result.is_valid = false;
                 result.error_code = ValidationError::OUTLIER_DETECTED;
                 result.confidence_score = 0.3f;
                 result.quality_score = 30;
                 return result;
             }
         }
     }
     
     // Check for stuck values
     if (history_count_ > 0) {
         for (size_t i = 0; i < 5; ++i) {
             if (voltages_mV[i] == 0) continue;
             
             if (isStuckValue(voltages_mV[i], history_[history_head_].voltages_mV[i])) {
                 result.is_valid = false;
                 result.error_code = ValidationError::STUCK_VALUE;
                 result.confidence_score = 0.2f;
                 result.quality_score = 20;
                 return result;
             }
         }
     }
     
     // Calculate confidence based on consistency
     result.confidence_score = calculateConfidence(result);
     result.quality_score = static_cast<uint8_t>(result.confidence_score * 100);
     
     return result;
 }
 
 DataValidator::ValidationResult DataValidator::validateTemperatures(
     const std::array<float, 5>& temperatures_C, uint32_t timestamp_ms) {
     
     ValidationResult result;
     result.validation_timestamp = timestamp_ms;
     
     // Check range for each temperature
     for (size_t i = 0; i < 5; ++i) {
         if (temperatures_C[i] < config_.min_temperature_C || 
             temperatures_C[i] > config_.max_temperature_C) {
             result.is_valid = false;
             result.error_code = ValidationError::OUT_OF_RANGE;
             result.confidence_score = 0.0f;
             result.quality_score = 0;
             return result;
         }
     }
     
     // Check for outliers
     if (history_count_ > 0) {
         for (size_t i = 0; i < 5; ++i) {
             if (isOutlier(temperatures_C[i], history_[history_head_].temperatures_C)) {
                 result.is_valid = false;
                 result.error_code = ValidationError::OUTLIER_DETECTED;
                 result.confidence_score = 0.3f;
                 result.quality_score = 30;
                 return result;
             }
         }
     }
     
     // Check rate of change
     if (history_count_ > 0) {
         uint32_t time_diff = timestamp_ms - history_[history_head_].timestamp_ms;
         if (time_diff > 0) {
             for (size_t i = 0; i < 5; ++i) {
                 if (isRateTooHigh(temperatures_C[i], history_[history_head_].temperatures_C[i], time_diff)) {
                     result.is_valid = false;
                     result.error_code = ValidationError::RATE_TOO_HIGH;
                     result.confidence_score = 0.4f;
                     result.quality_score = 40;
                     return result;
                 }
             }
         }
     }
     
     result.confidence_score = calculateConfidence(result);
     result.quality_score = static_cast<uint8_t>(result.confidence_score * 100);
     
     return result;
 }
 
 DataValidator::ValidationResult DataValidator::validateADCReadings(
     const std::array<uint16_t, 5>& adc_values, uint32_t timestamp_ms) {
     
     ValidationResult result;
     result.validation_timestamp = timestamp_ms;
     
     // Check range for each ADC value
     for (size_t i = 0; i < 5; ++i) {
         if (adc_values[i] < config_.min_adc_value || 
             adc_values[i] > config_.max_adc_value) {
             result.is_valid = false;
             result.error_code = ValidationError::OUT_OF_RANGE;
             result.confidence_score = 0.0f;
             result.quality_score = 0;
             return result;
         }
     }
     
     // Check for stuck values
     if (history_count_ > 0) {
         for (size_t i = 0; i < 5; ++i) {
             if (isStuckValue(adc_values[i], history_[history_head_].adc_values[i])) {
                 result.is_valid = false;
                 result.error_code = ValidationError::STUCK_VALUE;
                 result.confidence_score = 0.2f;
                 result.quality_score = 20;
                 return result;
             }
         }
     }
     
     // Check for excessive noise
     if (history_count_ > 0) {
         for (size_t i = 0; i < 5; ++i) {
             uint16_t noise = std::abs(static_cast<int16_t>(adc_values[i] - history_[history_head_].adc_values[i]));
             if (noise > config_.adc_noise_threshold) {
                 result.is_valid = false;
                 result.error_code = ValidationError::NOISE_EXCESSIVE;
                 result.confidence_score = 0.5f;
                 result.quality_score = 50;
                 return result;
             }
         }
     }
     
     result.confidence_score = calculateConfidence(result);
     result.quality_score = static_cast<uint8_t>(result.confidence_score * 100);
     
     return result;
 }
 
 DataValidator::ValidationResult DataValidator::validateAllData(
     const std::array<uint16_t, 5>& voltages_mV,
     const std::array<float, 5>& temperatures_C,
     const std::array<uint16_t, 5>& adc_values,
     uint32_t timestamp_ms) {
     
     ValidationResult voltage_result = validateCellVoltages(voltages_mV, timestamp_ms);
     ValidationResult temp_result = validateTemperatures(temperatures_C, timestamp_ms);
     ValidationResult adc_result = validateADCReadings(adc_values, timestamp_ms);
     
     ValidationResult combined_result;
     combined_result.validation_timestamp = timestamp_ms;
     
     // Combined validation fails if any individual validation fails
     if (!voltage_result.is_valid || !temp_result.is_valid || !adc_result.is_valid) {
         combined_result.is_valid = false;
         combined_result.error_code = ValidationError::INCONSISTENT_DATA;
         combined_result.confidence_score = 0.0f;
         combined_result.quality_score = 0;
     } else {
         combined_result.is_valid = true;
         combined_result.error_code = ValidationError::NONE;
         combined_result.quality_score = calculateOverallQuality(voltage_result, temp_result, adc_result);
         combined_result.confidence_score = combined_result.quality_score / 100.0f;
     }
     
     return combined_result;
 }
 
 void DataValidator::updateHistory(uint32_t timestamp_ms) {
     // This will be called after validation to store the data
 }
 
 void DataValidator::clearHistory() {
     history_count_ = 0;
     history_head_ = 0;
 }
 
 uint8_t DataValidator::calculateOverallQuality(
     const ValidationResult& voltage_result,
     const ValidationResult& temp_result,
     const ValidationResult& adc_result) const {
     
     uint8_t avg_quality = (voltage_result.quality_score + 
                           temp_result.quality_score + 
                           adc_result.quality_score) / 3;
     
     return avg_quality;
 }
 
 bool DataValidator::isOutlier(uint16_t value, const std::array<uint16_t, 5>& recent_values) const {
     // Simple outlier detection - check if value is significantly different from recent values
     uint32_t sum = 0;
     uint8_t count = 0;
     
     for (size_t i = 0; i < 5; ++i) {
         if (recent_values[i] != 0) {
             sum += recent_values[i];
             count++;
         }
     }
     
     if (count == 0) return false;
     
     float mean = static_cast<float>(sum) / count;
     float deviation = std::abs(static_cast<float>(value) - mean);
     float threshold = mean * (config_.max_voltage_deviation_percent / 100.0f);
     
     return deviation > threshold;
 }
 
 bool DataValidator::isOutlier(float value, const std::array<float, 5>& recent_values) const {
     // Similar outlier detection for temperatures
     float sum = 0.0f;
     uint8_t count = 0;
     
     for (size_t i = 0; i < 5; ++i) {
         if (recent_values[i] != 0.0f) {
             sum += recent_values[i];
             count++;
         }
     }
     
     if (count == 0) return false;
     
     float mean = sum / count;
     float deviation = std::abs(value - mean);
     float threshold = mean * 0.1f; // 10% threshold for temperature
     
     return deviation > threshold;
 }
 
 float DataValidator::calculateConfidence(const ValidationResult& result) const {
     // Base confidence on validation success and quality score
     if (!result.is_valid) {
         return 0.0f;
     }
     
     return static_cast<float>(result.quality_score) / 100.0f;
 }
 
 bool DataValidator::isStuckValue(uint16_t value, uint16_t previous_value) const {
     return value == previous_value;
 }
 
 bool DataValidator::isRateTooHigh(float current, float previous, uint32_t time_diff_ms) const {
     if (time_diff_ms == 0) return false;
     
     float rate = std::abs(current - previous) / (time_diff_ms / 1000.0f);
     return rate > config_.max_temp_rate_C_per_sec;
 }
 
 void DataValidator::addToHistory(const std::array<uint16_t, 5>& voltages_mV,
                                const std::array<float, 5>& temperatures_C,
                                const std::array<uint16_t, 5>& adc_values,
                                uint32_t timestamp_ms, bool valid) {
     
     history_[history_head_] = {voltages_mV, temperatures_C, adc_values, timestamp_ms, valid};
     history_head_ = (history_head_ + 1) % HISTORY_SIZE;
     if (history_count_ < HISTORY_SIZE) {
         history_count_++;
     }
 }

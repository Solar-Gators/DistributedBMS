/*
 * DataValidator.hpp
 *
 * Created on: Oct 27, 2025
 *      Author: samrb
 */

 #ifndef INC_DATAVALIDATOR_HPP_
 #define INC_DATAVALIDATOR_HPP_

 #pragma once
 #include <cstdint>
 #include <array>
 #include <cmath>

 class DataValidator {
 public:
    enum class ValidationError : uint8_t {
        NONE = 0,
        OUT_OF_RANGE = 1,
        OUTLIER_DETECTED = 2,
        NOISE_EXCESSIVE = 3,
        RATE_TOO_HIGH = 4,
        STUCK_VALUE = 5,
        INCONSISTENT_DATA = 6,
        SENSOR_FAILURE = 7,
        CALIBRATION_ERROR = 8
    };

    struct ValidationResult {
        bool is_valid;
        ValidationError error_code;
        float confidence_score;  // 0.0-1.0
        uint32_t validation_timestamp;
        uint8_t quality_score;  // 0-100
        
        ValidationResult() : is_valid(true), error_code(ValidationError::NONE), 
                           confidence_score(1.0f), validation_timestamp(0), quality_score(100) {}
    };

    struct ValidationConfig {
        // Voltage limits (mV)
        uint16_t min_cell_voltage_mV = 2000;  // 2.0V
        uint16_t max_cell_voltage_mV = 4500;  // 4.5V
        float max_voltage_deviation_percent = 10.0f;
        
        // Temperature limits (°C)
        float min_temperature_C = -40.0f;
        float max_temperature_C = 85.0f;
        float max_temp_rate_C_per_sec = 5.0f;
        
        // ADC limits
        uint16_t min_adc_value = 100;      // Avoid dead zones
        uint16_t max_adc_value = 3995;     // Avoid saturation
        uint16_t adc_noise_threshold = 50; // Max jitter
        
        // Outlier detection
        uint8_t outlier_window_size = 5;
        float outlier_threshold_sigma = 2.0f;
        
        // Quality thresholds
        uint8_t min_quality_score = 70;
        float min_confidence_score = 0.8f;
    };

    DataValidator();
    explicit DataValidator(const ValidationConfig& config);

    void setConfig(const ValidationConfig& config);
    const ValidationConfig& getConfig() const;

        // Cell voltage validation
    ValidationResult validateCellVoltages(const std::array<uint16_t, 5>& voltages_mV, 
        uint32_t timestamp_ms);

    // Temperature validation
    ValidationResult validateTemperatures(const std::array<float, 5>& temperatures_C, 
                uint32_t timestamp_ms);

    // ADC validation
    ValidationResult validateADCReadings(const std::array<uint16_t, 5>& adc_values, 
            uint32_t timestamp_ms);

    // Combined validation
    ValidationResult validateAllData(const std::array<uint16_t, 5>& voltages_mV,
        const std::array<float, 5>& temperatures_C,
        const std::array<uint16_t, 5>& adc_values,
        uint32_t timestamp_ms);

    // Historical data management
    void updateHistory(uint32_t timestamp_ms);
    void clearHistory();

    // Quality assessment
    uint8_t calculateOverallQuality(const ValidationResult& voltage_result,
                                const ValidationResult& temp_result,
                                const ValidationResult& adc_result) const;

private:
    ValidationConfig config_;

    // Historical data for outlier detection
    struct HistoryEntry {
        std::array<uint16_t, 5> voltages_mV;
        std::array<float, 5> temperatures_C;
        std::array<uint16_t, 5> adc_values;
        uint32_t timestamp_ms;
        bool valid;
    };
    
    static constexpr uint8_t HISTORY_SIZE = 10;
    std::array<HistoryEntry, HISTORY_SIZE> history_;
    uint8_t history_head_;
    uint8_t history_count_;
    
    // Helper functions
    bool isOutlier(uint16_t value, const std::array<uint16_t, 5>& recent_values) const;
    bool isOutlier(float value, const std::array<float, 5>& recent_values) const;
    float calculateConfidence(const ValidationResult& result) const;
    bool isStuckValue(uint16_t value, uint16_t previous_value) const;
    bool isRateTooHigh(float current, float previous, uint32_t time_diff_ms) const;
    
    void addToHistory(const std::array<uint16_t, 5>& voltages_mV,
                     const std::array<float, 5>& temperatures_C,
                     const std::array<uint16_t, 5>& adc_values,
                     uint32_t timestamp_ms, bool valid);
};

#endif /* INC_DATAVALIDATOR_HPP_ */
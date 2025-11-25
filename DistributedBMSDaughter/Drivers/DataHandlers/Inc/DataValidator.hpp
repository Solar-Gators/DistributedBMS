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

enum {
	Valid = 0,
	To_High = 1,
	To_Low = 2
};
 class DataValidator {
 public:


    struct ValidationConfig {
        // Voltage limits (mV)
        uint16_t min_cell_voltage_mV = 2000;  // 2.0V
        uint16_t max_cell_voltage_mV = 5000;  // 4.5V
        float max_voltage_deviation_percent = 10.0f;

        // ADC limits
        uint16_t min_adc_value = 100;      // Avoid dead zones
        uint16_t max_adc_value = 3995;     // Avoid saturation

    };


    DataValidator();
    explicit DataValidator(const ValidationConfig& config);

    void setConfig(const ValidationConfig& config);
    const ValidationConfig& getConfig() const;

        // Cell voltage validation
    uint8_t validateCellVoltages(const std::array<uint16_t, 5>& voltages_mV);

    // ADC validation
    uint8_t validateADCReadings(const std::array<uint16_t, 5>& adc_values);






private:
    ValidationConfig config_;
};

#endif /* INC_DATAVALIDATOR_HPP_ */

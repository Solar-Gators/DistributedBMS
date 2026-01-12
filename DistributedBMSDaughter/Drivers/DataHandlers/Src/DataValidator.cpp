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
 {
     // Use default config
 }
 
 DataValidator::DataValidator(const ValidationConfig& config) 
     : config_(config)
 {

 }
 
 void DataValidator::setConfig(const ValidationConfig& config) {
     config_ = config;
 }
 
 const DataValidator::ValidationConfig& DataValidator::getConfig() const {
     return config_;
 }
 
uint8_t DataValidator::validateCellVoltages(
     const std::array<uint16_t, 5>& voltages_mV) {
     
     // Check range for each cell
     for (size_t i = 0; i < 5; ++i) {
         if (voltages_mV[i] < 200) continue; // Skip unused cells
         
         if (voltages_mV[i] < config_.min_cell_voltage_mV){
        	 return(To_Low);
         }
         if(voltages_mV[i] > config_.max_cell_voltage_mV) {
        	 return(To_High);
         }
     }
     return(Valid);
 }
uint8_t DataValidator::validateADCReadings(
     const std::array<uint16_t, 5>& adc_values) {
     
	  // Check range for each cell
	   for (size_t i = 0; i < 5; ++i) {

		   if (adc_values[i] < config_.min_adc_value){
			   return(To_Low);
		   }
		   if(adc_values[i] > config_.max_adc_value) {
			   return(To_High);
		   }
	   }
	   return(Valid);


 }

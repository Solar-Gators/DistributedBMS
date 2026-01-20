/*
 * BmsManager.hpp
 *
 *  Created on: Jan 2025
 *      Author: samrb
 *
 * High-level BMS management class
 * Handles data storage, fault detection, state machine, and control outputs
 */

#ifndef INC_BMSMANAGER_HPP_
#define INC_BMSMANAGER_HPP_

#pragma once

#include "PrimaryBmsFleet.hpp"
#include "ads1115.hpp"
#include "ina226.hpp"
#include "stm32l5xx_hal.h"
#include <cstdint>

class BmsManager {
public:
    // ========== Fault Types ==========
    enum class FaultType : uint16_t {
        NONE = 0,
        // Voltage faults
        OVERVOLTAGE = 1 << 0,
        UNDERVOLTAGE = 1 << 1,
        CELL_IMBALANCE = 1 << 2,
        // Temperature faults
        OVERTEMPERATURE = 1 << 3,
        UNDERTEMPERATURE = 1 << 4,
        // Current faults
        BATTERY_OVERCURRENT = 1 << 5,
        AUX_OVERCURRENT = 1 << 6,
        // Communication faults
        FLEET_DATA_STALE = 1 << 7,
        // Critical faults
        EMERGENCY_SHUTDOWN = 1 << 8,
    };

    // ========== State Machine ==========
    enum class BmsState {
        INIT,           // Initialization
        IDLE,           // No load, contactors open
        OPERATIONAL,    // Contactors closed, system operational
        FAULT,          // Fault detected, contactors open
        SHUTDOWN,       // Emergency shutdown
    };

    // ========== Configuration ==========
    struct Config {
        // Voltage thresholds
        uint16_t cell_overvoltage_mV = 4200;
        uint16_t cell_undervoltage_mV = 2500;
        uint16_t cell_imbalance_mV = 100;  // Max difference between cells

        // Temperature thresholds
        float overtemp_C = 45.0f;
        float undertemp_C = -10.0f;

        // Current thresholds (battery current from ADS1115)
        float overcurrent_A = 100.0f;  // Absolute value (charge/discharge)

        // Auxiliary current thresholds (INA226)
        float aux_overcurrent_A = 20.0f;  // Max auxiliary system current

        // Hysteresis
        uint16_t voltage_hysteresis_mV = 50;
        float temp_hysteresis_C = 2.0f;
        float current_hysteresis_A = 5.0f;

        // Timing
        uint32_t fault_recovery_time_ms = 5000;
        uint32_t data_stale_timeout_ms = 5000;

        // Current measurement (ADS1115 to current conversion)
        uint8_t current_adc_channel = 0;  // ADS1115 channel for battery current
        float current_shunt_resistance_ohm = 0.001f;  // Shunt resistance
        float current_gain = 1.0f;  // Current sense amplifier gain
        float current_offset_V = 0.0f;  // ADC offset voltage

        // Fan control
        float fan_on_temp_C = 30.0f;
        float fan_max_temp_C = 40.0f;

        // Contactor sequencing
        // Delay between first and second main contactor closing (to limit inrush on coil supply)
        uint32_t contactor_stagger_delay_ms = 50;
    };

    // ========== Initialization ==========
    BmsManager(PrimaryBmsFleet* fleet,
               ADS1115* battery_current_adc,
               INA226* aux_current_monitor);

    void init();
    void update(uint32_t now_ms);  // Main update function - call regularly

    // ========== Data Access ==========
    const FleetSummaryData& getFleetSummary() const;
    float getBatteryCurrent_A() const;      // From ADS1115 (battery current)
    float getAuxCurrent_A() const;           // From INA226 (auxiliary systems)
    float getPackVoltage_V() const;
    bool hasValidData(uint32_t now_ms) const;

    // ========== Fault Management ==========
    uint16_t getActiveFaults() const;
    bool hasFault(FaultType fault) const;
    bool hasCriticalFault() const;
    const char* getFaultName(FaultType fault) const;

    // ========== State Machine ==========
    BmsState getState() const;
    const char* getStateName() const;
    bool canTransitionTo(BmsState new_state) const;

    // ========== Control ==========
    void requestContactorsClose();  // Request contactors to close
    void requestContactorsOpen();   // Request contactors to open
    void requestShutdown();         // Emergency shutdown
    void clearFaults();             // Clear non-critical faults

    // ========== Status ==========
    bool areContactorsClosed() const;
    uint8_t getFanSpeed() const;  // 0-100% (PWM duty cycle)
    float getStateOfCharge() const;  // 0-100% (if implemented, currently returns 0)

    // ========== Hardware Control ==========
    void setContactorGpio(GPIO_TypeDef* port, uint16_t pin);              // First main contactor (e.g. negative side)
    void setSecondContactorGpio(GPIO_TypeDef* port, uint16_t pin);        // Second main contactor (e.g. positive side)
    void setFanPwmTimer(TIM_HandleTypeDef* htim, uint32_t channel);  // Set PWM timer for fans

    // ========== Configuration ==========
    void setConfig(const Config& config);
    const Config& getConfig() const;

private:
    // Internal update methods
    void updateFaults(uint32_t now_ms);
    void updateStateMachine(uint32_t now_ms);
    void updateContactors(uint32_t now_ms);  // Controls GPIO pins
    void updateFans(uint32_t now_ms);        // Controls PWM duty cycle
    void updateCurrentMeasurements(uint32_t now_ms);

    // State machine handlers
    void enterState(BmsState new_state, uint32_t now_ms);
    void exitState(BmsState old_state);
    void processState(BmsState state, uint32_t now_ms);

    // Fault detection
    bool checkOvervoltage();
    bool checkUndervoltage();
    bool checkCellImbalance();
    bool checkOvertemperature();
    bool checkUndertemperature();
    bool checkBatteryOvercurrent();  // From ADS1115
    bool checkAuxOvercurrent();      // From INA226
    bool checkDataStale(uint32_t now_ms);

    // Current measurement
    float convertAdcToCurrent(float adc_voltage_V);  // Convert ADS1115 reading to current

    // Hardware control helpers
    void setContactorGpioState(bool closed);      // Set GPIO pin for first main contactor
    void setSecondContactorGpioState(bool closed);// Set GPIO pin for second main contactor
    void setFanPwmDuty(uint8_t percent);     // Set PWM duty cycle (0-100%)

    // Safety checks
    bool canCloseContactors() const;  // Check all safety conditions
    void emergencyShutdown();

    // Member variables
    PrimaryBmsFleet* fleet_;
    ADS1115* battery_current_adc_;
    INA226* aux_current_monitor_;

    // Hardware control interfaces
    GPIO_TypeDef* contactor_gpio_port_;
    uint16_t contactor_gpio_pin_;
    bool contactor_gpio_active_high_;  // true if GPIO high = contactor closed
    GPIO_TypeDef* second_contactor_gpio_port_;
    uint16_t second_contactor_gpio_pin_;
    bool second_contactor_gpio_active_high_;
    TIM_HandleTypeDef* fan_pwm_tim_;
    uint32_t fan_pwm_channel_;
    bool fan_pwm_initialized_;

    Config config_;
    BmsState state_;
    uint16_t active_faults_;

    // State machine timing
    uint32_t state_entry_time_ms_;
    uint32_t fault_recovery_start_time_ms_;

    // Current measurements
    float battery_current_A_;
    float aux_current_A_;
    float pack_voltage_V_;
    uint32_t last_current_update_ms_;

    // Contactor state
    bool contactors_closed_;
    bool contactor_close_request_;
    bool contactor_open_request_;
    uint32_t contactor_stage_start_time_ms_;
    bool contactor_stage_active_;

    // Fan control
    uint8_t fan_speed_percent_;  // 0-100% PWM duty cycle
};

#endif /* INC_BMSMANAGER_HPP_ */


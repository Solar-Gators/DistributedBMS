#pragma once

#include "FleetSummary.hpp"
#include "stm32g4xx_hal.h"

#include <cstdint>

class ADS1115;
class BmsFleet;
class INA226;

#define PWM_PERIOD 639
#define FAN_ON 25.0f
#define FAN_MAX 40.0f

class BmsManager {
public:
    enum class FaultType : uint16_t {
        NONE = 0,
        OVERVOLTAGE = 1 << 0,
        UNDERVOLTAGE = 1 << 1,
        CELL_IMBALANCE = 1 << 2,
        OVERTEMPERATURE = 1 << 3,
        UNDERTEMPERATURE = 1 << 4,
        BATTERY_OVERCURRENT = 1 << 5,
        AUX_OVERCURRENT = 1 << 6,
        FLEET_DATA_STALE = 1 << 7,
        EMERGENCY_SHUTDOWN = 1 << 8,
    };

    enum class BmsState {
        INIT,
        IDLE,
        OPERATIONAL,
        FAULT,
        SHUTDOWN,
    };

    struct Config {
        uint16_t cell_overvoltage_mV = 4200;
        uint16_t cell_undervoltage_mV = 2500;
        uint16_t cell_imbalance_mV = 400;

        float overtemp_C = 45.0f;
        float undertemp_C = -10.0f;

        float overcurrent_A = 100.0f;
        float aux_overcurrent_A = 20.0f;

        uint16_t voltage_hysteresis_mV = 50;
        float temp_hysteresis_C = 2.0f;
        float current_hysteresis_A = 5.0f;

        uint32_t fault_recovery_time_ms = 5000;
        uint32_t data_stale_timeout_ms = 5000;

        uint8_t current_adc_channel = 0;
        float current_shunt_resistance_ohm = 0.001f;
        float current_gain = 1.0f;
        float current_offset_V = 0.0f;

        uint32_t contactor_stagger_delay_ms = 500;
        uint32_t contactor_close_grace_period_ms = 500;

        struct DebugMode {
            bool enabled = false;
            bool force_contactors_closed = false;
            bool disable_fault_detection = false;
        } debug_mode;
    };

    BmsManager(BmsFleet* fleet, ADS1115* battery_current_adc, INA226* aux_current_monitor);

    void init();
    void update(uint32_t now_ms);

    const FleetSummaryData& getFleetSummary() const;
    float getBatteryCurrent_A() const;
    float getAuxCurrent_A() const;
    float getPackVoltage_V() const;
    bool hasValidData(uint32_t now_ms) const;

    uint16_t getActiveFaults() const;
    bool hasFault(FaultType fault) const;
    bool hasCriticalFault() const;
    const char* getFaultName(FaultType fault) const;

    BmsState getState() const;
    const char* getStateName() const;
    bool canTransitionTo(BmsState new_state) const;

    void requestContactorsClose();
    void requestContactorsOpen();
    void requestShutdown();
    void clearFaults();

    bool areContactorsClosed() const;
    uint8_t getFanSpeed() const;
    /** Bit N = 1 if module N has valid recent telemetry (daughter CAN). */
    uint8_t getDaughterBoardStatusBitmap(uint32_t now_ms) const;
    float getStateOfCharge() const;

    void setContactorGpio(GPIO_TypeDef* port, uint16_t pin);
    void setSecondContactorGpio(GPIO_TypeDef* port, uint16_t pin);
    void setFanPwmTimer(TIM_HandleTypeDef* tim, uint32_t channel);

    void setConfig(const Config& config);
    const Config& getConfig() const;

    void setDebugMode(bool enabled, bool force_contactors = false, bool disable_faults = false);
    bool isDebugModeEnabled() const;

private:
    void updateFaults(uint32_t now_ms);
    void updateStateMachine(uint32_t now_ms);
    void updateContactors(uint32_t now_ms);
    void updateFans(uint32_t now_ms);
    void updateCurrentMeasurements(uint32_t now_ms);

    void enterState(BmsState new_state, uint32_t now_ms);
    void exitState(BmsState old_state);
    void processState(BmsState state, uint32_t now_ms);

    bool checkOvervoltage();
    bool checkUndervoltage();
    bool checkCellImbalance();
    bool checkOvertemperature();
    bool checkUndertemperature();
    bool checkBatteryOvercurrent();
    bool checkAuxOvercurrent();
    bool checkDataStale(uint32_t now_ms);

    float convertAdcToCurrent(float adc_voltage_V);

    void setContactorGpioState(bool closed);
    void setSecondContactorGpioState(bool closed);
    void setFanPwmDuty(uint8_t percent);

    bool canCloseContactors() const;
    void emergencyShutdown();

    BmsFleet* fleet_;
    ADS1115* battery_current_adc_;
    INA226* aux_current_monitor_;

    GPIO_TypeDef* contactor_gpio_port_;
    uint16_t contactor_gpio_pin_;
    bool contactor_gpio_active_high_;
    GPIO_TypeDef* second_contactor_gpio_port_;
    uint16_t second_contactor_gpio_pin_;
    bool second_contactor_gpio_active_high_;
    TIM_HandleTypeDef* fan_pwm_tim_;
    uint32_t fan_pwm_channel_;
    bool fan_pwm_initialized_;

    Config config_;
    BmsState state_;
    uint16_t active_faults_;

    uint32_t state_entry_time_ms_;
    uint32_t fault_recovery_start_time_ms_;

    float battery_current_A_;
    float aux_current_A_;
    float pack_voltage_V_;
    uint32_t last_current_update_ms_;

    bool contactors_closed_;
    bool contactor_close_request_;
    bool contactor_open_request_;
    uint32_t contactor_stage_start_time_ms_;
    bool contactor_stage_active_;
    uint32_t contactor_close_time_ms_;

    uint8_t fan_speed_percent_;
};

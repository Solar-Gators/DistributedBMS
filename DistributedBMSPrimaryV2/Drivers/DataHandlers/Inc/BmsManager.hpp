#pragma once

#include "FleetSummary.hpp"
#include "PrimaryV2Contract.hpp"
#include "SocEstimator.hpp"
#include "ina226.hpp"
#include "cmsis_os.h"
#include "stm32g4xx_hal.h"

#include <cstdint>

class ADS1115;
class BmsFleet;

#define PWM_PERIOD 639

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
        uint16_t cell_imbalance_mV = 410;

        float overtemp_C = 60.0f;
        float undertemp_C = -10.0f;

        /**
         * Pack OC (abs). FSGP: charge 40 A, discharge 480 A.
         * Race: 1 turn → CH2 FS ±200 A; trip at FSGP charge 40 A (480 A not measurable).
         * Bench ASC wraps: temporarily raise current_sensor_turns to match wraps.
         */
        float overcurrent_A = 40.0f;
        float aux_overcurrent_A = 50.0f;
        bool enable_aux_overcurrent_fault = false;

        uint16_t voltage_hysteresis_mV = 50;
        float temp_hysteresis_C = 2.0f;
        float current_hysteresis_A = 1.0f;

        uint32_t fault_recovery_time_ms = 5000;
        uint32_t data_stale_timeout_ms = PrimaryV2Contract::DAUGHTER_STALE_TIMEOUT_MS;
        uint32_t ads1115_read_period_ms = PrimaryV2Contract::ADS1115_READ_PERIOD_MS;
        uint32_t ina226_read_period_ms = PrimaryV2Contract::INA226_READ_PERIOD_MS;

        /**
         * LEM DHAB S/134 → ADS1115 (race: 1 turn through aperture):
         *   AIN0 = CH1 low  (±50 A @ 40 mV/A)
         *   AIN1 = CH2 high (±200 A @ 10 mV/A)
         *   V0 = 2.5 V @ UC = 5 V
         *   I_pack = (Vadc - V0) / (G_V_per_A * N_turns)
         */
        uint8_t current_adc_channel_low = 0;   /* AIN0 */
        uint8_t current_adc_channel_high = 1;  /* AIN1 */
        float current_sensitivity_low_V_per_A = 0.040f;   /* CH1 */
        float current_sensitivity_high_V_per_A = 0.010f;  /* CH2 */
        float current_offset_V = 2.5f;
        uint8_t current_sensor_turns = 1;
        /** Prefer low channel when |I_high| below this (A pack). */
        float current_range_crossover_A = 2.5f;

        /* Legacy single-channel fields (kept for older call sites). */
        uint8_t current_adc_channel = 1;
        float current_shunt_resistance_ohm = 1.0f;
        float current_gain = 0.010f;
        /**
         * INA226 shunt on +12V_BUCK (R102). Value in ohms (20 mΩ = 0.02 Ω).
         * max_current_A must not exceed INA226::fullScaleCurrent_A(R) (~4.1 A at 20 mΩ).
         */
        float aux_shunt_resistance_ohm = 0.02f;
        float aux_max_current_A = INA226::fullScaleCurrent_A(0.02f);
        /** INA226 shunt on +12V_P fan rail (R513). */
        float fan_shunt_resistance_ohm = 0.02f;
        float fan_max_current_A = INA226::fullScaleCurrent_A(0.02f);
        float aux_current_offset_A = 0.0f;

        uint32_t contactor_stagger_delay_ms = 500;
        uint32_t contactor_close_grace_period_ms = 500;

        /** Fan PWM: 0% at fan_on_temp_C, linear to 100% at fan_max_temp_C. */
        float fan_on_temp_C = 25.0f;
        float fan_max_temp_C = 40.0f;

        struct DebugMode {
            bool enabled = false;
            bool force_contactors_closed = false;
            bool disable_fault_detection = false;
        } debug_mode;
    };

    BmsManager(BmsFleet* fleet,
               ADS1115* battery_current_adc,
               INA226* aux_current_monitor,
               INA226* fan_current_monitor = nullptr);

    void init();
    void update(uint32_t now_ms);

    /** Thread-safe snapshot (copies under fleet mutex when configured). */
    FleetSummaryData getFleetSummary() const;
    float getBatteryCurrent_A() const;
    float getPackCurrentAdc_V() const { return pack_adc_V_; }
    float getPackCurrentAdcLow_V() const { return pack_adc_low_V_; }
    float getPackCurrentAdcHigh_V() const { return pack_adc_high_V_; }
    float getAuxCurrent_A() const;
    float getFanCurrent_A() const;
    float getAuxShuntVoltage_V() const { return aux_shunt_V_; }
    float getAuxBusVoltage_V() const { return aux_bus_V_; }
    float getFanShuntVoltage_V() const { return fan_shunt_V_; }
    float getFanBusVoltage_V() const { return fan_bus_V_; }
    uint32_t getAuxCurrentReadOkCount() const { return aux_read_ok_count_; }
    uint32_t getAuxCurrentReadFailCount() const { return aux_read_fail_count_; }
    uint32_t getAuxCurrentLastHalStatus() const { return aux_last_hal_status_; }
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

    /** Protects BmsFleet reads/writes across CAN RX, fleet aggregate, and manager/vehicle tasks. */
    void setFleetAccessMutex(osMutexId_t mutex_id);
    /** Serialize I2C2 sensor access (ADS1115 + INA226). */
    void setI2cMutex(osMutexId_t mutex_id);

    void setDebugMode(bool enabled, bool force_contactors = false, bool disable_faults = false);
    bool isDebugModeEnabled() const;
    void setFanPwmDuty(uint8_t percent);
    /** When true, updateContactors() does not drive GPIO or clear contactors_closed_. */
    void setExternalContactorControl(bool enabled);
    void setReportedContactorsClosed(bool closed);

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
    float convertAdcToCurrent(float adc_voltage_V, float sensitivity_V_per_A) const;

    void lockFleet_() const;
    bool tryLockFleet_(uint32_t timeout_ticks) const;
    void unlockFleet_() const;
    void lockI2c_() const;
    void unlockI2c_() const;

    void setContactorGpioState(bool closed);
    void setSecondContactorGpioState(bool closed);


    bool canCloseContactors(uint32_t now_ms) const;
    void emergencyShutdown();

    BmsFleet* fleet_;
    osMutexId_t fleet_access_mutex_{nullptr};
    osMutexId_t i2c_mutex_{nullptr};
    ADS1115* battery_current_adc_;
    INA226* aux_current_monitor_;
    INA226* fan_current_monitor_;

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
    float fan_current_A_;
    float pack_voltage_V_;
    float pack_adc_V_;
    float pack_adc_low_V_{0.0f};
    float pack_adc_high_V_{0.0f};
    float aux_shunt_V_;
    float aux_bus_V_;
    float fan_shunt_V_;
    float fan_bus_V_;
    uint32_t aux_read_ok_count_;
    uint32_t aux_read_fail_count_;
    uint32_t aux_last_hal_status_;
    uint32_t last_battery_current_update_ms_;
    uint32_t last_aux_current_update_ms_;
    uint32_t last_fan_current_update_ms_;

    bool contactors_closed_;
    bool external_contactor_control_{false};
    bool contactor_close_request_;
    bool contactor_open_request_;
    uint32_t contactor_stage_start_time_ms_;
    bool contactor_stage_active_;
    uint32_t contactor_close_time_ms_;

    uint8_t fan_speed_percent_;

    SocEstimator soc_{};
    uint32_t last_soc_update_ms_{0};
    bool soc_initialized_{false};
};

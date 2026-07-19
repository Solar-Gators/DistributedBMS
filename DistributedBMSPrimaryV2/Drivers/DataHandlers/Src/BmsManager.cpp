/*
 * BmsManager.cpp
 *
 *  Created on: Jan 2025
 *      Author: samrb
 */

#include "BmsManager.hpp"
#include "BmsFleet.hpp"
#include "PrimaryV2Contract.hpp"
#include "ads1115.hpp"
#include "ina226.hpp"

#include "cmsis_os.h"
#include "stm32g4xx_hal_tim.h"

#include <cstdint>
#include <cmath>
#include <cstring>

// ========== Constructor ==========
BmsManager::BmsManager(BmsFleet* fleet,
                       ADS1115* battery_current_adc,
                       INA226* aux_current_monitor)
    : fleet_(fleet)
    , battery_current_adc_(battery_current_adc)
    , aux_current_monitor_(aux_current_monitor)
    , contactor_gpio_port_(nullptr)
    , contactor_gpio_pin_(0)
    , contactor_gpio_active_high_(true)  // Default: high = closed
    , second_contactor_gpio_port_(nullptr)
    , second_contactor_gpio_pin_(0)
    , second_contactor_gpio_active_high_(true)
    , fan_pwm_tim_(nullptr)
    , fan_pwm_channel_(0)
    , fan_pwm_initialized_(false)
    , state_(BmsState::INIT)
    , active_faults_(0)
    , state_entry_time_ms_(0)
    , fault_recovery_start_time_ms_(0)
    , battery_current_low(0.0f)
    , battery_current_high(0.0f)
    , battery_current_A_(0.0f)
    , aux_current_A_(0.0f)
    , pack_voltage_V_(0.0f)
    , last_battery_current_update_ms_(0)
    , last_aux_current_update_ms_(0)
    , contactors_closed_(false)
    , contactor_close_request_(false)
    , contactor_open_request_(false)
    , contactor_hold_open_(false)
    , contactor_stage_start_time_ms_(0)
    , contactor_stage_active_(false)
    , contactor_close_time_ms_(0)
    , fan_speed_percent_(0)
{
    // Config already initialized with defaults in struct
}

void BmsManager::setFleetAccessMutex(osMutexId_t mutex_id)
{
    fleet_access_mutex_ = mutex_id;
}

void BmsManager::lockFleet_() const
{
    if (fleet_access_mutex_ != nullptr) {
        (void)osMutexAcquire(fleet_access_mutex_, osWaitForever);
    }
}

void BmsManager::unlockFleet_() const
{
    if (fleet_access_mutex_ != nullptr) {
        (void)osMutexRelease(fleet_access_mutex_);
    }
}

// ========== Initialization ==========
void BmsManager::init()
{
    state_ = BmsState::INIT;
    state_entry_time_ms_ = HAL_GetTick();
    active_faults_ = 0;
    contactors_closed_ = false;
    fan_speed_percent_ = 0;
    debug_contactor_enable_ms_ =
        state_entry_time_ms_ + PrimaryV2Contract::DEBUG_CONTACTOR_STARTUP_GRACE_MS;



    if (fan_pwm_tim_ != nullptr) {
        HAL_TIM_PWM_Start(fan_pwm_tim_, fan_pwm_channel_);
        fan_pwm_initialized_ = true;
        setFanPwmDuty(50);
    }
}

// ========== Main Update Function ==========
void BmsManager::update(uint32_t now_ms)
{
    // Update current measurements
    updateCurrentMeasurements(now_ms);

    // Update fault detection
    updateFaults(now_ms);

    // Update state machine
    updateStateMachine(now_ms);

    // Update hardware outputs
    updateContactors(now_ms);
    //updateFans(now_ms);
}

// ========== Data Access ==========
FleetSummaryData BmsManager::getFleetSummary() const
{
    FleetSummaryData copy{};
    if (fleet_ == nullptr) {
        return copy;
    }
    lockFleet_();
    copy = fleet_->summary();
    unlockFleet_();
    return copy;
}

ModuleSummaryData BmsManager::getModuleSummary(uint8_t module_idx) const
{
    ModuleSummaryData copy{};
    if (fleet_ == nullptr || module_idx >= PrimaryBmsFleetCfg::MAX_MODULES) {
        return copy;
    }
    lockFleet_();
    copy = fleet_->moduleSnapshot(module_idx);
    unlockFleet_();
    return copy;
}

float BmsManager::getBatteryCurrent_A() const
{
    return battery_current_A_;
}

float BmsManager::getAuxCurrent_A() const
{
    return aux_current_A_;
}

float BmsManager::getPackVoltage_V() const
{
    return pack_voltage_V_;
}

bool BmsManager::hasValidData(uint32_t now_ms) const
{
    if (fleet_ == nullptr) {
        return false;
    }
    lockFleet_();
    const bool ok = fleet_->has_data(now_ms);
    unlockFleet_();
    return ok;
}

// ========== Fault Management ==========
uint16_t BmsManager::getActiveFaults() const
{
    return active_faults_;
}

bool BmsManager::hasFault(FaultType fault) const
{
    return (active_faults_ & static_cast<uint16_t>(fault)) != 0;
}

bool BmsManager::hasCriticalFault() const
{
    return hasFault(FaultType::EMERGENCY_SHUTDOWN);
}

const char* BmsManager::getFaultName(FaultType fault) const
{
    switch (fault) {
        case FaultType::NONE: return "NONE";
        case FaultType::OVERVOLTAGE: return "OVERVOLTAGE";
        case FaultType::UNDERVOLTAGE: return "UNDERVOLTAGE";
        case FaultType::OVERTEMPERATURE: return "OVERTEMPERATURE";
        case FaultType::CHARGE_OVERCURRENT: return "CHARGE_OVERCURRENT";
        case FaultType::DISCHARGE_OVERCURRENT: return "DISCHARGE_OVERCURRENT";
        case FaultType::FLEET_DATA_STALE: return "FLEET_DATA_STALE";
        case FaultType::EMERGENCY_SHUTDOWN: return "EMERGENCY_SHUTDOWN";
        default: return "UNKNOWN";
    }
}

// ========== State Machine ==========
BmsManager::BmsState BmsManager::getState() const
{
    return state_;
}

const char* BmsManager::getStateName() const
{
    switch (state_) {
        case BmsState::INIT: return "INIT";
        case BmsState::IDLE: return "IDLE";
        case BmsState::OPERATIONAL: return "OPERATIONAL";
        case BmsState::FAULT: return "FAULT";
        case BmsState::SHUTDOWN: return "SHUTDOWN";
        default: return "UNKNOWN";
    }
}

bool BmsManager::canTransitionTo(BmsState new_state) const
{
    // Define valid state transitions
    switch (state_) {
        case BmsState::INIT:
            return new_state == BmsState::IDLE;

        case BmsState::IDLE:
            return new_state == BmsState::OPERATIONAL ||
                   new_state == BmsState::FAULT ||
                   new_state == BmsState::SHUTDOWN;

        case BmsState::OPERATIONAL:
            return new_state == BmsState::IDLE ||
                   new_state == BmsState::FAULT ||
                   new_state == BmsState::SHUTDOWN;

        case BmsState::FAULT:
            return new_state == BmsState::IDLE ||
                   new_state == BmsState::SHUTDOWN;

        case BmsState::SHUTDOWN:
            return false;  // Shutdown is terminal until reset

        default:
            return false;
    }
}

// ========== Control ==========
void BmsManager::requestContactorsClose()
{
    contactor_close_request_ = true;
    contactor_open_request_ = false;
    contactor_hold_open_ = false;  // resume auto-close when safe
}

void BmsManager::requestContactorsOpen()
{
    contactor_open_request_ = true;
    contactor_close_request_ = false;
    contactor_hold_open_ = true;  // stay open until an explicit close
}

void BmsManager::requestShutdown()
{
    emergencyShutdown();
}

void BmsManager::clearFaults()
{
    // Clear latched sensor faults. EMERGENCY_SHUTDOWN is sticky until reboot/shutdown path.
    active_faults_ &= static_cast<uint16_t>(FaultType::EMERGENCY_SHUTDOWN);
    contactor_hold_open_ = false;  // allow auto-close after fault clear
}

// ========== Status ==========
bool BmsManager::areContactorsClosed() const
{
    return contactors_closed_;
}

uint8_t BmsManager::getFanSpeed() const
{
    return fan_speed_percent_;
}

uint8_t BmsManager::getDaughterBoardStatusBitmap(uint32_t now_ms) const
{
    uint8_t bitmap = 0;
    if (!fleet_) {
        return 0;
    }
    lockFleet_();
    const uint32_t stale_ms = config_.data_stale_timeout_ms;
    for (uint8_t i = 0; i < PrimaryBmsFleetCfg::MAX_MODULES; i++) {
        const auto& m = fleet_->moduleSnapshot(i);
        if (m.valid && (now_ms - m.last_update_ms) <= stale_ms) {
            bitmap |= (1u << i);
        }
    }
    unlockFleet_();
    return bitmap;
}

float BmsManager::getStateOfCharge() const
{
    // TODO: Implement SOC calculation if needed
    return 0.0f;
}

// ========== Hardware Control ==========
void BmsManager::setContactorGpio(GPIO_TypeDef* port, uint16_t pin)
{
    contactor_gpio_port_ = port;
    contactor_gpio_pin_ = pin;
}

void BmsManager::setSecondContactorGpio(GPIO_TypeDef* port, uint16_t pin)
{
    second_contactor_gpio_port_ = port;
    second_contactor_gpio_pin_ = pin;
}

// ========== Configuration ==========
void BmsManager::setConfig(const Config& config)
{
    config_ = config;
}

// ========== Debug Mode ==========
void BmsManager::setDebugMode(bool enabled, bool force_contactors, bool disable_faults)
{
    config_.debug_mode.enabled = enabled;
    config_.debug_mode.force_contactors_closed = force_contactors;
    config_.debug_mode.disable_fault_detection = disable_faults;
}

bool BmsManager::isDebugModeEnabled() const
{
    return config_.debug_mode.enabled;
}

const BmsManager::Config& BmsManager::getConfig() const
{
    return config_;
}

// ========== Private: Update Methods ==========
void BmsManager::updateCurrentMeasurements(uint32_t now_ms)
{
    // Rate-limit sensor reads to keep loop cadence deterministic.
    if (battery_current_adc_ != nullptr &&
        (now_ms - last_battery_current_update_ms_) >= config_.ads1115_read_period_ms) {
        float adc_voltage_V;
        float adc_voltage_V2;
        if (battery_current_adc_->readSingleEnded(config_.current_adc_channel, adc_voltage_V) == HAL_OK) {
            battery_current_low = convertAdcToCurrentLow(adc_voltage_V);
        }
        if (battery_current_adc_->readSingleEnded(1, adc_voltage_V2) == HAL_OK) {
			battery_current_high = convertAdcToCurrentHigh(adc_voltage_V2);
		}

        // Low-range sensor saturates ~50 A; switch to the high-range reading at 45 A.
        static constexpr float kCurrentCrossover_A = 45.0f;
        if (std::fabs(battery_current_low) >= kCurrentCrossover_A) {
            battery_current_A_ = battery_current_high;
        } else {
            battery_current_A_ = battery_current_low;
        }

        last_battery_current_update_ms_ = now_ms;
    }

    if (aux_current_monitor_ != nullptr &&
        (now_ms - last_aux_current_update_ms_) >= config_.ina226_read_period_ms) {
        INA226::Measurement m;
        if (aux_current_monitor_->readMeasurement(m) == HAL_OK) {
            aux_current_A_ = m.current_A;
        }
        last_aux_current_update_ms_ = now_ms;
    }

    // Update pack voltage from fleet data
    if (fleet_ != nullptr) {
        lockFleet_();
        const auto& summary = fleet_->summary();
        pack_voltage_V_ = summary.total_voltage_cV / 100.0f;
        unlockFleet_();
    }
}

void BmsManager::updateFaults(uint32_t now_ms)
{
    // Skip sensor fault detection if debug mode disables it.
    // Preserve latched EMERGENCY_SHUTDOWN so a CAN kill/shutdown still sticks.
    if (config_.debug_mode.enabled && config_.debug_mode.disable_fault_detection) {
        active_faults_ &= static_cast<uint16_t>(FaultType::EMERGENCY_SHUTDOWN);
        return;
    }

    uint16_t detected = 0;

    lockFleet_();
    const bool fleet_ok = (fleet_ != nullptr) && fleet_->has_data(now_ms);
    const bool voltage_ok = fleet_ok && hasValidCellVoltageData();
    const bool temp_ok = fleet_ok && hasValidTemperatureData();

    // Voltage / thermal faults require fresh, fully-populated daughter telemetry
    if (voltage_ok && checkOvervoltage()) {
        detected |= static_cast<uint16_t>(FaultType::OVERVOLTAGE);
    }
    if (voltage_ok && checkUndervoltage()) {
        detected |= static_cast<uint16_t>(FaultType::UNDERVOLTAGE);
    }
    if (temp_ok && checkOvertemperature()) {
        detected |= static_cast<uint16_t>(FaultType::OVERTEMPERATURE);
    }
    if (checkChargeOvercurrent()) {
        detected |= static_cast<uint16_t>(FaultType::CHARGE_OVERCURRENT);
    }
    if (checkDischargeOvercurrent()) {
        detected |= static_cast<uint16_t>(FaultType::DISCHARGE_OVERCURRENT);
    }
    if (checkDataStale(now_ms)) {
        detected |= static_cast<uint16_t>(FaultType::FLEET_DATA_STALE);
    }
    unlockFleet_();

    // Latch: once set, faults stay until clearFaults() (emergency never cleared here).
    active_faults_ |= detected;

    if (hasCriticalFault()) {
        emergencyShutdown();
    }
}

void BmsManager::updateStateMachine(uint32_t now_ms)
{
    BmsState next_state = state_;

    switch (state_) {
        case BmsState::INIT:
            // Transition to IDLE after initialization
            if ((now_ms - state_entry_time_ms_) > 100) {  // 100ms init delay
                next_state = BmsState::IDLE;
            }
            break;

        case BmsState::IDLE:
            // Auto-close when safe (no vehicle close command required).
            if (!contactor_hold_open_ && canCloseContactors()) {
                next_state = BmsState::OPERATIONAL;
            }
            if (active_faults_ != 0) {
                next_state = BmsState::FAULT;
            }
            break;

        case BmsState::OPERATIONAL:
            // Check for contactor open request
            if (contactor_open_request_) {
                next_state = BmsState::IDLE;
            }
            // Check for faults
            if (active_faults_ != 0) {
                next_state = BmsState::FAULT;
            }
            break;

        case BmsState::FAULT:
            // Check if faults cleared and recovery time elapsed
            if (active_faults_ == 0) {
                if (fault_recovery_start_time_ms_ == 0) {
                    fault_recovery_start_time_ms_ = now_ms;
                } else if ((now_ms - fault_recovery_start_time_ms_) >= config_.fault_recovery_time_ms) {
                    next_state = BmsState::IDLE;
                    fault_recovery_start_time_ms_ = 0;
                }
            } else {
                fault_recovery_start_time_ms_ = 0;  // Reset if fault returns
            }
            break;

        case BmsState::SHUTDOWN:
            // Terminal state - no transitions
            break;
    }

    // Perform state transition if needed
    if (next_state != state_ && canTransitionTo(next_state)) {
        exitState(state_);
        enterState(next_state, now_ms);
    }

    // Process current state
    processState(state_, now_ms);
}

void BmsManager::updateContactors(uint32_t now_ms)
{
    // Any latched fault opens contactors; no faults closes them.
    // Faults latch in updateFaults() and clear only via clearFaults().
    if (active_faults_ != 0) {
        setContactorGpioState(false);
        setSecondContactorGpioState(false);
        contactors_closed_ = false;
        contactor_stage_start_time_ms_ = 0;
        return;
    }

    // No faults: close first contactor now, second after the stagger delay.
    if (contactor_stage_start_time_ms_ == 0) {
        contactor_stage_start_time_ms_ = now_ms;
    }
    setContactorGpioState(true);

    if ((now_ms - contactor_stage_start_time_ms_) >= config_.contactor_stagger_delay_ms) {
        setSecondContactorGpioState(true);
        contactors_closed_ = true;
    }
}

void BmsManager::updateFans(uint32_t now_ms)
{
    (void)now_ms;  // Unused for now

    if (fleet_ == nullptr || !fan_pwm_initialized_ || fan_pwm_tim_ == nullptr) {
        return;
    }

    const auto& summary = fleet_->summary();
    float highest_temp = summary.highest_temp_C;

    // Calculate fan speed based on temperature
    uint8_t new_speed = 0;
    if (highest_temp > FAN_ON) {
        if (highest_temp >= FAN_MAX) {
            new_speed = 100;  // Max speed
        } else {
            // Linear interpolation between fan_on_temp and fan_max_temp
            float temp_range = FAN_MAX - FAN_ON;
            float temp_above_min = highest_temp - FAN_ON;
            new_speed = static_cast<uint8_t>((temp_above_min / temp_range) * 100.0f);
            if (new_speed > 100) new_speed = 100;
        }
    }

    // Update fan speed if changed
    if (new_speed != fan_speed_percent_) {
        setFanPwmDuty(new_speed);
        fan_speed_percent_ = new_speed;
    }
}

// ========== Private: State Machine Handlers ==========
void BmsManager::enterState(BmsState new_state, uint32_t now_ms)
{
    state_ = new_state;
    state_entry_time_ms_ = now_ms;

    switch (new_state) {
        case BmsState::INIT:
            // Initialization actions
            break;

        case BmsState::IDLE:
            // Contactors opened by updateContactors() while not OPERATIONAL.
            contactor_close_request_ = false;
            break;

        case BmsState::OPERATIONAL:
            contactor_open_request_ = false;
            contactor_close_request_ = false;
            break;

        case BmsState::FAULT:
            // Leaving OPERATIONAL opens contactors. Hold open until clearFaults().
            contactor_hold_open_ = true;
            contactor_close_request_ = false;
            fault_recovery_start_time_ms_ = 0;
            break;

        case BmsState::SHUTDOWN:
            contactor_open_request_ = true;
            contactor_close_request_ = false;
            contactor_hold_open_ = true;  // do not auto-reclose after kill
            break;
    }
}

void BmsManager::exitState(BmsState old_state)
{
    (void)old_state;  // Unused for now
    // Add exit actions if needed
}

void BmsManager::processState(BmsState state, uint32_t now_ms)
{
    (void)state;
    (void)now_ms;
    // Add per-state processing if needed
}

// ========== Private: Fault Detection ==========
bool BmsManager::hasValidCellVoltageData() const
{
    if (fleet_ == nullptr) {
        return false;
    }
    const auto& summary = fleet_->summary();
    // Defaults are 0 until VOLTAGE_EXTREMES arrives; reject incomplete samples.
    return summary.highest_cell_mV > 0 &&
           summary.lowest_cell_mV > 0 &&
           summary.highest_cell_mV >= summary.lowest_cell_mV;
}

bool BmsManager::hasValidTemperatureData() const
{
    if (fleet_ == nullptr) {
        return false;
    }
    // Module/fleet default highTemp is -1000 until HIGH_TEMP arrives.
    return fleet_->summary().highest_temp_C > -100.0f;
}

bool BmsManager::checkOvervoltage()
{
    const auto& summary = fleet_->summary();
    return summary.highest_cell_mV > config_.cell_overvoltage_mV;
}

bool BmsManager::checkUndervoltage()
{
    const auto& summary = fleet_->summary();
    return summary.lowest_cell_mV < config_.cell_undervoltage_mV;
}

bool BmsManager::checkOvertemperature()
{
    const auto& summary = fleet_->summary();
    return summary.highest_temp_C > config_.overtemp_C;
}

bool BmsManager::checkChargeOvercurrent()
{
    // Negative pack current = charging.
    return battery_current_A_ > config_.charge_overcurrent_A;
}

bool BmsManager::checkDischargeOvercurrent()
{
    // Positive pack current = discharging.
    return battery_current_A_ < -config_.discharge_overcurrent_A;
}

bool BmsManager::checkDataStale(uint32_t now_ms)
{
    if (fleet_ == nullptr) {
        return true;
    }
    return !fleet_->has_data(now_ms);
}

// ========== Private: Current Measurement ==========
float BmsManager::convertAdcToCurrentLow(float x)
{
    return (((((3.96652f*x
              -28.7536f)*x
              +78.1917f)*x
              -98.0655f)*x
              +92.6744f)*x
              -72.9240f);
}

float BmsManager::convertAdcToCurrentHigh(float x)
{
    return 150.63f*x - 250.41f;
}


//float BmsManager::convertAdcToCurrent(float adc_voltage_V)
//{
//    // Nonlinear calibration of battery current based on ADS1115 voltage reading.
//    // This uses the empirically derived curve from the User.cpp test code:
//    //   y = 101.4864 + (-29.84563 - 101.4864) / (1 + (x / 2.756892) ^ 2.643521)
//    // where:
//    //   x = ADC voltage (V)
//    //   y = battery current (A)
//    //
//    // Note: config_.current_* fields are currently unused by this fit.
//
//    //if(adc_voltage_V)//2.305V - charging
//	const float a = 101.4864f;
//    const float b = -29.84563f;
//    const float c = 2.756892f;
//    const float d = 2.643521f;
//
//
//
//    float x = adc_voltage_V;
//    float denom = 1.0f + std::pow(x / c, d);
//    float y = a + (b - a) / denom;
//
//    if (y > 47.25) {
//        y = 1000;
//    } else if (y < -21.33) {
//        y = -1000;
//    }
//
//    return y;
//}

// ========== Private: Hardware Control ==========
void BmsManager::setContactorGpioState(bool closed)
{
    if (contactor_gpio_port_ == nullptr) {
        return;  // GPIO not configured
    }

    GPIO_PinState pin_state;
    if (contactor_gpio_active_high_) {
        pin_state = closed ? GPIO_PIN_SET : GPIO_PIN_RESET;
    } else {
        pin_state = closed ? GPIO_PIN_RESET : GPIO_PIN_SET;
    }

    HAL_GPIO_WritePin(contactor_gpio_port_, contactor_gpio_pin_, pin_state);
}

void BmsManager::setSecondContactorGpioState(bool closed)
{
    if (second_contactor_gpio_port_ == nullptr) {
        return;
    }

    GPIO_PinState pin_state;
    if (second_contactor_gpio_active_high_) {
        pin_state = closed ? GPIO_PIN_SET : GPIO_PIN_RESET;
    } else {
        pin_state = closed ? GPIO_PIN_RESET : GPIO_PIN_SET;
    }

    HAL_GPIO_WritePin(second_contactor_gpio_port_, second_contactor_gpio_pin_, pin_state);
}

void BmsManager::setFanPwmDuty(uint8_t percent)
{
    if (!fan_pwm_initialized_ || fan_pwm_tim_ == nullptr) {
        return;
    }

    if (percent > 100) {
        percent = 100;
    }

    const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(fan_pwm_tim_);
    const uint32_t pulse = (arr * static_cast<uint32_t>(percent)) / 100U;
    __HAL_TIM_SET_COMPARE(fan_pwm_tim_, fan_pwm_channel_, pulse);
}

void BmsManager::setFanPwmTimer(TIM_HandleTypeDef* tim, uint32_t channel)
{
    fan_pwm_tim_ = tim;
    fan_pwm_channel_ = channel;
    fan_pwm_initialized_ = (tim != nullptr);
}

// ========== Private: Safety Checks ==========
bool BmsManager::canCloseContactors() const
{
    // Check all safety conditions before closing contactors
    if (active_faults_ != 0) {
        return false;  // No faults allowed
    }

    if (fleet_ == nullptr) {
        return false;
    }

    uint32_t now_ms = HAL_GetTick();
    lockFleet_();
    const bool ok = fleet_->has_data(now_ms);
    unlockFleet_();
    if (!ok) {
        return false;
    }

    return true;
}

void BmsManager::emergencyShutdown()
{
    active_faults_ |= static_cast<uint16_t>(FaultType::EMERGENCY_SHUTDOWN);
    
    if (canTransitionTo(BmsState::SHUTDOWN)) {
        exitState(state_);
        enterState(BmsState::SHUTDOWN, HAL_GetTick());
    }
}


/*
 * BmsManager.cpp
 *
 *  Created on: Jan 2025
 *      Author: samrb
 */

#include "BmsManager.hpp"
#include "BmsFleet.hpp"
#include "ads1115.hpp"
#include "ina226.hpp"

#include "cmsis_os.h"
#include "stm32g4xx_hal_tim.h"

#include <cmath>
#include <cstdint>
#include <cstring>

namespace {

bool isAdvancedPwmTimer(const TIM_HandleTypeDef* tim)
{
    return tim != nullptr && (tim->Instance == TIM1 || tim->Instance == TIM8);
}

/** TIM8 CH1/CH2 → PC6/PC7 (FAN1/FAN2 PWM). Advanced timers need MOE. */
void startFanPwmOutputs(TIM_HandleTypeDef* tim, uint32_t primary_channel)
{
    (void)HAL_TIM_PWM_Start(tim, primary_channel);
    if (tim->Instance == TIM8) {
        if (primary_channel != TIM_CHANNEL_1) {
            (void)HAL_TIM_PWM_Start(tim, TIM_CHANNEL_1);
        }
        if (primary_channel != TIM_CHANNEL_2) {
            (void)HAL_TIM_PWM_Start(tim, TIM_CHANNEL_2);
        }
        __HAL_TIM_MOE_ENABLE(tim);
    } else if (isAdvancedPwmTimer(tim)) {
        __HAL_TIM_MOE_ENABLE(tim);
    }
}

void setFanPwmCompareAll(TIM_HandleTypeDef* tim, uint32_t primary_channel, uint32_t pulse)
{
    if (tim->Instance == TIM8) {
        __HAL_TIM_SET_COMPARE(tim, TIM_CHANNEL_1, pulse);
        __HAL_TIM_SET_COMPARE(tim, TIM_CHANNEL_2, pulse);
    } else {
        __HAL_TIM_SET_COMPARE(tim, primary_channel, pulse);
    }
}

/** 0% at on_C, linear ramp to 100% at max_C (fleet highest filtered temp). */
uint8_t fanDutyPercentLinear(float temp_C, float on_C, float max_C)
{
    if (temp_C <= on_C || max_C <= on_C) {
        return 0;
    }
    if (temp_C >= max_C) {
        return 100;
    }
    const float frac = (temp_C - on_C) / (max_C - on_C);
    const float duty = frac * 100.0f;
    if (duty >= 99.5f) {
        return 100;
    }
    return static_cast<uint8_t>(duty + 0.5f);
}

bool fleetSummaryHasPlausibleVoltage(const FleetSummaryData& summary)
{
    if (!kFilterImplausibleFleetReadings) {
        return true;
    }
    return summary.highest_cell_mV >= kFleetMinPlausibleCell_mV &&
           summary.highest_cell_mV <= kFleetMaxPlausibleCell_mV &&
           summary.lowest_cell_mV >= kFleetMinPlausibleCell_mV &&
           summary.lowest_cell_mV <= kFleetMaxPlausibleCell_mV;
}

}  // namespace

// ========== Constructor ==========
BmsManager::BmsManager(BmsFleet* fleet,
                       ADS1115* battery_current_adc,
                       INA226* aux_current_monitor,
                       INA226* fan_current_monitor)
    : fleet_(fleet)
    , battery_current_adc_(battery_current_adc)
    , aux_current_monitor_(aux_current_monitor)
    , fan_current_monitor_(fan_current_monitor)
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
    , battery_current_A_(0.0f)
    , aux_current_A_(0.0f)
    , fan_current_A_(0.0f)
    , pack_voltage_V_(0.0f)
    , pack_adc_V_(0.0f)
    , aux_shunt_V_(0.0f)
    , aux_bus_V_(0.0f)
    , fan_shunt_V_(0.0f)
    , fan_bus_V_(0.0f)
    , aux_read_ok_count_(0)
    , aux_read_fail_count_(0)
    , aux_last_hal_status_(0)
    , last_battery_current_update_ms_(0)
    , last_aux_current_update_ms_(0)
    , last_fan_current_update_ms_(0)
    , contactors_closed_(false)
    , contactor_close_request_(false)
    , contactor_open_request_(false)
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

void BmsManager::setI2cMutex(osMutexId_t mutex_id)
{
    i2c_mutex_ = mutex_id;
}

void BmsManager::lockFleet_() const
{
    if (fleet_access_mutex_ != nullptr) {
        (void)osMutexAcquire(fleet_access_mutex_, osWaitForever);
    }
}

bool BmsManager::tryLockFleet_(uint32_t timeout_ticks) const
{
    if (fleet_access_mutex_ == nullptr) {
        return true;
    }
    return osMutexAcquire(fleet_access_mutex_, timeout_ticks) == osOK;
}

void BmsManager::unlockFleet_() const
{
    if (fleet_access_mutex_ != nullptr) {
        (void)osMutexRelease(fleet_access_mutex_);
    }
}

void BmsManager::lockI2c_() const
{
    if (i2c_mutex_ != nullptr) {
        (void)osMutexAcquire(i2c_mutex_, osWaitForever);
    }
}

void BmsManager::unlockI2c_() const
{
    if (i2c_mutex_ != nullptr) {
        (void)osMutexRelease(i2c_mutex_);
    }
}

// ========== Initialization ==========
void BmsManager::init()
{
    state_ = BmsState::INIT;
    state_entry_time_ms_ = 0;
    active_faults_ = 0;
    contactors_closed_ = false;
    fan_speed_percent_ = 0;



    if (fan_pwm_tim_ != nullptr) {
        startFanPwmOutputs(fan_pwm_tim_, fan_pwm_channel_);
        fan_pwm_initialized_ = true;
    }

    /* Force first sensor sample on the next update() without waiting a full period. */
    const uint32_t ina_period = config_.ina226_read_period_ms;
    const uint32_t ads_period = config_.ads1115_read_period_ms;
    last_aux_current_update_ms_ = UINT32_MAX - ina_period + 1u;
    last_fan_current_update_ms_ = UINT32_MAX - ina_period + 1u;
    last_battery_current_update_ms_ = UINT32_MAX - ads_period + 1u;
}

// ========== Main Update Function ==========
void BmsManager::update(uint32_t now_ms)
{
    if (state_ == BmsState::INIT && state_entry_time_ms_ == 0) {
        state_entry_time_ms_ = now_ms;
    }

    // Update current measurements
    updateCurrentMeasurements(now_ms);

    // Update fault detection
    updateFaults(now_ms);

    // Update state machine
    updateStateMachine(now_ms);

    // Update hardware outputs
    updateContactors(now_ms);
    updateFans(now_ms);

    /* SoC: EKF @ ~10 Hz using pack V + pack I (discharge > 0). */
    if (pack_voltage_V_ > 1.0f) {
        if (!soc_initialized_) {
            soc_.initFromPackVoltage(pack_voltage_V_);
            soc_initialized_ = true;
            last_soc_update_ms_ = now_ms;
        } else if ((now_ms - last_soc_update_ms_) >= 100u) {
            const float dt_s = static_cast<float>(now_ms - last_soc_update_ms_) * 0.001f;
            const float temp_C = getFleetSummary().highest_temp_C;
            soc_.update(pack_voltage_V_, battery_current_A_, dt_s, now_ms, temp_C);
            last_soc_update_ms_ = now_ms;
        }
    }
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

float BmsManager::getBatteryCurrent_A() const
{
    return battery_current_A_;
}

float BmsManager::getAuxCurrent_A() const
{
    return aux_current_A_;
}

float BmsManager::getFanCurrent_A() const
{
    return fan_current_A_;
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
        case FaultType::CELL_IMBALANCE: return "CELL_IMBALANCE";
        case FaultType::OVERTEMPERATURE: return "OVERTEMPERATURE";
        case FaultType::UNDERTEMPERATURE: return "UNDERTEMPERATURE";
        case FaultType::BATTERY_OVERCURRENT: return "BATTERY_OVERCURRENT";
        case FaultType::AUX_OVERCURRENT: return "AUX_OVERCURRENT";
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
}

void BmsManager::requestContactorsOpen()
{
    contactor_open_request_ = true;
    contactor_close_request_ = false;
}

void BmsManager::requestShutdown()
{
    emergencyShutdown();
}

void BmsManager::clearFaults()
{
    // Only clear non-critical faults
    active_faults_ &= ~static_cast<uint16_t>(FaultType::EMERGENCY_SHUTDOWN);
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
    return soc_.socPercent();
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
    lockI2c_();

    // Rate-limit sensor reads to keep loop cadence deterministic.
    if (battery_current_adc_ != nullptr &&
        (now_ms - last_battery_current_update_ms_) >= config_.ads1115_read_period_ms) {
        float v_low = 0.0f;
        float v_high = 0.0f;
        const HAL_StatusTypeDef st_low =
            battery_current_adc_->readSingleEnded(config_.current_adc_channel_low, v_low);
        const HAL_StatusTypeDef st_high =
            battery_current_adc_->readSingleEnded(config_.current_adc_channel_high, v_high);

        if (st_low == HAL_OK) {
            pack_adc_low_V_ = v_low;
        }
        if (st_high == HAL_OK) {
            pack_adc_high_V_ = v_high;
        }

        if (st_high == HAL_OK) {
            const float i_high =
                convertAdcToCurrent(v_high, config_.current_sensitivity_high_V_per_A);
            const float abs_high = (i_high < 0.0f) ? -i_high : i_high;

            if (st_low == HAL_OK && abs_high < config_.current_range_crossover_A) {
                battery_current_A_ =
                    convertAdcToCurrent(v_low, config_.current_sensitivity_low_V_per_A);
                pack_adc_V_ = v_low;
            } else {
                battery_current_A_ = i_high;
                pack_adc_V_ = v_high;
            }
        } else if (st_low == HAL_OK) {
            battery_current_A_ =
                convertAdcToCurrent(v_low, config_.current_sensitivity_low_V_per_A);
            pack_adc_V_ = v_low;
        }
        last_battery_current_update_ms_ = now_ms;
    }

    if (aux_current_monitor_ != nullptr &&
        (now_ms - last_aux_current_update_ms_) >= config_.ina226_read_period_ms) {
        INA226::Measurement m;
        const HAL_StatusTypeDef st = aux_current_monitor_->readMeasurement(m);
        aux_last_hal_status_ = static_cast<uint32_t>(st);
        if (st == HAL_OK) {
            aux_shunt_V_ = m.shunt_V;
            aux_bus_V_ = m.bus_V;
            aux_current_A_ = m.current_A - config_.aux_current_offset_A;
            ++aux_read_ok_count_;
        } else {
            ++aux_read_fail_count_;
        }
        last_aux_current_update_ms_ = now_ms;
    }

    if (fan_current_monitor_ != nullptr &&
        (now_ms - last_fan_current_update_ms_) >= config_.ina226_read_period_ms) {
        INA226::Measurement m;
        if (fan_current_monitor_->readMeasurement(m) == HAL_OK) {
            fan_shunt_V_ = m.shunt_V;
            fan_bus_V_ = m.bus_V;
            fan_current_A_ = m.current_A;
        }
        last_fan_current_update_ms_ = now_ms;
    }

    unlockI2c_();

    // Update pack voltage from fleet data
    if (fleet_ != nullptr) {
        lockFleet_();
        const auto& summary = fleet_->summary();
        pack_voltage_V_ = summary.total_voltage_mV / 1000.0f;
        unlockFleet_();
    }
}

void BmsManager::updateFaults(uint32_t now_ms)
{
    // Skip fault detection if debug mode disables it
    if (config_.debug_mode.enabled && config_.debug_mode.disable_fault_detection) {
        active_faults_ = 0;
        return;
    }

    uint16_t new_faults = 0;

    if (!tryLockFleet_(1u)) {
        return;
    }
    const bool fleet_ok = (fleet_ != nullptr) && fleet_->has_data(now_ms);

    // Voltage / thermal / imbalance faults require fresh daughter telemetry
    if (fleet_ok && checkOvervoltage()) {
        new_faults |= static_cast<uint16_t>(FaultType::OVERVOLTAGE);
    }
    if (fleet_ok && checkUndervoltage()) {
        new_faults |= static_cast<uint16_t>(FaultType::UNDERVOLTAGE);
    }
    if (fleet_ok && checkCellImbalance()) {
        new_faults |= static_cast<uint16_t>(FaultType::CELL_IMBALANCE);
    }
    if (fleet_ok && checkOvertemperature()) {
        new_faults |= static_cast<uint16_t>(FaultType::OVERTEMPERATURE);
    }
    if (fleet_ok && checkUndertemperature()) {
        new_faults |= static_cast<uint16_t>(FaultType::UNDERTEMPERATURE);
    }
    if (checkBatteryOvercurrent()) {
        new_faults |= static_cast<uint16_t>(FaultType::BATTERY_OVERCURRENT);
    }
    if (config_.enable_aux_overcurrent_fault && checkAuxOvercurrent()) {
        new_faults |= static_cast<uint16_t>(FaultType::AUX_OVERCURRENT);
    }
    if (checkDataStale(now_ms)) {
        new_faults |= static_cast<uint16_t>(FaultType::FLEET_DATA_STALE);
    }
    unlockFleet_();

    // Update active faults
    active_faults_ = new_faults;

    // If critical fault, trigger emergency shutdown
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
            /* Auto-close only when faults are clear AND live daughter data exists.
             * Stay IDLE (contactors open) until fleet is ready — avoids boot chatter. */
            if (canCloseContactors(now_ms)) {
                next_state = BmsState::OPERATIONAL;
            } else if (active_faults_ != 0) {
                next_state = BmsState::FAULT;
            }
            break;

        case BmsState::OPERATIONAL:
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
    if (external_contactor_control_) {
        return;
    }

    // Debug mode: Force contactors closed (bypasses all safety checks)
    if (config_.debug_mode.enabled && config_.debug_mode.force_contactors_closed) {
        if (!contactors_closed_) {
            // Close both contactors immediately (no sequencing in debug mode)
            if (contactor_gpio_port_ != nullptr) {
                setContactorGpioState(true);
            }
            if (second_contactor_gpio_port_ != nullptr) {
                setSecondContactorGpioState(true);
            } else if (contactor_gpio_port_ != nullptr) {
                // Only one contactor configured
            }
            contactors_closed_ = true;
            contactor_close_time_ms_ = now_ms;
            contactor_stage_active_ = false;
            contactor_stage_start_time_ms_ = 0;
        }
        return;  // Debug mode bypasses all normal logic
    }

    // Default: contactors closed only in OPERATIONAL with close permitted
    bool target_closed = (state_ == BmsState::OPERATIONAL) && canCloseContactors(now_ms);

    // Calculate time since contactors were closed (for grace period)
    uint32_t time_since_close = contactors_closed_ ? (now_ms - contactor_close_time_ms_) : UINT32_MAX;
    bool in_grace_period = (time_since_close < config_.contactor_close_grace_period_ms);

    // If we are not in OPERATIONAL (or we have an explicit open request),
    // force everything open and reset sequencing state.
    // Allow grace period after closing to prevent immediate reopening due to inrush/transients
    if (!target_closed || contactor_open_request_ || 
        (active_faults_ != 0 && !in_grace_period)) {
        setContactorGpioState(false);
        setSecondContactorGpioState(false);
        contactors_closed_ = false;
        contactor_stage_active_ = false;
        contactor_stage_start_time_ms_ = 0;
        contactor_close_time_ms_ = 0;
        return;
    }

    // At this point, we want contactors closed and we're in OPERATIONAL state.
    // If already closed, nothing to do.
    if (contactors_closed_) {
        return;
    }

    // If only one contactor GPIO is configured, close it immediately.
    if (second_contactor_gpio_port_ == nullptr) {
        setContactorGpioState(true);
        contactors_closed_ = true;
        contactor_close_time_ms_ = now_ms;  // Record when contactors were closed
        return;
    }

    // Two-main-contactor sequencing (e.g. negative then positive):
    // 1) Close first contactor (contactor_gpio_*).
    // 2) After contactor_stagger_delay_ms, close second contactor and keep both closed.

    if (!contactor_stage_active_) {
        // Start stage 1: close first contactor
        setContactorGpioState(true);
        contactor_stage_active_ = true;
        contactor_stage_start_time_ms_ = now_ms;
        return;
    }

    // Stage already active - check if delay has elapsed
    uint32_t elapsed = now_ms - contactor_stage_start_time_ms_;
    if (elapsed >= config_.contactor_stagger_delay_ms) {
        // Close second contactor; both remain closed
        setSecondContactorGpioState(true);
        contactors_closed_ = true;
        contactor_close_time_ms_ = now_ms;  // Record when contactors were fully closed

        contactor_stage_active_ = false;
        contactor_stage_start_time_ms_ = 0;
    }
}

void BmsManager::updateFans(uint32_t now_ms)
{
    (void)now_ms;

    if (fleet_ == nullptr || !fan_pwm_initialized_ || fan_pwm_tim_ == nullptr) {
        return;
    }

    lockFleet_();
    const auto& summary = fleet_->summary();
    const float highest_temp = summary.highest_temp_C;
    const bool fleet_fresh = summary.is_online(now_ms, config_.data_stale_timeout_ms);
    unlockFleet_();

    if (!fleet_fresh) {
        if (fan_speed_percent_ != 0) {
            setFanPwmDuty(0);
        }
        return;
    }

    const uint8_t new_speed = fanDutyPercentLinear(
        highest_temp, config_.fan_on_temp_C, config_.fan_max_temp_C);

    if (new_speed != fan_speed_percent_) {
        setFanPwmDuty(new_speed);
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
            contactor_open_request_ = false;
            contactor_close_request_ = false;
            break;

        case BmsState::OPERATIONAL:
            // Contactors will be closed by updateContactors()
            break;

        case BmsState::FAULT:
            // Open contactors immediately
            contactor_open_request_ = true;
            contactor_close_request_ = false;
            fault_recovery_start_time_ms_ = 0;
            break;

        case BmsState::SHUTDOWN:
            // Emergency shutdown actions
            contactor_open_request_ = true;
            contactor_close_request_ = false;
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
bool BmsManager::checkOvervoltage()
{
    const auto& summary = fleet_->summary();
    if (!fleetSummaryHasPlausibleVoltage(summary)) {
        return false;
    }
    uint16_t threshold = config_.cell_overvoltage_mV;
    uint16_t clear_threshold = threshold - config_.voltage_hysteresis_mV;

    if (summary.highest_cell_mV > threshold) {
        return true;
    } else if (summary.highest_cell_mV < clear_threshold) {
        return false;
    }
    // Hysteresis: keep previous state if in between
    return hasFault(FaultType::OVERVOLTAGE);
}

bool BmsManager::checkUndervoltage()
{
    const auto& summary = fleet_->summary();
    if (!fleetSummaryHasPlausibleVoltage(summary)) {
        return false;
    }
    uint16_t threshold = config_.cell_undervoltage_mV;
    uint16_t clear_threshold = threshold + config_.voltage_hysteresis_mV;

    if (summary.lowest_cell_mV < threshold) {
        return true;
    } else if (summary.lowest_cell_mV > clear_threshold) {
        return false;
    }
    // Hysteresis: keep previous state if in between
    return hasFault(FaultType::UNDERVOLTAGE);
}

bool BmsManager::checkCellImbalance()
{
    const auto& summary = fleet_->summary();
    if (!fleetSummaryHasPlausibleVoltage(summary)) {
        return false;
    }
    uint16_t imbalance = summary.highest_cell_mV - summary.lowest_cell_mV;
    uint16_t threshold = config_.cell_imbalance_mV;

    return imbalance > threshold;
}

bool BmsManager::checkOvertemperature()
{
    const auto& summary = fleet_->summary();
    if (kFilterImplausibleFleetReadings &&
        (!std::isfinite(summary.highest_temp_C) ||
         summary.highest_temp_C < kFleetMinPlausibleTemp_C ||
         summary.highest_temp_C > kFleetMaxPlausibleTemp_C)) {
        return false;
    }
    float threshold = config_.overtemp_C;
    float clear_threshold = threshold - config_.temp_hysteresis_C;

    if (summary.highest_temp_C > threshold) {
        return true;
    } else if (summary.highest_temp_C < clear_threshold) {
        return false;
    }
    // Hysteresis: keep previous state if in between
    return hasFault(FaultType::OVERTEMPERATURE);
}

bool BmsManager::checkUndertemperature()
{
    const auto& summary = fleet_->summary();
    if (kFilterImplausibleFleetReadings &&
        (!std::isfinite(summary.highest_temp_C) ||
         summary.highest_temp_C < kFleetMinPlausibleTemp_C ||
         summary.highest_temp_C > kFleetMaxPlausibleTemp_C)) {
        return false;
    }
    if (!std::isfinite(summary.highest_temp_C) || summary.highest_temp_C < -50.0f) {
        return false;
    }
    float threshold = config_.undertemp_C;
    float clear_threshold = threshold + config_.temp_hysteresis_C;

    if (summary.highest_temp_C < threshold) {
        return true;
    } else if (summary.highest_temp_C > clear_threshold) {
        return false;
    }
    // Hysteresis: keep previous state if in between
    return hasFault(FaultType::UNDERTEMPERATURE);
}

bool BmsManager::checkBatteryOvercurrent()
{
    float abs_current = (battery_current_A_ < 0) ? -battery_current_A_ : battery_current_A_;
    float threshold = config_.overcurrent_A;
    float clear_threshold = threshold - config_.current_hysteresis_A;

    if (abs_current > threshold) {
        return true;
    } else if (abs_current < clear_threshold) {
        return false;
    }
    // Hysteresis: keep previous state if in between
    return hasFault(FaultType::BATTERY_OVERCURRENT);
}

bool BmsManager::checkAuxOvercurrent()
{
    float abs_current = (aux_current_A_ < 0) ? -aux_current_A_ : aux_current_A_;
    float threshold = config_.aux_overcurrent_A;
    float clear_threshold = threshold - config_.current_hysteresis_A;

    if (abs_current > threshold) {
        return true;
    } else if (abs_current < clear_threshold) {
        return false;
    }
    // Hysteresis: keep previous state if in between
    return hasFault(FaultType::AUX_OVERCURRENT);
}

bool BmsManager::checkDataStale(uint32_t now_ms)
{
    if (fleet_ == nullptr) {
        return true;
    }
    return !fleet_->has_data(now_ms);
}

// ========== Private: Current Measurement ==========
float BmsManager::convertAdcToCurrent(float adc_voltage_V)
{
    return convertAdcToCurrent(adc_voltage_V, config_.current_sensitivity_high_V_per_A);
}

float BmsManager::convertAdcToCurrent(float adc_voltage_V, float sensitivity_V_per_A) const
{
    /* I_pack = (Vadc - V0) / (G * N_turns) */
    const float turns = (config_.current_sensor_turns == 0)
                            ? 1.0f
                            : static_cast<float>(config_.current_sensor_turns);
    const float denom = sensitivity_V_per_A * turns;
    if (denom <= 0.0f) {
        return 0.0f;
    }
    return (adc_voltage_V - config_.current_offset_V) / denom;
}

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
    const uint32_t pulse =
        ((arr + 1U) * static_cast<uint32_t>(percent)) / 100U;
    setFanPwmCompareAll(fan_pwm_tim_, fan_pwm_channel_, pulse);
    fan_speed_percent_ = percent;
}

void BmsManager::setExternalContactorControl(bool enabled)
{
    external_contactor_control_ = enabled;
}

void BmsManager::setReportedContactorsClosed(bool closed)
{
    contactors_closed_ = closed;
}

void BmsManager::setFanPwmTimer(TIM_HandleTypeDef* tim, uint32_t channel)
{
    fan_pwm_tim_ = tim;
    fan_pwm_channel_ = channel;
    fan_pwm_initialized_ = (tim != nullptr);
}

// ========== Private: Safety Checks ==========
bool BmsManager::canCloseContactors(uint32_t now_ms) const
{
    if (active_faults_ != 0) {
        return false;
    }
    if (fleet_ == nullptr) {
        return false;
    }
    /* Require at least one fresh daughter frame before energizing the bus. */
    return fleet_->has_data(now_ms);
}

void BmsManager::emergencyShutdown()
{
    active_faults_ |= static_cast<uint16_t>(FaultType::EMERGENCY_SHUTDOWN);
    
    if (canTransitionTo(BmsState::SHUTDOWN)) {
        exitState(state_);
        enterState(BmsState::SHUTDOWN, HAL_GetTick());
    }
}


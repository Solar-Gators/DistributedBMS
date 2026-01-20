/*
 * BmsManager.cpp
 *
 *  Created on: Jan 2025
 *      Author: samrb
 */

#include "BmsManager.hpp"
#include <cmath>
#include <cstring>

// ========== Constructor ==========
BmsManager::BmsManager(PrimaryBmsFleet* fleet,
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
    , battery_current_A_(0.0f)
    , aux_current_A_(0.0f)
    , pack_voltage_V_(0.0f)
    , last_current_update_ms_(0)
    , contactors_closed_(false)
    , contactor_close_request_(false)
    , contactor_open_request_(false)
    , contactor_stage_start_time_ms_(0)
    , contactor_stage_active_(false)
    , fan_speed_percent_(0)
{
    // Config already initialized with defaults in struct
}

// ========== Initialization ==========
void BmsManager::init()
{
    state_ = BmsState::INIT;
    state_entry_time_ms_ = HAL_GetTick();
    active_faults_ = 0;
    contactors_closed_ = false;
    fan_speed_percent_ = 0;

    // Initialize PWM if configured
    if (fan_pwm_tim_ != nullptr) {
        HAL_TIM_PWM_Start(fan_pwm_tim_, fan_pwm_channel_);
        fan_pwm_initialized_ = true;
        setFanPwmDuty(0);  // Start with fans off
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
    updateFans(now_ms);
}

// ========== Data Access ==========
const FleetSummaryData& BmsManager::getFleetSummary() const
{
    return fleet_->summary();
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
    return fleet_->has_data(now_ms);
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

void BmsManager::setFanPwmTimer(TIM_HandleTypeDef* htim, uint32_t channel)
{
    fan_pwm_tim_ = htim;
    fan_pwm_channel_ = channel;
}

// ========== Configuration ==========
void BmsManager::setConfig(const Config& config)
{
    config_ = config;
}

const BmsManager::Config& BmsManager::getConfig() const
{
    return config_;
}

// ========== Private: Update Methods ==========
void BmsManager::updateCurrentMeasurements(uint32_t now_ms)
{
    // Update battery current from ADS1115
    if (battery_current_adc_ != nullptr) {
        float adc_voltage_V;
        if (battery_current_adc_->readSingleEnded(config_.current_adc_channel, adc_voltage_V) == HAL_OK) {
            battery_current_A_ = convertAdcToCurrent(adc_voltage_V);
            last_current_update_ms_ = now_ms;
        }
    }

    // Update auxiliary current from INA226
    if (aux_current_monitor_ != nullptr) {
        INA226::Measurement m;
        if (aux_current_monitor_->readMeasurement(m) == HAL_OK) {
            aux_current_A_ = m.current_A;
        }
    }

    // Update pack voltage from fleet data
    const auto& summary = fleet_->summary();
    pack_voltage_V_ = summary.total_voltage_mV / 1000.0f;
}

void BmsManager::updateFaults(uint32_t now_ms)
{
    uint16_t new_faults = 0;

    // Check each fault condition
    if (checkOvervoltage()) {
        new_faults |= static_cast<uint16_t>(FaultType::OVERVOLTAGE);
    }
    if (checkUndervoltage()) {
        new_faults |= static_cast<uint16_t>(FaultType::UNDERVOLTAGE);
    }
    if (checkCellImbalance()) {
        new_faults |= static_cast<uint16_t>(FaultType::CELL_IMBALANCE);
    }
    if (checkOvertemperature()) {
        new_faults |= static_cast<uint16_t>(FaultType::OVERTEMPERATURE);
    }
    if (checkUndertemperature()) {
        new_faults |= static_cast<uint16_t>(FaultType::UNDERTEMPERATURE);
    }
    if (checkBatteryOvercurrent()) {
        new_faults |= static_cast<uint16_t>(FaultType::BATTERY_OVERCURRENT);
    }
    if (checkAuxOvercurrent()) {
        new_faults |= static_cast<uint16_t>(FaultType::AUX_OVERCURRENT);
    }
    if (checkDataStale(now_ms)) {
        new_faults |= static_cast<uint16_t>(FaultType::FLEET_DATA_STALE);
    }

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
            // Check for contactor close request
            if (contactor_close_request_ && canCloseContactors()) {
                next_state = BmsState::OPERATIONAL;
            }
            // Check for faults
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
    // Default: contactors should be open unless in OPERATIONAL state
    bool target_closed = (state_ == BmsState::OPERATIONAL);

    // If we are not in OPERATIONAL (or we have an explicit open request),
    // force everything open and reset sequencing state.
    if (!target_closed || contactor_open_request_ || active_faults_ != 0) {
        setContactorGpioState(false);
        setSecondContactorGpioState(false);
        contactors_closed_ = false;
        contactor_stage_active_ = false;
        contactor_stage_start_time_ms_ = 0;
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

        contactor_stage_active_ = false;
        contactor_stage_start_time_ms_ = 0;
    }
}

void BmsManager::updateFans(uint32_t now_ms)
{
    (void)now_ms;  // Unused for now

    if (!fan_pwm_initialized_ || fan_pwm_tim_ == nullptr) {
        return;
    }

    const auto& summary = fleet_->summary();
    float highest_temp = summary.highest_temp_C;

    // Calculate fan speed based on temperature
    uint8_t new_speed = 0;
    if (highest_temp > config_.fan_on_temp_C) {
        if (highest_temp >= config_.fan_max_temp_C) {
            new_speed = 100;  // Max speed
        } else {
            // Linear interpolation between fan_on_temp and fan_max_temp
            float temp_range = config_.fan_max_temp_C - config_.fan_on_temp_C;
            float temp_above_min = highest_temp - config_.fan_on_temp_C;
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
            // Ensure contactors are open
            contactor_open_request_ = true;
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
    uint16_t imbalance = summary.highest_cell_mV - summary.lowest_cell_mV;
    uint16_t threshold = config_.cell_imbalance_mV;

    return imbalance > threshold;
}

bool BmsManager::checkOvertemperature()
{
    const auto& summary = fleet_->summary();
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
    return !fleet_->has_data(now_ms);
}

// ========== Private: Current Measurement ==========
float BmsManager::convertAdcToCurrent(float adc_voltage_V)
{
    // Convert ADC voltage to current
    // Formula: I = (V_adc - V_offset) / (R_shunt * Gain)
    float voltage = adc_voltage_V - config_.current_offset_V;
    float current = voltage / (config_.current_shunt_resistance_ohm * config_.current_gain);
    return current;
}

// ========== Private: Hardware Control ==========
void BmsManager::setContactorGpioState(bool closed)
{
    if (contactor_gpio_port_ == nullptr) {
        return;
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

    if (percent > 100) percent = 100;

    // Calculate PWM compare value
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(fan_pwm_tim_);
    uint32_t compare = (period * percent) / 100;

    __HAL_TIM_SET_COMPARE(fan_pwm_tim_, fan_pwm_channel_, compare);
}

// ========== Private: Safety Checks ==========
bool BmsManager::canCloseContactors() const
{
    // Check all safety conditions before closing contactors
    if (active_faults_ != 0) {
        return false;  // No faults allowed
    }

    // Check if we have valid data
    uint32_t now_ms = HAL_GetTick();
    if (!fleet_->has_data(now_ms)) {
        return false;  // Need valid data
    }

    // Additional safety checks can be added here
    // e.g., voltage in safe range, temperature OK, etc.

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


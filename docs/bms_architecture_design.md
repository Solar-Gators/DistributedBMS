# High-Level BMS Architecture Design

**Date:** January 2025  
**Purpose:** Design a comprehensive high-level BMS class to manage data storage, fault detection, state machine, and overall BMS functionality

---

## Current Architecture Analysis

### Existing Components

1. **PrimaryBmsFleet** (`PrimaryBmsFleet.hpp/cpp`)
   - **Purpose:** Data storage and UART communication
   - **Responsibilities:**
     - Stores fleet summary data
     - Stores per-module data
     - Handles UART packet parsing
     - Manages data staleness/aging
   - **Status:** ✅ Well-structured, keep as-is

2. **BmsController** (`BmsController.hpp/cpp`)
   - **Purpose:** Basic fault detection
   - **Responsibilities:**
     - Detects overvoltage (>4200mV)
     - Detects undervoltage (<2500mV)
     - Detects overtemperature (>45°C)
     - Stores fault bitmask
   - **Status:** ⚠️ Very basic, needs expansion

3. **User.cpp Main Loop**
   - **Purpose:** Application main loop
   - **Responsibilities:**
     - Reads IMU, current monitor, ADC
     - Updates faults from fleet data
     - Controls LEDs
   - **Status:** ⚠️ Too much logic in main loop

### Gaps Identified

1. ❌ **No State Machine** - No operational/fault states
2. ❌ **No Contactor Control Logic** - Contactors not actually controlled
3. ❌ **No Battery Current Monitoring** - ADS1115 data not converted to current or used for faults
4. ❌ **No Auxiliary Current Monitoring** - INA226 data not used (monitors aux systems, not battery)
5. ❌ **No Cell Imbalance Detection** - Only min/max, no imbalance threshold
6. ❌ **No Fault Recovery Logic** - Faults set but never cleared with hysteresis
7. ❌ **No Safety Interlocks** - No checks before enabling contactors
8. ❌ **No Fan Control Integration** - PWM timers exist but fan control not implemented
9. ❌ **No Emergency Shutdown** - No critical fault handling
10. ❌ **No Data Logging** - No fault history or event logging

---

## Proposed Architecture

### High-Level BMS Class Design

```
┌─────────────────────────────────────────────────────────────┐
│                    BmsManager (New Class)                    │
│                                                               │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  Data Management                                     │    │
│  │  - Fleet data (uses PrimaryBmsFleet)                 │    │
│  │  - Current/voltage measurements                      │    │
│  │  - Historical data (optional)                        │    │
│  └─────────────────────────────────────────────────────┘    │
│                                                               │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  Fault Detection & Management                        │    │
│  │  - Voltage faults (over/under/imbalance)            │    │
│  │  - Temperature faults                               │    │
│  │  - Current faults (overcurrent)                      │    │
│  │  - Communication faults                             │    │
│  │  - Fault hysteresis & recovery                      │    │
│  └─────────────────────────────────────────────────────┘    │
│                                                               │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  State Machine                                       │    │
│  │  - IDLE, CHARGING, DISCHARGING, FAULT, SHUTDOWN     │    │
│  │  - State transitions                                 │    │
│  │  - State entry/exit actions                          │    │
│  └─────────────────────────────────────────────────────┘    │
│                                                               │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  Control Outputs                                     │    │
│  │  - Contactor control (via GPIO)                      │    │
│  │  - Fan control (via PWM)                             │    │
│  │  - LED indicators                                    │    │
│  └─────────────────────────────────────────────────────┘    │
│                                                               │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  Safety & Interlocks                                 │    │
│  │  - Safety checks before contactor close              │    │
│  │  - Emergency shutdown                                │    │
│  └─────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────┘
```

---

## Detailed Class Design

### BmsManager Class

#### Core Responsibilities

1. **Data Management**
   - Owns/uses `PrimaryBmsFleet` for battery data
   - Integrates current monitoring (INA226)
   - Tracks pack-level statistics
   - Manages data freshness

2. **Fault Detection**
   - Comprehensive fault detection with thresholds
   - Fault hysteresis (prevent oscillation)
   - Fault recovery logic
   - Fault priority/severity levels

3. **State Machine**
   - Manages BMS operational states
   - Handles state transitions
   - Executes state-specific actions

4. **Control Outputs**
   - Contactor control with safety checks
   - Fan speed control based on temperature
   - LED status indicators
   - CAN/FDCAN status messages

5. **Safety Systems**
   - Safety checks before contactor close
   - Emergency shutdown
   - Safety interlocks

---

## Class Interface Design

### Header Structure

```cpp
class BmsManager {
public:
    // ========== Initialization ==========
    BmsManager(PrimaryBmsFleet* fleet, 
               ADS1115* battery_current_adc,  // For battery current measurement
               INA226* aux_current_monitor);  // For auxiliary system current
    
    void init();
    void update(uint32_t now_ms);  // Main update function
    
    // ========== Data Access ==========
    const FleetSummaryData& getFleetSummary() const;
    float getBatteryCurrent_A() const;      // From ADS1115 (battery current)
    float getAuxCurrent_A() const;         // From INA226 (auxiliary systems)
    float getPackVoltage_V() const;
    bool hasValidData(uint32_t now_ms) const;
    
    // ========== Fault Management ==========
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
        OVERCURRENT_CHARGE = 1 << 5,
        OVERCURRENT_DISCHARGE = 1 << 6,
        // Communication faults
        FLEET_DATA_STALE = 1 << 7,
        // Critical faults
        EMERGENCY_SHUTDOWN = 1 << 8,
        // ... more as needed
    };
    
    uint16_t getActiveFaults() const;
    bool hasFault(FaultType fault) const;
    bool hasCriticalFault() const;
    const char* getFaultName(FaultType fault) const;
    
    // ========== State Machine ==========
    enum class BmsState {
        INIT,           // Initialization
        IDLE,           // No load, contactors open
        OPERATIONAL,    // Contactors closed, system operational
        FAULT,          // Fault detected, contactors open
        SHUTDOWN,       // Emergency shutdown
    };
    
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
    float getStateOfCharge() const;  // 0-100% (if implemented)
    
    // ========== Hardware Control ==========
    // Note: Contactor control via GPIO (set by BmsManager, user provides GPIO pins)
    // Note: Fan control via PWM (set by BmsManager, user provides TIM handle)
    void setContactorGpio(GPIO_TypeDef* port, uint16_t pin);  // Set GPIO for contactor control
    void setFanPwmTimer(TIM_HandleTypeDef* htim, uint32_t channel);  // Set PWM timer for fans
    
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
        uint32_t data_stale_timeout_ms = 2000;
        
        // Current measurement (ADS1115 to current conversion)
        float current_shunt_resistance_ohm = 0.001f;  // Shunt resistance
        float current_gain = 1.0f;  // Current sense amplifier gain
        
        // Fan control
        float fan_on_temp_C = 30.0f;
        float fan_max_temp_C = 40.0f;
    };
    
    void setConfig(const Config& config);
    const Config& getConfig() const;
    
private:
    // Internal methods
    void updateFaults(uint32_t now_ms);
    void updateStateMachine(uint32_t now_ms);
    void updateContactors(uint32_t now_ms);  // Controls GPIO pins
    void updateFans(uint32_t now_ms);        // Controls PWM duty cycle
    void updateLEDs();
    
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
    bool checkAuxOvercurrent();     // From INA226
    bool checkDataStale(uint32_t now_ms);
    
    // Current measurement
    float convertAdcToCurrent(float adc_voltage_V);  // Convert ADS1115 reading to current
    
    // Hardware control helpers
    void setContactorGpioState(bool closed);  // Set GPIO pin for contactor
    void setFanPwmDuty(uint8_t percent);     // Set PWM duty cycle (0-100%)
    
    // Safety checks
    bool canCloseContactors() const;  // Check all safety conditions
    void emergencyShutdown();
    
    // Members
    PrimaryBmsFleet* fleet_;
    ADS1115* battery_current_adc_;  // Battery current measurement
    INA226* aux_current_monitor_;   // Auxiliary system current
    
    // Hardware control interfaces
    GPIO_TypeDef* contactor_gpio_port_;  // GPIO port for contactor control
    uint16_t contactor_gpio_pin_;        // GPIO pin for contactor control
    TIM_HandleTypeDef* fan_pwm_tim_;     // PWM timer for fan control
    uint32_t fan_pwm_channel_;           // PWM channel for fan control
    
    Config config_;
    BmsState state_;
    uint16_t active_faults_;
    
    // State machine timing
    uint32_t state_entry_time_ms_;
    
    // Current measurements
    float battery_current_A_;      // Battery current (from ADS1115)
    float aux_current_A_;          // Auxiliary current (from INA226)
    float pack_voltage_V_;
    uint32_t last_current_update_ms_;
    
    // Contactor state
    bool contactors_closed_;
    bool contactor_request_;
    
    // Fan control
    uint8_t fan_speed_percent_;  // 0-100% PWM duty cycle
};
```

---

## State Machine Design

### State Diagram

```
     [Power On]
         |
         v
      [INIT]
         |
         v
      [IDLE] <------------------┐
         |                      |
    [Request Contactors Close]  |
         |                      |
         v                      |
   [OPERATIONAL]                |
         |                      |
    [Fault/Request Open]        |
         |                      |
         +----------------------+
         |
         v
      [FAULT]
         |
    [Recovery OK]
         |
         v
      [IDLE]
         |
    [Critical Fault]
         |
         v
    [SHUTDOWN] (latch until reset)
```

### State Descriptions

#### INIT
- **Entry:** System startup
- **Actions:** 
  - Initialize all subsystems
  - Check hardware health
  - Load configuration
- **Exit:** When initialization complete → IDLE

#### IDLE
- **Entry:** No active operation
- **Actions:**
  - Contactors open
  - Monitor for contactor close request
  - Monitor for faults
  - System ready but not operational
- **Exit:** 
  - Contactor close request (if safe) → OPERATIONAL
  - Fault detected → FAULT

#### OPERATIONAL
- **Entry:** Contactors requested to close and safety checks pass
- **Actions:**
  - Close main contactors
  - Monitor battery current (from ADS1115)
  - Monitor auxiliary current (from INA226)
  - Monitor all fault conditions
  - Control fans based on temperature
  - System can charge or discharge (direction determined externally)
- **Exit:**
  - Contactor open request → IDLE
  - Fault detected → FAULT
  - Critical fault → SHUTDOWN

#### FAULT
- **Entry:** Any fault detected
- **Actions:**
  - Open contactors immediately
  - Log fault
  - Wait for recovery time
  - Check if fault cleared
- **Exit:**
  - Fault cleared → IDLE
  - Critical fault → SHUTDOWN

#### SHUTDOWN
- **Entry:** Critical fault (emergency shutdown)
- **Actions:**
  - Open contactors
  - Disable all outputs
  - Set shutdown flag (latched)
- **Exit:** Only on system reset

---

## Fault Detection Design

### Fault Types & Thresholds

| Fault Type | Detection Condition | Recovery Condition | Priority |
|------------|---------------------|-------------------|----------|
| Overvoltage | Cell > 4200mV | Cell < 4150mV (hysteresis) | High |
| Undervoltage | Cell < 2500mV | Cell > 2550mV (hysteresis) | High |
| Cell Imbalance | Max - Min > 100mV | Max - Min < 80mV | Medium |
| Overtemperature | Temp > 45°C | Temp < 43°C | High |
| Undertemperature | Temp < -10°C | Temp > -8°C | Medium |
| Battery Overcurrent | |Current| > 100A | |Current| < 95A | High |
| Auxiliary Overcurrent | Aux current > 20A | Aux current < 18A | Medium |
| Data Stale | No update > 2s | Update received | Medium |
| Emergency Shutdown | Critical fault | System reset | Critical |

### Fault Hysteresis

All faults use hysteresis to prevent oscillation:
- **Set threshold:** Fault condition detected
- **Clear threshold:** Condition must improve by hysteresis amount
- **Recovery time:** Additional delay before clearing (prevents rapid toggling)

Example:
```cpp
// Overvoltage detection with hysteresis
if (cell_voltage > config_.cell_overvoltage_mV) {
    setFault(FaultType::OVERVOLTAGE);
} else if (cell_voltage < (config_.cell_overvoltage_mV - config_.voltage_hysteresis_mV)) {
    // Only clear if below threshold minus hysteresis
    clearFault(FaultType::OVERVOLTAGE);
}
```

---

## Integration Plan

### Phase 1: Core Structure
1. Create `BmsManager` class skeleton
2. Integrate with existing `PrimaryBmsFleet`
3. Move fault detection from `BmsController` to `BmsManager`
4. Basic state machine (IDLE, OPERATIONAL, FAULT states)
5. Implement ADS1115 to current conversion

### Phase 2: State Machine
1. Implement all states (INIT, IDLE, OPERATIONAL, FAULT, SHUTDOWN)
2. Add state transition logic
3. Add contactor control with safety checks
4. Add state entry/exit actions

### Phase 3: Advanced Features
1. Battery current monitoring (ADS1115 conversion and integration)
2. Auxiliary current monitoring (INA226 integration)
3. Fan control integration (PWM-based, temperature-controlled)
4. Contactor control (GPIO-based)
5. Cell imbalance detection
6. Fault recovery logic with hysteresis

### Phase 4: Safety & Polish
1. Emergency shutdown
2. Safety interlocks
3. Data logging
4. CAN status messages

---

## Usage Example

```cpp
// In User.cpp

// Create BMS manager
static BmsManager bms(&fleet, &adc, &ina);

void setup() {
    // ... existing setup ...
    
    // Configure hardware interfaces
    bms.setContactorGpio(CONTACTOR_GPIO_Port, CONTACTOR_Pin);  // Set GPIO for contactors
    bms.setFanPwmTimer(&htim1, TIM_CHANNEL_1);  // Set PWM timer for fans (example)
    
    // Configure BMS thresholds
    BmsManager::Config config;
    config.cell_overvoltage_mV = 4200;
    config.cell_undervoltage_mV = 2500;
    config.overtemp_C = 45.0f;
    config.overcurrent_A = 100.0f;
    config.aux_overcurrent_A = 20.0f;
    // Current measurement calibration
    config.current_shunt_resistance_ohm = 0.001f;  // 1mΩ shunt
    config.current_gain = 50.0f;  // Current sense amplifier gain
    bms.setConfig(config);
    
    // Initialize BMS
    bms.init();
}

void loop() {
    uint32_t now_ms = HAL_GetTick();
    
    // Update BMS (handles everything)
    bms.update(now_ms);
    
    // Check state and act accordingly
    if (bms.getState() == BmsManager::BmsState::FAULT) {
        // Handle fault state
        uint16_t faults = bms.getActiveFaults();
        // ... handle specific faults ...
    }
    
    // Request contactor operations (from CAN, GPIO, etc.)
    if (contactor_close_requested) {
        bms.requestContactorsClose();
    }
    if (contactor_open_requested) {
        bms.requestContactorsOpen();
    }
    
    // Update LEDs based on BMS state
    updateLEDs(bms.getState(), bms.getActiveFaults());
    
    HAL_Delay(100);
}
```

---

## Migration Strategy

### Step 1: Create BmsManager alongside BmsController
- Keep `BmsController` working
- Create `BmsManager` with basic functionality
- Test in parallel

### Step 2: Migrate functionality
- Move fault detection to `BmsManager`
- Update `User.cpp` to use `BmsManager`
- Keep `BmsController` for reference

### Step 3: Remove old code
- Delete `BmsController` once migration complete
- Clean up `User.cpp`

---

## Next Steps

1. **Review this design** - Get team feedback
2. **Create BmsManager.hpp** - Header file with interface
3. **Implement Phase 1** - Core structure and basic fault detection
4. **Test incrementally** - Test each phase before moving on
5. **Document as you go** - Keep documentation updated

---

## Questions to Consider

1. **Contactor GPIO:** Which GPIO port and pin are used for contactor control? Active high or low?
2. **Contactor requests:** How are close/open requests triggered? CAN message? GPIO? User input?
3. **Fan PWM:** Which timer and channel are used for fan PWM control? What's the PWM frequency?
3. **Current measurement:** What's the ADS1115 configuration? Which channel? What's the shunt resistance and amplifier gain for current calculation?
4. **State of Charge:** Do you want to implement SOC calculation, or just voltage-based?
5. **Data logging:** How much history do you need? Flash storage available?
6. **CAN messages:** What BMS status messages need to be sent to the rest of the car?
7. **Fan control:** What's the fan control strategy? Temperature-based PWM? Linear or step-based? How many fans (single PWM or multiple)?
8. **Charge/discharge direction:** The BMS doesn't control direction - is this handled externally? Do you need to detect direction from current sign?

---

## Implementation Notes

### Contactor Control (GPIO)
- **Interface:** Direct GPIO control (no BTS71040 needed for contactors)
- **Method:** `HAL_GPIO_WritePin()` to set contactor GPIO pin
- **Active Level:** User must specify if active high or low
- **Safety:** Always check `canCloseContactors()` before closing

### Fan Control (PWM)
- **Interface:** HAL TIM PWM functions
- **Method:** `HAL_TIM_PWM_Start()` and `__HAL_TIM_SET_COMPARE()` to set duty cycle
- **Timer:** User provides TIM handle and channel (e.g., TIM1, TIM_CHANNEL_1)
- **Strategy:** Temperature-based, linear or step-based control
- **Range:** 0-100% duty cycle

### Current Measurement (ADS1115)
- **Needs Implementation:** Conversion from ADC voltage reading to current
- **Formula:** `I = (V_adc - V_offset) / (R_shunt * Gain)`
- **Configuration:** Shunt resistance and amplifier gain in Config struct

---

**Ready to start implementation?** Let me know which phase you'd like to tackle first!


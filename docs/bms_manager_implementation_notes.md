# BmsManager Implementation Notes

**Date:** January 2025  
**Status:** Phase 1 Complete - Core Structure Implemented

---

## What's Been Implemented

### ✅ Phase 1: Core Structure (COMPLETE)

1. **BmsManager Class Created**
   - Header: `DistributedBMSPrimary/Core/Inc/BmsManager.hpp`
   - Implementation: `DistributedBMSPrimary/Core/Src/BmsManager.cpp`

2. **Core Features Implemented:**
   - ✅ Constructor and initialization
   - ✅ Main `update()` function
   - ✅ Data access methods
   - ✅ Fault detection with hysteresis
   - ✅ Basic state machine (INIT, IDLE, OPERATIONAL, FAULT, SHUTDOWN)
   - ✅ GPIO-based contactor control
   - ✅ PWM-based fan control
   - ✅ Current measurement integration (ADS1115 and INA226)
   - ✅ Configuration structure

3. **Fault Detection Implemented:**
   - ✅ Overvoltage detection (with hysteresis)
   - ✅ Undervoltage detection (with hysteresis)
   - ✅ Cell imbalance detection
   - ✅ Overtemperature detection (with hysteresis)
   - ✅ Undertemperature detection (with hysteresis)
   - ✅ Battery overcurrent detection (with hysteresis)
   - ✅ Auxiliary overcurrent detection (with hysteresis)
   - ✅ Data staleness detection

4. **State Machine Implemented:**
   - ✅ INIT → IDLE transition
   - ✅ IDLE → OPERATIONAL (with safety checks)
   - ✅ OPERATIONAL → IDLE/FAULT
   - ✅ FAULT → IDLE (after recovery time)
   - ✅ Any state → SHUTDOWN (on critical fault)

5. **Integration:**
   - ✅ Updated `User.cpp` to use BmsManager
   - ✅ Kept old `BmsController` for migration safety
   - ✅ Added BmsManager initialization in `setup()`
   - ✅ Added BmsManager update in `loop()`

---

## Configuration Required

### Before Using BmsManager

You need to configure the hardware interfaces in `User.cpp` `setup()`:

```cpp
// 1. Set contactor GPIO (replace with your actual GPIO)
bms_manager->setContactorGpio(CONTACTOR_GPIO_Port, CONTACTOR_Pin);

// 2. Set fan PWM timer (replace with your actual timer/channel)
bms_manager->setFanPwmTimer(&htim1, TIM_CHANNEL_1);  // or htim3, TIM_CHANNEL_4

// 3. Configure current measurement calibration
BmsManager::Config config = bms_manager->getConfig();
config.current_adc_channel = 0;  // ADS1115 channel for battery current
config.current_shunt_resistance_ohm = 0.001f;  // Your shunt resistance
config.current_gain = 50.0f;  // Your amplifier gain
config.current_offset_V = 0.0f;  // Calibrate this offset
bms_manager->setConfig(config);
```

---

## Current Status

### Working Features
- ✅ Fault detection logic
- ✅ State machine transitions
- ✅ Data access methods
- ✅ Configuration system

### Needs Configuration
- ⚠️ **Contactor GPIO** - Must be set in `setup()`
- ⚠️ **Fan PWM Timer** - Must be set in `setup()`
- ⚠️ **Current Calibration** - Must be calibrated for your hardware

### Not Yet Implemented (Future Phases)
- ⚪ CAN/FDCAN status messages
- ⚪ Data logging/history
- ⚪ State of Charge calculation
- ⚪ Advanced safety interlocks
- ⚪ Multiple fan control (currently single PWM)

---

## Usage Example

```cpp
// In loop()
uint32_t now_ms = HAL_GetTick();
bms_manager->update(now_ms);

// Check state
if (bms_manager->getState() == BmsManager::BmsState::OPERATIONAL) {
    // System is operational
}

// Request contactor operations
bms_manager->requestContactorsClose();
// or
bms_manager->requestContactorsOpen();

// Check faults
uint16_t faults = bms_manager->getActiveFaults();
if (bms_manager->hasFault(BmsManager::FaultType::OVERVOLTAGE)) {
    // Handle overvoltage
}

// Get measurements
float battery_current = bms_manager->getBatteryCurrent_A();
float aux_current = bms_manager->getAuxCurrent_A();
uint8_t fan_speed = bms_manager->getFanSpeed();
```

---

## Testing Checklist

- [ ] Configure contactor GPIO and test contactor control
- [ ] Configure fan PWM and test fan control
- [ ] Calibrate current measurement (ADS1115)
- [ ] Test fault detection with known values
- [ ] Test state machine transitions
- [ ] Test fault recovery
- [ ] Test emergency shutdown
- [ ] Verify fan speed changes with temperature
- [ ] Test contactor safety checks

---

## Known Issues / TODOs

1. **Current Measurement Calibration**
   - Need to determine actual shunt resistance
   - Need to determine amplifier gain
   - Need to calibrate offset voltage
   - Formula: `I = (V_adc - V_offset) / (R_shunt * Gain)`

2. **Contactor GPIO**
   - Need to identify which GPIO pin controls contactors
   - Need to determine if active high or low

3. **Fan PWM**
   - Need to identify which timer/channel for fans
   - TIM1 Channel 1 or TIM3 Channel 4 are available

4. **Current ADC Channel**
   - Currently using channel 0
   - Verify this is correct for battery current measurement

---

## Next Steps

1. **Configure Hardware Interfaces**
   - Set contactor GPIO in `setup()`
   - Set fan PWM in `setup()`
   - Calibrate current measurement

2. **Test Basic Functionality**
   - Verify fault detection works
   - Verify state machine transitions
   - Test contactor control
   - Test fan control

3. **Phase 2: Enhancements**
   - Add CAN status messages
   - Improve safety checks
   - Add data logging

---

## Migration from BmsController

The old `BmsController` is still in the code for safety. Once BmsManager is tested and working:

1. Remove `BmsController controller;` from User.cpp
2. Remove `#include "BmsController.hpp"`
3. Remove legacy fault update code
4. Use only BmsManager for all BMS logic

---

**Implementation Status:** ✅ Phase 1 Complete - Ready for Testing


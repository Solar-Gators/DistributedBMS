/*
 * DeviceConfig.hpp
 *
 * Created on: Oct 27, 2025
 *      Author: samrb
 */

#ifndef INC_DEVICECONFIG_HPP_
#define INC_DEVICECONFIG_HPP_

#pragma once
#include <cstdint>

namespace DeviceConfig {
    // Device Configuration
    // ===================
    // Per-board CAN slot (0-based). Reflash each daughter with a unique value.
    //   0 -> 0x100, 1 -> 0x101, ... 5 -> 0x105  (6 boards, matches PrimaryV2 MAX_MODULES)
    // CAN ID = BASE_CAN_ID + DEVICE_NUMBER
    static constexpr uint8_t DEVICE_NUMBER = 5;  // Set 0..5 uniquely per board before flashing
    
    // CAN ID Configuration
    // ===================
    static constexpr uint16_t BASE_CAN_ID = 0x100;  // Base CAN ID for daughter boards
    static constexpr uint16_t MAX_DEVICES = 6;      // Must match PrimaryV2 MAX_MODULES
    
    // Calculated CAN ID
    static constexpr uint16_t CAN_ID = BASE_CAN_ID + DEVICE_NUMBER;
    
    // Validation (0..5 → CAN IDs 0x100..0x105)
    static_assert(DEVICE_NUMBER < MAX_DEVICES,
                  "DEVICE_NUMBER must be 0 .. MAX_DEVICES-1");
    static_assert(CAN_ID <= 0x7FF, 
                  "Calculated CAN ID exceeds 11-bit limit");
    
    // Device Information
    // ==================
    static constexpr uint8_t CELL_COUNT_CONF = 6;        // Number of cells monitored by this device
    static constexpr uint32_t CYCLE_TIME_MS = 250;   // Main loop cycle time in milliseconds
    
    // Debug Configuration
    // ===================
    static constexpr bool ENABLE_DEBUG_PRINTS = false;  // Set to true for debug output
    static constexpr bool ENABLE_CAN_MONITORING = false; // Set to true to monitor CAN traffic
}

#endif /* INC_DEVICECONFIG_HPP_ */

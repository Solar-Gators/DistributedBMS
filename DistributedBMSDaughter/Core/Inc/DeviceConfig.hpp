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
    // Change this number to set the device ID for this daughter board
    // CAN ID will be automatically calculated as: BASE_CAN_ID + DEVICE_NUMBER
    static constexpr uint8_t DEVICE_NUMBER = 1;  // Change this: 1, 2, 3, 4, 5, 6, 7, 8
    
    // CAN ID Configuration
    // ===================
    static constexpr uint16_t BASE_CAN_ID = 0x100;  // Base CAN ID for daughter boards
    static constexpr uint16_t MAX_DEVICES = 8;      // Maximum number of daughter boards
    
    // Calculated CAN ID
    static constexpr uint16_t CAN_ID = BASE_CAN_ID + DEVICE_NUMBER;
    
    // Validation
    static_assert(DEVICE_NUMBER >= 1 && DEVICE_NUMBER <= MAX_DEVICES, 
                  "DEVICE_NUMBER must be between 1 and 8");
    static_assert(CAN_ID <= 0x7FF, 
                  "Calculated CAN ID exceeds 11-bit limit");
    
    // Device Information
    // ==================
    static constexpr uint8_t CELL_COUNT_CONF = 4;        // Number of cells monitored by this device
    static constexpr uint32_t CYCLE_TIME_MS = 250;   // Main loop cycle time in milliseconds
    
    // Debug Configuration
    // ===================
    static constexpr bool ENABLE_DEBUG_PRINTS = false;  // Set to true for debug output
    static constexpr bool ENABLE_CAN_MONITORING = false; // Set to true to monitor CAN traffic
}

#endif /* INC_DEVICECONFIG_HPP_ */

/*
 * BmsFleet.hpp
 *
 *  Created on: Nov 26, 2025
 *      Author: samrb
 */

#ifndef CUSTOM_INC_BMSFLEET_HPP_
#define CUSTOM_INC_BMSFLEET_HPP_

#pragma once

#include <cstdint>
#include <array>
#include "CanFrames.hpp"
#include "CanBus.hpp"

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------
constexpr uint8_t  MAX_MODULES = 8;
constexpr uint32_t STALE_MS    = 1500;

// -----------------------------------------------------------------------------
// UART message IDs
// -----------------------------------------------------------------------------
enum UartMessageType : uint8_t {
    UART_FLEET_SUMMARY  = 16,
    UART_MODULE_SUMMARY = 17,
};

// -----------------------------------------------------------------------------
// Module-level data (derived from CAN)
// -----------------------------------------------------------------------------
struct ModuleData {
    float    highTemp = -1000.0f;
    float    avgTemp  = 0.0f;
    uint8_t  highTempID = 0;

    uint16_t highVoltage = 0;
    uint16_t lowVoltage  = UINT16_MAX;
    uint8_t  lowVoltageID = 0;
    uint8_t  highVoltageID = 0;

    float    avgVoltage = 0.0f;

    uint8_t  num_cells = 0;
    uint8_t  faults = 0;

    uint32_t last_ms = 0;
    bool     online  = false;

    void clear();
};

// -----------------------------------------------------------------------------
// Fleet-level aggregated data
// -----------------------------------------------------------------------------
struct FleetData {
    uint32_t totalVoltage = 0;      // mV
    uint16_t highestVoltage = 0;    // mV
    uint16_t lowestVoltage  = 0;    // mV

    float    highestTemp = -1000.0f; // °C

    // Global cell indices (across entire pack)
    uint8_t highVoltageID = 0;
    uint8_t lowVoltageID  = 0;
    uint8_t highTempID    = 0;
};

// -----------------------------------------------------------------------------
// CAN-ID → module index mapping
// -----------------------------------------------------------------------------
struct IdMapElement {
    uint16_t can_id = 0;
    uint8_t  index  = 0;
    bool     used   = false;
};

// -----------------------------------------------------------------------------
// BmsFleet
// -----------------------------------------------------------------------------
class BmsFleet {
public:
    BmsFleet();

    // CAN handling
    bool registerDaughter(uint16_t can_id, uint8_t index);
    void handleMessage(const CanBus::Frame& msg, uint32_t now_ms);

    // Processing
    void processModules();
    void online(uint32_t now_ms);
    bool isOnline(uint8_t index);


    // Accessors
    ModuleData& module(uint8_t id);
    const FleetData& fleet() const { return fleet_; }

    // UART packing
    size_t packFleetData(uint8_t* out) const;
    size_t packModuleData(uint8_t* out, uint8_t module_idx) const;  // <-- NEW

private:
    std::array<ModuleData,  MAX_MODULES> modules_{};
    std::array<IdMapElement, MAX_MODULES> idmap_{};

    FleetData fleet_{};
};

#endif /* CUSTOM_INC_BMSFLEET_HPP_ */

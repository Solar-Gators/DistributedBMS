#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "CanBus.hpp"
#include "CanFrames.hpp"
#include "FleetSummary.hpp"
#include "PrimaryV2Contract.hpp"

constexpr uint8_t MAX_MODULES = 6;
constexpr uint32_t STALE_MS = PrimaryV2Contract::DAUGHTER_STALE_TIMEOUT_MS;

/**
 * Temporary bench filter: drop implausible daughter readings from pack
 * aggregation / safety faults. Set false when DB3 wire is fixed and fleet
 * validation is hardened.
 */
constexpr bool kFilterImplausibleFleetReadings = true;

/**
 * Bench: DB1 and DB4 are symmetric; DB4 NTC hardware is unreliable.
 * Copy DB1 temperature fields onto DB4 before fleet aggregation / debug snapshots.
 * Voltages and online status still come from each board's own CAN frames.
 */
constexpr bool kMirrorDb4TempsFromDb1 = true;  /* Bench: was true — DB4 temps copied from DB1 */
constexpr uint8_t kSymmetricTempSourceSlot = 1u;  /* DB1, CAN 0x101 */
constexpr uint8_t kSymmetricTempDestSlot = 4u;    /* DB4, CAN 0x104 */

constexpr uint16_t kFleetMinPlausibleCell_mV = 2000u;
constexpr uint16_t kFleetMaxPlausibleCell_mV = 5000u;
constexpr float kFleetMinPlausibleTemp_C = 10.0f;
constexpr float kFleetMaxPlausibleTemp_C = 60.0f;

enum UartMessageType : uint8_t {
    UART_FLEET_SUMMARY = 16,
    UART_MODULE_SUMMARY = 17,
};

struct ModuleData {
    float highTemp = -1000.0f;
    float lowTemp = 1000.0f;
    float avgTemp = 0.0f;
    uint8_t highTempID = 0;

    uint16_t highVoltage = 0;
    uint16_t lowVoltage = 0;
    uint8_t lowVoltageID = 0;
    uint8_t highVoltageID = 0;

    float avgVoltage = 0.0f;

    uint8_t num_cells = 0;
    uint8_t faults = 0;

    uint32_t last_ms = 0;
    bool online = false;

    void clear();
};

struct FleetData {
    uint32_t totalVoltage = 0;
    uint16_t highestVoltage = 0;
    uint16_t lowestVoltage = 0;

    float highestTemp = -1000.0f;

    uint8_t highVoltageID = 0;
    uint8_t lowVoltageID = 0;
    uint8_t highTempID = 0;
};

struct IdMapElement {
    uint16_t can_id = 0;
    uint8_t index = 0;
    bool used = false;
};

class BmsFleet {
public:
    BmsFleet();

    bool registerDaughter(uint16_t can_id, uint8_t index);
    void handleMessage(const CanBus::Frame& msg, uint32_t now_ms);

    void processModules();
    void online(uint32_t now_ms);
    bool isOnline(uint8_t index);

    ModuleData& module(uint8_t id);
    const FleetData& fleet() const { return fleet_; }

    /** Filled at end of processModules() for BmsManager (legacy FleetSummaryData shape). */
    const FleetSummaryData& summary() const { return summary_cache_; }
    const ModuleSummaryData& moduleSnapshot(uint8_t idx) const { return module_snapshots_[idx]; }
    bool has_data(uint32_t now_ms) const { return summary_cache_.is_online(now_ms); }

    /** Debug: map 11-bit CAN ID to fleet slot, or -1 if not registered. */
    int slotForStdCanId(uint16_t can_id) const;

    uint32_t decodeOkCount() const { return decode_ok_count_; }
    uint32_t decodeFailCount() const { return decode_fail_count_; }

    size_t packFleetData(uint8_t* out) const;
    size_t packModuleData(uint8_t* out, uint8_t module_idx) const;

private:
    void refreshSummaryCache(uint32_t now_ms);
    void applySymmetricTempMirrors_();
    /** Map RX frame to modules_[] index using registerDaughter(); -1 if unknown. Classic CAN only. */
    int daughterSlotForCanId_(const CanBus::Frame& msg) const;

    std::array<ModuleData, MAX_MODULES> modules_{};
    std::array<IdMapElement, MAX_MODULES> idmap_{};

    FleetData fleet_{};
    FleetSummaryData summary_cache_{};
    std::array<ModuleSummaryData, MAX_MODULES> module_snapshots_{};
    uint32_t decode_ok_count_ = 0;
    uint32_t decode_fail_count_ = 0;
};

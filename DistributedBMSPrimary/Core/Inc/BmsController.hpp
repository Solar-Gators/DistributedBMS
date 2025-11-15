/*
 * BmsController.hpp
 *
 *  Created on: Nov 14, 2025
 *      Author: samrb
 */

#ifndef INC_BMSCONTROLLER_HPP_
#define INC_BMSCONTROLLER_HPP_

#pragma once

#include <cstdint>
#include <array>
#include "PrimaryBmsFleet.hpp"

// ---------------- Pack-level view ----------------

struct PackState {
    // Copy of per-module summaries from PrimaryBmsFleet
    std::array<ModuleSummaryData, PrimaryBmsFleetCfg::MAX_MODULES> modules{};

    // Pack-level values
    float pack_current_A = 0.0f;   // + = discharge, - = charge
    float pack_voltage_V = 0.0f;   // simple sum of module voltages
    float pack_power_W   = 0.0f;   // V * I

    // Extremes
    uint16_t lowest_cell_mV  = 0;
    uint16_t highest_cell_mV = 0;
    float    hottest_temp_C  = -1000.0f;

    // Data freshness
    uint32_t last_update_ms = 0;
    bool     data_valid     = false;
};

// ---------------- Limits / config ----------------

struct PackLimits {
    // Cell voltage limits (mV)
    uint16_t cell_ovp_mV_chg = 4200;  // over-voltage during charge
    uint16_t cell_uvp_mV     = 2500;  // under-voltage

    // Temperatures (°C)
    float temp_max_discharge_C = 60.0f;
    float temp_max_charge_C    = 45.0f;
    float temp_min_charge_C    = 0.0f;

    // Currents (A)
    float ocp_discharge_A = 150.0f;   // over-current discharge
    float ocp_charge_A    = 80.0f;    // over-current charge

    // Timing
    uint32_t data_stale_ms      = PrimaryBmsFleetCfg::STALE_MS; // ~2 s

};

// ---------------- Faults ----------------

enum FaultFlags : uint32_t {
    FAULT_NONE            = 0,
    FAULT_CELL_OVP        = 1u << 0,
    FAULT_CELL_UVP        = 1u << 1,
    FAULT_TEMP_OVER       = 1u << 2,
    FAULT_TEMP_UNDER_CHG  = 1u << 3,
    FAULT_OCP_DISCHARGE   = 1u << 4,
    FAULT_OCP_CHARGE      = 1u << 5,
    FAULT_DATA_STALE      = 1u << 6,
};

struct FaultStatus {
    uint32_t active  = FAULT_NONE;  // currently true
    uint32_t latched = FAULT_NONE;  // sticky until cleared

    void clear_latched() { latched = FAULT_NONE; }
};

// ---------------- BMS state & contactors ----------------

enum class BmsState : uint8_t {
    OFF,
    RUN,
    FAULT_LATCHED,
    CHARGE_ONLY,
};

struct ContactorCommand {
    bool main_pos  = false; // main positive contactor
    bool main_neg  = false; // main negative / return contactor
    bool precharge = false; // precharge path
};

// ---------------- BmsController ----------------

class BmsController {
public:
    explicit BmsController(const PackLimits& limits);

    // Update pack_ from fleet + current measurement
    void update_from_sources(const PrimaryBmsFleet& fleet,
                             float pack_current_A,
                             uint32_t now_ms);

    // Run faults + state machine, update contactor commands
    void step(bool key_on,
              bool charger_present,
              uint32_t now_ms);

    // Accessors
    const PackState&   pack()   const { return pack_; }
    const FaultStatus& faults() const { return faults_; }
    BmsState           state()  const { return state_; }
    ContactorCommand   contactor_cmds() const { return contactor_cmds_; }

    // Allow external clear of latched faults (e.g. user reset)
    void clear_latched_faults() { faults_.clear_latched(); }

private:
    PackState      pack_{};
    PackLimits     limits_{};
    FaultStatus    faults_{};
    BmsState       state_ = BmsState::OFF;
    ContactorCommand contactor_cmds_{};

    uint32_t precharge_start_ms_ = 0;

    // helpers
    void aggregate_from_fleet(const PrimaryBmsFleet& fleet,
                              float pack_current_A,
                              uint32_t now_ms);

    void evaluate_faults(uint32_t now_ms);

    void run_state_machine(bool key_on,
                           bool charger_present,
                           uint32_t now_ms);

    static ContactorCommand commands_for_state(BmsState st);
};




#endif /* INC_BMSCONTROLLER_HPP_ */

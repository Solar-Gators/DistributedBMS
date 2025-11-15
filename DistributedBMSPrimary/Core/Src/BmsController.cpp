/*
 * BmsController.cpp
 *
 *  Created on: Nov 14, 2025
 *      Author: samrb
 */

#include "BmsController.hpp"
#include <algorithm> // std::min, std::max

// ---------------- ctor ----------------

BmsController::BmsController(const PackLimits& limits)
    : limits_(limits)
{
    // nothing else
}

// ---------------- public: update_from_sources ----------------

void BmsController::update_from_sources(const PrimaryBmsFleet& fleet,
                                        float pack_current_A,
                                        uint32_t now_ms)
{
    aggregate_from_fleet(fleet, pack_current_A, now_ms);
}

// ---------------- public: step ----------------

void BmsController::step(bool key_on,
                         bool charger_present,
                         uint32_t now_ms)
{
    evaluate_faults(now_ms);
    run_state_machine(key_on, charger_present, now_ms);
    contactor_cmds_ = commands_for_state(state_);
}

// ---------------- private: aggregate_from_fleet ----------------

void BmsController::aggregate_from_fleet(const PrimaryBmsFleet& fleet,
                                         float pack_current_A,
                                         uint32_t now_ms)
{
    float pack_mV_sum = 0.0f;
    uint16_t low_mV   = 0xFFFF;
    uint16_t high_mV  = 0;
    float hottest     = -1000.0f;

    // Copy module data & compute aggregates
    for (uint8_t i = 0; i < PrimaryBmsFleetCfg::MAX_MODULES; ++i) {
        const auto& M = fleet.module(i);
        pack_.modules[i] = M;  // copy for higher-level use

        if (!M.valid) continue;

        pack_mV_sum += static_cast<float>(M.avg_cell_mV) *
                       static_cast<float>(M.num_cells);

        low_mV  = std::min<uint16_t>(low_mV,  M.low_mV);
        high_mV = std::max<uint16_t>(high_mV, M.high_mV);
        hottest = std::max(hottest, M.high_temp_C);
    }

    pack_.pack_current_A = pack_current_A;
    pack_.pack_voltage_V = pack_mV_sum / 1000.0f;
    pack_.pack_power_W   = pack_.pack_voltage_V * pack_.pack_current_A;
    pack_.lowest_cell_mV = (low_mV == 0xFFFF) ? 0 : low_mV;
    pack_.highest_cell_mV = high_mV;
    pack_.hottest_temp_C  = hottest;

    pack_.last_update_ms = now_ms;
    // basic data_valid: rely on fleet.has_data()
    pack_.data_valid = fleet.has_data(now_ms);
}

// ---------------- private: evaluate_faults ----------------

void BmsController::evaluate_faults(uint32_t now_ms)
{
    uint32_t new_faults = FAULT_NONE;

    // Data stale?
    if (!pack_.data_valid ||
        (now_ms - pack_.last_update_ms) > limits_.data_stale_ms) {
        new_faults |= FAULT_DATA_STALE;
    }

    // Voltage faults
    if (pack_.highest_cell_mV > limits_.cell_ovp_mV_chg) {
        new_faults |= FAULT_CELL_OVP;
    }
    if (pack_.lowest_cell_mV > 0 &&
        pack_.lowest_cell_mV < limits_.cell_uvp_mV) {
        new_faults |= FAULT_CELL_UVP;
    }

    // Temperature faults
    if (pack_.hottest_temp_C > limits_.temp_max_discharge_C) {
        new_faults |= FAULT_TEMP_OVER;
    }
    // Charge-low temp: pack_current_A negative means charging
    if (pack_.pack_current_A < -0.5f && // some small threshold
        pack_.hottest_temp_C < limits_.temp_min_charge_C) {
        new_faults |= FAULT_TEMP_UNDER_CHG;
    }

    // Current faults
    // Discharge: positive current
    if (pack_.pack_current_A > limits_.ocp_discharge_A) {
        new_faults |= FAULT_OCP_DISCHARGE;
    }
    // Charge: negative current
    if (pack_.pack_current_A < -limits_.ocp_charge_A) {
        new_faults |= FAULT_OCP_CHARGE;
    }

    faults_.active  = new_faults;
    faults_.latched |= new_faults;
}

// ---------------- private: state machine ----------------
void BmsController::run_state_machine(bool key_on,
                                      bool charger_present,
                                      uint32_t now_ms)
{
    (void)now_ms; // unused for now, keeps compiler happy
    const bool any_fault_active = (faults_.active != FAULT_NONE);

    switch (state_) {
    case BmsState::OFF:
        // Go straight to RUN when key is on, data is valid, and no faults
        if (key_on && !any_fault_active && pack_.data_valid) {
            state_ = BmsState::RUN;
        }
        break;

    case BmsState::RUN: {
        if (any_fault_active) {
            state_ = BmsState::FAULT_LATCHED;
            break;
        }
        if (!key_on) {
            state_ = BmsState::OFF;
            break;
        }

        // Optional: CHARGE_ONLY mode if charger present and you want to distinguish it
        if (charger_present && /* condition for no driving */ false) {
            state_ = BmsState::CHARGE_ONLY;
        }
        break;
    }

    case BmsState::FAULT_LATCHED:
        // Stay here until key off + latched faults cleared
        if (!key_on) {
            if (faults_.latched == FAULT_NONE) {
                state_ = BmsState::OFF;
            }
        }
        break;

    case BmsState::CHARGE_ONLY:
        if (any_fault_active) {
            state_ = BmsState::FAULT_LATCHED;
            break;
        }
        if (!key_on) {
            state_ = BmsState::OFF;
            break;
        }
        // If charger removed, go back to RUN
        if (!charger_present) {
            state_ = BmsState::RUN;
        }
        break;
    }
}


// ---------------- private: commands_for_state ----------------
ContactorCommand BmsController::commands_for_state(BmsState st)
{
    ContactorCommand cmd{};
    switch (st) {
    case BmsState::OFF:
    case BmsState::FAULT_LATCHED:
        // all contactors open
        break;

    case BmsState::RUN:
        cmd.main_neg  = true;
        cmd.main_pos  = true;
        cmd.precharge = false;  // no precharge path in your system
        break;

    case BmsState::CHARGE_ONLY:
        // For now, same as RUN; you can specialize later if needed
        cmd.main_neg  = true;
        cmd.main_pos  = true;
        cmd.precharge = false;
        break;
    }
    return cmd;
}



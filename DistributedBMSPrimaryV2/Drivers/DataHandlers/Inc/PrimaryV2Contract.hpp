#pragma once

#include <cstdint>

namespace PrimaryV2Contract {

// Timing contract values mirrored from docs/PRIMARYV2_SYSTEM_CONTRACT.md.
constexpr uint32_t DAUGHTER_STALE_TIMEOUT_MS = 500;
constexpr uint32_t ADS1115_READ_PERIOD_MS = 100;
constexpr uint32_t VEHICLE_COMMAND_PROCESS_MS = 100;
constexpr uint32_t INA226_READ_PERIOD_MS = 500;
constexpr uint32_t CRITICAL_CAN_TX_MAX_PERIOD_MS = 1000;
constexpr uint32_t CONTACTOR_REACTION_MAX_MS = 500;

// Orchestration tick policy (User.cpp scheduler table).
constexpr uint32_t DEFAULT_TASK_LOOP_DELAY_MS = 10;
constexpr uint32_t FLEET_AGGREGATE_PERIOD_MS = 250;
constexpr uint32_t BMS_MANAGER_PERIOD_MS = 10;
constexpr uint32_t VEHICLE_IFACE_PERIOD_MS = 10;
constexpr uint32_t ADS131M02_SAMPLE_PERIOD_MS = 500;

// Deadline budgets used by the scheduler table for runtime contract checks.
constexpr uint32_t FLEET_AGGREGATE_BUDGET_MS = 10;
constexpr uint32_t BMS_MANAGER_BUDGET_MS = 10;
constexpr uint32_t VEHICLE_IFACE_BUDGET_MS = 10;
constexpr uint32_t ADS131M02_SAMPLE_BUDGET_MS = 10;

}  // namespace PrimaryV2Contract

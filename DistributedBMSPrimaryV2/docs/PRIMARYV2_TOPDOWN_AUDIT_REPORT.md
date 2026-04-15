# PrimaryV2 Top-Down Architecture Audit Report

This report implements the top-down audit plan against `docs/PRIMARYV2_SYSTEM_CONTRACT.md`.

## 1) Architecture Map (Module Responsibilities + Dependencies)

### Boot and setup ownership
- `Core/Src/main.cpp` initializes HAL/peripherals, then calls `setup()` before scheduler start.
- `Core/Src/User.cpp` owns application object composition:
  - CAN buses: `daughter_can` (FDCAN2), `vehicle_can` (FDCAN3)
  - Data/safety: `BmsFleet`, `BmsManager`, `BmsCanInterface`
  - Sensors: `ADS1115`, `INA226`, `Ads131m02`
  - Actuators: two contactor GPIOs + fan PWM timer configured through `BmsManager`

### Runtime execution model
- Single RTOS task (`StartDefaultTask`) runs the control loop.
- FDCAN receive is ISR-driven in `CanBus`:
  - HAL callback -> `CanBus::dispatchRxFifo0FromIsr(...)`
  - ISR drains FIFO0 into ring buffer
  - Task context consumes ring buffer via `CanBus::read(...)`
- Control loop cadence:
  - Daughter CAN consume loop: every cycle
  - Fleet aggregate (`fleet.processModules()`): every 250 ms
  - `BmsManager.update(now)`: every loop (~10 ms + work)
  - `BmsCanInterface.update(now)`: every loop (~10 ms + work)
  - ADS131M02 debug sample: every 500 ms if init succeeded
  - `osDelay(10)` at end of loop

### Module boundary summary
- `BmsFleet`: parses daughter telemetry and builds pack/module summary cache.
- `BmsManager`: safety decisions, fault/state machine, contactor/fan actuation.
- `BmsCanInterface`: vehicle command ingress + periodic/event egress.
- `CanBus`: FDCAN transport wrapper, ISR-to-task handoff and TX helper.

## 2) Safety Authority and State-Machine Audit

## Safety authority verdict: **Mostly aligned, with two policy leaks**

### What is correct
- Contactor hardware actuation is centralized in `BmsManager::updateContactors(...)`.
- Fault derivation and state transitions are centralized in `BmsManager`.
- Vehicle CAN commands are routed by `BmsCanInterface` into manager API calls (`requestContactorsClose/Open`, `clearFaults`, `requestShutdown`).
- Two-contactors-without-precharge behavior is implemented (staggered two-main closure path).

### Findings (Change required)
1. **Auto-close policy leak in `User.cpp`**
   - Loop force-requests close whenever state is `IDLE` and no faults.
   - This bypasses vehicle command authority intent from the contract.
   - Impact: contactors can close without explicit external command.

2. **`clearFaults()` does not clear most faults**
   - Current implementation only clears the emergency shutdown bit mask.
   - On next `updateFaults()`, active faults are recomputed anyway.
   - Impact: command semantics are ambiguous versus expected "clear-and-recover workflow".

### Safety path trace (fault -> action)
1. `BmsManager.update()` calls `updateFaults()`
2. Fault bits set from voltage/temp/current/staleness checks
3. State machine enters `FAULT` when active faults non-zero
4. `enterState(FAULT)` requests open
5. `updateContactors()` opens both contactors (unless in close grace period)

## 3) Runtime and Timing Matrix (Target vs Observed)

Observed values are code-derived cadence (worst-case estimate from loop structure), not bench measurements.

| Requirement (contract) | Target | Observed in code | Pass/Fail | Notes |
|---|---:|---:|---|---|
| Daughter staleness timeout | <= 500 ms | 1500 ms (`BmsFleet::STALE_MS`) | **Fail** | Also `FleetSummary` default is 2000 ms; manager config default 5000 ms (configured to 500 in `setup`) |
| ADS1115 pack-current read period | 100 ms | ~every loop (~10 ms + I2C blocking) | **Pass** (frequency), **Risk** (blocking) | No explicit period gate; reads attempted each loop |
| Vehicle CAN RX mechanism | interrupt-driven | ISR ring buffer + task drain | **Pass** | Implemented in `CanBus` callback path |
| Vehicle command processing period | 100 ms | ~every loop (~10 ms + work) | **Pass** (nominal), **Risk** (blocking) | Processed in `BmsCanInterface.update()` |
| INA226 read period | 500 ms | ~every loop (~10 ms + I2C blocking) | **Pass** (frequency), **Risk** (blocking) | No explicit 500 ms gate |
| Critical CAN TX period | <= 1 s | 50 ms defaults for 0x040..0x043 | **Pass** | `BmsCanInterface::Config` defaults meet target |
| Contactor reaction latency on critical fault | <= 500 ms | Next loop iteration + grace logic | **Conditional** | Can be delayed by loop blocking and `contactor_close_grace_period_ms` (500 ms) |

### Timing risk drivers
- Sensor drivers use `HAL_I2C_Mem_*` with `HAL_MAX_DELAY`.
- `INA226::readMeasurement()` performs 4 sequential register reads per loop.
- If I2C stalls, control loop latency can exceed command/fault reaction targets.

## 4) Interface + Data Integrity Audit

### Protocol and packing
- `0x040`..`0x043` periodic set is correctly isolated in `updatePeriodicTransmission()`.
- Decode/encode endianness is internally consistent for helper pack/unpack functions.

### Findings (Change required)
1. **`BatteryCurrent` frame uses raw float over CAN (`0x043`)**
   - Architecture risk: cross-platform endianness and ABI interpretation.
   - Safer alternative: fixed-point signed integer scaling.

2. **Config struct/data width mismatch in `ConfigResponseMsg`**
   - Struct includes `reserved2`, but encoder packs exactly 8 bytes and drops it.
   - Not breaking on-wire now, but contract and struct shape are inconsistent.

3. **Staleness constants are split across multiple modules**
   - `BmsFleet::STALE_MS = 1500`
   - `PrimaryBmsFleetCfg::STALE_MS = 2000`
   - `BmsManager::Config::data_stale_timeout_ms` default 5000 (set to 500 in setup)
   - Risk: behavior depends on which path consumes freshness.

4. **`registerDaughter(...)` map is currently unused in `handleMessage(...)`**
   - Incoming daughter frame index is derived from `id - 0x100` directly.
   - Limits flexibility if IDs diverge from contiguous mapping.

## 5) Agree / Change / Unsure Decision Log

## Agree
- Single-thread + ISR handoff architecture is coherent and maintainable for current scope.
- `BmsManager` remains the main safety arbiter for faults/state/contactor GPIO.
- Vehicle periodic telemetry IDs and cadence strategy are aligned with requested protocol set.
- Two-main-contactor behavior (no precharge) is reflected in control logic.

## Change
- Remove automatic close request from `User.cpp`; make close strictly command/policy-driven.
- Replace blocking sensor reads in hot loop with bounded-time or rate-limited sampling.
- Unify staleness/freshness thresholds into one source of truth matching contract (500 ms).
- Define deterministic semantics for `clearFaults()` and implement a clear-safe recovery policy.
- Replace float wire encoding for current with fixed-point scaled integer format.
- Resolve protocol struct/encoder inconsistencies (`ConfigResponseMsg` shape).

## Unsure
- Whether contactor close grace period should suppress **all** fault-driven opening, or only select transient faults.
- Whether emergency shutdown should remain terminal until reboot (currently can be command-cleared depending on policy path).
- Whether `0x043` float payload is fixed by external vehicle contract (if fixed, keep but document endianness explicitly).

## 6) Prioritized Remediation Backlog (Implementation-Ready)

## P0 (Safety/timing critical)
1. **Remove auto-close in runtime loop**
   - Files: `Core/Src/User.cpp`
   - Task: delete automatic `requestContactorsClose()` block; gate close through CAN command path or explicit policy module.
   - Verify: contactors stay open in IDLE until command received.

2. **Add bounded sensor scheduling**
   - Files: `Drivers/DataHandlers/Src/BmsManager.cpp`
   - Task: add explicit ADS1115/INA226 read periods (100 ms / 500 ms) and avoid unbounded blocking in control path.
   - Verify: loop jitter and max fault/command latency under I2C fault injection.

3. **Unify stale timeout constants to 500 ms**
   - Files: `Drivers/DataHandlers/Inc/BmsFleet.hpp`, `Drivers/DataHandlers/Inc/FleetSummary.hpp`, `Drivers/DataHandlers/Inc/BmsManager.hpp`
   - Task: remove divergent defaults; use one contract constant.
   - Verify: stale fault asserts by <=500 ms after daughter dropout.

## P1 (Protocol correctness/maintainability)
4. **Define `clearFaults()` policy and implement**
   - Files: `Drivers/DataHandlers/Src/BmsManager.cpp`, possibly `BmsCanProtocol.hpp` docs
   - Task: implement explicit clearable vs latched faults behavior.
   - Verify: command-driven recovery sequence test.

5. **Normalize `BatteryCurrent` payload representation**
   - Files: `Drivers/DataHandlers/Inc/BmsCanProtocol.hpp`, `Drivers/DataHandlers/Src/BmsCanProtocol.cpp`, consumers
   - Task: migrate to fixed-point (example: `int16_t current_dA` or `int32_t current_mA`) unless external contract locks float.
   - Verify: byte-level decode agreement with vehicle-side implementation.

6. **Fix config response definition mismatch**
   - Files: `Drivers/DataHandlers/Inc/BmsCanProtocol.hpp`, `Drivers/DataHandlers/Src/BmsCanProtocol.cpp`
   - Task: align struct fields with exact 8-byte wire frame.
   - Verify: static size checks and golden-frame tests.

## P2 (Robustness/future-proofing)
7. **Use daughter ID map in parser**
   - Files: `Drivers/DataHandlers/Src/BmsFleet.cpp`
   - Task: route frame ID via registered map instead of hardcoded contiguous ID arithmetic.
   - Verify: non-contiguous daughter IDs decode correctly.

8. **Document shutdown and grace-period safety policy**
   - Files: `docs/PRIMARYV2_SYSTEM_CONTRACT.md`, `Drivers/DataHandlers/Src/BmsManager.cpp`
   - Task: specify which faults may be grace-delayed and which are immediate-open.
   - Verify: policy tests match documentation.

## 7) Test Impact Checklist

- Unit tests for protocol packing/unpacking (all message IDs used in vehicle path).
- Fault-injection tests for stale data, overcurrent, over/under voltage, and command handling.
- Loop-latency timing measurements with I2C error/stall scenarios.
- Hardware-in-the-loop validation for two-contactor sequencing and fault-open timing.

## 8) Plan Addendum (Requested)

The following items are now added to the architecture plan backlog and should be treated as tracked implementation tasks.

1. Introduce a small scheduler/tick policy layer (or timing table) so each subsystem has explicit cadence and deadline budget.
2. Keep `User.cpp` as orchestration only; move all safety/policy decisions into `BmsManager`.
3. Add a central contract constants header for timing/safety thresholds used by all modules.
4. Convert wire protocol fields to explicit fixed-point integer representations where possible.
5. Add lightweight health telemetry counters (loop max time, sensor read failures, queue overruns) for runtime validation.

### Suggested placement in existing priority buckets

- **P0**
  - Scheduler/tick policy layer with cadence+deadline table.
  - `User.cpp` orchestration-only rule (no safety/policy logic outside `BmsManager`).
  - Contract constants centralized and consumed across modules.
- **P1**
  - Wire protocol migration from float payloads to fixed-point fields where feasible.
- **P2**
  - Health telemetry counters and periodic observability frame/log export.

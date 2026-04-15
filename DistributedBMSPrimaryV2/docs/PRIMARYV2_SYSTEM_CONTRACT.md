# PrimaryV2 System Contract

This contract defines what `DistributedBMSPrimaryV2` owns vs what daughter modules and the vehicle controller own.

## 1) System Boundaries

### PrimaryV2 Responsibilities
- Aggregate daughter telemetry into pack-level summary (`BmsFleet`).
- Measure local currents:
  - Pack current via `ADS1115` (current safety-relevant path).
  - Auxiliary currents via `INA226`.
  - Precision current via `ADS131M02` for future coulomb counting (non-safety path today).
- Run safety/fault evaluation and state machine (`BmsManager`).
- Control primary actuators:
  - Two main contactors only (negative-side and positive-side).
  - Fan PWM output.
  - Status LEDs.
- Serve as vehicle CAN endpoint:
  - Publish periodic and event BMS messages.
  - Receive vehicle commands/config requests.

### Daughter Responsibilities
- Measure per-module cell and temperature data.
- Publish module telemetry frames in expected formats.
- Maintain local sensing quality (Primary assumes decoded daughter values are trustworthy when fresh).

### Vehicle Controller Responsibilities
- Consume Primary BMS status/telemetry frames.
- Issue high-level commands (for example open/close request, shutdown, future kill-switch command behavior).
- Handle system-level policy outside Primary scope.

## 2) Interface Contract

### Inputs To PrimaryV2
- Daughter CAN telemetry on daughter bus.
- Vehicle CAN commands/config requests on vehicle bus.
- Local sensors:
  - `ADS1115` battery current.
  - `INA226` auxiliary currents.
  - `ADS131M02` precision current (non-safety path currently).
- System time base (`osKernelGetTickCount()` / `HAL_GetTick()`).

### Outputs From PrimaryV2
- Vehicle CAN periodic messages (`0x040`-`0x043` in current protocol set).
- Vehicle CAN event messages (fault/state/config response paths).
- Physical outputs:
  - Positive and negative contactor control.
  - Fan PWM command.
  - Status LED states.

## 3) Safety Contract

PrimaryV2 must prevent contactor close and/or force contactor open when unsafe, including:
- Voltage faults (over/under/imbalance).
- Thermal faults (over/under limits).
- Current faults (pack and aux, as configured).
- Data-validity faults (stale/missing daughter telemetry).

Behavioral guarantees:
- Contactors close only when safety gate passes.
- Non-operational states command contactors open.
- Operational faults force contactors open within required reaction latency.
- Emergency shutdown path transitions to shutdown state and applies shutdown fault handling.

## 4) State-Machine Contract

Nominal states:
- `INIT -> IDLE -> OPERATIONAL -> FAULT -> SHUTDOWN`

Core intent:
- `INIT` performs startup checks then enters `IDLE`.
- `IDLE` may enter `OPERATIONAL` only on valid close request and safety pass.
- `OPERATIONAL` returns to `IDLE` on open request.
- Active faults transition to `FAULT`.
- `FAULT` returns to `IDLE` only after configured clear/recovery conditions.
- `SHUTDOWN` requires explicit reset/reboot policy.

## 5) Timing Contract (Target Values)

- Daughter CAN staleness timeout (per daughter): **500 ms max**.
- Pack-current read period (`ADS1115`): **100 ms**.
- Vehicle CAN receive: **interrupt-driven RX**.
- Vehicle CAN command processing period: **100 ms**.
- `INA226` read period: **500 ms**.
- Critical CAN message transmit period: **<= 1 s**.
- Maximum contactor reaction latency for critical faults: **500 ms**.

## 6) Architectural Notes

- `ADS131M02` is currently non-safety and intended for future coulomb counting.
- Two-contactor system has no precharge contactor in this architecture.
- Kill-switch command behavior is expected from vehicle CAN and will be defined in detail later.


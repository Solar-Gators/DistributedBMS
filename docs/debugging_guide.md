# Debugging Guide for Distributed BMS

## Overview

This guide provides practical debugging strategies and variable references for troubleshooting the Distributed BMS system.

## System Architecture

```
Daughter Boards (CAN) → Secondary MCU (CAN) → Secondary MCU (UART) → Primary MCU (UART)
```

## Debugging by Layer

### 1. Daughter Board → Secondary MCU (CAN)

#### Daughter Board Debug Variables

**Location**: `DistributedBMSDaughter/Core/Src/main.cpp`

**Key Checkpoints**:
- `bms.results()`: Contains processed BMS data
- CAN transmission in three tasks:
  - `StartDefaultTask`: Averages message (type 2)
  - `StartVoltageTask`: Voltage extremes (type 1)
  - `StartTemperatureTask`: High temperature (type 0)

**Verification**:
1. Check CAN transmission success (ESR register)
2. Verify CAN ID is correct (0x101 for first board)
3. Check fault manager for communication errors

#### Secondary MCU CAN Reception

**Location**: `DistributedBMSSecondary/Core/Src/main.cpp`

**Key Variables**:
- `fleet`: BmsFleet instance storing module data
- `fleet.module(i)`: Access module data by index
- `fleet.module(i).got_type0/1/2`: Flags indicating which CAN messages received

**Debugging Steps**:
```cpp
// Check if module has received data
for (uint8_t i = 0; i < BmsFleetCfg::MAX_MODULES; ++i) {
    if (fleet.has_any_data(i)) {
        const auto& mod = fleet.module(i);
        // mod.high_C, mod.low_mV, etc.
    }
}

// Check specific message types received
bool has_temp = fleet.module(0).got_type0;   // HIGH_TEMP
bool has_volt = fleet.module(0).got_type1;  // VOLTAGE_EXTREMES
bool has_avg  = fleet.module(0).got_type2;   // AVERAGES
```

**CAN Callback Verification**:
- `allCallback()`: Should toggle `OK_Pin` on every CAN message
- `daughterOneCallback()`: Specific handler for ID 0x101

### 2. Secondary MCU → Primary MCU (UART)

#### Secondary MCU UART Transmission

**Location**: `DistributedBMSSecondary/Core/Src/main.cpp` - `StartDefaultTask()`

**Key Variables**:
```cpp
static uint8_t frame_rotation = 0;  // Current frame type (0=fleet, 1=module, 2=heartbeat)
static uint8_t module_cursor = 0;   // Current module in round-robin
static uint32_t last_heartbeat_ms = 0;
static uint32_t heartbeat_counter = 0;
static volatile uint32_t g_lastHeartbeatInterval = 0;  // Actual interval
```

**Debugging Steps**:

1. **Check if data is available**:
```cpp
bool have_data = false;
for (uint8_t i = 0; i < BmsFleetCfg::MAX_MODULES; ++i) {
    if (fleet.has_any_data(i)) {
        have_data = true;
        break;
    }
}
```

2. **Monitor frame rotation**:
   - `frame_rotation = 0`: Should send fleet summary
   - `frame_rotation = 1`: Should send module summary
   - `frame_rotation = 2`: Should send heartbeat (if due)

3. **Check heartbeat timing**:
   - `g_lastHeartbeatInterval`: Should be ~1000ms
   - `ERROR_Pin`: Should toggle every second when heartbeat sent

4. **Verify transmission success**:
   - Check `HAL_UART_Transmit()` return value
   - Monitor for `HAL_OK` vs errors

**Breakpoints**:
- Line ~405: After fleet summary transmission
- Line ~428: After module summary transmission
- Line ~449: After heartbeat transmission

#### Primary MCU UART Reception

**Location**: `DistributedBMSPrimary/Core/Src/main.cpp`

**Key Debug Variables** (all volatile, watch in debugger):

```cpp
// Reception tracking
static volatile uint8_t  last_uart_type = 0;        // Message type received
static volatile uint16_t last_uart_len = 0;         // Payload length
static volatile uint32_t last_uart_tick = 0;        // Reception timestamp
static volatile uint8_t  last_update_status = 0;    // Update result
static volatile uint8_t  last_module_idx = 0xFF;    // Module index (for module summary)
static volatile uint32_t last_heartbeat_counter = 0; // Heartbeat counter

// Error tracking
static volatile uint32_t uart_error_count = 0;       // Total errors
static volatile uint32_t uart_last_error_flags = 0; // Last error code
```

**Debugging Steps**:

1. **Check if frames are arriving**:
   - `last_uart_type`: Should cycle between 0x10, 0x11, 0x12
   - `last_uart_tick`: Should update regularly
   - `last_uart_len`: Should match expected payload sizes (12, 18, or 4)

2. **Verify parsing success**:
   - `last_update_status = 1`: Fleet summary OK
   - `last_update_status = 3`: Module summary OK
   - `last_update_status = 5`: Heartbeat OK
   - `last_update_status = 2/4/6`: Parsing failed

3. **Monitor errors**:
   - `uart_error_count`: Should stay low (ideally 0)
   - `uart_last_error_flags = 8`: Overrun error (data too fast)
   - `uart_last_error_flags = 2`: Framing error
   - `uart_last_error_flags = 4`: Noise error

4. **Check data freshness**:
```cpp
if (fleet.has_data(HAL_GetTick())) {
    const auto& summary = fleet.summary();
    // Data is fresh
} else {
    // Data is stale (>2000ms old)
}
```

**Breakpoints**:
- Line ~102: Fleet summary received
- Line ~111: Module summary received
- Line ~119: Heartbeat received
- Line ~151: UART error callback

## Common Debugging Scenarios

### Scenario 1: No Data Received on Primary MCU

**Symptoms**:
- `last_uart_type` never changes
- `uart_error_count` increasing
- `fleet.has_data()` always false

**Debugging Steps**:
1. Check secondary is transmitting:
   - Verify `have_data` is true
   - Check `frame_rotation` is cycling
   - Monitor `HAL_UART_Transmit()` return values

2. Check UART hardware:
   - Verify wiring (TX→RX, RX→TX, GND)
   - Check baud rate matches (115200)
   - Verify UART initialization

3. Check error flags:
   - `uart_last_error_flags`: Identifies specific issue
   - Overrun (8): Increase delays on secondary
   - Framing (2): Check baud rate/stop bits

### Scenario 2: Intermittent Data Reception

**Symptoms**:
- `last_uart_type` updates sometimes
- `uart_error_count` slowly increasing
- `last_update_status` shows mix of success/failure

**Debugging Steps**:
1. Check timing:
   - `g_lastHeartbeatInterval`: Should be consistent
   - Frame spacing: Should be ~300ms minimum

2. Monitor error patterns:
   - If errors cluster: Check for EMI/noise
   - If errors random: Check wiring connections

3. Verify frame rotation:
   - All three frame types should be sent
   - Check `frame_rotation` cycles 0→1→2→0

### Scenario 3: Missing Module Summaries

**Symptoms**:
- Fleet summaries received
- Heartbeats received
- Module summaries never received

**Debugging Steps**:
1. Check secondary:
   - Verify `fleet.has_any_data(i)` for modules
   - Check `module_cursor` is incrementing
   - Verify module summary is in rotation (frame_rotation=1)

2. Check primary:
   - `last_module_idx`: Should show module index
   - `last_update_status = 3`: Successful module update
   - `fleet.module_valid(i)`: Should be true for received modules

### Scenario 4: Heartbeat Not Received

**Symptoms**:
- Fleet and module summaries received
- Heartbeat never received
- `last_update_status` never = 5

**Debugging Steps**:
1. Check secondary heartbeat timing:
   - `g_lastHeartbeatInterval`: Should be ~1000ms
   - `ERROR_Pin`: Should toggle every second
   - `heartbeat_counter`: Should increment

2. Check rotation:
   - Heartbeat only sent when `frame_rotation = 2`
   - Must wait for 1 second interval

3. Verify transmission:
   - Check `HAL_UART_Transmit()` return value
   - Verify heartbeat is in transmission buffer

## Debugging Tools

### Watch Expressions (Recommended)

**Primary MCU**:
```
fleet.summary().hottest_temp_C
fleet.summary().num_online_modules
last_uart_type
last_update_status
uart_error_count
uart_last_error_flags
```

**Secondary MCU**:
```
fleet.module(0).high_C
fleet.module(0).got_type0
fleet.module(0).got_type1
fleet.module(0).got_type2
frame_rotation
g_lastHeartbeatInterval
```

### Breakpoint Strategy

1. **Reception Breakpoints** (Primary):
   - `on_uart_packet()`: Entry point
   - Switch cases for each message type
   - `HAL_UART_ErrorCallback()`: Error handling

2. **Transmission Breakpoints** (Secondary):
   - After each `HAL_UART_Transmit()`
   - Frame rotation switch cases
   - Heartbeat timing check

### Logging Strategy

If printf/debug UART available:

**Secondary**:
```cpp
printf("TX: type=%d, len=%d, status=%d\n", 
       frame_type, txlen, status);
```

**Primary**:
```cpp
printf("RX: type=0x%02X, len=%d, status=%d, errors=%lu\n",
       last_uart_type, last_uart_len, 
       last_update_status, uart_error_count);
```

## Performance Monitoring

### Expected Timings

- **Fleet Summary**: ~Every 900ms
- **Module Summary**: ~Every 900ms (one module per cycle)
- **Heartbeat**: ~Every 1000ms
- **Frame Spacing**: Minimum 300ms between frames

### Health Indicators

**Good Health**:
- `uart_error_count` = 0 or very low
- `last_update_status` = 1, 3, or 5 (success)
- `fleet.has_data()` = true
- All three message types received regularly

**Poor Health**:
- `uart_error_count` increasing rapidly
- `last_update_status` = 2, 4, or 6 (failures)
- `uart_last_error_flags` = 8 (overrun)
- `fleet.has_data()` = false (stale data)

## Troubleshooting Checklist

- [ ] CAN messages arriving at secondary (check `got_type*` flags)
- [ ] Secondary has data to send (`have_data` = true)
- [ ] Frame rotation working (`frame_rotation` cycles)
- [ ] UART transmission successful (check return values)
- [ ] Primary receiving frames (`last_uart_type` updates)
- [ ] No UART errors (`uart_error_count` low)
- [ ] Data parsing successful (`last_update_status` = 1/3/5)
- [ ] Data structures updating (`fleet.summary()` changes)
- [ ] Timing correct (heartbeat ~1000ms, summaries ~900ms)


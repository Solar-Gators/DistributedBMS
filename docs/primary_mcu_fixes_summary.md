# Primary MCU Fixes Summary

## Changes Made

### 1. Created PrimaryBmsFleet Structure

**Files Created:**
- `DistributedBMSPrimary/Core/Inc/PrimaryBmsFleet.hpp`
- `DistributedBMSPrimary/Core/Src/PrimaryBmsFleet.cpp`

**Purpose:**
- Stores fleet summary data received from Secondary MCU via UART
- Similar structure to `BmsFleet` on Secondary MCU, but adapted for UART data
- Tracks fleet-wide statistics: hottest module, lowest cell, online modules count
- Includes timestamp tracking and latency calculation

**Key Features:**
- `FleetSummaryData` structure to hold received data
- `PrimaryBmsFleet` class to manage the data
- Online/stale data detection
- Latency calculation between Secondary and Primary MCU timestamps

### 2. Fixed Primary MCU main.cpp

**Changes:**
1. **Added include**: `#include "PrimaryBmsFleet.hpp"`
2. **Created fleet instance**: `static PrimaryBmsFleet fleet;`
3. **Implemented `on_uart_packet()` callback**:
   - Parses UART payload based on message type
   - Updates fleet data structure with received summary
   - Toggles LED on successful reception
   - Handles different message types (Fleet Summary, Module Summary, Heartbeat)
4. **Fixed UART initialization**:
   - Removed conflicting `HAL_UART_Receive_IT(&huart4, rx_buff, 1)`
   - Added `UART1_Packets_Init()` call after UART initialization
5. **Added example usage in main loop**:
   - Demonstrates accessing fleet data
   - Shows how to check for critical conditions (high temp, low voltage)
   - Calculates and monitors communication latency
   - Handles offline detection

### 3. Enhanced Secondary MCU UART Transmission

**Changes to `DistributedBMSSecondary/Core/Src/main.cpp`:**
1. **Frame rotation system**:
   - Rotates between fleet summary, module summary, and heartbeat
   - Sends only one frame type per cycle to prevent overrun errors
   - 50ms delay after each transmission
   - 250ms main loop delay

2. **Data availability check**:
   - Uses `fleet.has_any_data()` to check if modules have received CAN data
   - Only transmits when data is available

3. **Error handling**:
   - Checks return value of `HAL_UART_Transmit()`
   - Handles transmission errors gracefully

4. **Heartbeat timing**:
   - Uses `osKernelGetTickCount()` for reliable timing
   - Sends heartbeat once per second when due
   - Tracks actual interval in `g_lastHeartbeatInterval`

## Data Flow (Fixed)

```
Daughter Boards (CAN)
    ↓
Secondary MCU (BmsFleet aggregates data)
    ↓
Secondary MCU (UART transmission - Frame rotation system)
    ├─ Fleet Summary (~900ms)
    ├─ Module Summary (~900ms, round-robin)
    └─ Heartbeat (~1000ms)
    ↓
Primary MCU (UART reception - Interrupt-driven)
    ↓
Primary MCU (PrimaryBmsFleet stores data)
    ├─ FleetSummaryData
    ├─ ModuleSummaryData[8]
    └─ HeartbeatData
    ↓
Primary MCU (Main loop processes data)
```

## Usage Example

```cpp
// In Primary MCU main loop:
if (fleet.has_data(HAL_GetTick())) {
    const auto& summary = fleet.summary();
    
    // Access fleet data:
    float hottest_temp = summary.hottest_temp_C;
    uint16_t lowest_voltage = summary.lowest_cell_mV;
    uint8_t online_count = summary.num_online_modules;
    
    // Check latency
    uint32_t latency = summary.latency_ms(HAL_GetTick());
    
    // Access module data
    for (uint8_t i = 0; i < PrimaryBmsFleetCfg::MAX_MODULES; ++i) {
        if (fleet.module_valid(i)) {
            const auto& mod = fleet.module(i);
            // mod.high_temp_C, mod.avg_cell_mV, etc.
        }
    }
    
    // Check heartbeat
    if (fleet.heartbeat_valid()) {
        uint32_t counter = fleet.heartbeat().counter;
    }
    
    // Make decisions based on data
    if (hottest_temp > 45.0f) {
        // Handle high temperature
    }
}
```

## Debugging Variables

**Primary MCU** (watch in debugger):
- `last_uart_type`: Last received message type (0x10, 0x11, 0x12)
- `last_update_status`: Update result (1/3/5 = success, 2/4/6 = fail)
- `uart_error_count`: Total UART errors
- `uart_last_error_flags`: Last error code (8 = overrun)

**Secondary MCU**:
- `frame_rotation`: Current frame type (0=fleet, 1=module, 2=heartbeat)
- `g_lastHeartbeatInterval`: Actual heartbeat interval (~1000ms)

## Testing Checklist

- [ ] Verify UART frames are received on Primary MCU
- [ ] Check that `on_uart_packet()` is called when frames arrive
- [ ] Verify fleet data is updated correctly
- [ ] Test with no online modules (should not send from Secondary)
- [ ] Test UART transmission errors (disconnect cable)
- [ ] Verify latency calculation
- [ ] Test stale data detection (stop Secondary MCU)
- [ ] Verify LED toggles on successful reception

## Notes

- The Primary MCU now has a similar fleet data structure to the Secondary MCU
- Data is automatically parsed and stored when UART frames are received
- The structure is designed to be extensible for future message types
- Error handling is in place but can be enhanced with fault management if needed


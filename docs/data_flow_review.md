# Data Flow Review: Daughter Boards → Secondary MCU → Primary MCU

## Executive Summary

This document reviews the complete data flow from daughter boards through the secondary BMS MCU to the primary BMS MCU, identifying strengths, issues, and recommendations.

## Data Flow Architecture

### 1. Daughter Board → Secondary MCU (CAN Bus)

**Path**: `Daughter Board` → `CAN Bus (500 kbps)` → `Secondary MCU`

#### Transmission (Daughter Board)

**Location**: `DistributedBMSDaughter/Core/Src/main.cpp`

**Three Tasks Transmit Different Message Types**:

1. **StartDefaultTask** (Line 631-707):
   - Transmits: `AVERAGES` message (type 2)
   - Frequency: Every `CYCLE_TIME_MS` (250ms)
   - CAN ID: `0x101` (hardcoded)
   - Format: `[type=2][avgTemp:float][avgVoltage:uint16][numCells:uint8]`

2. **StartVoltageTask** (Line 709-757):
   - Transmits: `VOLTAGE_EXTREMES` message (type 1)
   - Frequency: Every `CYCLE_TIME_MS` (250ms)
   - CAN ID: `0x101`
   - Format: `[type=1][highV:uint16][lowV:uint16][lowIdx:uint8][highIdx:uint8]`

3. **StartTemperatureTask** (Line 759-819):
   - Transmits: `HIGH_TEMP` message (type 0)
   - Frequency: Every `CYCLE_TIME_MS * 4` (1000ms)
   - CAN ID: `0x101`
   - Format: `[type=0][highTemp:float][highIndex:uint8]`

**Frame Creation**: Uses `CanFrames::make_*` functions in `CanFrame.cpp` which properly encode data.

#### Reception (Secondary MCU)

**Location**: `DistributedBMSSecondary/Core/Src/main.cpp`

**CAN Callback System**:
- Line 82-86: `allCallback()` - catches all CAN messages
- Line 88-92: `daughterOneCallback()` - specific callback for ID 0x101
- Both callbacks invoke `fleet.handle(msg, HAL_GetTick())`

**CAN Filter Configuration** (Line 165-172):
```cpp
can.AddFilterRange(0x101, 4, SG_CAN_ID_STD, SG_CAN_RTR_DATA, 0);
can.addCallbackRange(0x101, 4, SG_CAN_ID_STD, daughterOneCallback, NULL);
can.AddFilterId(0x101, SG_CAN_ID_STD, SG_CAN_RTR_DATA, 0);
can.addCallbackId(0x101, SG_CAN_ID_STD, daughterOneCallback, NULL);
```

**Node Registration** (Line 173):
```cpp
fleet.register_node(0x101, 0);  // Maps CAN ID 0x101 to module index 0
```

**Data Processing** (`BmsFleet::handle()`):
- Decodes message type using `CanFrames::getType()`
- Updates `ModuleData` structure based on message type
- Tracks `last_ms` timestamp for online detection
- Sets flags: `got_type0`, `got_type1`, `got_type2`

### 2. Secondary MCU → Primary MCU (UART)

**Path**: `Secondary MCU` → `UART (115200 baud, 8N1)` → `Primary MCU`

#### Transmission (Secondary MCU)

**Location**: `DistributedBMSSecondary/Core/Src/main.cpp` (Line 369-388)

**Implementation**:
```cpp
void StartDefaultTask(void *argument) {
    uint8_t txbuf[64];
    for(;;) {
        size_t txlen = uart_make_fleet_summary(fleet, HAL_GetTick(), txbuf, sizeof(txbuf));
        if (txlen > 0) {
            HAL_UART_Transmit(&huart2, txbuf, txlen, 1000);
        }
        osDelay(200);
    }
}
```

**Frame Format** (from `UartFramer.cpp`):
- `[SOF0=0xA5][SOF1=0x5A][LEN_LOW][LEN_HIGH][PAYLOAD...][CRC16_LOW][CRC16_HIGH]`
- Payload: `FleetSummaryPayload` (11 bytes)
- Total frame: 17 bytes (6 overhead + 11 payload)

**Payload Content** (`UartFleetPack.hpp`):
- Type: `UART_FLEET_SUMMARY` (0x10)
- Hottest module index and temperature (°C × 10)
- Lowest cell module index and voltage (mV)
- Number of online modules
- Timestamp (ms)

#### Reception (Primary MCU)

**Location**: `DistributedBMSPrimary/Core/Src/main.cpp`

**UART Reception Setup**:
- Line 76-77: Static buffer and receiver structure
- Line 81-86: `on_uart_packet()` callback (currently empty!)
- Line 88-92: Initialization function (not called in main!)
- Line 95-100: `HAL_UART_RxCpltCallback()` properly wired
- Line 102-107: `HAL_UART_ErrorCallback()` properly wired

**Frame Parsing** (`UartRxPacket.cpp`):
- State machine: `RX_WAIT_HDR` → `RX_WAIT_BODY`
- SOF alignment handling
- CRC16-CCITT validation
- Calls `on_packet()` callback on valid frame

## Issues Identified

### 🔴 Critical Issues

1. **Primary MCU Not Processing Received Data**
   - **Location**: `DistributedBMSPrimary/Core/Src/main.cpp:81-86`
   - **Problem**: `on_uart_packet()` callback is empty - no processing of received fleet summaries
   - **Impact**: Primary MCU receives data but does nothing with it
   - **Fix Required**: Implement payload parsing and processing

2. **UART Initialization Not Called**
   - **Location**: `DistributedBMSPrimary/Core/Src/main.cpp:88-92`
   - **Problem**: `UART1_Packets_Init()` is defined but never called in `main()`
   - **Impact**: UART packet receiver not initialized, reception won't work
   - **Fix Required**: Call `UART1_Packets_Init()` after UART initialization

3. **Conflicting UART Receive Setup**
   - **Location**: `DistributedBMSPrimary/Core/Src/main.cpp:151`
   - **Problem**: `HAL_UART_Receive_IT(&huart4, rx_buff, 1)` conflicts with `UartPktRx` system
   - **Impact**: Two different receive mechanisms competing
   - **Fix Required**: Remove manual `HAL_UART_Receive_IT()` call, use `uart_pkt_start()`

### ⚠️ Medium Priority Issues

4. **Hardcoded CAN ID in Daughter Board**
   - **Location**: `DistributedBMSDaughter/Core/Src/main.cpp:634, 713, 763`
   - **Problem**: All messages use CAN ID `0x101` - not configurable per board
   - **Impact**: Can only have one daughter board per secondary MCU
   - **Recommendation**: Make CAN ID configurable (e.g., via DeviceConfig)

5. **Missing Error Handling in UART Transmission**
   - **Location**: `DistributedBMSSecondary/Core/Src/main.cpp:382`
   - **Problem**: `HAL_UART_Transmit()` return value not checked
   - **Impact**: Transmission failures go unnoticed
   - **Recommendation**: Check return value and handle errors

6. **Incomplete Module Registration**
   - **Location**: `DistributedBMSSecondary/Core/Src/main.cpp:173`
   - **Problem**: Only module 0x101 registered, comment says "Add Daughters 2-6 here"
   - **Impact**: Additional daughter boards won't be processed
   - **Recommendation**: Document how to add more modules

7. **No Data Freshness Validation**
   - **Location**: `DistributedBMSSecondary/Core/Src/main.cpp:379`
   - **Problem**: Fleet summary generated even if no modules are online
   - **Impact**: May send stale or invalid data
   - **Recommendation**: Check `num_online > 0` before transmitting

### 💡 Low Priority / Recommendations

8. **Timing Mismatch**
   - Daughter boards send at different rates (250ms, 250ms, 1000ms)
   - Secondary MCU sends UART at 200ms
   - **Recommendation**: Consider synchronizing or documenting timing relationships

9. **Missing Heartbeat/Keepalive**
   - No mechanism to detect UART link failure
   - **Recommendation**: Implement periodic heartbeat messages

10. **No Flow Control**
    - UART transmission uses blocking `HAL_UART_Transmit()` with 1000ms timeout
    - **Recommendation**: Consider non-blocking or DMA-based transmission

11. **Unused Code**
    - `SOF0` and `SOF1` definitions in secondary MCU main.cpp (lines 38-39) are unused
    - **Recommendation**: Remove or document why they exist

## Strengths

✅ **Well-Structured Frame Encoding**
- Proper separation: `CanFrames` for CAN, `UartFramer` for UART
- Consistent frame formats with CRC protection
- Type-safe encoding/decoding functions

✅ **Good Modularity**
- `BmsFleet` class cleanly aggregates module data
- Clear separation between CAN and UART layers
- Reusable frame encoding/decoding

✅ **Proper Timestamp Tracking**
- `last_ms` used for online detection
- Timestamp included in UART payload for latency measurement

✅ **CRC Protection**
- UART frames protected with CRC16-CCITT
- Proper validation on reception

✅ **State Machine for UART Reception**
- Robust frame synchronization
- Handles byte alignment issues

## Recommended Fixes

### Priority 1: Make Primary MCU Functional

```cpp
// In DistributedBMSPrimary/Core/Src/main.cpp

// Add payload structure (matching UartFleetPack.hpp)
typedef struct __attribute__((packed)) {
    uint8_t  type;
    uint8_t  hottest_idx;
    int16_t  hottest_c_x10;
    uint8_t  lowest_idx;
    uint16_t lowest_mV;
    uint8_t  num_online;
    uint32_t now_ms;
} FleetSummaryPayload;

static void on_uart_packet(const uint8_t* payload, uint16_t len, void* user)
{
    if (len < sizeof(FleetSummaryPayload)) return;
    if (payload[0] != 0x10) return; // UART_FLEET_SUMMARY
    
    const FleetSummaryPayload* p = (const FleetSummaryPayload*)payload;
    
    // Process the fleet summary data
    // TODO: Add your processing logic here
    // Example: Update display, trigger alarms, log data, etc.
}

int main(void) {
    // ... existing initialization ...
    
    MX_UART4_Init();
    UART1_Packets_Init();  // ← ADD THIS LINE
    
    // Remove: HAL_UART_Receive_IT(&huart4, rx_buff, 1);  // ← REMOVE THIS
    
    // ... rest of main ...
}
```

### Priority 2: Add Error Handling

```cpp
// In DistributedBMSSecondary/Core/Src/main.cpp
void StartDefaultTask(void *argument) {
    uint8_t txbuf[64];
    for(;;) {
        size_t txlen = uart_make_fleet_summary(fleet, HAL_GetTick(), txbuf, sizeof(txbuf));
        
        if (txlen > 0) {
            HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, txbuf, txlen, 1000);
            if (status != HAL_OK) {
                // Handle transmission error
                // Could set a fault flag, retry, etc.
            }
        }
        
        osDelay(200);
    }
}
```

### Priority 3: Validate Data Before Transmission

```cpp
void StartDefaultTask(void *argument) {
    uint8_t txbuf[64];
    for(;;) {
        // Check if we have any online modules
        uint8_t online_count = 0;
        for (uint8_t i = 0; i < BmsFleetCfg::MAX_MODULES; ++i) {
            if (fleet.module(i).online(HAL_GetTick())) {
                online_count++;
            }
        }
        
        if (online_count > 0) {
            size_t txlen = uart_make_fleet_summary(fleet, HAL_GetTick(), txbuf, sizeof(txbuf));
            if (txlen > 0) {
                HAL_UART_Transmit(&huart2, txbuf, txlen, 1000);
            }
        }
        
        osDelay(200);
    }
}
```

## Data Flow Diagram

```
┌─────────────────┐
│ Daughter Board  │
│  (3 Tasks)      │
│                 │
│ Task 1: AVERAGES│──┐
│ Task 2: VOLT    │──┤
│ Task 3: TEMP    │──┼──> CAN Bus (500 kbps, ID 0x101)
└─────────────────┘  │
                     │
                     ▼
┌─────────────────────────────────┐
│     Secondary MCU               │
│                                 │
│  CAN Callbacks                  │
│    └─> fleet.handle()          │
│                                 │
│  BmsFleet aggregates data       │
│    └─> ModuleData per CAN ID    │
│                                 │
│  UART Task (200ms)              │
│    └─> uart_make_fleet_summary()│
│    └─> HAL_UART_Transmit()      │──┐
└─────────────────────────────────┘  │
                                     │
                                     ▼
                          UART (115200, 8N1)
                                     │
                                     ▼
┌─────────────────────────────────┐
│      Primary MCU                │
│                                 │
│  UartPktRx (interrupt-based)    │
│    └─> on_uart_packet()        │
│         ⚠️ CURRENTLY EMPTY!     │
│                                 │
│  ⚠️ NOT PROCESSING DATA!        │
└─────────────────────────────────┘
```

## Testing Recommendations

1. **CAN Communication**
   - Verify all three message types are received
   - Check module registration works correctly
   - Test with multiple daughter boards (when CAN IDs are configurable)

2. **UART Communication**
   - Verify frame reception on primary MCU
   - Test CRC error handling
   - Verify frame alignment after errors
   - Test with missing bytes (simulate errors)

3. **End-to-End**
   - Send data from daughter board, verify it appears in primary MCU
   - Test timing: ensure data freshness
   - Test error scenarios: CAN bus failure, UART disconnection

## Conclusion

The data flow architecture is well-designed with good separation of concerns. The main issues are:

1. **Primary MCU is not processing received data** (critical)
2. **UART initialization not called** (critical)
3. **Conflicting UART receive mechanisms** (critical)

Once these are fixed, the system should function correctly. Additional improvements around error handling and configurability would enhance robustness.


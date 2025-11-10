# UART Communication Protocol

## Overview

The UART protocol enables communication between the **Secondary MCU** and **Primary MCU** in the Distributed BMS system. It uses a framed packet format with CRC protection to ensure reliable data transmission.

## Physical Layer

- **Baud Rate**: 115200
- **Data Bits**: 8
- **Parity**: None
- **Stop Bits**: 1
- **Flow Control**: None

## Frame Format

All UART frames follow this structure:

```
[SOF0] [SOF1] [LEN_LOW] [LEN_HIGH] [PAYLOAD...] [CRC16_LOW] [CRC16_HIGH]
```

- **SOF0**: Start of Frame byte 0 = `0xA5`
- **SOF1**: Start of Frame byte 1 = `0x5A`
- **LEN_LOW/LEN_HIGH**: Payload length (little-endian, 16-bit)
- **PAYLOAD**: Variable length data (see message types below)
- **CRC16**: CRC-16-CCITT checksum over `[LEN_LOW] [LEN_HIGH] [PAYLOAD...]`

**Total Frame Size**: 6 bytes overhead + payload length

## Message Types

### 1. Fleet Summary (Type 0x10)

**Purpose**: Aggregated fleet-wide statistics from all modules

**Payload Structure** (12 bytes total):
```c
typedef struct {
    uint8_t  type;           // = 0x10 (UART_FLEET_SUMMARY)
    uint8_t  hottest_idx;    // Module index with highest temp (0xFF = none)
    int16_t  hottest_c_x10;  // Highest temperature in °C × 10
    uint8_t  lowest_idx;     // Module index with lowest cell voltage (0xFF = none)
    uint16_t lowest_mV;      // Lowest cell voltage in mV
    uint8_t  num_online;     // Number of online modules
    uint32_t now_ms;         // Secondary MCU timestamp (for latency calculation)
} UartFleetSummaryPayload;
```

**Transmission Rate**: ~Every 900ms (rotated with other message types)

**Example**:
- Hottest module: Index 0, 35.2°C → `hottest_idx=0`, `hottest_c_x10=352`
- Lowest cell: Module 0, 3650mV → `lowest_idx=0`, `lowest_mV=3650`
- 1 module online → `num_online=1`

### 2. Module Summary (Type 0x11)

**Purpose**: Detailed snapshot of a single module's data

**Payload Structure** (18 bytes total):
```c
typedef struct {
    uint8_t  type;           // = 0x11 (UART_MODULE_SUMMARY)
    uint8_t  module_idx;     // Module index (0-7)
    int16_t  high_c_x10;     // Highest temperature in module (°C × 10)
    uint8_t  high_temp_cell; // Cell index with highest temperature
    uint16_t high_mV;        // Highest cell voltage (mV)
    uint16_t low_mV;         // Lowest cell voltage (mV)
    uint8_t  low_idx;        // Cell index with lowest voltage
    uint8_t  high_idx;       // Cell index with highest voltage
    int16_t  avg_c_x10;      // Average temperature (°C × 10)
    uint16_t avg_cell_mV;    // Average cell voltage (mV)
    uint8_t  num_cells;      // Number of cells (3-5)
    uint16_t age_ms;         // Data age (now_ms - last_ms), saturated to 65535
} UartModuleSummaryPayload;
```

**Transmission Rate**: ~Every 900ms (one module per cycle, round-robin)

**Note**: Only modules that have received CAN data are included in rotation

### 3. Heartbeat (Type 0x12)

**Purpose**: Link health monitoring and keepalive

**Payload Structure** (4 bytes total):
```c
typedef struct {
    uint8_t type;            // = 0x12 (UART_HEARTBEAT)
    uint8_t counter_lsb;     // Counter bits 0-7
    uint8_t counter_mid;     // Counter bits 8-15
    uint8_t counter_msb;     // Counter bits 16-23
} UartHeartbeatPayload;
```

**Transmission Rate**: Once per second (when due, rotated with other messages)

**Counter**: Increments on each transmission, wraps at 24 bits (16,777,216)

## Transmission Strategy (Secondary MCU)

The Secondary MCU uses a **frame rotation system** to prevent UART overrun errors:

**Location**: `DistributedBMSSecondary/Core/Src/main.cpp` - `StartDefaultTask()`

**Rotation Pattern**:
1. **Cycle 1**: Send Fleet Summary (if data available)
2. **Cycle 2**: Send Module Summary (if data available, round-robin)
3. **Cycle 3**: Send Heartbeat (if 1 second elapsed)
4. **Cycle 4**: Back to Fleet Summary...

**Timing**:
- Main loop delay: 250ms
- Post-transmission delay: 50ms (after successful send)
- Effective frame spacing: ~300ms minimum between frames

**Code Flow**:
```cpp
static uint8_t frame_rotation = 0;  // 0=fleet, 1=module, 2=heartbeat

for(;;) {
    switch (frame_rotation) {
    case 0:  // Fleet summary
        if (have_data) {
            uart_make_fleet_summary(...);
            HAL_UART_Transmit(...);
            osDelay(50);  // Give receiver time
        }
        frame_rotation = 1;
        break;
    case 1:  // Module summary
        // Round-robin through modules
        frame_rotation = 2;
        break;
    case 2:  // Heartbeat
        if (1 second elapsed) {
            uart_make_heartbeat(...);
            HAL_UART_Transmit(...);
            osDelay(50);
        }
        frame_rotation = 0;
        break;
    }
    osDelay(250);
}
```

## Reception Strategy (Primary MCU)

**Location**: `DistributedBMSPrimary/Core/Src/main.cpp`

**Reception Method**: Interrupt-driven state machine

**Key Components**:
1. **UartPktRx** (`UartRxPacket.cpp`): Handles frame synchronization and CRC validation
2. **on_uart_packet()**: Callback invoked when valid frame received
3. **PrimaryBmsFleet**: Stores and manages received data

**Reception Flow**:
```
HAL_UART_RxCpltCallback() 
    → uart_pkt_on_rx_cplt()
        → State machine processes bytes
            → On valid frame: on_uart_packet() callback
                → Parse message type
                → Update PrimaryBmsFleet structure
```

**Error Handling**:
- `HAL_UART_ErrorCallback()` captures UART errors
- `uart_error_count`: Total error count
- `uart_last_error_flags`: Last error code (8 = overrun, 2 = framing, 4 = noise)
- Errors trigger frame resynchronization

## Data Structures

### Shared Definitions

**Location**: `DistributedBMSCommon/Inc/UartFleetTypes.hpp`

All payload structures are defined here to ensure sender and receiver use identical layouts. Both MCUs include this header.

**Key Functions**:
- `uart_to_cdeg10(float c)`: Convert °C to °C×10 (int16_t)
- `uart_from_cdeg10(int16_t c_x10)`: Convert °C×10 to °C (float)

### Primary MCU Data Storage

**Location**: `DistributedBMSPrimary/Core/Inc/PrimaryBmsFleet.hpp`

**FleetSummaryData**:
- Stores fleet summary information
- Tracks timestamps for latency calculation
- Provides `is_online()` to check data freshness

**ModuleSummaryData**:
- Array of 8 module summaries
- Each entry tracks detailed module statistics
- `valid` flag indicates if data has been received

**HeartbeatData**:
- Stores heartbeat counter
- Tracks last update time

## Debugging Variables

### Primary MCU (Volatile, accessible in debugger)

**Location**: `DistributedBMSPrimary/Core/Src/main.cpp`

```cpp
static volatile uint8_t  last_uart_type = 0;        // Last received message type
static volatile uint16_t last_uart_len = 0;         // Last received payload length
static volatile uint32_t last_uart_tick = 0;        // Timestamp of last reception
static volatile uint8_t  last_update_status = 0;    // Update result (see below)
static volatile uint8_t  last_module_idx = 0xFF;    // Last module index received
static volatile uint32_t last_heartbeat_counter = 0; // Last heartbeat counter
static volatile uint32_t uart_error_count = 0;      // Total UART errors
static volatile uint32_t uart_last_error_flags = 0; // Last error code
```

**last_update_status Values**:
- `0`: No update yet
- `1`: Fleet summary updated successfully
- `2`: Fleet summary update failed
- `3`: Module summary updated successfully
- `4`: Module summary update failed
- `5`: Heartbeat updated successfully
- `6`: Heartbeat update failed

### Secondary MCU

**Location**: `DistributedBMSSecondary/Core/Src/main.cpp`

```cpp
static volatile uint32_t g_lastHeartbeatInterval = 0; // Actual heartbeat interval
```

## Common Issues and Solutions

### Overrun Errors (Error Flag = 8)

**Symptom**: `uart_last_error_flags = 8` (HAL_UART_ERROR_ORE)

**Cause**: Data arriving faster than receiver can process

**Solutions**:
1. ✅ Frame rotation (already implemented)
2. ✅ Post-transmission delays (already implemented)
3. Increase delays further if needed
4. Check receiver buffer size

### Missing Heartbeats

**Symptom**: Heartbeat not received regularly

**Check**:
1. Verify `g_lastHeartbeatInterval` on secondary (~1000ms)
2. Check `ERROR_Pin` toggle on secondary (should toggle every second)
3. Verify `last_update_status = 5` on primary
4. Check for UART errors blocking reception

### Stale Data

**Symptom**: `fleet.has_data()` returns false

**Cause**: No frames received within stale timeout (2000ms)

**Check**:
1. Verify secondary is transmitting (`have_data` flag)
2. Check UART wiring/baud rate
3. Monitor `uart_error_count` for communication issues
4. Verify frame rotation is working

### Module Data Not Updating

**Symptom**: Module summaries not received

**Check**:
1. Verify CAN data is arriving (check `fleet.has_any_data()` on secondary)
2. Check `module_cursor` rotation
3. Verify module index in `last_module_idx`
4. Check `last_update_status = 3` for successful updates

## Timing Diagram

```
Secondary MCU Task (250ms cycle):
┌─────────────────────────────────────────────────────────┐
│ Cycle 1 (0ms):    Fleet Summary → 50ms delay → 250ms   │
│ Cycle 2 (300ms):  Module Summary → 50ms delay → 250ms  │
│ Cycle 3 (600ms):  Heartbeat → 50ms delay → 250ms       │
│ Cycle 4 (900ms):  Fleet Summary → 50ms delay → 250ms    │
└─────────────────────────────────────────────────────────┘

Primary MCU (Interrupt-driven):
┌─────────────────────────────────────────────────────────┐
│ Frame arrives → Interrupt → State machine → Callback    │
│ → Parse → Update data structure                         │
└─────────────────────────────────────────────────────────┘
```

## Testing Checklist

- [ ] Verify fleet summaries received regularly (~900ms)
- [ ] Verify module summaries received (round-robin)
- [ ] Verify heartbeat received (~1000ms)
- [ ] Check `uart_error_count` stays low
- [ ] Verify `last_update_status` shows successful updates
- [ ] Test with UART cable disconnected (should see errors)
- [ ] Test with secondary stopped (should see stale data)
- [ ] Verify latency calculation works
- [ ] Check all three message types in debugger

## Code Locations

**Secondary MCU**:
- Transmission: `DistributedBMSSecondary/Core/Src/main.cpp` - `StartDefaultTask()`
- Frame encoding: `DistributedBMSSecondary/Core/Inc/UartFleetPack.hpp`
- Frame formatting: `DistributedBMSSecondary/Core/Src/UartFramer.cpp`

**Primary MCU**:
- Reception: `DistributedBMSPrimary/Core/Src/main.cpp` - `on_uart_packet()`
- Frame parsing: `DistributedBMSPrimary/Core/Src/UartRxPacket.cpp`
- Data storage: `DistributedBMSPrimary/Core/Inc/PrimaryBmsFleet.hpp`

**Shared**:
- Payload definitions: `DistributedBMSCommon/Inc/UartFleetTypes.hpp`


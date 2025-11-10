# Distributed BMS Documentation

## Quick Start

1. **Architecture Overview**: See `architecture.md`
2. **Data Flow**: See `data_flow_review.md`
3. **UART Protocol**: See `uart_protocol.md` (NEW - Complete UART documentation)
4. **Debugging**: See `debugging_guide.md` (NEW - Comprehensive debugging guide)
5. **CAN Protocol**: See `can_protocol.md`
6. **API Reference**: See `api_reference.md`

## Documentation Files

### Core Documentation

- **`architecture.md`**: System architecture and component overview
- **`data_flow_review.md`**: Complete data flow from daughter boards to primary MCU
- **`uart_protocol.md`**: ⭐ **NEW** - Complete UART communication protocol documentation
- **`debugging_guide.md`**: ⭐ **NEW** - Step-by-step debugging guide with variables and strategies
- **`can_protocol.md`**: CAN bus protocol and message formats
- **`api_reference.md`**: API documentation for all classes and functions

### Configuration & Setup

- **`build_guide.md`**: Build instructions and project setup
- **`device_configuration.md`**: Device configuration parameters
- **`fault_management.md`**: Fault detection and handling
- **`data_validation.md`**: Data validation strategies

### Implementation Notes

- **`primary_mcu_fixes_summary.md`**: Summary of Primary MCU implementation fixes

## Key Features

### Communication Layers

1. **CAN Bus** (Daughter → Secondary):
   - 500 kbps
   - Three message types per daughter board
   - Filtered reception on secondary

2. **UART** (Secondary → Primary):
   - 115200 baud, 8N1
   - Framed packets with CRC protection
   - Frame rotation system prevents overrun
   - Three message types: Fleet Summary, Module Summary, Heartbeat

### Data Structures

- **BmsFleet** (Secondary): Aggregates CAN data from daughter boards
- **PrimaryBmsFleet** (Primary): Stores UART data from secondary MCU
- **Shared Types**: `DistributedBMSCommon/Inc/UartFleetTypes.hpp`

## Recent Updates

### UART Protocol Improvements

- ✅ Frame rotation system to prevent overrun errors
- ✅ Heartbeat and module summary support
- ✅ Comprehensive error tracking
- ✅ Shared payload definitions for type safety

### Debugging Enhancements

- ✅ Volatile debug variables on both MCUs
- ✅ Error flag tracking
- ✅ Timing interval monitoring
- ✅ Status code system for update results

## Quick Reference

### Message Types

| Type | ID | Payload Size | Frequency |
|------|-----|--------------|-----------|
| Fleet Summary | 0x10 | 12 bytes | ~900ms |
| Module Summary | 0x11 | 18 bytes | ~900ms (round-robin) |
| Heartbeat | 0x12 | 4 bytes | ~1000ms |

### Debug Variables (Primary MCU)

```cpp
last_uart_type          // Last message type received
last_update_status      // Update result (1/3/5 = success)
uart_error_count        // Total errors
uart_last_error_flags   // Last error (8 = overrun)
```

### Debug Variables (Secondary MCU)

```cpp
frame_rotation          // Current frame type (0/1/2)
g_lastHeartbeatInterval // Heartbeat interval
```

## Troubleshooting

See `debugging_guide.md` for:
- Common issues and solutions
- Step-by-step debugging procedures
- Watch expressions and breakpoints
- Performance monitoring

## Code Locations

### Secondary MCU
- UART Transmission: `DistributedBMSSecondary/Core/Src/main.cpp` - `StartDefaultTask()`
- Fleet Management: `DistributedBMSSecondary/Core/Inc/BmsFleet.hpp`
- Frame Encoding: `DistributedBMSSecondary/Core/Inc/UartFleetPack.hpp`

### Primary MCU
- UART Reception: `DistributedBMSPrimary/Core/Src/main.cpp` - `on_uart_packet()`
- Data Storage: `DistributedBMSPrimary/Core/Inc/PrimaryBmsFleet.hpp`
- Frame Parsing: `DistributedBMSPrimary/Core/Src/UartRxPacket.cpp`

### Shared
- Payload Definitions: `DistributedBMSCommon/Inc/UartFleetTypes.hpp`


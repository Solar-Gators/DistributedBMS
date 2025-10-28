# Device Configuration Guide

## Overview

The Distributed BMS system now includes a configurable device ID system that makes it easy to set up multiple daughter boards with unique CAN IDs. This guide explains how to configure each daughter board.

## Quick Start

### For Daughter Boards

1. **Open** `DistributedBMSDaughter/Core/Inc/DeviceConfig.hpp`
2. **Change** the `DEVICE_NUMBER` value (1-8)
3. **Rebuild** and flash the firmware
4. **Done!** The CAN ID is automatically calculated

### For Secondary Board

The secondary board automatically registers all possible daughter boards (1-8) and will receive data from whichever ones are connected.

## Configuration Files

### Daughter Board Configuration
**File**: `DistributedBMSDaughter/Core/Inc/DeviceConfig.hpp`

```cpp
namespace DeviceConfig {
    // Change this number to set the device ID
    static constexpr uint8_t DEVICE_NUMBER = 1;  // 1, 2, 3, 4, 5, 6, 7, 8
    
    // These are calculated automatically
    static constexpr uint16_t BASE_CAN_ID = 0x100;
    static constexpr uint16_t CAN_ID = BASE_CAN_ID + DEVICE_NUMBER;
    
    // Other configurable parameters
    static constexpr uint8_t CELL_COUNT = 4;        // Number of cells (3-5)
    static constexpr uint32_t CYCLE_TIME_MS = 250;   // Main loop cycle time
    static constexpr bool ENABLE_DEBUG_PRINTS = false;  // Debug output
    static constexpr bool ENABLE_CAN_MONITORING = false; // CAN monitoring
}
```

### Secondary Board Configuration
**File**: `DistributedBMSSecondary/Core/Inc/SecondaryConfig.hpp`

```cpp
namespace SecondaryConfig {
    static constexpr uint16_t BASE_CAN_ID = 0x100;  // Base CAN ID for daughter boards
    static constexpr uint8_t MAX_DAUGHTER_BOARDS = 8;  // Maximum daughter boards
    
    // Daughter board CAN IDs (automatically calculated)
    static constexpr uint16_t DAUGHTER_1_CAN_ID = 0x101;
    static constexpr uint16_t DAUGHTER_2_CAN_ID = 0x102;
    // ... and so on
    
    static constexpr bool ENABLE_DEBUG_PRINTS = false;    // Debug output
    static constexpr bool ENABLE_CAN_MONITORING = false;  // CAN monitoring
}
```

## CAN ID Assignment

| Device Number | CAN ID | Description |
|---------------|--------|-------------|
| 1 | 0x101 | Daughter board 1 |
| 2 | 0x102 | Daughter board 2 |
| 3 | 0x103 | Daughter board 3 |
| 4 | 0x104 | Daughter board 4 |
| 5 | 0x105 | Daughter board 5 |
| 6 | 0x106 | Daughter board 6 |
| 7 | 0x107 | Daughter board 7 |
| 8 | 0x108 | Daughter board 8 |

## Configuration Examples

### Example 1: Device 1 (4-cell, Debug Enabled)
```cpp
namespace DeviceConfig {
    static constexpr uint8_t DEVICE_NUMBER = 1;
    static constexpr uint8_t CELL_COUNT = 4;
    static constexpr bool ENABLE_DEBUG_PRINTS = true;
    static constexpr bool ENABLE_CAN_MONITORING = true;
}
```
**Result**: CAN ID = 0x101, 4-cell monitoring, debug output enabled

### Example 2: Device 3 (5-cell, Production Mode)
```cpp
namespace DeviceConfig {
    static constexpr uint8_t DEVICE_NUMBER = 3;
    static constexpr uint8_t CELL_COUNT = 5;
    static constexpr bool ENABLE_DEBUG_PRINTS = false;
    static constexpr bool ENABLE_CAN_MONITORING = false;
}
```
**Result**: CAN ID = 0x103, 5-cell monitoring, no debug output

### Example 3: Device 4 (3-cell, Fast Cycle)
```cpp
namespace DeviceConfig {
    static constexpr uint8_t DEVICE_NUMBER = 4;
    static constexpr uint8_t CELL_COUNT = 3;
    static constexpr uint32_t CYCLE_TIME_MS = 100;  // Faster cycle
    static constexpr bool ENABLE_DEBUG_PRINTS = false;
}
```
**Result**: CAN ID = 0x104, 3-cell monitoring, 100ms cycle time

## Debug Features

### Debug Output
When `ENABLE_DEBUG_PRINTS = true`, the system will output:

```
=== Device Configuration ===
Device Number: 1
CAN ID: 0x101
Cell Count: 4
Cycle Time: 250 ms
===========================
```

### CAN Monitoring
When `ENABLE_CAN_MONITORING = true`, the system will output:

```
CAN TX OK: HIGH_TEMP (ID: 0x101)
CAN TX OK: VOLTAGE_EXTREMES (ID: 0x101)
CAN TX OK: AVERAGES (ID: 0x101)
```

## Configuration Workflow

### Setting Up Multiple Daughter Boards

1. **Device 1**:
   - Set `DEVICE_NUMBER = 1` in DeviceConfig.hpp
   - Build and flash firmware
   - CAN ID will be 0x101

2. **Device 2**:
   - Set `DEVICE_NUMBER = 2` in DeviceConfig.hpp
   - Build and flash firmware
   - CAN ID will be 0x102

3. **Continue** for additional devices...

### Secondary Board Setup

The secondary board automatically registers all possible daughter boards (0x101-0x108) and will receive data from whichever ones are connected. No configuration changes needed.

## Validation

The configuration system includes compile-time validation:

- Device number must be between 1 and 8
- Calculated CAN ID must not exceed 11-bit limit (0x7FF)
- Cell count must be between 3 and 5

If validation fails, the code will not compile.

## Troubleshooting

### Common Issues

1. **Compilation Errors**:
   - Check that `DEVICE_NUMBER` is between 1-8
   - Verify all required headers are included

2. **CAN Communication Issues**:
   - Verify CAN ID is unique for each device
   - Check CAN bus termination (120Ω resistors)
   - Enable CAN monitoring for debugging

3. **Wrong CAN ID**:
   - Double-check `DEVICE_NUMBER` setting
   - Verify `BASE_CAN_ID` is correct (0x100)
   - Check calculated CAN ID in debug output

### Debug Commands

Enable debug output to verify configuration:

```cpp
// In DeviceConfig.hpp
static constexpr bool ENABLE_DEBUG_PRINTS = true;
static constexpr bool ENABLE_CAN_MONITORING = true;
```

This will show:
- Device configuration at startup
- CAN transmission status
- Received CAN messages (on secondary board)

## Best Practices

1. **Use Sequential Device Numbers**: Start with device 1, then 2, 3, etc.
2. **Enable Debug During Development**: Set debug flags to true during testing
3. **Disable Debug in Production**: Set debug flags to false for final deployment
4. **Document Your Configuration**: Keep track of which device number corresponds to which physical board
5. **Test Each Device**: Verify each daughter board transmits with the correct CAN ID

## Advanced Configuration

### Custom Base CAN ID
If you need to change the base CAN ID (e.g., to avoid conflicts):

```cpp
// In DeviceConfig.hpp
static constexpr uint16_t BASE_CAN_ID = 0x200;  // Custom base ID
// CAN IDs will be: 0x201, 0x202, 0x203, etc.
```

**Important**: You must also update the secondary board configuration to match:

```cpp
// In SecondaryConfig.hpp
static constexpr uint16_t BASE_CAN_ID = 0x200;  // Must match daughter boards
```

### Custom Cycle Times
For different applications, you might want different cycle times:

```cpp
// Fast monitoring (100ms)
static constexpr uint32_t CYCLE_TIME_MS = 100;

// Standard monitoring (250ms)
static constexpr uint32_t CYCLE_TIME_MS = 250;

// Slow monitoring (500ms)
static constexpr uint32_t CYCLE_TIME_MS = 500;
```

This configuration system makes it easy to deploy multiple daughter boards with unique identities while maintaining a consistent codebase.


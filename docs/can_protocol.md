# CAN Communication Protocol Specification

## Overview

The Distributed BMS system uses CAN (Controller Area Network) bus for communication between daughter boards and the secondary board. This document specifies the CAN protocol implementation, message formats, and communication procedures.

## Physical Layer

### CAN Bus Configuration

- **Standard**: CAN 2.0B
- **Bit Rate**: 500 kbps
- **Physical Layer**: ISO 11898-2 (High-Speed CAN)
- **Termination**: 120Ω resistors at both ends of the bus
- **Connector**: Standard CAN bus connector (9-pin D-sub)

### Electrical Characteristics

| Parameter | Value | Notes |
|-----------|-------|-------|
| Bit Rate | 500 kbps | Fixed rate for all nodes |
| Bus Length | Up to 100m | With proper termination |
| Node Count | Up to 8 | Daughter boards per secondary |
| Voltage Levels | 0V (Recessive), 3.3V (Dominant) | Differential signaling |
| Termination | 120Ω | Required at bus ends |

## Message Format

### CAN Frame Structure

```
┌─────────────┬─────────────┬─────────────┬─────────────┐
│   CAN ID    │    DLC      │    Type     │    Data     │
│  (11-bit)   │   (4-bit)   │   (1-byte)  │  (7-bytes)  │
└─────────────┴─────────────┴─────────────┴─────────────┘
```

### Frame Components

#### CAN ID (11-bit Standard)
- **Range**: 0x000 - 0x7FF
- **Usage**: Identifies the source daughter board
- **Assignment**: Each daughter board has a unique ID
- **Example**: Daughter board 1 uses ID 0x101

#### Data Length Code (DLC)
- **Value**: Always 8 (0x8)
- **Purpose**: Indicates 8-byte payload
- **Fixed**: All BMS messages use full 8-byte payload

#### Message Type (Byte 0)
- **Purpose**: Identifies the type of data in the message
- **Values**:
  - `0x00`: HIGH_TEMP
  - `0x01`: VOLTAGE_EXTREMES  
  - `0x02`: AVERAGES

#### Data Payload (Bytes 1-7)
- **Purpose**: Contains the actual measurement data
- **Format**: Depends on message type (see below)

## Message Types

### 1. HIGH_TEMP Message

**Purpose**: Transmits highest temperature reading and sensor index

**Format**:
```
Byte 0: Message Type (0x00)
Byte 1-4: Temperature (float, little-endian)
Byte 5: Temperature sensor index (0-4)
Byte 6-7: Reserved (0x00)
```

**Example**:
```cpp
// Temperature: 25.5°C, Sensor index: 2
uint8_t data[8] = {
    0x00,           // HIGH_TEMP message type
    0x00, 0x00, 0xCC, 0x41,  // 25.5f as float
    0x02,           // Sensor index 2
    0x00, 0x00      // Reserved
};
```

**Encoding Function**:
```cpp
Frame encodeHighTemp(float highTemp, uint8_t highIndex) {
    Frame f{};
    f.data[0] = HIGH_TEMP;
    std::memcpy(&f.data[1], &highTemp, 4);
    f.data[5] = highIndex;
    return f;
}
```

**Decoding Function**:
```cpp
bool decodeHighTemp(const uint8_t* data, float& temp, uint8_t& idx) {
    if (data[0] != HIGH_TEMP) return false;
    std::memcpy(&temp, &data[1], 4);
    idx = data[5];
    return true;
}
```

### 2. VOLTAGE_EXTREMES Message

**Purpose**: Transmits highest and lowest cell voltages with their indices

**Format**:
```
Byte 0: Message Type (0x01)
Byte 1-2: Highest voltage (uint16_t, little-endian)
Byte 3-4: Lowest voltage (uint16_t, little-endian)
Byte 5: Lowest voltage cell index (0-4)
Byte 6: Highest voltage cell index (0-4)
Byte 7: Reserved (0x00)
```

**Example**:
```cpp
// High: 3720mV (cell 2), Low: 3650mV (cell 1)
uint8_t data[8] = {
    0x01,           // VOLTAGE_EXTREMES message type
    0x88, 0x0E,     // 3720 as uint16_t
    0x44, 0x0E,     // 3650 as uint16_t
    0x01,           // Low voltage cell index
    0x02,           // High voltage cell index
    0x00            // Reserved
};
```

**Encoding Function**:
```cpp
Frame encodeVoltageExtremes(uint16_t highV, uint16_t lowV,
                          uint8_t lowIdx, uint8_t highIdx) {
    Frame f{};
    f.data[0] = VOLTAGE_EXTREMES;
    f.data[1] = highV & 0xFF;
    f.data[2] = highV >> 8;
    f.data[3] = lowV & 0xFF;
    f.data[4] = lowV >> 8;
    f.data[5] = lowIdx;
    f.data[6] = highIdx;
    return f;
}
```

**Decoding Function**:
```cpp
bool decodeVoltageExtremes(const uint8_t* data, uint16_t& highV,
                         uint16_t& lowV, uint8_t& lowIdx, uint8_t& highIdx) {
    if (data[0] != VOLTAGE_EXTREMES) return false;
    highV = (data[2] << 8) | data[1];
    lowV  = (data[4] << 8) | data[3];
    lowIdx = data[5];
    highIdx = data[6];
    return true;
}
```

### 3. AVERAGES Message

**Purpose**: Transmits average temperature, average voltage, and cell count

**Format**:
```
Byte 0: Message Type (0x02)
Byte 1-4: Average temperature (float, little-endian)
Byte 5-6: Average voltage (uint16_t, little-endian)
Byte 7: Number of cells (uint8_t)
```

**Example**:
```cpp
// Avg temp: 26.0°C, Avg voltage: 3685mV, 4 cells
uint8_t data[8] = {
    0x02,           // AVERAGES message type
    0x00, 0x00, 0xD0, 0x41,  // 26.0f as float
    0x65, 0x0E,     // 3685 as uint16_t
    0x04            // 4 cells
};
```

**Encoding Function**:
```cpp
Frame encodeAverages(float avgTemp, uint16_t avgVoltage, uint8_t numCells) {
    Frame f{};
    f.data[0] = AVERAGES;
    std::memcpy(&f.data[1], &avgTemp, 4);
    f.data[5] = avgVoltage & 0xFF;
    f.data[6] = avgVoltage >> 8;
    f.data[7] = numCells;
    return f;
}
```

**Decoding Function**:
```cpp
bool decodeAverages(const uint8_t* data, float& avgTemp,
                   uint16_t& avgVoltage, uint8_t& numCells) {
    if (data[0] != AVERAGES) return false;
    std::memcpy(&avgTemp, &data[1], 4);
    avgVoltage = (data[6] << 8) | data[5];
    numCells = data[7];
    return true;
}
```

## Communication Procedures

### Daughter Board Transmission

Each daughter board transmits three messages per cycle:

```cpp
// Daughter board main loop
while (1) {
    // ... data collection ...
    
    // Build messages
    auto f0 = CanFrames::make_high_temp(results);
    auto f1 = CanFrames::make_voltage_extremes(results);
    auto f2 = CanFrames::make_average_stats(results);
    
    // Transmit messages
    can.sendStd(0x07, f0.bytes, f0.dlc);
    can.sendStd(0x07, f1.bytes, f1.dlc);
    can.sendStd(0x07, f2.bytes, f2.dlc);
    
    HAL_Delay(250); // 250ms cycle time
}
```

### Secondary Board Reception

The secondary board receives and processes messages from all daughter boards:

```cpp
// Secondary board main loop
while (1) {
    CanBus::Frame rx;
    if (can.read(rx)) {
        fleet.handle(rx, HAL_GetTick());
    }
}
```

### Message Processing

The secondary board processes received messages based on type:

```cpp
void BmsFleet::handle(const CanBus::Frame& rx, uint32_t now_ms) {
    int mi = find_module_index(rx.id);
    if (mi < 0) return; // Not registered
    
    ModuleData& M = modules_[mi];
    const uint8_t* b = rx.data;
    
    switch (CanFrames::getType(b)) {
    case CanFrames::HIGH_TEMP: {
        float t; uint8_t idx;
        if (CanFrames::decodeHighTemp(b, t, idx)) {
            M.high_C = t;
            M.high_temp_idx = idx;
            M.last_ms = now_ms;
            M.got_type0 = true;
        }
    } break;
    
    case CanFrames::VOLTAGE_EXTREMES: {
        uint16_t hv, lv; uint8_t li, hi;
        if (CanFrames::decodeVoltageExtremes(b, hv, lv, li, hi)) {
            M.high_mV = hv; M.low_mV = lv;
            M.low_idx = li; M.high_idx = hi;
            M.last_ms = now_ms;
            M.got_type1 = true;
        }
    } break;
    
    case CanFrames::AVERAGES: {
        float at; uint16_t av; uint8_t nc;
        if (CanFrames::decodeAverages(b, at, av, nc)) {
            M.avg_C = at; M.avg_cell_mV = av; M.num_cells = nc;
            M.last_ms = now_ms;
            M.got_type2 = true;
        }
    } break;
    }
}
```

## Error Handling

### CAN Bus Errors

The system handles various CAN bus error conditions:

#### 1. Transmission Errors
```cpp
if (can.sendStd(0x07, data, 8) != CanBus::Result::Ok) {
    faultManager.setFault(FaultManager::FaultType::CAN_TRANSMIT_ERROR, true);
} else {
    faultManager.clearFault(FaultManager::FaultType::CAN_TRANSMIT_ERROR);
}
```

#### 2. Reception Errors
```cpp
// Check for dropped frames
if (can.rx_dropped() > 0) {
    faultManager.setFault(FaultManager::FaultType::CAN_RECEIVE_ERROR, true);
}
```

#### 3. Bus Off Recovery
```cpp
// Monitor bus status
if (HAL_CAN_GetError(&hcan1) != HAL_CAN_ERROR_NONE) {
    // Handle bus errors
    HAL_CAN_ResetError(&hcan1);
}
```

### Message Validation

All received messages are validated before processing:

```cpp
bool validateMessage(const CanBus::Frame& frame) {
    // Check DLC
    if (frame.dlc != 8) return false;
    
    // Check message type
    uint8_t type = CanFrames::getType(frame.data);
    if (type > 2) return false;
    
    // Check reserved bytes
    if (frame.data[6] != 0 || frame.data[7] != 0) return false;
    
    return true;
}
```

## Timing Requirements

### Transmission Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| Cycle Time | 250ms | Daughter board main loop |
| Message Interval | ~83ms | 3 messages per cycle |
| Transmission Time | <10ms | Per message |
| Bus Utilization | <5% | At 500 kbps |

### Reception Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| Processing Time | <1ms | Per received message |
| Queue Depth | 16 frames | Circular buffer size |
| Timeout | 1500ms | Module offline detection |

## CAN ID Assignment

### Daughter Board IDs

Each daughter board must have a unique CAN ID:

| Daughter Board | CAN ID | Description |
|----------------|--------|-------------|
| Board 1 | 0x101 | First daughter board |
| Board 2 | 0x102 | Second daughter board |
| Board 3 | 0x103 | Third daughter board |
| ... | ... | ... |
| Board 8 | 0x108 | Eighth daughter board |

### Secondary Board ID

The secondary board uses ID 0x200 for any responses or status messages.

### Reserved IDs

| ID Range | Purpose | Notes |
|----------|---------|-------|
| 0x000-0x100 | System messages | Reserved for system use |
| 0x109-0x1FF | Future expansion | Available for additional boards |
| 0x201-0x7FF | Future use | Available for other purposes |

## Filter Configuration

### Daughter Board Filters

Daughter boards configure filters to receive system messages:

```cpp
// Accept all messages (for system commands)
can.configureFilterAcceptAll();
```

### Secondary Board Filters

Secondary board configures filters to receive daughter board messages:

```cpp
// Filter for daughter board messages (0x101-0x108)
can.configureFilterStdMask(0x100, 0x1F8); // Mask: 0x1F8 = 1111111000
```

## Debugging and Monitoring

### CAN Bus Analyzer Integration

The protocol is designed to work with standard CAN bus analyzers:

#### Message Monitoring
```cpp
// Log all transmitted messages
printf("TX: ID=0x%03X, DLC=%d, Data=", frame.id, frame.dlc);
for (int i = 0; i < frame.dlc; i++) {
    printf("%02X ", frame.data[i]);
}
printf("\n");
```

#### Error Monitoring
```cpp
// Monitor CAN errors
uint32_t errors = HAL_CAN_GetError(&hcan1);
if (errors != HAL_CAN_ERROR_NONE) {
    printf("CAN Error: 0x%08X\n", errors);
}
```

### Statistics Collection

The system collects communication statistics:

```cpp
// Get transmission statistics
uint32_t tx_ok = can.tx_ok();
uint32_t tx_err = can.tx_err();
uint32_t rx_dropped = can.rx_dropped();

printf("CAN Stats: TX_OK=%d, TX_ERR=%d, RX_DROPPED=%d\n", 
       tx_ok, tx_err, rx_dropped);
```

## Protocol Extensions

### Future Message Types

The protocol can be extended with additional message types:

```cpp
enum MessageType : uint8_t {
    HIGH_TEMP = 0,
    VOLTAGE_EXTREMES = 1,
    AVERAGES = 2,
    // Future extensions
    BALANCING_STATUS = 3,
    FAULT_STATUS = 4,
    CALIBRATION_DATA = 5,
    // ... up to 255
};
```

### Message Versioning

Future protocol versions can include version information:

```cpp
// Extended message format with version
struct ExtendedFrame {
    uint8_t version;        // Protocol version
    uint8_t message_type;   // Message type
    uint8_t data[6];        // Payload data
};
```

## Compliance and Standards

### CAN Standards Compliance

- **ISO 11898-1**: CAN protocol specification
- **ISO 11898-2**: High-speed CAN physical layer
- **ISO 11898-3**: Low-speed CAN physical layer
- **CAN 2.0B**: Extended frame format support

### Automotive Standards

- **ISO 14229**: UDS (Unified Diagnostic Services)
- **ISO 15765**: Diagnostic communication over CAN
- **SAE J1939**: Heavy-duty vehicle communication

## Best Practices

### 1. Message Design
- Use consistent data formats across all message types
- Include validation fields in message structure
- Design for extensibility and backward compatibility
- Use meaningful message type identifiers

### 2. Error Handling
- Implement comprehensive error detection
- Provide graceful degradation on communication failures
- Log all communication errors for debugging
- Implement automatic recovery procedures

### 3. Performance
- Minimize message payload size
- Use efficient encoding/decoding algorithms
- Implement proper buffering for high-throughput scenarios
- Monitor bus utilization and performance

### 4. Security
- Validate all received messages
- Implement message authentication if required
- Protect against message injection attacks
- Use secure CAN ID assignment procedures

This CAN communication protocol provides a robust, scalable foundation for the distributed BMS system communication requirements.

# Distributed BMS - Feature Tracking & Assignment

**Last Updated:** 2026-01-20 (Primary contactor logic, current calibration, and CAN interface implemented)  
**Project:** Distributed Battery Management System  
**Purpose:** Track features, tasks, assignments, and progress across all BMS boards

---

## Legend

### Status
- 🟢 **Completed** - Feature is fully implemented and tested
- 🟡 **In Progress** - Currently being worked on
- 🔴 **Blocked** - Waiting on dependencies or external factors
- ⚪ **TODO** - Not yet started
- 🟣 **Review** - Needs code review or testing

### Priority
- **P0** - Critical (blocks other work)
- **P1** - High (important for core functionality)
- **P2** - Medium (important but not blocking)
- **P3** - Low (nice to have)

---

## Daughter Board (DistributedBMSDaughter)

### Core Functionality

| Feature | Status | Priority | Owner | Notes | Dependencies |
|---------|--------|----------|-------|-------|--------------|
| BQ76920 Initialization | 🟢 | P0 | - | Basic init with retry logic implemented | - |
| Cell Voltage Reading | 🟢 | P0 | - | Reading via I2C, validation implemented | BQ76920 Init |
| Temperature Reading (NTC) | 🟢 | P0 | - | ADC DMA reading implemented | ADC Init |
| CAN Communication | 🟢 | P0 | - | CAN driver with callback system | CAN Init |
| FreeRTOS Task Management | 🟢 | P0 | - | Two tasks: data collection & CAN transmission | - |
| Mutex Protection | 🟢 | P1 | - | BMS data mutex for thread safety | FreeRTOS |

### Data Processing

| Feature | Status | Priority | Owner | Notes | Dependencies |
|---------|--------|----------|-------|-------|--------------|
| BMS Core Processing | 🟢 | P0 | - | Cell voltage/temp stats calculation | - |
| Data Validation | 🟢 | P1 | - | Cell voltage and ADC validation | - |
| Fault Management | 🟢 | P1 | - | Hardware fault tracking system | - |
| CAN Frame Generation | 🟢 | P0 | - | Average stats, voltage extremes, high temp frames | BMS Core |

### Communication

| Feature | Status | Priority | Owner | Notes | Dependencies |
|---------|--------|----------|-------|-------|--------------|
| CAN Average Stats Frame | 🟢 | P0 | - | Type 2: avg temp, avg voltage, cell count | CAN Driver |
| CAN Voltage Extremes Frame | 🟢 | P0 | - | Type 1: min/max voltages with indices | CAN Driver |
| CAN High Temp Frame | 🟢 | P0 | - | Type 0: highest temperature with index | CAN Driver |
| CAN Individual Cell Voltages | ⚪ | P1 | - | Type 3: All individual cell voltages (may need multiple frames) | CAN Driver, BMS Core |
| CAN Individual Cell Temperatures | ⚪ | P1 | - | Type 4: All individual cell temperatures (may need multiple frames) | CAN Driver, BMS Core |
| CAN Cell Voltage Frame (Single) | ⚪ | P2 | - | Type 5: Single cell voltage with index | CAN Driver |
| CAN Cell Temperature Frame (Single) | ⚪ | P2 | - | Type 6: Single cell temperature with index | CAN Driver |
| CAN Fault Reporting | 🟡 | P1 | - | Send fault mask over CAN | Fault Manager |
| CAN Heartbeat | ⚪ | P2 | - | Periodic heartbeat message | CAN Driver |

### Hardware Drivers

| Feature | Status | Priority | Owner | Notes | Dependencies |
|---------|--------|----------|-------|-------|--------------|
| BQ7692000PW Driver | 🟢 | P0 | - | Full driver with calibration support | I2C |
| CAN Driver (CANDevice) | 🟢 | P0 | - | CMSIS-RTOS2 driven CAN/FDCAN driver | CAN HAL |
| ADC DMA Driver | 🟢 | P0 | - | Multi-channel ADC with DMA | ADC HAL |

### Fault Management

| Feature | Status | Priority | Owner | Notes | Dependencies |
|---------|--------|----------|-------|-------|--------------|
| BQ76920 Comm Error Detection | 🟢 | P1 | - | I2C communication monitoring | BQ76920 Driver |
| CAN Transmit Error Detection | 🟢 | P1 | - | CAN transmission failure tracking | CAN Driver |
| ADC Result Error Detection | 🟢 | P1 | - | ADC reading validation | Data Validator |
| BQ76920 Result Error Detection | 🟢 | P1 | - | Voltage reading validation | Data Validator |
| Fault LED Indication | 🟢 | P2 | - | GPIO fault pin control | GPIO |
| OK LED Indication | 🟢 | P2 | - | GPIO OK pin toggle on no faults | GPIO |

### Configuration & Calibration

| Feature | Status | Priority | Owner | Notes | Dependencies |
|---------|--------|----------|-------|-------|--------------|
| Device Configuration | 🟢 | P1 | - | Cell count, cycle time, CAN ID config | - |
| BQ76920 Calibration Reading | 🟢 | P2 | - | ADC gain/offset reading | BQ76920 Driver |
| Temperature Calibration | ⚪ | P2 | - | NTC thermistor calibration | - |

### Testing & Debugging

| Feature | Status | Priority | Owner | Notes | Dependencies |
|---------|--------|----------|-------|-------|--------------|
| Debug Mode Flag | 🟢 | P3 | - | Basic debug mode variable | - |
| Module-Level CAN Message Testing | 🟢 | P1 | - | Test HIGH_TEMP, VOLTAGE_EXTREMES, AVERAGES message transmission/reception | CAN Driver |
| CAN Message Encoding/Decoding Tests | 🟢 | P1 | - | Test encode/decode functions with edge cases (min/max values, invalid data) | CAN Frame Generation |
| CAN Message Validation Tests | 🟢 | P1 | - | Test message validation, error handling, and malformed message rejection | CAN Driver |
| CAN Message Round-Trip Testing | 🟢 | P1 | - | Test full round-trip: encode → transmit → receive → decode → verify | CAN Driver, Secondary Board |
| CAN Bus Stress Testing | ⚪ | P2 | - | Test CAN communication under high load with multiple modules | CAN Driver |
| Cell-Level Message Testing | ⚪ | P1 | - | Test new cell-level message types once implemented | CAN Individual Cell Messages |
| Unit Tests | ⚪ | P3 | - | Unit tests for data handlers | - |
| Integration Tests | ⚪ | P2 | - | Hardware-in-the-loop testing | - |

---

## Secondary Board (DistributedBMSSecondary)

### Core Functionality

| Feature | Status | Priority | Owner | Notes | Dependencies |
|---------|--------|----------|-------|-------|--------------|
| CAN Reception from Daughters | 🟢 | P0 | - | Receives CAN frames from daughter boards | CAN Driver |
| Fleet Management | 🟢 | P0 | - | Tracks multiple daughter board modules | - |
| UART Communication to Primary | 🟢 | P0 | - | Sends aggregated data to primary MCU | UART Driver |
| FreeRTOS Task Management | 🟢 | P0 | - | Multiple tasks for CAN and UART | - |

### Data Aggregation

| Feature | Status | Priority | Owner | Notes | Dependencies |
|---------|--------|----------|-------|-------|--------------|
| Module Data Collection | 🟢 | P0 | - | Collects data from each daughter board | CAN Reception |
| Fleet Summary Calculation | 🟢 | P0 | - | Calculates fleet-wide statistics | Module Data |
| Online/Offline Detection | 🟢 | P1 | - | Tracks which modules are online | CAN Reception |
| Data Staleness Detection | 🟢 | P1 | - | Detects stale data from modules | Timestamp Tracking |

### Communication

| Feature | Status | Priority | Owner | Notes | Dependencies |
|---------|--------|----------|-------|-------|--------------|
| UART Fleet Summary Frame | 🟢 | P0 | - | Sends fleet summary to primary | UART Driver |
| UART Module Summary Frame | 🟢 | P0 | - | Sends individual module data | UART Driver |
| UART Heartbeat Frame | 🟢 | P0 | - | Periodic heartbeat to primary | UART Driver |
| UART Cell-Level Data Frame | ⚪ | P1 | - | Type 0x13: Individual cell voltages/temps for a module | UART Driver, CAN Cell-Level Messages |
| Frame Rotation System | 🟢 | P1 | - | Rotates between frame types to prevent overrun | UART Driver |
| UART Packet Framing | 🟢 | P0 | - | Proper packet encoding/decoding | UART Driver |

### Hardware Drivers

| Feature | Status | Priority | Owner | Notes | Dependencies |
|---------|--------|----------|-------|-------|--------------|
| CAN Driver | 🟢 | P0 | - | CAN reception from daughter boards | CAN HAL |
| UART Driver | 🟢 | P0 | - | UART transmission to primary MCU | UART HAL |

### Testing & Debugging

| Feature | Status | Priority | Owner | Notes | Dependencies |
|---------|--------|----------|-------|-------|--------------|
| Module-Level Data Function Testing | 🟢 | P1 | - | Comprehensive testing of module data collection, aggregation, and processing | Module Data Collection |
| Fleet Summary Calculation Testing | 🟢 | P1 | - | Test fleet summary calculations with various module configurations | Fleet Summary Calculation |
| Online/Offline Detection Testing | 🟢 | P1 | - | Test module online/offline detection logic | Online/Offline Detection |
| Data Staleness Detection Testing | 🟢 | P1 | - | Test data staleness detection and timeout handling | Data Staleness Detection |
| UART Frame Reception Testing | ⚪ | P1 | - | Test UART frame reception and parsing for all message types | UART Packet Framing |
| UART Frame Rotation Testing | ⚪ | P2 | - | Test frame rotation system prevents overrun errors | Frame Rotation System |
| CAN Message Logging | ⚪ | P3 | - | Log received CAN messages | CAN Driver |
| UART Transmission Monitoring | ⚪ | P3 | - | Monitor UART transmission success | UART Driver |

---

## Primary Board (DistributedBMSPrimary)

### Core BMS Functionality

| Feature | Status | Priority | Owner | Notes | Dependencies |
|---------|--------|----------|-------|-------|--------------|
| Fleet Data Reception | 🟢 | P0 | - | Receives fleet data from secondary MCU (UART queue, CRC validation) | UART Driver |
| PrimaryBmsFleet Management | 🟢 | P0 | - | Stores and manages fleet summary data; data staleness detection in place | UART Reception |
| Contactor Control Logic | 🟢 | P0 | - | Dual main contactor sequencing with staggered close and safety gating in `BmsManager` | Fleet Data, BmsManager |
| Fault Decision Making | 🟡 | P0 | - | Core voltage/temp/current/data-stale faults implemented in `BmsManager` | Fleet Data |
| Battery State Machine | 🟡 | P1 | - | INIT/IDLE/OPERATIONAL/FAULT/SHUTDOWN implemented; charge/discharge modes TBD | Contactor Control |

### Power Management

| Feature | Status | Priority | Owner | Notes | Dependencies |
|---------|--------|----------|-------|-------|--------------|
| Fan Speed Control | 🟡 | P1 | - | Temperature-based PWM control in `BmsManager` (uses fleet highest temp) | BTS71040 Driver |
| Power Management for Auxiliaries | ⚪ | P2 | - | Power management for solar car auxiliaries | - |
| Current Monitoring | 🟡 | P1 | - | Pack current via ADS1115 with calibrated nonlinear fit; aux current via INA226 | INA226 Driver, ADS1115 Driver |

### Communication

| Feature | Status | Priority | Owner | Notes | Dependencies |
|---------|--------|----------|-------|-------|--------------|
| CAN FD Communication | 🟡 | P0 | - | Basic FDCAN driver (`CanFdBus`) and BMS CAN interface (heartbeat, pack status, commands) | FDCAN Driver |
| USB Debug Communication | 🟢 | P2 | - | USB CDC for debugging and data logging | USB Device |
| UART Reception from Secondary | 🟢 | P0 | - | Receives UART packets from secondary MCU | UART Driver |
| UART Packet Parsing | 🟢 | P0 | - | Parses different message types | UART Reception |

### Hardware Drivers

| Feature | Status | Priority | Owner | Notes | Dependencies |
|---------|--------|----------|-------|-------|--------------|
| BTS71040 Driver | 🟡 | P1 | - | High-side switch driver for fan control (used by BmsManager fan control) | SPI |
| INA226 Driver | 🟡 | P1 | - | Current/power monitor (aux current) | I2C |
| ADS1115 Driver | 🟡 | P2 | - | ADC for pack current measurement and other sensors | I2C |
| LSM6DSO32 Driver | ⚪ | P3 | - | IMU for vibration/shock detection | SPI |
| FDCAN Driver | 🟢 | P0 | - | `CanFdBus` implemented with RX FIFO handling, error callbacks, and bus-off recovery | FDCAN HAL |
| USB CDC Driver | 🟢 | P2 | - | USB communication device class | USB HAL |

### Safety & Monitoring

| Feature | Status | Priority | Owner | Notes | Dependencies |
|---------|--------|----------|-------|-------|--------------|
| Overvoltage Protection | ⚪ | P0 | - | Detect and respond to overvoltage | Fleet Data |
| Undervoltage Protection | ⚪ | P0 | - | Detect and respond to undervoltage | Fleet Data |
| Overcurrent Protection | ⚪ | P0 | - | Detect and respond to overcurrent | Current Monitoring |
| Overtemperature Protection | ⚪ | P0 | - | Detect and respond to overtemperature | Fleet Data |
| Cell Imbalance Detection | ⚪ | P1 | - | Detect significant cell voltage differences | Fleet Data |
| Emergency Shutdown | ⚪ | P0 | - | Emergency shutdown procedure | All Protection Systems |

### Data Logging & Diagnostics

| Feature | Status | Priority | Owner | Notes | Dependencies |
|---------|--------|----------|-------|-------|--------------|
| USB Data Streaming | 🟡 | P2 | - | Stream battery data over USB | USB CDC |
| Fault Logging | ⚪ | P2 | - | Log BMS faults for analysis | Fault Decision Making |
| Performance Metrics | ⚪ | P3 | - | Track system performance metrics | - |
| Latency Monitoring | 🟢 | P2 | - | Monitor communication latency | Fleet Data |

### Testing & Debugging

| Feature | Status | Priority | Owner | Notes | Dependencies |
|---------|--------|----------|-------|-------|--------------|
| Unit Tests | ⚪ | P3 | - | Unit tests for BMS logic | - |
| Hardware-in-the-Loop Tests | ⚪ | P2 | - | HIL testing for safety features | - |
| Integration Tests | ⚪ | P2 | - | Full system integration tests | - |

---

## Cross-Board Features

### Communication Protocols

| Feature | Status | Priority | Owner | Notes | Dependencies |
|---------|--------|----------|-------|-------|--------------|
| CAN Protocol Standardization | 🟢 | P0 | - | Standardized CAN message format | All Boards |
| CAN Cell-Level Protocol Design | ⚪ | P1 | - | Design protocol for cell-level data messages | CAN Protocol Standardization |
| UART Protocol Standardization | 🟢 | P0 | - | Standardized UART packet format | Secondary, Primary |
| UART Cell-Level Protocol Extension | ⚪ | P1 | - | Extend UART protocol for cell-level data | UART Protocol Standardization |
| Protocol Documentation | 🟢 | P1 | - | CAN and UART protocol docs | - |
| Cell-Level Protocol Documentation | ⚪ | P1 | - | Document new cell-level message types | CAN Cell-Level Protocol Design |

### System Integration

| Feature | Status | Priority | Owner | Notes | Dependencies |
|---------|--------|----------|-------|-------|--------------|
| End-to-End Data Flow | 🟢 | P0 | - | Data flows from daughter → secondary → primary | All Boards |
| System Health Monitoring | 🟡 | P1 | - | Monitor health across all boards | All Boards |
| Configuration Management | 🟡 | P2 | - | Centralized configuration system | All Boards |

### Documentation

| Feature | Status | Priority | Owner | Notes | Dependencies |
|---------|--------|----------|-------|-------|--------------|
| API Reference | 🟢 | P1 | - | API documentation for drivers | - |
| Build Guide | 🟢 | P1 | - | Build and deployment instructions | - |
| Fault Management Guide | 🟢 | P1 | - | Fault system usage guide | - |
| Data Validation Guide | 🟢 | P1 | - | Data validation documentation | - |
| CAN Protocol Spec | 🟢 | P1 | - | CAN protocol specification | - |
| UART Protocol Spec | 🟢 | P1 | - | UART protocol specification | - |
| Debugging Guide | 🟢 | P2 | - | Debugging procedures and tools | - |
| Architecture Documentation | ⚪ | P2 | - | System architecture overview | - |

---

## Known Issues & Blockers

| Issue | Board | Priority | Owner | Description | Status |
|-------|-------|----------|-------|-------------|--------|
| Cell-Level Data Bandwidth | All | P1 | - | Need to evaluate CAN bus bandwidth for cell-level messages (5 cells × 2 messages = 10 messages per module) | Under Review |

---

## Future Enhancements

| Feature | Board | Priority | Owner | Description | Notes |
|---------|-------|----------|-------|-------------|-------|
| Active Cell Balancing | Daughter | P2 | - | Implement active cell balancing via BQ76920 | Requires hardware support |
| Predictive Maintenance | Primary | P3 | - | Predict failures before they occur | ML/AI integration |
| Remote Diagnostics | Primary | P3 | - | Remote diagnostic capabilities | Requires connectivity |
| Energy Management | Primary | P2 | - | Advanced energy management algorithms | - |
| Multi-Pack Support | All | P2 | - | Support for multiple battery packs | Architecture change |

---

## Notes

### How to Use This Document

1. **Update Status**: Change status emoji when work progresses
2. **Assign Owners**: Add team member names to Owner column
3. **Add Notes**: Document blockers, decisions, or important information
4. **Track Dependencies**: Ensure dependencies are completed before starting dependent work
5. **Regular Updates**: Update this document during team meetings

### Status Update Guidelines

- Move items from ⚪ → 🟡 when work begins
- Move items from 🟡 → 🟣 when ready for review
- Move items from 🟣 → 🟢 when complete and tested
- Use 🔴 for blocked items and document the blocker in Notes

### Assignment Guidelines

- Assign based on expertise and availability
- Update assignments as team members take on tasks
- Ensure critical (P0) items have clear ownership

---

**Document Maintainer:** [Your Name/Team Lead]  
**Review Frequency:** Weekly during active development


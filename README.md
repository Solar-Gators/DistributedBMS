# Distributed BMS Project

A distributed Battery Management System (BMS) implementation using STM32 microcontrollers with CAN bus communication.

## Project Overview

This project implements a distributed BMS architecture consisting of:

- **Daughter Boards** (`DistributedBMSDaughter`): Monitor small groups of cells (3-5 cells each)
- **Secondary Board** (`DistributedBMSSecondary`): Aggregates data from daughter boards and communicates with main controller

## Architecture

```
┌─────────────────┐    CAN Bus    ┌─────────────────┐
│   Daughter 1    │◄─────────────►│                 │
│   (Cells 1-4)   │               │                 │
└─────────────────┘               │   Secondary     │
                                  │   Board         │
┌─────────────────┐               │                 │
│   Daughter 2    │◄─────────────►│                 │
│   (Cells 5-8)   │               │                 │
└─────────────────┘               │                 │
                                  │                 │
┌─────────────────┐               │                 │
│   Daughter N    │◄─────────────►│                 │
│   (Cells X-Y)   │               │                 │
└─────────────────┘               └─────────────────┘
                                           │
                                           │ UART
                                           ▼
                                  ┌─────────────────┐
                                  │   Main BMS      │
                                  │   Controller    │
                                  └─────────────────┘
```

## Key Features

### Daughter Board Features
- **Cell Monitoring**: Individual cell voltage measurement using BQ76920 IC
- **Temperature Monitoring**: NTC thermistor temperature sensing
- **CAN Communication**: Real-time data transmission to central BMS
- **Fault Management**: Comprehensive system health monitoring
- **Data Validation**: Robust data integrity checking
- **Configurable**: Support for 3-5 cell configurations

### Secondary Board Features
- **Data Aggregation**: Collects data from multiple daughter boards
- **Fleet Management**: Tracks multiple BMS modules
- **UART Communication**: Interfaces with main controller
- **CAN Bus Management**: Handles multiple CAN nodes

## Hardware Requirements

### Daughter Board
- **MCU**: STM32L433CCTx
- **Battery Monitor**: BQ76920PW (Texas Instruments)
- **Communication**: CAN transceiver
- **Temperature**: NTC thermistors
- **Power**: 3.3V operation

### Secondary Board
- **MCU**: STM32L433CCTx
- **Communication**: CAN transceiver, UART
- **Power**: 3.3V operation

## Software Architecture

### Core Components

#### 1. Fault Management System (`FaultManager`)
- Monitors system health and communication status
- Provides fault logging and recovery mechanisms
- Focuses on hardware functionality, not battery management decisions

#### 2. Data Validation System (`DataValidator`)
- Validates sensor readings for accuracy and consistency
- Detects outliers and sensor failures
- Provides data quality scoring

#### 3. BMS Core (`BMS`)
- Processes cell voltage and temperature data
- Calculates statistics (min, max, average)
- Handles NTC temperature conversion

#### 4. CAN Communication (`CanBus`, `CanFrames`)
- Robust CAN bus implementation with interrupt handling
- Standardized frame encoding/decoding
- Error detection and recovery

#### 5. BQ76920 Driver (`BQ7692000PW`)
- Low-level communication with BQ76920 IC
- Cell voltage and temperature reading
- Configuration and status monitoring

## Communication Protocol

### CAN Message Types

| Type | ID | Description | Data Format |
|------|----|-----------|-------------|
| HIGH_TEMP | 0x07 | Highest temperature and index | `[type][temp_float][index]` |
| VOLTAGE_EXTREMES | 0x07 | Min/max voltages and indices | `[type][high_v][low_v][indices]` |
| AVERAGES | 0x07 | Average temperature and voltage | `[type][avg_temp][avg_voltage][cell_count]` |

### Message Format
- **Type**: 1 byte (0=HIGH_TEMP, 1=VOLTAGE_EXTREMES, 2=AVERAGES)
- **Data**: 7 bytes of payload
- **DLC**: 8 bytes (fixed)

## Getting Started

### Prerequisites
- STM32CubeIDE or compatible development environment
- STM32L4xx HAL library
- CAN bus analyzer (for debugging)

### Building
1. Clone the repository
2. Open project in STM32CubeIDE
3. Configure hardware settings in `.ioc` files
4. Build and flash to target hardware

### Configuration
1. Set cell count in `main.cpp` (3-5 cells supported)
2. Configure CAN IDs for each daughter board
3. Adjust validation thresholds in `DataValidator`
4. Set fault management parameters

## Documentation

- [System Architecture](docs/architecture.md)
- [API Reference](docs/api_reference.md)
- [Fault Management Guide](docs/fault_management.md)
- [Data Validation Guide](docs/data_validation.md)
- [CAN Protocol Specification](docs/can_protocol.md)
- [Build and Deployment Guide](docs/build_guide.md)

## Safety Considerations

⚠️ **Important Safety Notes**:
- This is a monitoring system only - it does not control charging/discharging
- All battery management decisions are made by the central controller
- Daughter boards focus on data collection and system health
- Always validate data before making safety-critical decisions

## Contributing

1. Follow the existing code style
2. Add comprehensive documentation for new features
3. Include unit tests where applicable
4. Update this README for significant changes

## License

[Add your license information here]

## Support

For technical support or questions, please refer to the documentation or create an issue in the repository.

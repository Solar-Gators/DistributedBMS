# Distributed BMS Project

A distributed Battery Management System (BMS) design based arround a series of daughter boards which collect tempature and voltage data and communicate these back to a central BMS board. The central BMS board monitors current and controlls battery fans and contactors tacots as well as a few other tasks. All boards use STM32 Microcontollers and communciate over CAN Bus

## Project Overview

This project implements a distributed BMS architecture consisting of:

- **Daughter Boards** (`DistributedBMSDaughter`): Monitor small groups of cells (3-5 cells each)
- **Secondary Board** (`DistributedBMSSecondary`): Aggregates data from daughter boards and communicates with main controller

## Architecture

Each Daughter Board PCB monitors the tempature and voltage of between 3-5 battery modules, and communicates this over CAN to the secondary STM on the central PCB. The secondary STM then aggregates all this data, and passes it to the primary STM on the central board over UART. The primary STM is responsible for all the control tasks and communicating with the rest of the car.

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

#### 4. CAN Driver (`CanDriver`)

CANDriver::CANDevice is a CMSIS-RTOS2–driven CAN/FDCAN driver that:

- Configures acceptance filters (exact ID or aligned ranges) for bxCAN or FDCAN.
- Spawns RX/TX worker threads and RX/TX message queues.
- Uses an ISR to drain FIFO0 into an RX queue; a worker dispatches to registered callbacks (by ID, by range, or “catch-all”).
- Provides a Send() API that enqueues frames to a TX worker which pushes them out using HAL (FD mode/BRS enabled on FDCAN).
- It abstracts HAL differences (bxCAN vs FDCAN), converts DLC↔byte length, and exposes simple callback registration.

#### 5. BQ76920 Driver (`BQ7692000PW`)
- Low-level communication with BQ76920 IC
- Cell voltage and temperature reading
- Configuration and status monitoring

## Communication Protocol

### CAN Message Types

Daugter Board to Secondary Can Messages

| Type             | ID                     | Description                     | Data Format                                 |
|------------------|------------------------|---------------------------------|---------------------------------------------|
| HIGH_TEMP        | 0x100 + section number | Highest temperature and index   | `[type][temp_float][index]`                 |
| VOLTAGE_EXTREMES | 0x100 + section number | Min/max voltages and indices    | `[type][high_v][low_v][indices]`            |
| AVERAGES         | 0x100 + section number | Average temperature and voltage | `[type][avg_temp][avg_voltage][cell_count]` |

### Message Format
- **Type**: 1 byte (0=HIGH_TEMP, 1=VOLTAGE_EXTREMES, 2=AVERAGES)
- **Data**: 7 bytes of payload
- **DLC**: 8 bytes (fixed)

## Getting Started

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

# System Architecture Documentation

## Overview

The Distributed BMS system follows a hierarchical architecture designed for scalability, reliability, and maintainability. The system is built around the principle of separation of concerns, where each component has a specific responsibility.

## System Components

### 1. Daughter Boards (DistributedBMSDaughter)

**Purpose**: Monitor individual cell groups and report data to central system

**Responsibilities**:
- Cell voltage monitoring (3-5 cells per board)
- Temperature sensing (NTC thermistors)
- Data validation and quality assessment
- CAN communication with central system
- System health monitoring

**Key Classes**:
- `BMS`: Core battery monitoring logic
- `BQ7692000PW`: Driver for BQ76920 battery monitor IC
- `FaultManager`: System health and fault tracking
- `DataValidator`: Data integrity and validation
- `CanBus`: CAN communication interface
- `CanFrames`: Message encoding/decoding

### 2. Secondary Board (DistributedBMSSecondary)

**Purpose**: Aggregate data from daughter boards and interface with main controller

**Responsibilities**:
- Collect data from multiple daughter boards
- Manage CAN bus communication
- Aggregate and summarize fleet data
- UART communication with main controller
- Fleet health monitoring

**Key Classes**:
- `BmsFleet`: Fleet management and data aggregation
- `CanBus`: CAN communication interface
- `CanFrames`: Message decoding and processing

## Data Flow Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Daughter Board                            │
│                                                             │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐     │
│  │   Sensors    │    │   BQ76920   │    │    ADC      │     │
│  │  (NTC Temp) │    │   (Voltage) │    │  (External) │     │
│  └──────┬──────┘    └──────┬──────┘    └──────┬──────┘     │
│         │                  │                  │            │
│         └──────────────────┼──────────────────┘            │
│                            │                               │
│  ┌─────────────────────────▼─────────────────────────┐     │
│  │              DataValidator                         │     │
│  │  • Range checking                                  │     │
│  │  • Outlier detection                               │     │
│  │  • Quality assessment                              │     │
│  └─────────────────────────┬─────────────────────────┘     │
│                            │                               │
│  ┌─────────────────────────▼─────────────────────────┐     │
│  │                BMS Core                            │     │
│  │  • Statistics calculation                          │     │
│  │  • Temperature conversion                          │     │
│  │  • Data processing                                 │     │
│  └─────────────────────────┬─────────────────────────┘     │
│                            │                               │
│  ┌─────────────────────────▼─────────────────────────┐     │
│  │              FaultManager                         │     │
│  │  • System health monitoring                       │     │
│  │  • Fault logging                                  │     │
│  │  • Recovery management                            │     │
│  └─────────────────────────┬─────────────────────────┘     │
│                            │                               │
│  ┌─────────────────────────▼─────────────────────────┐     │
│  │              CAN Communication                     │     │
│  │  • Message encoding                               │     │
│  │  • Error handling                                 │     │
│  │  • Transmission                                   │     │
│  └─────────────────────────┘                         │
└─────────────────────────────────────────────────────────────┘
                            │ CAN Bus
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                  Secondary Board                            │
│                                                             │
│  ┌─────────────────────────┐    ┌─────────────────────────┐│
│  │      CAN Receiver       │    │     Fleet Manager        ││
│  │  • Message decoding     │    │  • Data aggregation      ││
│  │  • Error detection      │    │  • Module tracking       ││
│  │  • Frame validation     │    │  • Health monitoring     ││
│  └─────────────┬───────────┘    └─────────────┬───────────┘│
│                │                               │           │
│                └───────────────┬───────────────┘           │
│                                │                           │
│  ┌─────────────────────────────▼─────────────────────────┐ │
│  │              UART Interface                           │ │
│  │  • Data formatting                                     │ │
│  │  • Protocol handling                                   │ │
│  │  • Error reporting                                     │ │
│  └─────────────────────────────┘                         │
└─────────────────────────────────────────────────────────────┘
                            │ UART
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                Main Controller                              │
│  • Battery management decisions                             │
│  • Safety system control                                    │
│  • User interface                                           │
│  • Data logging                                             │
└─────────────────────────────────────────────────────────────┘
```

## Communication Architecture

### CAN Bus Protocol

**Physical Layer**:
- CAN 2.0B standard
- 500 kbps bit rate
- Differential signaling
- Termination resistors required

**Message Structure**:
```
┌─────────────┬─────────────┬─────────────┬─────────────┐
│   CAN ID    │    DLC      │    Type     │    Data     │
│  (11-bit)   │   (4-bit)   │   (1-byte)  │  (7-bytes)  │
└─────────────┴─────────────┴─────────────┴─────────────┘
```

**Message Types**:
1. **HIGH_TEMP** (Type 0): Highest temperature and sensor index
2. **VOLTAGE_EXTREMES** (Type 1): Min/max cell voltages and indices
3. **AVERAGES** (Type 2): Average temperature and voltage statistics

### UART Protocol

**Physical Layer**:
- 115200 baud rate
- 8 data bits, 1 stop bit, no parity
- RS-232 compatible

**Message Format**:
- JSON-based structured data
- Timestamped measurements
- Error codes and status information

## Error Handling Architecture

### Fault Management Hierarchy

```
┌─────────────────────────────────────────────────────────────┐
│                    Fault Levels                             │
│                                                             │
│  CRITICAL ──────────────────────────────────────────────── │
│  • System non-functional                                   │
│  • Immediate attention required                             │
│  • Safety implications                                     │
│                                                             │
│  ERROR ─────────────────────────────────────────────────── │
│  • System degraded but functional                          │
│  • Reduced performance                                     │
│  • Monitoring required                                     │
│                                                             │
│  WARNING ───────────────────────────────────────────────── │
│  • Monitor but continue operation                          │
│  • Preventive maintenance                                  │
│  • Trend analysis                                          │
│                                                             │
│  INFO ──────────────────────────────────────────────────── │
│  • Non-critical events                                     │
│  • Logging purposes                                        │
│  • Diagnostic information                                  │
└─────────────────────────────────────────────────────────────┘
```

### Fault Recovery Strategy

1. **Automatic Recovery**: Clear faults when conditions improve
2. **Retry Logic**: Implement exponential backoff for communication failures
3. **Graceful Degradation**: Continue operation with reduced functionality
4. **Fault Logging**: Maintain history for diagnostic purposes

## Data Validation Architecture

### Validation Pipeline

```
Raw Sensor Data
       │
       ▼
┌─────────────┐
│ Range Check │ ──► Pass ──► Outlier Detection ──► Pass ──► Rate Check
│             │ ──► Fail ──► Set Fault ──────────► Fail ──► Set Fault
└─────────────┘
       │
       ▼
┌─────────────┐
│ Consistency │ ──► Pass ──► Quality Assessment ──► Valid Data
│ Check       │ ──► Fail ──► Set Fault ──────────► Invalid Data
└─────────────┘
```

### Validation Criteria

**Voltage Validation**:
- Range: 2.0V - 4.5V (configurable)
- Outlier detection: >10% deviation from recent values
- Consistency: All cells within reasonable range

**Temperature Validation**:
- Range: -40°C to +85°C (configurable)
- Rate limit: <5°C/second change
- Sensor correlation: Multiple sensors should agree

**ADC Validation**:
- Range: 100-3995 (avoid dead zones)
- Noise threshold: <50 counts jitter
- Stuck value detection: No change over time

## Performance Characteristics

### Timing Requirements

**Daughter Board**:
- Main loop: 250ms cycle time
- CAN transmission: <10ms per message
- Data validation: <5ms per cycle
- Fault processing: <1ms per cycle

**Secondary Board**:
- CAN reception: Interrupt-driven (<1ms)
- Data processing: <5ms per module
- UART transmission: <20ms per update

### Memory Usage

**Daughter Board**:
- Flash: ~64KB (estimated)
- RAM: ~8KB (estimated)
- Stack: ~2KB (estimated)

**Secondary Board**:
- Flash: ~32KB (estimated)
- RAM: ~4KB (estimated)
- Stack: ~1KB (estimated)

## Scalability Considerations

### Horizontal Scaling
- Support up to 8 daughter boards per secondary board
- CAN bus can handle 100+ nodes
- Modular design allows easy expansion

### Vertical Scaling
- Configurable cell count per daughter board
- Adjustable validation parameters
- Flexible fault management thresholds

## Security Considerations

### Data Integrity
- CRC validation for critical data
- Sequence numbering for message tracking
- Timestamp validation

### Communication Security
- CAN bus isolation
- Error detection and correction
- Fault injection resistance

## Maintenance and Diagnostics

### Built-in Diagnostics
- System health monitoring
- Fault logging and history
- Performance metrics
- Self-test capabilities

### External Tools
- CAN bus analyzer integration
- UART debugging interface
- Configuration management
- Calibration procedures

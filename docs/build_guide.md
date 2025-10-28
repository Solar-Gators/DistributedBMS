# Build and Deployment Guide

## Overview

This guide provides step-by-step instructions for building, configuring, and deploying the Distributed BMS system. It covers both daughter boards and secondary boards, including hardware setup, software configuration, and testing procedures.

## Prerequisites

### Software Requirements

- **STM32CubeIDE**: Version 1.12.0 or later
- **STM32CubeMX**: Version 6.8.0 or later (included with CubeIDE)
- **STM32L4xx HAL Library**: Version 1.17.2 or later
- **ARM GCC Toolchain**: Included with CubeIDE
- **Git**: For version control

### Hardware Requirements

#### Daughter Board
- STM32L433CCTx microcontroller
- BQ76920PW battery monitor IC
- CAN transceiver (MCP2551 or equivalent)
- NTC thermistors (10kΩ @ 25°C)
- ADC reference components
- Power supply (3.3V)
- Debug interface (SWD)

#### Secondary Board
- STM32L433CCTx microcontroller
- CAN transceiver (MCP2551 or equivalent)
- UART interface (RS-232 or TTL)
- Power supply (3.3V)
- Debug interface (SWD)

#### Development Tools
- ST-Link V2 or V3 programmer/debugger
- CAN bus analyzer (optional but recommended)
- Multimeter
- Oscilloscope (optional)
- Logic analyzer (optional)

## Project Structure

```
DistributedBMSProject/
├── DistributedBMSDaughter/          # Daughter board project
│   ├── Core/
│   │   ├── Inc/                     # Header files
│   │   │   ├── BMS.hpp
│   │   │   ├── BQ7692000.hpp
│   │   │   ├── CanBus.hpp
│   │   │   ├── DataValidator.hpp
│   │   │   ├── FaultManager.hpp
│   │   │   └── main.h
│   │   └── Src/                     # Source files
│   │       ├── BMS.cpp
│   │       ├── BQ7692000.cpp
│   │       ├── CanBus.cpp
│   │       ├── CanFrame.cpp
│   │       ├── DataValidator.cpp
│   │       ├── FaultManager.cpp
│   │       └── main.cpp
│   ├── Drivers/                     # STM32 HAL drivers
│   ├── Debug/                       # Debug configuration
│   ├── DistributedBMSDaughter.ioc   # CubeMX configuration
│   └── STM32L433CCTX_FLASH.ld       # Linker script
├── DistributedBMSSecondary/         # Secondary board project
│   ├── Core/
│   │   ├── Inc/
│   │   │   ├── BmsFleet.hpp
│   │   │   ├── CanBus.hpp
│   │   │   ├── CanFrames.hpp
│   │   │   └── main.h
│   │   └── Src/
│   │       ├── BmsFleet.cpp
│   │       ├── CanBus.cpp
│   │       ├── CanFrames.cpp
│   │       └── main.cpp
│   ├── Drivers/
│   ├── Debug/
│   ├── DistributedBMSSecondary.ioc
│   └── STM32L433CCTX_FLASH.ld
└── docs/                            # Documentation
```

## Building the Projects

### 1. Clone the Repository

```bash
git clone <repository-url>
cd DistributedBMSProject
```

### 2. Open Projects in STM32CubeIDE

#### Daughter Board Project
1. Launch STM32CubeIDE
2. File → Import → Existing Projects into Workspace
3. Select `DistributedBMSDaughter` folder
4. Click Finish

#### Secondary Board Project
1. Repeat the import process for `DistributedBMSSecondary`

### 3. Configure Hardware Settings

#### Daughter Board Configuration

1. Open `DistributedBMSDaughter.ioc` in CubeMX
2. Configure the following peripherals:

**System Configuration**:
- Clock: MSI 4MHz → PLL → 80MHz
- Flash latency: 4 wait states
- Power: Voltage scaling 1

**GPIO Configuration**:
- PB0: External interrupt (rising edge)
- PB1, PB2: GPIO output (LEDs)
- PB3: GPIO output (NWC_Pin)
- PB4: GPIO output (OK_Pin)
- PB5: GPIO output (ERROR_Pin)
- PB6: GPIO output (Fault_Pin)

**ADC Configuration**:
- ADC1: 12-bit resolution
- Channels: ADC_IN5, ADC_IN6, ADC_IN7, ADC_IN8, ADC_IN9
- Sampling time: 24.5 cycles
- DMA: DMA1 Channel 1

**CAN Configuration**:
- CAN1: Normal mode
- Bit rate: 500 kbps
- Prescaler: 16
- Time segments: BS1=3, BS2=6, SJW=1

**I2C Configuration**:
- I2C2: Standard mode
- Timing: 0x10D19CE4
- Address: 7-bit addressing

**SPI Configuration**:
- SPI1: Master mode
- Baud rate: Prescaler 128
- Data size: 8-bit

3. Generate code and save

#### Secondary Board Configuration

1. Open `DistributedBMSSecondary.ioc` in CubeMX
2. Configure the following peripherals:

**System Configuration**:
- Clock: MSI 4MHz → 80MHz (no PLL)
- Flash latency: 0 wait states

**GPIO Configuration**:
- PB4: GPIO output (OK_Pin)

**CAN Configuration**:
- CAN1: Normal mode
- Bit rate: 500 kbps
- Prescaler: 2
- Time segments: BS1=2, BS2=1, SJW=1

**UART Configuration**:
- USART2: 115200 baud, 8N1
- TX: PA2, RX: PA3

3. Generate code and save

### 4. Build the Projects

#### Daughter Board
1. Right-click on `DistributedBMSDaughter` project
2. Select "Build Project"
3. Verify build completes without errors

#### Secondary Board
1. Right-click on `DistributedBMSSecondary` project
2. Select "Build Project"
3. Verify build completes without errors

## Configuration

### Daughter Board Configuration

#### 1. Cell Count Configuration

Edit `main.cpp` to set the number of cells:

```cpp
// Around line 88
BMS bms(4); // Change this to match your cell count (3-5)
```

#### 2. CAN ID Configuration

Each daughter board needs a unique CAN ID:

```cpp
// Around line 239-241
can.sendStd(0x101, f0.bytes, f0.dlc); // Change 0x101 to unique ID
can.sendStd(0x101, f1.bytes, f1.dlc);
can.sendStd(0x101, f2.bytes, f2.dlc);
```

**CAN ID Assignment**:
- Daughter Board 1: 0x101
- Daughter Board 2: 0x102
- Daughter Board 3: 0x103
- ... and so on

#### 3. Validation Configuration

Adjust validation parameters in `DataValidator`:

```cpp
// In main.cpp, after creating validator
DataValidator::ValidationConfig config;
config.min_cell_voltage_mV = 2000;  // Adjust as needed
config.max_cell_voltage_mV = 4500;  // Adjust as needed
config.min_temperature_C = -40.0f;  // Adjust as needed
config.max_temperature_C = 85.0f;   // Adjust as needed
dataValidator.setConfig(config);
```

### Secondary Board Configuration

#### 1. Daughter Board Registration

Register each daughter board in the fleet:

```cpp
// In main.cpp, around line 114
fleet.register_node(0x101, 0);  // Daughter board 1
fleet.register_node(0x102, 1);  // Daughter board 2
fleet.register_node(0x103, 2);  // Daughter board 3
// ... add more as needed
```

#### 2. UART Configuration

Configure UART parameters if needed:

```cpp
// UART is configured in CubeMX, but you can adjust baud rate
// Default: 115200 baud, 8N1
```

## Hardware Setup

### Daughter Board Assembly

#### 1. Component Placement
- Mount STM32L433CCTx microcontroller
- Install BQ76920PW battery monitor IC
- Add CAN transceiver (MCP2551)
- Install NTC thermistors
- Add power supply components

#### 2. Connections

**BQ76920PW Connections**:
- VCC → 3.3V
- GND → Ground
- SDA → PB11 (I2C2_SDA)
- SCL → PB10 (I2C2_SCL)
- VC1-VC5 → Cell voltage inputs

**CAN Transceiver Connections**:
- VCC → 3.3V
- GND → Ground
- TXD → PA12 (CAN1_TX)
- RXD → PA11 (CAN1_RX)

**NTC Thermistors**:
- Connect to ADC channels (PA5-PA9)
- Use voltage divider with 10kΩ resistor

**LEDs**:
- OK LED → PB4
- Fault LED → PB6
- Error LED → PB5

### Secondary Board Assembly

#### 1. Component Placement
- Mount STM32L433CCTx microcontroller
- Install CAN transceiver
- Add UART interface components
- Add power supply components

#### 2. Connections

**CAN Transceiver**:
- Same connections as daughter board

**UART Interface**:
- TX → PA2 (USART2_TX)
- RX → PA3 (USART2_RX)

**LEDs**:
- OK LED → PB4

### CAN Bus Wiring

#### 1. Bus Topology
```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│ Daughter 1  │    │ Daughter 2  │    │ Daughter N  │
│ CAN_H ──────┼────┼─────────────┼────┼─────────────┼──── CAN_H
│ CAN_L ──────┼────┼─────────────┼────┼─────────────┼──── CAN_L
└─────────────┘    └─────────────┘    └─────────────┘
       │                   │                   │
       └───────────────────┼───────────────────┘
                           │
                    ┌─────────────┐
                    │ Secondary   │
                    │ Board       │
                    └─────────────┘
```

#### 2. Termination
- Install 120Ω termination resistors at both ends of the bus
- Use twisted pair cable for CAN_H and CAN_L
- Keep bus length under 100m

## Deployment

### 1. Firmware Flashing

#### Daughter Board
1. Connect ST-Link to daughter board
2. In CubeIDE, right-click project → Debug As → STM32 C/C++ Application
3. Verify firmware loads successfully
4. Disconnect debugger

#### Secondary Board
1. Repeat the same process for secondary board

### 2. System Integration

#### 1. Power Up Sequence
1. Apply power to secondary board first
2. Apply power to daughter boards
3. Verify all boards initialize properly

#### 2. CAN Bus Verification
1. Use CAN analyzer to monitor bus traffic
2. Verify daughter boards transmit messages
3. Verify secondary board receives messages
4. Check message timing and content

#### 3. Data Validation
1. Monitor transmitted data for validity
2. Verify voltage and temperature readings
3. Check fault management operation
4. Test error conditions

### 3. Testing Procedures

#### Daughter Board Testing

**Voltage Reading Test**:
```cpp
// Monitor cell voltages
printf("Cell Voltages: %d, %d, %d, %d, %d mV\n",
       cellVoltages[0], cellVoltages[1], cellVoltages[2], 
       cellVoltages[3], cellVoltages[4]);
```

**Temperature Reading Test**:
```cpp
// Monitor temperatures
printf("Temperatures: %.1f, %.1f, %.1f, %.1f, %.1f °C\n",
       temperatures[0], temperatures[1], temperatures[2],
       temperatures[3], temperatures[4]);
```

**CAN Transmission Test**:
```cpp
// Monitor CAN transmission
if (can.sendStd(0x101, data, 8) == CanBus::Result::Ok) {
    printf("CAN TX: OK\n");
} else {
    printf("CAN TX: FAILED\n");
}
```

#### Secondary Board Testing

**Message Reception Test**:
```cpp
// Monitor received messages
CanBus::Frame frame;
if (can.read(frame)) {
    printf("RX: ID=0x%03X, Type=%d\n", frame.id, frame.data[0]);
}
```

**Fleet Status Test**:
```cpp
// Monitor fleet status
for (int i = 0; i < 8; i++) {
    const auto& module = fleet.module(i);
    if (module.online(HAL_GetTick())) {
        printf("Module %d: Online, Temp=%.1f°C, Voltage=%dmV\n",
               i, module.high_C, module.avg_cell_mV);
    }
}
```

## Troubleshooting

### Common Build Issues

#### 1. Compilation Errors
**Problem**: Missing includes or undefined references
**Solution**: 
- Verify all header files are included
- Check that all source files are added to project
- Ensure HAL library is properly linked

#### 2. Linker Errors
**Problem**: Undefined symbols or memory issues
**Solution**:
- Check linker script configuration
- Verify memory settings in CubeMX
- Ensure proper HAL library version

### Hardware Issues

#### 1. CAN Communication Problems
**Symptoms**: No messages on bus, transmission failures
**Solutions**:
- Check CAN transceiver connections
- Verify termination resistors (120Ω)
- Check bit rate configuration
- Monitor bus with analyzer

#### 2. I2C Communication Issues
**Symptoms**: BQ76920 communication failures
**Solutions**:
- Check I2C wiring (SDA/SCL)
- Verify pull-up resistors (4.7kΩ)
- Check BQ76920 power supply
- Verify I2C timing configuration

#### 3. ADC Reading Problems
**Symptoms**: Invalid temperature readings
**Solutions**:
- Check NTC thermistor connections
- Verify voltage divider circuit
- Check ADC reference voltage
- Calibrate ADC readings

### Software Issues

#### 1. Fault Management Problems
**Symptoms**: False faults or missed faults
**Solutions**:
- Adjust fault thresholds
- Check fault debouncing settings
- Verify fault clearing logic
- Monitor fault logs

#### 2. Data Validation Issues
**Symptoms**: Valid data rejected or invalid data accepted
**Solutions**:
- Adjust validation parameters
- Check validation algorithm logic
- Verify sensor calibration
- Monitor validation statistics

## Maintenance

### Regular Maintenance Tasks

#### 1. Firmware Updates
- Monitor for firmware updates
- Test updates in development environment
- Deploy updates systematically
- Verify system operation after updates

#### 2. Calibration
- Periodically calibrate sensors
- Verify voltage and temperature readings
- Adjust validation parameters as needed
- Document calibration procedures

#### 3. Performance Monitoring
- Monitor CAN bus utilization
- Check communication error rates
- Analyze fault patterns
- Optimize system performance

### Diagnostic Procedures

#### 1. System Health Check
```cpp
// Check system health
if (faultManager.isSystemFunctional()) {
    printf("System: HEALTHY\n");
} else {
    printf("System: FAULTY\n");
    uint32_t fault_mask = faultManager.getFaultMask();
    printf("Faults: 0x%08X\n", fault_mask);
}
```

#### 2. Communication Statistics
```cpp
// Monitor communication statistics
printf("CAN Stats: TX_OK=%d, TX_ERR=%d, RX_DROPPED=%d\n",
       can.tx_ok(), can.tx_err(), can.rx_dropped());
```

#### 3. Data Quality Assessment
```cpp
// Check data quality
const auto& results = bms.results();
printf("Data Quality: %d%%\n", results.data_quality_score);
printf("Validation: %s\n", results.validation_passed ? "PASS" : "FAIL");
```

## Safety Considerations

### Electrical Safety
- Always disconnect power before making connections
- Use proper grounding techniques
- Verify voltage levels before power-up
- Follow ESD protection procedures

### Battery Safety
- Never short battery terminals
- Use appropriate fuses and protection
- Monitor battery voltages continuously
- Follow battery manufacturer guidelines

### System Safety
- Implement proper fault handling
- Use redundant safety systems
- Monitor system health continuously
- Provide emergency shutdown procedures

## Support and Resources

### Documentation
- System Architecture: `docs/architecture.md`
- API Reference: `docs/api_reference.md`
- Data Validation: `docs/data_validation.md`
- CAN Protocol: `docs/can_protocol.md`

### Tools
- STM32CubeIDE: STM32 development environment
- STM32CubeMX: Hardware configuration tool
- CAN Analyzer: Bus monitoring and analysis
- Oscilloscope: Signal analysis

### Community
- STM32 Community Forum
- CAN Bus User Groups
- Battery Management System Forums
- GitHub Issues and Discussions

This comprehensive build and deployment guide should help you successfully implement and maintain the Distributed BMS system.

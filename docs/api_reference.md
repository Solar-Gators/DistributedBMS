# API Reference Documentation

## Overview

This document provides comprehensive API documentation for all classes and functions in the Distributed BMS system.

## Daughter Board Classes

### FaultManager

**Purpose**: System health monitoring and fault management

**Header**: `DistributedBMSDaughter/Core/Inc/FaultManager.hpp`

#### Enums

##### FaultType
```cpp
enum class FaultType : uint8_t {
    NONE = 0,
    COMMUNICATION_TIMEOUT = 1,
    BQ76920_COMM_ERROR = 2,
    ADC_READ_ERROR = 3,
    CAN_TRANSMIT_ERROR = 4,
    CAN_RECEIVE_ERROR = 5,
    EEPROM_WRITE_ERROR = 6,
    WATCHDOG_TIMEOUT = 7,
    TEMPERATURE_SENSOR_FAIL = 8,
    SPI_COMM_ERROR = 9,
    I2C_COMM_ERROR = 10,
    DMA_ERROR = 11,
    CLOCK_ERROR = 12,
    MEMORY_ERROR = 13,
    MAX_FAULTS = 32
};
```

##### FaultSeverity
```cpp
enum class FaultSeverity : uint8_t {
    INFO = 0,      // Non-critical, just logging
    WARNING = 1,   // Monitor but continue operation
    ERROR = 2,     // System degraded but functional
    CRITICAL = 3   // System non-functional
};
```

#### Structs

##### FaultInfo
```cpp
struct FaultInfo {
    FaultType type;
    FaultSeverity severity;
    uint32_t timestamp_ms;
    uint32_t count;
    bool active;
};
```

#### Constructor
```cpp
FaultManager();
```
Creates a new FaultManager instance with all faults initialized as inactive.

#### Public Methods

##### Fault Management
```cpp
void setFault(FaultType type, bool active, FaultSeverity severity = FaultSeverity::ERROR);
```
Sets or clears a fault condition.

**Parameters**:
- `type`: The type of fault to set/clear
- `active`: True to set fault, false to clear
- `severity`: Severity level (default: ERROR)

**Example**:
```cpp
faultManager.setFault(FaultManager::FaultType::BQ76920_COMM_ERROR, true);
faultManager.setFault(FaultManager::FaultType::ADC_READ_ERROR, false);
```

```cpp
void clearFault(FaultType type);
```
Clears a specific fault condition.

**Parameters**:
- `type`: The fault type to clear

```cpp
bool hasFault(FaultType type) const;
```
Checks if a specific fault is active.

**Returns**: True if fault is active, false otherwise

```cpp
bool hasActiveFaults() const;
```
Checks if any faults are currently active.

**Returns**: True if any faults are active

```cpp
bool hasCriticalFaults() const;
```
Checks if any critical faults are active.

**Returns**: True if any critical faults are active

##### System Status
```cpp
bool isSystemFunctional() const;
```
Checks if the system is functional for normal operation.

**Returns**: True if system can operate normally

```cpp
bool canReadVoltages() const;
```
Checks if voltage reading is possible.

**Returns**: True if voltage reading is functional

```cpp
bool canReadTemperatures() const;
```
Checks if temperature reading is possible.

**Returns**: True if temperature reading is functional

```cpp
bool canTransmitCAN() const;
```
Checks if CAN transmission is possible.

**Returns**: True if CAN transmission is functional

##### Information Retrieval
```cpp
uint32_t getFaultMask() const;
```
Gets the current fault mask (bitfield of active faults).

**Returns**: 32-bit mask with active fault bits set

```cpp
FaultInfo getFaultInfo(FaultType type) const;
```
Gets detailed information about a specific fault.

**Parameters**:
- `type`: The fault type to query

**Returns**: FaultInfo struct with fault details

```cpp
uint8_t getActiveFaultCount() const;
```
Gets the number of currently active faults.

**Returns**: Count of active faults (0-32)

##### Maintenance
```cpp
void update(uint32_t current_time_ms);
```
Updates the fault manager with current timestamp.

**Parameters**:
- `current_time_ms`: Current system time in milliseconds

```cpp
void clearAllFaults();
```
Clears all active faults.

---

### DataValidator

**Purpose**: Data validation and quality assessment

**Header**: `DistributedBMSDaughter/Core/Inc/DataValidator.hpp`

#### Enums

##### ValidationError
```cpp
enum class ValidationError : uint8_t {
    NONE = 0,
    OUT_OF_RANGE = 1,
    OUTLIER_DETECTED = 2,
    NOISE_EXCESSIVE = 3,
    RATE_TOO_HIGH = 4,
    STUCK_VALUE = 5,
    INCONSISTENT_DATA = 6,
    SENSOR_FAILURE = 7,
    CALIBRATION_ERROR = 8
};
```

#### Structs

##### ValidationResult
```cpp
struct ValidationResult {
    bool is_valid;
    ValidationError error_code;
    float confidence_score;  // 0.0-1.0
    uint32_t validation_timestamp;
    uint8_t quality_score;  // 0-100
};
```

##### ValidationConfig
```cpp
struct ValidationConfig {
    // Voltage limits (mV)
    uint16_t min_cell_voltage_mV = 2000;  // 2.0V
    uint16_t max_cell_voltage_mV = 4500;  // 4.5V
    float max_voltage_deviation_percent = 10.0f;
    
    // Temperature limits (°C)
    float min_temperature_C = -40.0f;
    float max_temperature_C = 85.0f;
    float max_temp_rate_C_per_sec = 5.0f;
    
    // ADC limits
    uint16_t min_adc_value = 100;
    uint16_t max_adc_value = 3995;
    uint16_t adc_noise_threshold = 50;
    
    // Outlier detection
    uint8_t outlier_window_size = 5;
    float outlier_threshold_sigma = 2.0f;
    
    // Quality thresholds
    uint8_t min_quality_score = 70;
    float min_confidence_score = 0.8f;
};
```

#### Constructors
```cpp
DataValidator();
DataValidator(const ValidationConfig& config);
```

#### Public Methods

##### Configuration
```cpp
void setConfig(const ValidationConfig& config);
```
Sets the validation configuration.

**Parameters**:
- `config`: New validation configuration

```cpp
const ValidationConfig& getConfig() const;
```
Gets the current validation configuration.

**Returns**: Current configuration reference

##### Validation Methods
```cpp
ValidationResult validateCellVoltages(const std::array<uint16_t, 5>& voltages_mV, 
                                    uint32_t timestamp_ms);
```
Validates cell voltage readings.

**Parameters**:
- `voltages_mV`: Array of cell voltages in millivolts
- `timestamp_ms`: Current timestamp

**Returns**: ValidationResult with validation status

**Example**:
```cpp
std::array<uint16_t, 5> cellVoltages = {3700, 3650, 3720, 3680, 0};
auto result = validator.validateCellVoltages(cellVoltages, HAL_GetTick());
if (!result.is_valid) {
    // Handle validation failure
}
```

```cpp
ValidationResult validateTemperatures(const std::array<float, 5>& temperatures_C, 
                                    uint32_t timestamp_ms);
```
Validates temperature readings.

**Parameters**:
- `temperatures_C`: Array of temperatures in Celsius
- `timestamp_ms`: Current timestamp

**Returns**: ValidationResult with validation status

```cpp
ValidationResult validateADCReadings(const std::array<uint16_t, 5>& adc_values, 
                                   uint32_t timestamp_ms);
```
Validates ADC readings.

**Parameters**:
- `adc_values`: Array of ADC values (0-4095)
- `timestamp_ms`: Current timestamp

**Returns**: ValidationResult with validation status

```cpp
ValidationResult validateAllData(const std::array<uint16_t, 5>& voltages_mV,
                               const std::array<float, 5>& temperatures_C,
                               const std::array<uint16_t, 5>& adc_values,
                               uint32_t timestamp_ms);
```
Validates all sensor data together.

**Parameters**:
- `voltages_mV`: Cell voltage array
- `temperatures_C`: Temperature array
- `adc_values`: ADC values array
- `timestamp_ms`: Current timestamp

**Returns**: Combined validation result

##### Historical Data
```cpp
void updateHistory(uint32_t timestamp_ms);
```
Updates historical data for outlier detection.

**Parameters**:
- `timestamp_ms`: Current timestamp

```cpp
void clearHistory();
```
Clears all historical data.

##### Quality Assessment
```cpp
uint8_t calculateOverallQuality(const ValidationResult& voltage_result,
                              const ValidationResult& temp_result,
                              const ValidationResult& adc_result) const;
```
Calculates overall data quality score.

**Parameters**:
- `voltage_result`: Voltage validation result
- `temp_result`: Temperature validation result
- `adc_result`: ADC validation result

**Returns**: Overall quality score (0-100)

---

### BMS

**Purpose**: Core battery monitoring and statistics calculation

**Header**: `DistributedBMSDaughter/Core/Inc/BMS.hpp`

#### Structs

##### ThermParams
```cpp
struct ThermParams {
    float A, B, C;           // NTC calculation coefficients
    float rfix_k;            // Fixed resistor value (kΩ)
    float vref;              // Reference voltage
    float adc_fs;            // ADC full scale value
    
    ThermParams() : A(0.002687481f), B(0.0002829040f), C(0.000001183565f),
                   rfix_k(10.0f), vref(3.3f), adc_fs(4096.0f) {}
};
```

##### Results
```cpp
struct Results {
    uint16_t avg_cell_mV = 0;
    uint16_t high_cell_mV = 0;
    uint16_t low_cell_mV = 0;
    uint8_t high_cell_phys_idx = 0;
    uint8_t low_cell_phys_idx = 0;

    std::array<float,5> ntc_C{};
    float avg_C = 0.0f;
    float high_C = -1000.0f;
    uint8_t high_temp_idx = 0;

    uint8_t num_cells = 3;
    
    // Validation fields
    DataValidator::ValidationResult voltage_validation;
    DataValidator::ValidationResult temperature_validation;
    DataValidator::ValidationResult adc_validation;
    uint8_t data_quality_score = 100;
    bool has_outliers = false;
    bool validation_passed = true;
};
```

#### Constructor
```cpp
explicit BMS(uint8_t num_cells = 4, ThermParams tp = ThermParams{});
```

**Parameters**:
- `num_cells`: Number of cells to monitor (3-5)
- `tp`: Thermistor calculation parameters

#### Public Methods

##### Configuration
```cpp
void set_num_cells(uint8_t n);
```
Sets the number of cells to monitor.

**Parameters**:
- `n`: Number of cells (3-5)

```cpp
uint8_t num_cells() const;
```
Gets the current number of cells.

**Returns**: Current cell count

##### Data Input
```cpp
void set_cell_mV(const std::array<uint16_t,5>& mV);
```
Sets cell voltage readings.

**Parameters**:
- `mV`: Array of cell voltages in millivolts

```cpp
void set_ntc_volts(const std::array<float,5>& v);
```
Sets NTC voltage readings.

**Parameters**:
- `v`: Array of NTC voltages

```cpp
void set_ntc_counts(const std::array<uint16_t,5>& counts);
```
Sets NTC ADC count readings.

**Parameters**:
- `counts`: Array of ADC counts

##### Processing
```cpp
void update();
```
Processes all input data and calculates statistics.

```cpp
const Results& results() const;
```
Gets the calculated results.

**Returns**: Reference to results structure

##### Data Access
```cpp
uint16_t average_cell_mV() const;
uint16_t high_cell_mV() const;
uint16_t low_cell_mV() const;
uint8_t high_cell_index() const;
uint8_t low_cell_index() const;
float average_temp_C() const;
float high_temp_C() const;
uint8_t high_temp_index() const;
```
Getters for individual calculated values.

##### Maintenance
```cpp
void clear();
```
Clears all data and resets results.

---

### CanBus

**Purpose**: CAN bus communication interface

**Header**: `DistributedBMSDaughter/Core/Inc/CanBus.hpp`

#### Enums

##### Result
```cpp
enum class Result : uint8_t {
    Ok, Busy, Error, Timeout, NoMailboxes
};
```

#### Structs

##### Frame
```cpp
struct Frame {
    uint32_t id = 0;         // 11-bit if extended==false, 29-bit if true
    bool extended = false;    // Extended frame flag
    bool rtr = false;        // Remote transmission request
    uint8_t dlc = 0;         // Data length code (0-8)
    uint8_t data[8]{};       // Data payload
    uint32_t timestamp = 0;  // Timestamp
};
```

#### Constructor
```cpp
explicit CanBus(CAN_HandleTypeDef& h);
```

**Parameters**:
- `h`: STM32 HAL CAN handle

#### Public Methods

##### Initialization
```cpp
bool start();
```
Starts the CAN bus and enables FIFO0 RX interrupt.

**Returns**: True if successful

##### Filter Configuration
```cpp
bool configureFilterAcceptAll(uint32_t bank = 0);
```
Configures filter to accept all messages.

**Parameters**:
- `bank`: Filter bank number

**Returns**: True if successful

```cpp
bool configureFilterStdMask(uint16_t filter, uint16_t mask,
                          uint32_t bank = 0, bool into_fifo0 = true);
```
Configures standard ID mask filter.

**Parameters**:
- `filter`: Filter ID
- `mask`: Filter mask
- `bank`: Filter bank number
- `into_fifo0`: Route to FIFO0

**Returns**: True if successful

##### Transmission
```cpp
Result sendStd(uint16_t id, const uint8_t* payload, uint8_t len, bool rtr = false);
Result sendStd(uint16_t id, const std::array<uint8_t,8>& p, uint8_t len, bool rtr = false);
Result sendExt(uint32_t id, const uint8_t* payload, uint8_t len, bool rtr = false);
Result sendExt(uint32_t id, const std::array<uint8_t,8>& p, uint8_t len, bool rtr = false);
```
Send CAN messages.

**Parameters**:
- `id`: CAN ID
- `payload`: Data payload
- `len`: Data length
- `rtr`: Remote transmission request flag

**Returns**: Transmission result

##### Reception
```cpp
bool available() const;
```
Checks if frames are available for reading.

**Returns**: True if frames available

```cpp
bool read(Frame& out);
```
Reads one frame from the receive queue.

**Parameters**:
- `out`: Frame structure to fill

**Returns**: True if frame read successfully

```cpp
size_t rx_count() const;
```
Gets the number of frames in receive queue.

**Returns**: Frame count

```cpp
size_t rx_dropped() const;
```
Gets the number of dropped frames.

**Returns**: Drop count

##### ISR Interface
```cpp
void onRxFifo0Pending();
```
Called from HAL_CAN_RxFifo0MsgPendingCallback.

##### Statistics
```cpp
uint32_t tx_ok() const;
uint32_t tx_err() const;
```
Gets transmission statistics.

**Returns**: Success/error counts

##### Static Methods
```cpp
static void attach_isr_instance(CanBus* inst);
static CanBus* isr_instance();
```
ISR instance management for interrupt handling.

---

### BQ7692000PW

**Purpose**: Driver for BQ76920 battery monitor IC

**Header**: `DistributedBMSDaughter/Core/Inc/BQ7692000.hpp`

#### Constructor
```cpp
BQ7692000PW(I2C_HandleTypeDef *h);
```

**Parameters**:
- `h`: STM32 HAL I2C handle

#### Public Methods

##### Initialization
```cpp
HAL_StatusTypeDef init();
```
Initializes the BQ76920 IC and enables Coulomb Counting and ADC.

**Returns**: HAL status

##### Voltage Reading
```cpp
HAL_StatusTypeDef getVC(std::array<uint16_t, CELL_COUNT> &vc_values);
```
Reads all cell voltages.

**Parameters**:
- `vc_values`: Array to store voltage values

**Returns**: HAL status

```cpp
HAL_StatusTypeDef getBAT(uint16_t *data);
```
Reads battery pack voltage.

**Parameters**:
- `data`: Pointer to store battery voltage

**Returns**: HAL status

##### Temperature Reading
```cpp
HAL_StatusTypeDef getDieTemp(uint16_t *data);
```
Reads die temperature.

**Parameters**:
- `data`: Pointer to store temperature

**Returns**: HAL status

##### Coulomb Counting
```cpp
HAL_StatusTypeDef getCC(uint16_t *data);
```
Reads Coulomb count register.

**Parameters**:
- `data`: Pointer to store CC value

**Returns**: HAL status

##### Balancing Control
```cpp
HAL_StatusTypeDef getActiveBalancing(uint8_t *activeBal);
HAL_StatusTypeDef setActiveBalancing(uint8_t *activeBal);
```
Gets/sets active balancing register.

**Parameters**:
- `activeBal`: Balancing register value

**Returns**: HAL status

---

## Secondary Board Classes

### BmsFleet

**Purpose**: Fleet management and data aggregation

**Header**: `DistributedBMSSecondary/Core/Inc/BmsFleet.hpp`

#### Structs

##### ModuleData
```cpp
struct ModuleData {
    float high_C = -1000.f;
    uint8_t high_temp_idx = 0;
    uint16_t high_mV = 0, low_mV = 0;
    uint8_t low_idx = 0, high_idx = 0;
    float avg_C = 0.f;
    uint16_t avg_cell_mV = 0;
    uint8_t num_cells = 0;
    uint32_t last_ms = 0;
    bool got_type0 = false, got_type1 = false, got_type2 = false;

    void clear();
    bool online(uint32_t now_ms, uint32_t stale_ms = BmsFleetCfg::STALE_MS) const;
};
```

##### IdMapEntry
```cpp
struct IdMapEntry {
    uint16_t can_id = 0;
    uint8_t index = 0;
    bool used = false;
};
```

#### Constructor
```cpp
BmsFleet();
```

#### Public Methods

##### Node Management
```cpp
bool register_node(uint16_t can_id, uint8_t idx);
```
Registers a new CAN node.

**Parameters**:
- `can_id`: CAN ID of the node
- `idx`: Module index

**Returns**: True if successful

##### Data Handling
```cpp
void handle(const CanBus::Frame& rx, uint32_t now_ms);
```
Handles incoming CAN frames.

**Parameters**:
- `rx`: Received CAN frame
- `now_ms`: Current timestamp

##### Data Access
```cpp
ModuleData& module(uint8_t idx);
const ModuleData& module(uint8_t idx) const;
```
Gets module data by index.

**Parameters**:
- `idx`: Module index

**Returns**: Module data reference

##### Fleet Analysis
```cpp
int hottest_module(uint32_t now_ms, float* out_temp = nullptr) const;
```
Finds the module with highest temperature.

**Parameters**:
- `now_ms`: Current timestamp
- `out_temp`: Pointer to store temperature (optional)

**Returns**: Module index or -1 if none found

```cpp
int lowest_cell_module(uint32_t now_ms, uint16_t* out_mV = nullptr) const;
```
Finds the module with lowest cell voltage.

**Parameters**:
- `now_ms`: Current timestamp
- `out_mV`: Pointer to store voltage (optional)

**Returns**: Module index or -1 if none found

---

## CAN Frame Utilities

### CanFrames Namespace

**Purpose**: CAN message encoding and decoding

**Headers**: 
- `DistributedBMSDaughter/Core/Src/CanFrame.cpp`
- `DistributedBMSSecondary/Core/Inc/CanFrames.hpp`

#### Enums

##### MessageType
```cpp
enum MessageType : uint8_t {
    HIGH_TEMP = 0,
    VOLTAGE_EXTREMES = 1,
    AVERAGES = 2
};
```

#### Structs

##### Frame
```cpp
struct Frame {
    std::array<uint8_t, 8> data{};
    uint8_t dlc = 8;
};
```

#### Encoding Functions
```cpp
Frame encodeHighTemp(float highTemp, uint8_t highIndex);
Frame encodeVoltageExtremes(uint16_t highV, uint16_t lowV,
                          uint8_t lowIdx, uint8_t highIdx);
Frame encodeAverages(float avgTemp, uint16_t avgVoltage, uint8_t numCells);
```

#### Decoding Functions
```cpp
uint8_t getType(const uint8_t* data);
bool decodeHighTemp(const uint8_t* data, float& temp, uint8_t& idx);
bool decodeVoltageExtremes(const uint8_t* data, uint16_t& highV,
                         uint16_t& lowV, uint8_t& lowIdx, uint8_t& highIdx);
bool decodeAverages(const uint8_t* data, float& avgTemp,
                   uint16_t& avgVoltage, uint8_t& numCells);
```

---

## Usage Examples

### Basic Fault Management
```cpp
FaultManager faultManager;

// Set a fault
faultManager.setFault(FaultManager::FaultType::BQ76920_COMM_ERROR, true);

// Check system status
if (faultManager.isSystemFunctional()) {
    // System is working normally
}

// Clear fault when condition improves
faultManager.clearFault(FaultManager::FaultType::BQ76920_COMM_ERROR);
```

### Data Validation
```cpp
DataValidator validator;

// Validate cell voltages
std::array<uint16_t, 5> voltages = {3700, 3650, 3720, 3680, 0};
auto result = validator.validateCellVoltages(voltages, HAL_GetTick());

if (result.is_valid) {
    // Data is valid, proceed with processing
    uint8_t quality = result.quality_score;
} else {
    // Handle validation failure
    ValidationError error = result.error_code;
}
```

### CAN Communication
```cpp
CanBus can(hcan1);
can.configureFilterAcceptAll();
can.start();

// Send data
std::array<uint8_t, 8> data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
if (can.sendStd(0x123, data, 8) == CanBus::Result::Ok) {
    // Transmission successful
}

// Receive data
CanBus::Frame frame;
if (can.read(frame)) {
    // Process received frame
    uint32_t id = frame.id;
    uint8_t* data = frame.data;
}
```

### BMS Processing
```cpp
BMS bms(4); // 4-cell configuration

// Set input data
std::array<uint16_t, 5> cellVoltages = {3700, 3650, 3720, 3680, 0};
std::array<uint16_t, 5> adcValues = {2048, 2000, 2100, 2050, 0};

bms.set_cell_mV(cellVoltages);
bms.set_ntc_counts(adcValues);
bms.update();

// Get results
const auto& results = bms.results();
uint16_t avgVoltage = results.avg_cell_mV;
float avgTemp = results.avg_C;
```

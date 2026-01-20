/*
 * User.cpp
 *
 *  Created on: Jan 2025
 *      Author: samrb
 */

#include "User.hpp"

// Lower Level Drivers
#include "bts71040.hpp"
#include "PrimaryBmsFleet.hpp"
#include "BmsManager.hpp"     // New high-level BMS manager
#include "lsm6dso32.hpp"
#include "ina226.hpp"
#include "ads1115.hpp"
#include "CanFdBus.hpp"
#include "BmsCanInterface.hpp"


// Shared utilities
#include "../../../DistributedBMSCommon/Inc/UartCrc16.hpp"

// C++ stuff
#include <string.h>
#include <cstdio>
#include <cstdint>
#include <cmath>

// Constants
#define USB_BUFLEN 128
#define RXBUF_SIZE    128
#define MAX_PACKET_LEN 128  // Maximum UART payload size

// UART message queue - simple ring buffer for decoupling ISR from main loop
#define UART_QUEUE_SIZE 16
struct UartMessage {
    uint8_t payload[MAX_PACKET_LEN];
    uint16_t len;
    uint32_t timestamp_ms;
};

static UartMessage uart_queue[UART_QUEUE_SIZE];
static volatile uint16_t uart_queue_write = 0;
static volatile uint16_t uart_queue_read = 0;
static volatile uint16_t uart_queue_count = 0;



// Fleet data structure to store received data from Secondary MCU
static PrimaryBmsFleet fleet;

// High-level BMS manager
static BmsManager* bms_manager = nullptr;

// CAN bus and interface
static CanFdBus* can_bus = nullptr;
static BmsCanInterface* can_interface = nullptr;

// UART buffers
static uint8_t rxbuf[128];
// Note: tempBuf and rx_size removed - no longer needed with queue-based processing

// Debug counters for UART reception tracking
volatile uint32_t uart_crc_errors = 0;           // Count of CRC failures
volatile uint32_t uart_fleet_summary_count = 0;  // Count of successful 0x10 messages
volatile uint32_t uart_module_summary_count = 0; // Count of successful 0x11 messages
volatile uint32_t uart_heartbeat_count = 0;      // Count of successful 0x12 messages
volatile uint32_t uart_errors = 0;               // Count of UART hardware errors (total)
volatile uint32_t uart_errors_ore = 0;           // Overrun errors (data too fast)
volatile uint32_t uart_errors_fe = 0;            // Framing errors (baud rate/noise)
volatile uint32_t uart_errors_ne = 0;            // Noise errors (electrical noise)
volatile uint32_t uart_errors_pe = 0;            // Parity errors (should be 0, parity disabled)
volatile uint32_t uart_invalid_length = 0;       // Count of invalid payload lengths
volatile uint32_t uart_invalid_type = 0;         // Count of unknown message types
volatile uint32_t uart_queue_full_count = 0;     // Count of messages dropped due to full queue
volatile uint8_t uart_last_message_type = 0;     // Last successfully received message type
volatile uint32_t uart_last_message_time = 0;    // Timestamp of last successful message

// USB buffer
uint8_t usbTxBuf[128];

// ADC driver
ADS1115 adc(
    &hi2c2,
    ADS1115::Addr7::GND,
    ADS1115::Pga::FS_4_096V,
    ADS1115::DataRate::SPS_128
);

// IMU driver (static allocation, initialized in setup)
static LSM6DSO32* imu = nullptr;

// Current monitor (static allocation, initialized in setup)
static INA226* ina = nullptr;

// UART packet enqueue - ISR-safe, just copies data to queue
static void uart_enqueue_packet(const uint8_t* payload, uint16_t len, uint32_t now_ms)
{
    if (len < 1 || len > MAX_PACKET_LEN) return;
    
    // Check if queue is full (simple atomic check)
    if (uart_queue_count >= UART_QUEUE_SIZE) {
        // Queue full - drop message and track for monitoring
        uart_queue_full_count++;
        return;
    }
    
    // Copy message to queue
    UartMessage* msg = &uart_queue[uart_queue_write];
    memcpy(msg->payload, payload, len);
    msg->len = len;
    msg->timestamp_ms = now_ms;
    
    // Update write pointer (wraps around)
    uart_queue_write = (uart_queue_write + 1) % UART_QUEUE_SIZE;
    
    // Atomically increment count
    uart_queue_count++;
}

// UART packet processing - called from main loop
static void process_uart_message(const UartMessage* msg)
{
    if (!msg || msg->len < 1) return;

    const uint8_t* payload = msg->payload;
    uint16_t len = msg->len;
    uint32_t now_ms = msg->timestamp_ms;

    uart_last_message_type = payload[0];
    uart_last_message_time = now_ms;

    switch (payload[0]) {
        case UART_MODULE_SUMMARY: {
            fleet.update_module_summary(payload, len, now_ms);
            uart_module_summary_count++;
            break;
        }
        case UART_FLEET_SUMMARY: {
            fleet.update_from_uart_payload(payload, len, now_ms);
            uart_fleet_summary_count++;
            break;
        }
        case UART_HEARTBEAT: {
            fleet.update_heartbeat(payload, len, now_ms);
            uart_heartbeat_count++;
            break;
        }
        default:
            uart_invalid_type++;
            break;
    }
}

// UART callbacks
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance != UART4) return;

    // Get timestamp once for all frames in this batch
    uint32_t now_ms = HAL_GetTick();

    // Optimized frame search - look for SOF pattern more efficiently
    uint16_t i = 0;
    const uint16_t size_minus_4 = size - 4;
    
    while (i <= size_minus_4) {
        // Look for SOF marker (0xA5, 0x5A) - optimized search
        if (rxbuf[i] == 0xA5) {
            if (rxbuf[i+1] == 0x5A) {
                // Found SOF - extract payload length (little-endian)
                uint16_t payload_len = rxbuf[i+2] | ((uint16_t)rxbuf[i+3] << 8);

                // Quick validation
                if (payload_len == 0 || payload_len > MAX_PACKET_LEN) {
                    uart_invalid_length++;
                    i++; // Skip one byte and continue search
                    continue;
                }

                // Frame format: [SOF0] [SOF1] [LEN_LOW] [LEN_HIGH] [PAYLOAD...] [CRC16_LOW] [CRC16_HIGH]
                // Total frame size = 4 (header) + payload_len + 2 (CRC) = 6 + payload_len
                uint16_t total_frame_size = 6 + payload_len;

                // Check if complete frame is available
                if (i + total_frame_size > size) {
                    // incomplete packet, wait for next RX
                    break;
                }

                // Validate CRC - CRC covers [LEN_LOW] [LEN_HIGH] [PAYLOAD...]
                const uint8_t* crc_data = &rxbuf[i + 2];
                if (UartCrc16::validate(crc_data, payload_len)) {
                    // CRC valid - enqueue for processing in main loop (minimal ISR work)
                    uart_enqueue_packet(&rxbuf[i + 4], payload_len, now_ms);
                    
                    // Advance past complete frame
                    i += total_frame_size;
                } else {
                    // CRC mismatch - discard frame
                    uart_crc_errors++;
                    i++; // Skip one byte and continue search
                }
            } else {
                i++; // Continue searching
            }
        } else {
            i++; // Continue searching
        }
    }
    
    // Restart reception AFTER processing - with FIFO mode enabled, this is safe
    // The 8-entry hardware FIFO provides buffering while we were processing
    HAL_UARTEx_ReceiveToIdle_IT(&huart4, rxbuf, sizeof(rxbuf));
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4)
    {
        uint32_t error_code = huart->ErrorCode;
        uart_errors++;
        
        // Track specific error types
        if (error_code & HAL_UART_ERROR_ORE) {
            uart_errors_ore++;
        }
        if (error_code & HAL_UART_ERROR_FE) {
            uart_errors_fe++;
        }
        if (error_code & HAL_UART_ERROR_NE) {
            uart_errors_ne++;
        }
        if (error_code & HAL_UART_ERROR_PE) {
            uart_errors_pe++;
        }
        
        // Clear all error flags
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);

        // Abort and restart reception immediately to prevent data loss
        HAL_UART_AbortReceive_IT(huart); // Use AbortReceive_IT instead of AbortReceive
        HAL_UARTEx_ReceiveToIdle_IT(&huart4, rxbuf, sizeof(rxbuf));
    }
}

// Setup Function
void setup()
{
    // Initialize UART packet receiver
    HAL_UARTEx_ReceiveToIdle_IT(&huart4, rxbuf, sizeof(rxbuf));

    // Create and initialize IMU driver
    imu = new LSM6DSO32(&hi2c2, LSM6DSO32::I2C_ADDR_SA0_LOW << 1);
    if (imu->init(LSM6DSO32::Odr::Hz_104,
                  LSM6DSO32::AccelFs::FS_4G,
                  LSM6DSO32::Odr::Hz_104,
                  LSM6DSO32::GyroFs::FS_2000DPS) != HAL_OK) {
        // handle init error
        while (1);
    }

    // Create and initialize current monitor
    ina = new INA226(&hi2c2, 0x40);
    // 2 mΩ shunt, max expected 100 A
    if (ina->init(0.002f, 100.0f) != HAL_OK) {
        // handle error (blink LED, etc.)
        while (1) {}
    }

    // Initialize ADC
    adc.init();

    // Create and initialize BMS Manager
    bms_manager = new BmsManager(&fleet, &adc, ina);

    // Configure hardware interfaces
    // Configure two main contactors on port B:
    //  - PB0: first main contactor (e.g. negative side)
    //  - PB1: second main contactor (e.g. positive side)
    bms_manager->setContactorGpio(GPIOB, GPIO_PIN_0);
    bms_manager->setSecondContactorGpio(GPIOB, GPIO_PIN_1);
    
    // TODO: Set actual PWM timer and channel for fans
    // bms_manager->setFanPwmTimer(&htim1, TIM_CHANNEL_1);

    // Configure BMS thresholds (optional - defaults are set)
    BmsManager::Config config;
    config.cell_overvoltage_mV = 4200;
    config.cell_undervoltage_mV = 2500;
    config.overtemp_C = 45.0f;
    config.overcurrent_A = 100.0f;
    config.aux_overcurrent_A = 20.0f;
    // Current measurement calibration - adjust based on your hardware
    config.current_shunt_resistance_ohm = 0.001f;  // 1mΩ shunt (adjust as needed)
    config.current_gain = 50.0f;  // Current sense amplifier gain (adjust as needed)
    config.current_offset_V = 0.0f;  // ADC offset (calibrate as needed)
    bms_manager->setConfig(config);

    // Initialize BMS Manager
    bms_manager->init();

    // Initialize CAN bus
    can_bus = new CanFdBus(hfdcan1);
    can_bus->configureFilterAcceptAll(0);  // Accept all messages for now
    if (!can_bus->start()) {
        // CAN initialization failed - handle error
        while (1) {}
    }

    // Initialize CAN interface
    can_interface = new BmsCanInterface(*can_bus, *bms_manager);
    BmsCanInterface::Config can_config;
    can_config.heartbeat_period_ms = 100;   // 10 Hz heartbeat
    can_config.pack_status_period_ms = 50;  // 20 Hz pack status
    can_config.node_id = 0x01;
    can_interface->init(can_config);

    // Allow sensors and measurements to settle before attempting to close contactors
    HAL_Delay(2000);

    // Request contactors to close; BmsManager will sequence both contactors
    bms_manager->requestContactorsClose();
}

// Main loop function
void loop()
{
    uint32_t now_ms = HAL_GetTick();

    // Process UART messages from queue (moved from ISR to main loop for better performance)
    while (uart_queue_count > 0) {
        UartMessage* msg = &uart_queue[uart_queue_read];
        process_uart_message(msg);
        
        // Update read pointer (wraps around)
        uart_queue_read = (uart_queue_read + 1) % UART_QUEUE_SIZE;
        
        // Atomically decrement count
        uart_queue_count--;
    }

    // Update BMS Manager (handles fault detection, state machine, control)
    if (bms_manager != nullptr) {
        bms_manager->update(now_ms);
    }

    // Update CAN bus (handles bus-off recovery)
    if (can_bus != nullptr) {
        can_bus->poll();
    }

    // Update CAN interface (sends periodic messages, processes commands)
    if (can_interface != nullptr) {
        can_interface->update(now_ms);
    }

    // Legacy code - keeping for now during migration
    // TODO: Remove once BmsManager is fully tested
    LSM6DSO32::ScaledData d;
    if (imu) {
        imu->readScaled(d);
    }
    
    INA226::Measurement m;
    if (ina) {
        ina->readMeasurement(m);
    }



    // Update fault LEDs based on BMS Manager state
    if (bms_manager != nullptr) {
        uint16_t faults = bms_manager->getActiveFaults();
        BmsManager::BmsState state = bms_manager->getState();

        // Update LEDs on GPIOC based on state and faults
        if (state == BmsManager::BmsState::SHUTDOWN || bms_manager->hasCriticalFault()) {
            // Critical fault - all LEDs off or error pattern
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
        } else if (faults == 0 && state == BmsManager::BmsState::OPERATIONAL) {
            // All good - LEDs on
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
        } else if (faults != 0) {
            // Fault detected - LEDs off
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
        } else {
            // IDLE state - heartbeat blink on LED0
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
        }
    }

	float result;
	adc.readSingleEnded(0, result);



	float y = 101.4864f
        + (-29.84563f - 101.4864f)
          / (1.0f + std::pow(result / 2.756892f, 2.643521f));


	//float intermediate = ((result-0.07)*1.5);
	//float finalCurrent = (intermediate-2.5)/0.04;

    // OK LED toggle (handled above using GPIOC)
}


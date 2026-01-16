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
#include "BmsController.hpp"
#include "lsm6dso32.hpp"
#include "ina226.hpp"
#include "ads1115.hpp"


// C++ stuff
#include <string.h>
#include <cstdio>
#include <cstdint>

// Constants
#define USB_BUFLEN 128
#define RXBUF_SIZE    128
#define TEMPBUF_SIZE  128
#define MAX_PACKET_LEN (TEMPBUF_SIZE)

// Hardware initialization
BmsController controller;

// Fleet data structure to store received data from Secondary MCU
static PrimaryBmsFleet fleet;

// UART buffers
static uint8_t rxbuf[128];
static volatile uint16_t rx_size;
static uint8_t tempBuf[128];

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

// UART packet callback
static void on_uart_packet(const uint8_t* payload, uint16_t len)
{
    if (len < 1) return;

    switch (payload[0]) {
        case UART_MODULE_SUMMARY: {
            fleet.update_module_summary(payload, len, HAL_GetTick());
            break;
        }
        case UART_FLEET_SUMMARY: {
            fleet.update_from_uart_payload(payload, len, HAL_GetTick());
            break;
        }
        case UART_HEARTBEAT: {
            fleet.update_heartbeat(payload, len, HAL_GetTick());
            break;
        }
        default:
            break;
    }
}

// UART callbacks
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance == UART4) {
        if (rxbuf[0] == 0xA5 && rxbuf[1] == 0x5A) {
            uint16_t length = (uint16_t)rxbuf[2] | ((uint16_t)rxbuf[3] << 8);

            if (length == 0 || length > MAX_PACKET_LEN) {
                // invalid length, resync / drop packet
                return;
            }
            if (length + 4 > RXBUF_SIZE) {
                // not enough data in buffer, drop / wait for more
                return;
            }

            memcpy(tempBuf, rxbuf + 4, length);
            on_uart_packet(tempBuf, length);
        }

        HAL_UARTEx_ReceiveToIdle_IT(&huart4, rxbuf, sizeof(rxbuf)); // re-arm
        HAL_GPIO_TogglePin(GPIOC, OK_Pin);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);

        HAL_UART_AbortReceive(huart); // ensure RxState READY
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
}

// Main loop function
void loop()
{
    LSM6DSO32::ScaledData d;
    if (imu) {
        imu->readScaled(d);
    }
    
    INA226::Measurement m;
    if (ina) {
        ina->readMeasurement(m);
    }

    uint32_t now_ms = HAL_GetTick();

    // Keep fleet summary fresh
    const auto& summary = fleet.summary();

    if(fleet.has_data(now_ms)){
        controller.updateFaultsFromSummary(summary);
    }

    // Update fault LEDs
    if(controller.faults == 0){
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_Delay(500);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    }else{
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_Delay(500);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    }

    // USB debug output
    const auto& M0 = fleet.module(0);

    int usbTxBufLen = snprintf(
        (char*)usbTxBuf,
        USB_BUFLEN,
        "DBG M0: valid=%d avg=%u mV cells=%u low=%u high=%u\r\n",
        M0.valid,
        (unsigned)M0.avg_cell_mV,
        (unsigned)M0.num_cells,
        (unsigned)M0.low_mV,
        (unsigned)M0.high_mV
    );
    if (usbTxBufLen > 0 && usbTxBufLen < USB_BUFLEN) {
        //CDC_Transmit_FS(usbTxBuf, usbTxBufLen);
    }

    // Read ADC
    float result;
    adc.readSingleEnded(0, result);

    float intermediate = ((result-0.07)*1.5)+0.02;

    HAL_GPIO_TogglePin(GPIOC, OK_Pin);
    HAL_Delay(200);
}


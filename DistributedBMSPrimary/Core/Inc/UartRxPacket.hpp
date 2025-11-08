/*
 * UartRxPacket.hpp
 *
 *  Created on: Nov 7, 2025
 *      Author: samrb
 */

#ifndef INC_UARTRXPACKET_HPP_
#define INC_UARTRXPACKET_HPP_
#pragma once

#include <stdint.h>
#include "stm32l5xx_hal.h"  // replace 'xxxx' with your family (e.g. stm32l4xx_hal.h)

#ifdef __cplusplus
extern "C" {
#endif

// Frame: A5 5A | LEN(lo) LEN(hi) | PAYLOAD[LEN] | CRC16(lo) CRC16(hi)
#define UART_SOF0 0xA5
#define UART_SOF1 0x5A

typedef void (*uart_pkt_cb_t)(const uint8_t* payload, uint16_t len, void* user);

typedef struct {
    UART_HandleTypeDef* huart;

    // buffers
    uint8_t  hdr[4];      // SOF0 SOF1 LENlo LENhi
    uint8_t* body;        // PAYLOAD[LEN] + CRC16 (2 bytes)
    uint16_t body_cap;    // capacity of 'body'

    // tiny state (enum lives inside struct → no scope issues)
    enum { RX_WAIT_HDR = 0, RX_WAIT_BODY } state;
    uint16_t expect_len;  // length parsed from header

    // callback
    uart_pkt_cb_t on_packet;
    void* user;
} UartPktRx;

// Init once (provide a body buffer). Then call uart_pkt_start() to arm the first interrupt.
void uart_pkt_init(UartPktRx* r,
                   UART_HandleTypeDef* huart,
                   uint8_t* body_storage,
                   uint16_t body_storage_cap,
                   uart_pkt_cb_t cb,
                   void* user);

// Arms the first interrupt to receive the 4-byte header.
HAL_StatusTypeDef uart_pkt_start(UartPktRx* r);

// Call these from HAL callbacks:
void uart_pkt_on_rx_cplt(UartPktRx* r);   // from HAL_UART_RxCpltCallback
void uart_pkt_on_error(UartPktRx* r);     // from HAL_UART_ErrorCallback (optional)

#ifdef __cplusplus
}
#endif



#endif /* INC_UARTRXPACKET_HPP_ */

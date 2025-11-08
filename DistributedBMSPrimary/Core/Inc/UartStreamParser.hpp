/*
 * UartStreamParser.hpp
 *
 *  Created on: Nov 7, 2025
 *      Author: samrb
 */

#ifndef INC_UARTSTREAMPARSER_HPP_
#define INC_UARTSTREAMPARSER_HPP_

#pragma once
#include <stdint.h>
#include <stddef.h>
#include "stm32l5xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// Same SOF as your encoder
#define UART_SOF0 0xA5
#define UART_SOF1 0x5A

// Application callback when a full payload is decoded
typedef void (*uart_payload_cb_t)(const uint8_t* payload, uint16_t len, void* user);

// Parser state
typedef struct {
    uint8_t  state;
    uint16_t expect_len;     // payload length from header
    uint16_t index;          // bytes collected so far in PAYLOAD
    uint16_t crc;            // rolling CRC16-CCITT over LEN+PAYLOAD
    uint8_t  payload_buf[512]; // adjust to your max payload size
    uart_payload_cb_t on_payload;
    void* user;
} UartStreamParser;

void usp_init(UartStreamParser* p, uart_payload_cb_t cb, void* user);
void usp_reset(UartStreamParser* p);

// Feed ONE received byte to the parser.
// Returns:
//   1 -> consumed ok (might have delivered a payload via callback)
//   0 -> waiting for more
//  -1 -> stream error; parser auto-resynced (you can ignore)
int  usp_feed(UartStreamParser* p, uint8_t byte);

// Optional: call from polling loop; reads from HAL one byte at a time
// and feeds the parser. Returns number of bytes consumed this call.
int  usp_poll_uart(UART_HandleTypeDef* huart, UartStreamParser* p, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif




#endif /* INC_UARTSTREAMPARSER_HPP_ */

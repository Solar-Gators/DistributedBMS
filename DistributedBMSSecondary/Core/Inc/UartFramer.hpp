/*
 * UartFramer.hpp
 *
 *  Created on: Nov 7, 2025
 *      Author: samrb
 */

#ifndef INC_UARTFRAMER_HPP_
#define INC_UARTFRAMER_HPP_


#pragma once
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UART_SOF0 0xA5
#define UART_SOF1 0x5A
#define UART_ENCODED_SIZE(payload_len) ((size_t)(6 + (payload_len)))

size_t uart_encode_frame(const uint8_t *payload,
                         uint16_t payload_len,
                         uint8_t *out_buf,
                         size_t out_max);

#ifdef __cplusplus
}
#endif



#endif /* INC_UARTFRAMER_HPP_ */

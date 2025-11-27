/*
 * UartFramer.cpp
 *
 *  Created on: Nov 7, 2025
 *      Author: samrb
 */


#include "UartFramer.hpp"
#include <string.h>

static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
    }
    return crc;
}

size_t uart_encode_frame(const uint8_t *payload,
                         uint16_t payload_len,
                         uint8_t *out_buf,
                         size_t out_max)
{
    if (!payload || !out_buf) return 0;
    const size_t need = UART_ENCODED_SIZE(payload_len);
    if (out_max < need) return 0;

    out_buf[0] = UART_SOF0;
    out_buf[1] = UART_SOF1;

    out_buf[2] = (uint8_t)(payload_len & 0xFF);
    out_buf[3] = (uint8_t)((payload_len >> 8) & 0xFF);

    if (payload_len) memcpy(&out_buf[4], payload, payload_len);

    const uint16_t crc = crc16_ccitt(&out_buf[2], (uint16_t)(2 + payload_len));
    out_buf[4 + payload_len] = (uint8_t)(crc & 0xFF);
    out_buf[5 + payload_len] = (uint8_t)((crc >> 8) & 0xFF);

    return need;
}



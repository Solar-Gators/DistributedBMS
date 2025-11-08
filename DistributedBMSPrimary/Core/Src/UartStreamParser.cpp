/*
 * UartStreamParser.cpp
 *
 *  Created on: Nov 7, 2025
 *      Author: samrb
 */


#include "UartStreamParser.hpp"
#include "stm32l5xx_hal.h" // replace xxxx with your family (e.g., stm32l4xx_hal.h)

enum {
    S_WAIT_SOF0 = 0,
    S_WAIT_SOF1,
    S_LEN0,
    S_LEN1,
    S_PAYLOAD,
    S_CRC0,
    S_CRC1
};

// CRC16-CCITT (poly 0x1021, init 0xFFFF)
static uint16_t crc16_step(uint16_t crc, uint8_t data)
{
    crc ^= (uint16_t)data << 8;
    for (int b = 0; b < 8; ++b) {
        crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                             : (uint16_t)(crc << 1);
    }
    return crc;
}

void usp_init(UartStreamParser* p, uart_payload_cb_t cb, void* user)
{
    p->on_payload = cb;
    p->user = user;
    usp_reset(p);
}

void usp_reset(UartStreamParser* p)
{
    p->state = S_WAIT_SOF0;
    p->expect_len = 0;
    p->index = 0;
    p->crc = 0xFFFF;
}

static void usp_deliver(UartStreamParser* p)
{
    if (p->on_payload) {
        p->on_payload(p->payload_buf, p->expect_len, p->user);
    }
}

int usp_feed(UartStreamParser* p, uint8_t byte)
{
    switch (p->state) {

    case S_WAIT_SOF0:
        if (byte == UART_SOF0) {
            p->state = S_WAIT_SOF1;
            p->crc = 0xFFFF; // reset here; we don't CRC SOF
        }
        return 0;

    case S_WAIT_SOF1:
        if (byte == UART_SOF1) {
            p->state = S_LEN0;
        } else {
            // stay hunting for SOF0 again
            p->state = S_WAIT_SOF0;
        }
        return 0;

    case S_LEN0:
        p->expect_len = (uint16_t)byte;           // LSB
        p->crc = crc16_step(p->crc, byte);        // CRC includes LEN LSB
        p->state = S_LEN1;
        return 0;

    case S_LEN1:
        p->expect_len |= ((uint16_t)byte << 8);   // MSB
        p->crc = crc16_step(p->crc, byte);        // CRC includes LEN MSB
        p->index = 0;

        if (p->expect_len > sizeof(p->payload_buf)) {
            // Length too large -> resync (drop to SOF hunt)
            p->state = S_WAIT_SOF0;
            return -1;
        }
        p->state = (p->expect_len == 0) ? S_CRC0 : S_PAYLOAD;
        return 0;

    case S_PAYLOAD:
        p->payload_buf[p->index++] = byte;
        p->crc = crc16_step(p->crc, byte);

        if (p->index >= p->expect_len) {
            p->state = S_CRC0;
        }
        return 0;

    case S_CRC0: {
        uint8_t crc_lo = (uint8_t)(p->crc & 0xFF);
        if (byte != crc_lo) {
            // CRC mismatch -> resync by sliding one byte (treat this byte as potential SOF0)
            p->state = (byte == UART_SOF0) ? S_WAIT_SOF1 : S_WAIT_SOF0;
            return -1;
        }
        p->state = S_CRC1;
        return 0;
    }

    case S_CRC1: {
        uint8_t crc_hi = (uint8_t)((p->crc >> 8) & 0xFF);
        if (byte != crc_hi) {
            p->state = (byte == UART_SOF0) ? S_WAIT_SOF1 : S_WAIT_SOF0;
            return -1;
        }
        // Good frame!
        usp_deliver(p);
        // After a valid frame, return to hunting next frame
        p->state = S_WAIT_SOF0;
        return 1;
    }

    default:
        p->state = S_WAIT_SOF0;
        return -1;
    }
}

// Optional polling helper: reads one byte (if available) and feeds parser.
int usp_poll_uart(UART_HandleTypeDef* huart, UartStreamParser* p, uint32_t timeout_ms)
{
    uint8_t ch;
    if (HAL_UART_Receive(huart, &ch, 1, timeout_ms) == HAL_OK) {
        (void)usp_feed(p, ch);
        return 1;
    }
    return 0;
}



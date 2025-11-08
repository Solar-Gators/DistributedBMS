#include "UartRxPacket.hpp"
#include <string.h>

// ---------- CRC16-CCITT (poly 0x1021, init 0xFFFF) ----------
static uint16_t crc16_step(uint16_t crc, uint8_t data)
{
    crc ^= (uint16_t)data << 8;
    for (int b = 0; b < 8; ++b)
        crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    return crc;
}

static uint16_t crc16_len_payload(uint8_t len_lo, uint8_t len_hi,
                                  const uint8_t* payload, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    crc = crc16_step(crc, len_lo);
    crc = crc16_step(crc, len_hi);
    for (uint16_t i = 0; i < len; ++i) crc = crc16_step(crc, payload[i]);
    return crc;
}

// ---------- Helpers ----------
static HAL_StatusTypeDef arm_header(UartPktRx* r)
{
    r->state = UartPktRx::RX_WAIT_HDR;
    return HAL_UART_Receive_IT(r->huart, r->hdr, 4);
}

// Returns shift 0..2 if SOF (A5 5A) found starting at hdr[shift], else -1.
static int find_sof_shift(const uint8_t h[4])
{
    for (int s = 0; s <= 2; ++s)
        if (h[s] == UART_SOF0 && h[s + 1] == UART_SOF1) return s;
    return -1;
}

// ---------- Public API ----------
void uart_pkt_init(UartPktRx* r,
                   UART_HandleTypeDef* huart,
                   uint8_t* body_storage,
                   uint16_t body_storage_cap,
                   uart_pkt_cb_t cb,
                   void* user)
{
    r->huart     = huart;
    r->body      = body_storage;
    r->body_cap  = body_storage_cap;
    r->on_packet = cb;
    r->user      = user;
    r->state     = UartPktRx::RX_WAIT_HDR;
    r->expect_len= 0;
}

HAL_StatusTypeDef uart_pkt_start(UartPktRx* r)
{
    return arm_header(r);
}

void uart_pkt_on_rx_cplt(UartPktRx* r)
{
    if (r->state == UartPktRx::RX_WAIT_HDR) {
        // Try to align on SOF in these 4 bytes
        int s = find_sof_shift(r->hdr);
        if (s == -1) {
            // No SOF in these 4 bytes → read 4 fresh bytes
            (void)arm_header(r);
            return;
        }
        if (s != 0) {
            // Slide bytes so header starts at index 0, then read the missing bytes
            // Example: s==1 → we have hdr[1..3]; need 1 more to complete the header
            uint8_t tmp[4];
            tmp[0] = r->hdr[s + 0];
            tmp[1] = r->hdr[s + 1];
            tmp[2] = (s + 2 < 4) ? r->hdr[s + 2] : 0;
            tmp[3] = (s + 3 < 4) ? r->hdr[s + 3] : 0;
            memcpy(r->hdr, tmp, 4);

            uint16_t remain = (uint16_t)s; // missing bytes to complete 4-byte header
            (void)HAL_UART_Receive_IT(r->huart, &r->hdr[4 - remain], remain);
            return; // next callback will re-enter here, s will become 0
        }

        // Header aligned at hdr[0..3]
        uint16_t len = (uint16_t)r->hdr[2] | ((uint16_t)r->hdr[3] << 8);
        r->expect_len = len;

        // Compute required body size and check capacity
        const uint16_t body_need = (uint16_t)(len + 2u); // payload + CRC16
        if (body_need > r->body_cap) {
            // Won't fit → drop and re-arm header
            (void)arm_header(r);
            return;
        }

        r->state = UartPktRx::RX_WAIT_BODY;
        (void)HAL_UART_Receive_IT(r->huart, r->body, body_need);
        return;
    }

    if (r->state == UartPktRx::RX_WAIT_BODY) {
        // Validate CRC and deliver payload
        const uint16_t len    = r->expect_len;
        const uint16_t rx_crc = (uint16_t)r->body[len] | ((uint16_t)r->body[len + 1] << 8);
        const uint16_t calc   = crc16_len_payload(r->hdr[2], r->hdr[3], r->body, len);

        if (calc == rx_crc && r->on_packet) {
            r->on_packet(r->body, len, r->user);
        }

        // Re-arm for next packet header regardless of CRC
        (void)arm_header(r);
        return;
    }

    // Fallback: always re-arm header
    (void)arm_header(r);
}

void uart_pkt_on_error(UartPktRx* r)
{
    // Clear common error flags and restart header reception
    __HAL_UART_CLEAR_OREFLAG(r->huart);
    __HAL_UART_CLEAR_FEFLAG(r->huart);
    __HAL_UART_CLEAR_NEFLAG(r->huart);
    (void)HAL_UART_AbortReceive_IT(r->huart);
    (void)arm_header(r);
}

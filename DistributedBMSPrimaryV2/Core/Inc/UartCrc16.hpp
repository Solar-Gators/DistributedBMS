/*
 * UartCrc16.hpp
 *
 * CRC-16-CCITT implementation for UART frame validation
 * Shared between Primary and Secondary MCUs
 */

#ifndef INC_UARTCRC16_HPP_
#define INC_UARTCRC16_HPP_

#pragma once

#include <cstdint>
#include <cstddef>

namespace UartCrc16 {
    /**
     * Calculate CRC-16-CCITT over data
     * 
     * Polynomial: 0x1021
     * Initial value: 0xFFFF
     * 
     * @param data Pointer to data
     * @param len Length of data in bytes
     * @return 16-bit CRC value
     */
    inline uint16_t calculate(const uint8_t* data, size_t len)
    {
        constexpr uint16_t POLY = 0x1021;
        constexpr uint16_t INIT = 0xFFFF;
        
        uint16_t crc = INIT;
        
        for (size_t i = 0; i < len; ++i) {
            crc ^= (static_cast<uint16_t>(data[i]) << 8);
            
            for (uint8_t j = 0; j < 8; ++j) {
                if (crc & 0x8000) {
                    crc = (crc << 1) ^ POLY;
                } else {
                    crc <<= 1;
                }
            }
        }
        
        return crc;
    }
    
    /**
     * Validate CRC-16-CCITT of a frame
     * 
     * Frame format: [LEN_LOW] [LEN_HIGH] [PAYLOAD...] [CRC16_LOW] [CRC16_HIGH]
     * CRC is calculated over [LEN_LOW] [LEN_HIGH] [PAYLOAD...]
     * 
     * @param frame_data Pointer to frame data (starting at length field)
     * @param payload_len Length of payload (from length field)
     * @return true if CRC is valid
     */
    inline bool validate(const uint8_t* frame_data, size_t payload_len)
    {
        // CRC covers: [LEN_LOW] [LEN_HIGH] [PAYLOAD...]
        size_t crc_data_len = 2 + payload_len;  // 2 bytes for length + payload
        
        // CRC is stored after payload
        const uint8_t* crc_data = frame_data;
        const uint8_t* received_crc = frame_data + crc_data_len;
        
        uint16_t calculated = calculate(crc_data, crc_data_len);
        uint16_t received = (static_cast<uint16_t>(received_crc[0]) | 
                            (static_cast<uint16_t>(received_crc[1]) << 8));
        
        return calculated == received;
    }
}

#endif /* INC_UARTCRC16_HPP_ */

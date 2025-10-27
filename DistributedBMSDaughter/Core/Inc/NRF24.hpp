#ifndef NRF24_HPP
#define NRF24_HPP

#include "stm32l4xx_hal.h"   // Change to match your MCU family

// === nRF24L01+ Commands ===
#define NRF24_CMD_R_REGISTER    0x00
#define NRF24_CMD_W_REGISTER    0x20
#define NRF24_CMD_R_RX_PAYLOAD  0x61
#define NRF24_CMD_W_TX_PAYLOAD  0xA0
#define NRF24_CMD_FLUSH_TX      0xE1
#define NRF24_CMD_FLUSH_RX      0xE2
#define NRF24_CMD_NOP           0xFF

// === nRF24L01+ Registers ===
#define NRF24_REG_CONFIG        0x00
#define NRF24_REG_EN_AA         0x01
#define NRF24_REG_EN_RXADDR     0x02
#define NRF24_REG_SETUP_AW      0x03
#define NRF24_REG_SETUP_RETR    0x04
#define NRF24_REG_RF_CH         0x05
#define NRF24_REG_RF_SETUP      0x06
#define NRF24_REG_STATUS        0x07
#define NRF24_REG_RX_ADDR_P0    0x0A
#define NRF24_REG_TX_ADDR       0x10
#define NRF24_REG_RX_PW_P0      0x11
#define NRF24_REG_FIFO_STATUS   0x17

class NRF24 {
public:
    NRF24(SPI_HandleTypeDef *hspi,
          GPIO_TypeDef *cePort, uint16_t cePin,
          GPIO_TypeDef *csnPort, uint16_t csnPin);

    void init(void);
    void setRxMode(uint8_t *address, uint8_t channel, uint8_t payloadSize);
    void setTxMode(uint8_t *address, uint8_t channel);

    bool sendData(uint8_t *data, uint8_t length);
    bool receiveData(uint8_t *data, uint8_t length);

    // === New helper for board bring-up ===
    bool selfTest();

private:
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cePort;
    uint16_t cePin;
    GPIO_TypeDef *csnPort;
    uint16_t csnPin;

    void csnLow();
    void csnHigh();
    void ceLow();
    void ceHigh();

    uint8_t readRegister(uint8_t reg);
    void writeRegister(uint8_t reg, uint8_t value);
    void writeRegisterMulti(uint8_t reg, uint8_t *data, uint8_t length);
    void readRegisterMulti(uint8_t reg, uint8_t *data, uint8_t length);
    void flushTX();
    void flushRX();
};

#endif // NRF24_HPP

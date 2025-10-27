#include "NRF24.hpp"

NRF24::NRF24(SPI_HandleTypeDef *hspi,
             GPIO_TypeDef *cePort, uint16_t cePin,
             GPIO_TypeDef *csnPort, uint16_t csnPin) {
    this->hspi = hspi;
    this->cePort = cePort;
    this->cePin = cePin;
    this->csnPort = csnPort;
    this->csnPin = csnPin;
}

void NRF24::init(void) {
    ceLow();
    csnHigh();
    HAL_Delay(5);

    writeRegister(NRF24_REG_CONFIG, 0x0C);   // EN_CRC, PWR_UP
    writeRegister(NRF24_REG_EN_AA, 0x01);    // Auto-ACK Pipe0
    writeRegister(NRF24_REG_EN_RXADDR, 0x01);// Enable Pipe0
    writeRegister(NRF24_REG_SETUP_AW, 0x03); // 5-byte address
    writeRegister(NRF24_REG_SETUP_RETR, 0x3F);// Retransmit
    writeRegister(NRF24_REG_RF_CH, 76);      // Channel
    writeRegister(NRF24_REG_RF_SETUP, 0x06); // 1Mbps, 0dBm

    flushTX();
    flushRX();
}

void NRF24::setRxMode(uint8_t *address, uint8_t channel, uint8_t payloadSize) {
    ceLow();
    writeRegister(NRF24_REG_RF_CH, channel);
    writeRegisterMulti(NRF24_REG_RX_ADDR_P0, address, 5);
    writeRegister(NRF24_REG_RX_PW_P0, payloadSize);

    uint8_t config = readRegister(NRF24_REG_CONFIG);
    writeRegister(NRF24_REG_CONFIG, config | 0x01); // PRIM_RX = 1
    ceHigh();
    HAL_Delay(1);
}

void NRF24::setTxMode(uint8_t *address, uint8_t channel) {
    ceLow();
    writeRegister(NRF24_REG_RF_CH, channel);
    writeRegisterMulti(NRF24_REG_TX_ADDR, address, 5);

    uint8_t config = readRegister(NRF24_REG_CONFIG);
    writeRegister(NRF24_REG_CONFIG, config & ~0x01); // PRIM_RX = 0
    ceHigh();
    HAL_Delay(1);
}

bool NRF24::sendData(uint8_t *data, uint8_t length) {
    ceLow();
    csnLow();

    uint8_t cmd = NRF24_CMD_W_TX_PAYLOAD;
    HAL_SPI_Transmit(hspi, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(hspi, data, length, HAL_MAX_DELAY);

    csnHigh();
    ceHigh();
    HAL_Delay(1);
    ceLow();

    uint8_t status;
    do {
        status = readRegister(NRF24_REG_STATUS);
    } while (!(status & (1 << 5)) && !(status & (1 << 4))); // TX_DS / MAX_RT

    writeRegister(NRF24_REG_STATUS, (1 << 5) | (1 << 4));   // Clear flags

    return (status & (1 << 5));
}

bool NRF24::receiveData(uint8_t *data, uint8_t length) {
    uint8_t status = readRegister(NRF24_REG_STATUS);
    if (status & (1 << 6)) { // RX_DR flag
        csnLow();
        uint8_t cmd = NRF24_CMD_R_RX_PAYLOAD;
        HAL_SPI_Transmit(hspi, &cmd, 1, HAL_MAX_DELAY);
        HAL_SPI_Receive(hspi, data, length, HAL_MAX_DELAY);
        csnHigh();

        writeRegister(NRF24_REG_STATUS, (1 << 6)); // Clear RX_DR
        return true;
    }
    return false;
}

// === Private Helpers ===
void NRF24::csnLow()  { HAL_GPIO_WritePin(csnPort, csnPin, GPIO_PIN_RESET); }
void NRF24::csnHigh() { HAL_GPIO_WritePin(csnPort, csnPin, GPIO_PIN_SET); }
void NRF24::ceLow()   { HAL_GPIO_WritePin(cePort, cePin, GPIO_PIN_RESET); }
void NRF24::ceHigh()  { HAL_GPIO_WritePin(cePort, cePin, GPIO_PIN_SET); }

uint8_t NRF24::readRegister(uint8_t reg) {
    uint8_t cmd = NRF24_CMD_R_REGISTER | (reg & 0x1F);
    uint8_t value;
    csnLow();
    HAL_SPI_Transmit(hspi, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(hspi, &value, 1, HAL_MAX_DELAY);
    csnHigh();
    return value;
}

void NRF24::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t cmd = NRF24_CMD_W_REGISTER | (reg & 0x1F);
    csnLow();
    HAL_SPI_Transmit(hspi, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(hspi, &value, 1, HAL_MAX_DELAY);
    csnHigh();
}

void NRF24::writeRegisterMulti(uint8_t reg, uint8_t *data, uint8_t length) {
    uint8_t cmd = NRF24_CMD_W_REGISTER | (reg & 0x1F);
    csnLow();
    HAL_SPI_Transmit(hspi, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(hspi, data, length, HAL_MAX_DELAY);
    csnHigh();
}

void NRF24::readRegisterMulti(uint8_t reg, uint8_t *data, uint8_t length) {
    uint8_t cmd = NRF24_CMD_R_REGISTER | (reg & 0x1F);
    csnLow();
    HAL_SPI_Transmit(hspi, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(hspi, data, length, HAL_MAX_DELAY);
    csnHigh();
}

void NRF24::flushTX() {
    uint8_t cmd = NRF24_CMD_FLUSH_TX;
    csnLow();
    HAL_SPI_Transmit(hspi, &cmd, 1, HAL_MAX_DELAY);
    csnHigh();
}

void NRF24::flushRX() {
    uint8_t cmd = NRF24_CMD_FLUSH_RX;
    csnLow();
    HAL_SPI_Transmit(hspi, &cmd, 1, HAL_MAX_DELAY);
    csnHigh();
}
bool NRF24::selfTest() {
    // Read STATUS (default 0x0E)
    uint8_t status = readRegister(NRF24_REG_STATUS);

    // Write and read CONFIG
    writeRegister(NRF24_REG_CONFIG, 0x0B);
    uint8_t config = readRegister(NRF24_REG_CONFIG);

    return (status == 0x0E && config == 0x0B);
}

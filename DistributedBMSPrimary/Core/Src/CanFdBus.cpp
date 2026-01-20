#include "CanFdBus.hpp"
#include "stm32l5xx_hal.h"
#include <cstring>

CanFdBus* CanFdBus::isr_instance_ = nullptr;

CanFdBus::CanFdBus(FDCAN_HandleTypeDef& h) : h_(h)
{
    if (!isr_instance_) {
        isr_instance_ = this;
    }
}

bool CanFdBus::start()
{
    if (HAL_FDCAN_Start(&h_) != HAL_OK) return false;

    // Activate notifications for RX FIFO0, RX FIFO1, bus-off, and errors
    if (HAL_FDCAN_ActivateNotification(&h_,
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
        FDCAN_IT_RX_FIFO1_NEW_MESSAGE |
        FDCAN_IT_BUS_OFF |
        FDCAN_IT_ERROR_WARNING |
        FDCAN_IT_ERROR_PASSIVE |
        FDCAN_IT_ERROR_BUS_OFF |
        FDCAN_IT_ARB_PROTOCOL_ERROR |
        FDCAN_IT_DATA_PROTOCOL_ERROR |
        FDCAN_IT_ERROR_LOGGING_OVERFLOW) != HAL_OK) {
        return false;
    }

    state_ = State::Healthy;
    return true;
}

// ---------------- DLC conversion helpers ----------------
uint8_t CanFdBus::dlcToLength(uint8_t dlc, bool fd_frame)
{
    if (!fd_frame) {
        // Classic CAN: DLC 0-8 maps directly to length
        return (dlc > 8) ? 8 : dlc;
    }
    
    // CAN FD: DLC encoding
    // DLC 0-8: maps to 0-8 bytes
    // DLC 9-15: maps to 12, 16, 20, 24, 32, 48, 64 bytes
    static const uint8_t fd_dlc_table[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    if (dlc < 16) {
        return fd_dlc_table[dlc];
    }
    return 64;
}

uint8_t CanFdBus::lengthToDlc(uint8_t len, bool fd_frame)
{
    if (!fd_frame) {
        // Classic CAN: length maps directly to DLC (max 8)
        return (len > 8) ? 8 : len;
    }
    
    // CAN FD: find appropriate DLC
    if (len <= 8) return len;
    if (len <= 12) return 9;
    if (len <= 16) return 10;
    if (len <= 20) return 11;
    if (len <= 24) return 12;
    if (len <= 32) return 13;
    if (len <= 48) return 14;
    return 15; // 64 bytes
}

// ---------------- Filters ----------------
bool CanFdBus::configureFilterAcceptAll(uint32_t filter_index)
{
    FDCAN_FilterTypeDef filter{};
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = filter_index;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0;
    filter.FilterID2 = 0;
    
    return HAL_FDCAN_ConfigFilter(&h_, &filter) == HAL_OK;
}

bool CanFdBus::configureFilterStdMask(uint16_t filter, uint16_t mask,
                                       uint32_t filter_index, uint32_t fifo)
{
    FDCAN_FilterTypeDef f{};
    f.IdType = FDCAN_STANDARD_ID;
    f.FilterIndex = filter_index;
    f.FilterType = FDCAN_FILTER_MASK;
    f.FilterConfig = (fifo == FDCAN_RX_FIFO0) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;
    f.FilterID1 = (filter & 0x7FFu) << 21;  // Standard ID in bits 21-31
    f.FilterID2 = (mask & 0x7FFu) << 21;    // Mask in bits 21-31
    
    return HAL_FDCAN_ConfigFilter(&h_, &f) == HAL_OK;
}

bool CanFdBus::configureFilterExtMask(uint32_t filter, uint32_t mask,
                                       uint32_t filter_index, uint32_t fifo)
{
    FDCAN_FilterTypeDef f{};
    f.IdType = FDCAN_EXTENDED_ID;
    f.FilterIndex = filter_index;
    f.FilterType = FDCAN_FILTER_MASK;
    f.FilterConfig = (fifo == FDCAN_RX_FIFO0) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;
    f.FilterID1 = filter & 0x1FFFFFFFu;  // Extended ID
    f.FilterID2 = mask & 0x1FFFFFFFu;     // Mask
    
    return HAL_FDCAN_ConfigFilter(&h_, &f) == HAL_OK;
}

// ---------------- TX - Classic CAN ----------------
CanFdBus::Result CanFdBus::sendStd(uint16_t id, const uint8_t* payload, uint8_t len, bool rtr)
{
    if (state_ == State::BusOff || state_ == State::Recovering)
        return Result::BusOff;

    if (len > 8) len = 8;

    FDCAN_TxHeaderTypeDef tx_header{};
    tx_header.Identifier = id & 0x7FFu;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = rtr ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8;  // Will be set based on len
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    // Set DLC based on length
    switch (len) {
        case 0: tx_header.DataLength = FDCAN_DLC_BYTES_0; break;
        case 1: tx_header.DataLength = FDCAN_DLC_BYTES_1; break;
        case 2: tx_header.DataLength = FDCAN_DLC_BYTES_2; break;
        case 3: tx_header.DataLength = FDCAN_DLC_BYTES_3; break;
        case 4: tx_header.DataLength = FDCAN_DLC_BYTES_4; break;
        case 5: tx_header.DataLength = FDCAN_DLC_BYTES_5; break;
        case 6: tx_header.DataLength = FDCAN_DLC_BYTES_6; break;
        case 7: tx_header.DataLength = FDCAN_DLC_BYTES_7; break;
        case 8: tx_header.DataLength = FDCAN_DLC_BYTES_8; break;
        default: tx_header.DataLength = FDCAN_DLC_BYTES_8; break;
    }

    HAL_StatusTypeDef st = HAL_FDCAN_AddMessageToTxFifoQ(&h_, &tx_header, const_cast<uint8_t*>(payload));

    if (st == HAL_OK) {
        ++tx_ok_;
        return Result::Ok;
    }

    ++tx_err_;
    if (st == HAL_BUSY) return Result::Busy;
    return Result::Error;
}

CanFdBus::Result CanFdBus::sendStd(uint16_t id,
                                   const std::array<uint8_t,8>& payload,
                                   uint8_t len, bool rtr)
{
    return sendStd(id, payload.data(), len, rtr);
}

CanFdBus::Result CanFdBus::sendExt(uint32_t id, const uint8_t* payload, uint8_t len, bool rtr)
{
    if (state_ != State::Healthy)
        return Result::BusOff;

    if (len > 8) len = 8;

    FDCAN_TxHeaderTypeDef tx_header{};
    tx_header.Identifier = id & 0x1FFFFFFFu;
    tx_header.IdType = FDCAN_EXTENDED_ID;
    tx_header.TxFrameType = rtr ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    // Set DLC based on length
    switch (len) {
        case 0: tx_header.DataLength = FDCAN_DLC_BYTES_0; break;
        case 1: tx_header.DataLength = FDCAN_DLC_BYTES_1; break;
        case 2: tx_header.DataLength = FDCAN_DLC_BYTES_2; break;
        case 3: tx_header.DataLength = FDCAN_DLC_BYTES_3; break;
        case 4: tx_header.DataLength = FDCAN_DLC_BYTES_4; break;
        case 5: tx_header.DataLength = FDCAN_DLC_BYTES_5; break;
        case 6: tx_header.DataLength = FDCAN_DLC_BYTES_6; break;
        case 7: tx_header.DataLength = FDCAN_DLC_BYTES_7; break;
        case 8: tx_header.DataLength = FDCAN_DLC_BYTES_8; break;
        default: tx_header.DataLength = FDCAN_DLC_BYTES_8; break;
    }

    HAL_StatusTypeDef st = HAL_FDCAN_AddMessageToTxFifoQ(&h_, &tx_header, const_cast<uint8_t*>(payload));

    if (st == HAL_OK) {
        ++tx_ok_;
        return Result::Ok;
    }

    ++tx_err_;
    if (st == HAL_BUSY) return Result::Busy;
    return Result::Error;
}

CanFdBus::Result CanFdBus::sendExt(uint32_t id,
                                   const std::array<uint8_t,8>& payload,
                                   uint8_t len, bool rtr)
{
    return sendExt(id, payload.data(), len, rtr);
}

// ---------------- TX - CAN FD ----------------
CanFdBus::Result CanFdBus::sendFdStd(uint16_t id, const uint8_t* payload, uint8_t len, bool brs)
{
    if (state_ == State::BusOff || state_ == State::Recovering)
        return Result::BusOff;

    if (len > 64) {
        len = 64;
    }

    uint8_t dlc = lengthToDlc(len, true);

    FDCAN_TxHeaderTypeDef tx_header{};
    tx_header.Identifier = id & 0x7FFu;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = static_cast<uint32_t>(dlc);
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = brs ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_FD_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    HAL_StatusTypeDef st = HAL_FDCAN_AddMessageToTxFifoQ(&h_, &tx_header, const_cast<uint8_t*>(payload));

    if (st == HAL_OK) {
        ++tx_ok_;
        return Result::Ok;
    }

    ++tx_err_;
    if (st == HAL_BUSY) return Result::Busy;
    return Result::Error;
}

CanFdBus::Result CanFdBus::sendFdExt(uint32_t id, const uint8_t* payload, uint8_t len, bool brs)
{
    if (state_ != State::Healthy)
        return Result::BusOff;

    if (len > 64) {
        len = 64;
    }

    uint8_t dlc = lengthToDlc(len, true);

    FDCAN_TxHeaderTypeDef tx_header{};
    tx_header.Identifier = id & 0x1FFFFFFFu;
    tx_header.IdType = FDCAN_EXTENDED_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = static_cast<uint32_t>(dlc);
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = brs ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_FD_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    HAL_StatusTypeDef st = HAL_FDCAN_AddMessageToTxFifoQ(&h_, &tx_header, const_cast<uint8_t*>(payload));

    if (st == HAL_OK) {
        ++tx_ok_;
        return Result::Ok;
    }

    ++tx_err_;
    if (st == HAL_BUSY) return Result::Busy;
    return Result::Error;
}

// ---------------- RX ----------------
bool CanFdBus::available() const { return rx_head_ != rx_tail_; }

size_t CanFdBus::rxCount() const
{
    int diff = int(rx_head_) - int(rx_tail_);
    if (diff < 0) diff += RX_CAPACITY;
    return size_t(diff);
}

size_t CanFdBus::rxDropped() const { return rx_dropped_; }

bool CanFdBus::rxPushIsr(const CanFdFrame& f)
{
    uint8_t next = idxNext(rx_head_);
    if (next == rx_tail_) {
        ++rx_dropped_;
        return false;
    }
    rx_[rx_head_] = f;
    rx_head_ = next;
    return true;
}

bool CanFdBus::rxPop(CanFdFrame& f)
{
    if (rx_head_ == rx_tail_) return false;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    f = rx_[rx_tail_];
    rx_tail_ = idxNext(rx_tail_);
    if (!primask) __enable_irq();
    return true;
}

bool CanFdBus::read(CanFdFrame& frame) { return rxPop(frame); }

// ---------------- ISR ----------------
void CanFdBus::onRxFifo0Pending()
{
    while (HAL_FDCAN_GetRxFifoFillLevel(&h_, FDCAN_RX_FIFO0) > 0) {
        FDCAN_RxHeaderTypeDef rx_header{};
        uint8_t rx_data[64]{};

        if (HAL_FDCAN_GetRxMessage(&h_, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
            break;

        CanFdFrame f{};
        f.extended = (rx_header.IdType == FDCAN_EXTENDED_ID);
        f.id = rx_header.Identifier;
        f.rtr = (rx_header.RxFrameType == FDCAN_REMOTE_FRAME);
        f.fd_frame = (rx_header.FDFormat == FDCAN_FD_CAN);
        f.brs = (rx_header.BitRateSwitch == FDCAN_BRS_ON);
        f.esi = (rx_header.ErrorStateIndicator == FDCAN_ESI_PASSIVE);
        
        // Convert DLC to actual data length
        uint8_t data_len = dlcToLength(static_cast<uint8_t>(rx_header.DataLength), f.fd_frame);
        f.dlc = static_cast<uint8_t>(rx_header.DataLength);

        // Copy data (up to actual length)
        for (uint8_t i = 0; i < data_len && i < 64; ++i)
            f.data[i] = rx_data[i];

        rxPushIsr(f);
    }
}

void CanFdBus::onRxFifo1Pending()
{
    while (HAL_FDCAN_GetRxFifoFillLevel(&h_, FDCAN_RX_FIFO1) > 0) {
        FDCAN_RxHeaderTypeDef rx_header{};
        uint8_t rx_data[64]{};

        if (HAL_FDCAN_GetRxMessage(&h_, FDCAN_RX_FIFO1, &rx_header, rx_data) != HAL_OK)
            break;

        CanFdFrame f{};
        f.extended = (rx_header.IdType == FDCAN_EXTENDED_ID);
        f.id = rx_header.Identifier;
        f.rtr = (rx_header.RxFrameType == FDCAN_REMOTE_FRAME);
        f.fd_frame = (rx_header.FDFormat == FDCAN_FD_CAN);
        f.brs = (rx_header.BitRateSwitch == FDCAN_BRS_ON);
        f.esi = (rx_header.ErrorStateIndicator == FDCAN_ESI_PASSIVE);
        
        // Convert DLC to actual data length
        uint8_t data_len = dlcToLength(static_cast<uint8_t>(rx_header.DataLength), f.fd_frame);
        f.dlc = static_cast<uint8_t>(rx_header.DataLength);

        // Copy data (up to actual length)
        for (uint8_t i = 0; i < data_len && i < 64; ++i)
            f.data[i] = rx_data[i];

        rxPushIsr(f);
    }
}

void CanFdBus::onBusOff()
{
    state_ = State::BusOff;
    bus_off_latched_ = true;
    recovery_scheduled_ = true;

    if (error_cb_) {
        error_cb_(*this, FDCAN_FLAG_BUS_OFF);
    }
}

void CanFdBus::onError(uint32_t err)
{
    if (error_cb_) {
        error_cb_(*this, err);
    }
}

// ---------------- Recovery ----------------
void CanFdBus::poll()
{
    if (recovery_scheduled_) {
        beginRecovery();
    }

    if (state_ == State::Recovering) {
        checkRecoveryProgress();
    }
}

void CanFdBus::beginRecovery()
{
    recovery_scheduled_ = false;

    HAL_FDCAN_Stop(&h_);
    HAL_FDCAN_DeInit(&h_);
    HAL_FDCAN_Init(&h_);
    HAL_FDCAN_Start(&h_);
    HAL_FDCAN_ActivateNotification(&h_,
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
        FDCAN_IT_RX_FIFO1_NEW_MESSAGE |
        FDCAN_IT_BUS_OFF |
        FDCAN_IT_ERROR_WARNING |
        FDCAN_IT_ERROR_PASSIVE |
        FDCAN_IT_ERROR_BUS_OFF |
        FDCAN_IT_ARB_PROTOCOL_ERROR |
        FDCAN_IT_DATA_PROTOCOL_ERROR |
        FDCAN_IT_ERROR_LOGGING_OVERFLOW);

    tx_ok_at_recovery_ = tx_ok_;
    recovery_start_tick_ = HAL_GetTick();
    state_ = State::Recovering;
}

void CanFdBus::checkRecoveryProgress()
{
    if (tx_ok_ > tx_ok_at_recovery_) {
        state_ = State::Healthy;
        return;
    }

    if (HAL_GetTick() - recovery_start_tick_ > 500) {
        state_ = State::BusOff;
        recovery_scheduled_ = true;
    }
}

// ---------------- Static HAL hooks ----------------
void CanFdBus::handleRxFifo0(FDCAN_HandleTypeDef* hfdcan)
{
    if (isr_instance_ && &isr_instance_->h_ == hfdcan)
        isr_instance_->onRxFifo0Pending();
}

void CanFdBus::handleRxFifo1(FDCAN_HandleTypeDef* hfdcan)
{
    if (isr_instance_ && &isr_instance_->h_ == hfdcan)
        isr_instance_->onRxFifo1Pending();
}

void CanFdBus::handleBusOff(FDCAN_HandleTypeDef* hfdcan)
{
    if (isr_instance_ && &isr_instance_->h_ == hfdcan)
        isr_instance_->onBusOff();
}

void CanFdBus::handleError(FDCAN_HandleTypeDef* hfdcan)
{
    if (isr_instance_ && &isr_instance_->h_ == hfdcan) {
        uint32_t err = HAL_FDCAN_GetError(hfdcan);
        isr_instance_->onError(err);
    }
}

// ---------------- HAL Weak Callback Overrides ----------------
// These override the weak HAL callbacks to route interrupts to CanFdBus

extern "C" {
    /**
     * @brief  Rx FIFO 0 callback.
     * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure.
     * @param  RxFifo0ITs indicates which Rx FIFO 0 interrupts are signalled.
     */
    void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
    {
        CanFdBus::handleRxFifo0(hfdcan);
    }

    /**
     * @brief  Rx FIFO 1 callback.
     * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure.
     * @param  RxFifo1ITs indicates which Rx FIFO 1 interrupts are signalled.
     */
    void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
    {
        CanFdBus::handleRxFifo1(hfdcan);
    }

    /**
     * @brief  Error Status callback.
     * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure.
     * @param  ErrorStatusITs indicates which error status interrupts are signalled.
     */
    void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
    {
        // Check for bus-off condition
        if (ErrorStatusITs & FDCAN_FLAG_BUS_OFF) {
            CanFdBus::handleBusOff(hfdcan);
        } else {
            // Other errors
            CanFdBus::handleError(hfdcan);
        }
    }
}

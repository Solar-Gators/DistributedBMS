#include "CanBus.hpp"
#include "cmsis_os.h"

CanBus* CanBus::isr_instance_ = nullptr;

CanBus::CanBus(CAN_HandleTypeDef& h) : h_(h)
{
    if (!isr_instance_) {
        isr_instance_ = this;
    }
}

bool CanBus::start()
{
    if (HAL_CAN_Start(&h_) != HAL_OK) return false;

    if (HAL_CAN_ActivateNotification(&h_,
        CAN_IT_RX_FIFO0_MSG_PENDING |
        CAN_IT_BUSOFF |
        CAN_IT_ERROR |
        CAN_IT_LAST_ERROR_CODE) != HAL_OK) {
        return false;
    }

    state_ = State::Healthy;
    return true;
}

// ---------------- Filters ----------------
bool CanBus::configureFilterAcceptAll(uint32_t bank)
{
    CAN_FilterTypeDef f{};
    f.FilterBank = bank;
    f.FilterMode = CAN_FILTERMODE_IDMASK;
    f.FilterScale = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh = 0;
    f.FilterIdLow = 0;
    f.FilterMaskIdHigh = 0;
    f.FilterMaskIdLow = 0;
    f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    f.FilterActivation = ENABLE;
#if defined(STM32F4xx) || defined(STM32F7xx) || defined(STM32G4xx) || defined(STM32H7xx)
    f.SlaveStartFilterBank = 14;
#endif
    return HAL_CAN_ConfigFilter(&h_, &f) == HAL_OK;
}

bool CanBus::configureFilterStdMask(uint16_t filter, uint16_t mask,
                                    uint32_t bank, bool into_fifo0)
{
    uint32_t id    = (filter & 0x7FFu) << 5;
    uint32_t idmsk = (mask   & 0x7FFu) << 5;

    CAN_FilterTypeDef f{};
    f.FilterBank = bank;
    f.FilterMode = CAN_FILTERMODE_IDMASK;
    f.FilterScale = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh = id >> 16;
    f.FilterIdLow  = id & 0xFFFFu;
    f.FilterMaskIdHigh = idmsk >> 16;
    f.FilterMaskIdLow  = idmsk & 0xFFFFu;
    f.FilterFIFOAssignment = into_fifo0 ? CAN_FILTER_FIFO0 : CAN_FILTER_FIFO1;
    f.FilterActivation = ENABLE;
#if defined(STM32F4xx) || defined(STM32F7xx) || defined(STM32G4xx) || defined(STM32H7xx)
    f.SlaveStartFilterBank = 14;
#endif
    return HAL_CAN_ConfigFilter(&h_, &f) == HAL_OK;
}

// ---------------- TX ----------------
CanBus::Result CanBus::sendStd(uint16_t id, const uint8_t* payload, uint8_t len, bool rtr)
{
    if (state_ == State::BusOff || state_ == State::Recovering)
        return Result::BusOff;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&h_) == 0)
        return Result::Busy;

    CAN_TxHeaderTypeDef hdr{};
    hdr.IDE = CAN_ID_STD;
    hdr.StdId = id & 0x7FFu;
    hdr.RTR = rtr ? CAN_RTR_REMOTE : CAN_RTR_DATA;
    hdr.DLC = (len > 8) ? 8 : len;

    uint32_t mbx;
    HAL_StatusTypeDef st =
        HAL_CAN_AddTxMessage(&h_, &hdr, const_cast<uint8_t*>(payload), &mbx);

    if (st == HAL_OK) {
        ++tx_ok_;
        return Result::Ok;
    }

    ++tx_err_;
    return Result::Error;
}

CanBus::Result CanBus::sendStd(uint16_t id,
                               const std::array<uint8_t,8>& payload,
                               uint8_t len, bool rtr)
{
    return sendStd(id, payload.data(), len, rtr);
}

CanBus::Result CanBus::sendExt(uint32_t id, const uint8_t* payload, uint8_t len, bool rtr)
{
    if (state_ != State::Healthy)
        return Result::BusOff;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&h_) == 0)
        return Result::NoMailboxes;

    CAN_TxHeaderTypeDef hdr{};
    hdr.IDE = CAN_ID_EXT;
    hdr.ExtId = id & 0x1FFFFFFFu;
    hdr.RTR = rtr ? CAN_RTR_REMOTE : CAN_RTR_DATA;
    hdr.DLC = (len > 8) ? 8 : len;

    uint32_t mbx;
    HAL_StatusTypeDef st =
        HAL_CAN_AddTxMessage(&h_, &hdr, const_cast<uint8_t*>(payload), &mbx);

    if (st == HAL_OK) {
        ++tx_ok_;
        return Result::Ok;
    }

    ++tx_err_;
    return Result::Error;
}

CanBus::Result CanBus::sendExt(uint32_t id,
                               const std::array<uint8_t,8>& payload,
                               uint8_t len, bool rtr)
{
    return sendExt(id, payload.data(), len, rtr);
}

// ---------------- RX ----------------
bool CanBus::available() const { return rx_head_ != rx_tail_; }

size_t CanBus::rxCount() const
{
    int diff = int(rx_head_) - int(rx_tail_);
    if (diff < 0) diff += RX_CAPACITY;
    return size_t(diff);
}

size_t CanBus::rxDropped() const { return rx_dropped_; }

bool CanBus::rxPushIsr(const CanFrame& f)
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

bool CanBus::rxPop(CanFrame& f)
{
    if (rx_head_ == rx_tail_) return false;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    f = rx_[rx_tail_];
    rx_tail_ = idxNext(rx_tail_);
    if (!primask) __enable_irq();
    return true;
}

bool CanBus::read(CanFrame& f) { return rxPop(f); }

// ---------------- ISR ----------------
void CanBus::onRxFifo0Pending()
{
    while (HAL_CAN_GetRxFifoFillLevel(&h_, CAN_RX_FIFO0) > 0) {
        CAN_RxHeaderTypeDef rh{};
        uint8_t buf[8]{};

        if (HAL_CAN_GetRxMessage(&h_, CAN_RX_FIFO0, &rh, buf) != HAL_OK)
            break;

        CanFrame f{};
        f.extended = (rh.IDE == CAN_ID_EXT);
        f.id = f.extended ? rh.ExtId : rh.StdId;
        f.rtr = (rh.RTR == CAN_RTR_REMOTE);
        f.dlc = rh.DLC > 8 ? 8 : rh.DLC;

        for (uint8_t i = 0; i < f.dlc; ++i)
            f.data[i] = buf[i];

        rxPushIsr(f);
    }
}

void CanBus::onBusOff()
{
    state_ = State::BusOff;
    bus_off_latched_ = true;
    recovery_scheduled_ = true;

    if (error_cb_) {
        error_cb_(*this, CAN_ESR_BOFF);
    }
}

void CanBus::onError(uint32_t err)
{
    if (error_cb_) {
        error_cb_(*this, err);
    }
}

// ---------------- Recovery ----------------
void CanBus::poll()
{
    if (recovery_scheduled_) {
        beginRecovery();
    }

    if (state_ == State::Recovering) {
        checkRecoveryProgress();
    }
}

void CanBus::beginRecovery()
{
    recovery_scheduled_ = false;

    HAL_CAN_Stop(&h_);
    HAL_CAN_DeInit(&h_);
    HAL_CAN_Init(&h_);
    HAL_CAN_Start(&h_);
    HAL_CAN_ActivateNotification(&h_,
        CAN_IT_RX_FIFO0_MSG_PENDING |
        CAN_IT_BUSOFF |
        CAN_IT_ERROR |
        CAN_IT_LAST_ERROR_CODE);

    tx_ok_at_recovery_ = tx_ok_;
    recovery_start_tick_ = osKernelGetTickCount();
    state_ = State::Recovering;
}

void CanBus::checkRecoveryProgress()
{
    if (tx_ok_ > tx_ok_at_recovery_) {
        state_ = State::Healthy;
        return;
    }

    if (osKernelGetTickCount() - recovery_start_tick_ > 500) {
        state_ = State::BusOff;
        recovery_scheduled_ = true;
    }
}

// ---------------- Static HAL hooks ----------------
void CanBus::handleRxFifo0(CAN_HandleTypeDef* hcan)
{
    if (isr_instance_ && &isr_instance_->h_ == hcan)
        isr_instance_->onRxFifo0Pending();
}

void CanBus::handleBusOff(CAN_HandleTypeDef* hcan)
{
    if (isr_instance_ && &isr_instance_->h_ == hcan)
        isr_instance_->onBusOff();
}

void CanBus::handleError(CAN_HandleTypeDef* hcan)
{
    if (isr_instance_ && &isr_instance_->h_ == hcan) {
        uint32_t err = HAL_CAN_GetError(hcan);
        isr_instance_->onError(err);
    }
}

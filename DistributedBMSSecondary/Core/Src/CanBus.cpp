#include "CanBus.hpp"
#include <cstddef>

// If your project provides a global umbrella (often pulls core/defines),
// it’s fine to include it; otherwise not required for this TU.
// #include "main.h"

// Static member definition
CanBus* CanBus::isr_instance_ = nullptr;

static_assert((CanBus::RX_CAPACITY & (CanBus::RX_CAPACITY - 1)) == 0,
              "RX_CAPACITY must be a power of two");

CanBus::CanBus(CAN_HandleTypeDef& h) : h_(h) {
    // If you only have one CAN, auto-attach for ISR convenience
    if (!isr_instance_) isr_instance_ = this;
}

bool CanBus::start() {
    if (HAL_CAN_Start(&h_) != HAL_OK) return false;
    if (HAL_CAN_ActivateNotification(&h_, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) return false;
    return true;
}

// Accept-all → FIFO0
bool CanBus::configureFilterAcceptAll(uint32_t bank) {
    CAN_FilterTypeDef f{};
    f.FilterBank = bank;
    f.FilterMode = CAN_FILTERMODE_IDMASK;
    f.FilterScale = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh     = 0x0000;
    f.FilterIdLow      = 0x0000;
    f.FilterMaskIdHigh = 0x0000;
    f.FilterMaskIdLow  = 0x0000;
    f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    f.FilterActivation = ENABLE;
#if defined(STM32F4xx) || defined(STM32F3xx) || defined(STM32F7xx) || defined(STM32G4xx) || defined(STM32H7xx)
    f.SlaveStartFilterBank = 14; // ignored on single-CAN devices; required field on some dual-CAN
#endif
    return HAL_CAN_ConfigFilter(&h_, &f) == HAL_OK;
}

// Standard-ID mask filter helper
bool CanBus::configureFilterStdMask(uint16_t filter, uint16_t mask,
                                    uint32_t bank, bool into_fifo0) {
    // bxCAN packs 11-bit std id into the upper bits of the 32-bit filter.
    const uint32_t id    = (static_cast<uint32_t>(filter) & 0x7FFu) << 5;
    const uint32_t idmsk = (static_cast<uint32_t>(mask)   & 0x7FFu) << 5;

    CAN_FilterTypeDef f{};
    f.FilterBank = bank;
    f.FilterMode = CAN_FILTERMODE_IDMASK;
    f.FilterScale = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh     = static_cast<uint16_t>(id >> 16);
    f.FilterIdLow      = static_cast<uint16_t>(id & 0xFFFFu);
    f.FilterMaskIdHigh = static_cast<uint16_t>(idmsk >> 16);
    f.FilterMaskIdLow  = static_cast<uint16_t>(idmsk & 0xFFFFu);
    f.FilterFIFOAssignment = into_fifo0 ? CAN_FILTER_FIFO0 : CAN_FILTER_FIFO1;
    f.FilterActivation = ENABLE;
#if defined(STM32F4xx) || defined(STM32F3xx) || defined(STM32F7xx) || defined(STM32G4xx) || defined(STM32H7xx)
    f.SlaveStartFilterBank = 14;
#endif
    return HAL_CAN_ConfigFilter(&h_, &f) == HAL_OK;
}

// ---- TX ----
CanBus::Result CanBus::sendStd(uint16_t id, const std::array<uint8_t,8>& p, uint8_t len, bool rtr) {
    return sendStd(id, p.data(), len, rtr);
}

CanBus::Result CanBus::sendStd(uint16_t id, const uint8_t* payload, uint8_t len, bool rtr) {
    if (len > 8) len = 8;

    CAN_TxHeaderTypeDef hdr{};
    hdr.IDE  = CAN_ID_STD;
    hdr.StdId = id & 0x7FFu;
    hdr.RTR  = rtr ? CAN_RTR_REMOTE : CAN_RTR_DATA;
    hdr.DLC  = len;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&h_) == 0) return Result::NoMailboxes;

    uint32_t mbx;
    const auto st = HAL_CAN_AddTxMessage(&h_, &hdr, const_cast<uint8_t*>(payload), &mbx);
    if (st == HAL_OK)   { ++tx_ok_;  return Result::Ok; }
    if (st == HAL_BUSY) { ++tx_err_; return Result::Busy; }
    return Result::Error;
}

CanBus::Result CanBus::sendExt(uint32_t id, const std::array<uint8_t,8>& p, uint8_t len, bool rtr) {
    return sendExt(id, p.data(), len, rtr);
}

CanBus::Result CanBus::sendExt(uint32_t id, const uint8_t* payload, uint8_t len, bool rtr) {
    if (len > 8) len = 8;

    CAN_TxHeaderTypeDef hdr{};
    hdr.IDE  = CAN_ID_EXT;
    hdr.ExtId = id & 0x1FFFFFFFu;
    hdr.RTR  = rtr ? CAN_RTR_REMOTE : CAN_RTR_DATA;
    hdr.DLC  = len;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&h_) == 0) return Result::NoMailboxes;

    uint32_t mbx;
    const auto st = HAL_CAN_AddTxMessage(&h_, &hdr, const_cast<uint8_t*>(payload), &mbx);
    if (st == HAL_OK)   { ++tx_ok_;  return Result::Ok; }
    if (st == HAL_BUSY) { ++tx_err_; return Result::Busy; }
    return Result::Error;
}

// ---- RX ring buffer ----
bool CanBus::available() const { return rx_head_ != rx_tail_; }

size_t CanBus::rx_count() const {
    int16_t diff = int16_t(rx_head_) - int16_t(rx_tail_);
    if (diff < 0) diff += RX_CAPACITY;
    return static_cast<size_t>(diff);
}

size_t CanBus::rx_dropped() const { return rx_dropped_; }

bool CanBus::rx_push_isr(const Frame& f) {
    const uint8_t next = idx_next(rx_head_);
    if (next == rx_tail_) { // overflow
        ++rx_dropped_;
        return false;
    }
    rx_[rx_head_] = f;   // copy
    rx_head_ = next;     // publish
    return true;
}

bool CanBus::rx_pop(Frame& out) {
    if (rx_head_ == rx_tail_) return false;

    // Brief critical section: avoid tearing a multi-byte struct copy
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    out = rx_[rx_tail_];
    rx_tail_ = idx_next(rx_tail_);
    if (!primask) __enable_irq();
    return true;
}

bool CanBus::read(Frame& out) { return rx_pop(out); }

// ---- ISR hook ----
void CanBus::onRxFifo0Pending() {
    while (HAL_CAN_GetRxFifoFillLevel(&h_, CAN_RX_FIFO0) > 0) {
        CAN_RxHeaderTypeDef rh{};
        uint8_t buf[8]{};
        if (HAL_CAN_GetRxMessage(&h_, CAN_RX_FIFO0, &rh, buf) != HAL_OK) break;

        Frame f{};
        if (rh.IDE == CAN_ID_STD) { f.extended = false; f.id = rh.StdId & 0x7FFu; }
        else                      { f.extended = true;  f.id = rh.ExtId & 0x1FFFFFFFu; }

        f.rtr = (rh.RTR == CAN_RTR_REMOTE);
        f.dlc = (rh.DLC <= 8) ? rh.DLC : 8;
        #if defined(CAN_RX_FIFO0) && defined(HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID)
        // Some families expose timestamp if enabled; keep zero otherwise.
        #endif
        for (uint8_t i=0; i<f.dlc; ++i) f.data[i] = buf[i];

        (void)rx_push_isr(f);
    }
}

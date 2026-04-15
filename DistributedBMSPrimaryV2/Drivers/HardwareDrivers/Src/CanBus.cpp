#include "CanBus.hpp"

#include <cstring>

static_assert((CanBus::RX_CAPACITY & (CanBus::RX_CAPACITY - 1)) == 0, "RX_CAPACITY must be a power of two");

CanBus* CanBus::s_instances_[CanBus::kMaxInstances_]{};
uint8_t CanBus::s_instance_count_ = 0;

namespace {

uint8_t dlcCodeToDataBytes(uint32_t dlc_code) {
    // CAN FD DLC decode per ISO 11898-1 mapping:
    // 0..8 => 0..8 bytes, then 9..15 => 12/16/20/24/32/48/64 bytes.
    static const uint8_t kMap[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    if (dlc_code < sizeof(kMap)) {
        return kMap[dlc_code];
    }
    return 0;
}

/** Map payload length to FDCAN DLC register value (0..15). */
uint32_t payloadLenToDlcCode(uint8_t len, bool allow_fd) {
    // Convert requested payload length to the nearest valid DLC encoding.
    // Classic CAN allows up to 8 bytes; CAN FD uses stepped payload sizes above 8.
    if (!allow_fd) {
        if (len > 8) {
            len = 8;
        }
        return len;
    }
    if (len > 64) {
        len = 64;
    }
    if (len <= 8) {
        return len;
    }
    if (len <= 12) {
        return FDCAN_DLC_BYTES_12;
    }
    if (len <= 16) {
        return FDCAN_DLC_BYTES_16;
    }
    if (len <= 20) {
        return FDCAN_DLC_BYTES_20;
    }
    if (len <= 24) {
        return FDCAN_DLC_BYTES_24;
    }
    if (len <= 32) {
        return FDCAN_DLC_BYTES_32;
    }
    if (len <= 48) {
        return FDCAN_DLC_BYTES_48;
    }
    return FDCAN_DLC_BYTES_64;
}

void padToDlc(uint8_t* dst, const uint8_t* src, uint8_t src_len, uint8_t dlc_bytes) {
    // HAL expects exactly DLC-sized payload bytes in TX buffer.
    // Zero-pad any unused bytes to avoid leaking stale stack data on the bus.
    if (src_len > dlc_bytes) {
        src_len = dlc_bytes;
    }
    if (src != nullptr && src_len > 0) {
        std::memcpy(dst, src, src_len);
    }
    if (dlc_bytes > src_len) {
        std::memset(dst + src_len, 0, dlc_bytes - src_len);
    }
}

}  // namespace

CanBus::CanBus(FDCAN_HandleTypeDef& hfdcan) : h_(hfdcan) {
    if (s_instance_count_ < kMaxInstances_) {
        s_instances_[s_instance_count_++] = this;
    }
}

bool CanBus::configureFilterAcceptAll() {
    // Route all standard/extended IDs to RX FIFO0; remote frames are not rejected.
    return HAL_FDCAN_ConfigGlobalFilter(&h_,
                                        FDCAN_ACCEPT_IN_RX_FIFO0,
                                        FDCAN_ACCEPT_IN_RX_FIFO0,
                                        FDCAN_FILTER_REMOTE,
                                        FDCAN_FILTER_REMOTE) == HAL_OK;
}

bool CanBus::configureFilterStdMask(uint16_t filter_id, uint16_t mask, uint32_t filter_index) {
    if (h_.Init.StdFiltersNbr == 0U) {
        return false;
    }
    if (filter_index >= h_.Init.StdFiltersNbr) {
        return false;
    }

    FDCAN_FilterTypeDef f{};
    f.IdType = FDCAN_STANDARD_ID;
    f.FilterIndex = filter_index;
    f.FilterType = FDCAN_FILTER_MASK;
    f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    f.FilterID1 = static_cast<uint32_t>(filter_id) & 0x7FFu;
    f.FilterID2 = static_cast<uint32_t>(mask) & 0x7FFu;
    return HAL_FDCAN_ConfigFilter(&h_, &f) == HAL_OK;
}

bool CanBus::start() {
    // RX FIFO0 new-message interrupt on line 0:
    // 1) route interrupt group to line, 2) start peripheral, 3) enable notification.
    if (HAL_FDCAN_ConfigInterruptLines(&h_, FDCAN_IT_GROUP_RX_FIFO0, FDCAN_INTERRUPT_LINE0) != HAL_OK) {
        return false;
    }
    if (HAL_FDCAN_Start(&h_) != HAL_OK) {
        return false;
    }
    if (HAL_FDCAN_ActivateNotification(&h_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        return false;
    }
    return true;
}

CanBus::Result CanBus::addTx(const FDCAN_TxHeaderTypeDef& hdr, const uint8_t* payload, uint8_t copy_len) {
    const uint8_t dlc_bytes = dlcCodeToDataBytes(hdr.DataLength);
    uint8_t buf[64]{};
    padToDlc(buf, payload, copy_len, dlc_bytes);

    const HAL_StatusTypeDef st = HAL_FDCAN_AddMessageToTxFifoQ(&h_, &hdr, buf);
    if (st == HAL_OK) {
        ++tx_ok_;
        return Result::Ok;
    }
    if (st == HAL_BUSY) {
        // HAL_BUSY commonly means TX FIFO/queue is full at this instant.
        ++tx_err_;
        return Result::Busy;
    }
    return Result::Error;
}

CanBus::Result CanBus::sendStd(uint16_t id, const std::array<uint8_t, 8>& p, uint8_t len, bool rtr) {
    return sendStd(id, p.data(), len, rtr);
}

CanBus::Result CanBus::sendStd(uint16_t id, const uint8_t* payload, uint8_t len, bool rtr) {
    if (len > 8) {
        len = 8;
    }

    FDCAN_TxHeaderTypeDef hdr{};
    hdr.Identifier = static_cast<uint32_t>(id) & 0x7FFu;
    hdr.IdType = FDCAN_STANDARD_ID;
    hdr.TxFrameType = rtr ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
    hdr.DataLength = payloadLenToDlcCode(len, false);
    hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    hdr.BitRateSwitch = FDCAN_BRS_OFF;
    hdr.FDFormat = FDCAN_CLASSIC_CAN;
    hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    hdr.MessageMarker = 0;

    return addTx(hdr, rtr ? nullptr : payload, rtr ? 0 : len);
}

CanBus::Result CanBus::sendExt(uint32_t id, const std::array<uint8_t, 8>& p, uint8_t len, bool rtr) {
    return sendExt(id, p.data(), len, rtr);
}

CanBus::Result CanBus::sendExt(uint32_t id, const uint8_t* payload, uint8_t len, bool rtr) {
    if (len > 8) {
        len = 8;
    }

    FDCAN_TxHeaderTypeDef hdr{};
    hdr.Identifier = id & 0x1FFFFFFFu;
    hdr.IdType = FDCAN_EXTENDED_ID;
    hdr.TxFrameType = rtr ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
    hdr.DataLength = payloadLenToDlcCode(len, false);
    hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    hdr.BitRateSwitch = FDCAN_BRS_OFF;
    hdr.FDFormat = FDCAN_CLASSIC_CAN;
    hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    hdr.MessageMarker = 0;

    return addTx(hdr, rtr ? nullptr : payload, rtr ? 0 : len);
}

CanBus::Result CanBus::sendFdStd(uint16_t id, const uint8_t* payload, uint8_t len, bool bit_rate_switch) {
    if (len > 64) {
        len = 64;
    }

    FDCAN_TxHeaderTypeDef hdr{};
    hdr.Identifier = static_cast<uint32_t>(id) & 0x7FFu;
    hdr.IdType = FDCAN_STANDARD_ID;
    hdr.TxFrameType = FDCAN_DATA_FRAME;
    hdr.DataLength = payloadLenToDlcCode(len, true);
    hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    hdr.BitRateSwitch = bit_rate_switch ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
    hdr.FDFormat = FDCAN_FD_CAN;
    hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    hdr.MessageMarker = 0;

    return addTx(hdr, payload, len);
}

CanBus::Result CanBus::sendFdExt(uint32_t id, const uint8_t* payload, uint8_t len, bool bit_rate_switch) {
    if (len > 64) {
        len = 64;
    }

    FDCAN_TxHeaderTypeDef hdr{};
    hdr.Identifier = id & 0x1FFFFFFFu;
    hdr.IdType = FDCAN_EXTENDED_ID;
    hdr.TxFrameType = FDCAN_DATA_FRAME;
    hdr.DataLength = payloadLenToDlcCode(len, true);
    hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    hdr.BitRateSwitch = bit_rate_switch ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
    hdr.FDFormat = FDCAN_FD_CAN;
    hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    hdr.MessageMarker = 0;

    return addTx(hdr, payload, len);
}

CanBus::Result CanBus::send(const Frame& tx) {
    if (tx.rtr) {
        if (tx.extended) {
            return sendExt(tx.id, tx.data.data(), 0, true);
        }
        return sendStd(static_cast<uint16_t>(tx.id & 0x7FFu), tx.data.data(), 0, true);
    }

    if (tx.fd) {
        if (tx.extended) {
            return sendFdExt(tx.id, tx.data.data(), tx.len, tx.brs);
        }
        return sendFdStd(static_cast<uint16_t>(tx.id & 0x7FFu), tx.data.data(), tx.len, tx.brs);
    }

    if (tx.extended) {
        return sendExt(tx.id, tx.data.data(), tx.len, false);
    }
    return sendStd(static_cast<uint16_t>(tx.id & 0x7FFu), tx.data.data(), tx.len, false);
}

bool CanBus::available() const {
    return rx_head_ != rx_tail_;
}

size_t CanBus::rx_count() const {
    int16_t diff = static_cast<int16_t>(rx_head_) - static_cast<int16_t>(rx_tail_);
    if (diff < 0) {
        diff += RX_CAPACITY;
    }
    return static_cast<size_t>(diff);
}

size_t CanBus::rx_dropped() const {
    return rx_dropped_;
}

bool CanBus::rx_push_isr(const Frame& f) {
    // Single-producer (ISR) ring buffer push.
    const uint8_t next = idx_next(rx_head_);
    if (next == rx_tail_) {
        ++rx_dropped_;
        return false;
    }
    rx_[rx_head_] = f;
    rx_head_ = next;
    return true;
}

bool CanBus::rx_pop(Frame& out) {
    if (rx_head_ == rx_tail_) {
        return false;
    }
    // Protect head/tail update against concurrent ISR writes.
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    out = rx_[rx_tail_];
    rx_tail_ = idx_next(rx_tail_);
    if (!primask) {
        __enable_irq();
    }
    return true;
}

bool CanBus::read(Frame& out) {
    return rx_pop(out);
}

void CanBus::onRxFifo0Pending() {
    // Drain FIFO0 fully for this interrupt to minimize overrun risk.
    while (HAL_FDCAN_GetRxFifoFillLevel(&h_, FDCAN_RX_FIFO0) > 0) {
        FDCAN_RxHeaderTypeDef rh{};
        uint8_t buf[64]{};
        if (HAL_FDCAN_GetRxMessage(&h_, FDCAN_RX_FIFO0, &rh, buf) != HAL_OK) {
            break;
        }

        Frame f{};
        if (rh.IdType == FDCAN_STANDARD_ID) {
            f.extended = false;
            f.id = rh.Identifier & 0x7FFu;
        } else {
            f.extended = true;
            f.id = rh.Identifier & 0x1FFFFFFFu;
        }

        f.rtr = (rh.RxFrameType == FDCAN_REMOTE_FRAME);
        f.fd = (rh.FDFormat == FDCAN_FD_CAN);
        f.brs = (rh.BitRateSwitch == FDCAN_BRS_ON);
        f.timestamp = rh.RxTimestamp;

        const uint8_t nbytes = dlcCodeToDataBytes(rh.DataLength);
        f.len = nbytes;
        // For RTR frames, len may be nonzero in header but data bytes are not meaningful.
        std::memcpy(f.data.data(), buf, nbytes);

        (void)rx_push_isr(f);
    }
}

void CanBus::dispatchRxFifo0FromIsr(FDCAN_HandleTypeDef* hfdcan) {
    // Map HAL callback handle back to owning C++ CanBus instance.
    for (uint8_t i = 0; i < s_instance_count_; ++i) {
        CanBus* inst = s_instances_[i];
        if (inst != nullptr && hfdcan == inst->busHandle()) {
            inst->onRxFifo0Pending();
            return;
        }
    }
}

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs) {
    (void)RxFifo0ITs;
    CanBus::dispatchRxFifo0FromIsr(hfdcan);
}

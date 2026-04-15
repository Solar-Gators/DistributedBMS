#pragma once

#include "stm32g4xx_hal.h"

#include <array>
#include <cstddef>
#include <cstdint>

/**
 * STM32G4 FDCAN peripheral used for normal ISO 11898-1 classic CAN (11/29-bit, DLC ≤ 8).
 * The bus traffic is standard CAN; FDCAN is just the IP that implements it on this MCU.
 *
 * sendStd/sendExt and received frames use FDCAN_CLASSIC_CAN / fd == false. Optional sendFd* /
 * Frame.fd support CAN FD if you ever enable an FD-capable Cube init and data timing.
 *
 * RX: HAL_FDCAN_RxFifo0Callback (CanBus.cpp) dispatches to the matching instance; TX is polling.
 * configureFilterStdMask() needs Init.StdFiltersNbr > 0 in Cube if you use it.
 */
class CanBus {
public:
    enum class Result : uint8_t { Ok, Busy, Error, Timeout, NoMailboxes };

    /** One received/transmitted frame (classic or FD). */
    struct Frame {
        uint32_t id = 0;  // 11-bit if !extended, else 29-bit
        bool extended = false;
        bool rtr = false;
        bool fd = false;  // false = ISO 11898-1 classic; true = CAN FD
        bool brs = false; // bit rate switch in data phase (only for FD)
        uint8_t len = 0;  // bytes stored in data[] (0..64)
        std::array<uint8_t, 64> data{};
        uint32_t timestamp = 0;
    };

    static constexpr uint8_t RX_CAPACITY = 16;

    explicit CanBus(FDCAN_HandleTypeDef& hfdcan);

    /** Call after MX_FDCANx_Init, before start(). Accept non-matching std/ext into FIFO0. */
    bool configureFilterAcceptAll();

    /**
     * Standard-ID mask filter → FIFO0. Requires Init.StdFiltersNbr > 0 in CubeMX.
     * @param filterIndex 0 .. (StdFiltersNbr - 1)
     */
    bool configureFilterStdMask(uint16_t filter_id, uint16_t mask, uint32_t filter_index = 0);

    /** Start peripheral, route RX FIFO0 IRQ line, enable new-message interrupt. */
    bool start();

    // ---- TX: classic (DLC 0..8, not CAN FD) ----
    Result sendStd(uint16_t id, const uint8_t* payload, uint8_t len, bool rtr = false);
    Result sendStd(uint16_t id, const std::array<uint8_t, 8>& p, uint8_t len, bool rtr = false);

    Result sendExt(uint32_t id, const uint8_t* payload, uint8_t len, bool rtr = false);
    Result sendExt(uint32_t id, const std::array<uint8_t, 8>& p, uint8_t len, bool rtr = false);

    // ---- TX: CAN FD data (len up to 64; DLC rounded to FD sizes 12/16/…/64) ----
    Result sendFdStd(uint16_t id, const uint8_t* payload, uint8_t len, bool bit_rate_switch = false);
    Result sendFdExt(uint32_t id, const uint8_t* payload, uint8_t len, bool bit_rate_switch = false);

    /** Encode TX from Frame (classic if !fd, FD if fd). */
    Result send(const Frame& tx);

    bool available() const;
    bool read(Frame& out);
    size_t rx_count() const;
    size_t rx_dropped() const;

    /** Drain hardware RX FIFO0; call from HAL_FDCAN_RxFifo0Callback. */
    void onRxFifo0Pending();

    /** Route HAL RX callback to the instance whose handle matches (multi-FDCAN). */
    static void dispatchRxFifo0FromIsr(FDCAN_HandleTypeDef* hfdcan);

    uint32_t tx_ok() const { return tx_ok_; }
    uint32_t tx_err() const { return tx_err_; }

    FDCAN_HandleTypeDef* busHandle() { return &h_; }
    const FDCAN_HandleTypeDef* busHandle() const { return &h_; }

private:
    FDCAN_HandleTypeDef& h_;

    std::array<Frame, RX_CAPACITY> rx_{};
    volatile uint8_t rx_head_ = 0;
    volatile uint8_t rx_tail_ = 0;
    volatile uint32_t rx_dropped_ = 0;

    volatile uint32_t tx_ok_ = 0;
    volatile uint32_t tx_err_ = 0;

    static constexpr size_t kMaxInstances_ = 3;
    static CanBus* s_instances_[kMaxInstances_];
    static uint8_t s_instance_count_;

    static constexpr uint8_t idx_next(uint8_t i) {
        return static_cast<uint8_t>((i + 1) & (RX_CAPACITY - 1));
    }

    bool rx_push_isr(const Frame& f);
    bool rx_pop(Frame& out);

    Result addTx(const FDCAN_TxHeaderTypeDef& hdr, const uint8_t* payload, uint8_t copy_len);
};

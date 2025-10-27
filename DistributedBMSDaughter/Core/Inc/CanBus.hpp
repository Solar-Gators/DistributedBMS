#pragma once
#include <cstdint>
#include <cstddef>
#include <array>

#include "main.h"
#include "stm32l4xx_hal_can.h"


class CanBus {
public:
    // ---- Types ----
    enum class Result : uint8_t { Ok, Busy, Error, Timeout, NoMailboxes };

    struct Frame {
        uint32_t id = 0;       // 11-bit if extended==false, 29-bit if true
        bool     extended = false;
        bool     rtr = false;  // remote transmission request
        uint8_t  dlc = 0;      // 0..8
        uint8_t  data[8]{};
        uint32_t timestamp = 0; // available on some families if enabled
    };

    // Must be a power of two for the simple index mask logic (16, 32, 64…).
    static constexpr uint8_t RX_CAPACITY = 16;

    // ---- Lifecycle ----
    explicit CanBus(CAN_HandleTypeDef& h);
    bool start(); // starts CAN + enables FIFO0 RX interrupt

    // ---- Filters ----
    // Accept all (std & ext) → FIFO0
    bool configureFilterAcceptAll(uint32_t bank = 0);
    // Standard ID mask filter: passes (ID & mask) == (filter & mask)
    bool configureFilterStdMask(uint16_t filter, uint16_t mask,
                                uint32_t bank = 0, bool into_fifo0 = true);

    // ---- TX ----
    Result sendStd(uint16_t id, const uint8_t* payload, uint8_t len, bool rtr = false);
    Result sendStd(uint16_t id, const std::array<uint8_t,8>& p, uint8_t len, bool rtr = false);
    Result sendExt(uint32_t id, const uint8_t* payload, uint8_t len, bool rtr = false);
    Result sendExt(uint32_t id, const std::array<uint8_t,8>& p, uint8_t len, bool rtr = false);

    // ---- RX (non-blocking) ----
    bool   available() const;   // any frame queued?
    bool   read(Frame& out);    // pop one frame if present
    size_t rx_count() const;    // frames currently queued
    size_t rx_dropped() const;  // overflow counter

    // ---- ISR hook ----
    // Call from HAL_CAN_RxFifo0MsgPendingCallback
    void onRxFifo0Pending();

    // Optional counters
    uint32_t tx_ok()  const { return tx_ok_; }
    uint32_t tx_err() const { return tx_err_; }

    // Default-instance to simplify ISR forwarding
    static void     attach_isr_instance(CanBus* inst) { isr_instance_ = inst; }
    static CanBus*  isr_instance() { return isr_instance_; }

private:
    CAN_HandleTypeDef& h_;

    std::array<Frame, RX_CAPACITY> rx_{};
    volatile uint8_t  rx_head_ = 0;   // ISR pushes
    volatile uint8_t  rx_tail_ = 0;   // main pops
    volatile uint32_t rx_dropped_ = 0;

    volatile uint32_t tx_ok_ = 0;
    volatile uint32_t tx_err_ = 0;

    static constexpr uint8_t idx_next(uint8_t i) { return uint8_t((i + 1) & (RX_CAPACITY - 1)); }

    bool rx_push_isr(const Frame& f); // returns false if overflow
    bool rx_pop(Frame& out);

    static CanBus* isr_instance_;
};

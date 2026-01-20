#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include "main.h"
#include "stm32l5xx_hal.h"

// ---------------- CAN FD frame ---------------- 
// Supports both classic CAN (up to 8 bytes) and CAN FD (up to 64 bytes)
struct CanFdFrame {
    bool extended = false;
    uint32_t id = 0;
    bool rtr = false;
    bool fd_frame = false;      // true for CAN FD, false for classic CAN
    bool brs = false;           // Bit Rate Switch (CAN FD only)
    bool esi = false;           // Error State Indicator (CAN FD only)
    uint8_t dlc = 0;            // Data Length Code (0-8 for classic, 0-15 for FD)
    std::array<uint8_t, 64> data{};  // Support up to 64 bytes for CAN FD
};

// ---------------- CAN FD bus ---------------- 
class CanFdBus {
public:
    enum class Result {
        Ok,
        Busy,
        Error,
        BusOff,
        NoMailboxes,
        InvalidDlc
    };

    enum class State {
        Init,
        Healthy,
        BusOff,
        Recovering
    };

    explicit CanFdBus(FDCAN_HandleTypeDef& h);

    bool start();
    void poll();   // MUST be called periodically from main loop

    // Filters
    bool configureFilterAcceptAll(uint32_t filter_index = 0);
    bool configureFilterStdMask(uint16_t filter, uint16_t mask,
                                uint32_t filter_index, uint32_t fifo = FDCAN_RX_FIFO0);
    bool configureFilterExtMask(uint32_t filter, uint32_t mask,
                                uint32_t filter_index, uint32_t fifo = FDCAN_RX_FIFO0);

    // Transmit - Classic CAN (up to 8 bytes)
    Result sendStd(uint16_t id, const uint8_t* payload, uint8_t len, bool rtr = false);
    Result sendStd(uint16_t id, const std::array<uint8_t,8>& payload, uint8_t len, bool rtr = false);
    Result sendExt(uint32_t id, const uint8_t* payload, uint8_t len, bool rtr = false);
    Result sendExt(uint32_t id, const std::array<uint8_t,8>& payload, uint8_t len, bool rtr = false);

    // Transmit - CAN FD (up to 64 bytes)
    Result sendFdStd(uint16_t id, const uint8_t* payload, uint8_t len, bool brs = true);
    Result sendFdExt(uint32_t id, const uint8_t* payload, uint8_t len, bool brs = true);

    // Receive
    bool available() const;
    size_t rxCount() const;
    size_t rxDropped() const;
    bool read(CanFdFrame& frame);

    // Status
    State state() const { return state_; }
    bool busOffLatched() const { return bus_off_latched_; }
    void clearBusOffLatched() { bus_off_latched_ = false; }

    uint32_t txOk() const { return tx_ok_; }
    uint32_t txError() const { return tx_err_; }

    // Error callback
    using ErrorCallback = void(*)(CanFdBus&, uint32_t);
    void setErrorCallback(ErrorCallback cb) { error_cb_ = cb; }

    // ISR entry points - call from interrupt handlers
    static void handleRxFifo0(FDCAN_HandleTypeDef* hfdcan);
    static void handleRxFifo1(FDCAN_HandleTypeDef* hfdcan);
    static void handleBusOff(FDCAN_HandleTypeDef* hfdcan);
    static void handleError(FDCAN_HandleTypeDef* hfdcan);

private:
    // ISR handlers
    void onRxFifo0Pending();
    void onRxFifo1Pending();
    void onBusOff();
    void onError(uint32_t err);

    // Recovery helpers
    void beginRecovery();
    void checkRecoveryProgress();

    // RX ring buffer
    static constexpr size_t RX_CAPACITY = 32;
    static_assert((RX_CAPACITY & (RX_CAPACITY - 1)) == 0, "RX_CAPACITY must be power of 2");

    uint8_t idxNext(uint8_t idx) const { return (idx + 1) & (RX_CAPACITY - 1); }
    bool rxPushIsr(const CanFdFrame& f);
    bool rxPop(CanFdFrame& f);

    // Helper to convert DLC to actual data length
    static uint8_t dlcToLength(uint8_t dlc, bool fd_frame);
    static uint8_t lengthToDlc(uint8_t len, bool fd_frame);

private:
    FDCAN_HandleTypeDef& h_;
    static CanFdBus* isr_instance_;

    // RX buffer
    std::array<CanFdFrame, RX_CAPACITY> rx_{};
    uint8_t rx_head_ = 0;
    uint8_t rx_tail_ = 0;
    size_t rx_dropped_ = 0;

    // State & errors
    volatile State state_ = State::Init;
    volatile bool bus_off_latched_ = false;
    volatile bool recovery_scheduled_ = false;

    uint32_t tx_ok_ = 0;
    uint32_t tx_err_ = 0;
    uint32_t tx_ok_at_recovery_ = 0;
    uint32_t recovery_start_tick_ = 0;

    ErrorCallback error_cb_ = nullptr;
};

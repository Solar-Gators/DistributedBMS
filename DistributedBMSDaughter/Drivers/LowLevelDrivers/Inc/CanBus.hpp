#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include "main.h"

// ---------------- CAN frame ----------------
struct CanFrame {
    bool extended = false;
    uint32_t id = 0;
    bool rtr = false;
    uint8_t dlc = 0;
    std::array<uint8_t, 8> data{};
};

// ---------------- CAN bus ----------------
class CanBus {
public:
    enum class Result {
        Ok,
        Busy,
        Error,
        BusOff,
        NoMailboxes
    };

    enum class State {
        Init,
        Healthy,
        BusOff,
        Recovering
    };

    explicit CanBus(CAN_HandleTypeDef& h);

    bool start();
    void poll();   // MUST be called periodically from a task

    // Filters
    bool configureFilterAcceptAll(uint32_t bank = 0);
    bool configureFilterStdMask(uint16_t filter, uint16_t mask,
                                uint32_t bank, bool into_fifo0 = true);

    // Transmit
    Result sendStd(uint16_t id, const uint8_t* payload, uint8_t len, bool rtr = false);
    Result sendStd(uint16_t id, const std::array<uint8_t,8>& payload, uint8_t len, bool rtr = false);
    Result sendExt(uint32_t id, const uint8_t* payload, uint8_t len, bool rtr = false);
    Result sendExt(uint32_t id, const std::array<uint8_t,8>& payload, uint8_t len, bool rtr = false);

    // Receive
    bool available() const;
    size_t rxCount() const;
    size_t rxDropped() const;
    bool read(CanFrame& frame);

    // Status
    State state() const { return state_; }
    bool busOffLatched() const { return bus_off_latched_; }
    void clearBusOffLatched() { bus_off_latched_ = false; }

    uint32_t txOk() const { return tx_ok_; }
    uint32_t txError() const { return tx_err_; }

    // Error callback
    using ErrorCallback = void(*)(CanBus&, uint32_t);
    void setErrorCallback(ErrorCallback cb) { error_cb_ = cb; }

    // ISR entry points
    static void handleRxFifo0(CAN_HandleTypeDef* hcan);
    static void handleBusOff(CAN_HandleTypeDef* hcan);
    static void handleError(CAN_HandleTypeDef* hcan);

private:
    // ISR handlers
    void onRxFifo0Pending();
    void onBusOff();
    void onError(uint32_t err);

    // Recovery helpers
    void beginRecovery();
    void checkRecoveryProgress();

    // RX ring buffer
    static constexpr size_t RX_CAPACITY = 32;
    static_assert((RX_CAPACITY & (RX_CAPACITY - 1)) == 0, "RX_CAPACITY must be power of 2");

    uint8_t idxNext(uint8_t idx) const { return (idx + 1) & (RX_CAPACITY - 1); }
    bool rxPushIsr(const CanFrame& f);
    bool rxPop(CanFrame& f);

private:
    CAN_HandleTypeDef& h_;
    static CanBus* isr_instance_;

    // RX buffer
    std::array<CanFrame, RX_CAPACITY> rx_{};
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

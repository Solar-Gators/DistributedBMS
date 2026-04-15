#pragma once

#include <array>
#include <cstdint>

/** Wire image for BmsCanProtocol encoders (classic or FD payload in data[]). */
struct CanFdFrame {
    bool extended = false;
    uint32_t id = 0;
    bool rtr = false;
    bool fd_frame = false;
    bool brs = false;
    bool esi = false;
    uint8_t dlc = 0;
    std::array<uint8_t, 64> data{};
};

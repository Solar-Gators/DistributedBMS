/*
 * bts71040.hpp
 *
 *  Created on: Oct 30, 2025
 *      Author: samrb
 */

#ifndef INC_BTS71040_HPP_
#define INC_BTS71040_HPP_

#pragma once
#include "stm32l5xx_hal.h"   // replace with your device's HAL header
#include <cstdint>
#include <cstddef>

/**
 * BTS71040-4ESA driver (STM32 HAL, C++)
 *
 * SPI: 8-bit frames, MSB first, CPOL=0, CPHA=1 (sample on falling edge, SO updates on rising).
 * Max SCLK 5 MHz. Responses are returned one frame later.
 *
 * You must configure and init HAL SPI (master) externally.
 */

class Bts71040 {
public:
    struct Pins {
        GPIO_TypeDef* ncs_port;  uint16_t ncs_pin;
        GPIO_TypeDef* in_port[4]; uint16_t in_pin[4];
    };

    // Parsed diagnostics (subset of datasheet’s STDDIAG/WRNDIAG/ERRDIAG)
    struct StdDiag {
        bool TER;    // transmission error
        bool CSV;    // checksum verification failed
        bool LHI;    // limp-home input high
        bool SLP;    // sleep mode monitor
        bool SBM;    // switch bypass monitor (for selected channel in DCR.MUX)
        bool VSMON;  // VS dropped below UV/Thermal prot
        uint8_t raw; // full 6-bit payload (lower 6 bits of STDDIAG frame)
    };

    struct WrnDiag {
        bool WRN[4]; // per-channel warning: OC/OT/delta-T
        uint8_t raw; // lower 4 bits of WRNDIAG
    };

    struct ErrDiag {
        bool ERR[4]; // per-channel latched-off
        uint8_t raw; // lower 4 bits of ERRDIAG
    };

    // Handle lifetime is external; we don't own SPI.
    Bts71040(SPI_HandleTypeDef* hspi, const Pins& pins);

    // Optional small extra delay around CS (nops). 0 by default.
    void setCsDelays(uint16_t setup_nops, uint16_t hold_nops);

    // --- Channel control on INx GPIOs ---
    void setChannel(uint8_t ch, bool on);          // ch = 1..4
    void writeInputsMask(uint8_t mask4);           // bit0..bit3 -> IN1..IN4
    uint8_t readInputsMask() const;                // read back GPIO state

    // --- SPI (8-bit) transfers; returns "current frame RX" and stores as last_rx_ ---
    HAL_StatusTypeDef xfer8(uint8_t tx, uint8_t& rx);
    // Do a dummy transfer to fetch the "previous frame" response without sending a command
    HAL_StatusTypeDef fetchPrev(uint8_t& prev_rx) { return xfer8(0x00, prev_rx); }

    // --- High-level commands (auto-handle one-frame delayed response where relevant) ---
    // OUT register (write channel ON/OFF via SPI register, independent of GPIO IN pins if COL=AND)
    HAL_StatusTypeDef writeOUT(uint8_t out_mask4, uint8_t* prev_resp = nullptr);
    HAL_StatusTypeDef readOUT(uint8_t& out_mask4); // does proper two-frame sequence internally

    // Diagnostics (each returns the response to the *previous* frame; we do a dummy pre-read)
    HAL_StatusTypeDef readSTDDIAG(StdDiag& d);
    HAL_StatusTypeDef readWRNDIAG(WrnDiag& d);
    HAL_StatusTypeDef readERRDIAG(ErrDiag& d);

    // Read Restart Counter Status (RCS) for MUX-selected channel (3-bit value 0..7)
    HAL_StatusTypeDef readRCS(uint8_t& rcs3);

    // DCR: set MUX (OFF/ON mapping) and/or SWR; raw 4-bit
    HAL_StatusTypeDef writeDCR(uint8_t dcr_low4, uint8_t* prev_resp = nullptr);

    // HWCR: CLC (clear all counters/latches), RST, SLP(read-only), COL
    HAL_StatusTypeDef writeHWCR(uint8_t hwcr_low4, uint8_t* prev_resp = nullptr);

    // PCS: per selected MUX channel — SRCS (slew), CLCS (clear), PCCn (parallel config)
    HAL_StatusTypeDef writePCS(uint8_t pcs_low4, bool setSWRBefore = true);

    // OCR/KRC/RCD writes (setSWRBefore controls DCR.SWR per table)
    HAL_StatusTypeDef writeOCR(uint8_t ocr_low4, bool setSWRBefore = false);
    HAL_StatusTypeDef writeKRC(uint8_t krc_low4, bool setSWRBefore = false);
    HAL_StatusTypeDef writeRCD(uint8_t rcd_low4, bool setSWRBefore = true);

    // ICS checksum input (4-bit). Call after configuring registers, then check STDDIAG.CSV.
    HAL_StatusTypeDef writeICS(uint8_t csr_low4, bool setSWRBefore = true);

    // Helpers
    // Compute 4-bit checksum from config fields per datasheet (pass the actual 4-bit fields you set)
    static uint8_t computeChecksum4(
        uint8_t OCR, uint8_t RCD, uint8_t KRC, uint8_t SRC,
        bool COL, uint8_t PCC);

private:
    SPI_HandleTypeDef* hspi_;
    Pins pins_{};
    uint16_t cs_setup_nops_ = 0;
    uint16_t cs_hold_nops_  = 0;
    uint8_t  last_rx_ = 0x00;

    // --- Wire protocol constants (8-bit commands) ---
    // Read commands (0xxxADDR)
    static constexpr uint8_t RD_OUT    = 0x00; // 0xxx0000
    static constexpr uint8_t RD_RCS    = 0x08; // 0xxx1000
    static constexpr uint8_t RD_SRC    = 0x09; // 0xxx1001
    static constexpr uint8_t RD_OCR    = 0x04; // 0xxx0100
    static constexpr uint8_t RD_RCD    = 0x0C; // 0xxx1100
    static constexpr uint8_t RD_KRC    = 0x05; // 0xxx0101
    static constexpr uint8_t RD_PCS    = 0x0D; // 0xxx1101
    static constexpr uint8_t RD_HWCR   = 0x06; // 0xxx0110
    static constexpr uint8_t RD_ICS    = 0x0E; // 0xxx1110
    static constexpr uint8_t RD_DCR    = 0x07; // 0xxxx111
    static constexpr uint8_t RD_WRNDIAG= 0x01; // 0xxxx001
    static constexpr uint8_t RD_STDDIAG= 0x02; // 0xxxx010
    static constexpr uint8_t RD_ERRDIAG= 0x03; // 0xxxx011

    // Write commands (11aa dddd / 10 dddddd patterns)
    static constexpr uint8_t WR_OUT    = 0x80; // 100x dddd (we send 1000 dddd)
    static constexpr uint8_t WR_OCR    = 0xC0; // 1100 dddd (SWR=0)
    static constexpr uint8_t WR_RCD    = 0xC0; // 1100 dddd (SWR=1)
    static constexpr uint8_t WR_KRC    = 0xD0; // 1101 dddd (SWR=0)
    static constexpr uint8_t WR_PCS    = 0xD0; // 1101 dddd (SWR=1)
    static constexpr uint8_t WR_HWCR   = 0xE0; // 1110 dddd (SWR=0)
    static constexpr uint8_t WR_ICS    = 0xE0; // 1110 dddd (SWR=1)
    static constexpr uint8_t WR_DCR    = 0xF0; // 1111 dddd (SWR=x)

    // DCR bit helpers
    static constexpr uint8_t DCR_SWR_MASK = 0x08; // bit3
    static constexpr uint8_t DCR_MUX_MASK = 0x07; // bits2:0

    // CS helpers
    inline void csLow();
    inline void csHigh();

    // Internal: perform one 8-bit TX/RX with CS handling
    HAL_StatusTypeDef transfer8(uint8_t tx, uint8_t& rx);

    // Convenience to set DCR.SWR bit without changing MUX
    HAL_StatusTypeDef setSWR(bool swr, uint8_t* prev_resp = nullptr);
};




#endif /* INC_BTS71040_HPP_ */

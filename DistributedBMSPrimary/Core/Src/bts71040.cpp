/*
 * bts71040.cpp
 *
 *  Created on: Oct 30, 2025
 *      Author: samrb
 */

#ifndef SRC_BTS71040_CPP_
#define SRC_BTS71040_CPP_


#include "bts71040.hpp"

static inline void spin_nops(uint16_t n) { for (uint16_t i=0;i<n;++i) __NOP(); }

Bts71040::Bts71040(SPI_HandleTypeDef* hspi, const Pins& pins)
: hspi_(hspi), pins_(pins) {
    // Ensure CS high
    HAL_GPIO_WritePin(pins_.ncs_port, pins_.ncs_pin, GPIO_PIN_SET);
}

void Bts71040::setCsDelays(uint16_t setup_nops, uint16_t hold_nops) {
    cs_setup_nops_ = setup_nops;
    cs_hold_nops_  = hold_nops;
}

inline void Bts71040::csLow()  {
    HAL_GPIO_WritePin(pins_.ncs_port, pins_.ncs_pin, GPIO_PIN_RESET);
    spin_nops(cs_setup_nops_);
}
inline void Bts71040::csHigh() {
    spin_nops(cs_hold_nops_);
    HAL_GPIO_WritePin(pins_.ncs_port, pins_.ncs_pin, GPIO_PIN_SET);
}

/* ---------------- GPIO INx helpers ---------------- */

void Bts71040::setChannel(uint8_t ch, bool on) {
    if (ch < 1 || ch > 4) return;
    HAL_GPIO_WritePin(pins_.in_port[ch-1], pins_.in_pin[ch-1], on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Bts71040::writeInputsMask(uint8_t mask4) {
    for (uint8_t i=0;i<4;++i) {
        GPIO_PinState s = (mask4 & (1u<<i)) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        HAL_GPIO_WritePin(pins_.in_port[i], pins_.in_pin[i], s);
    }
}

uint8_t Bts71040::readInputsMask() const {
    uint8_t m=0;
    for (uint8_t i=0;i<4;++i) {
        if (HAL_GPIO_ReadPin(pins_.in_port[i], pins_.in_pin[i]) == GPIO_PIN_SET)
            m |= (1u<<i);
    }
    return m;
}

/* ---------------- SPI core ---------------- */

HAL_StatusTypeDef Bts71040::transfer8(uint8_t tx, uint8_t& rx) {
    csLow();
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(hspi_, &tx, &rx, 1, 2);
    csHigh();
    last_rx_ = rx;
    return st;
}

HAL_StatusTypeDef Bts71040::xfer8(uint8_t tx, uint8_t& rx) {
    return transfer8(tx, rx);
}

/* ---------------- High-level commands ---------------- */

// OUT write: sends 1000dddd (we keep the 'x' bit at 0)
HAL_StatusTypeDef Bts71040::writeOUT(uint8_t out_mask4, uint8_t* prev_resp) {
    uint8_t rx=0;
    HAL_StatusTypeDef st = transfer8(static_cast<uint8_t>(WR_OUT | (out_mask4 & 0x0F)), rx);
    if (st == HAL_OK && prev_resp) *prev_resp = rx; // this is the response to the previous frame
    return st;
}

// OUT read: we must (1) send read command, (2) send dummy to fetch response
HAL_StatusTypeDef Bts71040::readOUT(uint8_t& out_mask4) {
    uint8_t rx=0;
    // Step 1: trigger read
    HAL_StatusTypeDef st = transfer8(RD_OUT, rx); // rx = response to previous frame, ignore
    if (st != HAL_OK) return st;
    // Step 2: fetch response
    st = transfer8(0x00, rx);
    if (st != HAL_OK) return st;
    // Response format (Table 32): OUT reply has high bits 1 0 0 x, lower nibble = OUT[3:0]
    out_mask4 = (rx & 0x0F);
    return HAL_OK;
}

// STDDIAG: send read cmd then dummy to get it; parse bits
HAL_StatusTypeDef Bts71040::readSTDDIAG(StdDiag& d) {
    uint8_t r=0;
    HAL_StatusTypeDef st = transfer8(RD_STDDIAG, r); // prime
    if (st != HAL_OK) return st;
    st = transfer8(0x00, r); // fetch
    if (st != HAL_OK) return st;

    // STDDIAG frame: 00 d d d d d d  (lower 6 bits meaningful)
    uint8_t payload = r & 0x3F;
    d.raw  = payload;
    d.TER  = (payload >> 5) & 0x1;
    d.CSV  = (payload >> 4) & 0x1;
    d.LHI  = (payload >> 3) & 0x1;
    d.SLP  = (payload >> 2) & 0x1;
    d.SBM  = (payload >> 1) & 0x1;
    d.VSMON= (payload >> 0) & 0x1;
    return HAL_OK;
}

HAL_StatusTypeDef Bts71040::readWRNDIAG(WrnDiag& d) {
    uint8_t r=0;
    HAL_StatusTypeDef st = transfer8(RD_WRNDIAG, r);
    if (st != HAL_OK) return st;
    st = transfer8(0x00, r);
    if (st != HAL_OK) return st;
    uint8_t payload = r & 0x0F;
    d.raw = payload;
    for (int i=0;i<4;++i) d.WRN[i] = (payload >> i) & 0x1;
    return HAL_OK;
}

HAL_StatusTypeDef Bts71040::readERRDIAG(ErrDiag& d) {
    uint8_t r=0;
    HAL_StatusTypeDef st = transfer8(RD_ERRDIAG, r);
    if (st != HAL_OK) return st;
    st = transfer8(0x00, r);
    if (st != HAL_OK) return st;
    uint8_t payload = r & 0x0F;
    d.raw = payload;
    for (int i=0;i<4;++i) d.ERR[i] = (payload >> i) & 0x1;
    return HAL_OK;
}

HAL_StatusTypeDef Bts71040::readRCS(uint8_t& rcs3) {
    uint8_t r=0;
    HAL_StatusTypeDef st = transfer8(RD_RCS, r);
    if (st != HAL_OK) return st;
    st = transfer8(0x00, r);
    if (st != HAL_OK) return st;
    rcs3 = (r & 0x07);
    return HAL_OK;
}

/* ---- Config writes ---- */

HAL_StatusTypeDef Bts71040::writeDCR(uint8_t dcr_low4, uint8_t* prev_resp) {
    uint8_t rx=0;
    HAL_StatusTypeDef st = transfer8(static_cast<uint8_t>(WR_DCR | (dcr_low4 & 0x0F)), rx);
    if (st == HAL_OK && prev_resp) *prev_resp = rx;
    return st;
}

HAL_StatusTypeDef Bts71040::setSWR(bool swr, uint8_t* prev_resp) {
    // Read current DCR is possible but not strictly required; we set low4 with SWR desired and preserve MUX=000 (safe)
    // Better approach: the caller tracks desired MUX; here we do minimal effect unless caller provides it explicitly.
    uint8_t cur_resp=0;
    // Try to read DCR to preserve MUX if SPI link is up
    (void)transfer8(RD_DCR, cur_resp);
    (void)transfer8(0x00, cur_resp);
    uint8_t low4 = cur_resp & 0x0F;        // DCR.SWR (bit3) and DCR.MUX (2:0)
    if (swr) low4 |= DCR_SWR_MASK; else low4 &= ~DCR_SWR_MASK;
    return writeDCR(low4, prev_resp);
}

HAL_StatusTypeDef Bts71040::writeHWCR(uint8_t hwcr_low4, uint8_t* prev_resp) {
    uint8_t rx=0;
    HAL_StatusTypeDef st = transfer8(static_cast<uint8_t>(WR_HWCR | (hwcr_low4 & 0x0F)), rx);
    if (st == HAL_OK && prev_resp) *prev_resp = rx;
    return st;
}

HAL_StatusTypeDef Bts71040::writePCS(uint8_t pcs_low4, bool setSWRBefore) {
    if (setSWRBefore) {
        HAL_StatusTypeDef st = setSWR(true, nullptr);
        if (st != HAL_OK) return st;
    }
    uint8_t rx=0;
    return transfer8(static_cast<uint8_t>(WR_PCS | (pcs_low4 & 0x0F)), rx);
}

HAL_StatusTypeDef Bts71040::writeOCR(uint8_t ocr_low4, bool setSWRBefore) {
    if (setSWRBefore) {
        HAL_StatusTypeDef st = setSWR(false, nullptr);
        if (st != HAL_OK) return st;
    }
    uint8_t rx=0;
    return transfer8(static_cast<uint8_t>(WR_OCR | (ocr_low4 & 0x0F)), rx);
}

HAL_StatusTypeDef Bts71040::writeKRC(uint8_t krc_low4, bool setSWRBefore) {
    if (setSWRBefore) {
        HAL_StatusTypeDef st = setSWR(false, nullptr);
        if (st != HAL_OK) return st;
    }
    uint8_t rx=0;
    return transfer8(static_cast<uint8_t>(WR_KRC | (krc_low4 & 0x0F)), rx);
}

HAL_StatusTypeDef Bts71040::writeRCD(uint8_t rcd_low4, bool setSWRBefore) {
    if (setSWRBefore) {
        HAL_StatusTypeDef st = setSWR(true, nullptr);
        if (st != HAL_OK) return st;
    }
    uint8_t rx=0;
    return transfer8(static_cast<uint8_t>(WR_RCD | (rcd_low4 & 0x0F)), rx);
}

HAL_StatusTypeDef Bts71040::writeICS(uint8_t csr_low4, bool setSWRBefore) {
    if (setSWRBefore) {
        HAL_StatusTypeDef st = setSWR(true, nullptr); // ICS requires SWR=1 bank
        if (st != HAL_OK) return st;
    }
    uint8_t rx=0;
    return transfer8(static_cast<uint8_t>(WR_ICS | (csr_low4 & 0x0F)), rx);
}

/* ---------------- Checksum calculation helper ----------------
   Parity over columns:
   Columns OCR,RCD,KRC,SRC,HWCR/PCS -> parity (even/odd/even/odd) -> CSR[3:0].
   Pass in the 4-bit fields you programmed (e.g., OCTn range, RCDn latch/auto, etc.).
*/
uint8_t Bts71040::computeChecksum4(
    uint8_t OCR, uint8_t RCD, uint8_t KRC, uint8_t SRC,
    bool COL, uint8_t PCC)
{
    // Build the 5x4 matrix columns (bit3..bit0) and compute parity
    auto parity_even = [](uint8_t col)->uint8_t {
        // even parity: result 0 if #ones even, 1 if odd
        uint8_t ones = __builtin_popcount(col & 0x1F); // we’ll pass 5 rows packed in LSBs
        return (ones & 1u) ? 1u : 0u;
    };
    auto parity_odd = [](uint8_t col)->uint8_t {
        // odd parity: inverse of even parity
        uint8_t ones = __builtin_popcount(col & 0x1F);
        return (ones & 1u) ? 0u : 1u;
    };

    // Compose each column from rows: OCR, RCD, KRC, SRC, HWCR/PCS
    // For HWCR/PCS we use: bit2=COL, bits3:2 = PCC1:PCC0; pack into same 4-bit per-column scheme.
    // Extract column k = bits[k] of each 4-bit word; pack as row-bits in positions 0..4
    auto make_col = [&](int bit)->uint8_t {
        uint8_t c = 0;
        c |= ((OCR >> bit) & 1u) << 0;
        c |= ((RCD >> bit) & 1u) << 1;
        c |= ((KRC >> bit) & 1u) << 2;
        c |= ((SRC >> bit) & 1u) << 3;

        // HWCR/PCS row: columns mapping per datasheet is (0, COL, PCC1, PCC0) across the four columns.
        // We map bit index -> which HWCR/PCS data enters that column:
        // bit3 column gets value: 0
        // bit2 column gets value: COL
        // bit1 column gets value: PCC1
        // bit0 column gets value: PCC0
        uint8_t hwpcs_row = 0;
        switch (bit) {
            case 3: hwpcs_row = 0;                      break;
            case 2: hwpcs_row = COL ? 1 : 0;            break;
            case 1: hwpcs_row = (PCC >> 1) & 1u;        break;
            case 0: hwpcs_row = (PCC >> 0) & 1u;        break;
            default: hwpcs_row = 0;                     break;
        }
        c |= (hwpcs_row & 1u) << 4;
        return c; // 5 rows packed in bits 0..4
    };

    uint8_t col3 = make_col(3);
    uint8_t col2 = make_col(2);
    uint8_t col1 = make_col(1);
    uint8_t col0 = make_col(0);

    // Parity rule: col3 even, col2 odd, col1 even, col0 odd -> CSR3..CSR0
    uint8_t CSR3 = parity_even(col3);
    uint8_t CSR2 = parity_odd (col2);
    uint8_t CSR1 = parity_even(col1);
    uint8_t CSR0 = parity_odd (col0);

    return static_cast<uint8_t>((CSR3<<3)|(CSR2<<2)|(CSR1<<1)|(CSR0<<0));
}



#endif /* SRC_BTS71040_CPP_ */

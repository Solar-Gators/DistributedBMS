# BQ76925PWR Driver — How It Works

This document reviews the BQ76925 driver and the voltage-measurement path so everything stays accurate relative to the [BQ76925 datasheet](https://www.ti.com/lit/ds/symlink/bq76925.pdf) (Rev. E, SLUSAM9E).

---

## 1. Device role

- The **BQ76925** is an analog front-end (AFE) only: it has **no internal ADC**.
- It exposes **analog outputs** (VCOUT, VIOUT, VREF, VTB) for the host MCU to digitize.
- The host talks to the AFE over **I²C** only for **configuration and status** (registers), not for digitized cell data.

So the driver does **I²C register read/write**; **cell voltages** are obtained by:
1. Selecting a cell on VCOUT via I²C (CELL_CTL),
2. Letting VCOUT settle,
3. Reading VCOUT with the **STM32 ADC** (e.g. channel 10, rank 6),
4. Converting ADC counts → VCOUT voltage → cell voltage in application code.

---

## 2. I²C addressing (section 8.5.1.1)

The BQ76925 uses a **combined 7‑bit address**:

- **Formula:** `ADDRESS[6:0] = (I2C_GROUP_ADDR[3:0] << 3) + REG_ADDR[4:0]`
- **I2C_GROUP_ADDR** = factory default `0b0100` (stored in EEPROM).
- **REG_ADDR** = 5‑bit register address (0x00–0x1F).

So each register has its own I²C address; there is no separate “device address + register subaddress” phase.

**In the driver:**

- `readReg(reg, buf, len)` and `writeReg(reg, buf, len)` compute:
  - `reg_addr = reg & 0x1F`
  - `addr7 = (I2C_GROUP_ADDR << 3) | reg_addr`  →  e.g. 0x20 for STATUS (0x00), 0x21 for CELL_CTL (0x01), etc.
- The HAL is called with **8‑bit address** `addr8 = addr7 << 1` (standard I²C 7‑bit addr shifted for the API in use).

This matches the datasheet; no Mem_Read/Mem_Write with a separate register byte.

---

## 3. Register map (section 8.6)

Driver enum vs datasheet:

| Register   | Address | Use in driver |
|-----------|---------|----------------|
| STATUS    | 0x00    | Read after init to confirm comms / clear POR |
| CELL_CTL  | 0x01    | Select which cell (or VREF fraction) is on VCOUT |
| BAL_CTL   | 0x02    | Reserved for future balancing |
| CONFIG_1  | 0x03    | Reserved (current gain, comparator, etc.) |
| CONFIG_2  | 0x04    | REF_SEL (0 = 1.5 V / gain 0.3, 1 = 3.0 V / gain 0.6) |
| POWER_CTL | 0x05    | REF_EN, VC_AMP_EN, etc. |
| CHIP_ID   | 0x07    | Read-only; optional for ID check |

All addresses and usage are consistent with section 8.6.

---

## 4. init() — power-up and configuration

**Order and meaning:**

1. **POWER_CTL** (Table 15):
   - `REF_EN` (D0) = 1  →  enable 1.5/3.0 V reference.
   - `VC_AMP_EN` (D2) = 1  →  enable cell voltage amplifier.
   - Other bits (VTB_EN, I_AMP_EN, I_COMP_EN, SLEEP, etc.) left 0.

2. **CONFIG_2** (Table 13):
   - `REF_SEL` (D0) = 1  →  3.0 V reference and **VCOUT gain = 0.6** (Table 14).
   - CRC_EN (D7) = 0  →  no CRC on writes.

3. **STATUS** (0x00) read:
   - Confirms I²C communication.
   - Reading STATUS can be used to clear or observe POR/ALERT/CRC_ERR as needed.

So after init, VCOUT = 0.6 × selected cell voltage (and VREF = 3.0 V nominal). This matches the datasheet.

---

## 5. setCellForVCOUT(cellIndex) — CELL_CTL

**Datasheet:**

- **CELL_CTL** (Table 6): D7 = 0, D6 unused, **D5:D4 = VCOUT_SEL**, **D3:D1 = CELL_SEL**, D0 unused.
- **VCOUT_SEL** (Table 7):  
  `00` = VSS, `01` = **VCn** (n from CELL_SEL), `10` = 0.5×VREF, `11` = 0.85×VREF.
- **CELL_SEL** (Table 8):  
  `000` = VC1 … `101` = VC6, `110` = V_TEMP_INT, `111` = Hi‑Z.

**Driver:**

- `cellIndex` 0..5 → VC1..VC6.
- `cell_sel = cellIndex & 0x07`  →  values 0..5, valid for VC1..VC6.
- `cell_ctl = (1u << 4) | (cell_sel << 1)`:
  - D4 = 1, D5 = 0  →  **VCOUT_SEL = 01**  →  VCOUT = VCn.
  - D3:D1 = CELL_SEL  →  correct mapping to VC1..VC6.

So the bit layout and mapping are correct.

---

## 6. Application side — User.cpp and ADC

**Flow:**

1. For each cell index 0..5:
   - `bq.setCellForVCOUT(cell)`  →  writes CELL_CTL so VCOUT = VC(cell+1).
   - Start ADC DMA for the full scan (7 channels).
   - Wait for DMA complete.
   - Take **adc_buf[5]** as the sample for **VCOUT** (rank 6 = channel 10 in `main.cpp`).

2. Convert to cell voltage:
   - `v_vcout = (adc_buf[5] / 4095.0f) * 3.3f`  (assuming 3.3 V ADC reference).
   - `v_cell = v_vcout / 0.6f`  (VCOUT gain with REF_SEL = 1).
   - `cell_mV[cell] = (uint16_t)(v_cell * 1000 + 0.5)`.

**Accuracy notes:**

- **0.6** is the nominal VCOUT gain from the datasheet when REF_SEL = 1; for higher accuracy you can later add the gain/offset correction from the EEPROM calibration registers (VCn_CAL, VREF_CAL, etc.) per section 8.3.2.2.
- **3.3 V** is the MCU’s ADC reference (VREF+). The AFE’s internal 3.0 V reference is only used inside the BQ76925; it does *not* have to match the MCU. Always use the MCU’s actual VREF+ (e.g. 3.3 V) in the conversion. If you later wire the BQ76925 VREF pin to the STM32 VREF+, then change the constant to 3.0 V.

---

## 7. Summary checklist

| Item | Status |
|------|--------|
| I²C combined address (8.5.1.1) | Correct: (GROUP<<3)+REG, 8‑bit addr = addr7<<1 |
| Register addresses 0x00–0x07 | Match section 8.6 |
| POWER_CTL REF_EN, VC_AMP_EN | Set in init() |
| CONFIG_2 REF_SEL = 1 (3.0 V, gain 0.6) | Set in init() |
| CELL_CTL VCOUT_SEL = 01, CELL_SEL = VC1..VC6 | Correct in setCellForVCOUT() |
| Cell count (6) | BQ76925_CELL_COUNT and CELLS/CELL_COUNT_CONF aligned |
| VCOUT → ADC index | adc_buf[5] = rank 6 = channel 10 |
| VCOUT → cell formula | v_cell = v_vcout / 0.6 with REF_SEL=1 |

The driver and application flow are consistent with the BQ76925 datasheet. Optional improvements: use VREF_CAL/VCn_CAL for calibrated gain/offset, and set the ADC reference constant to the actual hardware reference (e.g. 3.0 V from AFE if used).

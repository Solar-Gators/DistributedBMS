# ASC Battery Protection System — Preliminary Test Sheet

**Team:** University of Florida Solar Gators (Team #5)  
**Event:** ASC/FSGP 2026  
**BPS:** Custom Distributed BMS (PrimaryV2 + 6 Daughter boards)  
**Cell:** Molicel P50B (Li-Ion)  
**Pack:** 36S8P, 288 cells (per FSGP BPS Review)  
**Firmware baseline:** `DistributedBMSPrimaryV2` + `DistributedBMSDaughter`  
**Document status:** Preliminary — pack current = DHAB S/134 + 1 turn (race); FSGP OC trip 40 A  

**References:**
- FSGP2026 BPS Review (Florida, GREEN, 2026-06-05)
- ASC2019 Battery Protection System Test Procedure

---

## System Summary (for inspector)

| Item | Value |
|------|-------|
| Protection MCU | PrimaryV2 (STM32G4) |
| Cell monitoring | 6× Daughter boards, BQ76907, 6 cells each (36 cell channels total) |
| Daughter CAN | FDCAN2, IDs `0x100`–`0x105` |
| Vehicle CAN | FDCAN3, status `0x040`–`0x045`, command `0x1A0` |
| Pack current sense | **LEM DHAB S/134** → ADS1115 **AIN0=CH1 low**, **AIN1=CH2 high**, **1 turn (race)**, V0=2.5 V, PGA ±4.096 V |
| Pack OC trip (discharge) | FSGP manufacturer **480 A**; BPS trip **40 A** abs (CH2 FS ±200 A; 480 A not measurable) |
| Pack OC trip (charge) | FSGP **40 A** (5 A×8P); BMS often off when charging — charge test N/A |
| Bench OC method | N wraps: I_eff = I_supply × N; set `current_sensor_turns` to match wraps for that bench |
| Isolation | Dual main contactors (IN0/IN1 + BTS71040), active-high closed |
| Contactor stagger | 500 ms between first and second close |
| Fault evaluation period | 10 ms (`BMS_MANAGER_PERIOD_MS`) |
| Max contactor open latency (contract) | 500 ms |
| Fault recovery to IDLE | 5 s after all faults clear |
| Charging | BMS not powered during pack charge (charge OC test N/A) |

---

## Target Protection Setpoints (FSGP-aligned)

These are the values we intend to align in firmware before scrutineering.

| Parameter | Molicel / FSGP | Current code | **Target firmware** |
|-----------|----------------|--------------|---------------------|
| Cell OV trip | 4.20 V | 4.22 V | **4200 mV** |
| Cell UV trip | 2.50 V | 2.50 V | **2500 mV** |
| Cell imbalance | — | 410 mV | **410 mV** |
| OT trip (charge) | 60 °C | 100 °C | **60 °C** |
| OT trip (discharge) | 60 °C | 100 °C | **60 °C** |
| Pack OC trip (discharge) | 480 A | — | **40 A** abs (FSGP charge; 480 A beyond sensor) |
| Pack OC trip (charge) | 40 A | — | **40 A** abs (N/A if BMS off) |
| Undertemp trip | — | −10 °C | **−10 °C** (enabled) |
| Aux OC trip | — | 50 A | **Disabled** (not a BPS pack fault) |

### Hysteresis (current defaults, unless changed)

| Fault | Trip | Clear |
|-------|------|-------|
| Overvoltage | 4200 mV | 4150 mV |
| Undervoltage | 2500 mV | 2550 mV |
| Overtemperature | 60 °C | 58 °C |
| Pack overcurrent | 40 A | 39 A |

**Note:** Race = **1 turn**. FSGP charge OC **40 A** is the firmware abs trip. Discharge manufacturer **480 A** exceeds DHAB CH2 (±200 A) — document for inspector.

---

## Pack Current Analog Chain (DHAB S/134 → ADS1115)

**Hardware:**
- Sensor: LEM **DHAB S/134** (open-loop Hall), **1 turn** (race); N wraps for ASC bench only
- ADS1115: ADDR=GND (`0x48`), PGA **±4.096 V**, DR **128 SPS**, single-shot
- **AIN0** = CH1 low (±50 A @ 40 mV/A)
- **AIN1** = CH2 high (±200 A @ 10 mV/A)
- V0 = **2.5 V** at UC = 5 V

**Formula in code:**

```text
I_pack_A = (V_adc - 2.5) / (G_V_per_A × N_turns)
         = (V_adc - 2.5) / G     (race: N_turns = 1)
```

Firmware prefers AIN0 when `|I_high| < 2.5 A`, else AIN1.

| Pack I (1 turn) | CH2 Vadc (AIN1) | Notes |
|-----------------|-----------------|-------|
| 0 A | 2.50 V | zero |
| 40 A | 2.50 + 40×0.01 = **2.90 V** | FSGP trip |
| ~200 A | ~4.5 V | CH2 FS |
| 480 A | saturates | not measurable |

**ASC wrap bench:** `I_eff = I_supply × N_wraps`. Temporarily set `current_sensor_turns = N` so reported pack amps stay correct.

---

## ASC Test Form — Preliminary Fill-In

### OVER VOLTAGE (OV) TEST — **Cell** level

| Field | Value |
|-------|-------|
| Test level | **Cell** |
| Nominal voltage (Vnom) | **3.60 V** @ 25 °C (Molicel nominal) |
| Max voltage (Vmax) | **4.20 V** @ 25 °C |
| BPS max trip (Vmax_trip) | **4.20 V** (4200 mV) |
| BPS V resolution | **~1 mV** (BQ76907, uint16 mV) |
| BPS V range | 2000–5000 mV (daughter validator); trip at 4200 mV |
| BPS sample rate | ~4 S/s new cell data (250 ms); fault check 100 S/s |
| Filtering delay | ≤250 ms + 50 mV hysteresis |
| BPS disconnect delay | **≤0.5 s** |
| Pass / N/A / Fail | **Pass** (pending bench trip test) |

**Notes:** Inject at daughter cell sense test point ahead of BQ76907. Highest cell across fleet triggers Primary OV fault. Contactors open; `0x040` fault bit `0x0001`.

---

### UNDER VOLTAGE (UV) TEST — **Cell** level

| Field | Value |
|-------|-------|
| Test level | **Cell** |
| Nominal voltage (Vnom) | **3.60 V** @ 25 °C |
| Min voltage (Vmin) | **2.50 V** @ 25 °C |
| BPS min trip (Vmin_trip) | **2.50 V** (2500 mV) |
| BPS V resolution | **~1 mV** |
| BPS V range | 2000–5000 mV |
| BPS sample rate | ~4 S/s / 100 S/s |
| Filtering delay | ≤250 ms + 50 mV hysteresis |
| BPS disconnect delay | **≤0.5 s** |
| Pass / N/A / Fail | **Pass** (pending bench trip test) |

**Notes:** Lowest cell across fleet triggers UV. `0x040` fault bit `0x0002`.

---

### OVER CURRENT (OC) TEST — **String** level

| Field | Value |
|-------|-------|
| Test level | **String** (DHAB S/134 → ADS1115 AIN0/AIN1) |
| Max current (charge) | **40 A** @ 25 °C (manufacturer / 5 A×8P) |
| Max current (discharge) | **480 A** @ 25 °C (manufacturer / 60 A×8P) |
| BPS I trip (charge) | **40 A** (FSGP); **N/A** if BMS off when charging |
| BPS I trip (discharge) | **40 A** abs (same threshold; FSGP cell limit 480 A beyond sensor) |
| BPS I resolution | **~0.1 A** (ADS1115 + CH1/CH2 blend) |
| BPS I range | Pack ≈ **±200 A** max (CH2, 1 turn); CH1 better below ~2.5 A (±50 A) |
| BPS sample rate | **10 S/s** (100 ms ADS1115 period) |
| Filtering delay | 1 A hysteresis + up to 100 ms sample |
| BPS disconnect delay | **≤0.5 s** |
| Pass / N/A / Fail | **Discharge: pending bench** / **Charge: N/A if BMS off** |

**Hall wrap test (bench):**

```text
I_pack_trip = 40 A (FSGP)
With N wraps and current_sensor_turns = N:
  I_supply = 40 / N   (e.g. N=10 → 4 A supply)
At 1 turn race: V_AIN1 @ trip = 2.90 V
```

Prefer commanding known current and watching `g_dbg_pack_current_A` / AIN1 voltage.
**Notes:** Code uses `|I| > threshold` (no charge/discharge sign). `0x040` fault bit `0x0020`. Charge OC test N/A — BMS off during charging.

---

### UNDER TEMPERATURE (UT) TEST — **Cell** level

| Field | Value |
|-------|-------|
| Test level | **Cell** (NTC on daughter; Primary uses fleet min) |
| BPS T trip | **−10 °C** (fleet max temp < threshold) |
| BPS T resolution | **~1 °C** (12-bit NTC ADC) |
| BPS sample rate | ~4 S/s temp updates; fault check 100 S/s |
| Filtering delay | 2 °C hysteresis (clear at −8 °C) |
| BPS disconnect delay | **≤0.5 s** |
| Pass / N/A / Fail | **Pass** (pending bench trip test) |

**Notes:** Cold soak or chilled NTC test point on all daughters (trips when fleet **max** temp < −10 °C). Fleet summary does not yet track lowest cell temp separately. `0x040` fault bit `0x0010` (masked in vehicle status — see fault table).

---

### OVER TEMPERATURE (OT) TEST — **Cell** level

| Field | Value |
|-------|-------|
| Test level | **Cell** (NTC on daughter; Primary uses fleet max) |
| Max operating temp (charge) | **60 °C** |
| Max operating temp (discharge) | **60 °C** |
| BPS T trip (charge) | **60 °C** |
| BPS T trip (discharge) | **60 °C** |
| BPS T resolution | **~1 °C** (12-bit NTC ADC) |
| BPS T range | 10–60 °C plausible filter on fleet aggregation |
| BPS sample rate | ~4 S/s temp updates; fault check 100 S/s |
| BPS disconnect delay | **≤0.5 s** |
| Pass / N/A / Fail | **Pass** (pending bench trip test) |

**Notes:** DB4 temperatures mirrored from DB1 in firmware (`kMirrorDb4TempsFromDb1`). Heat element at any daughter NTC or DB1 for DB4 path. `0x040` fault bit `0x0008`.

---

## Vehicle CAN Values During / After Trip

| CAN ID | Field | On fault |
|--------|-------|----------|
| 0x040 | `bms_faults` | Fault bitmask (see below) |
| 0x040 | `contactors_state` | `0` = open |
| 0x040 | `daughter_board_status` | Bitmap of online daughters |
| 0x041 | `highest_cell_mV` / `lowest_cell_mV` | Live pack extremes |
| 0x042 | `high_temp_C_x10` | Live max temp × 10 |
| 0x043 | `current_A` | Pack current (float) |
| 0x180 | `state` | `3` = FAULT |
| 0x190 | `active_faults` | Full 16-bit fault word (event) |

### Fault bitmask (`BmsManager` / CAN)

| Bit | Name | Trip condition |
|-----|------|----------------|
| 0x0001 | OVERVOLTAGE | highest cell > 4200 mV |
| 0x0002 | UNDERVOLTAGE | lowest cell < 2500 mV |
| 0x0004 | CELL_IMBALANCE | spread > 410 mV |
| 0x0008 | OVERTEMPERATURE | max temp > 60 °C |
| 0x0010 | UNDERTEMPERATURE | fleet max temp < −10 °C |
| 0x0020 | BATTERY_OVERCURRENT | \|pack I\| > 40 A |
| 0x0040 | AUX_OVERCURRENT | disabled in firmware |
| 0x0080 | FLEET_DATA_STALE | no daughter data > 500 ms |

**Note:** `0x040` masks faults to `0x003F` (aux/stale not shown on vehicle status frame).

---

## Code Changes Applied

### Pack current (canonical)

```cpp
// User.cpp setup() — DHAB S/134, race 1 turn, FSGP 40 A OC
cfg.overcurrent_A = 40.0f;
cfg.current_adc_channel_low = 0;    // AIN0 = CH1 40 mV/A
cfg.current_adc_channel_high = 1;   // AIN1 = CH2 10 mV/A
cfg.current_sensitivity_low_V_per_A = 0.040f;
cfg.current_sensitivity_high_V_per_A = 0.010f;
cfg.current_offset_V = 2.5f;
cfg.current_sensor_turns = 1;
// I_pack = (Vadc - 2.5) / (G * N_turns)
```

ADS1115 driver: team reference (`Addr7::GND`, `Pga::FS_4_096V`, `SPS_128`, single-shot, OS poll).

### Still TBD

- Measured P50B OCV curve for SoC EKF
- Flash persist for SoC state
- Separate charge 40 A / discharge 480 A if BMS stays on during charge (needs higher-range sensor for 480 A)

### Not changing (per team direction)

- Fault latching behavior (auto-clear when condition removed + 5 s recovery)
- 36-cell monitoring scope (6 daughters × 6 cells)
- DB4←DB1 temperature mirror (enabled)
- Aux overcurrent fault (disabled — not a pack BPS fault)

---

## Open Items Before Final Sign-Off

1. **Bench-verify pack current** — zero ≈ 2.5 V on AIN0/AIN1; trip near **40 A** pack (1 turn) or wrap-equivalent
2. **Bench undertemp trip** — document actual trip below −10 °C
3. **Document for inspector** why OC trip is **40 A** (FSGP charge), not discharge **480 A** (beyond DHAB ±200 A)
4. **Confirm physical test points** on battery box schematic
5. **Trip bench log** — record actual DMM / supply values and update this sheet

---

## Inspector Handout Checklist

- [ ] This preliminary test sheet (final revision after bench)
- [ ] Block diagram (BMS, dual contactors, DHAB S/134 + ADS1115 current sense)
- [ ] Schematic with labeled BPS test points (V, I, T)
- [ ] Firmware lockout method (programming header seal)
- [ ] Fault strobe / dash indicator wiring description
- [ ] Reset procedure after fault (power cycle / clear command / 5 s recovery)

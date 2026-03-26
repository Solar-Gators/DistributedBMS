# BQ76925 + ADC: Weird or Inconsistent Results — What to Check

If cell voltages are wrong, jumpy, or inconsistent, work through these in order.

---

## 1. **VCOUT settling time**

After writing **CELL_CTL** to switch the mux, the BQ76925 needs time for VCOUT to settle (datasheet **t_VCOUT** ≤ 100 µs).

- **Check:** Code now does `HAL_Delay(1)` (1 ms) after each `setCellForVCOUT(cell)` before starting the ADC. If you removed it, add at least ~100 µs (e.g. 1 ms is safe).
- **If still noisy:** Try 2 ms or a short busy-loop (~100 µs) if you need faster cycles.

---

## 2. **ADC channel and buffer index**

The STM32 fills `adc_buf[]` in **rank order**: rank 1 → index 0, rank 2 → index 1, … rank 6 → index 5.

- **Check:** In `main.cpp`, **ADC channel 10** must be **rank 6** (so VCOUT is on the pin connected to channel 10).
- **Check:** In `readCellsFromAFE`, we use **`adc_buf[5]`** for VCOUT. If your CubeMX rank order changed (e.g. channel 10 is rank 4), the index must match: **index = rank − 1**.
- **Quick test:** Set AFE to a known state (e.g. VCOUT_SEL = 10 or 11 for 0.5×VREF or 0.85×VREF), read all `adc_buf[0]..adc_buf[6]` and confirm which index changes when you change the AFE output.

---

## 3. **I²C communication**

If init fails or CELL_CTL writes don’t take effect, you’ll get wrong or constant readings.

- **Check:** `bq.init()` returns `HAL_OK` and you’re not stuck in the retry loop (fault pin / BQ76920_COMM_ERROR).
- **Check:** BQ76925 uses **combined 7‑bit address** `(0b0100 << 3) | REG_ADDR`. No other device on the bus should use the same addresses (0x20–0x3F for reg 0x00–0x1F).
- **Check:** Pull-ups on SDA/SCL, 100 kHz or 400 kHz within spec; no long wires or heavy capacitance.

---

## 4. **AFE power and reference**

- **Check:** BAT and V3P3 are in spec; REF_EN and VC_AMP_EN are set in `init()` (POWER_CTL).
- **Check:** CONFIG_2 REF_SEL = 1 so VCOUT gain = 0.6. If REF_SEL = 0, gain is 0.3 and the formula must be `v_cell = v_vcout / 0.3`.

---

## 5. **ADC reference and formula**

- **Check:** **ADC_VREF_V** in `readCellsFromAFE` matches the **actual** MCU VREF+ (e.g. 3.3 V). If you use the BQ76925 VREF pin (3.0 V) as the ADC reference, set it to 3.0f.
- **Check:** Formula is `v_vcout = (counts / 4095) * ADC_VREF_V`, then `v_cell = v_vcout / 0.6`. No mix-up between 3.0 V (AFE internal) and 3.3 V (MCU).

---

## 6. **DMA and buffer usage**

- **Check:** `HAL_ADC_Start_DMA(&hadc1, adc_buf, ADC_CHANNEL_COUNT)` is called with **length 7** and `adc_buf` has 7 elements. No overrun (e.g. length 5 with 7 ranks).
- **Check:** You wait for **TempDMAComplete** before reading `adc_buf[5]`. If something else starts the same ADC+DMA (e.g. temperature) before the wait finishes, you can read stale data — keep the cell loop and temp read sequential as now.
- **Check:** `HAL_ADC_ConvCpltCallback` sets `TempDMAComplete = true` and is linked to the correct `hadc1` (no other ADC sharing the flag).

---

## 7. **Hardware (noise, grounding, wiring)**

- **Check:** VCOUT and ADC input are wired to the **same** pin (e.g. PA0 for channel 10 on your STM32L4).
- **Check:** Analog ground (VSS of BQ76925, ADC ground) and digital ground are tied correctly; avoid ground loops.
- **Check:** Short, direct path for VCOUT to the MCU pin; no long traces next to high‑current or switching nets.
- **Check:** Decoupling on BAT and V3P3 per datasheet; COUT on VCOUT (e.g. 0.1 µF) for stability.

---

## 8. **Optional: average or discard first sample**

- **Noise:** After selecting a cell, run 2–3 ADC conversions and average, or discard the first and use the second.
- **Consistency:** If one cell index is always off, verify that `setCellForVCOUT(index)` is writing the correct CELL_SEL (0→VC1 … 5→VC6) and that the physical VC1–VC6 inputs are wired to the right cells.

---

## Quick checklist

| Item | What to verify |
|------|----------------|
| Settling | ≥ ~100 µs (e.g. 1 ms) after each `setCellForVCOUT()` before ADC start |
| VCOUT buffer index | `adc_buf[5]` only if channel 10 is rank 6 in CubeMX |
| I²C | init() OK; no address conflict; good pull-ups and speed |
| AFE config | REF_EN, VC_AMP_EN, REF_SEL=1 in init |
| Formula | ADC_VREF_V = real MCU VREF+; v_cell = v_vcout / 0.6 |
| DMA | Length 7; wait for complete before using adc_buf |
| Wiring | VCOUT → correct ADC channel; solid grounds and decoupling |

If you describe the symptom (e.g. “cell 3 always 0”, “all values jump”, “off by 200 mV”), the checklist can be narrowed further.

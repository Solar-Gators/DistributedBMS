#pragma once

#include <cstddef>
#include <cstdint>

/**
 * P50B cell OCV / R0 LUTs (About:Energy Voltt + DCIR CSV).
 * Pack model: Uoc = Ns * OCV_cell(SOC); R0_pack = (Ns/Np) * R0_cell(SOC,T).
 * BmsManager feeds pack V and pack I — always use pack-scaled R0.
 */
namespace P50B {

// OCV(SOC): from single-cell 1C sim OCV chart. 0–5% flat-extrapolated (sim floor ~5.4%).
inline float InterpOcv(float soc_pct)
{
    static const float kOcvTable_V[] = {
        2.7354f,  // 0%  (extrapolated)
        2.7354f,  // 5%  (extrapolated)
        2.8146f,  // 10%
        2.9535f,  // 15%
        3.0749f,  // 20%
        3.1731f,  // 25%
        3.2669f,  // 30%
        3.3469f,  // 35%
        3.4172f,  // 40%
        3.4700f,  // 45%
        3.5390f,  // 50%
        3.6096f,  // 55%
        3.6716f,  // 60%
        3.7167f,  // 65%
        3.7546f,  // 70%
        3.7997f,  // 75%
        3.8664f,  // 80%
        3.9253f,  // 85%
        3.9665f,  // 90%
        3.9861f,  // 95%
        4.0190f,  // 100%
    };
    constexpr float kSocStep = 5.0f;
    constexpr size_t kN = sizeof(kOcvTable_V) / sizeof(kOcvTable_V[0]);

    if (soc_pct <= 0.0f) {
        return kOcvTable_V[0];
    }
    if (soc_pct >= 100.0f) {
        return kOcvTable_V[kN - 1];
    }
    const float idx_f = soc_pct / kSocStep;
    const size_t idx0 = static_cast<size_t>(idx_f);
    const size_t idx1 = idx0 + 1u;
    if (idx1 >= kN) {
        return kOcvTable_V[kN - 1];
    }
    const float frac = idx_f - static_cast<float>(idx0);
    return kOcvTable_V[idx0] + frac * (kOcvTable_V[idx1] - kOcvTable_V[idx0]);
}

/** Bilinear R0(SOC%, T°C) → ohms. DCIR from Car_4_5-5_cell.csv, 25–40 °C. */
inline float InterpR0_Ohm(float soc_pct, float temp_C)
{
    static const float kSocGrid[] = {5.0f, 10.0f, 20.0f, 30.0f, 50.0f, 70.0f, 94.0f};
    static const float kTempGrid_C[] = {25.0f, 30.0f, 35.0f, 40.0f};
    static const float kR0_mOhm[7][4] = {
        {38.4f, 34.7f, 31.3f, 28.1f},  // 5%
        {31.1f, 25.6f, 20.4f, 16.1f},  // 10%
        {13.2f, 12.0f, 11.0f, 10.5f},  // 20%
        {10.8f, 10.2f, 9.7f, 9.4f},    // 30%
        {11.3f, 10.5f, 9.9f, 9.4f},    // 50%
        {11.1f, 10.4f, 9.9f, 9.5f},    // 70%
        {11.0f, 10.3f, 9.8f, 9.4f},    // 94%
    };
    constexpr size_t kNs = 7;
    constexpr size_t kNt = 4;

    float soc_c = soc_pct;
    if (soc_c < kSocGrid[0]) {
        soc_c = kSocGrid[0];
    } else if (soc_c > kSocGrid[kNs - 1]) {
        soc_c = kSocGrid[kNs - 1];
    }
    float t_c = temp_C;
    if (t_c < kTempGrid_C[0]) {
        t_c = kTempGrid_C[0];
    } else if (t_c > kTempGrid_C[kNt - 1]) {
        t_c = kTempGrid_C[kNt - 1];
    }

    size_t si = 0;
    while (si + 1u < kNs - 1u && kSocGrid[si + 1u] < soc_c) {
        ++si;
    }
    size_t ti = 0;
    while (ti + 1u < kNt - 1u && kTempGrid_C[ti + 1u] < t_c) {
        ++ti;
    }

    const float sfrac = (soc_c - kSocGrid[si]) / (kSocGrid[si + 1u] - kSocGrid[si]);
    const float tfrac = (t_c - kTempGrid_C[ti]) / (kTempGrid_C[ti + 1u] - kTempGrid_C[ti]);

    const float v00 = kR0_mOhm[si][ti];
    const float v01 = kR0_mOhm[si][ti + 1u];
    const float v10 = kR0_mOhm[si + 1u][ti];
    const float v11 = kR0_mOhm[si + 1u][ti + 1u];
    const float v0 = v00 + tfrac * (v01 - v00);
    const float v1 = v10 + tfrac * (v11 - v10);
    return (v0 + sfrac * (v1 - v0)) * 0.001f;
}

}  // namespace P50B

/**
 * Pack SoC estimator for 36S8P Molicel P50B.
 *
 * Inspired by UBC Solar DRD EKF (soc.c):
 *   - Predict: coulomb counting + RC polarization
 *   - Update: pack voltage vs OCV(R0, Uc) model using P50B LUTs
 *
 * Sign convention: discharge current > 0 (reduces SoC).
 * Persist: call exportState() / importState() around power cycles (flash TBD).
 */
class SocEstimator {
public:
    struct Config {
        /** Pack capacity (Coulombs). 40 Ah × 3600 = 144000 (5 Ah cell × 8P). */
        float Q_total_C = 144000.0f;
        uint8_t series_count = 36;
        uint8_t parallel_count = 8;
        /** Low-pass on current for R0·I term. */
        float current_lpf_alpha = 0.9f;
        /** |I| below this (A) for rest_time_ms → OCV hard reset allowed. */
        float rest_current_A = 1.0f;
        uint32_t rest_time_ms = 30000;
        float soc_min = 0.0f;
        float soc_max = 1.04f;
        /** Fallback cell temp (°C) when caller does not pass a measurement. */
        float default_temp_C = 25.0f;
    };

    /** Serializable snapshot for EEPROM/flash. */
    struct PersistedState {
        float soc = 1.0f;
        float uc_V = 0.0f;
        uint32_t magic = 0x50C50C01u;
    };

    SocEstimator() = default;
    explicit SocEstimator(const Config& cfg) : cfg_(cfg) {}

    void setConfig(const Config& cfg) { cfg_ = cfg; }
    const Config& config() const { return cfg_; }

    /** Boot / charge reconnect: seed SoC from measured pack voltage (OCV LUT). */
    void initFromPackVoltage(float pack_voltage_V);

    void importState(const PersistedState& s);
    PersistedState exportState() const;

    /**
     * Run one EKF step.
     * @param pack_voltage_V  Sum of cell voltages (fleet total), volts
     * @param pack_current_A  Discharge positive, amps
     * @param dt_s            Timestep seconds (e.g. 0.1f)
     * @param now_ms          RTOS tick for rest detection
     * @param pack_temp_C     Fleet temp for R0(SOC,T); defaults to Config::default_temp_C
     */
    void update(float pack_voltage_V,
                float pack_current_A,
                float dt_s,
                uint32_t now_ms,
                float pack_temp_C = -1000.0f);

    float soc() const { return state_soc_; }
    float socPercent() const { return state_soc_ * 100.0f; }
    float predictedVoltage_V() const { return last_predicted_V_; }
    float polarization_V() const { return state_uc_; }
    bool rested() const { return rested_; }

    /** Cell / pack helpers (P50B LUT). soc in [0,1]. */
    static float cellOcvFromSoc(float soc);
    float packOcvFromSoc(float soc) const;
    float cellR0FromSoc(float soc) const;
    float packR0FromSoc(float soc) const;

private:
    void predict(float current_A, float dt_s);
    void correct(float measured_V, float current_A);
    void maybeOcvReset(float pack_voltage_V, float pack_current_A, uint32_t now_ms);

    static float clampf(float v, float lo, float hi);

    Config cfg_{};

    float state_soc_ = 1.0f;
    float state_uc_ = 0.0f;
    float filtered_I_ = 0.0f;
    float last_predicted_V_ = 0.0f;
    float pack_temp_C_ = 25.0f;

    float P_[2][2] = {{5e-3f, 0.0f}, {0.0f, 1e-1f}};

    uint32_t rest_start_ms_ = 0;
    bool rested_ = false;
};

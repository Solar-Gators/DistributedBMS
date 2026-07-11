#include "SocEstimator.hpp"

#include <cmath>

namespace {

float cellRp(float x)
{
    (void)x;
    return 0.020f;
}

float cellCp(float x)
{
    (void)x;
    return 800.0f;
}

void mat2Mul(const float A[2][2], const float B[2][2], float C[2][2])
{
    C[0][0] = A[0][0] * B[0][0] + A[0][1] * B[1][0];
    C[0][1] = A[0][0] * B[0][1] + A[0][1] * B[1][1];
    C[1][0] = A[1][0] * B[0][0] + A[1][1] * B[1][0];
    C[1][1] = A[1][0] * B[0][1] + A[1][1] * B[1][1];
}

void mat2Add(const float A[2][2], const float B[2][2], float C[2][2])
{
    C[0][0] = A[0][0] + B[0][0];
    C[0][1] = A[0][1] + B[0][1];
    C[1][0] = A[1][0] + B[1][0];
    C[1][1] = A[1][1] + B[1][1];
}

}  // namespace

float SocEstimator::clampf(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

float SocEstimator::cellOcvFromSoc(float soc)
{
    return P50B::InterpOcv(soc * 100.0f);
}

float SocEstimator::packOcvFromSoc(float soc) const
{
    return cellOcvFromSoc(soc) * static_cast<float>(cfg_.series_count);
}

float SocEstimator::cellR0FromSoc(float soc) const
{
    return P50B::InterpR0_Ohm(soc * 100.0f, pack_temp_C_);
}

float SocEstimator::packR0FromSoc(float soc) const
{
    const float ns = static_cast<float>(cfg_.series_count);
    const float np = static_cast<float>(cfg_.parallel_count);
    if (np <= 0.0f) {
        return cellR0FromSoc(soc) * ns;
    }
    return cellR0FromSoc(soc) * (ns / np);
}

void SocEstimator::initFromPackVoltage(float pack_voltage_V)
{
    const float n = static_cast<float>(cfg_.series_count);
    const float cell_v = pack_voltage_V / n;

    float lo = 0.0f;
    float hi = 1.04f;
    for (int i = 0; i < 24; ++i) {
        const float mid = 0.5f * (lo + hi);
        if (cellOcvFromSoc(mid) < cell_v) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    state_soc_ = clampf(0.5f * (lo + hi), cfg_.soc_min, cfg_.soc_max);
    state_uc_ = 0.0f;
    filtered_I_ = 0.0f;
    rested_ = true;
}

void SocEstimator::importState(const PersistedState& s)
{
    if (s.magic != 0x50C50C01u) {
        return;
    }
    state_soc_ = clampf(s.soc, cfg_.soc_min, cfg_.soc_max);
    state_uc_ = s.uc_V;
}

SocEstimator::PersistedState SocEstimator::exportState() const
{
    PersistedState s{};
    s.soc = state_soc_;
    s.uc_V = state_uc_;
    s.magic = 0x50C50C01u;
    return s;
}

void SocEstimator::maybeOcvReset(float pack_voltage_V, float pack_current_A, uint32_t now_ms)
{
    const float abs_i = (pack_current_A < 0.0f) ? -pack_current_A : pack_current_A;
    if (abs_i > cfg_.rest_current_A) {
        rest_start_ms_ = 0;
        rested_ = false;
        return;
    }
    if (rest_start_ms_ == 0) {
        rest_start_ms_ = now_ms;
        return;
    }
    if ((now_ms - rest_start_ms_) >= cfg_.rest_time_ms) {
        initFromPackVoltage(pack_voltage_V);
        rested_ = true;
    }
}

void SocEstimator::predict(float current_A, float dt_s)
{
    const float np = static_cast<float>(cfg_.parallel_count);
    const float ns = static_cast<float>(cfg_.series_count);
    const float rp = cellRp(state_soc_) * (ns / np);
    const float cp = cellCp(state_soc_) * (np / ns);
    const float tau = rp * cp;
    const float exp_dt = expf(-dt_s / tau);

    const float B0 = -dt_s / cfg_.Q_total_C;
    const float B1 = rp * (1.0f - exp_dt);

    const float F[2][2] = {{1.0f, 0.0f}, {0.0f, exp_dt}};

    const float x0 = F[0][0] * state_soc_ + F[0][1] * state_uc_ + B0 * current_A;
    const float x1 = F[1][0] * state_soc_ + F[1][1] * state_uc_ + B1 * current_A;
    state_soc_ = x0;
    state_uc_ = x1;

    float Ft[2][2] = {{F[0][0], F[1][0]}, {F[0][1], F[1][1]}};
    float FP[2][2];
    mat2Mul(F, P_, FP);
    mat2Mul(FP, Ft, P_);
    static const float Q_proc[2][2] = {{1e-11f, 0.0f}, {0.0f, 1e-7f}};
    mat2Add(P_, Q_proc, P_);
}

void SocEstimator::correct(float measured_V, float current_A)
{
    filtered_I_ = cfg_.current_lpf_alpha * filtered_I_ + (1.0f - cfg_.current_lpf_alpha) * current_A;

    /* Finite difference over LUT (h ~ 0.1% SoC). */
    constexpr float h = 0.001f;
    const float dU =
        (packOcvFromSoc(state_soc_ + h) - packOcvFromSoc(state_soc_ - h)) / (2.0f * h);
    const float dR =
        (packR0FromSoc(state_soc_ + h) - packR0FromSoc(state_soc_ - h)) / (2.0f * h);

    const float H0 = dU - dR * filtered_I_;
    const float H1 = -1.0f;

    const float PHt0 = P_[0][0] * H0 + P_[0][1] * H1;
    const float PHt1 = P_[1][0] * H0 + P_[1][1] * H1;
    constexpr float R_meas = 0.5f;
    const float S = H0 * PHt0 + H1 * PHt1 + R_meas;

    const float K0 = PHt0 / S;
    const float K1 = PHt1 / S;

    const float Uoc = packOcvFromSoc(state_soc_);
    const float R0 = packR0FromSoc(state_soc_);
    const float hx = Uoc - state_uc_ - R0 * filtered_I_;
    last_predicted_V_ = hx;

    const float y = measured_V - hx;
    state_soc_ += K0 * y;
    state_uc_ += K1 * y;

    const float IKH[2][2] = {{1.0f - K0 * H0, -K0 * H1}, {-K1 * H0, 1.0f - K1 * H1}};
    float IKHT[2][2] = {{IKH[0][0], IKH[1][0]}, {IKH[0][1], IKH[1][1]}};
    float tmp[2][2];
    mat2Mul(IKH, P_, tmp);
    mat2Mul(tmp, IKHT, P_);
    P_[0][0] += K0 * K0 * R_meas;
    P_[0][1] += K0 * K1 * R_meas;
    P_[1][0] += K1 * K0 * R_meas;
    P_[1][1] += K1 * K1 * R_meas;

    state_soc_ = clampf(state_soc_, cfg_.soc_min, cfg_.soc_max);
}

void SocEstimator::update(float pack_voltage_V,
                          float pack_current_A,
                          float dt_s,
                          uint32_t now_ms,
                          float pack_temp_C)
{
    if (!(dt_s > 0.0f) || !std::isfinite(pack_voltage_V) || !std::isfinite(pack_current_A)) {
        return;
    }

    if (std::isfinite(pack_temp_C) && pack_temp_C > -50.0f && pack_temp_C < 100.0f) {
        pack_temp_C_ = pack_temp_C;
    } else {
        pack_temp_C_ = cfg_.default_temp_C;
    }

    maybeOcvReset(pack_voltage_V, pack_current_A, now_ms);
    predict(pack_current_A, dt_s);
    correct(pack_voltage_V, pack_current_A);
}

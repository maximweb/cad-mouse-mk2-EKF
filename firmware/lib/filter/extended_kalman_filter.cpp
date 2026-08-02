#include "extended_kalman_filter.h"
#include "performance_profiler.h"
#include <algorithm> // for std::isfinite
#include <cmath>
#include <cstring>

ExtendedKalmanFilter::ExtendedKalmanFilter() {}

void ExtendedKalmanFilter::init(const float initial_state[6], float process_noise_std, float sensor_noise_std)
{
    // Initialize state vector, strictly guarding against uninitialized NaNs
    for (int i = 0; i < 6; ++i) {
        m_x[i] = std::isfinite(initial_state[i]) ? initial_state[i] : 0.0f;
    }
    for (int i = 6; i < 12; ++i) {
        m_x[i] = 0.0f;
    }

    std::memset(m_P, 0, sizeof(m_P));
    for (int i = 0; i < 6; ++i)
        m_P[i][i] = 0.5f;
    for (int i = 6; i < 12; ++i)
        m_P[i][i] = 1.0f;

    m_R_var = sensor_noise_std * sensor_noise_std;

    m_Q_pos = 0.01f * process_noise_std;
    m_Q_rot = 0.01f * process_noise_std;
    m_Q_vel = 1.0f * process_noise_std;
    m_Q_ang_vel = 1.0f * process_noise_std;

    std::memset(m_cached_H, 0, sizeof(m_cached_H));
    m_cached_H_valid = false;
    m_update_counter = 0;
    m_jacobian_reuse_streak = 0;
}

void ExtendedKalmanFilter::predict(float dt)
{
    // FIX: Cap dt to prevent massive integration jumps on first boot or thread stalls
    if (dt <= 0.0f)
        return;
    if (dt > 0.1f)
        dt = 0.1f;

    // FIX: Sanitize state before predicting to prevent garbage propagation
    for (int i = 0; i < 12; ++i) {
        if (!std::isfinite(m_x[i]))
            m_x[i] = 0.0f;
    }

    // 1. Predict State
    for (int i = 0; i < 6; ++i) {
        m_x[i] += m_x[i + 6] * dt;
    }

    // 2. Safe, Explicit F * P * F^T matrix propagation
    float F_P[12][12];
    for (int i = 0; i < 12; ++i) {
        for (int j = 0; j < 12; ++j) {
            if (i < 6)
                F_P[i][j] = m_P[i][j] + m_P[i + 6][j] * dt;
            else
                F_P[i][j] = m_P[i][j];
        }
    }

    for (int i = 0; i < 12; ++i) {
        for (int j = 0; j < 12; ++j) {
            if (j < 6)
                m_P[i][j] = F_P[i][j] + F_P[i][j + 6] * dt;
            else
                m_P[i][j] = F_P[i][j];
        }
    }

    // 3. Inject Process Noise Covariance (Q)
    for (int i = 0; i < 3; ++i)
        m_P[i][i] += m_Q_pos * dt;
    for (int i = 3; i < 6; ++i)
        m_P[i][i] += m_Q_rot * dt;
    for (int i = 6; i < 9; ++i)
        m_P[i][i] += m_Q_vel * dt;
    for (int i = 9; i < 12; ++i)
        m_P[i][i] += m_Q_ang_vel * dt;

    // FIX: Enforce Symmetry immediately after prediction
    for (int i = 0; i < 12; ++i) {
        for (int j = i + 1; j < 12; ++j) {
            float avg = (m_P[i][j] + m_P[j][i]) * 0.5f;
            m_P[i][j] = avg;
            m_P[j][i] = avg;
        }
    }
}

void ExtendedKalmanFilter::compute_jacobian(const float state[12], float H[9][12], DipoleModel& dipole_model, float base_readings_out[9])
{
    float local_base_B[9];
    float* base_B = (base_readings_out != nullptr) ? base_readings_out : local_base_B;
    std::memset(H, 0, sizeof(float) * 9 * 12);

#if EKF_JACOBIAN_MODE == 1
    float d_readings_d_xyz[9][3];
    dipole_model.get_expected_readings_with_translation_jacobian(state[0], state[1], state[2], state[3], state[4], state[5], base_B, d_readings_d_xyz);

    // Analytic translation columns.
    for (int i = 0; i < 9; ++i) {
        H[i][0] = d_readings_d_xyz[i][0];
        H[i][1] = d_readings_d_xyz[i][1];
        H[i][2] = d_readings_d_xyz[i][2];
    }

    // Numeric rotation columns.
    for (int j = 3; j < 6; ++j) {
        float perturbed_pose[6] = {state[0], state[1], state[2], state[3], state[4], state[5]};

        float state_val = perturbed_pose[j];
        if (!std::isfinite(state_val)) {
            state_val = 0.0f;
        }

        float h = 1e-4f * std::fabs(state_val);
        if (h < 1e-4f)
            h = 1e-4f;

        perturbed_pose[j] = state_val + h;

        float perturbed_B[9];
        dipole_model.get_expected_readings(perturbed_pose[0], perturbed_pose[1], perturbed_pose[2], perturbed_pose[3], perturbed_pose[4], perturbed_pose[5], perturbed_B);

        for (int i = 0; i < 9; ++i) {
            float diff = perturbed_B[i] - base_B[i];
            H[i][j] = std::isfinite(diff) ? (diff / h) : 0.0f;
        }
    }
#else
    dipole_model.get_expected_readings(state[0], state[1], state[2], state[3], state[4], state[5], base_B);

    for (int j = 0; j < 6; ++j) {
        float perturbed_pose[6] = {state[0], state[1], state[2], state[3], state[4], state[5]};

        float state_val = perturbed_pose[j];
        if (!std::isfinite(state_val)) {
            state_val = 0.0f;
        }

        float h = 1e-4f * std::fabs(state_val);
        if (h < 1e-4f)
            h = 1e-4f;

        perturbed_pose[j] = state_val + h;

        float perturbed_B[9];
        dipole_model.get_expected_readings(perturbed_pose[0], perturbed_pose[1], perturbed_pose[2], perturbed_pose[3], perturbed_pose[4], perturbed_pose[5], perturbed_B);

        for (int i = 0; i < 9; ++i) {
            float diff = perturbed_B[i] - base_B[i];
            H[i][j] = std::isfinite(diff) ? (diff / h) : 0.0f;
        }
    }
#endif
}

void ExtendedKalmanFilter::update(float sensor_readings[9], DipoleModel& dipole_model)
{
    // FIX: Reject bad I2C/ADC reads entirely instead of poisoning the filter
    for (int i = 0; i < 9; ++i) {
        if (!std::isfinite(sensor_readings[i]))
            return;
    }

    float h_x[9];
    float y[9];
    float innovation_norm_sq = 0.0f;
    auto update_innovation = [&]() {
        innovation_norm_sq = 0.0f;
        for (int i = 0; i < 9; ++i) {
            const float diff = sensor_readings[i] - h_x[i];
            y[i] = std::isfinite(diff) ? diff : 0.0f;
            innovation_norm_sq += y[i] * y[i];
        }
    };

#if EKF_JACOBIAN_REUSE_ENABLE
    PERFORMANCE_BEGIN(1, PerformanceProfiler::Section::EKF_MODEL_HX);
    dipole_model.get_expected_readings(m_x[0], m_x[1], m_x[2], m_x[3], m_x[4], m_x[5], h_x);
    PERFORMANCE_END(1, PerformanceProfiler::Section::EKF_MODEL_HX);
    update_innovation();
#endif

    ++m_update_counter;
    bool should_recompute_jacobian = !m_cached_H_valid;

#if EKF_JACOBIAN_REUSE_ENABLE
    if (!should_recompute_jacobian) {
        if (EKF_JACOBIAN_RECOMPUTE_INTERVAL > 0 && (m_update_counter % EKF_JACOBIAN_RECOMPUTE_INTERVAL) == 0) {
            should_recompute_jacobian = true;
        }

        if (!should_recompute_jacobian && EKF_JACOBIAN_INNOVATION_THRESHOLD > 0.0f) {
            const float innovation_threshold_sq = EKF_JACOBIAN_INNOVATION_THRESHOLD * EKF_JACOBIAN_INNOVATION_THRESHOLD;
            if (innovation_norm_sq > innovation_threshold_sq) {
                should_recompute_jacobian = true;
            }
        }

        if (!should_recompute_jacobian && m_jacobian_reuse_streak >= EKF_JACOBIAN_MAX_REUSE_STREAK) {
            should_recompute_jacobian = true;
        }
    }
#else
    should_recompute_jacobian = true;
#endif

    PERFORMANCE_BEGIN(1, PerformanceProfiler::Section::EKF_JACOBIAN);
    if (should_recompute_jacobian) {
        compute_jacobian(m_x,
                         m_cached_H,
                         dipole_model,
#if EKF_JACOBIAN_REUSE_ENABLE
                         nullptr
#else
                         h_x
#endif
        );
        m_cached_H_valid = true;
        m_jacobian_reuse_streak = 0;
    }
    else {
        ++m_jacobian_reuse_streak;
    }
    PERFORMANCE_END(1, PerformanceProfiler::Section::EKF_JACOBIAN);

#if !EKF_JACOBIAN_REUSE_ENABLE
    PERFORMANCE_BEGIN(1, PerformanceProfiler::Section::EKF_MODEL_HX);
    PERFORMANCE_END(1, PerformanceProfiler::Section::EKF_MODEL_HX);
    update_innovation();
#endif

    float (*H)[12] = m_cached_H;

#ifdef _KALMAN_FILTER_SERIAL_DEBUG
    Serial.print("[DEBUG] Residual Y[0]: ");
    Serial.print(y[0]);
    Serial.print(" | h_x[0]: ");
    Serial.println(h_x[0]);

    Serial.print("[DEBUG] Jacobian recompute: ");
    Serial.print(should_recompute_jacobian ? "yes" : "no");
    Serial.print(" | reuse_streak: ");
    Serial.println(m_jacobian_reuse_streak);
#endif

    // Measurement model depends only on pose states (0..5), so H = [H_pose, 0].
    // Exploit this sparsity to reduce multiplications in HP and S.
    float HP_pose[9][6] = {0}; // columns 0..5 of H*P
    float HP_vel[9][6] = {0};  // columns 6..11 of H*P
    PERFORMANCE_BEGIN(1, PerformanceProfiler::Section::EKF_HP);
    for (int i = 0; i < 9; ++i) {
        float sum_pose[6] = {0};
        float sum_vel[6] = {0};

        for (int k = 0; k < 6; ++k) {
            const float h_ik = H[i][k];
            for (int j = 0; j < 6; ++j) {
                sum_pose[j] += h_ik * m_P[k][j];
                sum_vel[j] += h_ik * m_P[k][j + 6];
            }
        }

        for (int j = 0; j < 6; ++j) {
            HP_pose[i][j] = sum_pose[j];
            HP_vel[i][j] = sum_vel[j];
        }
    }
    PERFORMANCE_END(1, PerformanceProfiler::Section::EKF_HP);

    float S[9][9] = {0};
    PERFORMANCE_BEGIN(1, PerformanceProfiler::Section::EKF_S);
    for (int i = 0; i < 9; ++i) {
        for (int j = i; j < 9; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < 6; ++k) {
                sum += HP_pose[i][k] * H[j][k];
            }
            if (i == j) {
                sum += m_R_var;
            }
            S[i][j] = sum;
            if (j != i) {
                S[j][i] = sum;
            }
        }
    }
    PERFORMANCE_END(1, PerformanceProfiler::Section::EKF_S);

    // --- 1. STABILIZED CHOLESKY DECOMPOSITION ---
    float L[9][9] = {0};
    const float eps = 1e-3f;

    PERFORMANCE_BEGIN(1, PerformanceProfiler::Section::EKF_CHOLESKY);
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j <= i; j++) {
            float sum = 0;
            for (int k = 0; k < j; k++)
                sum += L[i][k] * L[j][k];

            if (i == j) {
                float val = (S[i][i] + eps) - sum;
                if (val < 1e-5f)
                    val = 1e-5f;
                L[i][j] = std::sqrt(val);
            }
            else {
                L[i][j] = (S[i][j] - sum) / (L[j][j] < 1e-5f ? 1e-5f : L[j][j]);
            }
        }
    }
    PERFORMANCE_END(1, PerformanceProfiler::Section::EKF_CHOLESKY);

#ifdef _KALMAN_FILTER_SERIAL_DEBUG
    Serial.print("[DEBUG] Matrix S Diagonal [0]: ");
    Serial.print(S[0][0]);
    Serial.print(" | L Diagonal [0]: ");
    Serial.println(L[0][0]);
#endif

    // --- 2. SOLVE S * iY = y ---
    PERFORMANCE_BEGIN(1, PerformanceProfiler::Section::EKF_SOLVE_IY);
    float iY_temp[9] = {0};
    for (int i = 0; i < 9; i++) {
        float sum = 0;
        for (int k = 0; k < i; k++)
            sum += L[i][k] * iY_temp[k];
        iY_temp[i] = (y[i] - sum) / L[i][i];
    }
    float iY[9] = {0};
    for (int i = 8; i >= 0; i--) {
        float sum = 0;
        for (int k = i + 1; k < 9; k++)
            sum += L[k][i] * iY[k];
        iY[i] = (iY_temp[i] - sum) / L[i][i];
    }
    PERFORMANCE_END(1, PerformanceProfiler::Section::EKF_SOLVE_IY);

    // --- 3. STATE UPDATE ---
    PERFORMANCE_BEGIN(1, PerformanceProfiler::Section::EKF_STATE_CORRECTION);
    for (int i = 0; i < 12; ++i) {
        float correction = 0.0f;
        for (int j = 0; j < 9; ++j) {
            const float ph_t_ij = (i < 6) ? HP_pose[j][i] : HP_vel[j][i - 6];
            correction += ph_t_ij * iY[j];
        }

        if (std::isfinite(correction)) {
            m_x[i] += correction;
        }
    }
    PERFORMANCE_END(1, PerformanceProfiler::Section::EKF_STATE_CORRECTION);

    // --- 4. UPDATE COVARIANCE MATRIX ---
    PERFORMANCE_BEGIN(1, PerformanceProfiler::Section::EKF_COVARIANCE_UPDATE);
    float K_row[9];
    float row_temp[9];
    for (int row = 0; row < 12; ++row) {
        std::memset(K_row, 0, sizeof(K_row));
        std::memset(row_temp, 0, sizeof(row_temp));

        for (int i = 0; i < 9; i++) {
            float sum = 0;
            for (int k = 0; k < i; k++)
                sum += L[i][k] * row_temp[k];
            const float ph_t_row_i = (row < 6) ? HP_pose[i][row] : HP_vel[i][row - 6];
            row_temp[i] = (ph_t_row_i - sum) / L[i][i];
        }
        for (int i = 8; i >= 0; i--) {
            float sum = 0;
            for (int k = i + 1; k < 9; k++)
                sum += L[k][i] * K_row[k];
            K_row[i] = (row_temp[i] - sum) / L[i][i];
        }

        // Update only upper triangle and mirror immediately to preserve symmetry.
        for (int col = row; col < 12; ++col) {
            const float hp0 = (col < 6) ? HP_pose[0][col] : HP_vel[0][col - 6];
            const float hp1 = (col < 6) ? HP_pose[1][col] : HP_vel[1][col - 6];
            const float hp2 = (col < 6) ? HP_pose[2][col] : HP_vel[2][col - 6];
            const float hp3 = (col < 6) ? HP_pose[3][col] : HP_vel[3][col - 6];
            const float hp4 = (col < 6) ? HP_pose[4][col] : HP_vel[4][col - 6];
            const float hp5 = (col < 6) ? HP_pose[5][col] : HP_vel[5][col - 6];
            const float hp6 = (col < 6) ? HP_pose[6][col] : HP_vel[6][col - 6];
            const float hp7 = (col < 6) ? HP_pose[7][col] : HP_vel[7][col - 6];
            const float hp8 = (col < 6) ? HP_pose[8][col] : HP_vel[8][col - 6];

            const float delta = K_row[0] * hp0 + K_row[1] * hp1 + K_row[2] * hp2 + K_row[3] * hp3 + K_row[4] * hp4 + K_row[5] * hp5 + K_row[6] * hp6 + K_row[7] * hp7 + K_row[8] * hp8;

            float updated = m_P[row][col] - delta;
            if (!std::isfinite(updated)) {
                updated = m_P[row][col];
            }

            m_P[row][col] = updated;
            if (col != row) {
                m_P[col][row] = updated;
            }
        }
    }

    // Keep covariance numerically valid.
    for (int i = 0; i < 12; ++i) {
        if (m_P[i][i] < 1e-6f) {
            m_P[i][i] = 1e-6f;
        }
    }
    PERFORMANCE_END(1, PerformanceProfiler::Section::EKF_COVARIANCE_UPDATE);
}

void ExtendedKalmanFilter::get_state(float state_out[12])
{
    for (int i = 0; i < 12; ++i)
        state_out[i] = m_x[i];
}
#include "extended_kalman_filter.h"
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

void ExtendedKalmanFilter::compute_jacobian(const float state[12], float H[9][12], DipoleModel& dipole_model)
{
    float base_B[9];
    dipole_model.get_expected_readings(state[0], state[1], state[2], state[3], state[4], state[5], base_B);
    std::memset(H, 0, sizeof(float) * 9 * 12);

    for (int j = 0; j < 6; ++j) {
        float perturbed_pose[6] = {state[0], state[1], state[2], state[3], state[4], state[5]};

        float state_val = perturbed_pose[j];
        if (!std::isfinite(state_val)) {
            state_val = 0.0f;
        }

        float h = 1e-4f * std::fabs(state_val);
        if (h < 1e-4f)
            h = 1e-4f;

        // FIX: The root cause of the immediate NaN lockup.
        // We MUST overwrite perturbed_pose[j] entirely to flush out the old NaN.
        perturbed_pose[j] = state_val + h;

        float perturbed_B[9];
        dipole_model.get_expected_readings(perturbed_pose[0], perturbed_pose[1], perturbed_pose[2], perturbed_pose[3], perturbed_pose[4], perturbed_pose[5], perturbed_B);

        for (int i = 0; i < 9; ++i) {
            float diff = perturbed_B[i] - base_B[i];
            H[i][j] = std::isfinite(diff) ? (diff / h) : 0.0f;
        }
    }
}

void ExtendedKalmanFilter::update(float sensor_readings[9], DipoleModel& dipole_model)
{
    // FIX: Reject bad I2C/ADC reads entirely instead of poisoning the filter
    for (int i = 0; i < 9; ++i) {
        if (!std::isfinite(sensor_readings[i]))
            return;
    }

    float H[9][12];
    compute_jacobian(m_x, H, dipole_model);

    float h_x[9];
    dipole_model.get_expected_readings(m_x[0], m_x[1], m_x[2], m_x[3], m_x[4], m_x[5], h_x);

    float y[9];
    for (int i = 0; i < 9; ++i) {
        float diff = sensor_readings[i] - h_x[i];
        y[i] = std::isfinite(diff) ? diff : 0.0f;
    }

#ifdef _KALMAN_FILTER_SERIAL_DEBUG
    Serial.print("[DEBUG] Residual Y[0]: ");
    Serial.print(y[0]);
    Serial.print(" | h_x[0]: ");
    Serial.println(h_x[0]);
#endif

    float HP[9][12] = {0};
    for (int i = 0; i < 9; ++i) {
        for (int j = 0; j < 12; ++j) {
            for (int k = 0; k < 12; ++k) {
                HP[i][j] += H[i][k] * m_P[k][j];
            }
        }
    }

    float S[9][9] = {0};
    for (int i = 0; i < 9; ++i) {
        for (int j = 0; j < 9; ++j) {
            for (int k = 0; k < 12; ++k) {
                S[i][j] += HP[i][k] * H[j][k];
            }
            if (i == j)
                S[i][j] += m_R_var;
        }
    }

    // --- 1. STABILIZED CHOLESKY DECOMPOSITION ---
    float L[9][9] = {0};
    const float eps = 1e-3f;

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

#ifdef _KALMAN_FILTER_SERIAL_DEBUG
    Serial.print("[DEBUG] Matrix S Diagonal [0]: ");
    Serial.print(S[0][0]);
    Serial.print(" | L Diagonal [0]: ");
    Serial.println(L[0][0]);
#endif

    // --- 2. SOLVE S * iY = y ---
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

    // FIX / OPTIMIZATION: Since we strictly maintain P as symmetric, P * H^T is
    // exactly the transpose of H * P. We can skip 1,296 multiplications completely!
    float PH_T[12][9];
    for (int i = 0; i < 12; ++i) {
        for (int j = 0; j < 9; ++j) {
            PH_T[i][j] = HP[j][i];
        }
    }

    // --- 3. STATE UPDATE ---
    for (int i = 0; i < 12; ++i) {
        float correction = 0.0f;
        for (int j = 0; j < 9; ++j)
            correction += PH_T[i][j] * iY[j];

        if (std::isfinite(correction)) {
            m_x[i] += correction;
        }
    }

    // --- 4. UPDATE COVARIANCE MATRIX ---
    float K[12][9] = {0};
    for (int row = 0; row < 12; ++row) {
        float row_temp[9] = {0};
        for (int i = 0; i < 9; i++) {
            float sum = 0;
            for (int k = 0; k < i; k++)
                sum += L[i][k] * row_temp[k];
            row_temp[i] = (PH_T[row][i] - sum) / L[i][i];
        }
        for (int i = 8; i >= 0; i--) {
            float sum = 0;
            for (int k = i + 1; k < 9; k++)
                sum += L[k][i] * K[row][k];
            K[row][i] = (row_temp[i] - sum) / L[i][i];
        }
    }

    float KHP[12][12] = {0};
    for (int i = 0; i < 12; ++i) {
        for (int j = 0; j < 12; ++j) {
            for (int k = 0; k < 9; ++k)
                KHP[i][j] += K[i][k] * HP[k][j];
        }
    }

    for (int i = 0; i < 12; ++i) {
        for (int j = 0; j < 12; ++j) {
            m_P[i][j] -= KHP[i][j];
        }
    }

    // FIX: Force Covariance Matrix to stay valid (Symmetric + Positive Diagonals)
    for (int i = 0; i < 12; ++i) {
        for (int j = i + 1; j < 12; ++j) {
            float avg = (m_P[i][j] + m_P[j][i]) * 0.5f;
            m_P[i][j] = avg;
            m_P[j][i] = avg;
        }
        if (m_P[i][i] < 1e-6f) {
            m_P[i][i] = 1e-6f;
        }
    }
}

void ExtendedKalmanFilter::get_state(float state_out[12])
{
    for (int i = 0; i < 12; ++i)
        state_out[i] = m_x[i];
}
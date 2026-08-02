#include "config.h"

#include "dipole_model.h"
#include "performance_profiler.h"

#if DEBUG_DIPOLE_MODEL_SERIAL
#define DIPOLE_LOG_PRINT(...) Serial.print(__VA_ARGS__)
#define DIPOLE_LOG_PRINTLN(...) Serial.println(__VA_ARGS__)
#define DIPOLE_LOG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define DIPOLE_LOG_PRINT(...) \
    do {                    \
    } while (0)
#define DIPOLE_LOG_PRINTLN(...) \
    do {                      \
    } while (0)
#define DIPOLE_LOG_PRINTF(...) \
    do {                     \
    } while (0)
#endif

DipoleModel::DipoleModel()
: m_scaled_magnetic_moments{
    m_mu0_over_4pi * m_magnetic_moments[0],
    m_mu0_over_4pi * m_magnetic_moments[1],
    m_mu0_over_4pi * m_magnetic_moments[2],
  }
{
}

void DipoleModel::set_magnetic_moments(float magnetic_moments[3])
{
    for (int i = 0; i < 3; ++i) {
        m_magnetic_moments[i] = magnetic_moments[i];
        m_scaled_magnetic_moments[i] = m_mu0_over_4pi * m_magnetic_moments[i];
    }
}

void DipoleModel::get_magnetic_moments(float magnetic_moments[3])
{
    for (int i = 0; i < 3; ++i) {
        magnetic_moments[i] = m_magnetic_moments[i];
    }
}

void DipoleModel::set_offsets(float offsets[6])
{
    for (int i = 0; i < 6; ++i) {
        m_offsets[i] = offsets[i];
    }
}

void DipoleModel::get_offsets(float offsets[6])
{
    for (int i = 0; i < 6; ++i) {
        offsets[i] = m_offsets[i];
    }
}

void DipoleModel::get_expected_readings(float x, float y, float z, float rx, float ry, float rz, float readings[9])
{
    PERFORMANCE_BEGIN(1, PerformanceProfiler::Section::DIPOLE_TOTAL);

    // Apply offsets to the input position and rotation
    x += m_offsets[0];
    y += m_offsets[1];
    z += m_offsets[2];
    rx += m_offsets[3];
    ry += m_offsets[4];
    rz += m_offsets[5];

    DIPOLE_LOG_PRINT("μ_0 / (4 * pi): ");
    DIPOLE_LOG_PRINTF("%.6e\n", m_mu0_over_4pi);
    DIPOLE_LOG_PRINT("Magnetic moments: ");
    DIPOLE_LOG_PRINTF("%.6e, %.6e, %.6e\n", m_magnetic_moments[0], m_magnetic_moments[1], m_magnetic_moments[2]);
    DIPOLE_LOG_PRINT("Scaled magnetic moments: ");
    DIPOLE_LOG_PRINTF("%.6e, %.6e, %.6e\n", m_scaled_magnetic_moments[0], m_scaled_magnetic_moments[1], m_scaled_magnetic_moments[2]);

    // Precompute sin and cos for rotation angles
    PERFORMANCE_BEGIN(1, PerformanceProfiler::Section::DIPOLE_ROTATION);
    float cx = cosf(rx);
    float sx = sinf(rx);
    float cy = cosf(ry);
    float sy = sinf(ry);
    float cz = cosf(rz);
    float sz = sinf(rz);

    // Compute the rotation matrix
    float R[3][3];
    R[0][0] = cy * cz;
    R[0][1] = sx * sy * cz - cx * sz;
    R[0][2] = cx * sy * cz + sx * sz;
    R[1][0] = cy * sz;
    R[1][1] = sx * sy * sz + cx * cz;
    R[1][2] = cx * sy * sz - sx * cz;
    R[2][0] = -sy;
    R[2][1] = sx * cy;
    R[2][2] = cx * cy;
    PERFORMANCE_END(1, PerformanceProfiler::Section::DIPOLE_ROTATION);

#if DEBUG_DIPOLE_MODEL_SERIAL
    DIPOLE_LOG_PRINTLN("Rotation matrix R:");
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            DIPOLE_LOG_PRINT(R[i][j], 6);
            DIPOLE_LOG_PRINT(" ");
        }
        DIPOLE_LOG_PRINTLN();
    }
#endif

    // Neutral dipole direction is positive Z-axis. Hence, dipole direction is 3rd column of rotation matrix R
    float dipole_x = R[0][2];
    float dipole_y = R[1][2];
    float dipole_z = R[2][2];

    DIPOLE_LOG_PRINT("Dipole direction: ");
    DIPOLE_LOG_PRINT(dipole_x, 6);
    DIPOLE_LOG_PRINT(", ");
    DIPOLE_LOG_PRINT(dipole_y, 6);
    DIPOLE_LOG_PRINT(", ");
    DIPOLE_LOG_PRINTLN(dipole_z, 6);

    // Clear output matrix
    for (int i = 0; i < 9; ++i) {
        readings[i] = 0.0f;
    }

    // Calculate dipole superposition for each magnet and sensor
    PERFORMANCE_BEGIN(1, PerformanceProfiler::Section::DIPOLE_SUPERPOSITION);
    for (int i_magnet = 0; i_magnet < 3; ++i_magnet) {
        // First rotate, then translate
        float magnet_x = R[0][0] * m_magnet_neutral_positions[i_magnet][0] + R[0][1] * m_magnet_neutral_positions[i_magnet][1] + R[0][2] * m_magnet_neutral_positions[i_magnet][2] + x;
        float magnet_y = R[1][0] * m_magnet_neutral_positions[i_magnet][0] + R[1][1] * m_magnet_neutral_positions[i_magnet][1] + R[1][2] * m_magnet_neutral_positions[i_magnet][2] + y;
        float magnet_z = R[2][0] * m_magnet_neutral_positions[i_magnet][0] + R[2][1] * m_magnet_neutral_positions[i_magnet][1] + R[2][2] * m_magnet_neutral_positions[i_magnet][2] + z;

        DIPOLE_LOG_PRINT("Magnet ");
        DIPOLE_LOG_PRINT(i_magnet);
        DIPOLE_LOG_PRINT(" position: ");
        DIPOLE_LOG_PRINT(magnet_x, 6);
        DIPOLE_LOG_PRINT(", ");
        DIPOLE_LOG_PRINT(magnet_y, 6);
        DIPOLE_LOG_PRINT(", ");
        DIPOLE_LOG_PRINTLN(magnet_z, 6);

        // Calculate the expected readings for each sensor
        for (int i_sensor = 0; i_sensor < 3; ++i_sensor) {
            // Distance vector from true magnet to sensor (in meters)
            float r_x = (m_sensor_positions[i_sensor][0] - magnet_x); // in mm
            float r_y = (m_sensor_positions[i_sensor][1] - magnet_y); // in mm
            float r_z = (m_sensor_positions[i_sensor][2] - magnet_z); // in mm

            DIPOLE_LOG_PRINT("Sensor ");
            DIPOLE_LOG_PRINT(i_sensor);
            DIPOLE_LOG_PRINT(" distance vector r in mm: ");
            DIPOLE_LOG_PRINT(r_x, 6);
            DIPOLE_LOG_PRINT(", ");
            DIPOLE_LOG_PRINT(r_y, 6);
            DIPOLE_LOG_PRINT(", ");
            DIPOLE_LOG_PRINTLN(r_z, 6);

            // Preocmpute r^2 and r_norm
            float r2 = r_x * r_x + r_y * r_y + r_z * r_z; // in mm^2
            float r_norm = sqrtf(r2);                     // in mm

            DIPOLE_LOG_PRINT("Sensor ");
            DIPOLE_LOG_PRINT(i_sensor);
            DIPOLE_LOG_PRINT(" r^2 in mm^2: ");
            DIPOLE_LOG_PRINTF("%.6e", r2);
            DIPOLE_LOG_PRINT(", r_norm in mm: ");
            DIPOLE_LOG_PRINTF("%.6e\n", r_norm);

            // Bailout for small r_norm to avoid division by zero
            if (r_norm < 1e-6f) {
                DIPOLE_LOG_PRINT("Warning: r_norm is too small (");
                DIPOLE_LOG_PRINT(r_norm);
                DIPOLE_LOG_PRINTLN("). Skipping this sensor-magnet pair to avoid division by zero.");
                continue;
            }

            // TODO: Avoid powf as RP2350 PFU is optimized for multiplication and division, but not for powf.
            float r_pow5_inv = 1.0f / (r2 * r2 * r_norm); // 1/r^5

            DIPOLE_LOG_PRINT("Sensor ");
            DIPOLE_LOG_PRINT(i_sensor);
            DIPOLE_LOG_PRINT(" r_pow5_inv: ");
            DIPOLE_LOG_PRINTF("%.10e\n", r_pow5_inv);

            // Calculate dot product of dipole direction and r
            float m_dot_r = dipole_x * r_x + dipole_y * r_y + dipole_z * r_z;
            DIPOLE_LOG_PRINT("Sensor ");
            DIPOLE_LOG_PRINT(i_sensor);
            DIPOLE_LOG_PRINT(" m_dot_r: ");
            DIPOLE_LOG_PRINTF("%.10e", m_dot_r);
            DIPOLE_LOG_PRINT(", m_dot_r_scaled: ");
            DIPOLE_LOG_PRINTF("%.10e\n", m_dot_r * m_scaled_magnetic_moments[i_magnet]);

            // Calculate the expected magnetic field vector using the dipole formula
            readings[i_sensor * 3 + 0] += m_scaled_magnetic_moments[i_magnet] * r_pow5_inv * (3.0f * m_dot_r * r_x - dipole_x * r2);
            readings[i_sensor * 3 + 1] += m_scaled_magnetic_moments[i_magnet] * r_pow5_inv * (3.0f * m_dot_r * r_y - dipole_y * r2);
            readings[i_sensor * 3 + 2] += m_scaled_magnetic_moments[i_magnet] * r_pow5_inv * (3.0f * m_dot_r * r_z - dipole_z * r2);
        }
    }
    PERFORMANCE_END(1, PerformanceProfiler::Section::DIPOLE_SUPERPOSITION);

#if DEBUG_DIPOLE_MODEL_SERIAL
    DIPOLE_LOG_PRINTLN("Expected readings (in mT):");
    for (int i = 0; i < 3; ++i) {
        DIPOLE_LOG_PRINT("Sensor ");
        DIPOLE_LOG_PRINT(i);
        DIPOLE_LOG_PRINT(": ");
        DIPOLE_LOG_PRINT(readings[i * 3 + 0], 6);
        DIPOLE_LOG_PRINT(", ");
        DIPOLE_LOG_PRINT(readings[i * 3 + 1], 6);
        DIPOLE_LOG_PRINT(", ");
        DIPOLE_LOG_PRINTLN(readings[i * 3 + 2], 6);
    }
#endif

    PERFORMANCE_END(1, PerformanceProfiler::Section::DIPOLE_TOTAL);
}

void DipoleModel::get_expected_readings_with_translation_jacobian(float x, float y, float z, float rx, float ry, float rz, float readings[9], float d_readings_d_xyz[9][3])
{
    PERFORMANCE_BEGIN(1, PerformanceProfiler::Section::DIPOLE_TOTAL);

    x += m_offsets[0];
    y += m_offsets[1];
    z += m_offsets[2];
    rx += m_offsets[3];
    ry += m_offsets[4];
    rz += m_offsets[5];

    PERFORMANCE_BEGIN(1, PerformanceProfiler::Section::DIPOLE_ROTATION);
    float cx = cosf(rx);
    float sx = sinf(rx);
    float cy = cosf(ry);
    float sy = sinf(ry);
    float cz = cosf(rz);
    float sz = sinf(rz);

    float R[3][3];
    R[0][0] = cy * cz;
    R[0][1] = sx * sy * cz - cx * sz;
    R[0][2] = cx * sy * cz + sx * sz;
    R[1][0] = cy * sz;
    R[1][1] = sx * sy * sz + cx * cz;
    R[1][2] = cx * sy * sz - sx * cz;
    R[2][0] = -sy;
    R[2][1] = sx * cy;
    R[2][2] = cx * cy;
    PERFORMANCE_END(1, PerformanceProfiler::Section::DIPOLE_ROTATION);

    const float dipole_x = R[0][2];
    const float dipole_y = R[1][2];
    const float dipole_z = R[2][2];

    for (int i = 0; i < 9; ++i) {
        readings[i] = 0.0f;
        d_readings_d_xyz[i][0] = 0.0f;
        d_readings_d_xyz[i][1] = 0.0f;
        d_readings_d_xyz[i][2] = 0.0f;
    }

    PERFORMANCE_BEGIN(1, PerformanceProfiler::Section::DIPOLE_SUPERPOSITION);
    for (int i_magnet = 0; i_magnet < 3; ++i_magnet) {
        const float magnet_x = R[0][0] * m_magnet_neutral_positions[i_magnet][0] + R[0][1] * m_magnet_neutral_positions[i_magnet][1] + R[0][2] * m_magnet_neutral_positions[i_magnet][2] + x;
        const float magnet_y = R[1][0] * m_magnet_neutral_positions[i_magnet][0] + R[1][1] * m_magnet_neutral_positions[i_magnet][1] + R[1][2] * m_magnet_neutral_positions[i_magnet][2] + y;
        const float magnet_z = R[2][0] * m_magnet_neutral_positions[i_magnet][0] + R[2][1] * m_magnet_neutral_positions[i_magnet][1] + R[2][2] * m_magnet_neutral_positions[i_magnet][2] + z;

        const float scaled_m = m_scaled_magnetic_moments[i_magnet];

        for (int i_sensor = 0; i_sensor < 3; ++i_sensor) {
            const float r_x = (m_sensor_positions[i_sensor][0] - magnet_x);
            const float r_y = (m_sensor_positions[i_sensor][1] - magnet_y);
            const float r_z = (m_sensor_positions[i_sensor][2] - magnet_z);

            const float r2 = r_x * r_x + r_y * r_y + r_z * r_z;
            const float r_norm = sqrtf(r2);
            if (r_norm < 1e-6f) {
                continue;
            }

            const float r_pow5_inv = 1.0f / (r2 * r2 * r_norm);
            const float m_dot_r = dipole_x * r_x + dipole_y * r_y + dipole_z * r_z;

            const float F_x = 3.0f * m_dot_r * r_x - dipole_x * r2;
            const float F_y = 3.0f * m_dot_r * r_y - dipole_y * r2;
            const float F_z = 3.0f * m_dot_r * r_z - dipole_z * r2;

            const int base = i_sensor * 3;
            readings[base + 0] += scaled_m * r_pow5_inv * F_x;
            readings[base + 1] += scaled_m * r_pow5_inv * F_y;
            readings[base + 2] += scaled_m * r_pow5_inv * F_z;

            const float r_pow7_inv = r_pow5_inv / r2;
            const float r_vec[3] = {r_x, r_y, r_z};
            const float d_vec[3] = {dipole_x, dipole_y, dipole_z};
            const float F_vec[3] = {F_x, F_y, F_z};

            for (int j = 0; j < 3; ++j) {
                const float rj = r_vec[j];
                const float dj = d_vec[j];

                // d(inv_r5)/dr_j
                const float d_inv_r5_drj = -5.0f * rj * r_pow7_inv;

                // dF/dr_j = 3*(d_j*r + m_dot_r*e_j) - 2*d*r_j
                const float dF_x_drj = 3.0f * (dj * r_x + (j == 0 ? m_dot_r : 0.0f)) - 2.0f * dipole_x * rj;
                const float dF_y_drj = 3.0f * (dj * r_y + (j == 1 ? m_dot_r : 0.0f)) - 2.0f * dipole_y * rj;
                const float dF_z_drj = 3.0f * (dj * r_z + (j == 2 ? m_dot_r : 0.0f)) - 2.0f * dipole_z * rj;

                const float dB_x_drj = scaled_m * (d_inv_r5_drj * F_vec[0] + r_pow5_inv * dF_x_drj);
                const float dB_y_drj = scaled_m * (d_inv_r5_drj * F_vec[1] + r_pow5_inv * dF_y_drj);
                const float dB_z_drj = scaled_m * (d_inv_r5_drj * F_vec[2] + r_pow5_inv * dF_z_drj);

                // r = sensor - magnet and magnet_{x,y,z} includes translation directly.
                // Therefore dr_j/dt_j = -1 and dB/dt_j = -dB/dr_j.
                d_readings_d_xyz[base + 0][j] += -dB_x_drj;
                d_readings_d_xyz[base + 1][j] += -dB_y_drj;
                d_readings_d_xyz[base + 2][j] += -dB_z_drj;
            }
        }
    }
    PERFORMANCE_END(1, PerformanceProfiler::Section::DIPOLE_SUPERPOSITION);

    PERFORMANCE_END(1, PerformanceProfiler::Section::DIPOLE_TOTAL);
}
#include "config.h"

#include "dipole_model.h"

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
    // Apply offsets to the input position and rotation
    x += m_offsets[0];
    y += m_offsets[1];
    z += m_offsets[2];
    rx += m_offsets[3];
    ry += m_offsets[4];
    rz += m_offsets[5];

// TODO Serial print m_magnetic_moment and m_scaled_magnetic_moment and m_mu0_over_4pi
#ifdef _DIPOLE_MODEL_SERIAL_DEBUG
    Serial.print("μ_0 / (4 * pi): ");
    Serial.printf("%.6e\n", m_mu0_over_4pi);
    Serial.print("Magnetic moments: ");
    Serial.printf("%.6e, %.6e, %.6e\n", m_magnetic_moments[0], m_magnetic_moments[1], m_magnetic_moments[2]);
    Serial.print("Scaled magnetic moments: ");
    Serial.printf("%.6e, %.6e, %.6e\n", m_scaled_magnetic_moments[0], m_scaled_magnetic_moments[1], m_scaled_magnetic_moments[2]);
#endif

    // Precompute sin and cos for rotation angles
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

    // TODO Test print rotation matrix R
#ifdef _DIPOLE_MODEL_SERIAL_DEBUG
    Serial.println("Rotation matrix R:");
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            Serial.print(R[i][j], 6);
            Serial.print(" ");
        }
        Serial.println();
    }
#endif

    // Neutral dipole direction is positive Z-axis. Hence, dipole direction is 3rd column of rotation matrix R
    float dipole_x = R[0][2];
    float dipole_y = R[1][2];
    float dipole_z = R[2][2];

    // TODO Test print dipole direction
#ifdef _DIPOLE_MODEL_SERIAL_DEBUG
    Serial.print("Dipole direction: ");
    Serial.print(dipole_x, 6);
    Serial.print(", ");
    Serial.print(dipole_y, 6);
    Serial.print(", ");
    Serial.println(dipole_z, 6);
#endif

    // Clear output matrix
    for (int i = 0; i < 9; ++i) {
        readings[i] = 0.0f;
    }

    // Calculate dipole superposition for each magnet and sensor
    for (int i_magnet = 0; i_magnet < 3; ++i_magnet) {
        // First rotate, then translate
        float magnet_x = R[0][0] * m_magnet_neutral_positions[i_magnet][0] + R[0][1] * m_magnet_neutral_positions[i_magnet][1] + R[0][2] * m_magnet_neutral_positions[i_magnet][2] + x;
        float magnet_y = R[1][0] * m_magnet_neutral_positions[i_magnet][0] + R[1][1] * m_magnet_neutral_positions[i_magnet][1] + R[1][2] * m_magnet_neutral_positions[i_magnet][2] + y;
        float magnet_z = R[2][0] * m_magnet_neutral_positions[i_magnet][0] + R[2][1] * m_magnet_neutral_positions[i_magnet][1] + R[2][2] * m_magnet_neutral_positions[i_magnet][2] + z;

        // TODO Test print magnet position
#ifdef _DIPOLE_MODEL_SERIAL_DEBUG
        Serial.print("Magnet ");
        Serial.print(i_magnet);
        Serial.print(" position: ");
        Serial.print(magnet_x, 6);
        Serial.print(", ");
        Serial.print(magnet_y, 6);
        Serial.print(", ");
        Serial.println(magnet_z, 6);
#endif

        // Calculate the expected readings for each sensor
        for (int i_sensor = 0; i_sensor < 3; ++i_sensor) {
            // Distance vector from true magnet to sensor (in meters)
            float r_x = (m_sensor_positions[i_sensor][0] - magnet_x); // in mm
            float r_y = (m_sensor_positions[i_sensor][1] - magnet_y); // in mm
            float r_z = (m_sensor_positions[i_sensor][2] - magnet_z); // in mm

            // TODO Test print distance vector r
#ifdef _DIPOLE_MODEL_SERIAL_DEBUG
            Serial.print("Sensor ");
            Serial.print(i_sensor);
            Serial.print(" distance vector r in mm: ");
            Serial.print(r_x, 6);
            Serial.print(", ");
            Serial.print(r_y, 6);
            Serial.print(", ");
            Serial.println(r_z, 6);
#endif

            // Preocmpute r^2 and r_norm
            float r2 = r_x * r_x + r_y * r_y + r_z * r_z; // in mm^2
            float r_norm = sqrtf(r2);                     // in mm

            // TODO Test print r^2 and r_norm
#ifdef _DIPOLE_MODEL_SERIAL_DEBUG
            Serial.print("Sensor ");
            Serial.print(i_sensor);
            Serial.print(" r^2 in mm^2: ");
            Serial.printf("%.6e", r2);
            Serial.print(", r_norm in mm: ");
            Serial.printf("%.6e\n", r_norm);
#endif

            // Bailout for small r_norm to avoid division by zero
            if (r_norm < 1e-6f) {
#ifdef _DIPOLE_MODEL_SERIAL_DEBUG
                Serial.print("Warning: r_norm is too small (");
                Serial.print(r_norm);
                Serial.println("). Skipping this sensor-magnet pair to avoid division by zero.");
#endif
                continue;
            }

            // TODO: Avoid powf as RP2350 PFU is optimized for multiplication and division, but not for powf.
            float r_pow5_inv = 1.0f / (r2 * r2 * r_norm); // 1/r^5

            // TODO Test print r_pow5_inv
#ifdef _DIPOLE_MODEL_SERIAL_DEBUG
            Serial.print("Sensor ");
            Serial.print(i_sensor);
            Serial.print(" r_pow5_inv: ");
            Serial.printf("%.10e\n", r_pow5_inv);
#endif

            // Calculate dot product of dipole direction and r
            float m_dot_r = dipole_x * r_x + dipole_y * r_y + dipole_z * r_z;
            float m_dot_r_scaled = m_dot_r * m_scaled_magnetic_moments[i_magnet];

            // TODO Test print m_dot_r and m_dot_r_scaled
#ifdef _DIPOLE_MODEL_SERIAL_DEBUG
            Serial.print("Sensor ");
            Serial.print(i_sensor);
            Serial.print(" m_dot_r: ");
            Serial.printf("%.10e", m_dot_r);
            Serial.print(", m_dot_r_scaled: ");
            Serial.printf("%.10e\n", m_dot_r_scaled);
#endif

            // Calculate the expected magnetic field vector using the dipole formula
            readings[i_sensor * 3 + 0] += m_scaled_magnetic_moments[i_magnet] * r_pow5_inv * (3.0f * m_dot_r * r_x - dipole_x * r2);
            readings[i_sensor * 3 + 1] += m_scaled_magnetic_moments[i_magnet] * r_pow5_inv * (3.0f * m_dot_r * r_y - dipole_y * r2);
            readings[i_sensor * 3 + 2] += m_scaled_magnetic_moments[i_magnet] * r_pow5_inv * (3.0f * m_dot_r * r_z - dipole_z * r2);
        }
    }

// TODO Test print expected readings
#ifdef _DIPOLE_MODEL_SERIAL_DEBUG
    Serial.println("Expected readings (in mT):");
    for (int i = 0; i < 3; ++i) {
        Serial.print("Sensor ");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(readings[i * 3 + 0], 6);
        Serial.print(", ");
        Serial.print(readings[i * 3 + 1], 6);
        Serial.print(", ");
        Serial.println(readings[i * 3 + 2], 6);
    }
#endif
}
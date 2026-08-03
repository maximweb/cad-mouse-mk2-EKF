#pragma once

#include <Arduino.h>

// Uncomment the following line to enable serial debug output for the DipoleModel class
// #define _DIPOLE_MODEL_SERIAL_DEBUG 1

class DipoleModel {
public:
    DipoleModel(); // magnetic moments in A*m^2
    void set_magnetic_moments(float magnetic_moments[3]);
    void get_magnetic_moments(float magnetic_moments[3]);
    void set_offsets(float offsets[6]); // offsets for x, y, z, rx, ry, rz
    void get_offsets(float offsets[6]);
    void get_expected_readings(float x, float y, float z, float rx, float ry, float rz, float readings[9]);

private:
    // μ_0 / (4 * pi) = 1e-7 T*m/A
    // scaled by 1e3 for returns in mT
    // scaled by 1e9 as positions are in mm, so we need to scale by 1e9 to convert m^3 to mm^3
    const float m_mu0_over_4pi = 1e-7 * 1e3f * 1e9;

    float m_magnetic_moments[3] = {0.18f, 0.18f, 0.18f}; // Magnetic moments for each of the three magnets
    float m_scaled_magnetic_moments[3];                  // Scaled magnetic moments for calculations

    float m_offsets[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // Offsets for x, y, z, rx, ry, rz

    const float m_sensor_positions[3][3] = {
      {0.0f, -16.51f, -19.42f},  // Sensor 1 position (X, Y, Z) in mm
      {-14.29f, 8.26f, -19.42f}, // Sensor 2 position (X, Y, Z) in mm
      {14.29f, 8.26f, -19.42f}   // Sensor 3 position (X, Y, Z) in mm
    };
    const float m_magnet_neutral_positions[3][3] = {
      {0.0f, -16.5f, -11.0f},   // Magnet 1 neutral position (X, Y, Z) in mm
      {-14.29f, 8.26f, -11.0f}, // Magnet 2 neutral position (X, Y, Z) in mm
      {14.29f, 8.26f, -11.0f}   // Magnet 3 neutral position (X, Y, Z) in mm
    };
};
#include "helpers.h"

void Helpers::print_raw_sensor_data(float rawSensorData[9])
{
    Serial.print("Raw Sensor Data: ");
    for (int i = 0; i < 9; ++i) {
        Serial.print(rawSensorData[i], 3);
        if (i < 8) {
            Serial.print(", ");
        }
    }
    Serial.println();
}

void Helpers::print_calibration_data(const CalibrationData& data)
{
    Serial.print("Calibration Data - Magnetic Moments: ");
    for (int i = 0; i < 3; ++i) {
        Serial.print(data.magnetic_moments[i], 6);
        if (i < 2) {
            Serial.print(", ");
        }
    }
    Serial.print(" | Offsets: ");
    for (int i = 0; i < 6; ++i) {
        Serial.print(data.offsets[i], 6);
        if (i < 5) {
            Serial.print(", ");
        }
    }
    Serial.println();
}

void Helpers::print_condensed_estimated_state(float estimated_state[12])
{
    Serial.print("Estimated State: ");
    for (int i = 0; i < 12; ++i) {
        Serial.print(estimated_state[i], 3);
        if (i < 11) {
            Serial.print(", ");
        }
    }
    Serial.println();
}

void Helpers::print_estimated_state(float estimated_state[12])
{
    Serial.println("Estimated State:");
    Serial.print("Position (x, y, z): ");
    for (int i = 0; i < 3; ++i) {
        Serial.print(estimated_state[i], 3);
        if (i < 2) {
            Serial.print(", ");
        }
    }
    Serial.println();

    Serial.print("Orientation (rx, ry, rz): ");
    for (int i = 3; i < 6; ++i) {
        Serial.print(estimated_state[i], 3);
        if (i < 5) {
            Serial.print(", ");
        }
    }
    Serial.println();

    Serial.print("Velocity (vx, vy, vz): ");
    for (int i = 6; i < 9; ++i) {
        Serial.print(estimated_state[i], 3);
        if (i < 8) {
            Serial.print(", ");
        }
    }
    Serial.println();

    Serial.print("Angular Velocity (vrx, vry, vrz): ");
    for (int i = 9; i < 12; ++i) {
        Serial.print(estimated_state[i], 3);
        if (i < 11) {
            Serial.print(", ");
        }
    }
    Serial.println();
}
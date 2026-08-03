#pragma once

#include "config.h"

#include <Arduino.h>
#include <LittleFS.h>

#include "dipole_model.h"

// Uncomment the following line to enable serial debug output for the Calibration class
// #define _CALIBRATION_SERIAL_DEBUG 1

struct CalibrationData {
    float magnetic_moments[3]; // Magnetic moments for each of the three magnets
    float offsets[6];          // Offsets for x, y, z, rx, ry, rz
};

namespace Calibration {
    enum LoadState {
        NO_FILE_USING_DEFAULTS,
        FROM_FILE,
        CALIBRATION_FAILED,
        CALIBRATION_SUCCESSFUL_SAVED,
        CALIBRATION_SUCCESSFUL_SAVED_FAILED,
    };

    void reset_calibration_data();
    void add_sample(float raw_data[9]);
    uint8_t get_sample_count();
    void get_current_means(float means[9]);
    void get_current_stds(float stds[9]);

    void get_model_func_moments(float* params, float* readings, DipoleModel& model);
    void get_model_func_offsets(float* params, float* readings, DipoleModel& model);

    void solve_least_squares(float* params, int N, const float* target_readings, DipoleModel& model, void (*get_model_func)(float*, float*, DipoleModel&), const float* min_bounds, const float* max_bounds);
    void solve_linear_system(float A[6][6], float* b, float* x, int N);

    bool compute_calibration(float magnetic_moments[3], float offsets[6], DipoleModel& dipoleModel);

    bool initialize_filesystem();
    bool save_calibration_data(const CalibrationData& data);
    bool load_calibration_data(CalibrationData& data);
    bool delete_calibration_data();
}
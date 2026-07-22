#pragma once

#include <Arduino.h>

#include "calibration.h"

namespace Helpers {
    void print_raw_sensor_data(float rawSensorData[9]);
    void print_calibration_data(const CalibrationData& data);
    void print_condensed_estimated_state(float estimated_state[12]);
    void print_estimated_state(float estimated_state[12]);
}
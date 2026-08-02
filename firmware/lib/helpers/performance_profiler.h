#pragma once

#include "config.h"

#include <Arduino.h>

#ifdef ENABLE_PERFORMANCE_PROFILING

namespace PerformanceProfiler {
    enum class Section : uint8_t { CORE0_SENSOR_READ = 0, CORE0_HANDOVER, CORE1_TOTAL, CORE1_PREDICT, CORE1_UPDATE, CORE1_NORMALIZATION, EKF_JACOBIAN, EKF_MODEL_HX, EKF_HP, EKF_S, EKF_CHOLESKY, EKF_SOLVE_IY, EKF_STATE_CORRECTION, EKF_COVARIANCE_UPDATE, DIPOLE_TOTAL, DIPOLE_ROTATION, DIPOLE_SUPERPOSITION, SECTION_COUNT };

    struct Stats {
        uint32_t count;
        uint64_t sum_us;
        uint32_t max_us;
    };

    void begin(uint8_t core_id, Section section);
    void end(uint8_t core_id, Section section);
    void reset(uint8_t core_id);
    void print_if_due(uint8_t core_id, uint32_t now_ms, uint32_t interval_ms = 1000);
}

#define PERFORMANCE_BEGIN(core_id, section) PerformanceProfiler::begin((core_id), (section))
#define PERFORMANCE_END(core_id, section) PerformanceProfiler::end((core_id), (section))

#else

#define PERFORMANCE_BEGIN(core_id, section) \
    do {                                    \
    } while (0)
#define PERFORMANCE_END(core_id, section) \
    do {                                  \
    } while (0)

#endif

#include "performance_profiler.h"

#ifdef ENABLE_PERFORMANCE_PROFILING

namespace {
    constexpr uint8_t kCores = 2;
    constexpr uint8_t kSectionCount = static_cast<uint8_t>(PerformanceProfiler::Section::SECTION_COUNT);

    PerformanceProfiler::Stats g_stats[kCores][kSectionCount] = {};
    uint32_t g_t0_us[kCores][kSectionCount] = {};
    uint32_t g_last_print_ms[kCores] = {};

    const char* kSectionNames[kSectionCount] = {
      "core0_sensor_read",
      "core0_handover",
      "core1_total",
      "core1_predict",
      "core1_update",
      "core1_normalization",
      "ekf_jacobian",
      "ekf_model_hx",
      "ekf_hp",
      "ekf_s",
      "ekf_cholesky",
      "ekf_solve_iy",
      "ekf_state_correction",
      "ekf_covariance_update",
      "dipole_total",
      "dipole_rotation",
      "dipole_superposition",
    };

    inline bool is_valid_core(uint8_t core_id)
    {
        return core_id < kCores;
    }

    inline bool should_print_section(uint8_t core_id, uint8_t section_id)
    {
        if (core_id == 0) {
            return section_id <= static_cast<uint8_t>(PerformanceProfiler::Section::CORE0_HANDOVER);
        }
        return section_id >= static_cast<uint8_t>(PerformanceProfiler::Section::CORE1_TOTAL);
    }
}

void PerformanceProfiler::begin(uint8_t core_id, Section section)
{
    if (!is_valid_core(core_id)) {
        return;
    }
    const uint8_t sid = static_cast<uint8_t>(section);
    if (sid >= kSectionCount) {
        return;
    }
    g_t0_us[core_id][sid] = micros();
}

void PerformanceProfiler::end(uint8_t core_id, Section section)
{
    if (!is_valid_core(core_id)) {
        return;
    }
    const uint8_t sid = static_cast<uint8_t>(section);
    if (sid >= kSectionCount) {
        return;
    }

    const uint32_t t1_us = micros();
    const uint32_t dt = t1_us - g_t0_us[core_id][sid];

    Stats& s = g_stats[core_id][sid];
    s.count += 1;
    s.sum_us += dt;
    if (dt > s.max_us) {
        s.max_us = dt;
    }
}

void PerformanceProfiler::reset(uint8_t core_id)
{
    if (!is_valid_core(core_id)) {
        return;
    }

    for (uint8_t sid = 0; sid < kSectionCount; ++sid) {
        g_stats[core_id][sid] = {0, 0, 0};
        g_t0_us[core_id][sid] = 0;
    }
    g_last_print_ms[core_id] = millis();
}

void PerformanceProfiler::print_if_due(uint8_t core_id, uint32_t now_ms, uint32_t interval_ms)
{
    if (!is_valid_core(core_id) || interval_ms == 0) {
        return;
    }

    if (g_last_print_ms[core_id] == 0) {
        g_last_print_ms[core_id] = now_ms;
        return;
    }

    if ((now_ms - g_last_print_ms[core_id]) < interval_ms) {
        return;
    }

    g_last_print_ms[core_id] = now_ms;

    float ref_mean = 0.0f;
    if (core_id == 0) {
        const Stats& ref = g_stats[core_id][static_cast<uint8_t>(Section::CORE0_SENSOR_READ)];
        if (ref.count > 0) {
            ref_mean = static_cast<float>(ref.sum_us) / static_cast<float>(ref.count);
        }
    }
    else {
        const Stats& ref = g_stats[core_id][static_cast<uint8_t>(Section::CORE1_TOTAL)];
        if (ref.count > 0) {
            ref_mean = static_cast<float>(ref.sum_us) / static_cast<float>(ref.count);
        }
    }

    Serial.print("[PERFORMANCE][core ");
    Serial.print(core_id);
    Serial.println("]");

    for (uint8_t sid = 0; sid < kSectionCount; ++sid) {
        if (!should_print_section(core_id, sid)) {
            continue;
        }

        const Stats& s = g_stats[core_id][sid];
        if (s.count == 0) {
            continue;
        }

        const float mean_us = static_cast<float>(s.sum_us) / static_cast<float>(s.count);
        float share_pct = 0.0f;
        if (ref_mean > 1e-6f) {
            share_pct = 100.0f * (mean_us / ref_mean);
        }

        Serial.print("  ");
        Serial.print(kSectionNames[sid]);
        Serial.print(": n=");
        Serial.print(s.count);
        Serial.print(" avg=");
        Serial.print(mean_us, 2);
        Serial.print("us max=");
        Serial.print(s.max_us);
        Serial.print("us share=");
        Serial.print(share_pct, 1);
        Serial.println("%");
    }

    Serial.println();
}

#endif

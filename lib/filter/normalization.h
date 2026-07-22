#pragma once

#include <Arduino.h>

namespace Normalization {
    void apply_normalization_deadzone_isolation(float state[12], float return_state[12]);
}
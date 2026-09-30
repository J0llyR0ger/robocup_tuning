#pragma once
#include "config/ramp.hpp"
#include "config/position_tracking.hpp"
#include <atomic>
#include <cstdint>

namespace ramp_menu {
enum class Mode : uint32_t { None, X, Y };
constexpr int Y_STEP_MM = 100;
constexpr unsigned MAX_Y_STEP = static_cast<int>(FIELD_HEIGHT_Y_METERS * 1000.0f) / Y_STEP_MM;
constexpr uint32_t encode(Mode mode, unsigned y_step) {
    return y_step * 3 + static_cast<uint32_t>(mode);
}
constexpr Mode mode(uint32_t value) { return static_cast<Mode>(value % 3); }
constexpr unsigned y_step(uint32_t value) { return value / 3; }
constexpr uint32_t change_mode(uint32_t value, bool increase) {
    return encode(static_cast<Mode>((static_cast<unsigned>(mode(value)) + (increase ? 1 : 2)) % 3), y_step(value));
}
constexpr uint32_t change_y(uint32_t value, bool increase) {
    unsigned step = y_step(value);
    if (increase && step < MAX_Y_STEP) ++step;
    if (!increase && step > 0) --step;
    return encode(mode(value), step);
}
constexpr ramp::Region region(uint32_t value) {
    return {mode(value) != Mode::None, ramp_config::CENTRE_X_MM,
            static_cast<int>(y_step(value)) * Y_STEP_MM,
            mode(value) == Mode::X ? ramp::Axis::X : ramp::Axis::Y};
}
static_assert(ramp_config::CENTRE_Y_MM >= 0 &&
              ramp_config::CENTRE_Y_MM % Y_STEP_MM == 0 &&
              ramp_config::CENTRE_Y_MM / Y_STEP_MM <= MAX_Y_STEP);
constexpr Mode DEFAULT_MODE = !ramp_config::ENABLED ? Mode::None :
    (ramp_config::LENGTH_AXIS == ramp::Axis::X ? Mode::X : Mode::Y);
// One atomic snapshot prevents mixing orientation and position from different edits.
inline std::atomic<uint32_t> selection{encode(DEFAULT_MODE, ramp_config::CENTRE_Y_MM / Y_STEP_MM)};
}

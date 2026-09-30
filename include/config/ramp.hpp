#pragma once
#include "lib/ramp.hpp"

namespace ramp_config {
// Set false when the field has no ramp. Coordinates use the fixed field frame,
// in millimetres (same X/Y as the occupancy map), independent of team colour.
// PLACEHOLDER position: measure and update before using on the playing field.
inline constexpr bool ENABLED = true;
inline constexpr int CENTRE_X_MM = 1212;
inline constexpr int CENTRE_Y_MM = 1200;
// Axis::X = 1200 mm along X; Axis::Y = 1200 mm along Y. Width is always 400 mm.
inline constexpr ramp::Axis LENGTH_AXIS = ramp::Axis::Y;
inline constexpr ramp::Region REGION{ENABLED, CENTRE_X_MM, CENTRE_Y_MM, LENGTH_AXIS};
}

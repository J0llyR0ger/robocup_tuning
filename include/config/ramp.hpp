#pragma once
#include "lib/ramp.hpp"

namespace ramp_config {
// Boot defaults for the display menu. Centre X stays fixed; the joystick edits
// mode and centre Y (100 mm steps). Coordinates are millimetres in the fixed
// field frame, independent of team colour. Selections are not saved over power-off.
inline constexpr bool ENABLED = false;
inline constexpr int CENTRE_X_MM = 1212;
inline constexpr int CENTRE_Y_MM = 1200;
// Axis::X = 1200 mm along X; Axis::Y = 1200 mm along Y. Width is always 400 mm.
inline constexpr ramp::Axis LENGTH_AXIS = ramp::Axis::Y;
inline constexpr ramp::Region REGION{ENABLED, CENTRE_X_MM, CENTRE_Y_MM, LENGTH_AXIS};
}

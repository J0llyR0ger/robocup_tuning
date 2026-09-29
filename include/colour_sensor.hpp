#pragma once
// Called at 100 ms intervals from the display task. Returns a static label.
const char *poll_colour_sensor();

#include "lib/colour.hpp"

// Latest stable reading; unknown if missing or older than 500 ms.
colour::Value current_colour();

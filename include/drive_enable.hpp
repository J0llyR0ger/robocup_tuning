#pragma once

#include <atomic>

// Written only by DriveTrainTask; all other tasks read this state.
inline std::atomic<bool> drive_enabled{false};

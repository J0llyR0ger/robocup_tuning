#pragma once
#include <atomic>

// LCD M selection, read by IntakeTask to position the auxiliary servo.
inline std::atomic<bool> menu_m_enabled{false};

// Call after Wire.begin(), before starting the scheduler.
void start_display_task();

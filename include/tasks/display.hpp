#pragma once
#include <atomic>

// LCD M selection, read by IntakeTask to position the auxiliary servo.
inline std::atomic<bool> menu_m_enabled{false};

// Return mode: 3 uses the slot 3 probe; 4 uses the slot 4 probe or four-count trigger.
inline std::atomic<unsigned> menu_return_count{4};

// Call after Wire.begin(), before starting the scheduler.
void start_display_task();

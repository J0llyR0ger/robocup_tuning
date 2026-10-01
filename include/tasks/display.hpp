#pragma once
#include <atomic>
#include <cstdint>

// LCD M selection, read by IntakeTask to position the auxiliary servo.
inline std::atomic<bool> menu_m_enabled{false};

// Return mode: 3 uses the slot 3 probe; 4 uses the slot 4 probe or four-count trigger.
inline std::atomic<unsigned> menu_return_count{4};

// Intake publishes raw SX1509 levels; bit 16 marks a valid read.
inline std::atomic<uint32_t> display_intake_probe_pins{0};

// Call after Wire.begin(), before starting the scheduler.
void start_display_task();

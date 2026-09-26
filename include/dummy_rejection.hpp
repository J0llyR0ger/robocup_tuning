#pragma once
#include <atomic>
#include <cstdint>

// Intake starts rejection; autonomous owns subsequent phases.
enum class DummyRejectionPriority : uint8_t { Idle, ReverseDown, Lift, ReverseUp };
inline std::atomic<DummyRejectionPriority> dummy_rejection_priority{DummyRejectionPriority::Idle};
// DriveTrainTask publishes measured backward motion; IntakeTask publishes release.
inline std::atomic<bool> dummy_moving_backwards{false};
inline std::atomic<bool> dummy_release_confirmed{false};
inline bool rejection_rails_up(bool normal_up) {
    const auto phase = dummy_rejection_priority.load();
    return phase == DummyRejectionPriority::Idle ? normal_up : phase != DummyRejectionPriority::ReverseDown;
}

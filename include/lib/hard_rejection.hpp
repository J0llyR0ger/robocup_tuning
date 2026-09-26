#pragma once
#include <cstdint>

namespace hard_rejection {
constexpr uint32_t TRIGGER_MS = 70;
constexpr uint32_t EDGE_MS = 20;
constexpr uint32_t LOWER_MS = 100;
constexpr uint32_t REVERSE_LIMIT_MS = 3000;
constexpr uint32_t LIFT_MS = 400;
constexpr uint32_t CLEARANCE_MS = 3000;
constexpr uint32_t REARM_MS = 100;
constexpr uint32_t SENSOR_STALE_MS = 150;
constexpr float REVERSE_COMMAND = -0.10f;

enum class Phase : uint8_t {
    Idle, Qualifying, Lowering, ReverseDown, Lifting, ReverseUp, WaitClear, FaultHold
};

struct Command {
    Phase phase = Phase::Idle;
    uint32_t phase_started = 0;
    uint32_t sampled_at = 0;

    constexpr bool active() const { return phase != Phase::Idle; }
    // Once qualified, this remains true through release, the entire lift, and
    // clearance. Loss of the original dummy condition cannot cancel rejection.
    constexpr bool latched() const { return active() && phase != Phase::Qualifying; }
    constexpr bool rails_down() const {
        return phase == Phase::Qualifying || phase == Phase::Lowering ||
               phase == Phase::ReverseDown || phase == Phase::FaultHold;
    }
    constexpr bool reverse(uint32_t now) const {
        if (uint32_t(now - sampled_at) > SENSOR_STALE_MS) return false;
        return (phase == Phase::ReverseDown && uint32_t(now - phase_started) < REVERSE_LIMIT_MS) ||
               (phase == Phase::ReverseUp && uint32_t(now - phase_started) < CLEARANCE_MS);
    }
};

// Raw sensor interlock outranks even a latched rejection's lift/clearance phase.
constexpr bool reject_condition(bool pin0_high, bool pin1_high, bool pin2_high) {
    return (pin0_high && !pin1_high) || !pin2_high;
}
constexpr bool resolve_rail_target(bool sensor_blocked, const Command &hard, bool normal_up) {
    if (sensor_blocked) return false;
    return hard.active() ? !hard.rails_down() : normal_up;
}

class Controller {
    Command command{};
    bool release_previous = false;
    uint32_t release_changed = 0;
    bool clear_timing = false;
    uint32_t clear_started = 0;
    constexpr void enter(Phase next, uint32_t now) {
        command.phase = next;
        command.phase_started = now;
        clear_timing = false;
    }
public:
    constexpr Command status() const { return command; }
    constexpr Command update(uint32_t now, bool enabled, bool pin0_high,
                             bool pin1_high, bool pin2_high) {
        command.sampled_at = now;
        if (!enabled) { enter(Phase::Idle, now); return command; }
        const bool fake = reject_condition(pin0_high, pin1_high, pin2_high);
        // Pin 0 changing does not release the latch. Both entry switches must
        // be clear; in particular an upside-down object must leave pin 2.
        const bool released = pin1_high && pin2_high;
        switch (command.phase) {
        case Phase::Idle:
            if (fake) enter(Phase::Qualifying, now);
            break;
        case Phase::Qualifying:
            if (!fake) enter(Phase::Idle, now);
            else if (uint32_t(now - command.phase_started) >= TRIGGER_MS)
                enter(Phase::Lowering, now);
            break;
        case Phase::Lowering:
            if (uint32_t(now - command.phase_started) >= LOWER_MS) {
                enter(Phase::ReverseDown, now);
                release_previous = released;
                release_changed = now;
            }
            break;
        case Phase::ReverseDown:
            if (released != release_previous) {
                release_previous = released;
                release_changed = now;
            }
            if (released && uint32_t(now - release_changed) >= EDGE_MS)
                enter(Phase::Lifting, now);
            if (command.phase == Phase::ReverseDown &&
                uint32_t(now - command.phase_started) >= REVERSE_LIMIT_MS)
                enter(Phase::FaultHold, now);
            break;
        case Phase::Lifting:
            // Never count time spent with the rails vetoed down as lifting.
            if (!released) enter(Phase::FaultHold, now);
            else if (uint32_t(now - command.phase_started) >= LIFT_MS)
                enter(Phase::ReverseUp, now);
            break;
        case Phase::ReverseUp:
            if (fake) enter(Phase::FaultHold, now);
            else if (uint32_t(now - command.phase_started) >= CLEARANCE_MS)
                enter(Phase::WaitClear, now);
            break;
        case Phase::FaultHold:
            // A timeout stops the motors, not the rejection latch. Once released,
            // perform the complete lift before allowing anything else to resume.
            if (released) {
                if (!clear_timing) { clear_timing = true; clear_started = now; }
                if (uint32_t(now - clear_started) >= EDGE_MS)
                    enter(Phase::Lifting, now);
            } else clear_timing = false;
            break;
        case Phase::WaitClear:
            // Only the completed lift + clearance sequence can clear the latch.
            if (fake) {
                enter(Phase::FaultHold, now);
                break;
            }
            if (released) {
                if (!clear_timing) { clear_timing = true; clear_started = now; }
                if (uint32_t(now - clear_started) >= REARM_MS)
                    enter(Phase::Idle, now);
            } else clear_timing = false;
            break;
        }
        return command;
    }
};
} // namespace hard_rejection

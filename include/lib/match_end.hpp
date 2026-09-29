#pragma once
#include <cstdint>

namespace match_end {
constexpr uint32_t PREPARE_AT_MS = 110000;
constexpr uint32_t STOP_AT_MS = 120000;
constexpr uint32_t LOWER_TIME_MS = 200;
constexpr uint32_t LIFT_TIME_MS = 400;
constexpr uint32_t RELEASE_TIME_MS = 20;
constexpr uint32_t SENSOR_MAX_AGE_MS = 100;
constexpr float REVERSE_SPEED = 0.10f;
enum class Phase : uint8_t { Idle, Running, Lowering, Reversing, Lifting, Finished, DispenseLowering, Holding };
constexpr bool owns_intake(Phase p) {
    return p != Phase::Idle && p != Phase::Running;
}
constexpr bool rails_up(Phase p) {
    return p != Phase::Lowering && p != Phase::DispenseLowering && p != Phase::Reversing;
}
struct Sensors {
    bool valid = false;
    bool slot1_occupied = false, slot4_occupied = false;
    bool entry_switch = false, upside_down_switch = false;
    uint32_t sampled_at = 0;
};
constexpr bool fresh(uint32_t now, const Sensors &s) {
    return s.valid && uint32_t(now - s.sampled_at) <= SENSOR_MAX_AGE_MS;
}
class Controller {
    uint32_t started = 0, phase_started = 0, clear_started = 0;
    bool rail_timer_running = false, prepared = false;
    bool weight_seen = false, clear_timing = false;
    constexpr void enter(Phase next, uint32_t now) {
        phase = next; phase_started = now; rail_timer_running = false;
    }
public:
    Phase phase = Phase::Idle;
    constexpr void start(uint32_t now) {
        started = now; prepared = weight_seen = clear_timing = false;
        enter(Phase::Running, now);
    }
    constexpr void cancel() { phase = Phase::Idle; }
    constexpr Phase update(uint32_t now, const Sensors &s, bool intake_detected, Phase rails_applied, bool returning_home = false) {
        if (phase == Phase::Idle || phase == Phase::Finished) return phase;
        if (uint32_t(now - started) >= STOP_AT_MS) return phase = Phase::Finished;
        if (phase == Phase::Running && !prepared && uint32_t(now - started) >= PREPARE_AT_MS) {
            prepared = true;
            // Skip this one-time sequence when the home delivery is already active.
            if (!returning_home) enter(Phase::Lowering, now);
        } else if (phase == Phase::Lowering || phase == Phase::Lifting || phase == Phase::DispenseLowering) {
            // Measure physical movement time from the intake task's command acknowledgement.
            if (!rail_timer_running) {
                if (rails_applied == phase) { phase_started = now; rail_timer_running = true; }
            } else if (phase == Phase::Lowering && uint32_t(now - phase_started) >= LOWER_TIME_MS) {
                enter(Phase::Lifting, now);
            } else if (phase == Phase::Lifting && uint32_t(now - phase_started) >= LIFT_TIME_MS && fresh(now, s)) {
                // Require a sample taken after the lift has settled.
                if (uint32_t(s.sampled_at - phase_started) >= LIFT_TIME_MS &&
                    uint32_t(s.sampled_at - phase_started) <= uint32_t(now - phase_started)) {
                    weight_seen = clear_timing = false;
                    enter(s.slot1_occupied && s.slot4_occupied ? Phase::DispenseLowering : Phase::Running, now);
                }
            } else if (phase == Phase::DispenseLowering && uint32_t(now - phase_started) >= LOWER_TIME_MS) {
                enter(Phase::Reversing, now);
            }
        }
        if (phase == Phase::DispenseLowering || phase == Phase::Reversing) {
            // intake_detected latches brief switch hits only during this dispensing attempt.
            if (intake_detected || (fresh(now, s) && (s.entry_switch || s.upside_down_switch))) weight_seen = true;
            if (phase == Phase::Reversing && fresh(now, s) && weight_seen &&
                !s.entry_switch && !s.upside_down_switch) {
                if (!clear_timing) { clear_started = s.sampled_at; clear_timing = true; }
                if (uint32_t(s.sampled_at - clear_started) >= RELEASE_TIME_MS) enter(Phase::Holding, now);
            } else {
                clear_timing = false;
            }
        }
        return phase;
    }
};
} // namespace match_end

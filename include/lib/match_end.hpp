#pragma once
#include <cstdint>

namespace match_end {
constexpr uint32_t PREPARE_AT_MS = 110000;
constexpr uint32_t STOP_AT_MS = 120000;
constexpr uint32_t LOWER_TIME_MS = 200;
constexpr uint32_t LIFT_TIME_MS = 400;
constexpr uint32_t SENSOR_MAX_AGE_MS = 100;
constexpr float REVERSE_SPEED = 0.10f;
enum class Phase : uint8_t { Idle, Running, Lowering, Reversing, Lifting, Finished };
constexpr bool owns_intake(Phase p) {
    return p == Phase::Lowering || p == Phase::Reversing || p == Phase::Lifting || p == Phase::Finished;
}
constexpr bool rails_up(Phase p) { return p != Phase::Lowering && p != Phase::Reversing; }
struct Sensors {
    bool valid = false;
    bool slot1_occupied = false, slot4_occupied = false;
    bool entry_switch = false, upside_down_switch = false;
    uint32_t sampled_at = 0;
};
class Controller {
    uint32_t started = 0, phase_started = 0;
    bool checked = false, rail_timer_running = false;
public:
    Phase phase = Phase::Idle;
    constexpr void start(uint32_t now) {
        started = phase_started = now; checked = false; rail_timer_running = false; phase = Phase::Running;
    }
    constexpr void cancel() { phase = Phase::Idle; }
    constexpr Phase update(uint32_t now, const Sensors &s, bool intake_detected, Phase rails_applied) {
        if (phase == Phase::Idle || phase == Phase::Finished) return phase;
        if (uint32_t(now - started) >= STOP_AT_MS) return phase = Phase::Finished;
        const bool fresh = s.valid && uint32_t(now - s.sampled_at) <= SENSOR_MAX_AGE_MS;
        if (phase == Phase::Running && !checked && uint32_t(now - started) >= PREPARE_AT_MS) {
            checked = true;
            if (fresh && s.slot1_occupied && s.slot4_occupied) {
                phase = Phase::Lowering; phase_started = now;
            }
        } else if ((phase == Phase::Lowering || phase == Phase::Lifting) && !rail_timer_running) {
            if (rails_applied == phase) { phase_started = now; rail_timer_running = true; }
        } else if (phase == Phase::Lowering && rail_timer_running &&
                   uint32_t(now - phase_started) >= LOWER_TIME_MS) {
            phase = fresh && !intake_detected ? Phase::Reversing : Phase::Lifting;
            phase_started = now; rail_timer_running = false;
        } else if (phase == Phase::Reversing && (!fresh || intake_detected)) {
            phase = Phase::Lifting; phase_started = now; rail_timer_running = false;
        } else if (phase == Phase::Lifting && rails_applied == Phase::Lifting &&
                   uint32_t(now - phase_started) >= LIFT_TIME_MS) {
            phase = Phase::Finished;
        }
        return phase;
    }
};
} // namespace match_end

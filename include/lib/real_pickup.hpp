#pragma once
#include <cstdint>

namespace real_pickup {
// Maximum forward intake time, measured from entering Forward.
constexpr uint32_t FORWARD_TIMEOUT_MS = 800;
enum class State { Idle, Stopping, Reversing, Lowering, Forward, Lifting };
struct Command { State state = State::Idle; uint32_t started = 0; };
constexpr bool should_start(bool entry_low, bool conduction_low, bool upside_down,
                            bool rails_are_up, bool enabled, bool rejecting) {
    return entry_low && conduction_low && !upside_down && rails_are_up && enabled && !rejecting;
}
constexpr State advance(State state, uint32_t elapsed, bool storage_detected) {
    switch (state) {
    case State::Stopping: return elapsed >= 400 ? State::Reversing : state;
    case State::Reversing: return elapsed >= 500 ? State::Lowering : state;
    case State::Lowering: return elapsed >= 400 ? State::Forward : state;
    case State::Forward:
        return storage_detected || elapsed >= FORWARD_TIMEOUT_MS ? State::Lifting : state;
    case State::Lifting: return elapsed >= 400 ? State::Idle : state;
    case State::Idle: return state;
    }
    return state;
}
constexpr bool rails_up(State state, bool initial_up) {
    if (state == State::Lowering || state == State::Forward) return false;
    return state == State::Lifting ? true : initial_up;
}
}

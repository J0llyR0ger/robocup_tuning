#include "lib/match_end.hpp"
using namespace match_end;
constexpr Sensors sample(uint32_t t, bool full = true) { return {true, full, full, false, false, t}; }
constexpr bool sequence(uint32_t start = 0) {
    Controller c; c.start(start);
    if (c.update(start + 109999, sample(start + 109999), false, Phase::Idle) != Phase::Running) return false;
    if (c.update(start + 110000, sample(start + 110000), false, Phase::Idle) != Phase::Lowering) return false;
    // No reverse before intake has actually issued the lowering command.
    if (c.update(start + 111000, sample(start + 111000), false, Phase::Idle) != Phase::Lowering) return false;
    c.update(start + 111010, sample(start + 111010), false, Phase::Lowering);
    if (c.update(start + 111209, sample(start + 111209), false, Phase::Lowering) != Phase::Lowering) return false;
    if (c.update(start + 111210, sample(start + 111210), false, Phase::Lowering) != Phase::Reversing) return false;
    if (c.update(start + 111300, sample(start + 111300), true, Phase::Reversing) != Phase::Lifting) return false;
    if (c.update(start + 112000, sample(start + 112000), true, Phase::Reversing) != Phase::Lifting) return false;
    c.update(start + 112010, sample(start + 112010), true, Phase::Lifting);
    if (c.update(start + 112409, sample(start + 112409), true, Phase::Lifting) != Phase::Lifting) return false;
    return c.update(start + 112410, sample(start + 112410), true, Phase::Lifting) == Phase::Finished;
}
static_assert(sequence());
static_assert(sequence(0xffff0000u));
constexpr bool not_full() {
    Controller c; c.start(0);
    auto s = sample(110000); s.slot1_occupied = false;
    if (c.update(110000, s, false, Phase::Idle) != Phase::Running) return false;
    if (c.update(110100, sample(110100), false, Phase::Idle) != Phase::Running) return false;
    return c.update(120000, sample(120000), false, Phase::Idle) == Phase::Finished;
}
static_assert(not_full());
constexpr bool deadline_and_restart() {
    Controller c; c.start(0);
    c.update(110000, sample(110000), false, Phase::Idle);
    c.update(110010, sample(110010), false, Phase::Lowering);
    if (c.update(110210, sample(110210), false, Phase::Lowering) != Phase::Reversing) return false;
    if (c.update(120000, sample(120000), false, Phase::Reversing) != Phase::Finished) return false;
    c.start(120100);
    if (c.update(120200, sample(120200), false, Phase::Idle) != Phase::Running) return false;
    c.cancel();
    return c.update(250000, sample(250000), false, Phase::Idle) == Phase::Idle;
}
static_assert(deadline_and_restart());
constexpr bool sensor_loss_stops_reverse() {
    Controller c; c.start(0);
    c.update(110000, sample(110000), false, Phase::Idle);
    c.update(110010, sample(110010), false, Phase::Lowering);
    c.update(110210, sample(110210), false, Phase::Lowering);
    return c.update(110400, sample(110210), false, Phase::Reversing) == Phase::Lifting;
}
static_assert(sensor_loss_stops_reverse());
constexpr bool occupied_intake_does_not_reverse() {
    Controller c; c.start(0);
    c.update(110000, sample(110000), true, Phase::Idle);
    c.update(110010, sample(110010), true, Phase::Lowering);
    return c.update(110210, sample(110210), true, Phase::Lowering) == Phase::Lifting;
}
static_assert(occupied_intake_does_not_reverse());

#include "lib/match_end.hpp"
using namespace match_end;
constexpr Sensors sample(uint32_t t, bool full = true, bool entry = false, bool upside = false) {
    return {true, full, full, entry, upside, t};
}
constexpr bool sequence(uint32_t start, bool slot1, bool slot4) {
    Controller c; c.start(start);
    if (c.update(start + 109999, {}, false, Phase::Idle) != Phase::Running) return false;
    if (c.update(start + 110000, {}, false, Phase::Idle) != Phase::Lowering) return false;
    if (c.update(start + 111000, {}, false, Phase::Idle) != Phase::Lowering) return false;
    c.update(start + 111010, {}, false, Phase::Lowering);
    if (c.update(start + 111209, {}, false, Phase::Lowering) != Phase::Lowering) return false;
    if (c.update(start + 111210, {}, false, Phase::Lowering) != Phase::Lifting) return false;
    if (c.update(start + 112000, {}, false, Phase::Lowering) != Phase::Lifting) return false;
    c.update(start + 112010, {}, false, Phase::Lifting);
    if (c.update(start + 112409, sample(start + 112409), false, Phase::Lifting) != Phase::Lifting) return false;
    // Invalid, stale, and pre-settling samples cannot decide occupancy.
    if (c.update(start + 112410, {}, false, Phase::Lifting) != Phase::Lifting) return false;
    if (c.update(start + 112500, sample(start + 112300), false, Phase::Lifting) != Phase::Lifting) return false;
    if (c.update(start + 112500, sample(start + 112409), false, Phase::Lifting) != Phase::Lifting) return false;
    Sensors slots{true, slot1, slot4, false, false, start + 112510};
    const bool full = slot1 && slot4;
    if (c.update(start + 112510, slots, false, Phase::Lifting) !=
        (full ? Phase::DispenseLowering : Phase::Running)) return false;
    if (!full) {
        // Later occupancy must not retrigger the 110-second sequence.
        if (c.update(start + 119999, sample(start + 119999), false, Phase::Idle) != Phase::Running) return false;
    } else {
        if (c.update(start + 112900, sample(start + 112900), false, Phase::Lifting) != Phase::DispenseLowering) return false;
        c.update(start + 112910, sample(start + 112910), false, Phase::DispenseLowering);
        if (c.update(start + 113109, sample(start + 113109), false, Phase::DispenseLowering) != Phase::DispenseLowering) return false;
        if (c.update(start + 113110, sample(start + 113110), false, Phase::DispenseLowering) != Phase::Reversing) return false;
        // Clear switches alone do not mean a weight has left.
        if (c.update(start + 113200, sample(start + 113200), false, Phase::Reversing) != Phase::Reversing) return false;
        c.update(start + 113210, sample(start + 113210, true, true), true, Phase::Reversing);
        if (c.update(start + 113220, sample(start + 113220, true, false, true), true, Phase::Reversing) != Phase::Reversing) return false;
        c.update(start + 113230, sample(start + 113230), true, Phase::Reversing);
        if (c.update(start + 113249, sample(start + 113249), true, Phase::Reversing) != Phase::Reversing) return false;
        // Bounce resets the clear interval.
        c.update(start + 113250, sample(start + 113250, true, true), true, Phase::Reversing);
        c.update(start + 113260, sample(start + 113260), true, Phase::Reversing);
        if (c.update(start + 113280, sample(start + 113280), true, Phase::Reversing) != Phase::Holding) return false;
        if (!owns_intake(c.phase) || !rails_up(c.phase)) return false;
        if (c.update(start + 119999, {}, false, Phase::Holding) != Phase::Holding) return false;
    }
    if (c.update(start + 120000, {}, false, Phase::Idle) != Phase::Finished) return false;
    return c.update(start + 120001, {}, false, Phase::Idle) == Phase::Finished;
}
static_assert(sequence(0, false, false));
static_assert(sequence(0, true, false));
static_assert(sequence(0, false, true));
static_assert(sequence(0, true, true));
static_assert(sequence(0xffff0000u, true, true));
static_assert(sequence(0xffff0000u, false, true));
constexpr bool deadline_and_restart() {
    Controller c; c.start(0);
    c.update(110000, {}, false, Phase::Idle);
    if (c.update(120000, {}, false, Phase::Idle) != Phase::Finished) return false;
    c.cancel();
    if (c.update(120001, {}, false, Phase::Idle) != Phase::Idle) return false;
    c.start(200000);
    if (c.update(309999, {}, false, Phase::Idle) != Phase::Running) return false;
    return c.update(310000, {}, false, Phase::Idle) == Phase::Lowering;
}
static_assert(deadline_and_restart());
static_assert(!owns_intake(Phase::Running));
static_assert(!rails_up(Phase::DispenseLowering));
static_assert(!rails_up(Phase::Reversing));
static_assert(!fresh(1000, sample(899)));

constexpr bool skip_while_returning_home(uint32_t start) {
    Controller c; c.start(start);
    if (c.update(start + 109999, {}, false, Phase::Idle, true) != Phase::Running) return false;
    if (c.update(start + 110000, {}, false, Phase::Idle, true) != Phase::Running) return false;
    // Finishing the home delivery must not trigger the skipped sequence later.
    if (c.update(start + 115000, {}, false, Phase::Idle, false) != Phase::Running) return false;
    if (c.update(start + 119999, {}, false, Phase::Idle, true) != Phase::Running) return false;
    if (c.update(start + 120000, {}, false, Phase::Idle, true) != Phase::Finished) return false;
    // Restart clears the one-time skip.
    c.start(start + 200000);
    return c.update(start + 310000, {}, false, Phase::Idle, false) == Phase::Lowering;
}
static_assert(skip_while_returning_home(0));
static_assert(skip_while_returning_home(0xffff0000u));

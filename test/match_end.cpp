#include "lib/match_end.hpp"
using namespace match_end;

constexpr bool sequence(uint32_t start, Sensors sensors, bool intake_detected) {
    Controller c; c.start(start);
    if (c.update(start + 109999, sensors, intake_detected, Phase::Idle) != Phase::Running) return false;
    if (c.update(start + 110000, sensors, intake_detected, Phase::Idle) != Phase::Lowering) return false;
    // Wait for the actual lowering command before starting the hold timer.
    if (c.update(start + 111000, sensors, intake_detected, Phase::Idle) != Phase::Lowering) return false;
    c.update(start + 111010, sensors, intake_detected, Phase::Lowering);
    if (c.update(start + 111209, sensors, intake_detected, Phase::Lowering) != Phase::Lowering) return false;
    if (c.update(start + 111210, sensors, intake_detected, Phase::Lowering) != Phase::Lifting) return false;
    // Lifting also waits for the intake task to issue the command.
    if (c.update(start + 112000, sensors, intake_detected, Phase::Lowering) != Phase::Lifting) return false;
    c.update(start + 112010, sensors, intake_detected, Phase::Lifting);
    if (c.update(start + 112409, sensors, intake_detected, Phase::Lifting) != Phase::Lifting) return false;
    if (c.update(start + 112410, sensors, intake_detected, Phase::Lifting) != Phase::Finished) return false;
    return c.update(start + 113000, sensors, intake_detected, Phase::Lifting) == Phase::Finished;
}
static_assert(sequence(0, {}, false)); // Missing/invalid sensors do not block the cycle.
static_assert(sequence(0, {true, false, false, false, false, 110000}, false));
static_assert(sequence(0, {true, true, true, true, true, 110000}, true));
static_assert(sequence(0, {true, true, false, false, false, 0}, false)); // Stale sample.
static_assert(sequence(0xffff0000u, {}, true)); // Timer wraparound.

constexpr bool late_update_and_deadline() {
    Controller c; c.start(0);
    if (c.update(111000, {}, false, Phase::Idle) != Phase::Lowering) return false;
    // The hard stop still applies even if the intake task never acknowledges.
    return c.update(120000, {}, false, Phase::Idle) == Phase::Finished;
}
static_assert(late_update_and_deadline());

constexpr bool restart_and_cancel() {
    Controller c; c.start(0);
    c.update(110000, {}, false, Phase::Idle);
    c.update(110010, {}, false, Phase::Lowering);
    c.cancel();
    if (c.update(120000, {}, false, Phase::Lowering) != Phase::Idle) return false;
    c.start(200000);
    if (c.update(309999, {}, false, Phase::Idle) != Phase::Running) return false;
    if (c.update(310000, {}, false, Phase::Idle) != Phase::Lowering) return false;
    return c.update(310500, {}, false, Phase::Idle) == Phase::Lowering;
}
static_assert(restart_and_cancel());

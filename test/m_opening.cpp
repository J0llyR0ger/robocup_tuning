#include "lib/m_opening.hpp"
using namespace m_opening;
constexpr bool sequence(uint32_t start = 0) {
    Dwell d;
    if (d.update(Phase::Approaching, start, false, false, false) != Phase::Approaching) return false;
    if (d.update(Phase::Approaching, start + 1, true, false, false) != Phase::Braking) return false;
    if (d.update(Phase::Braking, start + 1000, true, false, false) != Phase::Braking) return false;
    if (d.update(Phase::Braking, start + 1100, true, true, false) != Phase::Braking) return false;
    if (d.update(Phase::Braking, start + 1399, true, true, false) != Phase::Braking) return false;
    if (d.update(Phase::Braking, start + 1400, true, true, false) != Phase::Raising) return false;
    if (d.update(Phase::Raising, start + 2000, true, true, false) != Phase::Raising) return false;
    if (d.update(Phase::Raising, start + 2100, true, true, true) != Phase::Raising) return false;
    if (d.update(Phase::Raising, start + 3099, true, true, true) != Phase::Raising) return false;
    if (d.update(Phase::Raising, start + 3100, true, true, true) != Phase::Done) return false;
    return d.update(Phase::Done, start + 5000, true, true, true) == Phase::Done;
}
static_assert(sequence());
static_assert(sequence(0xfffffff0u));
static_assert(servo_up(Phase::Idle, false));
static_assert(!servo_up(Phase::Idle, true));
static_assert(!servo_up(Phase::Approaching, true));
static_assert(servo_up(Phase::Raising, true));
static_assert(servo_up(Phase::Done, true));
static_assert(!active(Phase::Idle) && !active(Phase::Done));
static_assert(hold(Phase::Braking) && hold(Phase::Raising) && !hold(Phase::Approaching));

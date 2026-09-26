#include "lib/hard_rejection.hpp"
using namespace hard_rejection;

constexpr Controller reversing_controller(uint32_t base = 0) {
    Controller c;
    c.update(base, true, true, false, true);
    c.update(base + 70, true, true, false, true);
    c.update(base + 170, true, true, false, true);
    return c;
}
constexpr bool trigger_filter() {
    Controller c;
    if (c.update(0, true, true, false, true).phase != Phase::Qualifying) return false;
    if (c.update(69, true, true, false, true).latched()) return false;
    if (c.update(70, true, false, true, true).active()) return false;
    // A dropout resets the entire 70 ms confirmation interval.
    c.update(100, true, true, false, true);
    if (c.update(169, true, true, false, true).latched()) return false;
    if (c.update(170, true, true, false, true).phase != Phase::Lowering) return false;
    return !c.status().reverse(170) && c.status().rails_down();
}
constexpr bool normal_sequence() {
    auto c = reversing_controller();
    if (!c.status().reverse(170) || !c.status().rails_down()) return false;
    c.update(240, true, true, true, true);
    if (c.update(259, true, true, true, true).phase != Phase::ReverseDown) return false;
    auto stopped = c.update(260, true, true, true, true);
    if (stopped.phase != Phase::Lifting || stopped.reverse(260) || stopped.rails_down()) return false;
    if (c.update(659, true, true, true, true).phase != Phase::Lifting) return false;
    auto extra = c.update(660, true, true, true, true);
    if (extra.phase != Phase::ReverseUp || !extra.reverse(660) || extra.rails_down()) return false;
    if (!c.update(3659, true, true, true, true).reverse(3659)) return false;
    if (c.update(3660, true, true, true, true).reverse(3660)) return false;
    c.update(3670, true, true, true, true);
    return !c.update(3770, true, true, true, true).active();
}
constexpr bool bouncing_release() {
    auto c = reversing_controller();
    c.update(240, true, true, true, true);
    c.update(250, true, true, false, true);
    c.update(270, true, true, true, true);
    if (c.update(289, true, true, true, true).phase != Phase::ReverseDown) return false;
    return c.update(290, true, true, true, true).phase == Phase::Lifting;
}
constexpr bool initial_timeout() {
    auto c = reversing_controller();
    if (!c.update(3169, true, true, false, true).reverse(3169)) return false;
    auto timed_out = c.update(3170, true, true, false, true);
    if (timed_out.phase != Phase::FaultHold || timed_out.reverse(3170) || !timed_out.rails_down())
        return false;
    if (c.update(10040, true, true, false, true).phase != Phase::FaultHold) return false;
    c.update(10050, true, true, true, true);
    auto released = c.update(10150, true, true, true, true);
    return released.phase == Phase::Lifting && released.latched() &&
           !released.reverse(10150);
}
constexpr bool upside_down() {
    Controller c;
    c.update(0, true, false, true, false);
    if (c.update(70, true, false, true, false).phase != Phase::Lowering) return false;
    c.update(170, true, false, true, false);
    if (c.update(190, true, false, true, false).phase != Phase::ReverseDown) return false;
    c.update(200, true, false, true, true);
    return c.update(220, true, false, true, true).phase == Phase::Lifting;
}
constexpr bool real_weight_ignored() {
    Controller c;
    c.update(0, true, false, false, true);
    return !c.update(1040, true, false, false, true).active();
}
constexpr bool inhibit_and_stale() {
    auto c = reversing_controller();
    if (c.status().reverse(321)) return false;
    return !c.update(340, false, true, false, true).active();
}
constexpr bool wraparound_timeout() {
    constexpr uint32_t base = 0xffffff00u;
    auto c = reversing_controller(base);
    return c.update(uint32_t(base + 3170), true, true, false, true).phase == Phase::FaultHold;
}
constexpr bool held_switch_no_retrigger() {
    auto c = reversing_controller();
    c.update(240, true, true, true, true);
    c.update(260, true, true, true, true);
    c.update(660, true, true, false, true);
    c.update(3660, true, true, false, true);
    return c.update(10040, true, true, false, true).phase == Phase::FaultHold;
}
static_assert(trigger_filter());
static_assert(normal_sequence());
static_assert(bouncing_release());
static_assert(initial_timeout());
static_assert(upside_down());
static_assert(real_weight_ignored());
static_assert(inhibit_and_stale());
static_assert(wraparound_timeout());
static_assert(held_switch_no_retrigger());


// Test every pin combination and every rejection phase: neither a forced UP
// nor the rejection lift phase may defeat the raw sensor interlock.
constexpr bool rail_interlock() {
    for (int pins = 0; pins < 8; ++pins) {
        const bool p0 = (pins & 1) != 0, p1 = (pins & 2) != 0, p2 = (pins & 4) != 0;
        const bool blocked = reject_condition(p0, p1, p2);
        if (blocked != ((p0 && !p1) || !p2)) return false;
        for (int phase = 0; phase <= static_cast<int>(Phase::FaultHold); ++phase) {
            const Command command{static_cast<Phase>(phase), 0, 0};
            if (blocked && resolve_rail_target(blocked, command, true)) return false;
            if (blocked && resolve_rail_target(blocked, command, false)) return false;
        }
    }
    return resolve_rail_target(false, Command{Phase::Lifting, 0, 0}, false) &&
           resolve_rail_target(false, Command{}, true) &&
           !resolve_rail_target(true, Command{}, true); // startup / invalid sample
}
static_assert(rail_interlock());


// Changing conductivity must not cancel rejection while the entry is occupied.
constexpr bool latch_survives_detection_loss() {
    auto c = reversing_controller();
    const auto still_rejecting = c.update(200, true, false, false, true);
    if (!still_rejecting.latched() || !still_rejecting.rails_down()) return false;
    if (resolve_rail_target(false, still_rejecting, true)) return false;
    c.update(210, true, false, true, true);
    const auto lifting = c.update(230, true, false, true, true);
    if (!lifting.latched() || lifting.phase != Phase::Lifting) return false;
    return c.update(629, true, false, true, true).phase == Phase::Lifting &&
           c.update(630, true, false, true, true).latched();
}
constexpr bool interrupted_lift_remains_latched() {
    auto c = reversing_controller();
    c.update(200, true, true, true, true);
    c.update(220, true, true, true, true);
    const auto blocked = c.update(300, true, true, false, true);
    if (blocked.phase != Phase::FaultHold || !blocked.rails_down()) return false;
    c.update(400, true, true, true, true);
    if (c.update(420, true, true, true, true).phase != Phase::Lifting) return false;
    if (c.update(819, true, true, true, true).phase != Phase::Lifting) return false;
    return c.update(820, true, true, true, true).phase == Phase::ReverseUp;
}
static_assert(latch_survives_detection_loss());
static_assert(interrupted_lift_remains_latched());

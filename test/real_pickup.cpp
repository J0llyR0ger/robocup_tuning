#include "lib/real_pickup.hpp"
using namespace real_pickup;
constexpr bool pickup_sequence() {
    auto s = State::Stopping;
    if (!rails_up(s, true) || advance(s, 399, false) != s) return false;
    s = advance(s, 400, false);
    if (s != State::Reversing || !rails_up(s, true)) return false;
    s = advance(s, 500, false);
    if (s != State::Lowering || rails_up(s, true)) return false;
    s = advance(s, 400, false);
    if (s != State::Forward || rails_up(s, true)) return false;
    if (advance(s, FORWARD_TIMEOUT_MS - 1, false) != s) return false;
    s = advance(s, 200, true);
    if (s != State::Lifting || !rails_up(s, false)) return false;
    return advance(s, 399, true) == s && advance(s, 400, true) == State::Idle;
}
static_assert(pickup_sequence());
static_assert(!rails_up(State::Stopping, false));
static_assert(!rails_up(State::Reversing, false));
static_assert(should_start(true, true, false, true, true, false));
static_assert(!should_start(false, true, false, true, true, false));
static_assert(!should_start(true, false, false, true, true, false));
static_assert(!should_start(true, true, false, false, true, false));
static_assert(!should_start(true, true, true, true, true, false));
static_assert(!should_start(true, true, false, true, false, false));
static_assert(!should_start(true, true, false, true, true, true));

static_assert(advance(State::Forward, FORWARD_TIMEOUT_MS, false) == State::Lifting);
static_assert(advance(State::Forward, FORWARD_TIMEOUT_MS + 1, false) == State::Lifting);
static_assert(rails_up(advance(State::Forward, FORWARD_TIMEOUT_MS, false), false));

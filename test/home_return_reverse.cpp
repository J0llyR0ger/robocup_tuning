#include "lib/home_return_reverse.hpp"
using home_return::LimitReverse;
constexpr bool timed_reverse() {
    LimitReverse c;
    if (c.update(false, true, true, 0)) return false;
    if (c.update(true, false, true, 10)) return false;
    if (!c.update(true, true, true, 100)) return false;
    if (!c.update(true, true, false, 101)) return false;
    if (!c.update(true, false, false, 2099)) return false;
    if (c.update(true, true, true, 2100)) return false;
    if (c.update(true, true, true, 5000)) return false;
    if (c.update(true, false, false, 5001)) return false;
    if (c.update(true, true, true, 5002)) return false;
    if (c.update(true, true, false, 5003)) return false;
    if (!c.update(true, true, true, 5004)) return false;
    if (c.update(false, true, true, 5005)) return false;
    return c.update(true, true, true, 5006);
}
constexpr bool clock_wrap() {
    LimitReverse c;
    return c.update(true, true, true, UINT32_MAX - 999) &&
           c.update(true, true, true, 999) &&
           !c.update(true, true, true, 1000);
}
static_assert(timed_reverse());
static_assert(clock_wrap());

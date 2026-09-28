#pragma once
#include <cstdint>

namespace m_opening {
enum class Phase : uint8_t { Idle, Approaching, Braking, Raising, Done };
constexpr uint32_t BRAKE_MS = 300;
constexpr uint32_t SERVO_WAIT_MS = 1000;
constexpr bool active(Phase p) {
    return p == Phase::Approaching || p == Phase::Braking || p == Phase::Raising;
}
constexpr bool hold(Phase p) { return p == Phase::Braking || p == Phase::Raising; }
constexpr bool servo_up(Phase p, bool menu_m) {
    return p == Phase::Raising || p == Phase::Done || (p == Phase::Idle && !menu_m);
}
// Begin each dwell only after the owning task acknowledges its command.
class Dwell {
    Phase previous = Phase::Idle;
    bool timing = false;
    uint32_t since = 0;
public:
    constexpr Phase update(Phase p, uint32_t now, bool arrived, bool stopped, bool raised) {
        if (p != previous) { previous = p; timing = false; }
        if (p == Phase::Approaching && arrived) return Phase::Braking;
        const bool acknowledged = p == Phase::Braking ? stopped : p == Phase::Raising && raised;
        if (acknowledged && !timing) { timing = true; since = now; }
        if (timing && p == Phase::Braking && uint32_t(now - since) >= BRAKE_MS) return Phase::Raising;
        if (timing && p == Phase::Raising && uint32_t(now - since) >= SERVO_WAIT_MS) return Phase::Done;
        return p;
    }
};
}

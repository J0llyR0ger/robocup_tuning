#pragma once
#include <cstdint>

namespace home_return {
constexpr uint32_t LIMIT_REVERSE_MS = 2000;
class LimitReverse {
    bool reversing = false;
    bool armed = true;
    uint32_t started = 0;
public:
    constexpr bool update(bool enabled, bool valid, bool pressed, uint32_t now) {
        if (!enabled) {
            reversing = false;
            armed = true;
            return false;
        }
        if (reversing) {
            if (uint32_t(now - started) < LIMIT_REVERSE_MS) return true;
            reversing = false;
        }
        // Require a valid release after the manoeuvre before another trigger.
        if (valid && !pressed) armed = true;
        if (valid && pressed && armed) {
            armed = false;
            reversing = true;
            started = now;
        }
        return reversing;
    }
};
}

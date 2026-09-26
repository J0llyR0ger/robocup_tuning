#pragma once

// The entry LOW is not release. Inertia first carries the weight past the
// switch (HIGH); only a new LOW while moving backwards confirms its return.
class DummyReleaseDetector {
    bool saw_clear = false;
    bool previous_low = true;
    bool confirmed = false;
public:
    constexpr void reset() { saw_clear = false; previous_low = true; confirmed = false; }
    constexpr bool update(bool switch_low, bool moving_backwards) {
        if (!switch_low) saw_clear = true;
        if (saw_clear && !previous_low && switch_low && moving_backwards) confirmed = true;
        previous_low = switch_low;
        return confirmed;
    }
};

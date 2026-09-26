#include "lib/dummy_release.hpp"
constexpr bool release_sequence() {
    DummyReleaseDetector d;
    if (d.update(true, false)) return false; // Initial entry LOW.
    if (d.update(true, true)) return false;  // Held LOW while reversing is not a second edge.
    if (d.update(false, false)) return false; // Inertia carries it inside: HIGH.
    if (d.update(false, true)) return false; // Clear while reversing is not release.
    if (!d.update(true, true)) return false; // Second LOW confirms immediately.
    if (!d.update(false, false)) return false; // Latched for slower consumer.
    d.reset();
    return !d.update(true, true);
}
constexpr bool forward_edge_does_not_release() {
    DummyReleaseDetector d;
    d.update(true, false);
    d.update(false, false);
    if (d.update(true, false)) return false; // LOW while still moving forwards.
    if (d.update(true, true)) return false; // No new edge when direction changes.
    d.update(false, true);
    return d.update(true, true);
}
static_assert(release_sequence());
static_assert(forward_edge_does_not_release());

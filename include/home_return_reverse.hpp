#pragma once
#include <atomic>
#include "lib/home_return_reverse.hpp"

// Autonomous owns travelling; intake samples switches and owns the reverse timer.
inline std::atomic<bool> home_return_travelling{false};
// Covers release and the final reverse: departing weights must not trigger intake recovery.
inline std::atomic<bool> home_drop_active{false};
inline std::atomic<bool> home_return_limit_reversing{false};

#pragma once
#include "lib/match_end.hpp"
#include <atomic>

// DriveTrainTask owns the run timer and phase; IntakeTask acknowledges rail commands.
inline std::atomic<match_end::Phase> match_end_phase{match_end::Phase::Idle};
inline std::atomic<match_end::Phase> match_end_rails_applied{match_end::Phase::Idle};

// Latch brief switch hits at intake sampling frequency during the dispensing attempt only.
inline std::atomic<bool> match_end_limit_seen{false};

// Autonomous task publishes the entire return, unload, and departure sequence.
inline std::atomic<bool> match_end_returning_home{false};

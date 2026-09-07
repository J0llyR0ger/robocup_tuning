#include "stdint.h"

enum class DebounceState {
    Unknown,
    False,
    True,
};

class Debouncer {
  private:
    uint32_t transition_delay_ms;

    bool last_sensed_state = false;

    DebounceState state = DebounceState::Unknown;

    uint32_t last_sense_state_transition_timestamp = 0;
    uint32_t last_read_state_timestamp = 0;

  public:
    Debouncer(uint32_t transition_delay_ms);

    DebounceState read_state(bool sensed_state);

    void reset();
};
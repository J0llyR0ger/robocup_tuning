#include "lib/debouncer.hpp"
#include "Arduino.h"

Debouncer::Debouncer(uint32_t transition_delay_ms) : transition_delay_ms(transition_delay_ms) {}

DebounceState Debouncer::read_state(bool sensed_state) {
    uint32_t now = millis();

    if (last_read_state_timestamp != 0 &&
        now - last_read_state_timestamp > transition_delay_ms * 2) {
        state = DebounceState::Unknown;
        last_sense_state_transition_timestamp = now;
    }

    last_read_state_timestamp = now;

    if (last_sensed_state != sensed_state) {
        last_sense_state_transition_timestamp = now;
    }

    if (now - last_sense_state_transition_timestamp > transition_delay_ms) {
        if (sensed_state) {
            state = DebounceState::True;
        } else {
            state = DebounceState::False;
        }
    }

    last_sensed_state = sensed_state;

    return state;
}

void Debouncer::reset() {
    uint32_t now = millis();
    last_sense_state_transition_timestamp = now;
    last_read_state_timestamp = now;
    state = DebounceState::Unknown;
}
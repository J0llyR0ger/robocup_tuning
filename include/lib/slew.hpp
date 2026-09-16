#pragma once

#include <cstdint>
template <typename SlewType> class SlewRate {

  private:
    SlewType rate_limit;
    SlewType prev_val;
    uint32_t prev_time;

  public:
    SlewRate(SlewType rate_limit, SlewType init_val = 0)
        : rate_limit(rate_limit), prev_val(init_val) {
        prev_time = millis();
    }

    SlewType update(SlewType value) {
        uint32_t curr_t = millis();
        uint32_t delta_t = curr_t - prev_time;
        prev_time = curr_t;

        if (delta_t == 0) {
            return prev_val; // or delta_t = 1, depending on desired behavior
        }

        SlewType diff = value - prev_val;
        SlewType max_step = (rate_limit * (SlewType)delta_t) / (SlewType)1000;

        if (diff > max_step) {
            prev_val = prev_val + max_step;
        } else if (diff < -max_step) {
            prev_val = prev_val - max_step;
        } else {
            prev_val = value;
        }

        return prev_val;
    }
};
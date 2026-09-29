#pragma once
#include <cstdint>
#include "config/colour.hpp"
namespace colour {
enum class Value { Unknown, Black, Blue, Green };
constexpr Value classify(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
    if (c <= colour_config::BLACK_CLEAR_MAX) return Value::Black;
    // 21 integration cycles saturate at 21504 counts.
    if (c >= 21504 || r >= 21504 || g >= 21504 || b >= 21504) return Value::Unknown;
    const auto margin = colour_config::DOMINANCE_PERCENT;
    if (uint32_t(b)*100 > uint32_t(r)*margin && uint32_t(b)*100 > uint32_t(g)*margin) return Value::Blue;
    if (uint32_t(g)*100 > uint32_t(r)*margin && uint32_t(g)*100 > uint32_t(b)*margin) return Value::Green;
    return Value::Unknown;
}
constexpr const char *name(Value v) {
    switch (v) {
    case Value::Black: return "BLACK";
    case Value::Blue: return "BLUE";
    case Value::Green: return "GREEN";
    default: return "UNKNOWN";
    }
}
}

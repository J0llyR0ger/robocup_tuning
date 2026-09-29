#pragma once
#include <cstdint>
namespace colour_config {
// CON62 is RAW I2C1: the colour sensor uses Wire1, without a mux.
constexpr bool USE_MUX = false;
constexpr uint8_t MUX_ADDRESS = 0x70;
constexpr uint8_t MUX_CHANNEL = 6;
// Initial calibration at 50.4 ms integration, 4x gain; tune using Serial RGBC logs.
constexpr uint16_t BLACK_CLEAR_MAX = 150;
constexpr uint32_t DOMINANCE_PERCENT = 120;
constexpr unsigned STABLE_SAMPLES = 3;
}

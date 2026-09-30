#include "lib/ramp.hpp"
using namespace ramp;
constexpr Region along_y{true, 1200, 2400, Axis::Y};
constexpr Region along_x{true, 1200, 2400, Axis::X};
static_assert(along_y.min_x_mm() == 1000 && along_y.max_x_mm() == 1400);
static_assert(along_y.min_y_mm() == 1800 && along_y.max_y_mm() == 3000);
static_assert(along_x.min_x_mm() == 600 && along_x.max_x_mm() == 1800);
static_assert(along_x.min_y_mm() == 2200 && along_x.max_y_mm() == 2600);
constexpr bool geometry(Region r) {
    int edges = 0, interior = 0;
    for (int y = 0; y < 97; ++y) for (int x = 0; x < 48; ++x) {
        edges += r.cell(x, y) == Cell::Edge;
        interior += r.cell(x, y) == Cell::Interior;
    }
    return edges == 48 && interior == 144;
}
static_assert(geometry(along_x) && geometry(along_y));
static_assert(along_y.cell(24, 35) == Cell::Normal && along_y.cell(24, 60) == Cell::Normal);
static_assert(along_x.cell(11, 48) == Cell::Normal && along_x.cell(36, 48) == Cell::Normal);
static_assert(along_y.cell(20, 48) == Cell::Edge && along_y.cell(27, 48) == Cell::Edge);
static_assert(along_x.cell(24, 44) == Cell::Edge && along_x.cell(24, 51) == Cell::Edge);
constexpr Region disabled{false, 1200, 2400, Axis::Y};
static_assert(disabled.cell(20, 48) == Cell::Normal);
// Off-grid centres still produce continuous walls and open ends.
constexpr Region offset{true, 1213, 2427, Axis::Y};
static_assert(offset.cell(offset.x_begin(), 48) == Cell::Edge);
static_assert(offset.cell(offset.x_end() - 1, 48) == Cell::Edge);
static_assert(offset.cell(24, offset.y_end()) == Cell::Normal);
constexpr bool scores() {
    for (int score = 0; score <= 255; ++score) {
        if (constrain_score(Cell::Edge, score, 127) != 255) return false;
        if (constrain_score(Cell::Interior, score, 127) > 127) return false;
        if (constrain_score(Cell::Normal, score, 127) != score) return false;
    }
    uint8_t score = 127;
    for (int i = 0; i < 100; ++i) score = constrain_score(Cell::Interior, score + 16, 127);
    if (score != 127) return false;
    score = constrain_score(Cell::Interior, score - 4, 127);
    if (score != 123) return false;
    return constrain_score(Cell::Interior, score + 16, 127) == 127 &&
           constrain_score(Cell::Normal, 271, 127) == 255;
}
static_assert(scores());

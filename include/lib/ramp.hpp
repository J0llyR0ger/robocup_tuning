#pragma once
#include <cstdint>

namespace ramp {
enum class Axis { X, Y };
enum class Cell { Normal, Interior, Edge };
constexpr int TILE_MM = 50;
constexpr int LENGTH_MM = 1200;
constexpr int WIDTH_MM = 400;
constexpr int floor_cell(int mm) { return mm >= 0 ? mm / TILE_MM : (mm - TILE_MM + 1) / TILE_MM; }
constexpr int ceil_cell(int mm) { return -floor_cell(-mm); }

struct Region {
    bool enabled = false;
    int centre_x_mm = 1200;
    int centre_y_mm = 2400;
    Axis length_axis = Axis::Y;

    constexpr int min_x_mm() const { return centre_x_mm - (length_axis == Axis::X ? LENGTH_MM : WIDTH_MM) / 2; }
    constexpr int max_x_mm() const { return centre_x_mm + (length_axis == Axis::X ? LENGTH_MM : WIDTH_MM) / 2; }
    constexpr int min_y_mm() const { return centre_y_mm - (length_axis == Axis::Y ? LENGTH_MM : WIDTH_MM) / 2; }
    constexpr int max_y_mm() const { return centre_y_mm + (length_axis == Axis::Y ? LENGTH_MM : WIDTH_MM) / 2; }
    constexpr int x_begin() const { return floor_cell(min_x_mm()); }
    constexpr int x_end() const { return ceil_cell(max_x_mm()); }
    constexpr int y_begin() const { return floor_cell(min_y_mm()); }
    constexpr int y_end() const { return ceil_cell(max_y_mm()); }

    constexpr bool operator==(const Region &) const = default;

    constexpr Cell cell(int x, int y) const {
        if (!enabled || x < x_begin() || x >= x_end() || y < y_begin() || y >= y_end())
            return Cell::Normal;
        const bool edge = length_axis == Axis::X
            ? y == y_begin() || y == y_end() - 1
            : x == x_begin() || x == x_end() - 1;
        return edge ? Cell::Edge : Cell::Interior;
    }
    // A ramp clipped by the arena must never clear the arena boundary.
    constexpr Cell grid_cell(int x, int y, int width, int height) const {
        const auto result = cell(x, y);
        if (result != Cell::Normal &&
            (x == 0 || y == 0 || x == width - 1 || y == height - 1)) return Cell::Edge;
        return result;
    }
};

constexpr uint8_t constrain_score(Cell cell, uint16_t score, uint8_t unknown) {
    if (cell == Cell::Edge) return 255;
    const uint16_t ceiling = cell == Cell::Interior ? unknown : 255;
    return static_cast<uint8_t>(score > ceiling ? ceiling : score);
}
}

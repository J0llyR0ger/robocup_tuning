#include "lib/path_smoother.hpp"
#include <cmath>

Eigen::Vector2f PathSmoother::toWorld(uint16_t node) const {
    size_t x = OccupancyGridGraph::xOf(node);
    size_t y = OccupancyGridGraph::yOf(node);
    return Eigen::Vector2f((static_cast<float>(x) + 0.5f) * OccupancyGridMap::TILE_SIZE_METERS,
                           (static_cast<float>(y) + 0.5f) * OccupancyGridMap::TILE_SIZE_METERS);
}

// Bresenham line traversal between two grid nodes, checking every touched cell against
// the graph's block state. This is what makes "line of sight" mean "obstacle-free
// straight line," not just "unoccupied endpoints."
bool PathSmoother::lineOfSightClear(uint16_t a, uint16_t b) const {
    int x0 = static_cast<int>(OccupancyGridGraph::xOf(a));
    int y0 = static_cast<int>(OccupancyGridGraph::yOf(a));
    int x1 = static_cast<int>(OccupancyGridGraph::xOf(b));
    int y1 = static_cast<int>(OccupancyGridGraph::yOf(b));

    int dx = std::abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    int x = x0, y = y0;
    while (true) {
        if (graph.isBlocked(static_cast<size_t>(x), static_cast<size_t>(y)))
            return false;

        // Prevent the line from clipping diagonally past a blocked corner --
        // same corner-cutting rule used in forEachNeighbor, applied along the whole ray.
        if (x != x1 || y != y1) {
            int e2 = 2 * err;
            bool stepX = e2 >= dy;
            bool stepY = e2 <= dx;
            if (stepX && stepY) {
                if (graph.isBlocked(static_cast<size_t>(x + sx), static_cast<size_t>(y)) ||
                    graph.isBlocked(static_cast<size_t>(x), static_cast<size_t>(y + sy))) {
                    return false;
                }
            }
            if (stepX) {
                err += dy;
                x += sx;
            }
            if (stepY) {
                err += dx;
                y += sy;
            }
        } else {
            break;
        }
    }
    return true;
}

Eigen::Vector2f PathSmoother::catmullRom(const Eigen::Vector2f &p0, const Eigen::Vector2f &p1,
                                         const Eigen::Vector2f &p2, const Eigen::Vector2f &p3,
                                         float t) {
    float t2 = t * t;
    float t3 = t2 * t;

    return 0.5f * ((2.0f * p1) + (-p0 + p2) * t + (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t2 +
                   (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t3);
}
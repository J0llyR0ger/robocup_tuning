#pragma once

#include "lib/occupancy_grid_map.hpp"
#include <array>
#include <cstdint>
#include <limits>

class OccupancyGridGraph {
  public:
    static constexpr size_t NUM_NODES =
        OccupancyGridMap::GRID_WIDTH * OccupancyGridMap::GRID_HEIGHT;

    static_assert(NUM_NODES <= 65535, "Grid too large for uint16_t node indices -- widen node "
                                      "index type or reduce grid resolution");

    // Sentinel traversal cost for hard-blocked cells. Large enough that no real path
    // would ever be cheaper through it, but finite -- avoids INFINITY arithmetic/aliasing
    // with the AStarWorkspace "unvisited" sentinel.
    static constexpr float BLOCKED_COST = std::numeric_limits<float>::max() / 2.0f;

    explicit OccupancyGridGraph(const OccupancyGridMap &grid, uint8_t blockThreshold = 220,
                                float costWeight = 4.0f, int inflationRadius = 4,
                                float inflationWeight = 6.0f);

    static constexpr uint16_t idx(size_t x, size_t y) {
        return static_cast<uint16_t>(y * OccupancyGridMap::GRID_WIDTH + x);
    }
    static constexpr size_t xOf(uint16_t node) { return node % OccupancyGridMap::GRID_WIDTH; }
    static constexpr size_t yOf(uint16_t node) { return node / OccupancyGridMap::GRID_WIDTH; }

    template <typename Fn> void forEachNeighbor(uint16_t node, Fn &&fn) const {
        static const int8_t dx8[8] = {1, -1, 0, 0, 1, 1, -1, -1};
        static const int8_t dy8[8] = {0, 0, 1, -1, 1, -1, 1, -1};
        static const float cost8[8] = {1,           1,           1,           1,
                                       1.41421356f, 1.41421356f, 1.41421356f, 1.41421356f};

        int cx = static_cast<int>(xOf(node));
        int cy = static_cast<int>(yOf(node));

        for (int n = 0; n < 8; ++n) {
            int nx = cx + dx8[n];
            int ny = cy + dy8[n];
            if (!inBounds(nx, ny))
                continue;
            uint16_t ni = idx(static_cast<size_t>(nx), static_cast<size_t>(ny));
            if (traversalCost[ni] >= BLOCKED_COST)
                continue;

            // Prevent cutting corners through diagonal gaps
            if (dx8[n] != 0 && dy8[n] != 0) {
                if (!inBounds(cx + dx8[n], cy) || !inBounds(cx, cy + dy8[n]))
                    continue;
                uint16_t orthoA = idx(static_cast<size_t>(cx + dx8[n]), static_cast<size_t>(cy));
                uint16_t orthoB = idx(static_cast<size_t>(cx), static_cast<size_t>(cy + dy8[n]));
                if (traversalCost[orthoA] >= BLOCKED_COST || traversalCost[orthoB] >= BLOCKED_COST)
                    continue;
            }

            fn(ni, cost8[n] * traversalCost[ni]);
        }
    }

  private:
    bool inBounds(int x, int y) const;

    void precompute(const OccupancyGridMap &grid, uint8_t blockThreshold, float costWeight,
                    int inflationRadius, float inflationWeight);

    float nearbyMax(const std::array<uint8_t, NUM_NODES> &scores, int x, int y, int radius) const;

    std::array<float, NUM_NODES> traversalCost{};
};

struct OctileHeuristic {
    size_t goalX, goalY;
    OctileHeuristic(size_t gx, size_t gy) : goalX(gx), goalY(gy) {}

    inline float operator()(uint16_t node) const {
        size_t x = OccupancyGridGraph::xOf(node);
        size_t y = OccupancyGridGraph::yOf(node);
        float dx = std::fabs(static_cast<float>(goalX) - static_cast<float>(x));
        float dy = std::fabs(static_cast<float>(goalY) - static_cast<float>(y));
        constexpr float D = 1.0f, D2 = 1.41421356f;
        return D * (dx + dy) + (D2 - 2 * D) * std::min(dx, dy);
    }
};
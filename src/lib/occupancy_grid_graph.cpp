#include "lib/occupancy_grid_graph.hpp"
#include <algorithm>
#include <cmath>

OccupancyGridGraph::OccupancyGridGraph(const OccupancyGridMap &grid, uint8_t blockThreshold,
                                       float costWeight, int inflationRadius,
                                       float inflationWeight) {
    precompute(grid, blockThreshold, costWeight, inflationRadius, inflationWeight);
}

bool OccupancyGridGraph::inBounds(int x, int y) const {
    return x >= 0 && static_cast<size_t>(x) < OccupancyGridMap::GRID_WIDTH && y >= 0 &&
           static_cast<size_t>(y) < OccupancyGridMap::GRID_HEIGHT;
}

void OccupancyGridGraph::precompute(const OccupancyGridMap &grid, uint8_t blockThreshold,
                                    float costWeight, int inflationRadius, float inflationWeight) {
    const auto &scores = grid.get_scores();

    for (size_t y = 0; y < OccupancyGridMap::GRID_HEIGHT; ++y) {
        for (size_t x = 0; x < OccupancyGridMap::GRID_WIDTH; ++x) {
            uint16_t node = idx(x, y);
            uint8_t score = scores[node];

            if (score >= blockThreshold) {
                traversalCost[node] = BLOCKED_COST;
                continue;
            }

            float occFrac = static_cast<float>(score) / 255.0f;
            float proximity =
                nearbyMax(scores, static_cast<int>(x), static_cast<int>(y), inflationRadius);

            traversalCost[node] = 1.0f + costWeight * occFrac + inflationWeight * proximity;
        }
    }
}

float OccupancyGridGraph::nearbyMax(const std::array<uint8_t, NUM_NODES> &scores, int x, int y,
                                    int radius) const {
    float worst = 0.0f;
    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            if (dx == 0 && dy == 0)
                continue;
            int sx = x + dx;
            int sy = y + dy;
            if (!inBounds(sx, sy))
                continue;

            float dist = std::sqrt(static_cast<float>(dx * dx + dy * dy));
            if (dist > static_cast<float>(radius))
                continue; // circular window

            uint8_t s = scores[idx(static_cast<size_t>(sx), static_cast<size_t>(sy))];
            float occFrac = static_cast<float>(s) / 255.0f;
            float falloff = 1.0f - (dist / static_cast<float>(radius));
            worst = std::max(worst, occFrac * falloff);
        }
    }
    return worst;
}
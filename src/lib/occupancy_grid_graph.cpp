#include "lib/occupancy_grid_graph.hpp"
#include <algorithm>
#include <cmath>

OccupancyGridGraph::OccupancyGridGraph(const OccupancyGridMap &grid, uint8_t blockThreshold,
                                       float costWeight, int inflationRadius,
                                       float inflationWeight) {
    buildKernel(inflationRadius);
    precompute(grid, blockThreshold, costWeight, inflationWeight);
}

bool OccupancyGridGraph::inBounds(int x, int y) const {
    return x >= 0 && static_cast<size_t>(x) < OccupancyGridMap::GRID_WIDTH && y >= 0 &&
           static_cast<size_t>(y) < OccupancyGridMap::GRID_HEIGHT;
}

void OccupancyGridGraph::buildKernel(int radius) {
    kernel.clear();
    radius = std::min(radius, MAX_INFLATION_RADIUS);

    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            if (dx == 0 && dy == 0)
                continue;

            float dist = std::sqrt(static_cast<float>(dx * dx + dy * dy)); // computed once, ever
            if (dist > static_cast<float>(radius))
                continue; // circular window

            float falloff = 1.0f - (dist / static_cast<float>(radius));
            kernel.push_back({static_cast<int8_t>(dx), static_cast<int8_t>(dy), falloff});
        }
    }

    // Sort descending by weight so nearbyMax can break early: once the running best
    // is >= the next entry's weight, no later entry (occFrac <= 1) can beat it.
    std::sort(kernel.begin(), kernel.end(),
              [](const KernelEntry &a, const KernelEntry &b) { return a.weight > b.weight; });
}

void OccupancyGridGraph::precompute(const OccupancyGridMap &grid, uint8_t blockThreshold,
                                    float costWeight, float inflationWeight) {
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
            float proximity = nearbyMax(scores, static_cast<int>(x), static_cast<int>(y));

            traversalCost[node] = 1.0f + costWeight * occFrac + inflationWeight * proximity;
        }
    }
}

float OccupancyGridGraph::nearbyMax(const std::array<uint8_t, NUM_NODES> &scores, int x,
                                    int y) const {
    float worst = 0.0f;
    for (const auto &k : kernel) {
        // Early exit: kernel is sorted descending by weight, and occFrac <= 1, so no
        // later entry can produce a value exceeding its own weight.
        if (k.weight <= worst)
            break;

        int sx = x + k.dx;
        int sy = y + k.dy;
        if (!inBounds(sx, sy))
            continue;

        uint8_t s = scores[idx(static_cast<size_t>(sx), static_cast<size_t>(sy))];
        float occFrac = static_cast<float>(s) / 255.0f;
        worst = std::max(worst, occFrac * k.weight);
    }

    return worst;
}
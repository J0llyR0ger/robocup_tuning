#pragma once

#include "etl/vector.h"
#include "lib/occupancy_grid_graph.hpp"
#include "lib/occupancy_grid_map.hpp"
#include <Eigen/Geometry>
#include <cstddef>
#include <cstdint>

class PathSmoother {
  public:
    explicit PathSmoother(const OccupancyGridGraph &graph) : graph(graph) {}

    // Stage 1: reduce a dense grid-cell path to the minimal set of waypoints connectable
    // by straight, obstacle-free line segments. Greedy: from each anchor, find the
    // farthest point still in line-of-sight, jump there, repeat.
    template <size_t N>
    etl::vector<uint16_t, N> simplify(const etl::vector<uint16_t, N> &rawPath) const {
        etl::vector<uint16_t, N> waypoints;
        if (rawPath.empty())
            return waypoints;

        waypoints.push_back(rawPath[0]);
        size_t anchor = 0;

        while (anchor < rawPath.size() - 1) {
            size_t farthest = anchor + 1;
            for (size_t candidate = rawPath.size() - 1; candidate > anchor; --candidate) {
                if (lineOfSightClear(rawPath[anchor], rawPath[candidate])) {
                    farthest = candidate;
                    break;
                }
            }
            waypoints.push_back(rawPath[farthest]);
            anchor = farthest;
        }
        return waypoints;
    }

    // Stage 2: fit a Catmull-Rom spline through the waypoints and sample it densely,
    // producing a smooth world-space path with continuous curvature at each waypoint.
    template <size_t N, size_t M>
    void smooth(const etl::vector<uint16_t, N> &waypoints, etl::vector<Eigen::Vector2f, M> &outPath,
                int samplesPerSegment = 8) const {
        outPath.clear();
        if (waypoints.empty())
            return;
        if (waypoints.size() == 1) {
            outPath.push_back(toWorld(waypoints[0]));
            return;
        }

        int segmentCount = static_cast<int>(waypoints.size()) - 1;
        for (int i = 0; i < segmentCount; ++i) {
            Eigen::Vector2f p0 = worldAt(waypoints, i - 1);
            Eigen::Vector2f p1 = worldAt(waypoints, i);
            Eigen::Vector2f p2 = worldAt(waypoints, i + 1);
            Eigen::Vector2f p3 = worldAt(waypoints, i + 2);

            // Include the final sample (t=1) only on the last segment, so shared
            // waypoints between segments aren't duplicated in the output.
            int samples = (i == segmentCount - 1) ? samplesPerSegment + 1 : samplesPerSegment;

            for (int s = 0; s < samples; ++s) {
                if (outPath.full())
                    return;
                float t = static_cast<float>(s) / static_cast<float>(samplesPerSegment);
                outPath.push_back(catmullRom(p0, p1, p2, p3, t));
            }
        }
    }

  private:
    template <size_t N>
    Eigen::Vector2f worldAt(const etl::vector<uint16_t, N> &waypoints, int i) const {
        int clamped = i < 0 ? 0
                            : (i >= static_cast<int>(waypoints.size())
                                   ? static_cast<int>(waypoints.size()) - 1
                                   : i);
        return toWorld(waypoints[static_cast<size_t>(clamped)]);
    }

    Eigen::Vector2f toWorld(uint16_t node) const;
    bool lineOfSightClear(uint16_t a, uint16_t b) const;
    static Eigen::Vector2f catmullRom(const Eigen::Vector2f &p0, const Eigen::Vector2f &p1,
                                      const Eigen::Vector2f &p2, const Eigen::Vector2f &p3,
                                      float t);

    const OccupancyGridGraph &graph;
};
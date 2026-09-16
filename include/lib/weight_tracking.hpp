#pragma once

#undef B1
#include "Eigen/Geometry"
#include "lidar_processing.hpp"

struct WeightTrackedTarget {
    Cluster cluster{};
    float confidence = 0.0f;
    float last_observation_score = 0.0f;

    uint16_t missed_updates = 0;
    bool matched_this_update = false;
};

class WeightTracking {
  public:
    void update_from_clusters(ClusterList clusters);

    std::span<WeightTrackedTarget> get_tracked_weights();

  private:
    etl::vector<WeightTrackedTarget, WEIGHT_TARGET_MAX_TRACKS> tracked_targets;
};
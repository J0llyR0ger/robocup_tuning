#include <algorithm>
#include <cmath>

#include "config.hpp"
#include "lib/weight_tracking.hpp"

namespace {

float score_grid_cluster(const OccupancyGridMap::WeightCluster &cluster) {
    if (cluster.cells.empty() || cluster.cells.size() > OccupancyGridMap::MAX_WEIGHT_CLUSTER_SIZE) {
        return 0.0f;
    }

    return std::clamp(static_cast<float>(cluster.cells.size()) /
                          static_cast<float>(OccupancyGridMap::MAX_WEIGHT_CLUSTER_SIZE),
                      0.0f, 1.0f);
}

bool has_empty_radius_for_cluster(const OccupancyGridMap &grid,
                                  const OccupancyGridMap::WeightCluster &cluster, int radius) {
    for (const auto &cell : cluster.cells) {
        for (int dy = -radius; dy <= radius; ++dy) {
            for (int dx = -radius; dx <= radius; ++dx) {
                const int x = cell.x + dx;
                const int y = cell.y + dy;
                if (dx == 0 && dy == 0) {
                    continue;
                }

                int grid_x = 0;
                int grid_y = 0;
                if (!grid.world_to_grid(
                        Eigen::Vector2f(
                            (static_cast<float>(x) + 0.5f) * OccupancyGridMap::TILE_SIZE_METERS,
                            (static_cast<float>(y) + 0.5f) * OccupancyGridMap::TILE_SIZE_METERS),
                        grid_x, grid_y)) {
                    continue;
                }

                const float dist_sq = static_cast<float>(dx * dx + dy * dy);
                if (dist_sq <= static_cast<float>(radius * radius) &&
                    grid.get_score(static_cast<size_t>(grid_x), static_cast<size_t>(grid_y)) >=
                        OccupancyGridMap::WEIGHT_CLUSTER_OCCUPIED_THRESHOLD) {
                    return false;
                }
            }
        }
    }

    return true;
}

} // namespace

float score_cluster_for_weight_target(const Cluster &cluster) {
    if (cluster.count == 0 || cluster.count > WEIGHT_TARGET_MAX_CLUSTER_COUNT) {
        return 0.0f;
    }

    return std::clamp(static_cast<float>(cluster.count) /
                          static_cast<float>(WEIGHT_TARGET_MAX_CLUSTER_COUNT),
                      0.0f, 1.0f);
}

void apply_cluster_to_track(WeightTrackedTarget &track, const Cluster &observation,
                            float observation_score, float position_alpha) {
    const Eigen::Vector2f previous_centroid = track.cluster.centroid;
    track.cluster = observation;
    track.cluster.centroid =
        (1.0f - position_alpha) * previous_centroid + position_alpha * observation.centroid;
    track.last_observation_score = observation_score;
}

std::span<WeightTrackedTarget> WeightTracking::get_tracked_weights() {
    return this->tracked_targets;
}

void WeightTracking::update_from_clusters(ClusterList clusters) {
    for (auto &track : this->tracked_targets) {
        track.matched_this_update = false;
    }

    for (const auto &cluster : clusters) {
        const float observation_score = score_cluster_for_weight_target(cluster);

        if (observation_score < WEIGHT_TARGET_MIN_TRACK_CANDIDATE_SCORE) {
            continue;
        }

        int best_track_index = -1;
        float best_track_distance = WEIGHT_TARGET_TRACK_ASSOCIATION_DISTANCE_M;

        for (size_t track_index = 0; track_index < this->tracked_targets.size(); ++track_index) {
            const auto &track = this->tracked_targets[track_index];

            const float distance = (cluster.centroid - track.cluster.centroid).norm();

            if (distance <= best_track_distance) {
                best_track_distance = distance;
                best_track_index = static_cast<int>(track_index);
            }
        }

        if (best_track_index >= 0) {
            auto &track = this->tracked_targets[best_track_index];
            const float observation_gain = WEIGHT_TARGET_TRACK_CONFIDENCE_GAIN * observation_score;

            apply_cluster_to_track(track, cluster, observation_score,
                                   WEIGHT_TARGET_TRACK_POSITION_ALPHA);
            track.matched_this_update = true;
            track.missed_updates = 0;
            track.confidence = std::clamp(track.confidence + observation_gain, 0.0f, 1.0f);
            continue;
        }

        if (this->tracked_targets.full()) {
            continue;
        }

        WeightTrackedTarget new_track{};
        new_track.cluster = cluster;
        new_track.confidence = std::clamp(
            observation_score * WEIGHT_TARGET_TRACK_INITIAL_CONFIDENCE_SCALE, 0.0f, 1.0f);
        new_track.last_observation_score = observation_score;
        new_track.missed_updates = 0;
        new_track.matched_this_update = true;
        this->tracked_targets.push_back(new_track);
    }

    for (auto &track : this->tracked_targets) {
        if (!track.matched_this_update) {
            track.missed_updates += 1;
            track.confidence =
                std::max(0.0f, track.confidence - WEIGHT_TARGET_TRACK_CONFIDENCE_DECAY);
        }
    }

    for (size_t index = 0; index < this->tracked_targets.size();) {
        const auto &track = this->tracked_targets[index];
        if (track.confidence <= WEIGHT_TARGET_TRACK_FORGET_CONFIDENCE ||
            track.missed_updates > WEIGHT_TARGET_TRACK_MAX_MISSED_UPDATES) {
            this->tracked_targets.erase(this->tracked_targets.begin() + index);
        } else {
            index++;
        }
    }
}

void WeightTracking::update_from_occupancy_grid(const OccupancyGridMap &grid) {
    const auto clusters = grid.find_weight_clusters();

    ClusterList lidar_clusters;
    for (const auto &cluster : clusters) {
        Cluster observation{};
        observation.count = static_cast<size_t>(cluster.cells.size());
        observation.centroid = Eigen::Vector2f(cluster.centroid.x(), cluster.centroid.y());
        observation.range = cluster.centroid.norm();
        observation.spread = 1.0f;
        observation.max_extent = static_cast<float>(cluster.cells.size());
        observation.diameter_mm = static_cast<float>(cluster.cells.size() * 50);
        observation.aspect_ratio = 1.0f;
        lidar_clusters.push_back(observation);
    }

    this->update_from_clusters(lidar_clusters);
}
#include <algorithm>
#include <cmath>

#include "config.hpp"
#include "lib/weight_tracking.hpp"

void apply_cluster_to_track(WeightTrackedTarget &track, const Cluster &observation,
                            float observation_score, float position_alpha) {
    const Eigen::Vector2f previous_centroid = track.cluster.centroid;
    track.cluster = observation;
    track.cluster.centroid =
        (1.0f - position_alpha) * previous_centroid + position_alpha * observation.centroid;
    track.last_observation_score = observation_score;
}

float interpolate_weight_tuning_alpha(float range_m, float near_range_m, float far_range_m) {
    const float span = far_range_m - near_range_m;
    if (span <= 1e-6f) {
        return 0.0f;
    }

    return std::clamp((range_m - near_range_m) / span, 0.0f, 1.0f);
}

float lerp_weight_tuning(float near_value, float far_value, float alpha) {
    return near_value + (far_value - near_value) * alpha;
}

WeightScoreMetricTuning interpolate_metric_tuning(const WeightScoreMetricTuning &near_metric,
                                                  const WeightScoreMetricTuning &far_metric,
                                                  float alpha) {
    return {
        .desired = lerp_weight_tuning(near_metric.desired, far_metric.desired, alpha),
        .deviation = lerp_weight_tuning(near_metric.deviation, far_metric.deviation, alpha),
        .weight = lerp_weight_tuning(near_metric.weight, far_metric.weight, alpha),
    };
}

WeightScoreProfileTuning interpolate_weight_score_profile(float range_m) {
    const auto &first_sample = WEIGHT_TARGET_RANGE_TUNING.front();
    const auto &last_sample = WEIGHT_TARGET_RANGE_TUNING.back();

    if (range_m <= first_sample.range_m) {
        return first_sample.profile;
    }

    if (range_m >= last_sample.range_m) {
        return last_sample.profile;
    }

    for (size_t i = 0; i + 1 < WEIGHT_TARGET_RANGE_TUNING.size(); ++i) {
        const auto &near_sample = WEIGHT_TARGET_RANGE_TUNING[i];
        const auto &far_sample = WEIGHT_TARGET_RANGE_TUNING[i + 1];

        if (range_m <= far_sample.range_m) {
            const float alpha =
                interpolate_weight_tuning_alpha(range_m, near_sample.range_m, far_sample.range_m);

            return {
                .spread = interpolate_metric_tuning(near_sample.profile.spread,
                                                    far_sample.profile.spread, alpha),
                .extent = interpolate_metric_tuning(near_sample.profile.extent,
                                                    far_sample.profile.extent, alpha),
                .diameter_mm = interpolate_metric_tuning(near_sample.profile.diameter_mm,
                                                         far_sample.profile.diameter_mm, alpha),
                .aspect_ratio = interpolate_metric_tuning(near_sample.profile.aspect_ratio,
                                                          far_sample.profile.aspect_ratio, alpha),
            };
        }
    }

    return last_sample.profile;
}

float score_metric(float value, const WeightScoreMetricTuning &metric) {
    const float safe_stddev = std::max(metric.deviation, 1e-4f);
    const float z_score = (value - metric.desired) / safe_stddev;

    // Normal-distribution likelihood with peak 1.0 at desired value.
    return std::exp(-0.5f * z_score * z_score);
}

float score_cluster_for_weight_target(const Cluster &cluster) {
    if (cluster.count < WEIGHT_TARGET_MIN_CLUSTER_COUNT ||
        cluster.count > WEIGHT_TARGET_MAX_CLUSTER_COUNT) {
        return 0.0f;
    }

    const WeightScoreProfileTuning profile = interpolate_weight_score_profile(cluster.range);

    const float spread_score = score_metric(cluster.spread, profile.spread);
    const float extent_score = score_metric(cluster.max_extent, profile.extent);
    const float diameter_score = score_metric(cluster.diameter_mm, profile.diameter_mm);
    const float aspect_score = score_metric(cluster.aspect_ratio, profile.aspect_ratio);

    const float weight_sum = std::max(profile.spread.weight + profile.extent.weight +
                                          profile.diameter_mm.weight + profile.aspect_ratio.weight,
                                      1e-6f);

    return (spread_score * profile.spread.weight + extent_score * profile.extent.weight +
            diameter_score * profile.diameter_mm.weight +
            aspect_score * profile.aspect_ratio.weight) /
           weight_sum;
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
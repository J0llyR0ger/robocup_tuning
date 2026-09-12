#pragma once

#include <algorithm>
#include <cstdint>

struct WeightScoreMetricTuning {
    float desired = 0.0f;
    float deviation = 1.0f;
    float weight = 0.0f;
};

struct WeightScoreProfileTuning {
    WeightScoreMetricTuning spread;
    WeightScoreMetricTuning extent;
    WeightScoreMetricTuning diameter_mm;
    WeightScoreMetricTuning aspect_ratio;
};

struct WeightRangeTuning {
    float near_range_m = 0.0f;
    float far_range_m = 1.0f;

    WeightScoreProfileTuning near_profile;
    WeightScoreProfileTuning far_profile;
};

// Weight Tracking Tuning
static constexpr float WEIGHT_TARGET_MIN_SCORE = 0.5f;
static constexpr float WEIGHT_TARGET_MIN_TRACK_CANDIDATE_SCORE = 0.30f;

static constexpr size_t WEIGHT_TARGET_MAX_TRACKS = 8;
static constexpr float WEIGHT_TARGET_TRACK_ASSOCIATION_DISTANCE_M = 0.12f;
static constexpr float WEIGHT_TARGET_TRACK_POSITION_ALPHA = 0.30f;
static constexpr float WEIGHT_TARGET_TRACK_CONFIDENCE_GAIN = 0.12f;
static constexpr float WEIGHT_TARGET_TRACK_INITIAL_CONFIDENCE_SCALE = 0.60f;
static constexpr float WEIGHT_TARGET_TRACK_CONFIDENCE_DECAY = 0.04f;
static constexpr float WEIGHT_TARGET_TRACK_FORGET_CONFIDENCE = 0.05f;
static constexpr float WEIGHT_TARGET_TRACK_PUBLISH_CONFIDENCE = 0.40f;
static constexpr uint16_t WEIGHT_TARGET_TRACK_MAX_MISSED_UPDATES = 20;

// Cluster Identification Tuning
static constexpr int WEIGHT_TARGET_MIN_CLUSTER_COUNT = 4;
static constexpr int WEIGHT_TARGET_MAX_CLUSTER_COUNT = 120;
static constexpr WeightRangeTuning WEIGHT_TARGET_RANGE_TUNING = {
    .near_range_m = 0.05f,
    .far_range_m = 0.63f,
    .near_profile =
        {
            .spread = {.desired = 0.01f, .deviation = 0.005f, .weight = 0.30f},
            .extent = {.desired = 0.034f, .deviation = 0.010f, .weight = 0.20f},
            .diameter_mm = {.desired = 30.0f, .deviation = 15.0f, .weight = 1.0f},
            .aspect_ratio = {.desired = 1.5f, .deviation = 1.0f, .weight = 0.15f},
        },
    .far_profile =
        {
            .spread = {.desired = 0.015f, .deviation = 0.01f, .weight = 0.30f},
            .extent = {.desired = 0.03f, .deviation = 0.01f, .weight = 0.20f},
            .diameter_mm = {.desired = 30.0f, .deviation = 10.0f, .weight = 1.0f},
            .aspect_ratio = {.desired = 1.15f, .deviation = 0.5f, .weight = 0.15f},
        },
};

inline float interpolate_weight_tuning_alpha(float range_m) {
    const float span =
        WEIGHT_TARGET_RANGE_TUNING.far_range_m - WEIGHT_TARGET_RANGE_TUNING.near_range_m;
    if (span <= 1e-6f) {
        return 0.0f;
    }

    return std::clamp((range_m - WEIGHT_TARGET_RANGE_TUNING.near_range_m) / span, 0.0f, 1.0f);
}

inline float lerp_weight_tuning(float near_value, float far_value, float alpha) {
    return near_value + (far_value - near_value) * alpha;
}

inline WeightScoreMetricTuning interpolate_metric_tuning(const WeightScoreMetricTuning &near_metric,
                                                         const WeightScoreMetricTuning &far_metric,
                                                         float alpha) {
    return {
        .desired = lerp_weight_tuning(near_metric.desired, far_metric.desired, alpha),
        .deviation = lerp_weight_tuning(near_metric.deviation, far_metric.deviation, alpha),
        .weight = lerp_weight_tuning(near_metric.weight, far_metric.weight, alpha),
    };
}

inline WeightScoreProfileTuning interpolate_weight_score_profile(float range_m) {
    const float alpha = interpolate_weight_tuning_alpha(range_m);

    return {
        .spread = interpolate_metric_tuning(WEIGHT_TARGET_RANGE_TUNING.near_profile.spread,
                                            WEIGHT_TARGET_RANGE_TUNING.far_profile.spread, alpha),
        .extent = interpolate_metric_tuning(WEIGHT_TARGET_RANGE_TUNING.near_profile.extent,
                                            WEIGHT_TARGET_RANGE_TUNING.far_profile.extent, alpha),
        .diameter_mm =
            interpolate_metric_tuning(WEIGHT_TARGET_RANGE_TUNING.near_profile.diameter_mm,
                                      WEIGHT_TARGET_RANGE_TUNING.far_profile.diameter_mm, alpha),
        .aspect_ratio =
            interpolate_metric_tuning(WEIGHT_TARGET_RANGE_TUNING.near_profile.aspect_ratio,
                                      WEIGHT_TARGET_RANGE_TUNING.far_profile.aspect_ratio, alpha),
    };
}

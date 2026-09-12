#pragma once

#include <algorithm>
#include <array>
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
    float range_m = 0.0f;
    WeightScoreProfileTuning profile;
};

// Weight Tracking Tuning
static constexpr float WEIGHT_TARGET_MIN_TRACK_CANDIDATE_SCORE = 0.30f;

static constexpr size_t WEIGHT_TARGET_MAX_TRACKS = 8;
static constexpr float WEIGHT_TARGET_TRACK_ASSOCIATION_DISTANCE_M = 0.12f;
static constexpr float WEIGHT_TARGET_TRACK_POSITION_ALPHA = 0.30f;
static constexpr float WEIGHT_TARGET_TRACK_CONFIDENCE_GAIN = 0.12f;
static constexpr float WEIGHT_TARGET_TRACK_INITIAL_CONFIDENCE_SCALE = 0.60f;
static constexpr float WEIGHT_TARGET_TRACK_CONFIDENCE_DECAY = 0.04f;
static constexpr float WEIGHT_TARGET_TRACK_FORGET_CONFIDENCE = 0.05f;
static constexpr uint16_t WEIGHT_TARGET_TRACK_MAX_MISSED_UPDATES = 20;

// Cluster Identification Tuning
static constexpr int WEIGHT_TARGET_MIN_CLUSTER_COUNT = 4;
static constexpr int WEIGHT_TARGET_MAX_CLUSTER_COUNT = 120;

static constexpr std::array<WeightRangeTuning, 5> WEIGHT_TARGET_RANGE_TUNING = {
    WeightRangeTuning{
        .range_m = 0.05f,
        .profile =
            {
                .spread = {.desired = 0.0034f, .deviation = 0.0015f, .weight = 0.30f},
                .extent = {.desired = 0.0106f, .deviation = 0.0063f, .weight = 0.20f},
                .diameter_mm = {.desired = 10.6f, .deviation = 10.0f, .weight = 0.6f},
                .aspect_ratio = {.desired = 1.66f, .deviation = 0.721f, .weight = 0.15f},
            },
    },
    WeightRangeTuning{
        .range_m = 0.252f,
        .profile =
            {
                .spread = {.desired = 0.0150f, .deviation = 0.0027f, .weight = 0.30f},
                .extent = {.desired = 0.0449f, .deviation = 0.0096f, .weight = 0.20f},
                .diameter_mm = {.desired = 35.0f, .deviation = 10.0f, .weight = 0.6f},
                .aspect_ratio = {.desired = 2.068f, .deviation = 0.497f, .weight = 0.15f},
            },
    },
    WeightRangeTuning{
        .range_m = 0.430,
        .profile =
            {
                .spread = {.desired = 0.0152f, .deviation = 0.0013f, .weight = 0.30f},
                .extent = {.desired = 0.0433f, .deviation = 0.0040f, .weight = 0.20f},
                .diameter_mm = {.desired = 40.0f, .deviation = 10.0f, .weight = 0.6f},
                .aspect_ratio = {.desired = 2.295f, .deviation = 0.681f, .weight = 0.15f},
            },
    },
    WeightRangeTuning{
        .range_m = 0.643,
        .profile =
            {
                .spread = {.desired = 0.0114f, .deviation = 0.0016f, .weight = 0.30f},
                .extent = {.desired = 0.0294f, .deviation = 0.005f, .weight = 0.20f},
                .diameter_mm = {.desired = 29.39f, .deviation = 10.0f, .weight = 0.6f},
                .aspect_ratio = {.desired = 2.834f, .deviation = 1.349f, .weight = 0.15f},
            },
    },
    WeightRangeTuning{
        .range_m = 0.845f,
        .profile =
            {
                .spread = {.desired = 0.0119f, .deviation = 0.0019f, .weight = 0.30f},
                .extent = {.desired = 0.0246f, .deviation = 0.0048f, .weight = 0.20f},
                .diameter_mm = {.desired = 24.56f, .deviation = 4.84f, .weight = 0.6f},
                .aspect_ratio = {.desired = 1.941f, .deviation = 2.162f, .weight = 0.15f},
            },
    },
};

inline float interpolate_weight_tuning_alpha(float range_m, float near_range_m, float far_range_m) {
    const float span = far_range_m - near_range_m;
    if (span <= 1e-6f) {
        return 0.0f;
    }

    return std::clamp((range_m - near_range_m) / span, 0.0f, 1.0f);
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

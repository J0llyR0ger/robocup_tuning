#pragma once

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

// Minimum score required to start tracking cluster
static constexpr float WEIGHT_TARGET_MIN_TRACK_CANDIDATE_SCORE = 0.30f;

// Maximum number of trackable weights
static constexpr int WEIGHT_TARGET_MAX_TRACKS = 8;

// Max allowable target position noise
// TODO: Lower this, its way to big
static constexpr float WEIGHT_TARGET_TRACK_ASSOCIATION_DISTANCE_M = 0.12f;

// Position update aggressiveness
static constexpr float WEIGHT_TARGET_TRACK_POSITION_ALPHA = 0.30f;

static constexpr float WEIGHT_TARGET_TRACK_CONFIDENCE_GAIN = 0.12f;
static constexpr float WEIGHT_TARGET_TRACK_INITIAL_CONFIDENCE_SCALE = 0.60f;
static constexpr float WEIGHT_TARGET_TRACK_CONFIDENCE_DECAY = 0.04f;
static constexpr float WEIGHT_TARGET_TRACK_FORGET_CONFIDENCE = 0.05f;
static constexpr int WEIGHT_TARGET_TRACK_MAX_MISSED_UPDATES = 20;

// Cluster Identification Tuning
static constexpr int WEIGHT_TARGET_MIN_CLUSTER_COUNT = 1;
static constexpr int WEIGHT_TARGET_MAX_CLUSTER_COUNT = 4;
static constexpr uint8_t WEIGHT_TARGET_OCCUPIED_TILE_CONFIDENCE = 200;
static constexpr int WEIGHT_TARGET_MIN_EMPTY_RADIUS = 2;

static constexpr std::array<WeightRangeTuning, 5> WEIGHT_TARGET_RANGE_TUNING = {
    WeightRangeTuning{
        .range_m = 0.05f,
        .profile =
            {
                .spread = {.desired = 0.0095f, .deviation = 0.0017f, .weight = 0.30f},
                .extent = {.desired = 0.0283f, .deviation = 0.0063f, .weight = 0.20f},
                .diameter_mm = {.desired = 28.33f, .deviation = 6.28f, .weight = 0.6f},
                .aspect_ratio = {.desired = 1.73f, .deviation = 0.89f, .weight = 0.15f},
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
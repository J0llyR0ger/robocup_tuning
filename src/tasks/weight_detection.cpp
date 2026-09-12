#include "tasks/weight_detection.hpp"

#include "telemetry_bus.hpp"
#include "weight_tuning_config.hpp"

#include <algorithm>
#include <cmath>
#include <etl/vector.h>
#include <limits>

struct WeightClusterSummary {
    Eigen::Vector2f centroid = Eigen::Vector2f::Zero();
    float range = 0.0f;
    float spread = 0.0f;
    float max_extent = 0.0f;
    float diameter_mm = 0.0f;
    float aspect_ratio = 1.0f;
    float score = 0.0f;

    int count = 0;
};

void apply_summary_to_track(WeightTrackedTarget &track, const WeightClusterSummary &summary,
                            float position_alpha) {
    track.position = (1.0f - position_alpha) * track.position + position_alpha * summary.centroid;
    track.last_observation_score = summary.score;
    track.spread = summary.spread;
    track.max_extent = summary.max_extent;
    track.diameter_mm = summary.diameter_mm;
    track.aspect_ratio = summary.aspect_ratio;
    track.range = summary.range;
    track.count = summary.count;
}

float score_metric(float value, const WeightScoreMetricTuning &metric) {
    const float safe_deviation = std::max(metric.deviation, 1e-4f);
    return std::clamp(1.0f - std::fabs(value - metric.desired) / safe_deviation, 0.0f, 1.0f);
}

WeightClusterSummary summarize_weight_cluster(std::span<const LidarResponsePoint> cluster) {
    WeightClusterSummary summary;

    if (cluster.empty()) {
        return summary;
    }

    summary.count = cluster.size();

    Eigen::Vector2f centroid = Eigen::Vector2f::Zero();
    float range = 0.0;
    for (const auto &point : cluster) {
        centroid += point.position;
        range += point.range;
    }

    centroid /= (float)cluster.size();
    range /= (float)cluster.size();

    summary.centroid = centroid;
    summary.range = range;

    float min_x = INFINITY;
    float max_x = -INFINITY;
    float min_y = INFINITY;
    float max_y = -INFINITY;

    float sum_squared_distance = 0.0f;

    for (const auto &point : cluster) {
        const Eigen::Vector2f delta = point.position - centroid;

        sum_squared_distance += delta.squaredNorm();

        min_x = std::min(min_x, point.position.x());
        max_x = std::max(max_x, point.position.x());
        min_y = std::min(min_y, point.position.y());
        max_y = std::max(max_y, point.position.y());
    }

    summary.spread = std::sqrt(sum_squared_distance / (float)cluster.size());

    summary.max_extent = std::max(max_x - min_x, max_y - min_y);

    summary.diameter_mm = summary.max_extent * 1000.0f;

    const float minor_extent = std::min(max_x - min_x, max_y - min_y);
    summary.aspect_ratio = minor_extent > 1e-3f ? (summary.max_extent / minor_extent) : 1.0f;

    if (summary.count < WEIGHT_TARGET_MIN_CLUSTER_COUNT ||
        summary.count > WEIGHT_TARGET_MAX_CLUSTER_COUNT) {
        summary.score = 0.0f;
        return summary;
    }

    const WeightScoreProfileTuning profile = interpolate_weight_score_profile(summary.range);

    const float spread_score = score_metric(summary.spread, profile.spread);
    const float extent_score = score_metric(summary.max_extent, profile.extent);
    const float diameter_score = score_metric(summary.diameter_mm, profile.diameter_mm);
    const float aspect_score = score_metric(summary.aspect_ratio, profile.aspect_ratio);

    const float weight_sum = std::max(profile.spread.weight + profile.extent.weight +
                                          profile.diameter_mm.weight + profile.aspect_ratio.weight,
                                      1e-6f);

    summary.score =
        (spread_score * profile.spread.weight + extent_score * profile.extent.weight +
         diameter_score * profile.diameter_mm.weight + aspect_score * profile.aspect_ratio.weight) /
        weight_sum;

    return summary;
}

WeightDetectionTask::WeightDetectionTask(LidarProcessingTask *lidar_processing_task)
    : SchedulerTask("weight_detection"), lidar_processing_task(lidar_processing_task) {}

void WeightDetectionTask::setup() {
    if (!expander.begin(0x3E)) {
        Serial.println("Failed to communicate with SX1509. Check wiring!");
    }

    expander.pinMode(ENTRY_CONDUCTION_PIN, INPUT);
    expander.pinMode(ENTRY_SWITCH_PIN, INPUT);

    expander.debouncePin(ENTRY_CONDUCTION_PIN);
    expander.debouncePin(ENTRY_SWITCH_PIN);

    expander.debounceTime(WEIGHT_DETECTION_DEBOUNCE_MS);
}

const int IO_EXPANDER_ADDRESS = 0x3E;
const int PIN_INPUT_STATE_ADDRESS = 0x10;

uint16_t readPins() {
    // Manually read the pins instead of using the expander library, as the library uses redundant
    // multiple tranmissions when reading more than 1 pin

    Wire.beginTransmission(IO_EXPANDER_ADDRESS);
    Wire.write(PIN_INPUT_STATE_ADDRESS);
    Wire.endTransmission();
    Wire.requestFrom(IO_EXPANDER_ADDRESS, (uint8_t)2);

    uint16_t msb = (Wire.read() & 0x00FF) << 8;
    uint16_t lsb = (Wire.read() & 0x00FF);
    uint16_t readValue = msb | lsb;

    return msb | lsb;
}

void WeightDetectionTask::update_weight_target() {
    this->current_weight_target = WeightTarget{};

    auto clear_weight_target_telemetry = []() {
        telemetry::publish_f32(telemetry::KEY_WEIGHT_TARGET_X,
                               std::numeric_limits<float>::quiet_NaN());
        telemetry::publish_f32(telemetry::KEY_WEIGHT_TARGET_Y,
                               std::numeric_limits<float>::quiet_NaN());
        telemetry::publish_f32(telemetry::KEY_WEIGHT_TARGET_CONFIDENCE,
                               std::numeric_limits<float>::quiet_NaN());
        telemetry::publish_f32(telemetry::KEY_WEIGHT_SPREAD,
                               std::numeric_limits<float>::quiet_NaN());
        telemetry::publish_f32(telemetry::KEY_WEIGHT_EXTENT,
                               std::numeric_limits<float>::quiet_NaN());
        telemetry::publish_f32(telemetry::KEY_WEIGHT_DIAMETER,
                               std::numeric_limits<float>::quiet_NaN());
        telemetry::publish_f32(telemetry::KEY_WEIGHT_ASPECT,
                               std::numeric_limits<float>::quiet_NaN());
        telemetry::publish_f32(telemetry::KEY_WEIGHT_RANGE,
                               std::numeric_limits<float>::quiet_NaN());
        telemetry::publish_f32(telemetry::KEY_WEIGHT_POINT_COUNT,
                               std::numeric_limits<float>::quiet_NaN());
    };

    if (!this->lidar_processing_task->has_result()) {
        return;
    }

    const auto &result = this->lidar_processing_task->get_last_result();

    if (result.clusters.empty() || result.transformed_points.empty()) {
        return;
    }

    for (auto &track : this->tracked_targets) {
        track.matched_this_update = false;
    }

    for (const auto &cluster : result.clusters) {
        if (cluster.count < WEIGHT_TARGET_MIN_CLUSTER_COUNT ||
            cluster.count > WEIGHT_TARGET_MAX_CLUSTER_COUNT) {
            continue;
        }

        std::span<const LidarResponsePoint> cluster_points(
            result.transformed_points.data() + cluster.start, cluster.count);

        const WeightClusterSummary summary = summarize_weight_cluster(cluster_points);

        if (summary.score < WEIGHT_TARGET_MIN_TRACK_CANDIDATE_SCORE) {
            continue;
        }

        int best_track_index = -1;

        float best_track_distance = WEIGHT_TARGET_TRACK_ASSOCIATION_DISTANCE_M;

        for (size_t track_index = 0; track_index < this->tracked_targets.size(); ++track_index) {
            const auto &track = this->tracked_targets[track_index];

            const float distance = (summary.centroid - track.position).norm();

            if (distance <= best_track_distance) {
                best_track_distance = distance;
                best_track_index = track_index;
            }
        }

        if (best_track_index >= 0) {
            auto &track = this->tracked_targets[best_track_index];
            const float observation_gain = WEIGHT_TARGET_TRACK_CONFIDENCE_GAIN * summary.score;

            apply_summary_to_track(track, summary, WEIGHT_TARGET_TRACK_POSITION_ALPHA);
            track.matched_this_update = true;
            track.missed_updates = 0;
            track.confidence = std::clamp(track.confidence + observation_gain, 0.0f, 1.0f);
            continue;
        }

        if (this->tracked_targets.full()) {
            continue;
        }

        WeightTrackedTarget new_track{};
        new_track.position = summary.centroid;
        new_track.confidence =
            std::clamp(summary.score * WEIGHT_TARGET_TRACK_INITIAL_CONFIDENCE_SCALE, 0.0f, 1.0f);
        new_track.last_observation_score = summary.score;
        new_track.spread = summary.spread;
        new_track.max_extent = summary.max_extent;
        new_track.diameter_mm = summary.diameter_mm;
        new_track.aspect_ratio = summary.aspect_ratio;
        new_track.range = summary.range;
        new_track.count = summary.count;
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

    int best_track_index = -1;
    float best_track_confidence = WEIGHT_TARGET_MIN_SCORE;

    for (size_t track_index = 0; track_index < this->tracked_targets.size(); ++track_index) {
        const auto &track = this->tracked_targets[track_index];
        if (track.confidence < WEIGHT_TARGET_TRACK_PUBLISH_CONFIDENCE) {
            continue;
        }

        if (track.confidence > best_track_confidence) {
            best_track_confidence = track.confidence;
            best_track_index = static_cast<int>(track_index);
        }
    }

    if (best_track_index < 0) {
        clear_weight_target_telemetry();
        return;
    }

    const auto &best_track = this->tracked_targets[best_track_index];

    this->current_weight_target.position = best_track.position;
    this->current_weight_target.confidence = best_track.confidence;
    this->current_weight_target.valid = true;

    telemetry::publish_f32(telemetry::KEY_WEIGHT_TARGET_X,
                           this->current_weight_target.position.x());
    telemetry::publish_f32(telemetry::KEY_WEIGHT_TARGET_Y,
                           this->current_weight_target.position.y());
    telemetry::publish_f32(telemetry::KEY_WEIGHT_TARGET_CONFIDENCE,
                           this->current_weight_target.confidence);
    telemetry::publish_f32(telemetry::KEY_WEIGHT_SPREAD, best_track.spread);
    telemetry::publish_f32(telemetry::KEY_WEIGHT_EXTENT, best_track.max_extent);
    telemetry::publish_f32(telemetry::KEY_WEIGHT_DIAMETER, best_track.diameter_mm);
    telemetry::publish_f32(telemetry::KEY_WEIGHT_ASPECT, best_track.aspect_ratio);
    telemetry::publish_f32(telemetry::KEY_WEIGHT_RANGE, best_track.range);
    telemetry::publish_f32(telemetry::KEY_WEIGHT_POINT_COUNT, (float)best_track.count);
}

bool WeightDetectionTask::has_weight_target() const { return this->current_weight_target.valid; }

Eigen::Vector2f WeightDetectionTask::get_weight_target_position() const {
    return this->current_weight_target.position;
}

float WeightDetectionTask::get_weight_target_confidence() const {
    return this->current_weight_target.confidence;
}

void WeightDetectionTask::loop() {
    uint16_t pins = readPins();

    bool conduction_state = (pins & (1 << ENTRY_CONDUCTION_PIN)) == 0;
    bool switch_state = (pins & (1 << ENTRY_SWITCH_PIN)) == 0;

    switch (weight_intake_state) {
    case WeightIntakeState::None:
        if (switch_state) {
            weight_intake_state = WeightIntakeState::UnknownWeight;
        }

        break;
    case WeightIntakeState::UnknownWeight: {
        if (!switch_state) {
            weight_intake_state = WeightIntakeState::None;
        } else if (conduction_state) {
            weight_intake_state = WeightIntakeState::RealWeightDetected;
        } else if (!conduction_state) {
            weight_intake_state = WeightIntakeState::DummyWeightDetected;
        }

        break;
    }
    case WeightIntakeState::RealWeightDetected:
    case WeightIntakeState::DummyWeightDetected:
        if (!switch_state) {
            weight_intake_state = WeightIntakeState::None;
        }
        break;
    }

    update_weight_target();
}
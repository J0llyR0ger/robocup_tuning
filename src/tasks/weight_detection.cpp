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

WeightDetectionTask::WeightDetectionTask() : SchedulerTask("weight_detection") {}

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

    if (this->lidar_processing_task == nullptr || !this->lidar_processing_task->has_result()) {
        clear_weight_target_telemetry();
        return;
    }

    const auto &result = this->lidar_processing_task->get_last_result();
    if (result.clusters.empty() || result.transformed_points.empty()) {
        clear_weight_target_telemetry();
        return;
    }

    WeightClusterSummary best_summary{};
    float best_score = -1.0f;

    for (const auto &cluster : result.clusters) {
        std::span<const LidarResponsePoint> cluster_points(
            result.transformed_points.data() + cluster.start, cluster.count);
        auto summary = summarize_weight_cluster(cluster_points);

        if (summary.score > best_score) {
            best_score = summary.score;
            best_summary = summary;
        }
    }

    if (best_score <= WEIGHT_TARGET_MIN_SCORE) {
        clear_weight_target_telemetry();
        return;
    }

    this->current_weight_target.position = best_summary.centroid;
    this->current_weight_target.confidence = best_score;
    this->current_weight_target.valid = true;

    telemetry::publish_f32(telemetry::KEY_WEIGHT_TARGET_X,
                           this->current_weight_target.position.x());
    telemetry::publish_f32(telemetry::KEY_WEIGHT_TARGET_Y,
                           this->current_weight_target.position.y());
    telemetry::publish_f32(telemetry::KEY_WEIGHT_TARGET_CONFIDENCE,
                           this->current_weight_target.confidence);
    telemetry::publish_f32(telemetry::KEY_WEIGHT_SPREAD, best_summary.spread);
    telemetry::publish_f32(telemetry::KEY_WEIGHT_EXTENT, best_summary.max_extent);
    telemetry::publish_f32(telemetry::KEY_WEIGHT_DIAMETER, best_summary.diameter_mm);
    telemetry::publish_f32(telemetry::KEY_WEIGHT_ASPECT, best_summary.aspect_ratio);
    telemetry::publish_f32(telemetry::KEY_WEIGHT_RANGE, best_summary.range);
    telemetry::publish_f32(telemetry::KEY_WEIGHT_POINT_COUNT, (float)best_summary.count);
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
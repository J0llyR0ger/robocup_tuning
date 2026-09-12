#pragma once

#include "lib/debouncer.hpp"
#include "lib/lidar.hpp"
#include "lib/lidar_processing.hpp"
#include "lib/odometry.hpp"
#include "scheduler_task.hpp"
#include "tasks/lidar_processing.hpp"
#include "weight_tuning_config.hpp"
#include <Adafruit_BNO055.h>
#include <HardwareSerial.h>
#include <SparkFunSX1509.h>
#include <Wire.h>
#include <config.hpp>
#include <etl/vector.h>

struct WeightTarget {
    Eigen::Vector2f position = Eigen::Vector2f::Zero();
    float confidence = 0.0f;
    bool valid = false;
};

struct WeightTrackedTarget {
    Eigen::Vector2f position = Eigen::Vector2f::Zero();
    float confidence = 0.0f;
    float last_observation_score = 0.0f;

    float spread = 0.0f;
    float max_extent = 0.0f;
    float diameter_mm = 0.0f;
    float aspect_ratio = 1.0f;
    float range = 0.0f;
    int count = 0;

    uint16_t missed_updates = 0;
    bool matched_this_update = false;
};

enum class WeightIntakeState {
    None,
    UnknownWeight,
    RealWeightDetected,
    DummyWeightDetected,
};

class WeightDetectionTask : public SchedulerTask {
  private:
    SX1509 expander;
    LidarProcessingTask *lidar_processing_task;

    WeightIntakeState weight_intake_state = WeightIntakeState::None;
    WeightTarget current_weight_target;
    etl::vector<WeightTrackedTarget, WEIGHT_TARGET_MAX_TRACKS> tracked_targets;

    void update_weight_target();

  public:
    WeightDetectionTask(LidarProcessingTask *lidar_processing_task);

    void setup();
    void loop();

    bool has_weight_target() const;
    Eigen::Vector2f get_weight_target_position() const;
    float get_weight_target_confidence() const;

    int get_frequency() const override { return WEIGHT_DETECTION_FREQ; }
};

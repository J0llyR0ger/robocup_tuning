#pragma once

#include "lib/lidar_processing.hpp"
#include "scheduler_task.hpp"
#include "tasks/lidar.hpp"
#include "tasks/position_tracking.hpp"
#include "weight_tuning_config.hpp"
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

class LidarProcessingTask : public SchedulerTask {
  private:
    LidarTask *lidar_reader_task;
    PositionTrackingTask *position_tracking_task;
    LidarProcessingResult last_result{};
    bool has_last_result = false;
    etl::vector<WeightTrackedTarget, WEIGHT_TARGET_MAX_TRACKS> tracked_targets;

    void update_weight_target();

  public:
    LidarProcessingTask(LidarTask *lidar_reader_task, PositionTrackingTask *position_tracking_task);

    void setup();
    void loop();

    bool has_result() const;
    const LidarProcessingResult &get_last_result() const;

    std::span<WeightTrackedTarget> get_tracked_weights();

    int get_frequency() const override { return LIDAR_PROCESSING_FREQ; }
};
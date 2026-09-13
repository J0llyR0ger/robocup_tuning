#pragma once

#include "lib/lidar_processing.hpp"
#include "scheduler_task.hpp"
#include "tasks/lidar.hpp"
#include "tasks/position_tracking.hpp"
#include <etl/vector.h>
#include <lib/weight_tracking.hpp>

class LidarProcessingTask : public SchedulerTask {
  private:
    LidarProcessingResult last_result{};
    bool has_last_result = false;

    WeightTracking weight_tracking = WeightTracking();
    LidarProcessing lidar_processing = LidarProcessing();

  public:
    LidarProcessingTask();

    void setup();
    void loop();

    bool has_result() const;
    const LidarProcessingResult &get_last_result() const;

    std::span<WeightTrackedTarget> get_tracked_weights();

    uint32_t get_stack_depth() const override { return 1 << 15; };

    int get_frequency() const override { return LIDAR_PROCESSING_FREQ; }
};
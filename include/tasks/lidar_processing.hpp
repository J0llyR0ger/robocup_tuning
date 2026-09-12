#pragma once

#include "lib/lidar_processing.hpp"
#include "scheduler_task.hpp"
#include "tasks/lidar.hpp"
#include "tasks/position_tracking.hpp"

class LidarProcessingTask : public SchedulerTask {
  private:
    LidarTask *lidar_reader_task;
    PositionTrackingTask *position_tracking_task;
    LidarProcessingResult last_result{};
    bool has_last_result = false;

  public:
    LidarProcessingTask(LidarTask *lidar_reader_task, PositionTrackingTask *position_tracking_task);

    void setup();
    void loop();

    bool has_result() const;
    const LidarProcessingResult &get_last_result() const;

    int get_frequency() const override { return LIDAR_PROCESSING_FREQ; }
};
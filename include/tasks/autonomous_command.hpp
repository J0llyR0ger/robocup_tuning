#pragma once

#include "intake.hpp"
#include "lidar_processing.hpp"
#include "scheduler_task.hpp"
#include "tasks/motion_control.hpp"
#include <config.hpp>

class AutonomousCommandTask : public SchedulerTask {
  private:
    LidarProcessingTask *lidar_processing_task;
    PositionTrackingTask *position_tracking_task;
    MotionControlTask *motion_control_task;
    IntakeTask *intake_task;

    std::optional<Eigen::Vector2f> locked_weight_target = std::nullopt;

  public:
    AutonomousCommandTask(LidarProcessingTask *lidar_processing_task,
                          PositionTrackingTask *position_tracking_task,
                          MotionControlTask *motion_control_task, IntakeTask *intake_task);

    void setup();
    void loop();

    int get_frequency() const override { return IMU_TASK_FREQ; }
};
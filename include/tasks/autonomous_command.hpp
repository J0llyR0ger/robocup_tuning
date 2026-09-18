#pragma once

#include "intake.hpp"
#include "lidar_processing.hpp"
#include "scheduler_task.hpp"
#include "tasks/mapping.hpp"
#include "tasks/motion_control.hpp"
#include <config.hpp>

class AutonomousCommandTask : public SchedulerTask {
  private:
    std::optional<Pose> weight_sensed_pose = std::nullopt;
    std::optional<Eigen::Vector2f> locked_weight_position = std::nullopt;

  public:
    AutonomousCommandTask();

    void setup();
    void loop();

    int get_frequency() const override { return IMU_TASK_FREQ; }
};
#pragma once

#include "intake.hpp"
#include "lidar_processing.hpp"
#include "scheduler_task.hpp"
#include "tasks/mapping.hpp"
#include "tasks/motion_control.hpp"
#include <config.hpp>

class AutonomousCommandTask : public SchedulerTask {
  private:
    std::optional<Eigen::Vector2f> locked_weight_target = std::nullopt;

  public:
    AutonomousCommandTask();

    void setup();
    void loop();

    int get_frequency() const override { return IMU_TASK_FREQ; }
};
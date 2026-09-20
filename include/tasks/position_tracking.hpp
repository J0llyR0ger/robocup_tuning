#pragma once

#include "drive_train.hpp"
#include "eigen.h"
#include "imu.hpp"
#include "lib/monte_carlo_localization.hpp"
#include "lib/odometry.hpp"
#include "lidar.hpp"
#include "scheduler_task.hpp"

class PositionTrackingTask : public SchedulerTask {
  private:
    float last_left_wheel_position;
    float last_right_wheel_position;

    float last_heading;

    float accumulated_odometry_distance = 0.0f;
    Eigen::Vector2f last_mcl_position = Eigen::Vector2f::Zero();

    OdometryModule odometry;
    MonteCarloLocalization mcl;

  public:
    PositionTrackingTask();

    void setup();
    void loop();

    uint32_t get_stack_depth() const override { return 1 << 13; };

    int get_frequency() const override { return POSITION_TRACKING_TASK_FREQ; }
};
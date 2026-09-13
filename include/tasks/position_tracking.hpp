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

    OdometryModule odometry;
    MonteCarloLocalization mcl;

  public:
    PositionTrackingTask();

    void setup();
    void loop();

    int get_frequency() const override { return POSITION_TRACKING_TASK_FREQ; }
};
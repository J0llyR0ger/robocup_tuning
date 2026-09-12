#pragma once

#include "lib/pure_pursuit.hpp"
#include "scheduler_task.hpp"
#include "tasks/drive_train.hpp"
#include "tasks/position_tracking.hpp"

#define LIDAR_POINTS_HISTORY 1500

class MotionControlTask : public SchedulerTask {
  private:
    DriveTrainTask *drive_train_task;
    PositionTrackingTask *position_tracking_task;

    PurePursuit pure_pursuit;

    PIDController pid_drive;
    PIDController pid_turn;

  public:
    MotionControlTask(DriveTrainTask *drive_train_task,
                      PositionTrackingTask *position_tracking_task);

    void setup();
    void loop();

    void set_current_path(std::vector<Eigen::Vector2f> positions);

    int get_frequency() const override { return MOTION_CONTROL_TASK_FREQ; }
};
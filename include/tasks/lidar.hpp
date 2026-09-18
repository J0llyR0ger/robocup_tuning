#pragma once

#include "lib/lidar.hpp"
#include "scheduler_task.hpp"
#include <HardwareSerial.h>
#include <config.hpp>
#include <etl/vector.h>

class LidarTask : public SchedulerTask {
  private:
    LidarDataReader reader;

    etl::vector<LidarResponsePoint, MAX_LIDAR_POINTS> points;

    void upsert_point(const LidarResponsePoint &new_point);

    float cummulative_angle = 0.0;
    float last_loop_cummulative_angle = 0.0;

  public:
    LidarTask();

    void setup();
    void loop();

    uint32_t get_stack_depth() const override { return 1 << 12; };

    int get_frequency() const override { return LIDAR_TASK_FREQ; }
};
#pragma once

#include "eigen.h"
#include "scheduler_task.hpp"
#include <Adafruit_BNO055.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <config.hpp>

class ImuTask : public SchedulerTask {
  private:
    Adafruit_BNO055 imu;

    imu::Vector<3> last_euler_angles;

    Eigen::Vector3f get_euler_angles();

  public:
    ImuTask();

    void setup();
    void loop();

    int get_frequency() const override { return IMU_TASK_FREQ; }
};
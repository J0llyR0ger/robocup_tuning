#pragma once

#include "lib/debouncer.hpp"
#include "lib/lidar.hpp"
#include "lib/lidar_processing.hpp"
#include "lib/odometry.hpp"
#include "scheduler_task.hpp"
#include "tasks/lidar_processing.hpp"
#include <Adafruit_BNO055.h>
#include <HardwareSerial.h>
#include <SparkFunSX1509.h>
#include <Wire.h>
#include <config.hpp>

struct WeightTarget {
    Eigen::Vector2f position = Eigen::Vector2f::Zero();
    float confidence = 0.0f;
    bool valid = false;
};

enum class WeightIntakeState {
    None,
    UnknownWeight,
    RealWeightDetected,
    DummyWeightDetected,
};

class WeightDetectionTask : public SchedulerTask {
  private:
    SX1509 expander;
    LidarProcessingTask *lidar_processing_task = nullptr;

    WeightIntakeState weight_intake_state = WeightIntakeState::None;
    WeightTarget current_weight_target;

    void update_weight_target();

  public:
    WeightDetectionTask();
    WeightDetectionTask(LidarProcessingTask *lidar_processing_task);

    void setup();
    void loop();

    bool has_weight_target() const;
    Eigen::Vector2f get_weight_target_position() const;
    float get_weight_target_confidence() const;

    int get_frequency() const override { return WEIGHT_DETECTION_FREQ; }
};

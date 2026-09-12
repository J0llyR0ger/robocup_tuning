
#pragma once
#include <Arduino.h>
#include <Wire.h>

#include "scheduler_task.hpp"
#include <SparkFunSX1509.h>
#include <config.hpp>

enum class WeightIntakeState {
    None,
    UnknownWeight,
    RealWeightDetected,
    DummyWeightDetected,
};

class WeightDetectionTask : public SchedulerTask {
  private:
    SX1509 expander;

    WeightIntakeState weight_intake_state = WeightIntakeState::None;

  public:
    WeightDetectionTask();

    void setup();
    void loop();

    int get_frequency() const override { return WEIGHT_DETECTION_FREQ; }
};

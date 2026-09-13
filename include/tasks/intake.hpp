#pragma once

#include "scheduler_task.hpp"
#include <HardwareSerial.h>
#include <HerkulexServo.h>
#include <Wire.h>

#include <SparkFunSX1509.h>
#include <config.hpp>

enum class WeightIntakeState {
    None,
    UnknownWeight,
    RealWeightDetected,
    DummyWeightDetected,
};

class IntakeTask : public SchedulerTask {
  private:
    HerkulexServoBus herkulexBus = HerkulexServoBus(Serial7);
    HerkulexServo left_servo = HerkulexServo(herkulexBus, 3);
    HerkulexServo right_servo = HerkulexServo(herkulexBus, 2);

    unsigned long last_update = 0;
    unsigned long now = 0;
    bool toggle = false;

    SX1509 expander;

    WeightIntakeState weight_intake_state = WeightIntakeState::None;

  public:
    IntakeTask();

    void setup();
    void loop();

    void set_position(bool up);

    int get_frequency() const override { return INTAKE_TASK_FREQ; }
};
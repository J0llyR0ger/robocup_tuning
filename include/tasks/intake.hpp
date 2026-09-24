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

    SX1509 expander;

    WeightIntakeState weight_intake_state = WeightIntakeState::None;

    int last_conduction_time = 0;

    bool rail_position_commanded = false;
    bool commanded_rails_up = true;
    void set_position(bool up);

  public:
    int total_weights = 0;

    IntakeTask();

    void setup();
    void loop();

    int get_frequency() const override { return INTAKE_TASK_FREQ; }
};
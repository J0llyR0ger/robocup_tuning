#pragma once

#include "scheduler_task.hpp"
#include <HardwareSerial.h>
#include <HerkulexServo.h>
#include <config.hpp>

class IntakeTask : public SchedulerTask {
  private:
    HerkulexServoBus herkulexBus = HerkulexServoBus(Serial7);
    HerkulexServo left_servo = HerkulexServo(herkulexBus, 3);
    HerkulexServo right_servo = HerkulexServo(herkulexBus, 2);

    unsigned long last_update = 0;
    unsigned long now = 0;
    bool toggle = false;

  public:
    IntakeTask();

    void setup();
    void loop();

    void set_position(bool up);

    int get_frequency() const override { return INTAKE_TASK_FREQ; }
};
#include "tasks/intake.hpp"
#include "Arduino.h"

IntakeTask::IntakeTask() : SchedulerTask("intake_task") {}

void IntakeTask::setup() {
    Serial7.begin(115200);

    left_servo.setTorqueOn();
    right_servo.setTorqueOn();

    this->set_position(true);
}

int angleToNum(float angle) { return 512 + (int)(angle / 0.325); }

void IntakeTask::set_position(bool up) {
    if (up) {
        left_servo.setPosition(angleToNum(-15.0), 100, HerkulexLed::Green);
        right_servo.setPosition(angleToNum(9.0), 100, HerkulexLed::Green);
    } else {
        left_servo.setPosition(angleToNum(45.0), 100, HerkulexLed::Blue);
        right_servo.setPosition(angleToNum(-51.0), 100, HerkulexLed::Blue);
    }
}

void IntakeTask::loop() { herkulexBus.update(); }

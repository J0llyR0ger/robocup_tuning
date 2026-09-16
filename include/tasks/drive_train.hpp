#pragma once

#include "config.hpp"
#include "lib/slew.hpp"
#include "scheduler_task.hpp"

#include <Encoder.h>
#include <Servo.h>

const float SLEW_RATE = 4.0;

class DriveTrainTask : public SchedulerTask {
  private:
    Servo left_motor;
    Servo right_motor;

    Encoder left_encoder = Encoder(LEFT_MOTOR_ENCODER_PIN_A, LEFT_MOTOR_ENCODER_PIN_B);
    Encoder right_encoder = Encoder(RIGHT_MOTOR_ENCODER_PIN_A, RIGHT_MOTOR_ENCODER_PIN_B);

    uint32_t last_timestamp;

    int last_left_ticks = 0;
    int last_right_ticks = 0;

    float left_command = 0.0;
    float right_command = 0.0;

    SlewRate<float> left_slew = SlewRate<float>(SLEW_RATE);
    SlewRate<float> right_slew = SlewRate<float>(SLEW_RATE);

  public:
    DriveTrainTask();

    void setup();
    void loop();

    int get_frequency() const override { return DRIVE_TRAIN_TASK_FREQ; }
};
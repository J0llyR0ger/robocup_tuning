#include "tasks/drive_train.hpp"
#include "telemetry_bus.hpp"
#include <wiring.h>

DriveTrainTask::DriveTrainTask() : SchedulerTask("drive_train_task") {}

void DriveTrainTask::setup() {
    left_motor.attach(LEFT_MOTOR_CONTROL_PIN);
    right_motor.attach(RIGHT_MOTOR_CONTROL_PIN);
}

static const float RADIANS_PER_TICK = 2.0 * PI / (float)TICKS_PER_REVOLUTION;

static const float LEFT_SCALE = 0.75;

void DriveTrainTask::loop() {
    this->left_command = std::clamp(this->left_command, -1.0f, 1.0f);
    this->right_command = std::clamp(this->right_command, -1.0f, 1.0f);

    float left_out = this->left_command;

    if (left_out > 0.0) {
        left_out *= LEFT_SCALE;
    }

    left_motor.writeMicroseconds(map(left_out, 1.0, -1.0, FORWARD_MS, REVERSE_MS));
    right_motor.writeMicroseconds(map(this->right_command, 1.0, -1.0, REVERSE_MS, FORWARD_MS));

    telemetry::publish_f32(telemetry::KEY_LEFT_COMMAND, this->left_command);
    telemetry::publish_f32(telemetry::KEY_RIGHT_COMMAND, this->right_command);

    uint32_t timestamp = micros();

    float dt = (float)(timestamp - last_timestamp) / 1e6;

    last_timestamp = timestamp;

    int left_ticks = left_encoder.read();
    int right_ticks = -right_encoder.read();

    float left_velocity = RADIANS_PER_TICK * (float)(left_ticks - last_left_ticks) / dt;
    float right_velocity = RADIANS_PER_TICK * (float)(right_ticks - last_right_ticks) / dt;

    last_left_ticks = left_ticks;
    last_right_ticks = right_ticks;

    telemetry::publish_f32(telemetry::KEY_LEFT_WHEEL_VELOCITY, left_velocity);
    telemetry::publish_f32(telemetry::KEY_RIGHT_WHEEL_VELOCITY, right_velocity);
}

float DriveTrainTask::get_left_wheel_position() {
    return RADIANS_PER_TICK * this->last_left_ticks; //
}

float DriveTrainTask::get_right_wheel_position() {
    return RADIANS_PER_TICK * this->last_right_ticks; //
}

void DriveTrainTask::set_commands(float left, float right) {
    this->left_command = left;
    this->right_command = right;
}

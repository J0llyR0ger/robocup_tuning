#include "tasks/drive_train.hpp"
#include "queues.hpp"
#include "telemetry_bus.hpp"
#include <wiring.h>

DriveTrainTask::DriveTrainTask() : SchedulerTask("drive_train_task") {}

void DriveTrainTask::setup() {
    left_motor.attach(LEFT_MOTOR_CONTROL_PIN);
    right_motor.attach(RIGHT_MOTOR_CONTROL_PIN);
}

static const int TICKS_PER_REVOLUTION = 2652;
static const int FORWARD_MS = 1950;
static const int REVERSE_MS = 1050;

static const float RADIANS_PER_TICK = 2.0 * PI / (float)TICKS_PER_REVOLUTION;

static const float LEFT_SCALE = 0.75;

static const float KICKOFF_VALUE = 0.2;

static float apply_kickoff(float command) {
    if (command == 0.0f) {
        return 0.0f;
    }

    float magnitude = std::abs(command);
    float remapped = KICKOFF_VALUE + (1.0f - KICKOFF_VALUE) * magnitude;

    return command > 0.0f ? remapped : -remapped;
}

void DriveTrainTask::loop() {
    std::tuple<float, float> new_commands;

    if (xQueueReceive(motionControl_ChassisCommandsQueue, &new_commands, 0)) {
        this->left_command = std::clamp(std::get<0>(new_commands), -1.0f, 1.0f);
        this->right_command = std::clamp(std::get<1>(new_commands), -1.0f, 1.0f);
    }

    float left_out = this->left_command;
    float right_out = this->right_command;

    if (left_out > 0.0) {
        left_out *= LEFT_SCALE;
    }

    left_out = apply_kickoff(left_out);
    right_out = apply_kickoff(right_out);

    float left_rate = this->left_slew.update(left_out);
    float right_rate = this->right_slew.update(right_out);

    left_motor.writeMicroseconds(map(left_rate, 1.0, -1.0, FORWARD_MS, REVERSE_MS));
    right_motor.writeMicroseconds(map(right_rate, 1.0, -1.0, REVERSE_MS, FORWARD_MS));

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

    auto wheel_positions = std::make_tuple(RADIANS_PER_TICK * (float)last_left_ticks,
                                           RADIANS_PER_TICK * (float)last_right_ticks);

    xQueueSendToFront(driveTrain_positionTrackingWheelPositionQueue, &wheel_positions, 0);

    telemetry::publish_f32(telemetry::KEY_LEFT_WHEEL_VELOCITY, left_velocity);
    telemetry::publish_f32(telemetry::KEY_RIGHT_WHEEL_VELOCITY, right_velocity);
}

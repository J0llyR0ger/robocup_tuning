#include "tasks/motion_control.hpp"
#include "telemetry_bus.hpp"

#define DRIVE_KP 35e-1
#define DRIVE_KI 0 // 20e-4

#define TURN_KP 2.0
#define TURN_KI 0
#define TURN_KD 16e-1

MotionControlTask::MotionControlTask(DriveTrainTask *drive_train_task,
                                     PositionTrackingTask *position_tracking_task)
    : SchedulerTask("motion_control"), drive_train_task(drive_train_task),
      position_tracking_task(position_tracking_task), pure_pursuit(0.3),
      pid_drive(PIDController(DRIVE_KP, DRIVE_KI, 0, 10)
                    .with_output_limits(-0.5, 0.5)
                    .with_integral_bounds(-100, 100)),
      pid_turn(PIDController(TURN_KP, TURN_KI, TURN_KD, 3 * DEG_TO_RAD)
                   .with_output_limits(-2.0, 2.0)
                   .with_integral_bounds(-30 * DEG_TO_RAD, 30 * DEG_TO_RAD)) {}

void MotionControlTask::setup() {}

void MotionControlTask::set_current_path(std::vector<Eigen::Vector2f> positions) {
    this->pure_pursuit.set_current_path(positions);
}

void MotionControlTask::loop() {
    auto pose = position_tracking_task->get_current_pose();

    auto [drive_error, turn_error] = pure_pursuit.compute_errors(pose);

    telemetry::publish_f32(telemetry::KEY_TURN_ERROR, turn_error);
    telemetry::publish_f32(telemetry::KEY_DRIVE_ERROR, drive_error);

    float drive_output = pid_drive.update(drive_error);
    float turn_output = pid_turn.update(turn_error);

    float left_drive = drive_output + turn_output;
    float right_drive = drive_output - turn_output;
    float largest_cmd = fabs(fmax(left_drive, right_drive));

    if (largest_cmd > 1.0) {
        left_drive = left_drive / largest_cmd;
        right_drive = right_drive / largest_cmd;
    }

    drive_train_task->set_commands(left_drive, right_drive);
}
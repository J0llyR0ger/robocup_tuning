#include "tasks/autonomous_command.hpp"
#include "telemetry_bus.hpp"

AutonomousCommandTask::AutonomousCommandTask(LidarProcessingTask *lidar_processing_task,
                                             PositionTrackingTask *position_tracking_task,
                                             MotionControlTask *motion_control_task,
                                             IntakeTask *intake_task)
    : SchedulerTask("autonomous_command"), lidar_processing_task(lidar_processing_task),
      position_tracking_task(position_tracking_task), motion_control_task(motion_control_task),
      intake_task(intake_task) {}

void AutonomousCommandTask::setup() {}

void AutonomousCommandTask::loop() {
    auto robot_position = this->position_tracking_task->get_current_pose().position;

    if (this->intake_task->total_weights >= 4 && !this->locked_weight_target.has_value()) {
        this->motion_control_task->set_current_path({robot_position, {0.5, 0.5}});
    } else if (this->locked_weight_target.has_value()) {
        auto drive_target = this->locked_weight_target.value();

        this->motion_control_task->set_current_path({robot_position, drive_target});

        if ((drive_target - robot_position).norm() < 0.15) {
            this->locked_weight_target = std::nullopt;
            this->intake_task->set_position(true);
        }
    } else {
        auto targets = this->lidar_processing_task->get_tracked_weights();

        int best_track_index = -1;
        float closest_weight_distance = 5.0;

        for (size_t track_index = 0; track_index < targets.size(); ++track_index) {
            const auto &track = targets[track_index];
            if (track.confidence < 0.7) {
                continue;
            }

            auto dist = (track.cluster.centroid - robot_position).norm();

            if (dist < closest_weight_distance) {
                closest_weight_distance = dist;
                best_track_index = static_cast<int>(track_index);
            }
        }

        if (best_track_index < 0) {
            this->motion_control_task->set_current_path(
                {}); // TODO: In this case, go into discovery mode
            return;
        }

        const auto &best_track = targets[best_track_index];

        this->motion_control_task->set_current_path({robot_position, best_track.cluster.centroid});

        if (closest_weight_distance < 0.7) {
            this->locked_weight_target = best_track.cluster.centroid;
            this->intake_task->set_position(false);
        }
    }
}

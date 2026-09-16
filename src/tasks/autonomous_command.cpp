#include "tasks/autonomous_command.hpp"
#include "telemetry_bus.hpp"
#include <mutexes.hpp>
#include <queues.hpp>

AutonomousCommandTask::AutonomousCommandTask() : SchedulerTask("autonomous_command") {}

void AutonomousCommandTask::setup() {}

void AutonomousCommandTask::loop() {
    auto robot_position = get_global_pose().get_center_pose().position;

    WeightTrackingPayload weight_targets;

    xQueuePeek(weight_tracking_queue, &weight_targets, portMAX_DELAY);

    int best_track_index = -1;
    float closest_weight_distance = 5.0;

    for (size_t track_index = 0; track_index < weight_targets.count; ++track_index) {
        const auto &track = weight_targets.targets[track_index];
        if (track.confidence < 0.7) {
            continue;
        }

        auto dist = (track.cluster.centroid - robot_position).norm();

        if (dist < closest_weight_distance) {
            closest_weight_distance = dist;
            best_track_index = static_cast<int>(track_index);
        }
    }

    uint8_t total_weights_carried;

    xQueuePeek(carried_weight_count, &total_weights_carried, portMAX_DELAY);

    if (total_weights_carried >= 4 && !this->locked_weight_target.has_value()) {
        set_motion_control_path(get_home_path());
    } else if (this->locked_weight_target.has_value()) {
        auto drive_target = this->locked_weight_target.value();

        set_motion_control_path({robot_position, drive_target});

        if ((drive_target - robot_position).norm() < 0.15) {
            this->locked_weight_target = std::nullopt;
            bool sendVal = true;
            xQueueOverwrite(intake_position_queue, &sendVal);
        }
    } else if (best_track_index >= 0) {
        const auto &best_track = weight_targets.targets[best_track_index];

        set_motion_control_path({robot_position, best_track.cluster.centroid});

        if (closest_weight_distance < 0.7) {
            this->locked_weight_target = best_track.cluster.centroid;
            bool sendVal = false;
            xQueueOverwrite(intake_position_queue, &sendVal);
        }
    } else {
        set_motion_control_path(get_discovery_path());
    }
}

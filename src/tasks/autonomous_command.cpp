#include "tasks/autonomous_command.hpp"
#include "telemetry_bus.hpp"
#include <mutexes.hpp>
#include <queues.hpp>

AutonomousCommandTask::AutonomousCommandTask() : SchedulerTask("autonomous_command") {}

void AutonomousCommandTask::setup() {}

void AutonomousCommandTask::loop() {
    auto robot_pose = get_global_pose();

    WeightTrackingPayload weight_targets;

    xQueuePeek(weight_tracking_queue, &weight_targets, portMAX_DELAY);

    int best_track_index = -1;
    float closest_weight_distance = 5.0;

    for (size_t track_index = 0; track_index < weight_targets.count; ++track_index) {
        const auto &track = weight_targets.targets[track_index];
        if (track.confidence < 0.7) {
            continue;
        }

        auto dist = (track.cluster.centroid - robot_pose.position).norm();

        if (dist < closest_weight_distance) {
            closest_weight_distance = dist;
            best_track_index = static_cast<int>(track_index);
        }
    }

    uint8_t total_weights_carried;

    xQueuePeek(carried_weight_count, &total_weights_carried, portMAX_DELAY);

    bool is_real;

    if (total_weights_carried >= 4 && !this->weight_sensed_pose.has_value()) {
        set_motion_control_path({get_home_path(), 1.0});
    } else if (this->weight_sensed_pose.has_value()) {
        auto sensed_pose = this->weight_sensed_pose.value();

        set_motion_control_path(
            {{sensed_pose.position + sensed_pose.get_direction_vector() * 1.0}, 0.3});

        if ((sensed_pose.position - robot_pose.position).norm() > 0.2) {
            this->weight_sensed_pose = std::nullopt;
            // TODO: Track total weight counting from here
        }
    } else if (xQueueReceive(intake_entry_queue, &is_real, 0)) {
        this->weight_sensed_pose = robot_pose;
        bool sendVal = false;
        xQueueOverwrite(intake_position_queue, &sendVal);
        // Todo: ignore fake weights

    } else if (best_track_index >= 0) {
        const auto &best_track = weight_targets.targets[best_track_index];

        float speed = 1.0;

        if (closest_weight_distance < 0.7) {
            speed = map(closest_weight_distance, 0.7, 0.4, 1.0, 0.3);
        }

        if (closest_weight_distance < 0.7)

            set_motion_control_path({{best_track.cluster.centroid}, speed});
    } else {
        set_motion_control_path({get_discovery_path(), 1.0});
    }
}

#include "tasks/autonomous_command.hpp"
#include "telemetry_bus.hpp"
#include <mutexes.hpp>
#include <queues.hpp>

AutonomousCommandTask::AutonomousCommandTask() : SchedulerTask("autonomous_command") {}

void AutonomousCommandTask::setup() {}

void AutonomousCommandTask::ignore_locked_weight() {
    if (this->locked_weight_position.has_value()) {
        this->failed_weight_positions.push_back(this->locked_weight_position.value());
        this->locked_weight_position = std::nullopt;
    }
}

void AutonomousCommandTask::loop() {
    auto robot_pose = get_global_pose();

    Eigen::Vector2f locking_center = robot_pose.position + robot_pose.get_direction_vector() * 0.5;

    WeightTrackingPayload weight_targets;

    xQueuePeek(weight_tracking_queue, &weight_targets, portMAX_DELAY);

    bool is_real;

    // A non-conductive entry means the robot reached the locked target but did not
    // collect a real weight. Keep the coordinate locally so the map can continue
    // tracking it, while autonomous selection moves on to another target.
    if (xQueueReceive(intake_entry_queue, &is_real, 0)) {
        if (!is_real && this->locked_weight_position.has_value()) {
            this->ignore_locked_weight();
        } else if (is_real) {
            this->weight_sensed_pose = robot_pose;
            this->locked_weight_position = std::nullopt;
        }
    }

    int best_track_index = -1;
    float closest_weight_distance = 5.0;

    for (size_t track_index = 0; track_index < weight_targets.count; ++track_index) {
        const auto &track = weight_targets.targets[track_index];

        bool previously_failed = false;

        for (const auto &failed_position : this->failed_weight_positions) {
            if ((track - failed_position).norm() < 0.3) {
                previously_failed = true;
                break;
            }
        }

        if (previously_failed) {
            continue;
        }

        auto dist = (track - robot_pose.position).norm();

        if (dist < closest_weight_distance) {
            closest_weight_distance = dist;
            best_track_index = static_cast<int>(track_index);
        }
    }

    uint8_t total_weights_carried;

    xQueuePeek(carried_weight_count, &total_weights_carried, portMAX_DELAY);

    bool intake_position = false;

    if (total_weights_carried >= 4 && !this->weight_sensed_pose.has_value()) {
        set_motion_control_path({get_home_path(), 1.0});
        intake_position = true;
    } else if (this->weight_sensed_pose.has_value()) {
        auto sensed_pose = this->weight_sensed_pose.value();

        set_motion_control_path(
            {{sensed_pose.position + sensed_pose.get_direction_vector() * 1.0}, 1.0});

        if ((sensed_pose.position - robot_pose.position).norm() > 0.2) {
            this->weight_sensed_pose = std::nullopt;
            intake_position = true;
        } else {
            intake_position = false;
        }
    } else if (this->locked_weight_position.has_value()) {
        if ((this->locked_weight_position.value() - locking_center).norm() > 0.3) {
            // Preserve the original lock-loss behaviour, except do not allow the
            // same known coordinate to be locked again after a missed pickup.
            this->ignore_locked_weight();
        } else {
            set_motion_control_path({{this->locked_weight_position.value()}, 1.0});
            intake_position = false;
        }
    } else if (best_track_index >= 0) {
        const auto &best_track = weight_targets.targets[best_track_index];

        float speed = 1.0;

        if ((best_track - locking_center).norm() < 0.3) {
            this->locked_weight_position = best_track;
            intake_position = false;
        } else {
            intake_position = true;
        }

        set_motion_control_path({{best_track}, speed});
    } else {
        intake_position = true;
        set_motion_control_path({get_discovery_path(), 1.0});
    }

    xQueueOverwrite(intake_position_queue, &intake_position);
}

#include "lib/pure_pursuit.hpp"
#include "telemetry_bus.hpp"
#include <utils.hpp>

using Eigen::Vector2f;

const float TURN_DEADZONE = 0.01; // TODO: Tune this

PurePursuit::PurePursuit(float look_ahead_distance) : look_ahead_distance(look_ahead_distance) {}

void PurePursuit::set_current_path(std::vector<Vector2f> positions, bool backwards) {
    current_path = positions;
    drive_path_backwards = backwards;
}

void PurePursuit::set_drive_direction(bool backwards) { drive_path_backwards = backwards; }

std::tuple<float, float> PurePursuit::compute_errors(Pose current_pose) {
    auto current_position = current_pose.position;

    float turn_error = 0;
    float drive_error = 0;

    if (current_path.size() >= 1) {
        // Apply regular pure pursuit
        Vector2f last_point;
        Vector2f next_point;

        if (current_path.size() == 1) {
            last_point = current_position;
            next_point = current_path[0];
        } else {
            last_point = current_path[0];
            next_point = current_path[1];

            // Ensure lookahead point is always in right place by clearing old parts of path
            // (This is important with very fine paths)
            while (current_path.size() >= 2 &&
                   (next_point - current_position).norm() <= look_ahead_distance) {
                last_point = current_path[0];
                next_point = current_path[1];

                current_path.erase(current_path.begin());
            }
        }

        auto current_direction = current_pose.get_direction_vector();

        float current_heading = current_pose.heading;

        if (drive_path_backwards) {
            current_heading = diff_angle(std::numbers::pi, current_heading);
        }

        auto lookahead_point = get_snapped_radius_target(current_position, look_ahead_distance,
                                                         last_point, next_point);

        telemetry::publish_f32(telemetry::KEY_LOOKAHEAD_X, lookahead_point.x());
        telemetry::publish_f32(telemetry::KEY_LOOKAHEAD_Y, lookahead_point.y());

        telemetry::publish_f32(telemetry::KEY_NEXTPOINT_X, next_point.x());
        telemetry::publish_f32(telemetry::KEY_NEXTPOINT_Y, next_point.y());

        // Initialize Drive Error as distance to lookahead point plus distance from lookahead point
        // to next point
        drive_error =
            (current_position - lookahead_point).norm() + (lookahead_point - next_point).norm();

        // Loop through the remaining path segments adding that distance to both remaining distance
        // and drive error
        for (int i = 1; i < current_path.size() - 1; i++) {
            float segment_length = (current_path[i] - current_path[i + 1]).norm();

            drive_error += segment_length;
        }

        drive_error =
            drive_error * (lookahead_point - current_position).normalized().dot(current_direction);

        auto direction = lookahead_point - current_position;

        turn_error = diff_angle(current_heading, std::atan2(direction.x(), direction.y()));

        // Only enable turn PID outside a certain distance of target point (this should only
        // matter when approaching the last point in the current path)
        if (current_path.size() <= 2 && (next_point - current_position).norm() <= TURN_DEADZONE) {
            turn_error = 0;
        }
    }

    return std::make_tuple(drive_error, turn_error);
}

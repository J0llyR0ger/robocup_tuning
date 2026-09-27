#pragma once
#include <atomic>
#include <cstdint>
#include "config/position_tracking.hpp"
#include "lib/odometry.hpp"

// The display applies the selected base immediately while drive is inhibited.
inline std::atomic<bool> selected_home_blue{false};
inline std::atomic<bool> active_home_blue{false};
inline std::atomic<bool> drive_start_pending{false};
// Request the default green home at boot too, even without a menu change.
inline std::atomic<uint32_t> home_request_generation{1};
inline std::atomic<uint32_t> home_pose_generation{0};
inline std::atomic<uint32_t> home_map_generation{0};

inline Pose home_start_pose(bool blue) {
    return {.position = Eigen::Vector2f(blue ? FIELD_WIDTH_X_METERS - INITIAL_X : INITIAL_X,
                                       INITIAL_Y),
            .heading = blue ? -INITIAL_HEADING : INITIAL_HEADING};
}

// Navigation and unloading use the home applied by the setup menu.
inline Eigen::Vector2f active_home_position() {
    return home_start_pose(active_home_blue.load()).position;
}

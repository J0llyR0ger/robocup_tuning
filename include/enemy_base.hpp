#pragma once
#include "home_selection.hpp"
#include <algorithm>
#include <cmath>

// Match the corner base area already reserved by OccupancyGridMap::clear().
constexpr float ENEMY_BASE_SIZE_M = 0.65f;
constexpr float ENEMY_BASE_CLEARANCE_M = 0.00f;

// Segment/rectangle intersection, including the clearance around the base.
// Own blue => enemy green at low X; own green => enemy blue at high X.
inline bool crosses_enemy_base(const Eigen::Vector2f &a, const Eigen::Vector2f &b) {
    const float size = ENEMY_BASE_SIZE_M + ENEMY_BASE_CLEARANCE_M;
    const float xmin = active_home_blue.load() ? -size : FIELD_WIDTH_X_METERS - size;
    const float xmax = active_home_blue.load() ? size : FIELD_WIDTH_X_METERS + size;
    const float mins[2] = {xmin, -size};
    const float maxs[2] = {xmax, size};
    float enter = 0.0f, leave = 1.0f;
    for (int axis = 0; axis < 2; ++axis) {
        const float delta = b[axis] - a[axis];
        if (std::fabs(delta) < 1e-6f) {
            if (a[axis] < mins[axis] || a[axis] > maxs[axis]) return false;
        } else {
            float near = (mins[axis] - a[axis]) / delta;
            float far = (maxs[axis] - a[axis]) / delta;
            if (near > far) std::swap(near, far);
            enter = std::max(enter, near);
            leave = std::min(leave, far);
            if (enter > leave) return false;
        }
    }
    return true;
}
inline bool inside_enemy_base(const Eigen::Vector2f &point) {
    return crosses_enemy_base(point, point);
}

// Signed clearance through either field-facing edge (negative while inside).
inline float enemy_base_exit_clearance(const Eigen::Vector2f &point) {
    const float size = ENEMY_BASE_SIZE_M + ENEMY_BASE_CLEARANCE_M;
    const float x_clearance = active_home_blue.load() ? point.x() - size
        : FIELD_WIDTH_X_METERS - size - point.x();
    return std::max(x_clearance, point.y() - size);
}

inline Eigen::Vector2f enemy_base_escape_direction(const Eigen::Vector2f &point) {
    const float size = ENEMY_BASE_SIZE_M + ENEMY_BASE_CLEARANCE_M;
    const bool blue = active_home_blue.load();
    const float x_distance = blue ? size - point.x()
        : point.x() - (FIELD_WIDTH_X_METERS - size);
    return x_distance <= size - point.y()
        ? Eigen::Vector2f(blue ? 1.0f : -1.0f, 0.0f)
        : Eigen::Vector2f(0.0f, 1.0f);
}

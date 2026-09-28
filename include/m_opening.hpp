#pragma once
#include "lib/m_opening.hpp"
#include "enemy_base.hpp"
#include <atomic>

inline std::atomic<m_opening::Phase> m_opening_phase{m_opening::Phase::Idle};
inline std::atomic<bool> m_opening_stopped{false};
inline std::atomic<bool> m_opening_servo_raised{false};
constexpr float M_OPENING_BASE_OFFSET_M = 0.65f;
constexpr float M_OPENING_ARRIVAL_M = 0.10f;
constexpr float M_OPENING_DROP_BEHIND_M = 0.030f;
inline Eigen::Vector2f m_opening_target() {
    const float inset = ENEMY_BASE_SIZE_M + M_OPENING_BASE_OFFSET_M;
    return {active_home_blue.load() ? inset : FIELD_WIDTH_X_METERS - inset, INITIAL_Y};
}

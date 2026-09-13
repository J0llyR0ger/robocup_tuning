#pragma once

#include <numbers>

// Position Tracking
static constexpr float FIELD_WIDTH_X_METERS = 2.425;
static constexpr float FIELD_HEIGHT_Y_METERS = 4.85;

static const float INITIAL_X = 0.15;
static const float INITIAL_Y = 0.3;
static const float INITIAL_HEADING = std::numbers::pi / 2.0;

// Sensor Properties
static const float HEADING_NOISE_PER_RADIAN = 0.01;
static const float POSITION_NOISE_PER_METER = 0.2;
static const float LIDAR_NOISE = 0.02;
static const float LIDAR_MAX_DISTANCE = 12.0;
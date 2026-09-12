#pragma once

// Lidar
static const float LIDAR_OFFSEST_X = -0.08;
static const float LIDAR_OFFSET_Y = 0.330;
static const float LIDAR_ANGLE = 135.0;

// This is applied before the rotation offset. Measured anticlockwise from X axis
static const float LIDAR_START_ANGLE = 135.0;
static const float LIDAR_END_ANGLE = 350.0;

// Chassis
static const float DRIVE_WIDTH_MM = 255.0;
static const float WHEEL_RADIUS_MM = 35.0 * (48.0 / 24.0);

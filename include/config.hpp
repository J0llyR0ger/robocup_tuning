#pragma once

//
// Hardware
//

// Motor Control
static const int LEFT_MOTOR_CONTROL_PIN = 1;
static const int RIGHT_MOTOR_CONTROL_PIN = 0;
static const int FORWARD_MS = 1950;
static const int REVERSE_MS = 1050;

// Motor Feedback
static const int LEFT_MOTOR_ENCODER_PIN_A = 4;
static const int LEFT_MOTOR_ENCODER_PIN_B = 5;
static const int RIGHT_MOTOR_ENCODER_PIN_A = 2;
static const int RIGHT_MOTOR_ENCODER_PIN_B = 3;
static const int TICKS_PER_REVOLUTION = 2985;

// Lidar
static const float LIDAR_OFFSEST_X = -0.08;
static const float LIDAR_OFFSET_Y = 0.330;
static const float LIDAR_ANGLE = 135.0;

// This is applied before the rotation offset. Measured anticlockwise from X axis
static const float LIDAR_START_ANGLE = 135.0;
static const float LIDAR_END_ANGLE = 360.0;

// Position Tracking
static const float DRIVE_WIDTH_MM = 255.0;
static constexpr float FIELD_WIDTH_X_METERS = 2.425;
static constexpr float FIELD_HEIGHT_Y_METERS = 4.85;

static const float WHEEL_RADIUS_MM = 35.0 * (48.0 / 24.0);

static const float INITIAL_X = 0.4;
static const float INITIAL_Y = 0.4;
static const float INITIAL_HEADING = 1.57079632679;

// Sensor Properties
static const float HEADING_NOISE_PER_RADIAN = 0.01;
static const float POSITION_NOISE_PER_METER = 0.2;
static const float LIDAR_NOISE = 0.02;
static const float LIDAR_MAX_DISTANCE = 12.0;

// Weight Detection
static const int ENTRY_CONDUCTION_PIN = 0;
static const int ENTRY_SWITCH_PIN = 1;

//
// Software
//

// Task Frequencies
static const int LIDAR_TASK_FREQ = 500;
static const int MOTION_CONTROL_TASK_FREQ = 60;
static const int POSITION_TRACKING_TASK_FREQ = 60;
static const int MAPPING_TASK_FREQ = 60;
static const int IMU_TASK_FREQ = 60;
static const int LIDAR_PROCESSING_FREQ = 5;
static const int WEIGHT_DETECTION_FREQ = 200;

static const int DRIVE_TRAIN_TASK_FREQ = 60;
static const int INTAKE_TASK_FREQ = 20;
static const int TELEMETRY_TASK_FREQ = 10;
static const int USER_COMMAND_TASK_FREQ = 50;

// Lidar Point Classification
static const float COARSE_THRESHOLD_RANGE_MULTIPLIER = 0.1;
static const float COARSE_THRESHOLD_OFFSET = 0.04;
static const float MAX_CIRCLE_RADIUS = 0.075;
static const float MIN_CIRCLE_RADIUS = 0.01;
static const float MAX_CIRCLE_NOISE = 0.02;

static const int MIN_POINTS_PER_OBJECT = 5;

static const int MAX_LIDAR_POINTS = 512;

static const bool ENABLE_TASK_LOGGING = true;

static const int WEIGHT_DETECTION_DEBOUNCE_MS = 8;
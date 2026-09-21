#pragma once

#include "lib/lidar.hpp"
#include "lib/weight_tracking.hpp"
#include <FreeRTOS.h>
#include <queue.h>

// Position Sensors
inline QueueHandle_t imu_positionTrackingHeadingQueue = xQueueCreate(1, sizeof(float));
inline QueueHandle_t driveTrain_positionTrackingWheelPositionQueue =
    xQueueCreate(1, sizeof(std::tuple<float, float>));

// Lidar Scans
struct LidarScanPayload {
    LidarResponsePoint points[MAX_LIDAR_POINTS];
    size_t count;
};

inline QueueHandle_t lidarReader_lidarProcessingScanQueue =
    xQueueCreate(1, sizeof(LidarScanPayload));

inline QueueHandle_t lidarReader_PositionTrackingScanQueue =
    xQueueCreate(1, sizeof(LidarScanPayload));

inline QueueHandle_t lidarReader_MappingScanQueue = xQueueCreate(1, sizeof(LidarScanPayload));

// Chassis Commands

inline QueueHandle_t motionControl_ChassisCommandsQueue =
    xQueueCreate(1, sizeof(std::tuple<float, float>));

// Requests that temporarily take precedence over path following.  The command
// task owns the lifetime of these requests; MotionControlTask owns the motors.
enum class MotionControlOverride : uint8_t {
    None,
    DummyWeightReverse,
    HomeDropReverse,
};

inline QueueHandle_t motion_control_override_queue =
    xQueueCreate(1, sizeof(MotionControlOverride));

// Published by MotionControlTask while its encoder/motion-mismatch recovery
// reverse is active.  AutonomousCommandTask uses it to keep the intake safe.
inline QueueHandle_t motion_control_recovery_reversing_queue = xQueueCreate(1, sizeof(bool));

// Weight Scans
struct WeightTrackingPayload {
    Eigen::Vector2f targets[WEIGHT_TARGET_MAX_TRACKS];
    size_t count;
};

inline QueueHandle_t weight_tracking_queue = xQueueCreate(1, sizeof(WeightTrackingPayload));

// Intake

inline QueueHandle_t carried_weight_count = xQueueCreate(1, sizeof(uint8_t));
inline QueueHandle_t intake_reset_carried_weight_count_queue = xQueueCreate(1, sizeof(bool));

inline QueueHandle_t intake_position_queue = xQueueCreate(1, sizeof(bool));

// True is no metal (probe high); false is metal present (probe low).
inline QueueHandle_t storage_voltage_probe_queue = xQueueCreate(1, sizeof(bool));

// True means rule, false means dummy
inline QueueHandle_t intake_entry_queue = xQueueCreate(10, sizeof(bool));

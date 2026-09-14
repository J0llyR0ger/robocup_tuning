#include "lib/lidar.hpp"
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
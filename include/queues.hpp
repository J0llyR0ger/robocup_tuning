#include "lib/lidar.hpp"
#include <FreeRTOS.h>
#include <queue.h>

inline QueueHandle_t imu_positionTrackingHeadingQueue = xQueueCreate(1, sizeof(float));
inline QueueHandle_t driveTrain_positionTrackingWheelPositionQueue =
    xQueueCreate(1, sizeof(std::tuple<float, float>));

inline QueueHandle_t lidarReader_lidarProcessingScanQueue =
    xQueueCreate(1, sizeof(etl::vector<LidarResponsePoint, MAX_LIDAR_POINTS>));

#include "tasks/lidar_processing.hpp"
#include "lib/lidar_processing.hpp"
#include "telemetry_bus.hpp"
#include <mutexes.hpp>
#include <queues.hpp>

LidarProcessingTask::LidarProcessingTask() : SchedulerTask("lidar_processing") {}

void LidarProcessingTask::setup() {}

void LidarProcessingTask::loop() {
    LidarScanPayload payload;
    xQueueReceive(lidarReader_lidarProcessingScanQueue, &payload, portMAX_DELAY);

    etl::vector<LidarResponsePoint, MAX_LIDAR_POINTS> points;
    points.assign(payload.points, payload.points + payload.count);

    auto pose = get_global_pose();

    this->lidar_processing.process_points(points, pose, this->last_result);

    this->weight_tracking.update_from_clusters(this->last_result.clusters);
    telemetry::publish_tracked_weights(this->weight_tracking.get_tracked_weights());
    telemetry::publish_lidar_points(this->last_result.transformed_points);
    // telemetry::publish_lidar_processing(this->last_result); // Stack Overflowing at the moment
}

std::span<WeightTrackedTarget> LidarProcessingTask::get_tracked_weights() {
    return this->weight_tracking.get_tracked_weights();
}

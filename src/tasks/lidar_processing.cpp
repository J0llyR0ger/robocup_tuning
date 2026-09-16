#include "tasks/lidar_processing.hpp"
#include "lib/lidar_processing.hpp"
#include "telemetry_bus.hpp"
#include <mutexes.hpp>
#include <queues.hpp>

LidarProcessingTask::LidarProcessingTask() : SchedulerTask("lidar_processing") {}

void LidarProcessingTask::setup() {}

void LidarProcessingTask::loop() {
    LidarScanPayload lidar_payload;
    xQueueReceive(lidarReader_lidarProcessingScanQueue, &lidar_payload, portMAX_DELAY);

    etl::vector<LidarResponsePoint, MAX_LIDAR_POINTS> points;
    points.assign(lidar_payload.points, lidar_payload.points + lidar_payload.count);

    auto pose = get_global_pose();

    this->lidar_processing.process_points(points, pose, this->last_result);

    this->weight_tracking.update_from_clusters(this->last_result.clusters);

    auto tracked_weights = this->weight_tracking.get_tracked_weights();

    WeightTrackingPayload weight_payload;
    weight_payload.count = tracked_weights.size();
    memcpy(weight_payload.targets, tracked_weights.data(),
           weight_payload.count * sizeof(WeightTrackedTarget));
    xQueueOverwrite(weight_tracking_queue, &weight_payload);

    telemetry::publish_tracked_weights(this->weight_tracking.get_tracked_weights());
    telemetry::publish_lidar_points(this->last_result.transformed_points);
    // telemetry::publish_lidar_processing(this->last_result); // Stack Overflowing at the moment
}

std::span<WeightTrackedTarget> LidarProcessingTask::get_tracked_weights() {
    return this->weight_tracking.get_tracked_weights();
}

#include "tasks/lidar_processing.hpp"
#include "lib/lidar_processing.hpp"
#include "telemetry_bus.hpp"

LidarProcessingTask::LidarProcessingTask(LidarTask *lidar_reader_task,
                                         PositionTrackingTask *position_tracking_task)
    : SchedulerTask("lidar_processing"), lidar_reader_task(lidar_reader_task),
      position_tracking_task(position_tracking_task) {}

void LidarProcessingTask::setup() {}

void LidarProcessingTask::loop() {
    auto points = lidar_reader_task->get_points();
    auto pose = position_tracking_task->get_current_pose();

    LidarProcessing processing;

    this->last_result = processing.process_points(points, pose);
    this->has_last_result = true;

    telemetry::publish_lidar_points(this->last_result.transformed_points);
    telemetry::publish_lidar_processing(this->last_result);
}

bool LidarProcessingTask::has_result() const { return this->has_last_result; }

const LidarProcessingResult &LidarProcessingTask::get_last_result() const {
    return this->last_result;
}

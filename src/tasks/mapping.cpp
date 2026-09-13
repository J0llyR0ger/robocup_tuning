#include "tasks/mapping.hpp"
#include "telemetry_bus.hpp"
#include <Arduino.h>

MappingTask::MappingTask(PositionTrackingTask *position_tracking_task, LidarTask *lidar_task)
    : SchedulerTask("mapping"), position_tracking_task(position_tracking_task),
      lidar_task(lidar_task) {}

void MappingTask::setup() { this->occupancy_grid.clear(); }

void MappingTask::loop() {
    static uint32_t next_grid_publish_ms = 0;

    Pose pose = this->position_tracking_task->get_current_pose();
    this->occupancy_grid.update_from_lidar(pose, this->lidar_task->get_points());

    this->occupancy_graph = OccupancyGridGraph(this->occupancy_grid);

    uint32_t now_ms = millis();
    if (now_ms >= next_grid_publish_ms) {
        telemetry::publish_occupancy_grid(this->occupancy_grid);
        next_grid_publish_ms = now_ms + 1000;
    }
}

const OccupancyGridMap &MappingTask::get_occupancy_grid() const { return this->occupancy_grid; }

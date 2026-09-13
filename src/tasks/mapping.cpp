#include "tasks/mapping.hpp"
#include "lib/a_star.hpp"
#include "lib/path_smoother.hpp"
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
    PathSmoother smoother = PathSmoother(this->occupancy_graph);

    int startX;
    int startY;

    this->occupancy_grid.world_to_grid(this->position_tracking_task->get_current_pose().position,
                                       startX, startY);

    int goalX = 40;
    int goalY = 70;

    OctileHeuristic heuristic(goalX, goalY);

    auto path = aStarSearch(this->occupancy_graph, OccupancyGridGraph::idx(startX, startY),
                            OccupancyGridGraph::idx(goalX, goalY), heuristic);

    auto simplified_path = smoother.simplify(path);

    smoother.smooth(simplified_path, this->discovery_path, /*samplesPerSegment=*/1);

    uint32_t now_ms = millis();
    if (now_ms >= next_grid_publish_ms) {
        telemetry::publish_occupancy_grid(this->occupancy_grid);
        telemetry::publish_grid_path(this->discovery_path);
        next_grid_publish_ms = now_ms + 1000;
    }
}

std::vector<Eigen::Vector2f> MappingTask::get_discovery_path() { return this->discovery_path; }

const OccupancyGridMap &MappingTask::get_occupancy_grid() const { return this->occupancy_grid; }

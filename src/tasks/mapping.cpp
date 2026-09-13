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

    int goalX = 40;
    int goalY = 90;

    OctileHeuristic heuristic(goalX, goalY);

    auto path = aStarSearch(this->occupancy_graph, OccupancyGridGraph::idx(10, 10),
                            OccupancyGridGraph::idx(goalX, goalY), heuristic);

    auto simplified_path = smoother.simplify(path);

    etl::vector<Eigen::Vector2f, 1024> smoothPath;
    smoother.smooth(simplified_path, smoothPath, /*samplesPerSegment=*/8);

    uint32_t now_ms = millis();
    if (now_ms >= next_grid_publish_ms) {
        telemetry::publish_occupancy_grid(this->occupancy_grid);
        telemetry::publish_grid_path(
            std::span<const Eigen::Vector2f>(smoothPath.data(), smoothPath.size()));
        next_grid_publish_ms = now_ms + 1000;
    }
}

const OccupancyGridMap &MappingTask::get_occupancy_grid() const { return this->occupancy_grid; }

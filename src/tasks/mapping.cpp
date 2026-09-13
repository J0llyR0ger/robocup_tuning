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

    auto robot_position = this->position_tracking_task->get_current_pose().position;

    this->discovery_path = get_path_between_world_points(
        robot_position, {FIELD_WIDTH_X_METERS / 2.0, FIELD_HEIGHT_Y_METERS / 2.0});

    this->home_path = get_path_between_world_points(robot_position, {0.5, 0.5});

    uint32_t now_ms = millis();
    if (now_ms >= next_grid_publish_ms) {
        telemetry::publish_occupancy_grid(this->occupancy_grid);
        telemetry::publish_grid_path(this->discovery_path);
        next_grid_publish_ms = now_ms + 1000;
    }
}

std::vector<Eigen::Vector2f> MappingTask::get_discovery_path() { return this->discovery_path; }
std::vector<Eigen::Vector2f> MappingTask::get_home_path() { return this->home_path; }

const OccupancyGridMap &MappingTask::get_occupancy_grid() const { return this->occupancy_grid; }

std::vector<Eigen::Vector2f> MappingTask::get_path_between_world_points(Eigen::Vector2f start,
                                                                        Eigen::Vector2f end) {
    PathSmoother smoother = PathSmoother(this->occupancy_graph);

    int startX, startY;

    this->occupancy_grid.world_to_grid(start, startX, startY);

    int goalX, goalY;
    this->occupancy_grid.world_to_grid(end, goalX, goalY);

    OctileHeuristic heuristic(goalX, goalY);

    auto path = aStarSearch(this->occupancy_graph, OccupancyGridGraph::idx(startX, startY),
                            OccupancyGridGraph::idx(goalX, goalY), heuristic);

    auto simplified_path = smoother.simplify(path);

    std::vector<Eigen::Vector2f> smoothed_path;

    smoother.smooth(simplified_path, smoothed_path, /*samplesPerSegment=*/8);

    return smoothed_path;
}

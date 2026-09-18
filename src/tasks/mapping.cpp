#include "tasks/mapping.hpp"
#include "lib/a_star.hpp"
#include "lib/path_smoother.hpp"
#include "telemetry_bus.hpp"
#include <Arduino.h>
#include <mutexes.hpp>
#include <queues.hpp>

MappingTask::MappingTask() : SchedulerTask("mapping") {}

void MappingTask::setup() { this->occupancy_grid.clear(); }

const float SIZE_DISTANCE_WEIGHTING = 0.5;

void MappingTask::loop() {
    static uint32_t next_grid_publish_ms = 0;

    Pose pose = get_global_pose();

    LidarScanPayload payload;
    xQueueReceive(lidarReader_MappingScanQueue, &payload, portMAX_DELAY);

    etl::vector<LidarResponsePoint, MAX_LIDAR_POINTS> points;
    points.assign(payload.points, payload.points + payload.count);

    this->occupancy_grid.update_from_lidar(pose, points);

    this->occupancy_graph = OccupancyGridGraph(this->occupancy_grid);

    float best_frontier_score = 0.0;

    int best_frontier_index = -1;

    for (int i = 0; i < this->occupancy_grid.get_frontier_clusters().size(); i++) {
        auto cluster = this->occupancy_grid.get_frontier_clusters()[i];

        float distance = (pose.position - cluster.centroid).norm();

        float score = (float)cluster.cell_count() / powf(distance, SIZE_DISTANCE_WEIGHTING);

        if (score > best_frontier_score) {
            best_frontier_score = score;
            best_frontier_index = i;
        }
    }

    if (best_frontier_index >= 0) {
        auto discovery_path = get_path_between_world_points(
            pose.position,

            this->occupancy_grid.get_frontier_clusters()[best_frontier_index].centroid);
        set_discovery_path(discovery_path);

        telemetry::publish_grid_path(discovery_path);
    } else {
        set_discovery_path({});
    }

    set_home_path(get_path_between_world_points(pose.position, {0.5, 0.5}));

    telemetry::publish_occupancy_grid(this->occupancy_grid);
}

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

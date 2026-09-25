#include "tasks/mapping.hpp"
#include "lib/a_star.hpp"
#include "lib/path_smoother.hpp"
#include "telemetry_bus.hpp"
#include "home_selection.hpp"
#include <Arduino.h>
#include <mutexes.hpp>
#include <queues.hpp>

MappingTask::MappingTask() : SchedulerTask("mapping") {}

void MappingTask::setup() {
    this->occupancy_grid.clear();
    this->home_position = home_start_pose(false).position;
    set_home_position(this->home_position);
}

const float SIZE_DISTANCE_WEIGHTING = 0.5;

void MappingTask::loop() {
    static uint32_t next_grid_publish_ms = 0;

    LidarScanPayload payload;
    xQueueReceive(lidarReader_MappingScanQueue, &payload, portMAX_DELAY);

    const uint32_t generation = home_request_generation.load();
    if (home_pose_generation.load() != generation) return;
    if (home_generation != generation) {
        const bool blue = active_home_blue.load();
        if (blue != applied_home_blue) {
            this->occupancy_grid.clear();
            applied_home_blue = blue;
        }
        this->home_position = home_start_pose(blue).position;
        set_home_position(this->home_position);
        set_home_path({});
        set_discovery_path({});
        set_motion_control_path({{}, 0.0f});
        home_generation = generation;
    }
    // Refresh from the latched team every cycle; cached startup coordinates
    // must never select the other team's destination.
    this->home_position = active_home_position();
    set_home_position(this->home_position);
    Pose pose = get_global_pose();

    etl::vector<LidarResponsePoint, MAX_LIDAR_POINTS> points;
    points.assign(payload.points, payload.points + payload.count);

    this->occupancy_grid.update_from_lidar(pose, points);
    telemetry::publish_weight_clusters(this->occupancy_grid.get_weight_clusters());

    WeightTrackingPayload weight_payload;
    weight_payload.count =
        std::min((int)this->occupancy_grid.get_weight_clusters().size(), WEIGHT_TARGET_MAX_TRACKS);

    for (int i = 0; i < weight_payload.count; i++) {
        weight_payload.targets[i] = this->occupancy_grid.get_weight_clusters()[i].centroid;
    }

    xQueueOverwrite(weight_tracking_queue, &weight_payload);

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

    auto home_path = get_path_between_world_points(pose.position, this->home_position);
    if (home_path.empty() && (pose.position - this->home_position).norm() > 0.1f) {
        // A* can temporarily have no route while the occupancy grid is still
        // being populated.  A direct home waypoint keeps return-to-base from
        // becoming a stationary command in that case.
        home_path.push_back(this->home_position);
    }
    set_home_path(home_path);
    // DriveTrainTask waits for both localization and fresh paths before enabling.
    home_map_generation.store(home_generation);

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

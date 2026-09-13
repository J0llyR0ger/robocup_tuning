#pragma once

#include "Eigen/Geometry"
#include "lib/occupancy_grid_graph.hpp"
#include "lib/occupancy_grid_map.hpp"
#include "scheduler_task.hpp"
#include "tasks/lidar.hpp"
#include "tasks/position_tracking.hpp"

class MappingTask : public SchedulerTask {
  private:
    PositionTrackingTask *position_tracking_task;
    LidarTask *lidar_task;

    OccupancyGridMap occupancy_grid = OccupancyGridMap();
    OccupancyGridGraph occupancy_graph = OccupancyGridGraph(occupancy_grid);

    std::vector<Eigen::Vector2f> discovery_path;
    std::vector<Eigen::Vector2f> home_path;

    std::vector<Eigen::Vector2f> get_path_between_world_points(Eigen::Vector2f start,
                                                               Eigen::Vector2f end);

  public:
    MappingTask(PositionTrackingTask *position_tracking_task, LidarTask *lidar_task);

    void setup() override;
    void loop() override;

    int get_frequency() const override { return MAPPING_TASK_FREQ; }

    std::vector<Eigen::Vector2f> get_discovery_path();
    std::vector<Eigen::Vector2f> get_home_path();

    const OccupancyGridMap &get_occupancy_grid() const;
};

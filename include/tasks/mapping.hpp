#pragma once

#include "Eigen/Geometry"
#include "lib/occupancy_grid_graph.hpp"
#include "lib/occupancy_grid_map.hpp"
#include "scheduler_task.hpp"
#include "tasks/lidar.hpp"
#include "tasks/position_tracking.hpp"

class MappingTask : public SchedulerTask {
  private:
    uint32_t home_generation = 0;
    bool applied_home_blue = false;
    OccupancyGridMap occupancy_grid = OccupancyGridMap();
    OccupancyGridGraph occupancy_graph = OccupancyGridGraph(occupancy_grid);
    Eigen::Vector2f home_position = Eigen::Vector2f::Zero();

    std::vector<Eigen::Vector2f> get_path_between_world_points(Eigen::Vector2f start,
                                                               Eigen::Vector2f end);

    const OccupancyGridMap &get_occupancy_grid() const;

  public:
    MappingTask();

    void setup() override;
    void loop() override;

    int get_frequency() const override { return MAPPING_TASK_FREQ; }

    uint32_t get_stack_depth() const override { return 1024 * 20; };
};

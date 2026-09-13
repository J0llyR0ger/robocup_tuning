#pragma once

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

  public:
    MappingTask(PositionTrackingTask *position_tracking_task, LidarTask *lidar_task);

    void setup() override;
    void loop() override;

    int get_frequency() const override { return MAPPING_TASK_FREQ; }

    const OccupancyGridMap &get_occupancy_grid() const;
};

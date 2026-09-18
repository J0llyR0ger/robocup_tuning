#pragma once

#include "config.hpp"
#include "lib/lidar.hpp"
#include "lib/odometry.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

class OccupancyGridMap {
  public:
    static constexpr float TILE_SIZE_METERS = 0.05f;

    static constexpr size_t GRID_WIDTH = FIELD_WIDTH_X_METERS / TILE_SIZE_METERS;
    static constexpr size_t GRID_HEIGHT = FIELD_HEIGHT_Y_METERS / TILE_SIZE_METERS;

    static constexpr uint8_t UNKNOWN_SCORE = 127;
    static constexpr uint8_t FREE_DECREMENT = 2;
    static constexpr uint8_t OCCUPIED_INCREMENT = 8;

    static constexpr size_t MIN_FRONTIER_CLUSTER_SIZE = 4;
    static constexpr uint8_t WEIGHT_CLUSTER_OCCUPIED_THRESHOLD = 200;
    static constexpr size_t MAX_WEIGHT_CLUSTER_SIZE = 4;
    static constexpr int WEIGHT_CLUSTER_CLEAR_RADIUS = 2;

    struct FrontierCell {
        int x = 0;
        int y = 0;
    };

    struct FrontierCluster {
        std::vector<FrontierCell> cells;
        Eigen::Vector2f centroid{0.0f, 0.0f};
        size_t cell_count() const { return this->cells.size(); }
    };

    using WeightCluster = FrontierCluster;

    OccupancyGridMap();

    void clear(uint8_t score = UNKNOWN_SCORE);

    void update_from_lidar(const Pose &robot_pose, std::span<const LidarResponsePoint> points);

    uint8_t get_score(size_t x, size_t y) const;
    const std::array<uint8_t, GRID_WIDTH * GRID_HEIGHT> &get_scores() const;

    size_t width() const;
    size_t height() const;
    float tile_size_meters() const;

    bool world_to_grid(const Eigen::Vector2f &position, int &grid_x, int &grid_y) const;

    const std::vector<FrontierCluster> &get_frontier_clusters() const;
    const std::vector<FrontierCluster> &frontiers() const;
    std::vector<FrontierCell> get_frontier_cells() const;
    std::vector<Eigen::Vector2f> get_frontier_points() const;
    std::vector<Eigen::Vector2f> get_frontier_centroids() const;

    const std::vector<WeightCluster> &get_weight_clusters() const;
    std::vector<WeightCluster> find_weight_clusters() const;

  private:
    std::array<uint8_t, GRID_WIDTH * GRID_HEIGHT> scores;
    std::vector<FrontierCluster> frontier_clusters;
    std::vector<WeightCluster> weight_clusters;

    static size_t to_index(size_t x, size_t y);

    void apply_beam(const Eigen::Vector2f &origin_world, const Eigen::Vector2f &hit_world);
    void update_frontiers();
    void update_weight_clusters();

    void increase_cell(int grid_x, int grid_y);
    void decrease_cell(int grid_x, int grid_y);
};

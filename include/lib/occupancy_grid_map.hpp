#pragma once

#include "config.hpp"
#include "lib/lidar.hpp"
#include "lib/odometry.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
class OccupancyGridMap {
  public:
    static constexpr float TILE_SIZE_METERS = 0.05f;

    static constexpr size_t GRID_WIDTH = FIELD_WIDTH_X_METERS / TILE_SIZE_METERS;
    static constexpr size_t GRID_HEIGHT = FIELD_HEIGHT_Y_METERS / TILE_SIZE_METERS;

    static constexpr uint8_t UNKNOWN_SCORE = 127;
    static constexpr uint8_t FREE_DECREMENT = 2;
    static constexpr uint8_t OCCUPIED_INCREMENT = 8;

    OccupancyGridMap();

    void clear(uint8_t score = UNKNOWN_SCORE);

    void update_from_lidar(const Pose &robot_pose, std::span<const LidarResponsePoint> points);

    uint8_t get_score(size_t x, size_t y) const;
    const std::array<uint8_t, GRID_WIDTH * GRID_HEIGHT> &get_scores() const;

    size_t width() const;
    size_t height() const;
    float tile_size_meters() const;

  private:
    std::array<uint8_t, GRID_WIDTH * GRID_HEIGHT> scores;

    static size_t to_index(size_t x, size_t y);

    bool world_to_grid(const Eigen::Vector2f &position, int &grid_x, int &grid_y) const;

    void apply_beam(const Eigen::Vector2f &origin_world, const Eigen::Vector2f &hit_world);

    void increase_cell(int grid_x, int grid_y);
    void decrease_cell(int grid_x, int grid_y);
};

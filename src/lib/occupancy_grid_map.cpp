#include "lib/occupancy_grid_map.hpp"
#include <algorithm>
#include <cmath>
#include <queue>
#include <vector>
#include <wiring.h>

const float MAX_EMPTY_CONFIRMATION_DISTANCE = 1.0;

namespace {

int abs_int(int value) { return value < 0 ? -value : value; }

bool in_bounds(int x, int y) {
    return x >= 0 && y >= 0 && x < (int)OccupancyGridMap::GRID_WIDTH &&
           y < (int)OccupancyGridMap::GRID_HEIGHT;
}

bool is_frontier_cell(
    const std::array<uint8_t, OccupancyGridMap::GRID_WIDTH * OccupancyGridMap::GRID_HEIGHT> &scores,
    int x, int y) {
    const size_t idx =
        static_cast<size_t>(y) * OccupancyGridMap::GRID_WIDTH + static_cast<size_t>(x);
    if (scores[idx] >= OccupancyGridMap::UNKNOWN_SCORE) {
        return false;
    }

    static const int8_t dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    static const int8_t dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

    for (int i = 0; i < 8; ++i) {
        int nx = x + dx[i];
        int ny = y + dy[i];
        if (!in_bounds(nx, ny)) {
            continue;
        }

        const size_t neighbor_idx =
            static_cast<size_t>(ny) * OccupancyGridMap::GRID_WIDTH + static_cast<size_t>(nx);
        if (scores[neighbor_idx] == OccupancyGridMap::UNKNOWN_SCORE) {
            return true;
        }
    }

    return false;
}

} // namespace

OccupancyGridMap::OccupancyGridMap() { this->clear(); }

void OccupancyGridMap::clear(uint8_t score) {
    this->scores.fill(score);
    this->frontier_clusters.clear();

    if (score != UNKNOWN_SCORE) {
        return;
    }

    constexpr float START_ZONE_SIZE_METERS = 0.65f;
    size_t start_zone_size_tiles = START_ZONE_SIZE_METERS / TILE_SIZE_METERS;

    // Starting zone cannot be wider than half the smallest dimension of the arena
    assert(start_zone_size_tiles <= std::min(GRID_WIDTH, GRID_HEIGHT) / 2);

    // Fill in the two home bases as empty
    for (size_t y = 0; y < start_zone_size_tiles; y++) {
        for (size_t x = 0; x < start_zone_size_tiles; x++) {
            this->scores[to_index(x, y)] = 0;
        }

        for (size_t x = GRID_WIDTH - start_zone_size_tiles; x < GRID_WIDTH; x++) {
            this->scores[to_index(x, y)] = 0;
        }
    }

    // Fill in the edge tiles as full
    for (size_t x = 0; x < GRID_WIDTH; x++) {
        this->scores[to_index(x, 0)] = 255;
        this->scores[to_index(x, GRID_HEIGHT - 1)] = 255;
    }

    for (size_t y = 0; y < GRID_HEIGHT; y++) {
        this->scores[to_index(0, y)] = 255;
        this->scores[to_index(GRID_WIDTH - 1, y)] = 255;
    }
}

void OccupancyGridMap::update_from_lidar(const Pose &robot_pose,
                                         std::span<const LidarResponsePoint> points) {
    float heading = robot_pose.heading;
    float heading_sin = sinf(heading);
    float heading_cos = cosf(heading);

    // Lidar points from the Lidar Task are corrected to be relative to robot frame

    // We must convert their position to world frame, and the origin must be the actual lidar
    // location

    Eigen::Vector2f lidar_origin_offset = {
        heading_cos * LIDAR_OFFSEST_X + heading_sin * LIDAR_OFFSET_Y,
        -heading_sin * LIDAR_OFFSEST_X + heading_cos * LIDAR_OFFSET_Y};

    Eigen::Vector2f lidar_origin = robot_pose.position + lidar_origin_offset;

    for (const LidarResponsePoint &point : points) {
        if (point.range <= 0.01f) {
            continue;
        }

        Eigen::Vector2f world_direction(
            heading_cos * point.position.x() + heading_sin * point.position.y(),
            -heading_sin * point.position.x() + heading_cos * point.position.y());

        Eigen::Vector2f hit_world = robot_pose.position + world_direction;

        this->apply_beam(lidar_origin, hit_world);
    }

    this->update_frontiers();
}

uint8_t OccupancyGridMap::get_score(size_t x, size_t y) const {
    if (x >= GRID_WIDTH || y >= GRID_HEIGHT) {
        return UNKNOWN_SCORE;
    }

    return this->scores[to_index(x, y)];
}

const std::array<uint8_t, OccupancyGridMap::GRID_WIDTH * OccupancyGridMap::GRID_HEIGHT> &
OccupancyGridMap::get_scores() const {
    return this->scores;
}

size_t OccupancyGridMap::width() const { return GRID_WIDTH; }

size_t OccupancyGridMap::height() const { return GRID_HEIGHT; }

float OccupancyGridMap::tile_size_meters() const { return TILE_SIZE_METERS; }

size_t OccupancyGridMap::to_index(size_t x, size_t y) { return y * GRID_WIDTH + x; }

bool OccupancyGridMap::world_to_grid(const Eigen::Vector2f &position, int &grid_x,
                                     int &grid_y) const {
    int x = (int)floorf(position.x() / TILE_SIZE_METERS);
    int y = (int)floorf(position.y() / TILE_SIZE_METERS);

    if (x < 0 || y < 0 || x >= (int)GRID_WIDTH || y >= (int)GRID_HEIGHT) {
        return false;
    }

    grid_x = x;
    grid_y = y;
    return true;
}

const std::vector<OccupancyGridMap::FrontierCluster> &
OccupancyGridMap::get_frontier_clusters() const {
    return this->frontier_clusters;
}

const std::vector<OccupancyGridMap::FrontierCluster> &OccupancyGridMap::frontiers() const {
    return this->frontier_clusters;
}

std::vector<OccupancyGridMap::FrontierCell> OccupancyGridMap::get_frontier_cells() const {
    std::vector<FrontierCell> frontier_cells;
    frontier_cells.reserve(this->frontier_clusters.size() * 8);

    for (const FrontierCluster &cluster : this->frontier_clusters) {
        frontier_cells.insert(frontier_cells.end(), cluster.cells.begin(), cluster.cells.end());
    }

    return frontier_cells;
}

void OccupancyGridMap::update_frontiers() {
    std::vector<FrontierCell> frontier_cells;
    frontier_cells.reserve(GRID_WIDTH * GRID_HEIGHT / 8);

    for (size_t y = 1; y + 1 < GRID_HEIGHT; ++y) {
        for (size_t x = 1; x + 1 < GRID_WIDTH; ++x) {
            if (!is_frontier_cell(this->scores, (int)x, (int)y)) {
                continue;
            }

            frontier_cells.push_back({(int)x, (int)y});
        }
    }

    this->frontier_clusters.clear();
    std::array<uint8_t, GRID_WIDTH * GRID_HEIGHT> visited{};

    static const int8_t dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    static const int8_t dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

    for (const FrontierCell &seed : frontier_cells) {
        const size_t seed_idx =
            static_cast<size_t>(seed.y) * GRID_WIDTH + static_cast<size_t>(seed.x);
        if (visited[seed_idx]) {
            continue;
        }

        std::queue<FrontierCell> queue;
        queue.push(seed);
        visited[seed_idx] = 1;

        FrontierCluster cluster;
        cluster.cells.reserve(8);

        while (!queue.empty()) {
            FrontierCell current = queue.front();
            queue.pop();

            cluster.cells.push_back(current);

            for (int i = 0; i < 8; ++i) {
                int nx = current.x + dx[i];
                int ny = current.y + dy[i];
                if (!in_bounds(nx, ny)) {
                    continue;
                }

                const size_t neighbor_idx =
                    static_cast<size_t>(ny) * GRID_WIDTH + static_cast<size_t>(nx);
                if (visited[neighbor_idx] || !is_frontier_cell(this->scores, nx, ny)) {
                    continue;
                }

                visited[neighbor_idx] = 1;
                queue.push({nx, ny});
            }
        }

        if (cluster.cells.size() < MIN_FRONTIER_CLUSTER_SIZE) {
            continue;
        }

        float centroid_x = 0.0f;
        float centroid_y = 0.0f;
        for (const FrontierCell &cell : cluster.cells) {
            centroid_x += static_cast<float>(cell.x) + 0.5f;
            centroid_y += static_cast<float>(cell.y) + 0.5f;
        }

        centroid_x *= TILE_SIZE_METERS / static_cast<float>(cluster.cells.size());
        centroid_y *= TILE_SIZE_METERS / static_cast<float>(cluster.cells.size());
        cluster.centroid = {centroid_x, centroid_y};

        this->frontier_clusters.push_back(cluster);
    }
}

void OccupancyGridMap::apply_beam(const Eigen::Vector2f &origin_world,
                                  const Eigen::Vector2f &hit_world) {
    int origin_x = 0;
    int origin_y = 0;
    int hit_x = 0;
    int hit_y = 0;

    if (!this->world_to_grid(origin_world, origin_x, origin_y)) {
        return;
    }

    bool endpoint_in_grid = this->world_to_grid(hit_world, hit_x, hit_y);

    if (!endpoint_in_grid) {
        hit_x = (int)floorf(hit_world.x() / TILE_SIZE_METERS);
        hit_y = (int)floorf(hit_world.y() / TILE_SIZE_METERS);
    }

    int x = origin_x;
    int y = origin_y;

    int dx = abs_int(hit_x - origin_x);
    int sx = origin_x < hit_x ? 1 : -1;
    int dy = -abs_int(hit_y - origin_y);
    int sy = origin_y < hit_y ? 1 : -1;
    int error = dx + dy;

    while (true) {
        if (endpoint_in_grid && x == hit_x && y == hit_y) {
            this->increase_cell(x, y);
            break;
        }

        int e2 = 2 * error;

        int next_x = x;
        int next_y = y;
        int next_error = error;

        if (e2 >= dy) {
            next_error += dy;
            next_x += sx;
        }

        if (e2 <= dx) {
            next_error += dx;
            next_y += sy;
        }

        bool next_out_of_bounds =
            next_x < 0 || next_y < 0 || next_x >= (int)GRID_WIDTH || next_y >= (int)GRID_HEIGHT;

        if (!endpoint_in_grid && next_out_of_bounds) {
            this->increase_cell(x, y);
            break;
        }

        // Maximum empty confirmation radius
        if (sqrtf((float)((x - origin_x) * (x - origin_x)) +
                  (float)((y - origin_y) * (y - origin_y))) *
                    TILE_SIZE_METERS <
                MAX_EMPTY_CONFIRMATION_DISTANCE ||
            get_score(x, y) > UNKNOWN_SCORE) {
            this->decrease_cell(x, y);
        }

        x = next_x;
        y = next_y;
        error = next_error;
    }
}

void OccupancyGridMap::increase_cell(int grid_x, int grid_y) {
    size_t idx = to_index((size_t)grid_x, (size_t)grid_y);
    uint16_t increased = (uint16_t)this->scores[idx] + OCCUPIED_INCREMENT;
    this->scores[idx] = (uint8_t)std::min<uint16_t>(255, increased);
}

void OccupancyGridMap::decrease_cell(int grid_x, int grid_y) {
    size_t idx = to_index((size_t)grid_x, (size_t)grid_y);
    uint8_t current = this->scores[idx];

    if (current <= FREE_DECREMENT) {
        this->scores[idx] = 0;
        return;
    }

    this->scores[idx] = (uint8_t)(current - FREE_DECREMENT);
}

std::vector<Eigen::Vector2f> OccupancyGridMap::get_frontier_points() const {
    std::vector<Eigen::Vector2f> points;
    points.reserve(this->frontier_clusters.size() * 8);

    for (const FrontierCluster &cluster : this->frontier_clusters) {
        for (const FrontierCell &cell : cluster.cells) {
            points.emplace_back((static_cast<float>(cell.x) + 0.5f) * TILE_SIZE_METERS,
                                (static_cast<float>(cell.y) + 0.5f) * TILE_SIZE_METERS);
        }
    }

    return points;
}

std::vector<Eigen::Vector2f> OccupancyGridMap::get_frontier_centroids() const {
    std::vector<Eigen::Vector2f> centroids;
    centroids.reserve(this->frontier_clusters.size());

    for (const FrontierCluster &cluster : this->frontier_clusters) {
        centroids.push_back(cluster.centroid);
    }

    return centroids;
}

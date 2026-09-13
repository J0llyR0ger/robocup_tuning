#include "lib/lidar_processing.hpp"
#include "etl/vector.h"
#include <Eigen/QR>
#include <algorithm>
#include <cmath>
#include <lib/geometry.hpp>

bool coarse_cluster_check(LidarResponsePoint point1, LidarResponsePoint point2) {
    float threshold = std::min(point1.range, point2.range) * COARSE_THRESHOLD_RANGE_MULTIPLIER +
                      COARSE_THRESHOLD_OFFSET;

    return (point1.position - point2.position).norm() < threshold;
}

void summarize_cluster(Cluster &cluster, std::span<const LidarResponsePoint> points) {
    cluster.centroid = Eigen::Vector2f::Zero();
    cluster.range = 0.0f;
    cluster.spread = 0.0f;
    cluster.max_extent = 0.0f;
    cluster.diameter_mm = 0.0f;
    cluster.aspect_ratio = 1.0f;

    if (cluster.count == 0 || cluster.start + cluster.count > points.size()) {
        return;
    }

    std::span<const LidarResponsePoint> cluster_points(points.data() + cluster.start,
                                                       cluster.count);

    Eigen::Vector2f centroid = Eigen::Vector2f::Zero();
    float range = 0.0f;

    for (const auto &point : cluster_points) {
        centroid += point.position;
        range += point.range;
    }

    centroid /= (float)cluster_points.size();
    range /= (float)cluster_points.size();

    cluster.centroid = centroid;
    cluster.range = range;

    float min_x = INFINITY;
    float max_x = -INFINITY;
    float min_y = INFINITY;
    float max_y = -INFINITY;
    float sum_squared_distance = 0.0f;

    for (const auto &point : cluster_points) {
        const Eigen::Vector2f delta = point.position - centroid;

        sum_squared_distance += delta.squaredNorm();

        min_x = std::min(min_x, point.position.x());
        max_x = std::max(max_x, point.position.x());
        min_y = std::min(min_y, point.position.y());
        max_y = std::max(max_y, point.position.y());
    }

    cluster.spread = std::sqrt(sum_squared_distance / (float)cluster_points.size());
    cluster.max_extent = std::max(max_x - min_x, max_y - min_y);
    cluster.diameter_mm = cluster.max_extent * 1000.0f;

    const float minor_extent = std::min(max_x - min_x, max_y - min_y);
    cluster.aspect_ratio = minor_extent > 1e-3f ? (cluster.max_extent / minor_extent) : 1.0f;
}

ClusterList get_coarse_clusters(std::span<LidarResponsePoint> points) {
    ClusterList clusters;

    if (points.empty()) {
        return clusters;
    }

    size_t start = 0, count = 1;

    for (size_t i = 1; i < points.size(); i++) {
        if (coarse_cluster_check(points[i], points[start + count - 1])) {
            count++;
        } else {
            clusters.push_back({start, count});
            start = i;
            count = 1;
        }
    }

    clusters.push_back({start, count});

    // TODO: This will need fixing

    // if (clusters.size() >= 2) {
    //     auto first = clusters.front();
    //     auto last = clusters.back();

    //     if (coarse_cluster_check(points[first.start], points[last.start + last.count - 1])) {
    //         std::rotate(points.begin(), points.begin() + first.count, points.end());

    //         // Every span computed before the rotate is now stale — must rebuild.
    //         ClusterList updated = {{.start = points.size() - last.count - first.count,
    //                                 .count = last.count + first.count}};

    //         size_t offset = updated[0].count;

    //         for (size_t k = 1; k + 1 < clusters.size(); k++) {
    //             updated.push_back({offset, clusters[k].count});
    //             offset += clusters[k].count;
    //         }

    //         clusters = updated;
    //     }
    // }

    for (auto &cluster : clusters) {
        summarize_cluster(cluster, points);
    }

    return clusters;
}

const float LINE_NOISE_THRESHOLD = 0.02f;

void fit_clusters(const ClusterList &coarse_clusters, std::span<const LidarResponsePoint> points,
                  LidarProcessingResult &result) {
    ClusterList work_stack;

    for (auto s : coarse_clusters) {
        work_stack.push_back(s);
    }

    while (!work_stack.empty()) {
        auto range = work_stack.back();
        work_stack.pop_back();

        if (range.count >= MIN_POINTS_PER_OBJECT) {
            auto circle_fit = fit_circle(points, range);

            if (MIN_CIRCLE_RADIUS < circle_fit.r && circle_fit.r < MAX_CIRCLE_RADIUS &&
                circle_fit.radius_deviation < MAX_CIRCLE_NOISE) {
                result.circles.push_back(circle_fit);

                continue;
            }
        }

        auto line = fit_line(points, range);

        float norm = sqrtf(line.slope * line.slope + 1);
        float furthest_distance = 0.0f;

        size_t split = 0;

        for (int i = 0; i < range.count; i++) {
            auto p = points[range.start + i].position;

            float distance = fabsf(line.slope * p.x() - p.y() + line.intercept) / norm;

            if (distance > furthest_distance) {
                furthest_distance = distance;
                split = i;
            }
        }

        if (furthest_distance <= LINE_NOISE_THRESHOLD) {
            result.line_segments.push_back(line);
            continue;
        }

        Cluster first, second;

        if (split == range.count - 1) {
            // Furthest point is the last point in range. subspan(0, split+1) would be
            // the whole range and subspan(split+1) would be empty — infinite loop.

            first = {range.start, split};
            second = {range.start + split, 0};
        } else {
            first = {range.start, split + 1};
            second = {range.start + split + 1, range.count - split + 1};
        }

        if (first.count >= MIN_POINTS_PER_OBJECT) {
            work_stack.push_back(first);
        }

        if (second.count >= MIN_POINTS_PER_OBJECT) {
            work_stack.push_back(second);
        }
    }
}

void LidarProcessing::process_points(std::span<LidarResponsePoint> points, Pose robot_pose,
                                     LidarProcessingResult &result) {
    result.line_segments.clear();
    result.circles.clear();
    result.clusters.clear();
    result.transformed_points.clear();

    std::sort(points.begin(), points.end(),
              [](const auto &a, const auto &b) { return a.angle < b.angle; });

    const float heading = robot_pose.heading;
    const float heading_sin = sinf(heading);
    const float heading_cos = cosf(heading);

    for (auto &point : points) {
        const float robot_x = point.position.x();
        const float robot_y = point.position.y();

        const float world_x =
            robot_pose.position.x() + heading_cos * robot_x + heading_sin * robot_y;
        const float world_y =
            robot_pose.position.y() - heading_sin * robot_x + heading_cos * robot_y;

        point.position = Eigen::Vector2f(world_x, world_y);
    }

    result.clusters = get_coarse_clusters(points);

    fit_clusters(result.clusters, points, result);

    result.transformed_points.assign(points.begin(), points.end());
}
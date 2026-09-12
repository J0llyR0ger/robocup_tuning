#include "lib/lidar_processing.hpp"
#include "etl/vector.h"
#include <Eigen/QR>
#include <lib/geometry.hpp>

bool coarse_cluster_check(LidarResponsePoint point1, LidarResponsePoint point2) {
    float threshold = std::min(point1.range, point2.range) * COARSE_THRESHOLD_RANGE_MULTIPLIER +
                      COARSE_THRESHOLD_OFFSET;

    return (point1.position - point2.position).norm() < threshold;
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

    return clusters;
}

const float LINE_NOISE_THRESHOLD = 0.02f;

LidarProcessingResult fit_clusters(const ClusterList &coarse_clusters,
                                   etl::vector<LidarResponsePoint, MAX_LIDAR_POINTS> points) {
    ClusterList work_stack;
    etl::vector<LineFit, MAX_LIDAR_POINTS> line_fits;
    etl::vector<CircleFit, MAX_LIDAR_POINTS> circle_fits;

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
                circle_fits.push_back(circle_fit);

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
            line_fits.push_back(line);
            continue;
        }

        PointSpan first, second;

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

    return {line_fits, circle_fits};
}

LidarProcessingResult LidarProcessing::process_points(std::span<LidarResponsePoint> unsorted_points,
                                                      const Pose &robot_pose) {
    etl::vector<LidarResponsePoint, MAX_LIDAR_POINTS> points(unsorted_points.begin(),
                                                             unsorted_points.end());

    std::sort(points.begin(), points.end(), [](auto a, auto b) { return a.angle < b.angle; });

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

    ClusterList coarse_clusters = get_coarse_clusters(points);

    auto result = fit_clusters(coarse_clusters, points);
    result.clusters = coarse_clusters;
    result.transformed_points = points;

    return result;
}
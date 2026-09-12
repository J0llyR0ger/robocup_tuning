#pragma once

#include "config.hpp"
#include "etl/vector.h"
#include "lib/lidar.hpp"
#include "lib/odometry.hpp"

struct LineFit {
    float x1;
    float x2;
    float y1;
    float y2;

    float slope;
    float intercept;
};

struct CircleFit {
    float xc, yc, r;

    int worst_index;
    float worst_residual;

    float radius_deviation;
};

struct PointSpan {
    size_t start;
    size_t count;
};

// Worst case every point is its own cluster, so cap at MAX_POINTS
using ClusterList = etl::vector<PointSpan, MAX_LIDAR_POINTS>;

struct LidarProcessingResult {
    etl::vector<LineFit, MAX_LIDAR_POINTS> line_segments;
    etl::vector<CircleFit, MAX_LIDAR_POINTS> circles;
    ClusterList clusters;
    etl::vector<LidarResponsePoint, MAX_LIDAR_POINTS> transformed_points;
};

class LidarProcessing {
  public:
    LidarProcessingResult process_points(std::span<LidarResponsePoint> points,
                                         const Pose &robot_pose);
};
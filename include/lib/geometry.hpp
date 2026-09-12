#pragma once

#include "lib/lidar_processing.hpp"
#include "lidar.hpp"

LineFit fit_line(etl::vector<LidarResponsePoint, MAX_LIDAR_POINTS> points, Cluster range);

CircleFit fit_circle(etl::vector<LidarResponsePoint, MAX_LIDAR_POINTS> points, Cluster range);

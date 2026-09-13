#pragma once

#include "lib/lidar_processing.hpp"
#include "lidar.hpp"

LineFit fit_line(std::span<const LidarResponsePoint> points, Cluster range);

CircleFit fit_circle(std::span<const LidarResponsePoint> points, Cluster range);

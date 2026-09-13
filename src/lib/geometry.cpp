#include "lib/geometry.hpp"

LineFit fit_line(std::span<const LidarResponsePoint> points, Cluster range) {
    float sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;

    for (int i = range.start; i < range.start + range.count; i++) {
        auto &p = points[i].position;

        sum_x += p.x();
        sum_y += p.y();
        sum_xy += p.x() * p.y();
        sum_xx += p.x() * p.x();
    }

    int n = range.count;

    float denom = n * sum_xx - sum_x * sum_x;

    auto start_point = points[range.start].position;
    auto end_point = points[range.start + range.count - 1].position;

    if (fabsf(denom) < 1e-6f)
        // TODO: Return start/end points
        return {0.0, 0.0, 0.0, 0.0, 0.0f, sum_y / n}; // degenerate/near-vertical guard

    float slope = (n * sum_xy - sum_x * sum_y) / denom;
    float intercept = (sum_y - slope * sum_x) / n;

    float x1 =
        (start_point.x() + slope * start_point.y() - slope * intercept) / (1 + slope * slope);

    float x2 = (end_point.x() + slope * end_point.y() - slope * intercept) / (1 + slope * slope);

    float y1 = (slope * start_point.x() + slope * slope * start_point.y() + intercept) /
               (1 + slope * slope);

    float y2 =
        (slope * end_point.x() + slope * slope * end_point.y() + intercept) / (1 + slope * slope);

    return {x1, x2, y1, y2, slope, intercept};
}

CircleFit fit_circle(std::span<const LidarResponsePoint> points, Cluster range) {
    assert(range.count >= 3);

    float sum_x = 0, sum_y = 0;

    for (size_t i = range.start; i < range.start + range.count; ++i) {
        sum_x += points[i].position.x();
        sum_y += points[i].position.y();
    }

    float mean_x = sum_x / range.count;
    float mean_y = sum_y / range.count;

    // Compute moments (Kasa/Pratt algebraic reduction)
    float Mxx = 0, Myy = 0, Mxy = 0, Mxz = 0, Myz = 0, Mzz = 0;

    for (size_t i = range.start; i < range.start + range.count; ++i) {
        float xi = points[i].position.x() - mean_x;
        float yi = points[i].position.y() - mean_y;
        float zi = xi * xi + yi * yi;

        Mxx += xi * xi;
        Myy += yi * yi;
        Mxy += xi * yi;
        Mxz += xi * zi;
        Myz += yi * zi;
        Mzz += zi * zi;
    }

    // Solving parameters via simplified 3x3 determinant/matrix reduction...
    // Center calculation (Algebraic approximation)
    float denominator = 2.0 * (Mxx * Myy - Mxy * Mxy);
    float xc = 0.0;
    float yc = 0.0;

    if (denominator != 0.0) {
        xc = (Myy * Mxz - Mxy * Myz) / denominator;
        yc = (Mxx * Myz - Mxy * Mxz) / denominator;
    }

    float cx = xc + mean_x;
    float cy = yc + mean_y;

    // Radius calculation

    int furthest_away = -1;
    float furthest_distance = 0;

    float radius_sum = 0;
    float sq_radius_sum = 0;

    for (size_t i = range.start; i < range.start + range.count; ++i) {
        float dx = points[i].position.x() - cx;
        float dy = points[i].position.y() - cy;

        float dist = std::sqrt(dx * dx + dy * dy);

        if (dist > furthest_distance) {
            furthest_away = i;
            furthest_distance = dist;
        }

        radius_sum += dist;
        sq_radius_sum += dist * dist;
    }

    float r = radius_sum / range.count;

    float variance = (sq_radius_sum / range.count) - (r * r);

    return {cx, cy, r, furthest_away, furthest_distance, std::sqrt(variance)};
}
#pragma once
#undef B1

#include <Eigen/Geometry>

static float CENTER_OFFSET = 0.22;

struct Pose {
    Eigen::Vector2f position;
    float heading;

    Pose get_center_pose() {
        return {
            position + get_direction_vector() * CENTER_OFFSET,
            heading,
        };
    }

    Eigen::Vector2f get_direction_vector() {
        return Eigen::Rotation2Df(std::numbers::pi / 2.0 - heading) * Eigen::Vector2f::UnitX();
    }
};

struct TrackingWheel {
    Eigen::Vector2f location;
    Eigen::Vector2f direction;
};

#define NUM_WHEELS 2

class OdometryModule {
  private:
    std::array<TrackingWheel, NUM_WHEELS> tracking_wheels;

  public:
    OdometryModule(const std::array<TrackingWheel, NUM_WHEELS> &tracking_wheels);

    Eigen::Matrix<float, NUM_WHEELS, 2> weighting_matrix;
    Eigen::Vector2f compute_travel(float wheel_travels[NUM_WHEELS], float angle_change);
};

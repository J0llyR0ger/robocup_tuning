#include "tasks/position_tracking.hpp"
#include "lib/odometry.hpp"
#include "telemetry_bus.hpp"
#include "utils.hpp"
#include <mutexes.hpp>
#include <queues.hpp>
#undef B1
#include <Eigen/Geometry>

PositionTrackingTask::PositionTrackingTask()
    : SchedulerTask("position_tracking"),
      odometry(OdometryModule({{
          {Eigen::Vector2f(-DRIVE_WIDTH_MM / 2.0f, 0.0f), Eigen::Vector2f(0.0f, 1.0f)},
          {Eigen::Vector2f(DRIVE_WIDTH_MM / 2.0f, 0.0f), Eigen::Vector2f(0.0f, 1.0f)},
      }})),
      mcl(FieldMap::make_rectangle(FIELD_WIDTH_X_METERS, FIELD_HEIGHT_Y_METERS)) {}

void PositionTrackingTask::setup() {
    xQueueReceive(imu_positionTrackingHeadingQueue, &this->last_heading, portMAX_DELAY);

    Pose initial_pose = {.position = Eigen::Vector2f(INITIAL_X, INITIAL_Y),
                         .heading = INITIAL_HEADING};

    this->mcl.set_initial_pose(initial_pose, INITIAL_POSITION_NOISE, INITIAL_HEADING_NOISE);
    this->last_mcl_position = initial_pose.position;
    set_global_pose(initial_pose);
}

void PositionTrackingTask::loop() {
    // Odometry can be run at a faster rate than position resampling
    while (uxQueueMessagesWaiting(lidarReader_PositionTrackingScanQueue) == 0) {
        std::tuple<float, float> wheel_positions;

        xQueueReceive(driveTrain_positionTrackingWheelPositionQueue, &wheel_positions,
                      portMAX_DELAY);

        auto [left_wheel_position, right_wheel_position] = wheel_positions;

        float left_change = left_wheel_position - this->last_left_wheel_position;
        float right_change = right_wheel_position - this->last_right_wheel_position;

        float wheel_travels[2] = {
            left_change * WHEEL_RADIUS_MM,
            right_change * WHEEL_RADIUS_MM
        };

        float current_heading;

        xQueueReceive(imu_positionTrackingHeadingQueue, &current_heading, portMAX_DELAY);

        float heading_change = diff_angle(last_heading, current_heading);

        Eigen::Vector2f robot_travel =
            this->odometry.compute_travel(wheel_travels, heading_change) * 1e-3;

        this->accumulated_odometry_distance += robot_travel.norm();

        this->mcl.predict(robot_travel, heading_change);

        this->last_left_wheel_position = left_wheel_position;
        this->last_right_wheel_position = right_wheel_position;
        this->last_heading = current_heading;

        set_global_pose(this->mcl.get_estimated_pose());
    }

    LidarScanPayload payload;
    xQueueReceive(lidarReader_PositionTrackingScanQueue, &payload, portMAX_DELAY);

    etl::vector<LidarResponsePoint, MAX_LIDAR_POINTS> points;
    points.assign(payload.points, payload.points + payload.count);

    this->mcl.update_beam_model(points);

    auto current_pose = this->mcl.get_estimated_pose();

    float mcl_distance =
        (current_pose.position - this->last_mcl_position).norm();

    float odometry_distance =
        this->accumulated_odometry_distance;

    bool motion_mismatch =
        odometry_distance < 0.0060f &&
        mcl_distance > 0.015f;

    Serial.print("ODO: ");
    Serial.print(odometry_distance * 1000.0f);
    Serial.print(" mm | MCL: ");
    Serial.print(mcl_distance * 1000.0f);
    Serial.print(" mm | Ratio: ");

    if (odometry_distance > 0.0f) {
        Serial.print(mcl_distance / odometry_distance);
    } else {
        Serial.print(0.0f);
    }

    Serial.print(" | Mismatch: ");
    Serial.println(motion_mismatch ? "YES" : "NO");

    set_robot_motion_mismatch(motion_mismatch);

    this->accumulated_odometry_distance = 0.0f;
    this->last_mcl_position = current_pose.position;

    telemetry::publish_f32(telemetry::KEY_HEADING, current_pose.heading);
    telemetry::publish_f32(telemetry::KEY_POSITION_X, current_pose.position.x());
    telemetry::publish_f32(telemetry::KEY_POSITION_Y, current_pose.position.y());
    telemetry::publish_f32(telemetry::KEY_POSITION_UNCERTAINTY,
                           this->mcl.get_position_uncertainty());

    set_global_pose(current_pose);
}

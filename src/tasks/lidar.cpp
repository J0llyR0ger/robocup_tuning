#include "tasks/lidar.hpp"
#include "Eigen/Geometry"
#include "queues.hpp"
#include <wiring.h>

LidarTask::LidarTask() : SchedulerTask("lidar_reading_task") {}

static uint8_t SERIAL_MEMORY[200];

constexpr uint32_t LIDAR_TELEMETRY_PERIOD_MICROS = 100;
constexpr float LIDAR_REPLACEMENT_ANGLE_EPSILON = 1.0f * DEG_TO_RAD;

float wrap_angle_positive(float angle) {
    float wrapped = fmodf(angle, 2.0 * PI);

    if (wrapped < 0.0f) {
        wrapped += 2.0 * PI;
    }

    return wrapped;
}

LidarResponsePoint to_robot_frame(const LidarResponsePoint &point) {
    auto rotated = Eigen::Rotation2Df(-LIDAR_ANGLE * DEG_TO_RAD) * point.position;

    Eigen::Vector2f offset = {LIDAR_OFFSEST_X, LIDAR_OFFSET_Y};

    return {
        .position = rotated + offset,
        .intensity = point.intensity,
        .angle = point.angle,
        .range = point.range,
    };
}

float smallest_angular_difference(float a, float b) {
    float diff = wrap_angle_positive(a) - wrap_angle_positive(b);

    if (diff > PI) {
        diff -= 2.0f * PI;
    } else if (diff < -PI) {
        diff += 2.0f * PI;
    }

    return fabsf(diff);
}

void upsert_point_by_angle(etl::vector<LidarResponsePoint, MAX_LIDAR_POINTS> &points,
                           const LidarResponsePoint &new_point) {
    if (points.size() > 350) {
        points.erase(points.begin());
    }

    points.push_back(new_point);
}

bool is_angle_valid(float angle_radians) {
    float angle = wrap_angle_positive(angle_radians);
    float start = wrap_angle_positive(LIDAR_START_ANGLE * DEG_TO_RAD);
    float end = wrap_angle_positive(LIDAR_END_ANGLE * DEG_TO_RAD);

    if (start <= end) {
        return angle >= start && angle <= end;
    }

    return angle >= start || angle <= end;
}

void LidarTask::setup() {
    Serial2.begin(230400);
    Serial2.addMemoryForRead(SERIAL_MEMORY, sizeof(SERIAL_MEMORY));
    Serial2.setTimeout(0);
}

void LidarTask::loop() {
    static uint8_t buffer[100];
    static uint32_t next_lidar_telemetry_publish = 0;

    int available = Serial2.available();

    if (available <= 0) {
        return;
    }

    size_t to_read = std::min((size_t)available, sizeof(buffer));

    int length = Serial2.readBytes(buffer, to_read);
    if (length <= 0) {
        return;
    }

    auto response = this->reader.read_span({buffer, length});

    std::visit(
        [this](auto &&arg) {
            using T = std::decay_t<decltype(arg)>;

            if constexpr (std::is_same_v<T, std::optional<LidarResponseData>>) {
                if (arg.has_value()) {
                    for (int i = 0; i < POINTS_PER_PACK; i++) {
                        auto point = (*arg).points[i];

                        float point_angle = point.angle;

                        if (!is_angle_valid(point_angle)) {
                            continue;
                        }

                        if (point.range < 0.01) {
                            continue;
                        }

                        LidarResponsePoint corrected = to_robot_frame(point);
                        upsert_point_by_angle(this->points, corrected);
                    }

                    xQueueSendToFront(lidarReader_lidarProcessingScanQueue, &this->points, 0);
                }
            } else if constexpr (std::is_same_v<T, PacketParseError>) {
                switch (arg) {
                case PacketParseError::BufferFull:
                    log_err("Serial Buffer Full");
                    break;
                case PacketParseError::IncorrectSize:
                    log_err("Incorrect data size");
                    break;
                case PacketParseError::CrcError:
                    // These are common so we ignore
                    // log_err("CRC Error");
                    break;
                }
            }
        },
        response);
}

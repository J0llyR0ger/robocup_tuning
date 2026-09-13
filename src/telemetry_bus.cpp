#include "telemetry_bus.hpp"

#include <config.hpp>
#include <cstring>
#include <lib/lidar_processing.hpp>
#include <lib/weight_tracking.hpp>

namespace telemetry {

namespace {

constexpr uint8_t FRAME_PREAMBLE[4] = {0xAB, 0xCD, 0xEF, 0x42};
constexpr uint8_t FRAME_VERSION = 1;

enum class FrameType : uint8_t {
    Values = 1,
    LidarPoints = 2,
    LidarProcessing = 3,
    OccupancyGrid = 4,
    TrackedWeights = 5,
    GridPath = 6,
};

struct __attribute__((packed)) FrameHeader {
    uint8_t preamble[4];
    uint8_t frame_type;
    uint8_t version;
    uint16_t payload_len;
};

struct __attribute__((packed)) ValueEntry {
    uint16_t key;
    uint8_t type;
    uint8_t flags;
    uint32_t payload;
};

struct __attribute__((packed)) LidarPointEntry {
    int16_t x_mm;
    int16_t y_mm;
    uint8_t intensity;
    uint8_t flags;
};

struct __attribute__((packed)) TrackedWeightEntry {
    float x_m;
    float y_m;
    float confidence;
};

constexpr size_t MAX_PENDING_VALUES = 128;
constexpr size_t MAX_GRID_PATH_NODES_PER_FRAME = 512;

Stream *TELEMETRY_PORT = &Serial;
ValueEntry PENDING_VALUES[MAX_PENDING_VALUES];
size_t PENDING_COUNT = 0;

template <typename T> uint32_t to_u32_bits(T value) {
    uint32_t bits = 0;
    static_assert(sizeof(T) <= sizeof(bits));
    memcpy(&bits, &value, sizeof(T));
    return bits;
}

void write_frame(FrameType frame_type, const uint8_t *payload, uint16_t payload_len) {
    FrameHeader header = {
        .preamble = {FRAME_PREAMBLE[0], FRAME_PREAMBLE[1], FRAME_PREAMBLE[2], FRAME_PREAMBLE[3]},
        .frame_type = static_cast<uint8_t>(frame_type),
        .version = FRAME_VERSION,
        .payload_len = payload_len,
    };

    if (ENABLE_TASK_LOGGING) {
        return;
    }

    TELEMETRY_PORT->write(reinterpret_cast<uint8_t *>(&header), sizeof(header));
    TELEMETRY_PORT->write(payload, payload_len);
}

bool upsert_value(uint16_t key, ValueType type, uint32_t payload) {
    for (size_t i = 0; i < PENDING_COUNT; i++) {
        if (PENDING_VALUES[i].key == key) {
            PENDING_VALUES[i].type = static_cast<uint8_t>(type);
            PENDING_VALUES[i].payload = payload;
            return true;
        }
    }

    if (PENDING_COUNT >= MAX_PENDING_VALUES) {
        return false;
    }

    PENDING_VALUES[PENDING_COUNT++] = {
        .key = key,
        .type = static_cast<uint8_t>(type),
        .flags = 0,
        .payload = payload,
    };

    return true;
}

} // namespace

void begin(Stream *serial_port) {
    if (serial_port != nullptr) {
        TELEMETRY_PORT = serial_port;
    }
}

bool publish_f32(uint16_t key, float value) {
    return upsert_value(key, ValueType::Float32, to_u32_bits(value));
}

bool publish_i32(uint16_t key, int32_t value) {
    return upsert_value(key, ValueType::Int32, to_u32_bits(value));
}

bool publish_u32(uint16_t key, uint32_t value) {
    return upsert_value(key, ValueType::UInt32, to_u32_bits(value));
}

bool publish_bool(uint16_t key, bool value) {
    return upsert_value(key, ValueType::Bool, value ? 1 : 0);
}

size_t flush_values(uint16_t max_entries_per_frame) {
    if (PENDING_COUNT == 0 || max_entries_per_frame == 0) {
        return 0;
    }

    if (max_entries_per_frame > 32) {
        max_entries_per_frame = 32;
    }

    size_t to_send = PENDING_COUNT;
    if (to_send > max_entries_per_frame) {
        to_send = max_entries_per_frame;
    }

    const uint16_t payload_len =
        static_cast<uint16_t>(sizeof(uint16_t) + to_send * sizeof(ValueEntry));
    uint8_t payload[sizeof(uint16_t) + 32 * sizeof(ValueEntry)] = {0};

    uint16_t count = static_cast<uint16_t>(to_send);
    memcpy(payload, &count, sizeof(count));
    memcpy(payload + sizeof(uint16_t), PENDING_VALUES, to_send * sizeof(ValueEntry));

    write_frame(FrameType::Values, payload, payload_len);

    size_t remaining = PENDING_COUNT - to_send;
    if (remaining > 0) {
        memmove(PENDING_VALUES, PENDING_VALUES + to_send, remaining * sizeof(ValueEntry));
    }
    PENDING_COUNT = remaining;

    return to_send;
}

void publish_lidar_points(std::span<LidarResponsePoint> points) {
    LidarPointEntry packed_points[MAX_LIDAR_POINTS] = {};

    for (int i = 0; i < points.size(); i++) {
        auto point = points[i];

        packed_points[i] = {
            .x_mm = (int16_t)(point.position.x() * 1000.0f),
            .y_mm = (int16_t)(point.position.y() * 1000.0f),
            .intensity = point.intensity,
            .flags = 0,
        };
    }

    uint16_t count = points.size();

    constexpr uint16_t MAX_PAYLOAD_LEN =
        sizeof(uint16_t) + MAX_LIDAR_POINTS * sizeof(LidarPointEntry);
    uint8_t payload[MAX_PAYLOAD_LEN] = {0};

    memcpy(payload, &count, sizeof(count));
    memcpy(payload + sizeof(count), packed_points, count * sizeof(LidarPointEntry));

    write_frame(FrameType::LidarPoints, payload, sizeof(count) + count * sizeof(LidarPointEntry));
}

struct __attribute__((packed)) TelemetryCluster {
    uint16_t start;
    uint16_t count;
    float centroid_x;
    float centroid_y;
    float range;
    float spread;
    float max_extent;
    float diameter_mm;
    float aspect_ratio;
};

void publish_lidar_processing(LidarProcessingResult result) {
    uint16_t lines_count = result.line_segments.size();
    LineFit line_fits[MAX_LIDAR_POINTS] = {};
    uint16_t clusters_count = result.clusters.size();
    TelemetryCluster clusters[MAX_LIDAR_POINTS] = {};

    for (int i = 0; i < result.line_segments.size(); i++) {
        line_fits[i] = result.line_segments[i];
    }

    for (int i = 0; i < result.clusters.size(); i++) {
        const auto &cluster = result.clusters[i];

        clusters[i] = {
            .start = static_cast<uint16_t>(cluster.start),
            .count = static_cast<uint16_t>(cluster.count),
            .centroid_x = cluster.centroid.x(),
            .centroid_y = cluster.centroid.y(),
            .range = cluster.range,
            .spread = cluster.spread,
            .max_extent = cluster.max_extent,
            .diameter_mm = cluster.diameter_mm,
            .aspect_ratio = cluster.aspect_ratio,
        };
    }

    constexpr uint16_t MAX_PAYLOAD_LEN = sizeof(lines_count) + MAX_LIDAR_POINTS * sizeof(LineFit) +
                                         sizeof(clusters_count) +
                                         MAX_LIDAR_POINTS * sizeof(TelemetryCluster);

    uint8_t payload[MAX_PAYLOAD_LEN] = {0};

    memcpy(payload, &lines_count, sizeof(lines_count));
    memcpy(payload + sizeof(lines_count), line_fits, lines_count * sizeof(LineFit));
    memcpy(payload + sizeof(lines_count) + lines_count * sizeof(LineFit), &clusters_count,
           sizeof(clusters_count));
    memcpy(payload + sizeof(lines_count) + lines_count * sizeof(LineFit) + sizeof(clusters_count),
           clusters, clusters_count * sizeof(TelemetryCluster));

    write_frame(FrameType::LidarProcessing, payload,
                sizeof(lines_count) + lines_count * sizeof(LineFit) + sizeof(clusters_count) +
                    clusters_count * sizeof(TelemetryCluster));
}

void publish_occupancy_grid(const OccupancyGridMap &grid) {
    uint16_t width = (uint16_t)grid.width();
    uint16_t height = (uint16_t)grid.height();
    uint16_t tile_size_mm = (uint16_t)(grid.tile_size_meters() * 1000.0f);

    const auto &scores = grid.get_scores();
    uint16_t payload_len =
        (uint16_t)(sizeof(width) + sizeof(height) + sizeof(tile_size_mm) + scores.size());

    std::array<uint8_t, sizeof(width) + sizeof(height) + sizeof(tile_size_mm) +
                            OccupancyGridMap::GRID_WIDTH * OccupancyGridMap::GRID_HEIGHT>
        payload = {};

    size_t offset = 0;
    memcpy(payload.data() + offset, &width, sizeof(width));
    offset += sizeof(width);

    memcpy(payload.data() + offset, &height, sizeof(height));
    offset += sizeof(height);

    memcpy(payload.data() + offset, &tile_size_mm, sizeof(tile_size_mm));
    offset += sizeof(tile_size_mm);

    memcpy(payload.data() + offset, scores.data(), scores.size());

    write_frame(FrameType::OccupancyGrid, payload.data(), payload_len);
}

void publish_grid_path(std::span<const uint16_t> path_indices) {
    size_t count = path_indices.size();
    if (count > MAX_GRID_PATH_NODES_PER_FRAME) {
        count = MAX_GRID_PATH_NODES_PER_FRAME;
    }

    constexpr uint16_t MAX_PAYLOAD_LEN =
        sizeof(uint16_t) + MAX_GRID_PATH_NODES_PER_FRAME * sizeof(uint16_t);
    uint8_t payload[MAX_PAYLOAD_LEN] = {0};

    uint16_t count_u16 = static_cast<uint16_t>(count);
    memcpy(payload, &count_u16, sizeof(count_u16));

    if (count > 0) {
        memcpy(payload + sizeof(count_u16), path_indices.data(), count * sizeof(uint16_t));
    }

    write_frame(FrameType::GridPath, payload,
                sizeof(count_u16) + static_cast<uint16_t>(count * sizeof(uint16_t)));
}

void publish_tracked_weights(std::span<WeightTrackedTarget> tracked_weights) {
    constexpr size_t MAX_TRACKED_WEIGHTS_PER_FRAME = 8;

    TrackedWeightEntry entries[MAX_TRACKED_WEIGHTS_PER_FRAME] = {};
    size_t count = tracked_weights.size();
    if (count > MAX_TRACKED_WEIGHTS_PER_FRAME) {
        count = MAX_TRACKED_WEIGHTS_PER_FRAME;
    }

    for (size_t i = 0; i < count; ++i) {
        entries[i] = {
            .x_m = tracked_weights[i].cluster.centroid.x(),
            .y_m = tracked_weights[i].cluster.centroid.y(),
            .confidence = tracked_weights[i].confidence,
        };
    }

    constexpr uint16_t MAX_PAYLOAD_LEN =
        sizeof(uint16_t) + MAX_TRACKED_WEIGHTS_PER_FRAME * sizeof(TrackedWeightEntry);
    uint8_t payload[MAX_PAYLOAD_LEN] = {0};

    uint16_t count_u16 = static_cast<uint16_t>(count);
    memcpy(payload, &count_u16, sizeof(count_u16));
    memcpy(payload + sizeof(count_u16), entries, count * sizeof(TrackedWeightEntry));

    write_frame(FrameType::TrackedWeights, payload,
                sizeof(count_u16) + count * sizeof(TrackedWeightEntry));
}

} // namespace telemetry

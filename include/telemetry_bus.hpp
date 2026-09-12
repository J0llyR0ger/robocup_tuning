#pragma once

#include "config.hpp"
#include "lib/lidar.hpp"
#include "lib/lidar_processing.hpp"
#include "lib/occupancy_grid_map.hpp"
#include <Arduino.h>
#include <array>
#include <cstdint>

namespace telemetry {

enum class ValueType : uint8_t {
    Float32 = 1,
    Int32 = 2,
    UInt32 = 3,
    Bool = 4,
};

enum Key : uint16_t {
    KEY_HEADING = 1,
    KEY_PITCH = 2,
    KEY_LEFT_WHEEL_VELOCITY = 3,
    KEY_RIGHT_WHEEL_VELOCITY = 4,
    KEY_POSITION_X = 5,
    KEY_POSITION_Y = 6,
    KEY_POSITION_UNCERTAINTY = 7,
    KEY_DRIVE_ERROR = 8,
    KEY_TURN_ERROR = 9,
    KEY_LEFT_COMMAND = 10,
    KEY_RIGHT_COMMAND = 11,
    KEY_LOOKAHEAD_X = 12,
    KEY_LOOKAHEAD_Y = 13,
    KEY_NEXTPOINT_X = 14,
    KEY_NEXTPOINT_Y = 15,
    KEY_WEIGHT_TARGET_X = 16,
    KEY_WEIGHT_TARGET_Y = 17,
    KEY_WEIGHT_TARGET_CONFIDENCE = 18,
    KEY_WEIGHT_SPREAD = 19,
    KEY_WEIGHT_EXTENT = 20,
    KEY_WEIGHT_DIAMETER = 21,
    KEY_WEIGHT_ASPECT = 22,
    KEY_WEIGHT_RANGE = 23,
};

void begin(Stream *serial_port = &Serial);

bool publish_f32(uint16_t key, float value);
bool publish_i32(uint16_t key, int32_t value);
bool publish_u32(uint16_t key, uint32_t value);
bool publish_bool(uint16_t key, bool value);

size_t flush_values(uint16_t max_entries_per_frame = 32);
void publish_lidar_points(std::span<LidarResponsePoint> points);
void publish_lidar_processing(LidarProcessingResult result);
void publish_occupancy_grid(const OccupancyGridMap &grid);

} // namespace telemetry

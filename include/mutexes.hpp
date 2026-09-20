#include "FreeRTOS.h"
#include "lib/odometry.hpp"
#include <semphr.h>

#pragma once
inline SemaphoreHandle_t i2cMutex = nullptr;

inline SemaphoreHandle_t globalPoseMutex = nullptr;
static inline Pose globalPose;

void setupMutexes();
Pose get_global_pose();
void set_global_pose(Pose pose);

//----- Joel edits -------
inline bool robot_motion_mismatch = false;
inline SemaphoreHandle_t robotMotionMismatchMutex = nullptr;

bool get_robot_motion_mismatch();
void set_robot_motion_mismatch(bool mismatch);
//------------------------

struct MotionControlPath {
    std::vector<Eigen::Vector2f> path;
    float speed;
};
 

MotionControlPath get_motion_control_path();
void set_motion_control_path(MotionControlPath path);
inline SemaphoreHandle_t motionControlPathMutex = nullptr;
static inline MotionControlPath motionControlPath;

std::vector<Eigen::Vector2f> get_discovery_path();
void set_discovery_path(std::vector<Eigen::Vector2f> path);
inline SemaphoreHandle_t discoveryPathMutex = nullptr;
static inline std::vector<Eigen::Vector2f> discoveryPath;

std::vector<Eigen::Vector2f> get_home_path();
void set_home_path(std::vector<Eigen::Vector2f> path);
inline SemaphoreHandle_t homePathMutex = nullptr;
static inline std::vector<Eigen::Vector2f> homePath;
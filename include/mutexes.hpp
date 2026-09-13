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

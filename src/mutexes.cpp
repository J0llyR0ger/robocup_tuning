#include "mutexes.hpp"

void setupMutexes() {
    i2cMutex = xSemaphoreCreateMutex();
    globalPoseMutex = xSemaphoreCreateMutex();
}

Pose get_global_pose() {
    xSemaphoreTake(globalPoseMutex, portMAX_DELAY);

    Pose pose = globalPose;

    xSemaphoreGive(globalPoseMutex);

    return pose;
}

void set_global_pose(Pose pose) {
    xSemaphoreTake(globalPoseMutex, portMAX_DELAY);

    globalPose = pose;

    xSemaphoreGive(globalPoseMutex);
}
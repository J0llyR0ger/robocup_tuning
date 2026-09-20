#include "mutexes.hpp"

void setupMutexes() {
    i2cMutex = xSemaphoreCreateMutex();
    globalPoseMutex = xSemaphoreCreateMutex();
    motionControlPathMutex = xSemaphoreCreateMutex();
    discoveryPathMutex = xSemaphoreCreateMutex();
    homePathMutex = xSemaphoreCreateMutex();
    robotMotionMismatchMutex = xSemaphoreCreateMutex();
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

MotionControlPath get_motion_control_path() {
    xSemaphoreTake(motionControlPathMutex, portMAX_DELAY);

    MotionControlPath path = motionControlPath;

    xSemaphoreGive(motionControlPathMutex);

    return path;
}

void set_motion_control_path(MotionControlPath path) {
    xSemaphoreTake(motionControlPathMutex, portMAX_DELAY);

    motionControlPath = path;

    xSemaphoreGive(motionControlPathMutex);
}

std::vector<Eigen::Vector2f> get_discovery_path() {
    xSemaphoreTake(discoveryPathMutex, portMAX_DELAY);

    std::vector<Eigen::Vector2f> path = discoveryPath;

    xSemaphoreGive(discoveryPathMutex);

    return path;
}

void set_discovery_path(std::vector<Eigen::Vector2f> path) {
    xSemaphoreTake(discoveryPathMutex, portMAX_DELAY);

    discoveryPath = path;

    xSemaphoreGive(discoveryPathMutex);
}

std::vector<Eigen::Vector2f> get_home_path() {
    xSemaphoreTake(homePathMutex, portMAX_DELAY);

    std::vector<Eigen::Vector2f> path = homePath;

    xSemaphoreGive(homePathMutex);

    return path;
}

void set_home_path(std::vector<Eigen::Vector2f> path) {
    xSemaphoreTake(homePathMutex, portMAX_DELAY);

    homePath = path;

    xSemaphoreGive(homePathMutex);
}

//------ Joel Edits -------
bool get_robot_motion_mismatch() {
    xSemaphoreTake(robotMotionMismatchMutex, portMAX_DELAY);

    bool mismatch = robot_motion_mismatch;

    xSemaphoreGive(robotMotionMismatchMutex);

    return mismatch;
}

void set_robot_motion_mismatch(bool mismatch) {
    xSemaphoreTake(robotMotionMismatchMutex, portMAX_DELAY);

    robot_motion_mismatch = mismatch;

    xSemaphoreGive(robotMotionMismatchMutex);
}
//---------------------------------
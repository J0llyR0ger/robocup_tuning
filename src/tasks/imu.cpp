#pragma once

#include "tasks/imu.hpp"
#include "mutexes.hpp"
#include "telemetry_bus.hpp"
#include <queues.hpp>

ImuTask::ImuTask() : SchedulerTask("imu_task"), imu(55, 0x28) {}

void ImuTask::setup() {
    if (!this->imu.begin()) {
        log_err("No IMU detected");
    }
}

void ImuTask::loop() {
    // Wait for I2C bus to be free
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
        this->last_euler_angles = this->imu.getVector(Adafruit_BNO055::VECTOR_EULER);

        // Send IMU heading data to position tracking task
        xQueueSendToFront(imu_positionTrackingHeadingQueue, &this->get_euler_angles().y(), 0);

        xSemaphoreGive(i2cMutex);
    }

    telemetry::publish_f32(telemetry::KEY_PITCH, this->get_euler_angles().x());
}

Eigen::Vector3f ImuTask::get_euler_angles() {
    auto eulers = this->last_euler_angles;

    // Compass-style convention:
    // +heading is clockwise, 0 is forward, matching the robot/world visualization.
    return {-eulers.y() * DEG_TO_RAD, eulers.x() * DEG_TO_RAD, eulers.z() * DEG_TO_RAD};
}

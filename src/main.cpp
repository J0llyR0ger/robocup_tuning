#include "tasks/autonomous_command.hpp"
#include "tasks/imu.hpp"
#include "tasks/intake.hpp"
#include "tasks/lidar.hpp"
#include "tasks/lidar_processing.hpp"
#include "tasks/mapping.hpp"
#include "tasks/motion_control.hpp"
#include "tasks/position_tracking.hpp"
#include "tasks/telemetry.hpp"
#include "tasks/user_command.hpp"
#include "tasks/weight_detection.hpp"
#include <Arduino.h>
#include <memory>

static ImuTask imu_task = ImuTask(&Wire);
static LidarTask lidar_task = LidarTask();
static DriveTrainTask drive_train_task = DriveTrainTask();

static PositionTrackingTask position_tracking_task =
    PositionTrackingTask(&imu_task, &drive_train_task, &lidar_task);

static LidarProcessingTask lidar_processing_task =
    LidarProcessingTask(&lidar_task, &position_tracking_task);

static MappingTask mapping_task = MappingTask(&position_tracking_task, &lidar_task);

static MotionControlTask motion_control_task =
    MotionControlTask(&drive_train_task, &position_tracking_task);

static IntakeTask intake_task = IntakeTask();
static TelemetryTask telemetry_task = TelemetryTask();
static UserCommandTask user_command_task = UserCommandTask(&drive_train_task);
static WeightDetectionTask weight_detection_task = WeightDetectionTask();

static AutonomousCommandTask autonomous_command_task = AutonomousCommandTask(
    &lidar_processing_task, &position_tracking_task, &motion_control_task, &intake_task);

const size_t NUM_TASKS = 12;

std::array<SchedulerTask *, NUM_TASKS> tasks = {
    &lidar_task,
    &drive_train_task,
    &imu_task,
    &position_tracking_task,
    &mapping_task,
    &motion_control_task,
    &intake_task,
    &telemetry_task,
    &user_command_task,
    &lidar_processing_task,
    &weight_detection_task,
    &autonomous_command_task,
};
std::array<uint32_t, NUM_TASKS> next_runs = {0};

struct TimingData {
    int32_t total_samples;
    int32_t average_time;
    int32_t min_time;
    int32_t max_time;
};

std::array<TimingData, NUM_TASKS> task_timing_data = {{{
    0,
    0,
    100000,
    0,
}}};

void setup() {
    for (size_t i = 0; i < NUM_TASKS; i++) {
        tasks[i]->setup();
    }

    Wire.setClock(400e3);
}

static uint32_t next_log_timings = 0;

void loop() {
    // Loop through tasks, run them, and log the time taken
    for (size_t i = 0; i < NUM_TASKS; i++) {
        uint32_t ticks = micros();

        if (ticks >= next_runs[i]) {
            tasks[i]->loop();
            int32_t time_taken = micros() - ticks;

            task_timing_data[i].total_samples++;
            task_timing_data[i].average_time +=
                (time_taken - task_timing_data[i].average_time) / task_timing_data[i].total_samples;

            if (task_timing_data[i].max_time < time_taken) {
                task_timing_data[i].max_time = time_taken;
            }

            if (task_timing_data[i].min_time > time_taken) {
                task_timing_data[i].min_time = time_taken;
            }

            int total_period = tasks[i]->get_period_micros();

            next_runs[i] += total_period;
        }
    }

    // Handle logging of task statistics
    if (micros() >= next_log_timings) {
        next_log_timings += 5e6;

        if (!ENABLE_TASK_LOGGING) {
            return;
        }

        Serial.println("=======BEGIN TIMING INFO=======");

        int summed_average_time = 0;
        int summed_period_time = 0;
        for (size_t i = 0; i < NUM_TASKS; i++) {
            Serial.printf("%s: %dus\n", tasks[i]->task_name, tasks[i]->get_period_micros());

            Serial.printf("|  Average: %dus\n", task_timing_data[i].average_time);
            Serial.printf("|  Min: %dus\n", task_timing_data[i].min_time);
            Serial.printf("|  Max: %dus\n", task_timing_data[i].max_time);
            Serial.printf("|  Average task load: %d%%\n",
                          (int)(100.0 * (float)task_timing_data[i].average_time /
                                (float)tasks[i]->get_period_micros()));

            summed_average_time += task_timing_data[i].average_time;
            summed_period_time += tasks[i]->get_period_micros();
        }

        Serial.printf("Average CPU load: %d%%\n",
                      (int)(100.0 * (float)summed_average_time / (float)summed_period_time));

        Serial.println("=======END TIMING INFO=======");
    }
}

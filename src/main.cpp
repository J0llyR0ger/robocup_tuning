#include "arduino_freertos.h"
#include "mutexes.hpp"
#include "tasks/autonomous_command.hpp"
#include "tasks/imu.hpp"
#include "tasks/intake.hpp"
#include "tasks/lidar.hpp"
#include "tasks/lidar_processing.hpp"
#include "tasks/mapping.hpp"
#include "tasks/motion_control.hpp"
#include "tasks/telemetry.hpp"
#include "tasks/user_command.hpp"
#include <Arduino.h>
#include <algorithm>
#include <array>
#include <memory>

static ImuTask imu_task = ImuTask();
static IntakeTask intake_task = IntakeTask();
static TelemetryTask telemetry_task = TelemetryTask();
static LidarTask lidar_task = LidarTask();
static DriveTrainTask drive_train_task = DriveTrainTask();
static PositionTrackingTask position_tracking_task = PositionTrackingTask();

static MappingTask mapping_task = MappingTask();
static MotionControlTask motion_control_task = MotionControlTask();

static AutonomousCommandTask autonomous_command_task = AutonomousCommandTask();

const size_t NUM_TASKS = 9;

std::array<SchedulerTask *, NUM_TASKS> tasks = {
    &imu_task,     &intake_task,         &telemetry_task,
    &lidar_task,   &drive_train_task,    &position_tracking_task,
    &mapping_task, &motion_control_task, &autonomous_command_task,
};

/// Rate-monotonic priority assignment: tasks with higher frequencies get
/// higher FreeRTOS priorities so they preempt slower tasks. Tasks sharing a
/// frequency share a priority. Priorities start at 1 (0 is the idle task).
static void start_tasks_rate_monotonic() {
    // Collect distinct frequencies, sorted ascending
    std::array<int, NUM_TASKS> frequencies;
    for (size_t i = 0; i < NUM_TASKS; i++) {
        frequencies[i] = tasks[i]->get_frequency();
    }
    std::sort(frequencies.begin(), frequencies.end());
    auto unique_end = std::unique(frequencies.begin(), frequencies.end());
    size_t num_levels = unique_end - frequencies.begin();

    // Leave headroom below configMAX_PRIORITIES for internal FreeRTOS tasks
    // (e.g. the timer daemon runs at configMAX_PRIORITIES - 1).
    const uint32_t max_priority = configMAX_PRIORITIES - 2;

    for (size_t i = 0; i < NUM_TASKS; i++) {
        size_t level =
            std::lower_bound(frequencies.begin(), unique_end, tasks[i]->get_frequency()) -
            frequencies.begin();

        // Map lowest frequency -> priority 1, highest -> at most max_priority
        uint32_t priority = 1 + level;
        if (priority > max_priority) {
            priority = max_priority;
        }

        tasks[i]->start(priority);
    }
}

/// Low priority task which periodically logs task timing statistics.
static void timing_log_task(void *) {
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(5000));

        if (!ENABLE_TASK_LOGGING) {
            continue;
        }

        Serial.println("=======BEGIN TIMING INFO=======");

        int summed_average_time = 0;
        int summed_period_time = 0;
        for (size_t i = 0; i < NUM_TASKS; i++) {
            const SchedulerTask::TimingData &timing = tasks[i]->timing_data;

            Serial.printf("%s: %dus\n", tasks[i]->task_name, tasks[i]->get_period_micros());

            Serial.printf("|  Average: %dus\n", timing.average_time);
            Serial.printf("|  Min: %dus\n", timing.min_time);
            Serial.printf("|  Max: %dus\n", timing.max_time);
            Serial.printf(
                "|  Average task load: %d%%\n",
                (int)(100.0 * (float)timing.average_time / (float)tasks[i]->get_period_micros()));

            summed_average_time += timing.average_time;
            summed_period_time += tasks[i]->get_period_micros();
        }

        Serial.printf("Average CPU load: %d%%\n",
                      (int)(100.0 * (float)summed_average_time / (float)summed_period_time));

        Serial.println("=======END TIMING INFO=======");
    }
}

void setup() {
    Serial.begin(0);

    if (CrashReport) {
        Serial.print(CrashReport);
        Serial.println();
        Serial.flush();
    }

    Serial.println(PSTR("\r\nBooting FreeRTOS kernel " tskKERNEL_VERSION_NUMBER
                        ". Built by gcc " __VERSION__ " (newlib " _NEWLIB_VERSION ") on " __DATE__
                        ". ***\r\n"));

    setupMutexes();

    Wire.begin();
    Wire.setClock(400e3);

    start_tasks_rate_monotonic();

    xTaskCreate(timing_log_task, "timing_log", 1024, nullptr, 1, nullptr);

    vTaskStartScheduler();
}

void loop() {}

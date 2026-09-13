#include "scheduler_task.hpp"
#include "Arduino.h"
#include "arduino_freertos.h"

SchedulerTask::SchedulerTask(const char *task_name) : task_name(task_name) {}

void SchedulerTask::task_entry(void *instance) {
    SchedulerTask *task = static_cast<SchedulerTask *>(instance);

    TickType_t period_ticks = pdUS_TO_TICKS(task->get_period_micros());
    if (period_ticks < 1) {
        period_ticks = 1;
    }

    TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {
        uint32_t start = micros();

        task->loop();

        int32_t time_taken = micros() - start;

        TimingData &timing = task->timing_data;
        timing.total_samples++;
        timing.average_time += (time_taken - timing.average_time) / timing.total_samples;

        if (timing.max_time < time_taken) {
            timing.max_time = time_taken;
        }

        if (timing.min_time > time_taken) {
            timing.min_time = time_taken;
        }

        // Blocks until the next period boundary; higher priority (higher
        // frequency) tasks will preempt lower priority ones while running.
        xTaskDelayUntil(&last_wake_time, period_ticks);
    }
}

void SchedulerTask::start(uint32_t priority) {
    xTaskCreate(task_entry, task_name, get_stack_depth(), this, priority, &task_handle);
}

void SchedulerTask::log(const char *format, ...) {
    return;

    Serial.printf("[LOG][%s]: ", task_name);

    va_list args;

    va_start(args, format);
    Serial.printf(format, args);
    va_end(args);

    Serial.println("");
}

void SchedulerTask::log_err(const char *format, ...) {
    return;

    Serial.printf("[ERR][%s]: ", task_name);

    va_list args;

    va_start(args, format);
    Serial.printf(format, args);
    va_end(args);

    Serial.println("");
}
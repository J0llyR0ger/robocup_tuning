#pragma once

#include <cstdint>

// Forward declaration of the FreeRTOS task handle type so this header does
// not have to pull the full FreeRTOS headers into every task header.
typedef struct tskTaskControlBlock *TaskHandle_t;

class SchedulerTask {
  protected:
    SchedulerTask(const char *name);

    TaskHandle_t task_handle = nullptr;

    /// FreeRTOS task entry point. Runs loop() periodically at get_frequency().
    static void task_entry(void *instance);

  public:
    struct TimingData {
        int32_t total_samples = 0;
        int32_t average_time = 0;
        int32_t min_time = 100000;
        int32_t max_time = 0;
    };

    const char *task_name;
    TimingData timing_data;

    virtual void setup() = 0;
    virtual void loop() = 0;
    virtual int get_frequency() const = 0;

    /// Stack depth for this task's FreeRTOS task, in 32-bit words (not bytes).
    /// Override for tasks with large stack requirements.
    virtual uint32_t get_stack_depth() const { return 2048; }

    int get_period_micros() const { return 1e6 / this->get_frequency(); };

    /// Spawn the FreeRTOS task that runs loop() at get_frequency().
    /// Higher priority values preempt lower ones.
    void start(uint32_t priority);

    void log(const char *format, ...);
    void log_err(const char *format, ...);
};
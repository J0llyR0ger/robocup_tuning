#pragma once

#include "intake.hpp"
#include "lidar_processing.hpp"
#include "scheduler_task.hpp"
#include "tasks/mapping.hpp"
#include "tasks/motion_control.hpp"
#include <config.hpp>
#include <vector>

class AutonomousCommandTask : public SchedulerTask {
  private:
    enum class DummyWeightRejectionState {
        Idle,
        ReverseUntilSwitchRetrigger,
        ReverseForClearance,
    };

    enum class HomeReturnState {
        Searching,
        ReturningHome,
        ReleasingWeights,
        ReverseFromDrop,
    };

    std::optional<Pose> weight_sensed_pose = std::nullopt;
    std::optional<Eigen::Vector2f> locked_weight_position = std::nullopt;
    // These remain in the map; they are only excluded from autonomous selection.
    std::vector<Eigen::Vector2f> failed_weight_positions;
    struct MissedWeight {
        Eigen::Vector2f position;
        uint32_t missed_at;
    };
    std::vector<MissedWeight> missed_weights;
    std::optional<Eigen::Vector2f> approached_weight_position = std::nullopt;
    uint32_t approach_start_time = 0;
    void defer_missed_weight(const Eigen::Vector2f &position);

    DummyWeightRejectionState dummy_weight_rejection_state =
        DummyWeightRejectionState::Idle;
    uint32_t dummy_weight_rejection_start_time = 0;
    uint32_t dummy_weight_clearance_start_time = 0;
    bool pickup_attempt_rails_down = false;
    uint32_t pickup_attempt_start_time = 0;
    HomeReturnState home_return_state = HomeReturnState::Searching;
    uint32_t home_return_state_start_time = 0;
    std::optional<Eigen::Vector2f> home_progress_position = std::nullopt;
    uint32_t home_progress_start_time = 0;

    void ignore_locked_weight();

  public:
    AutonomousCommandTask();

    void setup();
    void loop();

    int get_frequency() const override { return IMU_TASK_FREQ; }
};

#include "tasks/autonomous_command.hpp"
#include "Arduino.h"
#include "telemetry_bus.hpp"
#include <mutexes.hpp>
#include <queues.hpp>
#include <algorithm>

AutonomousCommandTask::AutonomousCommandTask() : SchedulerTask("autonomous_command") {}

// This uses the same magnitude as MotionControlTask's existing stuck-recovery
// reverse.  It is deliberately much slower than normal autonomous driving.
static const uint32_t DUMMY_WEIGHT_REVERSE_CLEARANCE_MS = 1400;
// Stop rejection if the departing dummy never triggers the entry switch again.
static const uint32_t DUMMY_WEIGHT_RETRIGGER_TIMEOUT_MS = 3000;
// Once a target enters the final pickup zone, do not let intermittent tracking
// raise the rails before it reaches the intake switch.  This is only a failsafe
// for a missed target; either intake entry clears it immediately.
static const uint32_t PICKUP_ATTEMPT_TIMEOUT_MS = 2000;
// Raise loaded rails even when friction prevents the normal 20 cm drive-through.
static const uint32_t REAL_WEIGHT_RAIL_RAISE_TIMEOUT_MS = 750;
static const uint32_t WEIGHT_APPROACH_TIMEOUT_MS = 8000;
static const uint32_t MISSED_WEIGHT_RETRY_DELAY_MS = 10000;
static const float MISSED_WEIGHT_RADIUS_M = 0.3f;
static const float HOME_ARRIVAL_DISTANCE_M = 0.4f;
// Allow a stuck arrival despite localization error near the home corner.
static const float HOME_STUCK_ARRIVAL_DISTANCE_M = 1.2f;
static const float HOME_PROGRESS_DISTANCE_M = 0.05f;
// Retain near-home progress history through modest localization jitter.
static const float HOME_STUCK_EXIT_DISTANCE_M = 1.5f;
static const uint32_t HOME_STUCK_TIMEOUT_MS = 1500;
static const uint32_t HOME_DROP_RELEASE_TIME_MS = 1000;
static const uint32_t HOME_DROP_REVERSE_TIME_MS = 2500;

void AutonomousCommandTask::setup() {}

void AutonomousCommandTask::ignore_locked_weight() {
    const auto rejected_position = this->locked_weight_position.has_value()
        ? this->locked_weight_position : this->intake_weight_position;
    if (rejected_position.has_value()) {
        this->failed_weight_positions.push_back(rejected_position.value());
    }
    this->locked_weight_position = std::nullopt;
    this->intake_weight_position = std::nullopt;
}

void AutonomousCommandTask::defer_missed_weight(const Eigen::Vector2f &position) {
    // Exclude nearby observations too, so tracking jitter cannot immediately relock it.
    this->missed_weights.push_back({position, millis()});
    this->locked_weight_position = std::nullopt;
    this->approached_weight_position = std::nullopt;
    this->pickup_attempt_rails_down = false;
}

void AutonomousCommandTask::loop() {
    auto robot_pose = get_global_pose();

    Eigen::Vector2f locking_center = robot_pose.position + robot_pose.get_direction_vector() * 0.5;

    WeightTrackingPayload weight_targets{};

    xQueuePeek(weight_tracking_queue, &weight_targets, 0);

    bool is_real;

    // A non-conductive switch entry is the intake's existing dummy-weight
    // determination.  The next such entry is the same switch being released and
    // triggered again while reversing, which is the point at which the rails can
    // safely move to storage.
    if (xQueueReceive(intake_entry_queue, &is_real, 0)) {
        // Preserve the object's coordinate if metal detection precedes rejection.
        // With no tracked target, use the estimated intake location.
        if (this->dummy_weight_rejection_state == DummyWeightRejectionState::Idle) {
            if (this->locked_weight_position.has_value()) {
                this->intake_weight_position = this->locked_weight_position;
            } else if (this->approached_weight_position.has_value()) {
                this->intake_weight_position = this->approached_weight_position;
            } else if (is_real || !this->intake_weight_position.has_value()) {
                this->intake_weight_position = locking_center;
            }
        }
        this->pickup_attempt_rails_down = false;
        this->approached_weight_position = std::nullopt;
        if (!is_real) {
            // An upside-down switch can reject an object after its metal probe fired.
            this->weight_sensed_pose = std::nullopt;
        }

        if (!is_real && this->dummy_weight_rejection_state ==
                            DummyWeightRejectionState::ReverseUntilSwitchRetrigger) {
            this->dummy_weight_rejection_state = DummyWeightRejectionState::ReverseForClearance;
            this->dummy_weight_clearance_start_time = millis();
        } else if (!is_real && this->dummy_weight_rejection_state ==
                                   DummyWeightRejectionState::Idle) {
            this->ignore_locked_weight();
            this->dummy_weight_rejection_state =
                DummyWeightRejectionState::ReverseUntilSwitchRetrigger;
            this->dummy_weight_rejection_start_time = millis();
        } else if (is_real) {
            this->weight_sensed_pose = robot_pose;
            this->real_weight_detected_time = millis();
            this->locked_weight_position = std::nullopt;
        }
    }

    // A missing second switch event must not latch the reverse override forever.
    if (this->dummy_weight_rejection_state ==
            DummyWeightRejectionState::ReverseUntilSwitchRetrigger &&
        millis() - this->dummy_weight_rejection_start_time >=
            DUMMY_WEIGHT_RETRIGGER_TIMEOUT_MS) {
        this->dummy_weight_rejection_state = DummyWeightRejectionState::Idle;
        MotionControlOverride override_command = MotionControlOverride::None;
        xQueueOverwrite(motion_control_override_queue, &override_command);
        bool intake_position = true;
        xQueueOverwrite(intake_position_queue, &intake_position);
        return;
    }

    if (this->dummy_weight_rejection_state != DummyWeightRejectionState::Idle) {
        if (this->dummy_weight_rejection_state ==
                DummyWeightRejectionState::ReverseForClearance &&
            millis() - this->dummy_weight_clearance_start_time >=
                DUMMY_WEIGHT_REVERSE_CLEARANCE_MS) {
            this->dummy_weight_rejection_state = DummyWeightRejectionState::Idle;
        } else {
            MotionControlOverride override_command = MotionControlOverride::DummyWeightReverse;
            xQueueOverwrite(motion_control_override_queue, &override_command);

            // The rails stay down until the dummy produces the second entry-switch
            // event.  The storage command is then held while it is left behind.
            bool intake_position = this->dummy_weight_rejection_state ==
                                   DummyWeightRejectionState::ReverseForClearance;
            xQueueOverwrite(intake_position_queue, &intake_position);
            return;
        }
    }

    MotionControlOverride override_command =
        this->home_return_state == HomeReturnState::ReleasingWeights
            ? MotionControlOverride::HomeDropHold
            : MotionControlOverride::None;
    xQueueOverwrite(motion_control_override_queue, &override_command);

    bool recovery_reversing = false;
    xQueuePeek(motion_control_recovery_reversing_queue, &recovery_reversing, 0);

    bool storage_voltage_probe_high = true;
    xQueuePeek(storage_voltage_probe_queue, &storage_voltage_probe_high, 0);

    uint8_t total_weights_carried = 0;
    xQueuePeek(carried_weight_count, &total_weights_carried, 0);
    const bool full_by_count = total_weights_carried >= 4 && !this->weight_sensed_pose.has_value();
    if (this->home_return_state == HomeReturnState::Searching &&
        (!storage_voltage_probe_high || full_by_count)) {
        // Both storage detection and a full count must run the complete drop sequence.
        Serial.println("HOME: returning");
        this->weight_sensed_pose = std::nullopt;
        this->locked_weight_position = std::nullopt;
        this->approached_weight_position = std::nullopt;
        this->pickup_attempt_rails_down = false;
        this->home_best_distance = std::nullopt;
        this->home_return_state = HomeReturnState::ReturningHome;
    }

    if (this->home_return_state != HomeReturnState::Searching) {
        if (this->home_return_state == HomeReturnState::ReturningHome) {
            set_motion_control_path({get_home_path(), 1.0});

            const float home_distance = (robot_pose.position - get_home_position()).norm();
            const uint32_t home_now = millis();
            const bool near_home = home_distance <= HOME_STUCK_ARRIVAL_DISTANCE_M;
            if (home_distance > HOME_STUCK_EXIT_DISTANCE_M) {
                this->home_best_distance = std::nullopt;
            } else if (near_home && !this->home_best_distance.has_value()) {
                this->home_best_distance = home_distance;
                this->home_progress_start_time = home_now;
            } else if (this->home_best_distance.has_value() &&
                       home_distance <= this->home_best_distance.value() - HOME_PROGRESS_DISTANCE_M) {
                // Only actual progress toward home restarts the deadline.
                this->home_best_distance = home_distance;
                this->home_progress_start_time = home_now;
            }
            const bool stuck_near_home = this->home_best_distance.has_value() &&
                (recovery_reversing ||
                 home_now - this->home_progress_start_time >= HOME_STUCK_TIMEOUT_MS);

            if (home_distance <= HOME_ARRIVAL_DISTANCE_M || stuck_near_home) {
                Serial.printf("HOME: releasing, distance=%.2f m, stuck=%d\n",
                              home_distance, stuck_near_home);
                this->home_return_state = HomeReturnState::ReleasingWeights;
                this->home_return_state_start_time = home_now;
                this->home_best_distance = std::nullopt;
            } else {
                bool intake_position = true;
                xQueueOverwrite(intake_position_queue, &intake_position);
                return;
            }
        }

        if (this->home_return_state == HomeReturnState::ReleasingWeights) {
            // Stop pushing into the wall and cancel ordinary recovery during release.
            MotionControlOverride hold_override = MotionControlOverride::HomeDropHold;
            xQueueOverwrite(motion_control_override_queue, &hold_override);
            set_motion_control_path({{}, 0.0});
            // DOWN is the existing pickup/release position.  Wait until the
            // active-low storage probe confirms the metal has left storage.
            bool intake_position = false;
            xQueueOverwrite(intake_position_queue, &intake_position);

            if (storage_voltage_probe_high &&
                millis() - this->home_return_state_start_time >= HOME_DROP_RELEASE_TIME_MS) {
                bool reset_count = true;
                xQueueOverwrite(intake_reset_carried_weight_count_queue, &reset_count);
                this->home_return_state = HomeReturnState::ReverseFromDrop;
                this->home_return_state_start_time = millis();
            }
            return;
        }

        if (this->home_return_state == HomeReturnState::ReverseFromDrop) {
            if (millis() - this->home_return_state_start_time >= HOME_DROP_REVERSE_TIME_MS) {
                this->home_return_state = HomeReturnState::Searching;
            } else {
                MotionControlOverride drop_override = MotionControlOverride::HomeDropReverse;
                xQueueOverwrite(motion_control_override_queue, &drop_override);

                bool intake_position = false;
                xQueueOverwrite(intake_position_queue, &intake_position);
                return;
            }
        }
    }

    if (recovery_reversing) {
        // A mismatch-recovery reverse is not a pickup manoeuvre.  Keep the
        // intake out of the way until MotionControlTask declares it complete.
        bool intake_position = true;
        xQueueOverwrite(intake_position_queue, &intake_position);
        return;
    }

    const uint32_t now = millis();
    this->missed_weights.erase(
        std::remove_if(this->missed_weights.begin(), this->missed_weights.end(),
                       [now](const MissedWeight &miss) {
                           return now - miss.missed_at >= MISSED_WEIGHT_RETRY_DELAY_MS;
                       }),
        this->missed_weights.end());

    // Release failed targets before selection so a fresh path is published this loop.
    if (this->locked_weight_position.has_value()) {
        if (now - this->pickup_attempt_start_time >= PICKUP_ATTEMPT_TIMEOUT_MS ||
            (this->locked_weight_position.value() - locking_center).norm() > 0.3f) {
            this->defer_missed_weight(this->locked_weight_position.value());
        }
    } else if (this->approached_weight_position.has_value() &&
               now - this->approach_start_time >= WEIGHT_APPROACH_TIMEOUT_MS) {
        this->defer_missed_weight(this->approached_weight_position.value());
    }

    int best_track_index = -1;
    float closest_weight_distance = 5.0;

    for (size_t track_index = 0; track_index < weight_targets.count; ++track_index) {
        const auto &track = weight_targets.targets[track_index];

        bool previously_failed = false;

        for (const auto &failed_position : this->failed_weight_positions) {
            if ((track - failed_position).norm() < 0.3) {
                previously_failed = true;
                break;
            }
        }

        if (previously_failed) {
            continue;
        }
        const bool cooling_down = std::any_of(
            this->missed_weights.begin(), this->missed_weights.end(),
            [&track](const MissedWeight &miss) {
                return (track - miss.position).norm() < MISSED_WEIGHT_RADIUS_M;
            });
        if (cooling_down) {
            continue;
        }

        auto dist = (track - robot_pose.position).norm();

        if (dist < closest_weight_distance) {
            closest_weight_distance = dist;
            best_track_index = static_cast<int>(track_index);
        }
    }

    bool intake_position = false;

    if (this->weight_sensed_pose.has_value()) {
        auto sensed_pose = this->weight_sensed_pose.value();

        set_motion_control_path(
            {{sensed_pose.position + sensed_pose.get_direction_vector() * 1.0}, 1.0});

        if ((sensed_pose.position - robot_pose.position).norm() > 0.2 ||
            millis() - this->real_weight_detected_time >= REAL_WEIGHT_RAIL_RAISE_TIMEOUT_MS) {
            this->weight_sensed_pose = std::nullopt;
            intake_position = true;
        } else {
            intake_position = false;
        }
    } else if (this->locked_weight_position.has_value()) {
        set_motion_control_path({{this->locked_weight_position.value()}, 1.0});
        intake_position = false;
    } else if (best_track_index >= 0) {
        const auto &best_track = weight_targets.targets[best_track_index];
        // A new approach supersedes the previously collected object's location.
        this->intake_weight_position = std::nullopt;

        // Keep the deadline for the same target despite small tracking movements.
        if (!this->approached_weight_position.has_value() ||
            (best_track - this->approached_weight_position.value()).norm() >=
                MISSED_WEIGHT_RADIUS_M) {
            this->approached_weight_position = best_track;
            this->approach_start_time = now;
        }

        float speed = 1.0;

        if ((best_track - locking_center).norm() < 0.3) {
            this->locked_weight_position = best_track;
            this->pickup_attempt_rails_down = true;
            this->pickup_attempt_start_time = millis();
            intake_position = false;
        } else {
            intake_position = true;
        }

        set_motion_control_path({{best_track}, speed});
    } else {
        intake_position = true;
        set_motion_control_path({get_discovery_path(), 1.0});
    }

    // This latch has priority over target/map transitions until the object has
    // actually entered the intake (or the missed-target timeout expires).
    if (this->pickup_attempt_rails_down) {
        intake_position = false;
    }

    xQueueOverwrite(intake_position_queue, &intake_position);
}

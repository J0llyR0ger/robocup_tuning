#include "tasks/autonomous_command.hpp"
#include "Arduino.h"
#include "drive_enable.hpp"
#include "telemetry_bus.hpp"
#include <mutexes.hpp>
#include <queues.hpp>
#include <algorithm>

AutonomousCommandTask::AutonomousCommandTask() : SchedulerTask("autonomous_command") {}

// Stop and retain the remaining weights as soon as the entry sensors release.
static const uint32_t DUMMY_WEIGHT_LIFT_MS = 400;
static const uint32_t DUMMY_WEIGHT_CLEARANCE_MS = 1400;
static const uint32_t DUMMY_WEIGHT_RELEASE_TIMEOUT_MS = 3000;
// Once a target enters the final pickup zone, do not let intermittent tracking
// raise the rails before it reaches the intake switch.  This is only a failsafe
// for a missed target; either intake entry clears it immediately.
static const uint32_t PICKUP_ATTEMPT_TIMEOUT_MS = 2000;
// Raise loaded rails even when friction prevents the normal 20 cm drive-through.
static const uint32_t REAL_WEIGHT_RAIL_RAISE_TIMEOUT_MS = 750;
// Keep rails at intake level through realignment, braking, and forward pickup.
static const uint32_t PICKUP_BRAKE_MS = 400;
static const uint32_t PICKUP_REVERSE_MS = 500;
// One bounded forward interval at the faster pickup speed; entry does not restart it.
static const uint32_t PICKUP_FORWARD_MS = 400;
// Finish the loaded lift before target selection can request rails down again.
static const uint32_t PICKUP_LIFT_MS = 400;
// Scale forward drive during the time-limited final pickup and drive-through.
static const float PICKUP_DRIVE_MULTIPLIER = 1.5f;
static const uint32_t WEIGHT_APPROACH_TIMEOUT_MS = 8000;
static const uint32_t MISSED_WEIGHT_RETRY_DELAY_MS = 5000;
static const float MISSED_WEIGHT_RADIUS_M = 0.3f;
// Do not select deposited weights around our saved starting/home position.
static const float HOME_PICKUP_EXCLUSION_RADIUS_M = 0.65f;
static const float ENEMY_PICKUP_EXCLUSION_RADIUS_M = 0.65f;
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
    if (!drive_enabled.load()) {
        // Cancel movement sequences without resetting mapping or carried weights.
        pickup_state = PickupState::Idle;
        dummy_weight_rejection_state = DummyWeightRejectionState::Idle;
        home_return_state = HomeReturnState::Searching;
        pickup_attempt_rails_down = false;
        weight_sensed_pose.reset();
        intake_weight_position.reset();
        locked_weight_position.reset();
        approached_weight_position.reset();
        home_best_distance.reset();
        early_intake_probe_was_active = false;
        const bool inactive = false;
        xQueueOverwrite(intake_pickup_active_queue, &inactive);
        // Sensor events observed while stationary must not trigger old manoeuvres.
        xQueueReset(intake_entry_queue);
        set_motion_control_path({{}, 0.0f});
        const auto no_override = MotionControlOverride::None;
        xQueueOverwrite(motion_control_override_queue, &no_override);
        return;
    }
    auto robot_pose = get_global_pose();

    Eigen::Vector2f locking_center = robot_pose.position + robot_pose.get_direction_vector() * 0.5;

    WeightTrackingPayload weight_targets{};

    xQueuePeek(weight_tracking_queue, &weight_targets, 0);

    bool early_probe_active = false;
    xQueuePeek(early_intake_probe_queue, &early_probe_active, 0);
    const bool early_entry = early_probe_active && !this->early_intake_probe_was_active;
    this->early_intake_probe_was_active = early_probe_active;

    bool storage_clear = true;
    uint8_t carried_count = 0;
    xQueuePeek(storage_voltage_probe_queue, &storage_clear, 0);
    xQueuePeek(carried_weight_count, &carried_count, 0);
    // Home return takes priority, including a full load detected this same tick.
    if (early_entry && storage_clear && carried_count < 4 &&
        this->home_return_state == HomeReturnState::Searching &&
        this->pickup_state == PickupState::Idle &&
        this->dummy_weight_rejection_state == DummyWeightRejectionState::Idle) {
        this->pickup_state = PickupState::Forward;
        this->pickup_state_start_time = millis();
        this->intake_weight_position = this->locked_weight_position.has_value()
            ? this->locked_weight_position
            : (this->approached_weight_position.has_value()
                ? this->approached_weight_position : std::optional<Eigen::Vector2f>(locking_center));
        this->locked_weight_position = std::nullopt;
        this->approached_weight_position = std::nullopt;
        this->pickup_attempt_rails_down = false;
        bool pickup_active = true;
        xQueueOverwrite(intake_pickup_active_queue, &pickup_active);
        Serial.println("PICKUP: early probe, forward with rails down");
    }

    bool is_real;

    // A non-conductive or upside-down entry starts dummy rejection.
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
            this->pickup_state = PickupState::Idle;
            bool pickup_active = false;
            xQueueOverwrite(intake_pickup_active_queue, &pickup_active);
        }

        if (!is_real && this->dummy_weight_rejection_state ==
                            DummyWeightRejectionState::Idle) {
            this->ignore_locked_weight();
            this->dummy_weight_rejection_state =
                DummyWeightRejectionState::ReverseUntilRelease;
            this->dummy_weight_rejection_start_time = millis();
        } else if (is_real && this->pickup_state == PickupState::Idle &&
                   this->dummy_weight_rejection_state == DummyWeightRejectionState::Idle &&
                   this->home_return_state == HomeReturnState::Searching) {
            this->pickup_state = PickupState::Reversing;
            this->pickup_state_start_time = millis();
            Serial.println("PICKUP: realigning with rails down");
            bool pickup_active = true;
            xQueueOverwrite(intake_pickup_active_queue, &pickup_active);
            this->weight_sensed_pose = robot_pose;
            this->real_weight_detected_time = millis();
            this->locked_weight_position = std::nullopt;
        }
    }

    // Complete the pickup before normal target selection or a count-based home return.
    // Dummy/upside-down rejection above cancels this sequence immediately.
    if (this->pickup_state != PickupState::Idle) {
        const uint32_t now = millis();
        const uint32_t elapsed = now - this->pickup_state_start_time;
        if (this->pickup_state == PickupState::Reversing && elapsed >= PICKUP_REVERSE_MS) {
            this->pickup_state = PickupState::Braking;
            this->pickup_state_start_time = now;
            Serial.println("PICKUP: stopping with rails down");
        } else if (this->pickup_state == PickupState::Braking && elapsed >= PICKUP_BRAKE_MS) {
            this->pickup_state = PickupState::Forward;
            this->pickup_state_start_time = now;
            Serial.println("PICKUP: forward with rails down");
        } else if (this->pickup_state == PickupState::Forward && elapsed >= PICKUP_FORWARD_MS) {
            this->pickup_state = PickupState::Lifting;
            this->pickup_state_start_time = now;
            Serial.println("PICKUP: lifting into storage");
        } else if (this->pickup_state == PickupState::Lifting && elapsed >= PICKUP_LIFT_MS) {
            this->pickup_state = PickupState::Idle;
            this->weight_sensed_pose = std::nullopt;
            bool pickup_active = false;
            xQueueOverwrite(intake_pickup_active_queue, &pickup_active);
        }

        MotionControlOverride pickup_override = MotionControlOverride::None;
        bool rails_up = true;
        switch (this->pickup_state) {
        case PickupState::Braking:
            pickup_override = MotionControlOverride::PickupHold;
            rails_up = false;
            break;
        case PickupState::Lifting:
            pickup_override = MotionControlOverride::PickupHold;
            break;
        case PickupState::Reversing:
            pickup_override = MotionControlOverride::PickupReverse;
            rails_up = false;
            break;
        case PickupState::Forward:
            pickup_override = MotionControlOverride::PickupForward;
            rails_up = false;
            break;
        case PickupState::Idle:
            set_motion_control_path({{}, 0.0});
            break;
        }
        xQueueOverwrite(motion_control_override_queue, &pickup_override);
        xQueueOverwrite(intake_position_queue, &rails_up);
        return;
    }

    if (this->dummy_weight_rejection_state != DummyWeightRejectionState::Idle) {
        bool release_clear = false;
        xQueuePeek(intake_release_clear_queue, &release_clear, 0);
        const uint32_t now = millis();
        if (this->dummy_weight_rejection_state == DummyWeightRejectionState::ReverseUntilRelease &&
            (release_clear || now - this->dummy_weight_rejection_start_time >=
                                  DUMMY_WEIGHT_RELEASE_TIMEOUT_MS)) {
            this->dummy_weight_rejection_state = DummyWeightRejectionState::RaisingRails;
            this->dummy_weight_lift_start_time = now;
            Serial.println(release_clear ? "DUMMY: released, lifting rails"
                                         : "DUMMY: release timeout, stopping and lifting");
        }
        if (this->dummy_weight_rejection_state == DummyWeightRejectionState::RaisingRails &&
            now - this->dummy_weight_lift_start_time >= DUMMY_WEIGHT_LIFT_MS) {
            this->dummy_weight_rejection_state = DummyWeightRejectionState::ReverseForClearance;
            this->dummy_weight_clearance_start_time = now;
            Serial.println("DUMMY: reversing clear with rails up");
        }
        if (this->dummy_weight_rejection_state == DummyWeightRejectionState::ReverseForClearance &&
            now - this->dummy_weight_clearance_start_time >= DUMMY_WEIGHT_CLEARANCE_MS) {
            this->dummy_weight_rejection_state = DummyWeightRejectionState::Idle;
        } else {
            bool rails_up = this->dummy_weight_rejection_state != DummyWeightRejectionState::ReverseUntilRelease;
            MotionControlOverride rejection_override = MotionControlOverride::DummyWeightReverse;
            if (this->dummy_weight_rejection_state == DummyWeightRejectionState::RaisingRails) {
                rejection_override = MotionControlOverride::DummyWeightHold;
            } else if (this->dummy_weight_rejection_state == DummyWeightRejectionState::ReverseForClearance) {
                rejection_override = MotionControlOverride::DummyWeightClearanceReverse;
            }
            xQueueOverwrite(motion_control_override_queue, &rejection_override);
            xQueueOverwrite(intake_position_queue, &rails_up);
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

            if (home_distance <= HOME_ARRIVAL_DISTANCE_M) {
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

    const Eigen::Vector2f home_position = get_home_position();
    // Bases are at opposite ends of the same short wall. X is the short axis;
    // mirroring the saved start works with either base assigned as our home.
    const Eigen::Vector2f enemy_position(FIELD_WIDTH_X_METERS - home_position.x(),
                                         home_position.y());
    const auto inside_base_pickup_zone = [&home_position, &enemy_position](
                                            const Eigen::Vector2f &position) {
        return (position - home_position).norm() <= HOME_PICKUP_EXCLUSION_RADIUS_M ||
               (position - enemy_position).norm() <= ENEMY_PICKUP_EXCLUSION_RADIUS_M;
    };
    if ((this->locked_weight_position.has_value() &&
         inside_base_pickup_zone(this->locked_weight_position.value())) ||
        (this->approached_weight_position.has_value() &&
         inside_base_pickup_zone(this->approached_weight_position.value()))) {
        this->locked_weight_position = std::nullopt;
        this->approached_weight_position = std::nullopt;
        this->pickup_attempt_rails_down = false;
    }

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
        if (inside_base_pickup_zone(track)) {
            continue;
        }

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
            {{sensed_pose.position + sensed_pose.get_direction_vector() * 1.0}, PICKUP_DRIVE_MULTIPLIER});

        if ((sensed_pose.position - robot_pose.position).norm() > 0.2 ||
            millis() - this->real_weight_detected_time >= REAL_WEIGHT_RAIL_RAISE_TIMEOUT_MS) {
            this->weight_sensed_pose = std::nullopt;
            set_motion_control_path(
                {{sensed_pose.position + sensed_pose.get_direction_vector() * 1.0}, 1.0});
            intake_position = true;
        } else {
            intake_position = false;
        }
    } else if (this->locked_weight_position.has_value()) {
        set_motion_control_path({{this->locked_weight_position.value()}, PICKUP_DRIVE_MULTIPLIER});
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
            speed = PICKUP_DRIVE_MULTIPLIER;
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

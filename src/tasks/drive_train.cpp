#include "tasks/drive_train.hpp"
#include "queues.hpp"
#include "home_selection.hpp"
#include "enemy_base.hpp"
#include "telemetry_bus.hpp"
#include <mutexes.hpp>
#include "m_opening.hpp"
#include "drive_enable.hpp"
#include "dummy_rejection.hpp"
#include "match_end.hpp"
#include "tasks/display.hpp"
#include <wiring.h>

DriveTrainTask::DriveTrainTask() : SchedulerTask("drive_train_task") {}

void DriveTrainTask::setup() {
    left_motor.attach(LEFT_MOTOR_CONTROL_PIN);
    right_motor.attach(RIGHT_MOTOR_CONTROL_PIN);
    left_motor.writeMicroseconds(1500);
    right_motor.writeMicroseconds(1500);
    drive_enabled.store(false);
    pinMode(BLUE_BUTTON_PIN, BLUE_BUTTON_ACTIVE_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);
    button_raw_pressed = digitalRead(BLUE_BUTTON_PIN) ==
                         (BLUE_BUTTON_ACTIVE_LOW ? LOW : HIGH);
    button_changed_at = millis();
    button_armed = false; // Require a stable release, including at boot.
    last_timestamp = micros();
    // Serial.println("DRIVE: inhibited; press blue to enable");
}

void DriveTrainTask::update_drive_button() {
    // Match completion locks out the start button until reboot.
    if (match_controller.phase == match_end::Phase::Finished) {
        button_armed = false;
        return;
    }
    const uint32_t now = millis();
    const bool pressed = digitalRead(BLUE_BUTTON_PIN) ==
                         (BLUE_BUTTON_ACTIVE_LOW ? LOW : HIGH);
    if (pressed != button_raw_pressed) {
        button_raw_pressed = pressed;
        button_changed_at = now;
    }
    if (now - button_changed_at < BLUE_BUTTON_DEBOUNCE_MS) return;
    if (!pressed) {
        button_armed = true;
        return;
    }
    if (button_armed) {
        // Start only. Consume presses during a run without changing drive state.
        button_armed = false;
        if (drive_enabled.load() || drive_start_pending.load()) return;
        // Discard commands from before this transition.
        xQueueReset(motionControl_ChassisCommandsQueue);
        left_command = right_command = 0.0f;
        left_slew = SlewRate<float>(SLEW_RATE);
        right_slew = SlewRate<float>(SLEW_RATE);
        left_motor.writeMicroseconds(1500);
        right_motor.writeMicroseconds(1500);
        drive_enabled.store(false);
        m_opening_stopped.store(false);
        m_opening_servo_raised.store(false);
        m_opening_phase.store(menu_m_enabled.load() ? m_opening::Phase::Approaching : m_opening::Phase::Idle);
        set_opening_path({});
        match_end_rails_applied.store(match_end::Phase::Idle);
        match_end_limit_seen.store(false);
        match_end_returning_home.store(false);
        match_controller.start(now);
        match_end_phase.store(match_controller.phase);
        // Home is already applied by the display. Wait for its pose/map update.
        drive_start_pending.store(true);
        // Serial.println(active_home_blue.load() ? "HOME: blue selected" : "HOME: green selected");
    }
}

static const int TICKS_PER_REVOLUTION = 2652;
static const int FORWARD_MS = 1950;
static const int REVERSE_MS = 1050;

static const float RADIANS_PER_TICK = 2.0 * PI / (float)TICKS_PER_REVOLUTION;

static const float RIGHT_SCALE = 0.75;

static const float KICKOFF_VALUE = 0.2;

static float apply_kickoff(float command) {
    if (command == 0.0f) {
        return 0.0f;
    }

    float magnitude = std::abs(command);
    float remapped = KICKOFF_VALUE + (1.0f - KICKOFF_VALUE) * magnitude;

    return command > 0.0f ? remapped : -remapped;
}

void DriveTrainTask::loop() {
    update_drive_button();
    match_end::Sensors end_sensors;
    xQueuePeek(match_end_sensors_queue, &end_sensors, 0);
    const auto old_phase = match_controller.phase;
    const auto end_phase = match_controller.update(millis(), end_sensors,
        match_end_limit_seen.load(),
        match_end_rails_applied.load(), match_end_returning_home.load());
    match_end_phase.store(end_phase);
    if (end_phase != old_phase) {
        // Serial.printf("MATCH: end phase=%u\n", static_cast<unsigned>(end_phase));
    }
    if (end_phase == match_end::Phase::Finished) {
        drive_enabled.store(false);
        drive_start_pending.store(false);
        left_command = right_command = 0.0f;
        xQueueReset(motionControl_ChassisCommandsQueue);
        if (old_phase != end_phase) {
            // A button held across the deadline cannot immediately restart the robot.
            button_armed = false;
            // Serial.println("DRIVE: inhibited; match complete");
        }
    }
    const uint32_t generation = home_request_generation.load();
    if (drive_start_pending.load() && home_pose_generation.load() == generation &&
        home_map_generation.load() == generation) {
        xQueueReset(motionControl_ChassisCommandsQueue);
        left_command = right_command = 0.0f;
        drive_start_pending.store(false);
        drive_enabled.store(true);
        // Serial.println("DRIVE: enabled; home ready");
    }
    std::tuple<float, float> new_commands;

    if (xQueueReceive(motionControl_ChassisCommandsQueue, &new_commands, 0)) {
        this->left_command = std::clamp(std::get<0>(new_commands), -1.0f, 1.0f);
        this->right_command = std::clamp(std::get<1>(new_commands), -1.0f, 1.0f);
    }

    if (!drive_enabled.load()) {
        left_command = right_command = 0.0f;
        left_slew = SlewRate<float>(SLEW_RATE);
        right_slew = SlewRate<float>(SLEW_RATE);
    }

    float left_out = this->left_command;
    float right_out = this->right_command;

    right_out *= RIGHT_SCALE;

    left_out = apply_kickoff(left_out);
    right_out = apply_kickoff(right_out);

    float left_rate = this->left_slew.update(left_out);
    float right_rate = this->right_slew.update(right_out);

    // Final guard covers path following, timed pickups, and reverse overrides.
    // Reset the ramp as well, so a stop is not delayed by slew limiting.
    if (drive_enabled.load()) {
        const auto pose = get_global_pose();
        const float translation = (left_rate + right_rate) * 0.5f;
        float preview_distance = 0.35f;
        // Do not project beyond the opening destination when approaching it.
        if (m_opening_phase.load() == m_opening::Phase::Approaching && translation > 0.0f) {
            preview_distance = std::min(preview_distance, (m_opening_target() - pose.position).norm());
        }
        const float preview = translation > 0.0f ? preview_distance : translation < 0.0f ? -0.35f : 0.0f;
        // Heading is clockwise from +Y, in radians (same convention as steering).
        const Eigen::Vector2f direction(std::sin(pose.heading), std::cos(pose.heading));
        bool blocked = crosses_enemy_base(pose.position,
                              pose.position + direction * preview);
        if (inside_enemy_base(pose.position)) {
            if (left_rate * right_rate <= 0.0f) {
                // Cancel translation from unequal wheel scaling while turning to escape.
                const float turn = (left_rate - right_rate) * 0.5f;
                left_rate = turn;
                right_rate = -turn;
                blocked = false;
            } else {
                const auto projected = pose.position + direction * preview;
                blocked = enemy_base_exit_clearance(projected) <=
                          enemy_base_exit_clearance(pose.position);
            }
        }
        if (blocked) {
            // Remove translation but preserve steering so the robot can turn away.
            const float turn = (left_rate - right_rate) * 0.5f;
            left_rate = turn;
            right_rate = -turn;
            left_command = right_command = 0.0f;
            left_slew = SlewRate<float>(SLEW_RATE);
            right_slew = SlewRate<float>(SLEW_RATE);
        }
    }

    // Final behavioural authority, including over slew and navigation boundaries.
    // The physical drive-disable remains absolute.
    const auto rejection = dummy_rejection_priority.load();
    if (rejection != DummyRejectionPriority::Idle && drive_enabled.load()) {
        left_command = right_command = rejection == DummyRejectionPriority::Lift ? 0.0f : -0.10f;
        left_slew = SlewRate<float>(SLEW_RATE);
        right_slew = SlewRate<float>(SLEW_RATE);
        left_rate = right_rate = apply_kickoff(left_command);
    }
    // Bypass slew immediately while the entry contact is awaiting classification.
    if (intake_classification_pending.load() && rejection == DummyRejectionPriority::Idle) {
        left_command = right_command = left_rate = right_rate = 0.0f;
        left_slew = SlewRate<float>(SLEW_RATE);
        right_slew = SlewRate<float>(SLEW_RATE);
    }
    const bool opening_hold = m_opening::hold(m_opening_phase.load());
    if (opening_hold) {
        left_command = right_command = left_rate = right_rate = 0.0f;
        left_slew = SlewRate<float>(SLEW_RATE);
        right_slew = SlewRate<float>(SLEW_RATE);
    }
    // End-of-match motion overrides navigation and rejection; drive-disable is absolute.
    if (match_end::owns_intake(end_phase)) {
        left_command = right_command = end_phase == match_end::Phase::Reversing &&
            match_end::fresh(millis(), end_sensors)
            ? -match_end::REVERSE_SPEED : 0.0f;
        left_slew = SlewRate<float>(SLEW_RATE);
        right_slew = SlewRate<float>(SLEW_RATE);
        left_rate = right_rate = apply_kickoff(left_command);
    }
    if (!drive_enabled.load()) left_rate = right_rate = 0.0f;

    left_motor.writeMicroseconds(map(left_rate, 1.0, -1.0, FORWARD_MS, REVERSE_MS));
    right_motor.writeMicroseconds(map(right_rate, 1.0, -1.0, REVERSE_MS, FORWARD_MS));

    m_opening_stopped.store(opening_hold && left_rate == 0.0f && right_rate == 0.0f);

    telemetry::publish_f32(telemetry::KEY_LEFT_COMMAND, this->left_command);
    telemetry::publish_f32(telemetry::KEY_RIGHT_COMMAND, this->right_command);

    uint32_t timestamp = micros();

    float dt = (float)(timestamp - last_timestamp) / 1e6;

    last_timestamp = timestamp;

    int left_ticks = left_encoder.read();
    int right_ticks = -right_encoder.read();

    // Require encoder travel backwards, not merely a reverse command while
    // inertia is still carrying the robot forwards.
    const int left_delta = left_ticks - last_left_ticks;
    const int right_delta = right_ticks - last_right_ticks;
    dummy_moving_backwards.store(drive_enabled.load() &&
        dummy_rejection_priority.load() == DummyRejectionPriority::ReverseDown &&
        left_command < 0.0f && right_command < 0.0f &&
        left_delta <= 0 && right_delta <= 0 && (left_delta < 0 || right_delta < 0));

    float left_velocity = RADIANS_PER_TICK * (float)(left_ticks - last_left_ticks) / dt;
    float right_velocity = RADIANS_PER_TICK * (float)(right_ticks - last_right_ticks) / dt;

    last_left_ticks = left_ticks;
    last_right_ticks = right_ticks;

    auto wheel_positions = std::make_tuple(RADIANS_PER_TICK * (float)last_left_ticks,
                                           RADIANS_PER_TICK * (float)last_right_ticks);

    xQueueSendToFront(driveTrain_positionTrackingWheelPositionQueue, &wheel_positions, 0);

    telemetry::publish_f32(telemetry::KEY_LEFT_WHEEL_VELOCITY, left_velocity);
    telemetry::publish_f32(telemetry::KEY_RIGHT_WHEEL_VELOCITY, right_velocity);
}

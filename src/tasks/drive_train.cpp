#include "tasks/drive_train.hpp"
#include "queues.hpp"
#include "home_selection.hpp"
#include "enemy_base.hpp"
#include "telemetry_bus.hpp"
#include <mutexes.hpp>
#include "drive_enable.hpp"
#include "dummy_rejection.hpp"
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
    button_stable_pressed = button_raw_pressed;
    button_changed_at = millis();
    button_armed = false; // Require a stable release, including at boot.
    last_timestamp = micros();
    Serial.println("DRIVE: inhibited; press blue to enable");
}

void DriveTrainTask::update_drive_button() {
    const uint32_t now = millis();
    const bool pressed = digitalRead(BLUE_BUTTON_PIN) ==
                         (BLUE_BUTTON_ACTIVE_LOW ? LOW : HIGH);
    if (pressed != button_raw_pressed) {
        button_raw_pressed = pressed;
        button_changed_at = now;
    }
    if (now - button_changed_at < BLUE_BUTTON_DEBOUNCE_MS) return;
    if (!pressed) button_armed = true;
    if (pressed == button_stable_pressed) return;
    button_stable_pressed = pressed;
    if (pressed && button_armed) {
        button_armed = false;
        const bool enabled = !drive_enabled.load() && !drive_start_pending.load();
        // Discard commands from before this transition.
        xQueueReset(motionControl_ChassisCommandsQueue);
        left_command = right_command = 0.0f;
        left_slew = SlewRate<float>(SLEW_RATE);
        right_slew = SlewRate<float>(SLEW_RATE);
        left_motor.writeMicroseconds(1500);
        right_motor.writeMicroseconds(1500);
        drive_enabled.store(false);
        if (enabled) {
            const bool blue = selected_home_blue.load();
            const bool new_base = home_request_generation.load() == 0 ||
                                  blue != active_home_blue.load();
            drive_start_pending.store(true);
            if (new_base) {
                active_home_blue.store(blue);
                home_request_generation.fetch_add(1);
            }
            Serial.println(blue ? "HOME: blue latched" : "HOME: green latched");
        } else {
            drive_start_pending.store(false);
            Serial.println("DRIVE: inhibited");
        }
    }
}

static const int TICKS_PER_REVOLUTION = 2652;
static const int FORWARD_MS = 1950;
static const int REVERSE_MS = 1050;

static const float RADIANS_PER_TICK = 2.0 * PI / (float)TICKS_PER_REVOLUTION;

static const float LEFT_SCALE = 0.75;

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
    const uint32_t generation = home_request_generation.load();
    if (drive_start_pending.load() && home_pose_generation.load() == generation &&
        home_map_generation.load() == generation) {
        xQueueReset(motionControl_ChassisCommandsQueue);
        left_command = right_command = 0.0f;
        drive_start_pending.store(false);
        drive_enabled.store(true);
        Serial.println("DRIVE: enabled; home ready");
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

    if (left_out > 0.0) {
        left_out *= LEFT_SCALE;
    }

    left_out = apply_kickoff(left_out);
    right_out = apply_kickoff(right_out);

    float left_rate = this->left_slew.update(left_out);
    float right_rate = this->right_slew.update(right_out);

    // Final guard covers path following, timed pickups, and reverse overrides.
    // Reset the ramp as well, so a stop is not delayed by slew limiting.
    if (drive_enabled.load()) {
        const auto pose = get_global_pose();
        const float translation = (left_rate + right_rate) * 0.5f;
        const float preview = translation > 0.0f ? 0.35f : translation < 0.0f ? -0.35f : 0.0f;
        if (crosses_enemy_base(pose.position,
                              pose.position + pose.get_direction_vector() * preview)) {
            left_rate = right_rate = 0.0f;
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
    if (!drive_enabled.load()) left_rate = right_rate = 0.0f;

    left_motor.writeMicroseconds(map(left_rate, 1.0, -1.0, FORWARD_MS, REVERSE_MS));
    right_motor.writeMicroseconds(map(right_rate, 1.0, -1.0, REVERSE_MS, FORWARD_MS));

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

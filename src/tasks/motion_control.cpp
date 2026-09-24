#include "tasks/motion_control.hpp"
#include "queues.hpp"
#include "telemetry_bus.hpp"
#include <mutexes.hpp>

#define DRIVE_KP 25e-1 //alex 40e-1
#define DRIVE_KI 0 // 20e-4

#define TURN_KP 1.4//alex - 1
#define TURN_KI 0
#define TURN_KD 50e-2 //alex - 30e-2
       
//------- Joel edits -------
static const uint32_t STUCK_TIME_MS = 120;
static const uint32_t STUCK_RECOVERY_STARTUP_DELAY_MS = 3000;
static const uint32_t REVERSE_TIME_MS = 1400;

static const float REVERSE_COMMAND = 0.20;
static const float DUMMY_WEIGHT_REVERSE_SPEED = 0.10f;
static const float HOME_DROP_REVERSE_SPEED = 0.20f;
static const float PICKUP_REVERSE_SPEED = 0.20f;
// Match the original maximum forward pickup command (0.5 * 1.5).
static const float PICKUP_FORWARD_SPEED = 0.75f;
// Extra inner-wheel reduction per unit of steering command during forward arcs.
// Increase for tighter turns; zero leaves the standard drive/turn mix unchanged.
static const float INNER_WHEEL_TURN_REDUCTION = 0.5f;

static uint32_t motion_control_start_time = 0;
static bool stuck_recovery_enabled = false;
static bool stuck_timer_running = false;
static bool reversing = false;

static uint32_t stuck_start_time = 0;
static uint32_t reverse_start_time = 0;

static MotionControlOverride motion_override = MotionControlOverride::None;


//---------------------------

MotionControlTask::MotionControlTask()
    : SchedulerTask("motion_control"), pure_pursuit(0.6),
      pid_drive(PIDController(DRIVE_KP, DRIVE_KI, 0, 10)
                    .with_output_limits(-0.5, 0.5)
                    .with_integral_bounds(-100, 100)),
      pid_turn(PIDController(TURN_KP, TURN_KI, TURN_KD, 3 * DEG_TO_RAD)
                   .with_output_limits(-2.0, 2.0)
                   .with_integral_bounds(-30 * DEG_TO_RAD, 30 * DEG_TO_RAD)) {}

void MotionControlTask::setup() {
    motion_control_start_time = millis();
    stuck_recovery_enabled = false;
    stuck_timer_running = false;
    reversing = false;
}

void MotionControlTask::loop() {
    MotionControlOverride new_override;
    if (xQueueReceive(motion_control_override_queue, &new_override, 0)) {
        motion_override = new_override;
    }

    auto path = get_motion_control_path();
    this->pure_pursuit.set_current_path(path.path);

    auto pose = get_global_pose();

    auto [drive_error, turn_error] = pure_pursuit.compute_errors(pose);

    telemetry::publish_f32(telemetry::KEY_TURN_ERROR, turn_error);
    telemetry::publish_f32(telemetry::KEY_DRIVE_ERROR, drive_error);

    float drive_output = pid_drive.update(drive_error) * path.speed;
    float turn_output = pid_turn.update(turn_error);

    float left_drive = drive_output + turn_output;
    float right_drive = drive_output - turn_output;

    //------- Joel edits -------
    bool motion_mismatch = get_robot_motion_mismatch();
    uint32_t now = millis();

    // Enable once after startup; do not accumulate stuck time during the delay.
    if (!stuck_recovery_enabled &&
        now - motion_control_start_time >= STUCK_RECOVERY_STARTUP_DELAY_MS) {
        stuck_recovery_enabled = true;
    }

    if (motion_override == MotionControlOverride::DummyWeightReverse ||
        motion_override == MotionControlOverride::DummyWeightHold ||
        motion_override == MotionControlOverride::DummyWeightClearanceReverse ||
        motion_override == MotionControlOverride::HomeDropHold ||
        motion_override == MotionControlOverride::HomeDropReverse ||
        motion_override == MotionControlOverride::PickupReverse ||
        motion_override == MotionControlOverride::PickupHold ||
        motion_override == MotionControlOverride::PickupForward) {
        reversing = false;
        stuck_timer_running = false;
    } else if (!stuck_recovery_enabled) {
        stuck_timer_running = false;
    } else if (!reversing) {
        if (motion_mismatch) {
            if (!stuck_timer_running) {
                stuck_timer_running = true;
                stuck_start_time = now;
            } else if (now - stuck_start_time >= STUCK_TIME_MS) {
                reversing = true;
                reverse_start_time = now;
                stuck_timer_running = false;
            }
        } else {
            stuck_timer_running = false;
        }
    }

    if (reversing) {
        if (now - reverse_start_time < REVERSE_TIME_MS) {
            left_drive = -REVERSE_COMMAND;
            right_drive = -REVERSE_COMMAND;
        } else {
            reversing = false;
        }
    }
    //---------------------------

    // Let autonomous raise the rails and suspend pickup decisions while this
    // existing encoder/motion-mismatch recovery is backing the robot away.
    bool recovery_reversing = reversing;
    xQueueOverwrite(motion_control_recovery_reversing_queue, &recovery_reversing);

    // Autonomous dummy rejection takes precedence over path following (and the
    // normal stuck-recovery reverse) while the intake sequence is active.
    if (motion_override == MotionControlOverride::PickupReverse) {
        left_drive = right_drive = -PICKUP_REVERSE_SPEED;
    } else if (motion_override == MotionControlOverride::PickupForward) {
        left_drive = right_drive = PICKUP_FORWARD_SPEED;
    } else if (motion_override == MotionControlOverride::DummyWeightHold ||
               motion_override == MotionControlOverride::HomeDropHold ||
               motion_override == MotionControlOverride::PickupHold) {
        left_drive = 0.0f;
        right_drive = 0.0f;
    } else if (motion_override == MotionControlOverride::DummyWeightReverse ||
               motion_override == MotionControlOverride::DummyWeightClearanceReverse) {
        left_drive = -DUMMY_WEIGHT_REVERSE_SPEED;
        right_drive = -DUMMY_WEIGHT_REVERSE_SPEED;
    } else if (motion_override == MotionControlOverride::HomeDropReverse) {
        left_drive = -HOME_DROP_REVERSE_SPEED;
        right_drive = -HOME_DROP_REVERSE_SPEED;
    }

    // Tighten forward arcs by slowing the inner wheel in proportion to steering.
    // Preserve reverse overrides, recovery, and turns with a wheel already reversing.
    if (motion_override == MotionControlOverride::None && !reversing &&
        drive_output > 0.0f && left_drive >= 0.0f && right_drive >= 0.0f) {
        float reduction = INNER_WHEEL_TURN_REDUCTION * fabs(turn_output);
        if (turn_output > 0.0f) {
            right_drive = fmax(0.0f, right_drive - reduction);
        } else if (turn_output < 0.0f) {
            left_drive = fmax(0.0f, left_drive - reduction);
        }
    }

    float largest_cmd = fabs(fmax(left_drive, right_drive));

    if (largest_cmd > 1.0) {
        left_drive = left_drive / largest_cmd;
        right_drive = right_drive / largest_cmd;
    }

    // Recovery raises the rails; deliberate release, rejection, and pickup realignment do not.
    bool force_rails_up = motion_override == MotionControlOverride::DummyWeightHold ||
                         (left_drive + right_drive < 0.0f &&
                          motion_override != MotionControlOverride::HomeDropReverse &&
                          motion_override != MotionControlOverride::DummyWeightReverse &&
                          motion_override != MotionControlOverride::PickupReverse);
    xQueueOverwrite(motion_control_force_rails_up_queue, &force_rails_up);

    std::tuple<float, float> commands = std::make_tuple(left_drive, right_drive);

    xQueueSendToFront(motionControl_ChassisCommandsQueue, &commands, 0);
}

#include "tasks/intake.hpp"
#include "Arduino.h"
#include <mutexes.hpp>
#include <queues.hpp>

IntakeTask::IntakeTask() : SchedulerTask("intake_task") {}

static const int WEIGHT_DETECTION_DEBOUNCE_MS = 8;
static bool storage_voltage_probe_initialized = false;
static bool storage_voltage_probe_last_high = false;
static uint32_t storage_voltage_probe_last_print_time = 0;
static const uint32_t STORAGE_VOLTAGE_PROBE_PRINT_INTERVAL_MS = 500;

const int KI = 0x0200;

int angleToNum(float angle) { return 512 + (int)(angle / 0.325); }

void IntakeTask::setup() {
    Serial.println("INTAKE SETUP START");

    Serial7.begin(115200);

    left_servo.setTorqueOn();
    right_servo.setTorqueOn();

    this->set_position(true);

    if (!expander.begin(0x3E)) {
        Serial.println("Failed to communicate with SX1509. Check wiring!");
    }

    expander.pinMode(ENTRY_CONDUCTION_PIN, INPUT);
    expander.pinMode(ENTRY_SWITCH_PIN, INPUT);
    expander.pinMode(UPSIDE_DOWN_WEIGHT_SWITCH_PIN, INPUT);
    expander.pinMode(STORAGE_VOLTAGE_PROBE_PIN, INPUT);
    expander.pinMode(EARLY_INTAKE_PROBE_PIN, INPUT);
    expander.debouncePin(EARLY_INTAKE_PROBE_PIN);

    expander.debouncePin(ENTRY_CONDUCTION_PIN);
    expander.debouncePin(ENTRY_SWITCH_PIN);
    expander.debouncePin(UPSIDE_DOWN_WEIGHT_SWITCH_PIN);

    expander.debounceTime(WEIGHT_DETECTION_DEBOUNCE_MS);

    left_servo.writeRam(HerkulexRamRegister::AccelerationRatio, 0x10);
    left_servo.writeRam2(HerkulexRamRegister::PositionKd, 0);
    left_servo.writeRam2(HerkulexRamRegister::PositionKi, KI);

    right_servo.writeRam(HerkulexRamRegister::AccelerationRatio, 0x10);
    right_servo.writeRam2(HerkulexRamRegister::PositionKd, 0);
    right_servo.writeRam2(HerkulexRamRegister::PositionKi, KI);
}

void IntakeTask::set_position(bool up) {
    // Guard every servo command, including direct sensor-triggered drops.
    bool force_rails_up = false;
    xQueuePeek(motion_control_force_rails_up_queue, &force_rails_up, 0);
    up = up || force_rails_up;

    // Send each target once so repeated task ticks do not restart a timed move.
    if (rail_position_commanded && commanded_rails_up == up) {
        return;
    }
    rail_position_commanded = true;
    commanded_rails_up = up;

    if (up) {
        left_servo.setPosition(angleToNum(-5.0), 25, HerkulexLed::Green);
        right_servo.setPosition(angleToNum(25.0), 25, HerkulexLed::Green);
    } else {
        left_servo.setPosition(angleToNum(45.0), 5, HerkulexLed::Blue);
        right_servo.setPosition(angleToNum(-25.0), 5, HerkulexLed::Blue);
    }
}

// Alternate servos: each is checked twice a second without waiting for a reply.
static const uint32_t SERVO_STATUS_INTERVAL_MS = 250;
static const uint32_t SERVO_STATUS_TIMEOUT_MS = 100;
static const uint32_t SERVO_RESET_DELAY_MS = 2000;
static const uint8_t SERVO_MAX_RESET_ATTEMPTS = 3;
static const uint8_t RAIL_SERVO_IDS[] = {3, 2};
static const char *RAIL_SERVO_NAMES[] = {"left", "right"};

static void print_servo_health(uint8_t index, uint8_t error, uint8_t detail) {
    Serial.printf("RAIL SERVO %s id=%u error=0x%02X detail=0x%02X motor=%s:",
                  RAIL_SERVO_NAMES[index], RAIL_SERVO_IDS[index], error, detail,
                  (detail & 0x40) ? "ON" : "OFF");
    if (error & 0x01) Serial.print(" INPUT_VOLTAGE");
    if (error & 0x02) Serial.print(" POSITION_LIMIT");
    if (error & 0x04) Serial.print(" TEMPERATURE_LIMIT");
    if (error & 0x08) Serial.print(" INVALID_PACKET");
    if (error & 0x10) Serial.print(" OVERLOAD");
    if (error & 0x20) Serial.print(" DRIVER_FAULT");
    if (error & 0x40) Serial.print(" EEPROM_FAULT");
    if (error & 0x80) Serial.print(" RESERVED_ERROR");
    if (detail & 0x04) Serial.print(" CHECKSUM_ERROR");
    if (detail & 0x08) Serial.print(" UNKNOWN_COMMAND");
    if (detail & 0x10) Serial.print(" REGISTER_RANGE");
    if (detail & 0x20) Serial.print(" GARBAGE_DETECTED");
    if (!(detail & 0x40)) Serial.print(" MOTOR_DISABLED");
    if (!error && !(detail & 0x3C) && (detail & 0x40)) Serial.print(" OK");
    Serial.println();
}

void IntakeTask::monitor_servos() {
    const uint32_t now = millis();
    // Drain a bounded number of packets, including acknowledgements to movement commands.
    // Only a valid STAT reply from the requested servo completes a status poll.
    for (int i = 0; i < 4; ++i) {
        herkulexBus.update();
        HerkulexPacket packet{};
        if (!herkulexBus.getPacket(packet)) break;
        if (!servo_status_pending || packet.error != HerkulexPacketError::None ||
            packet.size != 9 || packet.cmd != HerkulexCommand::Stat ||
            packet.id != RAIL_SERVO_IDS[servo_status_index]) continue;

        auto &health = servo_health[servo_status_index];
        // Ignore normal moving/in-position changes when deciding whether to log.
        const uint8_t detail = packet.status_detail & 0xFC;
        if (!health.initialized || !health.responding ||
            health.error != packet.status_error || health.detail != detail) {
            if (health.initialized && !health.responding) {
                Serial.printf("RAIL SERVO %s id=%u: communication restored\n",
                              RAIL_SERVO_NAMES[servo_status_index], packet.id);
            }
            print_servo_health(servo_status_index, packet.status_error, packet.status_detail);
        }
        health.initialized = true;
        health.responding = true;
        health.error = packet.status_error;
        health.detail = detail;
        const bool needs_reset = health.error != 0 || (detail & 0x3C) != 0 || !(detail & 0x40);
        // Do not re-enable a servo reporting voltage, heat, driver, or EEPROM faults.
        const bool reset_allowed = (health.error & 0xE5) == 0;
        if (!needs_reset || !reset_allowed) {
            health.fault_pending = false;
        } else if (!health.fault_pending) {
            health.fault_pending = true;
            health.fault_since = now;
        } else if (health.reset_attempts < SERVO_MAX_RESET_ATTEMPTS &&
                   now - health.fault_since >= SERVO_RESET_DELAY_MS) {
            auto &servo = servo_status_index == 0 ? left_servo : right_servo;
            ++health.reset_attempts;
            health.fault_since = now;
            Serial.printf("RAIL SERVO %s id=%u: reset attempt %u/%u\n",
                          RAIL_SERVO_NAMES[servo_status_index], servo.getID(),
                          health.reset_attempts, SERVO_MAX_RESET_ATTEMPTS);
            // Clear the two status bytes, re-enable torque, and restore the target.
            servo.writeRam2(HerkulexRamRegister::StatusError, 0);
            servo.setTorqueOn();
            bool up = commanded_rails_up;
            bool force_up = false;
            xQueuePeek(motion_control_force_rails_up_queue, &force_up, 0);
            up = up || force_up;
            if (servo_status_index == 0) {
                servo.setPosition(angleToNum(up ? -5.0f : 45.0f), up ? 25 : 5,
                                  up ? HerkulexLed::Green : HerkulexLed::Blue);
            } else {
                servo.setPosition(angleToNum(up ? 25.0f : -25.0f), up ? 25 : 5,
                                  up ? HerkulexLed::Green : HerkulexLed::Blue);
            }
            // Only later valid status replies can confirm recovery.
            if (health.reset_attempts == SERVO_MAX_RESET_ATTEMPTS) {
                Serial.printf("RAIL SERVO %s: automatic reset budget exhausted until restart\n",
                              RAIL_SERVO_NAMES[servo_status_index]);
            }
        }
        servo_status_pending = false;
        servo_status_index ^= 1;
    }

    if (servo_status_pending && now - servo_status_last_request >= SERVO_STATUS_TIMEOUT_MS) {
        auto &health = servo_health[servo_status_index];
        if (!health.initialized || health.responding) {
            Serial.printf("RAIL SERVO %s id=%u: NO VALID STATUS REPLY (fault unknown)\n",
                          RAIL_SERVO_NAMES[servo_status_index], RAIL_SERVO_IDS[servo_status_index]);
        }
        health.initialized = true;
        health.responding = false;
        health.fault_pending = false;
        servo_status_pending = false;
        servo_status_index ^= 1;
    }
    if (!servo_status_pending && now - servo_status_last_request >= SERVO_STATUS_INTERVAL_MS) {
        herkulexBus.sendPacket(RAIL_SERVO_IDS[servo_status_index], HerkulexCommand::Stat);
        servo_status_last_request = now;
        servo_status_pending = true;
    }
}

const int IO_EXPANDER_ADDRESS = 0x3E;
const int PIN_INPUT_STATE_ADDRESS = 0x10;

const int CONDUCTION_DEBOUNCER_TIME = 100;

uint16_t readPins() {
    // Manually read the pins instead of using the expander library, as the library uses redundant
    // multiple tranmissions when reading more than 1 pin

    if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
        Wire.beginTransmission(IO_EXPANDER_ADDRESS);
        Wire.write(PIN_INPUT_STATE_ADDRESS);
        Wire.endTransmission();
        Wire.requestFrom(IO_EXPANDER_ADDRESS, (uint8_t)2);

        uint16_t msb = (Wire.read() & 0x00FF) << 8;
        uint16_t lsb = (Wire.read() & 0x00FF);
        uint16_t readValue = msb | lsb;

        xSemaphoreGive(i2cMutex);

        return msb | lsb;
    }
}

void IntakeTask::loop() {
    monitor_servos();

    uint16_t pins = readPins();
    bool early_probe_active = (pins & (1 << EARLY_INTAKE_PROBE_PIN)) == 0;
    xQueueOverwrite(early_intake_probe_queue, &early_probe_active);

    bool reset_carried_weight_count;
    if (xQueueReceive(intake_reset_carried_weight_count_queue, &reset_carried_weight_count, 0) &&
        reset_carried_weight_count) {
        this->total_weights = 0;
    }

    bool intake_command = true;
    bool force_rails_up = false;
    xQueuePeek(motion_control_force_rails_up_queue, &force_rails_up, 0);
    const bool has_intake_command = xQueueReceive(intake_position_queue, &intake_command, 0);

    // Raise on the next intake tick even without an autonomous command.
    if (has_intake_command || force_rails_up) {
        set_position(intake_command);
    }

    bool conduction_state = (pins & (1 << ENTRY_CONDUCTION_PIN)) == 0;
    bool switch_state = (pins & (1 << ENTRY_SWITCH_PIN)) == 0;
    bool upside_down_state = (pins & (1 << UPSIDE_DOWN_WEIGHT_SWITCH_PIN)) == 0;
    bool release_clear = !conduction_state && !switch_state && !upside_down_state;
    xQueueOverwrite(intake_release_clear_queue, &release_clear);
    bool storage_voltage_probe_high = (pins & (1 << STORAGE_VOLTAGE_PROBE_PIN)) != 0;
    xQueueOverwrite(storage_voltage_probe_queue, &storage_voltage_probe_high);
    uint32_t now = millis();

    if (!storage_voltage_probe_initialized ||
        storage_voltage_probe_high != storage_voltage_probe_last_high ||
        now - storage_voltage_probe_last_print_time >= STORAGE_VOLTAGE_PROBE_PRINT_INTERVAL_MS) {
        Serial.println(storage_voltage_probe_high ? "HIGH" : "LOW");
        storage_voltage_probe_last_high = storage_voltage_probe_high;
        storage_voltage_probe_initialized = true;
        storage_voltage_probe_last_print_time = now;
    }

    bool pickup_active = false;
    xQueuePeek(intake_pickup_active_queue, &pickup_active, 0);

    int time = millis();

    if (conduction_state) {
        this->last_conduction_time = time;
    }

    // Upside-down metal is rejected just like a non-conductive dummy. Keep the
    // classification latched until the object clears both switches and the probe.
    if (upside_down_state) {
        if (weight_intake_state != WeightIntakeState::DummyWeightDetected) {
            if (weight_intake_state == WeightIntakeState::RealWeightDetected && total_weights > 0) {
                --total_weights;
            }
            weight_intake_state = WeightIntakeState::DummyWeightDetected;
            set_position(false);
            bool val = false;
            xQueueSend(intake_entry_queue, &val, 0);
        }
    } else if (conduction_state && weight_intake_state != WeightIntakeState::DummyWeightDetected) {
        if (weight_intake_state != WeightIntakeState::RealWeightDetected) {
            weight_intake_state = WeightIntakeState::RealWeightDetected;
            total_weights++;
            bool val = true;
            xQueueSend(intake_entry_queue, &val, 0);
        }
    }

    switch (weight_intake_state) {
    case WeightIntakeState::None:
        if (switch_state) {
            weight_intake_state = WeightIntakeState::UnknownWeight;
        }

        break;
    case WeightIntakeState::UnknownWeight: {
        if (!switch_state) {
            weight_intake_state = WeightIntakeState::None;
        } else if (conduction_state) {
            weight_intake_state = WeightIntakeState::RealWeightDetected;
            total_weights++;
            bool val = true;
            xQueueSend(intake_entry_queue, &val, 0);
        } else if (!conduction_state) {
            weight_intake_state = WeightIntakeState::DummyWeightDetected;

            // Do this in the sensor-owning task rather than waiting for the
            // autonomous queue consumer.  A command to raise the rails may
            // already be pending from the approach phase; the dummy must not
            // be allowed to ride that command into the intake.
            set_position(false);

            bool val = false;
            xQueueSend(intake_entry_queue, &val, 0);
        }

        break;
    }
    case WeightIntakeState::RealWeightDetected:
    case WeightIntakeState::DummyWeightDetected:
        // Backing away deliberately clears the probe. Keep this weight counted
        // once until the forward pickup finishes; upside-down rejection still wins.
        if (!(pickup_active && weight_intake_state == WeightIntakeState::RealWeightDetected) &&
            !switch_state && !upside_down_state &&
            time - last_conduction_time > CONDUCTION_DEBOUNCER_TIME) {
            weight_intake_state = WeightIntakeState::None;
        }
        break;
    }

    xQueueOverwrite(carried_weight_count, &this->total_weights);
}

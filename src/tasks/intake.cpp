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
    if (up) {
        left_servo.setPosition(angleToNum(-5.0), 25, HerkulexLed::Green);
        right_servo.setPosition(angleToNum(25.0), 25, HerkulexLed::Green);
    } else {
        left_servo.setPosition(angleToNum(45.0), 5, HerkulexLed::Blue);
        right_servo.setPosition(angleToNum(-25.0), 5, HerkulexLed::Blue);
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
    herkulexBus.update();

    uint16_t pins = readPins();

    bool reset_carried_weight_count;
    if (xQueueReceive(intake_reset_carried_weight_count_queue, &reset_carried_weight_count, 0) &&
        reset_carried_weight_count) {
        this->total_weights = 0;
    }

    bool intake_command;

    if (xQueueReceive(intake_position_queue, &intake_command, 0)) {
        set_position(intake_command);
    }

    bool conduction_state = (pins & (1 << ENTRY_CONDUCTION_PIN)) == 0;
    bool switch_state = (pins & (1 << ENTRY_SWITCH_PIN)) == 0;
    bool upside_down_state = (pins & (1 << UPSIDE_DOWN_WEIGHT_SWITCH_PIN)) == 0;
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
        if (!switch_state && !upside_down_state &&
            time - last_conduction_time > CONDUCTION_DEBOUNCER_TIME) {
            weight_intake_state = WeightIntakeState::None;
        }
        break;
    }

    xQueueOverwrite(carried_weight_count, &this->total_weights);
}

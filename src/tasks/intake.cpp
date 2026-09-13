#include "tasks/intake.hpp"
#include "Arduino.h"
#include <mutexes.hpp>

IntakeTask::IntakeTask() : SchedulerTask("intake_task") {}

static const int WEIGHT_DETECTION_DEBOUNCE_MS = 8;

const int KI = 0x0200;

int angleToNum(float angle) { return 512 + (int)(angle / 0.325); }

void IntakeTask::setup() {
    Serial7.begin(115200);

    left_servo.setTorqueOn();
    right_servo.setTorqueOn();

    this->set_position(true);

    if (!expander.begin(0x3E)) {
        Serial.println("Failed to communicate with SX1509. Check wiring!");
    }

    expander.pinMode(ENTRY_CONDUCTION_PIN, INPUT);
    expander.pinMode(ENTRY_SWITCH_PIN, INPUT);

    expander.debouncePin(ENTRY_CONDUCTION_PIN);
    expander.debouncePin(ENTRY_SWITCH_PIN);

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
        left_servo.setPosition(angleToNum(45.0), 25, HerkulexLed::Blue);
        right_servo.setPosition(angleToNum(-25.0), 25, HerkulexLed::Blue);
    }
}

const int IO_EXPANDER_ADDRESS = 0x3E;
const int PIN_INPUT_STATE_ADDRESS = 0x10;

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

    bool conduction_state = (pins & (1 << ENTRY_CONDUCTION_PIN)) == 0;
    bool switch_state = (pins & (1 << ENTRY_SWITCH_PIN)) == 0;

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
        } else if (!conduction_state) {
            weight_intake_state = WeightIntakeState::DummyWeightDetected;
            total_weights++;
        }

        break;
    }
    case WeightIntakeState::RealWeightDetected:
    case WeightIntakeState::DummyWeightDetected:
        if (!switch_state) {
            weight_intake_state = WeightIntakeState::None;
        }
        break;
    }
}

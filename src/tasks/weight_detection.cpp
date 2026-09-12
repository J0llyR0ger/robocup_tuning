#include "tasks/weight_detection.hpp"

WeightDetectionTask::WeightDetectionTask() : SchedulerTask("weight_detection") {}

static const int WEIGHT_DETECTION_DEBOUNCE_MS = 8;

void WeightDetectionTask::setup() {
    if (!expander.begin(0x3E)) {
        Serial.println("Failed to communicate with SX1509. Check wiring!");
    }

    expander.pinMode(ENTRY_CONDUCTION_PIN, INPUT);
    expander.pinMode(ENTRY_SWITCH_PIN, INPUT);

    expander.debouncePin(ENTRY_CONDUCTION_PIN);
    expander.debouncePin(ENTRY_SWITCH_PIN);

    expander.debounceTime(WEIGHT_DETECTION_DEBOUNCE_MS);
}

const int IO_EXPANDER_ADDRESS = 0x3E;
const int PIN_INPUT_STATE_ADDRESS = 0x10;

uint16_t readPins() {
    // Manually read the pins instead of using the expander library, as the library uses redundant
    // multiple tranmissions when reading more than 1 pin

    Wire.beginTransmission(IO_EXPANDER_ADDRESS);
    Wire.write(PIN_INPUT_STATE_ADDRESS);
    Wire.endTransmission();
    Wire.requestFrom(IO_EXPANDER_ADDRESS, (uint8_t)2);

    uint16_t msb = (Wire.read() & 0x00FF) << 8;
    uint16_t lsb = (Wire.read() & 0x00FF);
    uint16_t readValue = msb | lsb;

    return msb | lsb;
}

void WeightDetectionTask::loop() {
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
        } else if (!conduction_state) {
            weight_intake_state = WeightIntakeState::DummyWeightDetected;
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
// Register reference: https://github.com/adafruit/Adafruit_TCS34725
// Mux reference: https://www.ti.com/lit/ds/symlink/tca9548a.pdf
#include "arduino_freertos.h"
#include "colour_sensor.hpp"
#include "lib/colour.hpp"
#include "mutexes.hpp"
#include <Arduino.h>
#include <Wire.h>
#include <atomic>

namespace {
constexpr uint8_t ADDRESS = 0x29;
enum class State { Offline, Powering, Integrating, Ready };
State state = State::Offline;
uint32_t changed_at = 0, last_good = 0, last_log = 0;
bool attempted = false;
colour::Value candidate = colour::Value::Unknown, stable = colour::Value::Unknown;
unsigned count = 0;
const char *label = "NO SENSOR";
std::atomic<colour::Value> published_colour{colour::Value::Unknown};
std::atomic<uint32_t> published_at{0};

bool write_register(uint8_t reg, uint8_t value) {
    Wire1.beginTransmission(ADDRESS);
    Wire1.write(uint8_t(0x80 | reg));
    Wire1.write(value);
    return Wire1.endTransmission() == 0;
}
bool read_registers(uint8_t reg, uint8_t *data, uint8_t size) {
    Wire1.beginTransmission(ADDRESS);
    Wire1.write(uint8_t(0xA0 | reg)); // Command bit plus auto-increment.
    if (Wire1.endTransmission(false) != 0) return false;
    if (Wire1.requestFrom(ADDRESS, size) != size) {
        while (Wire1.available()) Wire1.read();
        return false;
    }
    for (uint8_t i = 0; i < size; ++i) data[i] = Wire1.read();
    return true;
}
bool mux_write(uint8_t value) {
    Wire1.beginTransmission(colour_config::MUX_ADDRESS);
    Wire1.write(value);
    return Wire1.endTransmission() == 0;
}
// Preserve the previous channel mask, and hold the bus mutex through restoration.
struct Bus {
    bool locked = false, selected = false, ok = false;
    uint8_t previous = 0;
    Bus() {
        locked = xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5)) == pdTRUE;
        if (!locked) return;
        ok = true;
        if (!colour_config::USE_MUX) return;
        Wire1.beginTransmission(colour_config::MUX_ADDRESS);
        if (Wire1.endTransmission() != 0) return; // Direct-bus fallback.
        if (Wire1.requestFrom(colour_config::MUX_ADDRESS, uint8_t(1)) != 1) { ok = false; return; }
        previous = Wire1.read();
        selected = true;
        ok = mux_write(uint8_t(1u << colour_config::MUX_CHANNEL));
    }
    ~Bus() {
        if (selected && !mux_write(previous)) {
            state = State::Offline;
            label = "NO SENSOR";
        }
        if (locked) xSemaphoreGive(i2cMutex);
    }
};
void failed(uint32_t now) {
    state = State::Offline; changed_at = now; count = 0; label = "NO SENSOR";
}
}

static const char *read_colour_sensor() {
    const uint32_t now = millis();
    if (state == State::Ready && now - last_good > 500) label = "NO SENSOR";
    if (state == State::Offline && attempted && now - changed_at < 1000) return label;
    {
        Bus bus;
        if (!bus.locked) return label;
        if (!bus.ok) { attempted = true; failed(now); }
        else if (state == State::Offline) {
            attempted = true;
            uint8_t id = 0;
            // Verify identity; an address ACK alone is not sufficient.
            if (!read_registers(0x12, &id, 1) || (id != 0x44 && id != 0x4d) ||
                !write_register(0x01, 0xeb) || !write_register(0x0f, 0x01) ||
                !write_register(0x00, 0x01)) failed(now);
            else {
                state = State::Powering; changed_at = now; label = "STARTING";
                Serial.printf("COLOUR: TCS34725 on Wire1 (CON62), %s, channel=%u\n", bus.selected ? "mux" : "direct", colour_config::MUX_CHANNEL);
            }
        } else if (state == State::Powering && now - changed_at >= 3) {
            if (!write_register(0x00, 0x03)) failed(now);
            else { state = State::Integrating; changed_at = now; }
        } else if (state == State::Ready || (state == State::Integrating && now - changed_at >= 60)) {
            uint8_t status = 0, raw[8] = {};
            if (!read_registers(0x13, &status, 1) || !(status & 1) || !read_registers(0x14, raw, 8)) failed(now);
            else {
                const uint16_t c = raw[0] | (uint16_t(raw[1]) << 8);
                const uint16_t r = raw[2] | (uint16_t(raw[3]) << 8);
                const uint16_t g = raw[4] | (uint16_t(raw[5]) << 8);
                const uint16_t b = raw[6] | (uint16_t(raw[7]) << 8);
                const auto value = colour::classify(r, g, b, c);
                if (value != candidate || count == 0) { candidate = value; count = 1; }
                else if (count < colour_config::STABLE_SAMPLES) ++count;
                if (count >= colour_config::STABLE_SAMPLES) stable = candidate;
                state = State::Ready; last_good = now;
                label = count >= colour_config::STABLE_SAMPLES ? colour::name(stable) : "READING";
                if (now - last_log >= 1000) {
                    last_log = now;
                    Serial.printf("COLOUR: %s R=%u G=%u B=%u C=%u\n", label, r, g, b, c);
                }
            }
        }
    }
    return label;
}

const char *poll_colour_sensor() {
    const char *result = read_colour_sensor();
    published_colour.store(colour::Value::Unknown);
    published_at.store(last_good);
    if (state == State::Ready && count >= colour_config::STABLE_SAMPLES &&
        millis() - last_good <= 500) published_colour.store(stable);
    return result;
}

colour::Value current_colour() {
    const uint32_t sampled_at = published_at.load();
    const auto value = published_colour.load();
    return millis() - sampled_at <= 500 ? value : colour::Value::Unknown;
}

#include "arduino_freertos.h"
#include "tasks/display.hpp"
#include "config/display.hpp"
#include "drive_enable.hpp"
#include "mutexes.hpp"
#include <Arduino.h>
#include <Wire.h>
#include <U8x8lib.h>
#include <cstdio>
#include <cstring>

namespace {
#if DISPLAY_USE_SH1106
U8X8_SH1106_128X64_NONAME_HW_I2C display(U8X8_PIN_NONE);
#else
U8X8_SSD1306_128X64_NONAME_HW_I2C display(U8X8_PIN_NONE);
#endif

// CON68 A0Z = VRx, CON69 A1Z = VRy. The Teensy ADC defaults to 10 bits.
constexpr int JOYSTICK_X_PIN = A0;
constexpr int JOYSTICK_Y_PIN = A1;
constexpr int JOYSTICK_CENTRE = 512;
constexpr int JOYSTICK_DEAD_ZONE = 100;
// A smaller return zone prevents noise at the edge from counting as a new move.
constexpr int JOYSTICK_RETURN_ZONE = 60;
unsigned selected_item = 0; // 0 = Home, 1 = M
bool home_blue = false;
bool menu_m = false;
bool joystick_armed = false;
unsigned centred_samples = 0;
constexpr uint8_t DISPLAY_ROWS[] = {0, 2, 3, 5};
char displayed_lines[4][17] = {};

void display_loop(void *) {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100));
        char lines[4][17] = {};
        bool inhibited = !drive_enabled.load();
        if (inhibited) {
            const int x = analogRead(JOYSTICK_X_PIN) - JOYSTICK_CENTRE;
            if (drive_enabled.load()) {
                joystick_armed = false;
                centred_samples = 0;
                continue;
            }
            const int y = analogRead(JOYSTICK_Y_PIN) - JOYSTICK_CENTRE;
            const int abs_x = abs(x);
            const int abs_y = abs(y);
            // Only the short menu-state update is protected; ADC/I2C stay outside.
            taskENTER_CRITICAL();
            if (drive_enabled.load()) {
                joystick_armed = false;
                centred_samples = 0;
            } else if (abs_x <= JOYSTICK_RETURN_ZONE && abs_y <= JOYSTICK_RETURN_ZONE) {
                if (centred_samples < 2) ++centred_samples;
                if (centred_samples >= 2) joystick_armed = true;
            } else {
                centred_samples = 0;
                if (joystick_armed &&
                    (abs_x > JOYSTICK_DEAD_ZONE || abs_y > JOYSTICK_DEAD_ZONE)) {
                    joystick_armed = false;
                    // A diagonal move acts on one axis only, whichever is stronger.
                    if (abs_y > abs_x) {
                        selected_item ^= 1;
                    } else if (selected_item == 0) {
                        home_blue = !home_blue;
                    } else {
                        menu_m = !menu_m;
                    }
                }
            }
            taskEXIT_CRITICAL();
            snprintf(lines[0], sizeof(lines[0]), "SETUP");
            snprintf(lines[1], sizeof(lines[1]), "%c Home: %s",
                     selected_item == 0 ? '>' : ' ', home_blue ? "blue" : "green");
            snprintf(lines[2], sizeof(lines[2]), "%c M: %s",
                     selected_item == 1 ? '>' : ' ', menu_m ? "Y" : "N");
            snprintf(lines[3], sizeof(lines[3]), "MOTORS INHIBITED");
        } else {
            // Re-entering setup requires centring, so a held stick cannot change it.
            joystick_armed = false;
            centred_samples = 0;
        }

        for (unsigned row = 0; row < 4; ++row) {
            if (drive_enabled.load()) {
                // Blank all rows, including any drawn before the state changed.
                if (inhibited) {
                    inhibited = false;
                    joystick_armed = false;
                    centred_samples = 0;
                    memset(lines, 0, sizeof(lines));
                    row = 0;
                }
            }
            char padded[17];
            snprintf(padded, sizeof(padded), "%-16s", lines[row]);
            if (strcmp(padded, displayed_lines[row]) == 0) continue;
            // Release the shared bus between rows so sensors can run.
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5)) != pdTRUE) continue;
            display.drawString(0, DISPLAY_ROWS[row], padded);
            xSemaphoreGive(i2cMutex);
            memcpy(displayed_lines[row], padded, sizeof(padded));
        }
    }
}
}

void start_display_task() {
    // CON60 is RAW I2C0 (Wire). Probe only the usual OLED addresses.
    // An ACK establishes presence, not which controller is fitted.
    uint8_t address = 0;
    for (uint8_t candidate = 0x3C; candidate <= 0x3D; ++candidate) {
        Wire.beginTransmission(candidate);
        if (Wire.endTransmission() == 0) {
            address = candidate;
            break;
        }
    }
    if (address == 0) {
        Serial.println("DISPLAY: no OLED at 0x3C/0x3D; display task skipped");
        return;
    }

    // Initialize before other tasks run, so no shared-bus lock is needed here.
    display.setI2CAddress(address << 1); // U8x8 uses the shifted address.
    display.setBusClock(400000);
    display.begin();
    display.setFont(u8x8_font_chroma48medium8_r);
    // Configure inputs only; sampling happens exclusively in the inhibited branch.
    pinMode(JOYSTICK_X_PIN, INPUT);
    pinMode(JOYSTICK_Y_PIN, INPUT);
    Serial.printf("DISPLAY: OLED responding at 0x%02X\n", address);

    if (xTaskCreate(display_loop, "display", 1024, nullptr, 1, nullptr) != pdPASS) {
        display.clearDisplay();
        Serial.println("DISPLAY: task creation failed");
    }
}

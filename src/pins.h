#pragma once

#include <Arduino.h>

// Main conveyor
constexpr uint8_t PIN_STEP_PUL = 13;   // DM542 PUL

// Shift carriage
constexpr uint8_t PIN_SHIFT_DIR = 14;  // Shift DIR
constexpr uint8_t PIN_SHIFT_PUL = 12;  // Shift PUL
constexpr uint8_t PIN_SHIFT_SENSOR_Z = 33; // Reed switch Z edge
constexpr uint8_t PIN_SHIFT_SENSOR_C = 25; // Reed switch C edge

// Pneumatics and sensors
constexpr uint8_t PIN_FLAG = 27;       // Flag cylinder
constexpr uint8_t PIN_SENSOR = 26;     // E18-D50NK

// Positional conveyor
constexpr uint8_t PIN_POS_PUL = 32;    // EVA25 PUL

#pragma once
#include <Arduino.h>

// The Arduino must stay powered when this relay cuts actuator power.
// UNO R4 WiFi: only D2/D3 support attachInterrupt. Move the button to D2
// and the left active IR sensor to D10 (see sensors.h).
constexpr uint8_t KILL_SWITCH_PIN = 2;  // UNO R4 external interrupt pin
constexpr uint8_t POWER_RELAY_PIN = 0;

// Existing active-LOW relay module.
constexpr uint8_t RELAY_ON = LOW;
constexpr uint8_t RELAY_OFF = HIGH;

constexpr unsigned long DOUBLE_PRESS_MS = 1000UL;
constexpr unsigned long REPOSITION_MS = 60000UL;
constexpr unsigned long EARLY_START_MS = 2000UL;

void setupSafety();
void updateSafety();
bool missionStartReady();
void enableIdlePower();
void enableRobotPower();
bool robotEnabled();
void disableRobotPower();
void inhibitRobotPower();

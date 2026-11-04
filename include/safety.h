#pragma once
#include <Arduino.h>

constexpr uint8_t KILL_SWITCH_PIN = 10;
constexpr uint8_t POWER_RELAY_PIN = 0;

constexpr uint8_t RELAY_ON = LOW;
constexpr uint8_t RELAY_OFF = HIGH;

void setupSafety();
void enableRobotPower();
bool robotEnabled();
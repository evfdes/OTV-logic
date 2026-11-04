#pragma once
#include <Arduino.h>

constexpr uint8_t FAN_PIN = 1;

void setupFan();
void turnFanOn();
void turnFanOff();
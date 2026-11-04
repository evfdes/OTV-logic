#include <Arduino.h>
#include "mission.h"

void setupFan() {
    pinMode(FAN_PIN, OUTPUT);
}

void turnFanOn() {
    digitalWrite(FAN_PIN, HIGH);
}

void turnFanOff() {
    digitalWrite(FAN_PIN, LOW);
}
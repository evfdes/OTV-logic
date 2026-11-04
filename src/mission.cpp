#include <Enes100.h>
#include "config.h"
#include "drive.h"
#include "mission.h"
#include "safety.h"
#include "sensors.h"

namespace {
    bool waitEnabled(unsigned long duration) {
        const unsigned long start = millis();
        while (robotEnabled() && millis() - start < duration)
            delay(20);

        return robotEnabled();
    }
}

void setupFan() {
    digitalWrite(FAN_PIN, LOW);
    pinMode(FAN_PIN, OUTPUT);
}

void turnFanOn() {
    stopMoving();
    if (robotEnabled())
        digitalWrite(FAN_PIN, HIGH);
}

void turnFanOff() {
    digitalWrite(FAN_PIN, LOW);
}

bool extinguishFlames() {
    
    stopMoving();
    if (!robotEnabled())
        return false;

    Enes100.mission(NUM_CANDLES, countFlamesDetected() + CENTER_CANDLES);

    for (uint8_t attempt = 0; attempt < FAN_ATTEMPTS; ++attempt) {
        turnFanOn();
        const bool enabled = waitEnabled(FAN_RUN_MS);
        turnFanOff();
        if (!enabled)
            return false;

        // Read only with the fan OFF; the four sensors observe the outer flames.
        if (!isFlameDetected())
            return waitEnabled(POST_EXTINGUISH_WAIT_MS);
    }
    return false;
}

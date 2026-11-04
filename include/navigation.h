#pragma once
#include <Arduino.h>

inline bool startedAtBottom() {
    static const bool startBottom = (Enes100.getY() < 1.0);
    return startBottom;
}

void turnToPayload();

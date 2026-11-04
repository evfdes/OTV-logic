#include <Arduino.h>
#include "navigation.h"
#include "Enes100.h"
#include "drive.cpp"

void turnToPayload() {
    int startTheta = getThetaDegrees();

    if (startedAtBottom()) {
        int payloadThetaDegrees = 90;
        rotateTo(payloadThetaDegrees);
    } else {
        int payloadThetaDegrees = 270;
        rotateTo(payloadThetaDegrees);
    }
}
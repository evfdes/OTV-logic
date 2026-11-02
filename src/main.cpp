#include <Arduino.h>
#include "drive.h"

void setup() {
    initDrivePins();

    moveForward();
    delay(2000);
    stopMoving();
}

void loop() {
    // Your main code here
}
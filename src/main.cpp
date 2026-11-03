#include <Arduino.h>
#include "drive.h"
#include <Enes100.h>

void setup() {
    initDrivePins();

    moveForward();
    delay(2000);
    stopMoving();

    Enes100.begin("Your Team", FIRE, markerId, roomNumber);
}

void loop() {
    // Your main code here
}
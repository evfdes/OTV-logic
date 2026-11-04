#include <Arduino.h>
#include "drive.h"
#include "sensors.h"
#include <Enes100.h>
#include "navigation.h"
#include "safety.h"

void setup() {

    Enes100.begin("Pyromaniacs", FIRE, markerId, 1116); //wifi module tx/rx ignored for r4 wifi

    setupSafety();

    unsigned long startupTime = millis();

    initDrivePins();
    setupSensors();

    // Initialize remaining systems here

    // Keep actuators OFF for at least 10 seconds
    while (millis() - startupTime < 5000UL) {
        delay(1);
    }

    enableRobotPower();

    startedAtBottom(); // Saves starting position permanently
    turnToPayload();
}

void loop() {
    if (!robotEnabled()) {
        return;
    }
    
    // Your main code here
}
#include <Arduino.h>
#include <math.h>
#include <Enes100.h>
#include "config.h"
#include "drive.h"
#include "mission.h"
#include "navigation.h"
#include "safety.h"
#include "sensors.h"

// Ensure the safety setup routine is declared before it is called.
void setupSafety();
void inhibitRobotPower();
void enableRobotPower();
void updateSafety();
bool robotEnabled();
void enableIdlePower();
void missionStartReady();
void disableRobotPower();

// Only start when the camera is connected and can see a valid marker position.
bool visionReady() {
    if (!Enes100.isConnected() || !Enes100.isVisible())
        return false;

    const float x = Enes100.getX();
    const float y = Enes100.getY();
    const float angle = Enes100.getTheta();
    
    return isfinite(x) && isfinite(y) && isfinite(angle) &&
           x >= 0 && x <= ARENA_LENGTH_M &&
           y >= 0 && y <= ARENA_WIDTH_M;
}

void setup() {
    setupSafety();
    Serial.begin(9600);
    initDrivePins();
    setupSensors();
    setupFan();

    if (MARKER_ID < 0 || !Enes100.begin("Pyromaniacs", FIRE, MARKER_ID, ROOM_NUMBER)) {
        Serial.println("Set MARKER_ID in config.h; vision must initialize before running.");
        inhibitRobotPower();
    }
}

void loop() {
    updateSafety();

    if (!robotEnabled()) {
        // Before re-energizing the relay, clear any commands remaining
        // from an interrupted run. Keep both actuators idle during the wait.
        stopMoving();
        turnFanOff();
        enableIdlePower();
    }

    missionStartReady();
    if (!visionReady()) {
        delay(20);
        return;
    }

    // Restart the COMPLETE route from the OTV's new position. The Arduino
    // itself and its vision connection do not need another hardware reset.
    setupNavigation();
    enableRobotPower();
    if (!robotEnabled()) return;
    startRunTimer(); // The 5-minute drive-speed timer begins at mission start.

    const bool success = approachPayload() && extinguishFlames() && leavePayload() && navigateObstacles();
    
    stopMoving();
    turnFanOff();
    
    const bool completedWithoutKill = robotEnabled();
   
    disableRobotPower();
    if (completedWithoutKill) {
        Serial.println(success ? "Route complete." : "Stopped before completing route.");
        Enes100.println(success ? "Route complete." : "Stopped before completing route.");
    }
}

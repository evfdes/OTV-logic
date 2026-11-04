#include <Enes100.h>
#include "config.h"
#include "drive.h"
#include "navigation.h"
#include "sensors.h"

namespace {
    bool startBottom = false;
    float approachX, approachY;

    bool payloadWithinRange() {
        const int distance = getFrontDistance();
        return distance >= 0 && distance <= PAYLOAD_STANDOFF_CM;
    }

    bool laneBlocked() {
        const int distance = getFrontDistance();
        return distance >= 0 && distance <= OBSTACLE_DETECTION_CM;
    }
}

void setupNavigation() {
    startBottom = Enes100.getY() < MIDDLE_LANE_Y_M;
    
    approachX = startBottom ? BOTTOM_APPROACH_X_M : TOP_APPROACH_X_M;
    
    // Line through (0, 1) and (0.37, 0.62); reflect y for the upper payload.
    const float lowerY = MIDDLE_LANE_Y_M -
        (MIDDLE_LANE_Y_M - LOWER_PAYLOAD_CORNER_Y_M) * approachX /
        LOWER_PAYLOAD_CORNER_X_M;

    approachY = startBottom ? ARENA_WIDTH_M - lowerY : lowerY;
}

bool startedAtBottom() {
    return startBottom;
}

bool turnToPayload() {
    return rotateTo(startBottom ? 90 : 270);
}

bool approachPayload() {
    const float lowerStagingY = LOWER_PAYLOAD_CENTER_Y_M + PAYLOAD_HALF_SIZE_M +
                               ROBOT_HALF_LENGTH_M + PAYLOAD_STANDOFF_CM / 100.0f;
    const float stagingY = startBottom ? ARENA_WIDTH_M - lowerStagingY : lowerStagingY;

    if (!turnToPayload() ||
        !moveTo(Enes100.getX(), stagingY, false, payloadWithinRange) ||
        !moveTo(approachX, Enes100.getY()) ||
        !moveTo(approachX, approachY))
            return false;

    // Continue along the diagonal until either KW12 switch makes contact.
    const float lowerTargetY = MIDDLE_LANE_Y_M -
        (MIDDLE_LANE_Y_M - LOWER_PAYLOAD_CORNER_Y_M) * PAYLOAD_CENTER_X_M /
        LOWER_PAYLOAD_CORNER_X_M;
    const float targetY = startBottom ? ARENA_WIDTH_M - lowerTargetY : lowerTargetY;
    
    if (!moveTo(PAYLOAD_CENTER_X_M, targetY, false, isPayloadTouched) || !isPayloadTouched())
        return false;

    // Hold the approach heading while seating fully in the gap.
    const int seatingHeading = getThetaDegrees();
    const unsigned long start = millis();
    
    while (isMotionAllowed() && millis() - start < SEATING_TIME_MS) {
        driveHeading(seatingHeading);
        delay(20);
    }
    
    stopMoving();
    return isMotionAllowed();
}

bool leavePayload() {
    if (!moveTo(approachX, approachY, true) ||
        !moveTo(approachX, MIDDLE_LANE_Y_M) ||
        !moveTo(PAYLOAD_CENTER_X_M, MIDDLE_LANE_Y_M) || !rotateTo(0))
            return false;

    // The payload is on the left after a bottom start, otherwise on the right.
    // ArUco x supplies an endpoint if the side IR still sees the payload.
    const unsigned long start = millis();
    while (isMotionAllowed() && millis() - start < MOVE_TIMEOUT_MS) {
        const bool alongside = startBottom ? isLeftObstacleDetected() : isRightObstacleDetected();
        if (!alongside || Enes100.getX() >= PAYLOAD_EXIT_X_M) {
            stopMoving();
            return isMotionAllowed();
        }
        driveHeading(0);
        delay(20);
    }
    stopMoving();
    return false;
}

bool navigateObstacles() {
    float laneY = MIDDLE_LANE_Y_M;
    for (unsigned int column = 0; column < sizeof(OBSTACLE_FRONT_X_M) / sizeof(float); ++column) {
        const float scanX = OBSTACLE_FRONT_X_M[column] -
                            ROBOT_HALF_LENGTH_M - OBSTACLE_STANDOFF_M;
        if (!moveTo(scanX, laneY) || !rotateTo(0)) return false;

        const bool blocked = laneBlocked();
        Enes100.print("Obstacle column ");
        Enes100.print(column + 1);
        Enes100.println(blocked ? ": blocked" : ": clear");
        if (blocked) {
            laneY = laneY == MIDDLE_LANE_Y_M ? LOWER_LANE_Y_M : MIDDLE_LANE_Y_M;
            if (!moveTo(scanX, laneY) || !rotateTo(0) || laneBlocked())
                return false;
        }

        const float clearX = OBSTACLE_FRONT_X_M[column] + OBSTACLE_DEPTH_M +
                             ROBOT_HALF_LENGTH_M + OBSTACLE_REAR_CLEARANCE_M;
        if (!moveTo(clearX, laneY))
            return false;
    }
    return moveTo(FINISH_X_M, laneY);
}

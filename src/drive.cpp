#include <math.h>
#include <Enes100.h>
#include "config.h"
#include "drive.h"
#include "mission.h"
#include "safety.h"

namespace {
    unsigned long runStartTime = 0;
}

void startRunTimer() {
    runStartTime = millis();
}

int getDriveSpeed() {
    return millis() - runStartTime >= RUN_DURATION_MS - HIGH_SPEED_WINDOW_MS ? FAST_DRIVE_SPEED : DRIVE_SPEED;
}


void setupMotors() {
    pinMode(LEFT_MOTORS_PWM_PIN, OUTPUT);
    pinMode(LEFT_MOTORS_IN1_PIN, OUTPUT);
    pinMode(LEFT_MOTORS_IN2_PIN, OUTPUT);

    pinMode(RIGHT_MOTORS_PWM_PIN, OUTPUT);
    pinMode(RIGHT_MOTORS_IN1_PIN, OUTPUT);
    pinMode(RIGHT_MOTORS_IN2_PIN, OUTPUT);

    stopMoving();
}

void setMotors (int leftSpeed, int rightSpeed) {

    if (!robotEnabled())
        leftSpeed = rightSpeed = 0;

    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    if (leftSpeed != 0 || rightSpeed != 0)
        turnFanOff();

   //left motor control
   if (leftSpeed == 0){
        digitalWrite(LEFT_MOTORS_IN1_PIN, LOW);
        digitalWrite(LEFT_MOTORS_IN2_PIN, LOW);
        analogWrite(LEFT_MOTORS_PWM_PIN, 0);
   } else {
        digitalWrite(LEFT_MOTORS_IN1_PIN, leftSpeed >=0 ? HIGH : LOW);
        digitalWrite(LEFT_MOTORS_IN2_PIN, leftSpeed >=0 ? LOW : HIGH);
        analogWrite(LEFT_MOTORS_PWM_PIN, abs(leftSpeed));
   }
    
    //right motor control
    if (rightSpeed == 0){
        digitalWrite(RIGHT_MOTORS_IN1_PIN, LOW);
        digitalWrite(RIGHT_MOTORS_IN2_PIN, LOW);
        analogWrite(RIGHT_MOTORS_PWM_PIN, 0);
   } else {
        digitalWrite(RIGHT_MOTORS_IN1_PIN, rightSpeed >= 0 ? HIGH : LOW);
        digitalWrite(RIGHT_MOTORS_IN2_PIN, rightSpeed >= 0 ? LOW : HIGH);
        analogWrite(RIGHT_MOTORS_PWM_PIN, abs(rightSpeed));
   }
}

//move forward
void moveForward() {
    setMotors(DRIVE_SPEED, DRIVE_SPEED);
}

//move backward
void moveBackward() {
    setMotors(-DRIVE_SPEED, -DRIVE_SPEED);
}

bool rotateTo(int targetDegrees) {
    stopMoving();
    const unsigned long start = millis();
    while (isMotionAllowed() && millis() - start < TURN_TIMEOUT_MS) {
        const int error = angleError(normalizeAngle(targetDegrees), getThetaDegrees());
        if (abs(error) <= HEADING_TOLERANCE_DEG) return true;
        setMotors(error > 0 ? -TURN_SPEED : TURN_SPEED,
                  error > 0 ? TURN_SPEED : -TURN_SPEED);
        delay(20);
        stopMoving();
    }
    stopMoving();
    return false;
}

bool rotate(int degrees) {
    if (!isMotionAllowed()) { stopMoving(); return false; }
    return rotateTo(getThetaDegrees() + degrees);
}

bool rotateCW90() {
    return rotate(-90);
}

bool rotateCCW90() {
    return rotate(90);
}

bool rotateCW45() {
    return rotate(-45);
}

bool rotateCCW45() {
    return rotate(45);
}

void stopMoving() {
    setMotors(0, 0);
}

int normalizeAngle(int angle) {
    return (angle % 360 + 360) % 360;
}

int angleError(int target, int current) {
    return (target - current + 540) % 360 - 180;
}

int getThetaDegrees() {
    return static_cast<int>(Enes100.getTheta() * 180.0f / PI);
}

void driveHeading(int headingDegrees, bool backwards, int speed) {
    const int error = angleError(headingDegrees, getThetaDegrees());

    if (speed == 0)
        speed = getDriveSpeed();

    const int correction = constrain(error, -speed / 2, speed / 2);
    const int direction = backwards ? -speed : speed;

    setMotors(direction - correction, direction + correction);
}

bool moveTo(float x, float y, bool backwards, bool (*stopCondition)()) {
    stopMoving();
    if (!isMotionAllowed())
        return false;
    
    const float travelX = x - Enes100.getX(), travelY = y - Enes100.getY();
    const unsigned long start = millis();

    while (isMotionAllowed() && millis() - start < MOVE_TIMEOUT_MS) {
        if (stopCondition && stopCondition()) {
            stopMoving();
            return isMotionAllowed();
        }

        const float dx = x - Enes100.getX(), dy = y - Enes100.getY();

        if (dx * dx + dy * dy <= POSITION_TOLERANCE_M * POSITION_TOLERANCE_M || dx * travelX + dy * travelY <= 0) {
            stopMoving();
            return true;
        }

        const int heading = normalizeAngle(lroundf(atan2f(dy, dx) * 180.0f / PI) + (backwards ? 180 : 0));
        
        if (abs(angleError(heading, getThetaDegrees())) > HEADING_TOLERANCE_DEG * 2) {
            if (!rotateTo(heading))
                return false;
        } else {
            // Reduce speed near a waypoint to avoid overshooting after the boost.
            const int speed = constrain(static_cast<int>(sqrtf(dx * dx + dy * dy) * 1000), DRIVE_SPEED, getDriveSpeed());
            driveHeading(heading, backwards, speed);
            delay(20);
        }
    }
    stopMoving();
    return false;
}
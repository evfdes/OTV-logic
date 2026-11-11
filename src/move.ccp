#include <math.h>
#include <Enes100.h>
#include "config.h"
#include "move.h"
#include "mission.h"

#include "safety.h"

bool robotEnabled();

namespace {
    unsigned long runStartTime = 0;
}

void startRunTimer() {
    runStartTime = millis();
}

int getDriveSpeed() {
    return millis() - runStartTime >= RUN_DURATION_MS - HIGH_SPEED_WINDOW_MS ? FAST_DRIVE_SPEED : DRIVE_SPEED;
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

bool isMotionAllowed() {
    if (!robotEnabled() || !Enes100.isConnected() || !Enes100.isVisible())
        return false;

    const float x = Enes100.getX(), y = Enes100.getY();
    const float theta = Enes100.getTheta();
    
    return isfinite(x) && isfinite(y) && isfinite(theta) &&
           x >= 0 && x <= ARENA_LENGTH_M && y >= 0 && y <= ARENA_WIDTH_M;
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


void stopMoving() {
    setMotors(0, 0);
}
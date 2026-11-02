#include "drive.h"

void initDrivePins() {
    pinMode(MOTOR_LEFT_PWM, OUTPUT);
    pinMode(MOTOR_LEFT_DIR, OUTPUT);
    pinMode(MOTOR_RIGHT_PWM, OUTPUT);
    pinMode(MOTOR_RIGHT_DIR, OUTPUT);
    stopMoving();
}

void setMotors (int leftSpeed, int rightSpeed) {
   //left motor control
    digitalWrite(MOTOR_LEFT_DIR, leftSpeed > 0 ? HIGH : LOW);
    analogWrite(MOTOR_LEFT_PWM, abs(leftSpeed));
    
    //right motor control
    digitalWrite(MOTOR_RIGHT_DIR, rightSpeed > 0 ? HIGH : LOW);
    analogWrite(MOTOR_RIGHT_PWM, abs(rightSpeed));
}

//move forward
void moveForward() {
    setMotors(DRIVE_SPEED, DRIVE_SPEED);
}

// rotate 90 degrees left back, right forward
void rotateCCW90() {
    setMotors(-TURN_SPEED, TURN_SPEED);
    delay(TURN_90_TIMES_MS);
    stopMoving();
}

// rotate 90 degrees right back, left forward
void rotateCW90() {
    setMotors(TURN_SPEED, -TURN_SPEED);
    delay(TURN_90_TIMES_MS);
    stopMoving();
}

void stopMoving() {
    setMotors(0, 0);
}
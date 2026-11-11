#include "drive.h"

void initDrivePins() {
    pinMode(LEFT_MOTORS_PWM_PIN, OUTPUT);
    pinMode(LEFT_MOTORS_IN1_PIN, OUTPUT);
    pinMode(LEFT_MOTORS_IN2_PIN, OUTPUT);

    pinMode(RIGHT_MOTORS_PWM_PIN, OUTPUT);
    pinMode(RIGHT_MOTORS_IN1_PIN, OUTPUT);
    pinMode(RIGHT_MOTORS_IN2_PIN, OUTPUT);

    stopMoving();
}

void setMotors(int leftSpeed, int rightSpeed) {
    // Left Motors Direction
    if (leftSpeed > 0) {
        digitalWrite(LEFT_MOTORS_IN1_PIN, HIGH);
        digitalWrite(LEFT_MOTORS_IN2_PIN, LOW);
    } else if (leftSpeed < 0) {
        digitalWrite(LEFT_MOTORS_IN1_PIN, LOW);
        digitalWrite(LEFT_MOTORS_IN2_PIN, HIGH);
    } else {
        digitalWrite(LEFT_MOTORS_IN1_PIN, LOW);
        digitalWrite(LEFT_MOTORS_IN2_PIN, LOW);
    }
    analogWrite(LEFT_MOTORS_PWM_PIN, constrain(abs(leftSpeed), 0, 255));

    // Right Motors Direction
    if (rightSpeed > 0) {
        digitalWrite(RIGHT_MOTORS_IN1_PIN, HIGH);
        digitalWrite(RIGHT_MOTORS_IN2_PIN, LOW);
    } else if (rightSpeed < 0) {
        digitalWrite(RIGHT_MOTORS_IN1_PIN, LOW);
        digitalWrite(RIGHT_MOTORS_IN2_PIN, HIGH);
    } else {
        digitalWrite(RIGHT_MOTORS_IN1_PIN, LOW);
        digitalWrite(RIGHT_MOTORS_IN2_PIN, LOW);
    }
    analogWrite(RIGHT_MOTORS_PWM_PIN, constrain(abs(rightSpeed), 0, 255));
}

void stopMoving() {
    setMotors(0, 0);
}

void moveForward() {
    setMotors(DRIVE_SPEED, DRIVE_SPEED);
}

void moveBackward() {
    setMotors(-DRIVE_SPEED, -DRIVE_SPEED);
}
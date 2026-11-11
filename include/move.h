#pragma once
#include <Arduino.h>
// H-Bridge motor pins
constexpr uint8_t LEFT_MOTORS_PWM_PIN = 5;
constexpr uint8_t LEFT_MOTORS_IN1_PIN = 7;
constexpr uint8_t LEFT_MOTORS_IN2_PIN = 4;

constexpr uint8_t RIGHT_MOTORS_PWM_PIN = 6;
constexpr uint8_t RIGHT_MOTORS_IN1_PIN = 8;
constexpr uint8_t RIGHT_MOTORS_IN2_PIN = 9;



// Drive speed constants
constexpr int DRIVE_SPEED = 25;
constexpr int TURN_SPEED = 25;

// function declarations
void initDrivePins();
void setMotors(int leftSpeed, int rightSpeed);
void stopMoving() { setMotors(0, 0); }


void moveForward();
void moveBackward();


int getThetaDegrees();
int normalizeAngle(int angle);
int angleError(int target, int current);
bool isMotionAllowed();
void startRunTimer();
int getDriveSpeed();
void driveHeading(int headingDegrees, bool backwards = false, int speed = 0);
bool moveTo(float x, float y, bool backwards = false,
            bool (*stopCondition)() = nullptr);


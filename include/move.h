#pragma once
#include <Arduino.h>

// H-Bridge motor pins
constexpr uint8_t LEFT_MOTORS_PWM_PIN  = 5;
constexpr uint8_t LEFT_MOTORS_IN1_PIN  = 7;
constexpr uint8_t LEFT_MOTORS_IN2_PIN  = 4;

constexpr uint8_t RIGHT_MOTORS_PWM_PIN = 6;
constexpr uint8_t RIGHT_MOTORS_IN1_PIN = 8;
constexpr uint8_t RIGHT_MOTORS_IN2_PIN = 9;

// Drive speed constants (0 - 255)
// Set to 100 so motors overcome physical friction (25 is usually too low)
constexpr int DRIVE_SPEED = 100;
constexpr int TURN_SPEED  = 100;

// Function Declarations
void initDrivePins();
void setMotors(int leftSpeed, int rightSpeed);
void stopMoving();
void moveForward();
void moveBackward();
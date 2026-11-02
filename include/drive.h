#ifindef DRIVE_H
#define DRIVE_H

#include <Arduino.h>

// H-Bridge motor pins
const int MOTOR_LEFT_PWM = 5;
const int MOTOR_LEFT_DIR = 4;
const int MOTOR_RIGHT_PWM = 6;
const int MOTOR_RIGHT_DIR = 7;

//Speed and timing constants 
const int DRIVE_SPEED = 150; 
const int TURN_SPEED  = 130;
const int TURN_90_TIMES_MS = 850;


// function declarations
void initDrivepins();
void setMotors(int leftSpeed, int rightSpeed);
void moveForward();
void rotateCCW90();
void rotateCW90();
void stopMoving();

#endif 


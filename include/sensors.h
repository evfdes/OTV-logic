#pragma once
#include <Arduino.h>

//Ultrasonic Distance Sensor Pin Definitions
constexpr uint8_t COMMON_ULTRASONIC_TRIG_PIN = 13;

constexpr uint8_t LEFT_ULTRASONIC_ECHO_PIN = A0;
constexpr uint8_t RIGHT_ULTRASONIC_ECHO_PIN = A1;

//Active IR Sensor Pin Definitions
constexpr uint8_t LEFT_ACTIVE_IR_PIN = 10; //swapped with kill switch
constexpr uint8_t RIGHT_ACTIVE_IR_PIN = 3;

//Flame IR Sensor Pin Definitions
constexpr uint8_t TOP_FLAME_IR_PIN = A2;
constexpr uint8_t BOTTOM_FLAME_IR_PIN = A3;
constexpr uint8_t LEFT_FLAME_IR_PIN = A4;
constexpr uint8_t RIGHT_FLAME_IR_PIN = A5;

//KW12-3 Switch Pin Definitions
constexpr uint8_t LEFT_KW12_3_PIN = 11;
constexpr uint8_t RIGHT_KW12_3_PIN = 12;

void setupSensors();
int getFrontDistance();
boolean isLeftObstacleDetected();
boolean isRightObstacleDetected();
boolean isPayloadTouched();
boolean isFlameDetected();
int countFlamesDetected();
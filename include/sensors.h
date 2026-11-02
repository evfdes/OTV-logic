//Ultrasonic Sensor Variables (Set as if we are using two Ultrasonic sensors)
#pragma once
#include <Arduino.h>

int Ultrasonic_TRIG_PIN1 = 9;
int Ultrasonic_ECHO_PIN1 = 11;

int Ultrasonic_TRIG_PIN2 = 10;
int Ultrasonic_ECHO_PIN2 = 12;

//Active IR Sensor variables

const int ActiveIR_OUTPUT1 = A0;
int sensorValue1 = analogRead(ActiveIR_OUTPUT1); 
const int ActiveIR_Output2 = A1;
int sensorValue2 = analogRead(ActiveIR_Output2);


//Flame IR Sensor Variables
const int FlameIR_OUTPUT1 = A2;
int sensorValue3 = analogRead(FlameIR_OUTPUT1);
const int FlameIR_OUTPUT2 = A3;
int sensorValue4 = analogRead(FlameIR_OUTPUT2);
const int FlameIR_OUTPUT3 = A4;
int sensorValue5 = analogRead(FlameIR_OUTPUT3);
const int FlameIR_OUTPUT4  = A5;
int sensorValue6 = analogRead(FlameIR_OUTPUT4);

//KW12-3 Switch Variables
const int KW_pin1 = 0;
const int KW_pin2 = 1;

void setupSensors();
void getFrontDistance();
void detectObstacleTest();
void detectFlameTest();
void detectObstacleInteractionTest();   
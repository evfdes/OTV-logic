#include <Arduino.h>
#include "sensors.h"
long duration1;
float distance1;    

void setupSensors() {
    //Pin modes set to Output
    pinMode(Ultrasonic_TRIG_PIN1, OUTPUT);
    pinMode(Ultrasonic_TRIG_PIN2, OUTPUT);
        
    //Pin modes set to Input
    pinMode(Ultrasonic_ECHO_PIN1, INPUT);
    pinMode(Ultrasonic_ECHO_PIN2, INPUT);

    pinMode(ActiveIR_OUTPUT1, INPUT);
    pinMode(ActiveIR_Output2, INPUT);
    pinMode(KW_pin1, INPUT_PULLUP);
    pinMode(KW_pin2, INPUT_PULLUP);
}

void getFrontDistance() {
//Sensor 1
digitalWrite(Ultrasonic_TRIG_PIN1, LOW);
delayMicroseconds(2);
digitalWrite(Ultrasonic_TRIG_PIN1, HIGH);
delayMicroseconds(10);
digitalWrite(Ultrasonic_TRIG_PIN1, LOW);

long duration1  = pulseIn(Ultrasonic_ECHO_PIN1, HIGH);
float distance1 = duration1 * 0.034/2;

delay(60);

//Sensor 2

digitalWrite(Ultrasonic_TRIG_PIN2, LOW);
delayMicroseconds(2);
digitalWrite(Ultrasonic_TRIG_PIN2, HIGH);
delayMicroseconds(10);
digitalWrite(Ultrasonic_TRIG_PIN2, LOW);

long duration2  = pulseIn(Ultrasonic_ECHO_PIN2, HIGH);
float distance2 = duration2 * 0.034/2;

Serial.print("Distance 1: ");
Serial.print(distance1);
Serial.print(" cm");
Serial.print(" | Distance 2: ");
Serial.print(distance2);
Serial.println(" cm");

    delay(300);
}


void detectObstacleTest() {
    int sensorValue1 = digitalRead(ActiveIR_OUTPUT1);
    Serial.print("Active IR: ");
    Serial.println(sensorValue1);
    delay(200);
    //To test the other sensor, switch the value to print to sensorValue2
}


void detectFlameTest() {
    int sensorValue3 = analogRead(FlameIR_OUTPUT1);
    Serial.print("Flame analog value: ");
    Serial.println(sensorValue3);
    delay(200);
}


void detectObstacleInteractionTest(){
    int state = digitalRead(KW_pin1);
    if(state == LOW)
        Serial.println("Touched");
        else
        Serial.println("Untouched");
    delay(200);
}
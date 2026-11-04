#include <Arduino.h>
#include "config.h"
#include "sensors.h"

void setupSensors() {
    //Digital Output
    pinMode(COMMON_ULTRASONIC_TRIG_PIN, OUTPUT);
        
    //Digital Input
    pinMode(LEFT_ULTRASONIC_ECHO_PIN, INPUT);
    pinMode(RIGHT_ULTRASONIC_ECHO_PIN, INPUT);

    //Digital Input
    pinMode(LEFT_ACTIVE_IR_PIN, INPUT);
    pinMode(RIGHT_ACTIVE_IR_PIN, INPUT);

    //Analog Input
    pinMode(TOP_FLAME_IR_PIN, INPUT);
    pinMode(BOTTOM_FLAME_IR_PIN, INPUT);
    pinMode(LEFT_FLAME_IR_PIN, INPUT);
    pinMode(RIGHT_FLAME_IR_PIN, INPUT);

    //Digital Input with Pullup
    pinMode(LEFT_KW12_3_PIN, INPUT_PULLUP);
    pinMode(RIGHT_KW12_3_PIN, INPUT_PULLUP);
}

int getFrontDistance() {

    static unsigned long lastTrigger = 0;
    const unsigned long elapsed = millis() - lastTrigger;
    if (elapsed < 60) delay(60 - elapsed);
    lastTrigger = millis();

    digitalWrite(COMMON_ULTRASONIC_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(COMMON_ULTRASONIC_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(COMMON_ULTRASONIC_TRIG_PIN, LOW);

    unsigned long startTime = micros();
    unsigned long leftStart = 0, rightStart = 0;
    unsigned long leftDuration = 0, rightDuration = 0;

    // 0 = waiting for HIGH, 1 = waiting for LOW, 2 = finished
    uint8_t leftState = 0, rightState = 0;

    while ((leftState != 2 || rightState != 2) &&
           (micros() - startTime < 25000UL)) {

        unsigned long now = micros();

        if (leftState == 0 && digitalRead(LEFT_ULTRASONIC_ECHO_PIN)) {
            leftStart = now;
            leftState = 1;
        } else if (leftState == 1 && !digitalRead(LEFT_ULTRASONIC_ECHO_PIN)) {
            leftDuration = now - leftStart;
            leftState = 2;
        }

        if (rightState == 0 && digitalRead(RIGHT_ULTRASONIC_ECHO_PIN)) {
            rightStart = now;
            rightState = 1;
        } else if (rightState == 1 && !digitalRead(RIGHT_ULTRASONIC_ECHO_PIN)) {
            rightDuration = now - rightStart;
            rightState = 2;
        }
    }

    if (leftState != 2 && rightState != 2)
        return -1;
    
    // Either front sensor can see a blocking obstacle; use the nearer echo.
    const unsigned long duration = leftState != 2 ? rightDuration :
        rightState != 2 ? leftDuration : min(leftDuration, rightDuration);
    
        return (duration * 343UL + 10000UL) / 20000UL;
}

boolean isLeftObstacleDetected() {
    bool leftObjectDetected = !digitalRead(LEFT_ACTIVE_IR_PIN);
    Serial.println("Obstacle to left:  " + String(leftObjectDetected));
    return leftObjectDetected;
}

boolean isRightObstacleDetected() {
    bool rightObjectDetected = !digitalRead(RIGHT_ACTIVE_IR_PIN);
    Serial.println("Obstacle to right:  " + String(rightObjectDetected));
    return rightObjectDetected;
}
    
boolean isPayloadTouched() {
    bool leftSwitch = digitalRead(LEFT_KW12_3_PIN);
    bool rightSwitch = digitalRead(RIGHT_KW12_3_PIN);

    if(leftSwitch == LOW || rightSwitch == LOW) {
        Serial.println("Touched");
        return true;
    } else {
        Serial.println("Untouched");
        return false;
    }
}

int countFlamesDetected() {
    int topFlameValue = analogRead(TOP_FLAME_IR_PIN);
    int bottomFlameValue = analogRead(BOTTOM_FLAME_IR_PIN);
    int leftFlameValue = analogRead(LEFT_FLAME_IR_PIN);
    int rightFlameValue = analogRead(RIGHT_FLAME_IR_PIN);

    Serial.println("Top Flame Analog value: " + String(topFlameValue));
    Serial.println("Bottom Flame Analog value: " + String(bottomFlameValue));
    Serial.println("Left Flame Analog value: " + String(leftFlameValue));
    Serial.println("Right Flame Analog value: " + String(rightFlameValue));

    int count = 0;
    if (topFlameValue < FLAME_THRESHOLD) count++;
    if (bottomFlameValue < FLAME_THRESHOLD) count++;
    if (leftFlameValue < FLAME_THRESHOLD) count++;
    if (rightFlameValue < FLAME_THRESHOLD) count++;

    return count;
}

boolean isFlameDetected() {
    return countFlamesDetected() > 0;
}
#include "drive.h"
#include <Enes100.h>

void initDrivePins() {
    pinMode(LEFT_MOTORS_PWM_PIN, OUTPUT);
    pinMode(LEFT_MOTORS_IN1_PIN, OUTPUT);
    pinMode(LEFT_MOTORS_IN2_PIN, OUTPUT);

    pinMode(RIGHT_MOTORS_PWM_PIN, OUTPUT);
    pinMode(RIGHT_MOTORS_IN1_PIN, OUTPUT);
    pinMode(RIGHT_MOTORS_IN2_PIN, OUTPUT);

    stopMoving();
}

void setMotors (int leftSpeed, int rightSpeed) {
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

//move forward
void moveForward() {
    setMotors(DRIVE_SPEED, DRIVE_SPEED);
}

//move backward
void moveBackward() {
    setMotors(-DRIVE_SPEED, -DRIVE_SPEED);
}

void rotate(int degrees) {
    if (degrees == 0) return;

    // Verify vision is available
    if (!Enes100.isConnected() || !Enes100.isVisible()) {
        stopMoving();
        Serial.println("Error: ArUco unavailable");
        return;
    }

    int targetTheta = static_cast<int>(std::fmod(getThetaDegrees() + std::fmod(degrees, 360.0) + 360.0, 360.0));

    for (int i = 0; i < 5; i++) {

        if (!Enes100.isConnected() || !Enes100.isVisible()) {
            stopMoving();
            Serial.println("Error: ArUco lost");
            return;
        }

        int currentTheta = getThetaDegrees();
        int remainingDegrees = angleError(targetTheta, currentTheta);

        if (std::fabs(remainingDegrees) <= 5) {
            break;
        }

        unsigned long turnTimeMs = calculateTurnTimeMs(remainingDegrees);

        // Positive angles assumed CCW
        if (remainingDegrees > 0) {
            setMotors(-TURN_SPEED, TURN_SPEED);
        } else {
            setMotors(TURN_SPEED, -TURN_SPEED);
        }

        delay(turnTimeMs);
        stopMoving();

        // Allow the robot to settle and vision to update
        delay(300);
    }

    Serial.println("Final turn error: " + String(angleError(targetTheta, getThetaDegrees())));
    if (std::fabs(angleError(targetTheta, getThetaDegrees())) > 5) {
        Serial.println("Turn did not reach target tolerance");
    }
}

void rotateCW90() {
    rotate(90);
}

void rotateCCW90() {
    rotate(-90);
}

void rotateCW45() {
    rotate(45);
}

void rotateCCW45() {
    rotate(-45);
}

void stopMoving() {
    setMotors(0, 0);
}

int normalizeAngle(int angle) {
    return (angle % 360 + 360) % 360;
}

int angleError(int target, int current) {
    return (target - current + 540) % 360 - 180;
}

int getThetaDegrees() {
    return static_cast<int>(Enes100.getTheta() * 180.0f / PI);
}

unsigned long calculateTurnTimeMs(int degrees) {
    // Calculate the time required to turn the specified number of degrees based on the robot's geometry and wheel RPM.

    //t = (degrees * track_width) / (12 * wheel_radius * wheel_rpm)
    float wheelRadiusCM = 5.0f;
    float wheelDistanceCM = 8.5f;
    float turnWheelRPM = 20.0f;

    float seconds = (abs(degrees) * wheelDistanceCM) / (12.0f * wheelRadiusCM * turnWheelRPM);

    return static_cast<unsigned long>(seconds * 1000.0f);
}